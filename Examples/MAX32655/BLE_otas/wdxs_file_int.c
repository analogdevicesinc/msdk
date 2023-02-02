/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Wireless Data Exchange profile implementation - File Example.
 *
 *  Copyright (c) 2013-2018 Arm Ltd. All Rights Reserved.
 *  ARM Ltd. confidential and proprietary.
 *
 *  IMPORTANT.  Your use of this file is governed by a Software License Agreement
 *  ("Agreement") that must be accepted in order to download or otherwise receive a
 *  copy of this file.  You may not use or copy this file for any purpose other than
 *  as described in the Agreement.  If you do not agree to all of the terms of the
 *  Agreement do not use this file and delete all copies in your possession or control;
 *  if you do not have a copy of the Agreement, you must contact ARM Ltd. prior
 *  to any use, copying or further distribution of this software.
 */
/*************************************************************************************************/

#include <string.h>
#include <stdlib.h>
#include "mxc_device.h"
#include "mxc_delay.h"
#include "wsf_types.h"
#include "util/wstr.h"
#include "wsf_trace.h"
#include "wsf_assert.h"
#include "wsf_efs.h"
#include "wsf_cs.h"
#include "wsf_msg.h"
#include "wsf_buf.h"
#include "util/bstream.h"
#include "svc_wdxs.h"
#include "wdxs/wdxs_api.h"
#include "wdxs/wdxs_main.h"
#include "wdxs_file.h"
#include "dm_api.h"
#include "att_api.h"
#include "app_api.h"
#include "flc.h"
#include "sch_api.h"

#ifndef FW_VERSION_MAJOR
#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 0
#endif

#define ERASE_DELAY 1 // ms

extern uint32_t _flash_update;
extern uint32_t _eflash_update;

static uint32_t eraseAddress, erasePages;
wsfHandlerId_t eraseHandlerId;
wsfTimer_t eraseTimer;

wsfHandlerId_t writeHandlerId;
wsfQueue_t writeQueue;
static bool_t savedHeader = FALSE;

/* Prototypes for file functions */
static uint8_t wdxsFileInitMedia(void);
static uint8_t wdxsFileErase(uint8_t *address, uint32_t size);
static uint8_t wdxsFileRead(uint8_t *pBuf, uint8_t *pAddress, uint32_t size);
static uint8_t wdxsFileWrite(const uint8_t *pBuf, uint8_t *pAddress, uint32_t size);
static uint8_t wsfFileHandle(uint8_t cmd, uint32_t param);

static fileHeader_t fileHeader = { .fileCRC = 0, .fileLen = 0 };
wsfEfsHandle_t otaFileHdl;

/* Use the second half of the flash space for scratch space */
static const wsfEfsMedia_t WDXS_FileMedia = {

    /*   uint32_t                startAddress;  Start address. */ ((uint32_t)&_flash_update),
    /*   uint32_t                endAddress;    End address. */ ((uint32_t)&_eflash_update),
    /*   uint32_t                pageSize;      Page size. */ MXC_FLASH_PAGE_SIZE,
    /*   wsfMediaInitFunc_t      *init;         Media intialization callback. */ wdxsFileInitMedia,
    /*   wsfMediaEraseFunc_t     *erase;        Media erase callback. */ wdxsFileErase,
    /*   wsfMediaReadFunc_t      *read;         Media read callback. */ wdxsFileRead,
    /*   wsfMediaWriteFunc_t     *write;        Media write callback. */ wdxsFileWrite,
    /*   wsfMediaHandleCmdFunc_t *handleCmd;    Media command handler callback. */ wsfFileHandle
};

#define HEADER_LEN (sizeof(fileHeader_t))
#define HEADER_LOCATION WDXS_FileMedia.startAddress

/*************************************************************************************************/
/*!
 *  \brief  WSF event handler for file erase.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void wdxsFileEraseHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
    int err = 0;

    if (erasePages) {

        /* Only erase the flash if the scheduler is idle. */
        if(SchGetState() != SCH_STATE_IDLE) {
            /* Pend the erase */
            WsfTimerStartMs(&eraseTimer, ERASE_DELAY);
            return;
        }

        APP_TRACE_INFO1(">>> Erasing address 0x%x in internal flash <<<", eraseAddress);

        /* The flash can not be accessed while the write is being performed. */
        WsfCsEnter();
        err = MXC_FLC_PageErase((uint32_t)eraseAddress);
        WsfCsExit();
        if (err != E_NO_ERROR) {
            APP_TRACE_INFO0("There was an erase error");
            return;
        }
        erasePages--;
        eraseAddress += MXC_FLASH_PAGE_SIZE;
        /* Continue next erase */
        WsfTimerStartMs(&eraseTimer, ERASE_DELAY);
    } else {
        /* Erase is complete */
        APP_TRACE_INFO0(">>> Internal flash erase complete <<<");
        wdxsFtcSendRsp(1, WDX_FTC_OP_PUT_RSP, otaFileHdl, WDX_FTC_ST_SUCCESS);
    }
}

/*************************************************************************************************/
/*!
 *  \brief  Enqueue the next message and send an indication to the handler.
 *
 *  \param  address     Flash address.
 *  \param  size        Data length.
 *  \param  pBuf        Data to write.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void wdxsFileWriteMessage(uint32_t address, uint32_t size, const uint8_t *pBuf)
{
    /* Allocate the message */
    uint8_t* writeBuf = WsfMsgAlloc(size + 4 + 4);

    if(writeBuf == NULL) {
        WSF_ASSERT(0);
    }

    /* Copy in the address, size, and data */
    memcpy(&writeBuf[0], &address, sizeof(uint32_t));
    memcpy(&writeBuf[sizeof(uint32_t)], &size, sizeof(uint32_t));
    memcpy(&writeBuf[2*sizeof(uint32_t)], pBuf, size);

    /* Enqueue the message */
    WsfMsgEnq(&writeQueue, 0, writeBuf);

    /* Signal the handler */
    WsfSetEvent(writeHandlerId, 1);
}

/*************************************************************************************************/
/*!
 *  \brief  WSF event handler for file write.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void wdxsFileWriteHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
    int err;
    static const unsigned writeBufLen = 256;
    uint32_t writeAddress, writeLen;
    uint32_t writeBuf[writeBufLen/sizeof(uint32_t)];
    uint8_t* pBuf;
    wsfHandlerId_t retHandler;
    wsfMsgHdr_t *queueMsg;

    /* Dequeue the next message */
    queueMsg = WsfMsgDeq(&writeQueue, &retHandler);

    /* Perform all of the pending writes */
    while(queueMsg != NULL) {

        /* Get a uint8_t pointer into the message */
        pBuf = (uint8_t*)queueMsg;

        /* Get the address and length from the buffer */
        memcpy(&writeAddress, &pBuf[0], sizeof(uint32_t));
        memcpy(&writeLen, &pBuf[sizeof(uint32_t)], sizeof(uint32_t));

        /* Make sure message doesn't overflow */
        WSF_ASSERT(writeLen <= writeBufLen);

        /* Align the data */
        memcpy(writeBuf, &pBuf[2*sizeof(uint32_t)], writeLen);

        /* Only write the flash if the scheduler is idle. */
        if(SchGetState() != SCH_STATE_IDLE) {
            /* Re-queue the message */
            WsfMsgFree(queueMsg);
            wdxsFileWriteMessage(writeAddress, writeLen, (const uint8_t*)writeBuf);
            return;
        }

        /* Perform the write, use critical section because we must execute from SRAM.
         * The flash can not be accessed while the write is being performed.
         */
        WsfCsEnter();
        err = MXC_FLC_Write(writeAddress, writeLen, writeBuf);
        WSF_ASSERT(err == E_NO_ERROR);

        /* Free the message */
        WsfMsgFree(queueMsg);

        WsfCsExit();

        APP_TRACE_INFO2("Int. Flash: Wrote %d bytes @ 0x%x", writeLen, writeAddress);

        /* Get the next message */
        queueMsg = WsfMsgDeq(&writeQueue, &retHandler);
    }
}
/*************************************************************************************************/
/*!
 *  \brief  Media Init function, called when media is registered.
 *
 *  \return Status of the operation.
 */
/*************************************************************************************************/
static uint8_t wdxsFileInitMedia(void)
{
    MXC_FLC_Init();
    APP_TRACE_INFO2("FW_VERSION: %d.%d", FW_VERSION_MAJOR, FW_VERSION_MINOR);

    /* Setup the erase handler */
    eraseHandlerId = WsfOsSetNextHandler(wdxsFileEraseHandler);
    eraseTimer.handlerId = eraseHandlerId;

    /* Setup the write handler */
    writeHandlerId = WsfOsSetNextHandler(wdxsFileWriteHandler);
    WSF_QUEUE_INIT(&writeQueue);

    return WSF_EFS_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  File erase function. Must be page aligned.
 *
 *  \param  pAddress Address in media to start erasing.
 *  \param  size     Number of bytes to erase.
 *
 *  \return Status of the operation.
 */
/*************************************************************************************************/
static uint8_t wdxsFileErase(uint8_t *address, uint32_t size)
{
    uint32_t address32 = (uint32_t)address;

    if (fileHeader.fileLen != 0) {
        int err = 0;
        /* Setup the erase handler variables */
        eraseAddress = address32;
        /* calculate pages needed to erase */
        erasePages = (fileHeader.fileLen / MXC_FLASH_PAGE_SIZE) + 1;
        /* Initiate the erase */
        WsfCsEnter();
        err = MXC_FLC_PageErase((uint32_t)address32);
        WsfCsExit();
        if (err != E_NO_ERROR) {
            APP_TRACE_INFO1("Flash page erase error at 0x%0x", address32);
            return WSF_EFS_FAILURE;
        } else {
            APP_TRACE_INFO1(">>> Initiating erase of %d pages of internal flash <<<", erasePages);
        }
        erasePages--;
        eraseAddress += MXC_FLASH_PAGE_SIZE;

        savedHeader = FALSE;

        /* Wait ERASE_DELAY ms before staring next erase */
        WsfTimerStartMs(&eraseTimer, ERASE_DELAY);

        return WSF_EFS_SUCCESS;
    } else {
        APP_TRACE_INFO0(">>> File size is unknown <<<");
        return WSF_EFS_FAILURE;
    }

    return WSF_EFS_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  Media Read function.
 *
 *  \param  pBuf     Buffer to hold data.
 *  \param  pAddress Address in media to read from.
 *  \param  size     Size of pBuf in bytes.
 *
 *  \return Status of the operation.
 */
/*************************************************************************************************/
static uint8_t wdxsFileRead(uint8_t *pBuf, uint8_t *pAddress, uint32_t size)
{
    memcpy(pBuf, pAddress, size);
    return WSF_EFS_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  File Write function.
 *
 *  \param  pBuf     Buffer with data to be written.
 *  \param  address  Address in media to write to.
 *  \param  size     Size of pBuf in bytes.
 *
 *  \return Status of the operation.
 */
/*************************************************************************************************/
static uint8_t wdxsFileWrite(const uint8_t *pBuf, uint8_t *pAddress, uint32_t size)
{
    uint32_t address = (uint32_t)pAddress;

    if(!savedHeader) {
        wdxsFileWriteMessage(HEADER_LOCATION, HEADER_LEN, (const uint8_t *)&fileHeader);
        savedHeader = TRUE;
    }

    /* offset by the header length written into flash */
    address += HEADER_LEN;

    wdxsFileWriteMessage(address, size, pBuf);

    return WSF_EFS_SUCCESS;
}

// http://home.thep.lu.se/~bjorn/crc/
/*************************************************************************************************/
/*!
 *  \brief  Create the CRC32 table.
 *
 *  \param  r       Index into the table
 *
 *  \return None.
 */
/*************************************************************************************************/
uint32_t crc32_for_byte(uint32_t r)
{
    for (int j = 0; j < 8; ++j) r = (r & 1 ? 0 : (uint32_t)0xEDB88320L) ^ r >> 1;
    return r ^ (uint32_t)0xFF000000L;
}

/*************************************************************************************************/
/*!
 *  \brief  Calculate the CRC32 value for the given buffer.
 *
 *  \param  data    Pointer to the data.
 *  \param  n_bytes Number of bytes in the buffer.
 *  \param  crc     Pointer to store the result.
 *
 *  \return None.
 */
/*************************************************************************************************/
static uint32_t table[0x100] = { 0 };
void crc32(const void *data, size_t n_bytes, uint32_t *crc)
{
    if (!*table) {
        for (size_t i = 0; i < 0x100; ++i) table[i] = crc32_for_byte(i);
    }
    for (size_t i = 0; i < n_bytes; ++i) {
        *crc = table[(uint8_t)*crc ^ ((uint8_t *)data)[i]] ^ *crc >> 8;
    }
}

/*************************************************************************************************/
/*!
 *  \brief  Media Specific Command handler.
 *
 *  \param  cmd    Identifier of the media specific command.
 *  \param  param  Optional Parameter to the command.
 *
 *  \return Status of the operation.
 */
/*************************************************************************************************/
static uint8_t wsfFileHandle(uint8_t cmd, uint32_t param)
{
    switch (cmd) {
    case WSF_EFS_WDXS_PUT_COMPLETE_CMD: {
        /* Currently unimplemented */
        return WDX_FTC_ST_SUCCESS;
    } break;
    case WSF_EFS_VALIDATE_CMD:
    default: {
        /* Validate the image with CRC32 */
        uint32_t crcResult = 0;

        APP_TRACE_INFO2("CRC start addr: 0x%08X Len: 0x%08X", WDXS_FileMedia.startAddress,
                        fileHeader.fileLen);

        crc32((const void *)(WDXS_FileMedia.startAddress + HEADER_LEN),
            fileHeader.fileLen, &crcResult);

        APP_TRACE_INFO1("CRC From File : 0x%08x", fileHeader.fileCRC);
        APP_TRACE_INFO1("CRC Calculated: 0x%08X", crcResult);

        /* Check the calculated CRC32 against what was received, 32 bits is 4 bytes */
        if (fileHeader.fileCRC != crcResult) {
            APP_TRACE_INFO0("Update file verification failure");
            return WDX_FTC_ST_VERIFICATION;
        }
        return WDX_FTC_ST_SUCCESS;
    } break;
    }
    return WDX_FTC_ST_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \brief  Example of creating a WDXS stream.
 *
 *  \param  none
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxsFileInit(void)
{
    wsfEsfAttributes_t attr;
    char versionString[WSF_EFS_VERSION_LEN];

    /* Add major number */
    versionString[0] = FW_VERSION_MAJOR;
    /* Add "." */
    versionString[1] = '.';
    /* Minor number */
    versionString[2] = FW_VERSION_MINOR;
    /* Add termination character */
    versionString[3] = 0;

    /* Register the media for the stream */
    WsfEfsRegisterMedia(&WDXS_FileMedia, WDX_FLASH_MEDIA);

    /* Set the attributes for the stream */
    attr.permissions = (WSF_EFS_REMOTE_GET_PERMITTED | WSF_EFS_REMOTE_PUT_PERMITTED |
                        WSF_EFS_REMOTE_ERASE_PERMITTED | WSF_EFS_REMOTE_VERIFY_PERMITTED |
                        WSF_EFS_LOCAL_GET_PERMITTED | WSF_EFS_LOCAL_PUT_PERMITTED |
                        WSF_EFS_LOCAL_ERASE_PERMITTED | WSF_EFS_REMOTE_VISIBLE);

    attr.type = WSF_EFS_FILE_TYPE_BULK;

    /* Potential buffer overrun is intentional to zero out fixed length field */
    /* coverity[overrun-buffer-arg] */
    WstrnCpy(attr.name, "File", WSF_EFS_NAME_LEN);
    /* coverity[overrun-buffer-arg] */
    WstrnCpy(attr.version, versionString, WSF_EFS_VERSION_LEN);

    /* Add a file for the stream */
    otaFileHdl = WsfEfsAddFile(WDXS_FileMedia.endAddress - WDXS_FileMedia.startAddress,
                               WDX_FLASH_MEDIA, &attr, 0);
    APP_TRACE_INFO1("File Hdl: %d", otaFileHdl);
}

/*************************************************************************************************/
/*!
 *  \brief  Get the base address of the WDXS file.
 *
 *  \return Base address of WDXS file.
 */
/*************************************************************************************************/
uint32_t WdxsFileGetBaseAddr(void)
{
    return WDXS_FileMedia.startAddress;
}

/*************************************************************************************************/
/*!
 *  \brief  Get the length of the last verified WDXS file.
 *
 *  \return Verified length of WDXS file.
 */
/*************************************************************************************************/
uint32_t WdxsFileGetVerifiedLength(void)
{
    return fileHeader.fileLen;
}

/*************************************************************************************************/
/*!
 *  \brief  Get the firmware version of the WDXS file.
 *
 *  \return Firmware version of WDXS file.
 */
/*************************************************************************************************/
uint16_t WdxsFileGetFirmwareVersion(void)
{
    wsfEsfAttributes_t attr;
    uint8_t minor, major;

    WsfEfsGetAttributes(otaFileHdl, &attr);
    major = attr.version[0];
    minor = attr.version[2];
    // store major in upper byte and minor in lower byte
    return (uint16_t)major << 8 | minor;
}

void initHeader(fileHeader_t *header)
{
    fileHeader.fileLen = header->fileLen;
    fileHeader.fileCRC = header->fileCRC;
}
