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
#include "mxc_device.h"
#include "wsf_types.h"
#include "util/wstr.h"
#include "wsf_trace.h"
#include "wsf_assert.h"
#include "wsf_efs.h"
#include "wsf_cs.h"
#include "util/bstream.h"
#include "svc_wdxs.h"
#include "wdxs/wdxs_api.h"
#include "wdxs/wdxs_main.h"
#include "wdxs_file.h"
#include "dm_api.h"
#include "att_api.h"
#include "app_api.h"
#include "flc.h"
#include "Ext_Flash.h"
#ifndef FW_VERSION
#define FW_VERSION 1
#endif
#define EXT_FLASH_PAGE_SIZE 256
#define EXT_FLASH_SECTOR_SIZE ((uint32_t)0x00010000)
#define HEADER_LOCATION ((uint32_t)0x00000000)
static volatile uint32_t verifyLen;
static volatile uint8_t *lastWriteAddr;
static volatile uint32_t lastWriteLen;
static uint32_t crcResult;
/* Prototypes for file functions */
static uint8_t wdxsFileInitMedia(void);
static uint8_t wdxsFileErase(uint8_t *address, uint32_t size);
static uint8_t wdxsFileRead(uint8_t *pBuf, uint8_t *pAddress, uint32_t size);
static uint8_t wdxsFileWrite(const uint8_t *pBuf, uint8_t *pAddress, uint32_t size);
static uint8_t wsfFileHandle(uint8_t cmd, uint32_t param);

static fileHeader_t fileHeader = { .fileCRC = 0, .fileLen = 0 };
#define HEADER_LEN (sizeof(fileHeader_t))
/* Use the second half of the flash space for scratch space */
static const wsfEfsMedia_t WDXS_FileMedia = {
    /*   uint32_t                startAddress;  Start address + size of header. */ (
        (uint32_t)0x00000000),
    /*   uint32_t                endAddress;    End address. */ ((uint32_t)0x01000000),
    /*   uint32_t                pageSize;      Page size. */ EXT_FLASH_PAGE_SIZE,
    /*   wsfMediaInitFunc_t      *init;         Media intialization callback. */ wdxsFileInitMedia,
    /*   wsfMediaEraseFunc_t     *erase;        Media erase callback. */ wdxsFileErase,
    /*   wsfMediaReadFunc_t      *read;         Media read callback. */ wdxsFileRead,
    /*   wsfMediaWriteFunc_t     *write;        Media write callback. */ wdxsFileWrite,
    /*   wsfMediaHandleCmdFunc_t *handleCmd;    Media command handler callback. */ wsfFileHandle
};

/*************************************************************************************************/
/*!
 *  \brief  Media Init function, called when media is registered.
 *
 *  \return Status of the operation.
 */
/*************************************************************************************************/
static uint8_t wdxsFileInitMedia(void)
{
    int err = 0;
    MXC_FLC_Init();
    err += Ext_Flash_Init();
    err += Ext_Flash_Quad(1);
    if (err)
        APP_TRACE_INFO0("Error initializing external flash");
    APP_TRACE_INFO1("FW_VERSION: %d", FW_VERSION);
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
    uint32_t sectors = 0; // hard coded for now because image has no len data
    volatile int i;
    if (fileHeader.fileLen != 0) {
        /* calculate sectors needed to erase */
        sectors = (fileHeader.fileLen / EXT_FLASH_SECTOR_SIZE) + 1;
        APP_TRACE_INFO1(">>> Erasing %d 64K sectors in external flash <<<", sectors);
        while (sectors) {
            /* TODO:  Debug as to why this is needed */
            for (i = 0; i < 0xFFFF; i++) {}
            Ext_Flash_Erase(address32, Ext_Flash_Erase_64K);
            sectors--;
            address32 += EXT_FLASH_SECTOR_SIZE;
        }
        return WSF_EFS_SUCCESS;
    } else {
        APP_TRACE_INFO0(">>> File size is unknown <<<");
        return WSF_EFS_FAILURE;
    }
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
    Ext_Flash_Read((uint32_t)pAddress, pBuf, size, Ext_Flash_DataLine_Quad);
    return WSF_EFS_SUCCESS;
}

/* http://home.thep.lu.se/~bjorn/crc/ */
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
    static bool_t savedHeader = FALSE;
    int err = 0;
    uint8_t attempts = 2;
    uint8_t tempBuff[size];
    /* helps silence compiler warnings over discarded const qualifier */
    uint32_t addressToBuf = (uint32_t)pBuf;
    /* write the header in flash device */
    if (!savedHeader) {
        err += Ext_Flash_Program_Page(HEADER_LOCATION, (uint8_t *)&fileHeader, sizeof(fileHeader_t),
                                      Ext_Flash_DataLine_Quad);
        /* verify header was written correctly */
        err += Ext_Flash_Read(HEADER_LOCATION, tempBuff, sizeof(fileHeader_t),
                              Ext_Flash_DataLine_Quad);
        if (memcmp(tempBuff, (uint8_t *)&fileHeader, sizeof(fileHeader_t)) != 0) {
            APP_TRACE_INFO0("Error writting header to external flash");
        }
        savedHeader = TRUE;
    }
    /* offset by the header thats already written */
    pAddress += HEADER_LEN;
    crc32((const void *)pBuf, size, &crcResult);
    while (attempts) {
        err += Ext_Flash_Program_Page((uint32_t)pAddress, (uint8_t *)addressToBuf, size,
                                      Ext_Flash_DataLine_Quad);
        err += Ext_Flash_Read((uint32_t)pAddress, tempBuff, size, Ext_Flash_DataLine_Quad);
        /* verify data was written correctly */
        if (memcmp(tempBuff, pBuf, size) != 0) {
            attempts--;
            if (attempts == 0)
                err++;
        } else
            attempts = 0;
    }
    if (err == E_NO_ERROR) {
        lastWriteAddr = pAddress;
        lastWriteLen = size;
        return WSF_EFS_SUCCESS;
    }
    APP_TRACE_ERR1("Error writing to flash 0x%08X", (uint32_t)pAddress);

    return WSF_EFS_FAILURE;
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
        verifyLen = (uint32_t)lastWriteAddr - WDXS_FileMedia.startAddress - sizeof(fileHeader_t) +
                    lastWriteLen;

        APP_TRACE_INFO2("CRC start addr: 0x%08X Len: 0x%08X", WDXS_FileMedia.startAddress,
                        verifyLen);
        APP_TRACE_INFO1("CRC From File : 0x%08x", fileHeader.fileCRC);
        APP_TRACE_INFO1("CRC Calculated: 0x%08X", crcResult);

        /* Check the calculated CRC32 against what was received, 32 bits is 4 bytes */
        if (fileHeader.fileCRC != crcResult) {
            APP_TRACE_INFO0("Update file verification failure");
            crcResult = 0;
            return WDX_FTC_ST_VERIFICATION;
        }
        crcResult = 0;
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
    versionString[0] = FW_VERSION & 0xFF;
    /* Add "." */
    versionString[1] = (FW_VERSION & 0xFF00) >> 8;
    /* Minor number */
    versionString[2] = (FW_VERSION & 0xFF0000) >> 16;
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
    WsfEfsAddFile(WDXS_FileMedia.endAddress - WDXS_FileMedia.startAddress, WDX_FLASH_MEDIA, &attr,
                  0);
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
    return verifyLen;
}

void initHeader(fileHeader_t *header)
{
    fileHeader.fileLen = header->fileLen;
    fileHeader.fileCRC = header->fileCRC;
}