/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/

/*******************************************************************************
  Includes
*******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "mxc_device.h"
#include "mxc_delay.h"
#include "board.h"
#include "led.h"
#include "pb.h"
#include "tmr.h"
#include "lp.h"

#include "pal_bb.h"
#include "pal_bb_ble.h"
#include "wsf_cs.h"
#include "pal_radio.h"

#include "scn_stripped.h"

/*******************************************************************************
  Definitions
*******************************************************************************/
#define DUE_OFFSET              1000
#define MAX_RX_LEN              39
#define HEADER_LEN              2
#define ADDR_LEN                6
#define CHMAP_LEN               5
#define CONN_DATA_LEN           22

#define SCAN_REQ_HDR_MSB        0x30U
#define SCAN_REQ_HDR_LSB        0x0CU
#define CONN_IND_HDR_MSB        0x54U
#define CONN_IND_HDR_LSB        0x22U

/*! \brief  Scanner states */
enum {
    SCN_LISTEN_ADV_IND,
    SCN_SEND_SCAN_REQ,
    SCN_LISTEN_SCAN_RSP,
    SCN_SEND_CONN_IND,
    SCN_CONNECTED
};

/*******************************************************************************
  Macros
*******************************************************************************/
#if DEBUG == 1
#define DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...) ((void)0)
#endif

/*******************************************************************************
  Global Variables
*******************************************************************************/
uint8_t scanRxData[MAX_RX_LEN];
volatile uint8_t scanState;


volatile bool_t scanEvtFinished;
volatile bool_t scanEvtStatus;

static uint8_t addrA[ADDR_LEN];
uint8_t addrS[ADDR_LEN];
uint8_t scanReq_pktLen;
uint8_t connInd_pktLen;

PalBbBleTxBufDesc_t scanReqBuf;
PalBbBleTxBufDesc_t connIndBuf;

/*******************************************************************************
  Functions
*******************************************************************************/
void scanRxCallback(uint8_t status, int8_t rssi, uint32_t crc, uint32_t timestamp, uint8_t rxPhyOptions);
void scanTxCallback(uint8_t status);
void fillScanReqPacket(PalBbBleTxBufDesc_t *bufDesc);
void fillConnIndPacket(PalBbBleTxBufDesc_t *bufDesc, ConnDataParams_t *pConn);
void initConnData(uint8_t *pBuf, ConnDataParams_t *pConn);

/*************************************************************************************************/
/*!
 *  \brief      Initialize pseudo-scanner.
 *
 *  \param      advAddr             Advertising address.
 *  \param      scanAddr            Scanning address.
 *  \param      pChan               Pointer to the channelization parameters.
 *  \param      pConn               Pointer to the connection parameters.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void initScanner(uint8_t *advAddr, uint8_t *scanAddr, PalBbBleChan_t *pChan, ConnDataParams_t *pConn)
{
    uint8_t i;
    for (i=0; i<ADDR_LEN; i++) {
        addrA[i] = advAddr[i];
        addrS[i] = scanAddr[i];
    }

    scanReq_pktLen = HEADER_LEN + 2*ADDR_LEN;
    connInd_pktLen = HEADER_LEN + 2*ADDR_LEN + CONN_DATA_LEN;

    PalBbBleSetChannelParam(pChan);
    fillScanReqPacket(&scanReqBuf);
    fillConnIndPacket(&connIndBuf, pConn);

    scanState = SCN_LISTEN_ADV_IND;
}

/*************************************************************************************************/
/*!
 *  \brief      Create a pseudo-scan request packet.
 *
 *  \param      bufDesc             Pointer to the packet buffer descriptor.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void fillScanReqPacket(PalBbBleTxBufDesc_t *bufDesc)
{   
    uint8_t pBuf[scanReq_pktLen];
    uint8_t i;

    pBuf[0] = SCAN_REQ_HDR_LSB;
    pBuf[1] = SCAN_REQ_HDR_MSB;
    for (i=0; i<ADDR_LEN; i++)
    {
        pBuf[i+HEADER_LEN] = addrS[(ADDR_LEN - 1) - i];
    }
    for (i=0; i<ADDR_LEN; i++)
    {
        pBuf[i+HEADER_LEN+ADDR_LEN] = addrA[(ADDR_LEN - 1) - i];
    }
    bufDesc->pBuf = pBuf;
    bufDesc->len = scanReq_pktLen;
}

/*************************************************************************************************/
/*!
 *  \brief      Create a pseudo-connection indication packet.
 *
 *  \param      bufDesc             Pointer to the packet buffer descriptor.
 *  \param      pConn               Pointer to the connection parameters.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void fillConnIndPacket(PalBbBleTxBufDesc_t *bufDesc, ConnDataParams_t *pConn)
{
    uint8_t pBuf[connInd_pktLen];
    uint8_t i;

    initConnData(pBuf, pConn);

    pBuf[0] = CONN_IND_HDR_LSB;
    pBuf[1] = CONN_IND_HDR_LSB;
    for (i=0; i<ADDR_LEN; i++)
    {
        pBuf[i+2] = addrS[(ADDR_LEN - 1) - i];
    }
    for (i=0; i<ADDR_LEN; i++)
    {
        pBuf[i+HEADER_LEN+ADDR_LEN] = addrA[(ADDR_LEN - 1) - i];
    }

    bufDesc->pBuf = pBuf;
    bufDesc->len = connInd_pktLen;
}

/*************************************************************************************************/
/*!
 *  \brief      Initialize pseudo-connection parameters.
 *
 *  \param      pBuf                Pointer to the packet buffer.
 *  \param      pConn               Pointer to the connection parameters.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void initConnData(uint8_t *pBuf, ConnDataParams_t *pConn) {
    int i;
    uint8_t dataStart = HEADER_LEN + 2*ADDR_LEN;
    for (i=0; i<dataStart; i++)
    {
        pBuf[i] = 0xFF;
    }
    for (i=0; i<4; i++)
    {
        pBuf[dataStart+i] = (pConn->accessAddress & (0xFF << (8*i))) >> (8*i);
    }
    dataStart += 4;
    for (i=0; i<3; i++)
    {
        pBuf[dataStart+i] = (pConn->crcInit & (0xFF << (8*i))) >> (8*i);
    }
    dataStart += 3;
    pBuf[dataStart] = pConn->winSize;
    dataStart++;
    pBuf[dataStart] = pConn->winOffset & 0xFF;
    pBuf[dataStart+1] = (pConn->winOffset & 0xFF00) >> 8;
    dataStart += 2;
    pBuf[dataStart] = pConn->interval & 0xFF;
    pBuf[dataStart+1] = (pConn->interval & 0xFF00) >> 8;
    dataStart += 2;
    pBuf[dataStart] = pConn->latency & 0xFF;
    pBuf[dataStart+1] = (pConn->latency & 0xFF00) >> 8;
    dataStart += 2;
    pBuf[dataStart] = pConn->timeout & 0xFF;
    pBuf[dataStart+1] = (pConn->timeout & 0xFF00) >> 8;
    dataStart += 2;
    for (i=0; i<CHMAP_LEN; i++)
    {
        pBuf[dataStart+i] = pConn->chM[(CHMAP_LEN - 1) - i];
    }
    dataStart += 5;
    pBuf[dataStart] = pConn->hop | pConn->sca;
}

/*************************************************************************************************/
/*!
 *  \brief      Start scanning.
 *
 *  \return     TRUE if a connection was established, else FALSE.
 */
/*************************************************************************************************/
bool_t startScanning(uint32_t scanTime)
{
    uint32_t dueTime;
    PalBbBleOpParam_t opParams = { 0 };
    PalBbBleSetOpParams(&opParams);
    scanEvtFinished = FALSE;
    scanState = SCN_SEND_SCAN_REQ;
    memset(scanRxData, 0, sizeof(scanRxData));
    PalBbGetTimestamp(&dueTime);
    dueTime += DUE_OFFSET;
    PalBbBleDataParam_t dataParams = {
        .txCback = scanTxCallback,
        .rxCback = scanRxCallback,
        .dueUsec = dueTime,
        .rxTimeoutUsec = scanTime
    };
    // printf("DUETIME: %lu", dueTime);
    PalBbBleSetDataParams(&dataParams);
    PalBbBleRxData(scanRxData, MAX_RX_LEN);
    while (!scanEvtFinished) {}
    if (scanEvtStatus != BB_STATUS_SUCCESS)
    {
        
        return FALSE;
    }
    return TRUE;
}

/*************************************************************************************************/
void scanRxCallback(uint8_t status, int8_t rssi, uint32_t crc, uint32_t timestamp, uint8_t rxPhyOptions)
{

    if (status != BB_STATUS_SUCCESS)
    {
        // DEBUG_PRINTF("** RX FAILED WITH STATUS %i **\n", status);
        scanEvtStatus = status;
        scanState = SCN_LISTEN_ADV_IND;
        PalBbBleCancelTifs();
        scanEvtFinished = TRUE;
        return;
    }
    
    uint32_t dueTime;
    switch (scanState) {
        case SCN_SEND_SCAN_REQ:
            scanState = SCN_LISTEN_SCAN_RSP;
            // PalBbGetTimestamp(&dueTime);
            // dueTime += 300;
            PalBbBleOpParam_t opParams = {
                .ifsMode = PAL_BB_IFS_MODE_TOGGLE_TIFS,
                .ifsTime = 0,
                .pIfsChan = NULL
            };
            // PalBbBleDataParam_t dataParams = {
            //     .txCback = scanTxCallback,
            //     .rxCback = scanRxCallback,
            //     .dueUsec = dueTime,
            //     .rxTimeoutUsec = 10000
            // };
            PalBbBleSetOpParams(&opParams);
            // PalBbBleSetDataParams(&dataParams);
            // DEBUG_PRINTF("--> Advertising Indication Received\n");
            PalBbBleTxTifsData(&scanReqBuf, 1);
            // PalBbBleTxData(&scanReqBuf, 1);
            break;
        case SCN_SEND_CONN_IND:
            // DEBUG_PRINTF("--> Scan Response Received\n");
            scanState = SCN_CONNECTED;
            PalBbBleTxTifsData(&connIndBuf, 1);
            break;
        default:
            DEBUG_PRINTF("STATE ERROR. RESTARTING.");
            scanState = SCN_LISTEN_ADV_IND;
    }
    
}

/*************************************************************************************************/
void scanTxCallback(uint8_t status)
{
    if (status != BB_STATUS_SUCCESS)
    {
        DEBUG_PRINTF("** TX FAILED WITH STATUS %i **\n", status);
        scanEvtStatus = status;
        scanState = SCN_LISTEN_ADV_IND;
        PalBbBleCancelTifs();
        scanEvtFinished = TRUE;
        return;
    }

    switch (scanState) {
        case SCN_LISTEN_SCAN_RSP:
            memset(scanRxData, 0, sizeof(scanRxData));
            scanState = SCN_SEND_CONN_IND;
            PalBbBleRxTifsData(scanRxData, MAX_RX_LEN);
            break;
        case SCN_CONNECTED:
            DEBUG_PRINTF("--> Connection Established\n");
            // scanEvtStatus = BB_STATUS_SUCCESS;
            scanEvtStatus = BB_STATUS_FAILED;
            scanState = SCN_LISTEN_ADV_IND;
            PalBbBleCancelTifs();
            scanEvtFinished = TRUE;
            break;
        default:
            DEBUG_PRINTF("STATE ERROR. RESTARTING.");
            scanState = SCN_LISTEN_ADV_IND;
    }
}
