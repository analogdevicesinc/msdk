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

#include "cordioless_scan.h"
#include "common.h"

/*******************************************************************************
  Definitions
*******************************************************************************/
#define MAX_RX_LEN          39
#define SCAN_REQ_HDR        0x0C03
#define CONN_IND_HDR        0x2225
#define CONN_DATA_LEN       22
#define SCAN_REQ_PACKET_LEN 14
#define CONN_IND_PACKET_LEN 36

/*! \brief  Scanner states */
enum {
    SCAN_IDLE,
    SCAN_LISTEN_ADV_IND,
    SCAN_SEND_SCAN_REQ,
    SCAN_LISTEN_SCAN_RSP,
    SCAN_SEND_CONN_IND,
    SCAN_CONNECTED
};

/*! \brief  Scanner control block */
typedef struct
{
    uint8_t prevState;
    uint8_t currState;
    uint8_t nextState;

    uint8_t *addrA;
    uint8_t *addrS;

    ConnDataParams_t connParams;

    PalBbBleTxBufDesc_t scanReqBuf;
    PalBbBleTxBufDesc_t connIndBuf;
} scanCb_t;

/*******************************************************************************
  Global Variables
*******************************************************************************/
uint8_t scanRxData[MAX_RX_LEN];
uint8_t scanReqTxData[SCAN_REQ_PACKET_LEN];
uint8_t connIndTxData[CONN_IND_PACKET_LEN];
static scanCb_t scanCb;
volatile uint8_t scan_commFinished;
volatile uint8_t scan_commStatus;

/*******************************************************************************
  Functions
*******************************************************************************/
void scanRxCallback(uint8_t status, int8_t rssi, uint32_t crc, uint32_t timestamp, uint8_t rxPhyOptions);
void scanTxCallback(uint8_t status);
void fillScanReqPacket(void);
void fillConnIndPacket(void);

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
    scanCb.addrA = advAddr;
    scanCb.addrS = scanAddr;

    // scanCb.scanReq_pktLen = HEADER_LEN + 2*ADDR_LEN;
    // scanCb.connInd_pktLen = HEADER_LEN + 2*ADDR_LEN + CONN_DATA_LEN;
    scanCb.connParams = *pConn;
    // PalBbBleTxBufDesc_t scanReq;
    // PalBbBleTxBufDesc_t connInd;
    PalBbBleSetChannelParam(pChan);
    fillScanReqPacket();
    fillConnIndPacket();

    scanCb.scanReqBuf.pBuf = scanReqTxData;
    scanCb.scanReqBuf.len = SCAN_REQ_PACKET_LEN;
    // scanCb.scanReqBuf.len = SCAN_REQ_PACKET_LEN;
    scanCb.connIndBuf.pBuf = connIndTxData;
    scanCb.connIndBuf.len = CONN_IND_PACKET_LEN;
    // scanCb.scanReqBuf = scanReq;
    // scanCb.connIndBuf = connInd;

    scanCb.prevState = SCAN_IDLE;
    scanCb.currState = SCAN_IDLE;
    scanCb.nextState = SCAN_IDLE;
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
void fillScanReqPacket(void)
{
    uint8_t i;
    uint8_t pktIdx = 0;
    
    scanReqTxData[pktIdx++] = SCAN_REQ_HDR & 0xFF;
    scanReqTxData[pktIdx++] = (SCAN_REQ_HDR & 0xFF00) >> 8;
    for (i=0; i<ADDR_LEN; i++)
    {
        scanReqTxData[pktIdx++] = scanCb.addrS[i];
    }
    for (i=0; i<ADDR_LEN; i++)
    {
        scanReqTxData[pktIdx++] = scanCb.addrA[i];
    }
    // bufDesc->len = scanCb.scanReq_pktLen;
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
void fillConnIndPacket(void)
{
    uint8_t i;
    uint8_t pktIdx = 0;

    connIndTxData[pktIdx++] = CONN_IND_HDR & 0xFF;
    connIndTxData[pktIdx++] = (CONN_IND_HDR & 0xFF00) >> 8;

    for (i=0; i<ADDR_LEN; i++)
    {
        connIndTxData[pktIdx++] = scanCb.addrS[i];
    }

    for (i=0; i<ADDR_LEN; i++)
    {
        connIndTxData[pktIdx++] = scanCb.addrA[i];
    }

    connIndTxData[pktIdx++] = (scanCb.connParams.accessAddress & 0xFF000000) >> 24;
    connIndTxData[pktIdx++] = (scanCb.connParams.accessAddress & 0xFF0000) >> 16;
    connIndTxData[pktIdx++] = (scanCb.connParams.accessAddress & 0xFF00) >> 8;
    connIndTxData[pktIdx++] = (scanCb.connParams.accessAddress & 0xFF);

    connIndTxData[pktIdx++] = (scanCb.connParams.crcInit & 0xFF);
    connIndTxData[pktIdx++] = (scanCb.connParams.crcInit & 0xFF00) >> 8;
    connIndTxData[pktIdx++] = (scanCb.connParams.crcInit & 0xFF0000) >> 16;

    connIndTxData[pktIdx++] = scanCb.connParams.winSize;
    
    connIndTxData[pktIdx++] = scanCb.connParams.winOffset & 0xFF;
    connIndTxData[pktIdx++] = (scanCb.connParams.winOffset & 0xFF00) >> 8;

    connIndTxData[pktIdx++] = scanCb.connParams.interval & 0xFF;
    connIndTxData[pktIdx++] = (scanCb.connParams.interval & 0xFF00) >> 8;

    connIndTxData[pktIdx++] = scanCb.connParams.latency & 0xFF;
    connIndTxData[pktIdx++] = (scanCb.connParams.latency & 0xFF00) >> 8;

    connIndTxData[pktIdx++] = scanCb.connParams.timeout & 0xFF;
    connIndTxData[pktIdx++] = (scanCb.connParams.timeout & 0xFF00) >> 8;

    for (i=0; i<CHM_LEN; i++)
    {
        connIndTxData[pktIdx++] = scanCb.connParams.chM[i];
    }

    connIndTxData[pktIdx++] = scanCb.connParams.hop | scanCb.connParams.sca;
    // bufDesc->len = scanCb.connInd_pktLen;
}

/*************************************************************************************************/
/*!
 *  \brief      Start scanning.
 *
 *  \param      scanWin             Scanning window.
 * 
 *  \return     TRUE if a connection was established, else FALSE.
 */
/*************************************************************************************************/
bool_t startScanning(uint32_t scanWin)
{
    bool_t evtFinished = FALSE;
    bool_t connectionMade = FALSE;
    // PalBbBleTxBufDesc_t txBuf;
    const uint8_t txdata[2] = {0x0, 0x30};
    while (!evtFinished)
    {
        switch (scanCb.currState)
        {
            case SCAN_IDLE:
                {

                memset(scanRxData, 0, sizeof(scanRxData));
                common_clearTifs();
                uint32_t dueTime;
                if (!PalBbGetTimestamp(&dueTime))
                {
                    evtFinished = TRUE;
                    scanCb.nextState = SCAN_IDLE;
                    DEBUG_PRINTF("Invalid due time received.\n");
                    break;
                }
                dueTime += DUE_OFFSET_US;
                PalBbBleDataParam_t dataParams = {
                    .txCback = scanTxCallback,
                    .rxCback = scanRxCallback,
                    .dueUsec = dueTime,
                    .rxTimeoutUsec = scanWin
                };
                PalBbBleSetDataParams(&dataParams);
                scanCb.nextState = SCAN_LISTEN_ADV_IND;
                break;
                }
            case SCAN_LISTEN_ADV_IND:
                scan_commFinished = FALSE;
                PalBbBleRxData(scanRxData, MAX_RX_LEN);
                while (!scan_commFinished) {}
                if (scan_commStatus != BB_STATUS_SUCCESS)
                {
                    evtFinished = TRUE;
                    scanCb.nextState = SCAN_IDLE;
                    DEBUG_PRINTF("RX ADV_IND failed with status %u\n", scan_commStatus);
                    break;
                }
                // DEBUG_PRINTF("--> Advertising Indicator Received.");
                common_setTifs();
                scanCb.nextState = SCAN_SEND_SCAN_REQ;
                break;
            case SCAN_SEND_SCAN_REQ:
                {
                    // common_setTifs();
                // PalBbBleDataParam_t dataParams = {
                //     .txCback = scanTxCallback,
                //     .rxCback = scanRxCallback,
                //     // .dueUsec = PalBbGetCurrentTime() + 150,
                //     .rxTimeoutUsec = scanWin
                // };
                // PalBbBleSetDataParams(&dataParams);
                // static volatile PalBbBleTxBufDesc_t txData1;
                // txData1.len = 14;
                // txData1.pBuf = scanReqTxData;
         
                // PalBbBleTxTifsData(&txData1, 1);
                PalBbBleTxTifsData(&scanCb.scanReqBuf, 1);
                scan_commFinished = FALSE;
                while (!scan_commFinished) {}
                if (scan_commStatus != BB_STATUS_SUCCESS)
                {
                    evtFinished = TRUE;
                    scanCb.nextState = SCAN_IDLE;
                    DEBUG_PRINTF("TX SCAN_REQ failed with status %u\n", scan_commStatus);
                    break;
                }
                scanCb.nextState = SCAN_LISTEN_SCAN_RSP;
                break;

                }
            case SCAN_LISTEN_SCAN_RSP:
                memset(scanRxData, 0, sizeof(scanRxData));
                scan_commFinished = FALSE;
                PalBbBleRxTifsData(scanRxData, MAX_RX_LEN);
                while (!scan_commFinished) {}
                if (scan_commStatus != BB_STATUS_SUCCESS)
                {
                    evtFinished = TRUE;
                    scanCb.nextState = SCAN_IDLE;
                    DEBUG_PRINTF("RX SCAN_RSP failed with status %u\n", scan_commStatus);
                    break;
                }
                // DEBUG_PRINTF("--> Scan Response Received\n");
                scanCb.nextState = SCAN_SEND_CONN_IND;
                break;
            case SCAN_SEND_CONN_IND:
            {
                // PalBbBleTxBufDesc_t txData2;
                // txData2.len = 36;
                //     // .pBuf = (uint8_t[2]){0x0, 0x30}
                // txData2.pBuf = connIndTxData;
               
                scan_commFinished = FALSE;
                // PalBbBleTxTifsData(&txData2, 1);
                PalBbBleTxTifsData(&scanCb.connIndBuf, 1);
                while (!scan_commFinished) {}
                if (scan_commStatus != BB_STATUS_SUCCESS)
                {
                    evtFinished = TRUE;
                    scanCb.nextState = SCAN_IDLE;
                    DEBUG_PRINTF("RX CONN_IND failed with status %u\n", scan_commStatus);
                    break;
                }
                scanCb.nextState = SCAN_CONNECTED;
                break;

            }
            case SCAN_CONNECTED:
                connectionMade = TRUE;
                // DEBUG_PRINTF("--> Connection Established\n");
                evtFinished = TRUE;
                scanCb.nextState = SCAN_CONNECTED;
                break;
            default:
                DEBUG_PRINTF("State error encountered. Aborting SCAN event.\n");
                scanCb.prevState = SCAN_IDLE;
                scanCb.currState = SCAN_IDLE;
                scanCb.nextState = SCAN_IDLE;
                evtFinished = TRUE;
        }
        scanCb.prevState = scanCb.currState;
        scanCb.currState = scanCb.nextState;
    }
    return connectionMade;
}

/*************************************************************************************************/
void scanRxCallback(uint8_t status, int8_t rssi, uint32_t crc, uint32_t timestamp, uint8_t rxPhyOptions)
{
    // UNUSED
    (void)rssi;
    (void)crc;
    (void)timestamp;
    (void)rxPhyOptions;

    scan_commStatus = status;
    scan_commFinished = TRUE;


}

/*************************************************************************************************/
void scanTxCallback(uint8_t status)
{
    scan_commStatus = status;
    scan_commFinished = TRUE;

    
}

