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
#include "pal_radio.h"

#include "cordioless_adv.h"
#include "common.h"

/*******************************************************************************
  Definitions
*******************************************************************************/
#define MAX_RX_LEN          36
#define ADV_IND_HDR(x)      (uint16_t)(0x20 | (x << 8))
#define SCAN_RSP_HDR(x)     (uint16_t)(0x04 | (x << 8))
#define MAX_PACKET_LEN      39

/*! \brief  Advertiser states */
enum {
    ADV_IDLE,
    ADV_SEND_ADV_IND,
    ADV_LISTEN_SCAN_REQ,
    ADV_SEND_SCAN_RSP,
    ADV_LISTEN_CONN_IND,
    ADV_CONNECTED
};

/*! \brief  Advertiser control block */
typedef struct
{
    uint8_t prevState;
    uint8_t currState;
    uint8_t nextState;

    uint8_t *addrA;

    PalBbBleTxBufDesc_t advIndBuf;
    PalBbBleTxBufDesc_t scanRspBuf;
} advCb_t;

/*******************************************************************************
  Global Variables
*******************************************************************************/
uint8_t advRxData[MAX_RX_LEN];
uint8_t advIndTxData[MAX_PACKET_LEN];
uint8_t scanRspTxData[MAX_PACKET_LEN];
static advCb_t advCb;
volatile uint8_t adv_commFinished;
volatile uint8_t adv_commStatus;

/*******************************************************************************
  Functions
*******************************************************************************/
void advRxCallback(uint8_t status, int8_t rssi, uint32_t crc, uint32_t timestamp, uint8_t rxPhyOptions);
void advTxCallback(uint8_t status);
void fillAdvIndPacket(uint8_t payloadLen);
void fillScanRspPacket(uint8_t payloadLen);

/*************************************************************************************************/
/*!
 *  \brief      Initialize pseudo-advertiser.
 *
 *  \param      advAddr             Advertising address.
 *  \param      advIndPldLen        Advertising indicator packet payload length.
 *  \param      scanRspPldLen       Scan response packet payload length.
 *  \param      pChan               Channelization parameters.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void initAdvertiser(uint8_t *advAddr, uint8_t advInd_payloadLen, uint8_t scanRsp_payloadLen, PalBbBleChan_t *pChan)
{
    advCb.addrA = advAddr;
    uint8_t advInd_pktLen = HEADER_LEN + ADDR_LEN + advInd_payloadLen;
    uint8_t scanRsp_pktLen = HEADER_LEN + ADDR_LEN + scanRsp_payloadLen;

    PalBbBleSetChannelParam(pChan);
    fillAdvIndPacket(advInd_payloadLen);
    fillScanRspPacket(scanRsp_payloadLen);

    advCb.advIndBuf.pBuf = advIndTxData;
    advCb.advIndBuf.len = advInd_pktLen;
    advCb.scanRspBuf.pBuf = scanRspTxData;
    advCb.scanRspBuf.len = scanRsp_pktLen;

    advCb.prevState = ADV_IDLE;
    advCb.currState = ADV_IDLE;
    advCb.nextState = ADV_IDLE;
}

/*************************************************************************************************/
/*!
 *  \brief      Create a pseudo-advertising indicator packet.
 *
 *  \param      bufDesc             Pointer to the packet buffer descriptor.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void fillAdvIndPacket(uint8_t payloadLen)
{
    uint8_t i;
    // uint8_t pldLen = advCb.advInd_pktLen - HEADER_LEN;
    uint16_t pktHdr = ADV_IND_HDR(payloadLen);
    uint8_t *pldData = common_getData();
    uint8_t pktIdx = 0;
    // uint8_t val = (uint8_t)(pktHdr & 0x00FFu);
    advIndTxData[pktIdx++] = pktHdr & 0xFF;
    advIndTxData[pktIdx++] = (pktHdr & 0xFF00) >> 8;
    for (i=0; i<ADDR_LEN; i++)
    {
        advIndTxData[pktIdx++] = advCb.addrA[i];
    }
    for (i=0; i<payloadLen; i++) {
        advIndTxData[pktIdx++] = pldData[i];
    }
    // bufDesc->len = pldLen + HEADER_LEN;
}

/*************************************************************************************************/
/*!
 *  \brief      Create a pseudo-scan response packet.
 *
 *  \param      bufDesc             Pointer to the packet buffer descriptor.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void fillScanRspPacket(uint8_t payloadLen)
{
    uint8_t i;
    uint16_t pktHdr = SCAN_RSP_HDR(payloadLen);
    uint8_t *pldData = common_getData();
    uint8_t pktIdx = 0;

    scanRspTxData[pktIdx++] = pktHdr & 0xFF;
    scanRspTxData[pktIdx++] = (pktHdr & 0xFF00) >> 8;
    for (i=0; i<ADDR_LEN; i++)
    {
        scanRspTxData[pktIdx++] = advCb.addrA[i];
    }
    for (i=0; i<payloadLen; i++) {
        scanRspTxData[pktIdx++] = pldData[i];
    }
    // bufDesc->len = pldLen + HEADER_LEN;
}

/*************************************************************************************************/
/*!
 *  \brief      Send an advertising indicator packet.
 *
 *  \return     TRUE if a connection was established, else FALSE.
 */
/*************************************************************************************************/
bool_t sendAdvIndPacket(void)
{
    bool_t evtFinished = FALSE;
    bool_t connectionMade = FALSE;
    PalBbBleTxBufDesc_t txBuf;
    uint8_t txdata[2] = {0x0, 0x30};
    while (!evtFinished)
    {
        switch (advCb.currState)
        {
            case ADV_IDLE:
                common_clearTifs();
                uint32_t dueTime;
                if (!PalBbGetTimestamp(&dueTime))
                {
                    evtFinished = TRUE;
                    advCb.nextState = ADV_IDLE;
                    DEBUG_PRINTF("Invalid due time received.\n");
                    break;
                }
                dueTime += DUE_OFFSET_US;
                PalBbBleDataParam_t dataParams = {
                    .txCback = advTxCallback,
                    .rxCback = advRxCallback,
                    .dueUsec = dueTime,
                    .rxTimeoutUsec = RX_TIMEOUT_US
                };
                PalBbBleSetDataParams(&dataParams);
                advCb.nextState = ADV_SEND_ADV_IND;
                // DEBUG_PRINTF("HERE");
                break;
            case ADV_SEND_ADV_IND:
            {
                // txBuf.len = 39;
                // txBuf.pBuf = advIndTxData;
                adv_commFinished = FALSE;
                // static volatile PalBbBleTxBufDesc_t txData1;
                // txData1.len = 39;
                //     // .pBuf = (uint8_t[2]){0x0, 0x30}
                // txData1.pBuf = advIndTxData;
                // PalBbBleTxData(&advCb.advIndBuf, 1);
                // PalBbBleTxData(&txData1, 1);
                PalBbBleTxData(&advCb.advIndBuf, 1);
                while (!adv_commFinished) {}
                if (adv_commStatus != BB_STATUS_SUCCESS)
                {
                    evtFinished = TRUE;
                    advCb.nextState = ADV_IDLE;
                    DEBUG_PRINTF("TX ADV_IND failed with status %u\n", adv_commStatus);
                    break;
                }
                common_setTifs();
                advCb.nextState = ADV_LISTEN_SCAN_REQ;
                break;
            }
            case ADV_LISTEN_SCAN_REQ:
            {
                memset(advRxData, 0, sizeof(advRxData));
                adv_commFinished = FALSE;
                PalBbBleRxTifsData(advRxData, MAX_RX_LEN);
                while (!adv_commFinished) {}
                if (adv_commStatus != BB_STATUS_SUCCESS)
                {
                    evtFinished = TRUE;
                    advCb.nextState = ADV_IDLE;
                    DEBUG_PRINTF("RX SCAN_REQ failed with status %u\n", adv_commStatus);
                    break;
                }
                // DEBUG_PRINTF("--> Scan Request Received\n");
                advCb.nextState = ADV_SEND_SCAN_RSP;
                break;
            }
            case ADV_SEND_SCAN_RSP:
            {
                // txBuf.len = 39;
                // txBuf.pBuf = scanRspTxData;
                // PalBbBleTxBufDesc_t txData2;
                // txData2.len = 39;
                    // .pBuf = (uint8_t[2]){0x0, 0x30}
                // txData2.pBuf = scanRspTxData;
                adv_commFinished = FALSE;
                // PalBbBleTxTifsData(&advCb.scanRspBuf, 1);
                // PalBbBleTxTifsData(&txData2, 1);
                PalBbBleTxTifsData(&advCb.scanRspBuf, 1);
                while (!adv_commFinished) {}
                if (adv_commStatus != BB_STATUS_SUCCESS)
                {
                    evtFinished = TRUE;
                    advCb.nextState = ADV_IDLE;
                    DEBUG_PRINTF("TX SCAN_RSP failed with status %u\n", adv_commStatus);
                    break;
                }
                advCb.nextState = ADV_LISTEN_CONN_IND;
                break;
            }
            case ADV_LISTEN_CONN_IND:
            {
                memset(advRxData, 0, sizeof(advRxData));
                adv_commFinished = FALSE;
                PalBbBleRxTifsData(advRxData, MAX_RX_LEN);
                while (!adv_commFinished) {}
                if (adv_commStatus != BB_STATUS_SUCCESS)
                {
                    evtFinished = TRUE;
                    advCb.nextState = ADV_IDLE;
                    DEBUG_PRINTF("RX CONN_IND failed with status %u\n", adv_commStatus);
                    break;
                }
                advCb.nextState = ADV_CONNECTED;
                // DEBUG_PRINTF("--> Connection Indication Received\n");
                break;
            }
            case ADV_CONNECTED:
                connectionMade = TRUE;
                // DEBUG_PRINTF("--> Connection Established\n");
                evtFinished = TRUE;
                advCb.nextState = ADV_CONNECTED;
                break;
            default:
                DEBUG_PRINTF("State error encountered. Aborting ADV event.\n");
                advCb.prevState = ADV_IDLE;
                advCb.currState = ADV_IDLE;
                advCb.nextState = ADV_IDLE;
                evtFinished = TRUE;
        }
        advCb.prevState = advCb.currState;
        advCb.currState = advCb.nextState;
    }
    return connectionMade;
}

/*************************************************************************************************/
void advRxCallback(uint8_t status, int8_t rssi, uint32_t crc, uint32_t timestamp, uint8_t rxPhyOptions)
{
    // UNUSED
    (void)rssi;
    (void)crc;
    (void)timestamp;
    (void)rxPhyOptions;

    adv_commStatus = status;
    adv_commFinished = TRUE;
}

/*************************************************************************************************/
void advTxCallback(uint8_t status)
{
    adv_commStatus = status;
    adv_commFinished = TRUE;
}
