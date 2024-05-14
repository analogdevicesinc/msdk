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

#include "adv_stripped.h"

/*******************************************************************************
  Definitions
*******************************************************************************/
#define DUE_OFFSET              1000
#define RX_TIMEOUT              10000
#define MAX_RX_LEN              36
#define HEADER_LEN              2
#define ADDR_LEN                6
#define ADV_IND_HDR_MSB         0x04
#define SCAN_RSP_HDR_MSB        0x40
/*! \brief  Advertiser states */
enum {
    ADV_SEND_ADV_IND,
    ADV_LISTEN_SCAN_REQ,
    ADV_SEND_SCAN_RSP,
    ADV_LISTEN_CONN_IND,
    ADV_CONNECTED
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
uint8_t advRxData[MAX_RX_LEN];
uint8_t advState;


volatile bool_t advEvtFinished;
volatile bool_t advEvtStatus;

static uint8_t addrA[ADDR_LEN];
uint8_t advInd_pktLen;
uint8_t scanRsp_pktLen;

PalBbBleTxBufDesc_t advIndBuf;
PalBbBleTxBufDesc_t scanRspBuf;

static uint8_t bbTxData[] = {
    0xFF, 0xF9, 0xF8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01, 0x7F, 0xCA, 0xAE, 0x66, 0x88, 0x9E, 0x1D,
    0x7D, 0x35, 0x3B, 0x3A, 0x45, 0x8F, 0x21, 0x47, 0xCF, 0x51, 0x4C, 0x31, 0x04, 0x3F, 0x05, 0x40,
    0xCF, 0xEE, 0xA9, 0x66, 0x37, 0x7B, 0x4A, 0x4E, 0x71, 0x74, 0x34, 0xFB, 0x15, 0xBC, 0xDA, 0xED,
    0x96, 0xDC, 0x92, 0xE3, 0x97, 0xA3, 0x58, 0x4D, 0xF1, 0x2B, 0xC6, 0x50, 0x8C, 0x1E, 0xFD, 0x6A,
    0xC9, 0x91, 0xDC, 0x2D, 0x06, 0xC0, 0x90, 0x16, 0x01, 0x7F, 0xCA, 0xAE, 0x66, 0x88, 0x9E, 0x1D,
    0x7D, 0x35, 0x3B, 0x3A, 0x45, 0x8F, 0x21, 0x47, 0xCF, 0x51, 0x4C, 0x31, 0x04, 0x3F, 0x05, 0x40,
    0xCF, 0xEE, 0xA9, 0x66, 0x37, 0x7B, 0x4A, 0x4E, 0x71, 0x74, 0x34, 0xFB, 0x15, 0xBC, 0xDA, 0xED,
    0x96, 0xDC, 0x92, 0xE3, 0x97, 0xA3, 0x58, 0x4D, 0xF1, 0x2B, 0xC6, 0x50, 0x8C, 0x1E, 0xFD, 0x6A,
    0xC9, 0x91, 0xDC, 0x2D, 0x06, 0xC0, 0x90, 0x16, 0x01, 0x7F, 0xCA, 0xAE, 0x66, 0x88, 0x9E, 0x1D,
    0x7D, 0x35, 0x3B, 0x3A, 0x45, 0x8F, 0x21, 0x47, 0xCF, 0x51, 0x4C, 0x31, 0x04, 0x3F, 0x05, 0x40,
    0xCF, 0xEE, 0xA9, 0x66, 0x37, 0x7B, 0x4A, 0x4E, 0x71, 0x74, 0x34, 0xFB, 0x15, 0xBC, 0xDA, 0xED,
    0x96, 0xDC, 0x92, 0xE3, 0x97, 0xA3, 0x58, 0x4D, 0xF1, 0x2B, 0xC6, 0x50, 0x8C, 0x1E, 0xFD, 0x6A,
    0xC9, 0x91, 0xDC, 0x2D, 0x06, 0xC0, 0x90, 0x16, 0x01, 0x7F, 0xCA, 0xAE, 0x66, 0x88, 0x9E, 0x1D,
    0x7D, 0x35, 0x3B, 0x3A, 0x45, 0x8F, 0x21, 0x47, 0xCF, 0x51, 0x4C, 0x31, 0x04, 0x3F, 0x05, 0x40,
    0xCF, 0xEE, 0xA9, 0x66, 0x37, 0x7B, 0x4A, 0x4E, 0x71, 0x74, 0x34, 0xFB, 0x15, 0xBC, 0xDA, 0xED,
    0x96, 0xDC, 0x92, 0xE3, 0x97, 0xA3, 0x58, 0x4D, 0xF1, 0x2B, 0xC6,
};

/*******************************************************************************
  Functions
*******************************************************************************/
void advRxCallback(uint8_t status, int8_t rssi, uint32_t crc, uint32_t timestamp, uint8_t rxPhyOptions);
void advTxCallback(uint8_t status);
void fillAdvIndPacket(PalBbBleTxBufDesc_t *bufDesc);
void fillScanRspPacket(PalBbBleTxBufDesc_t *bufDesc);
bool_t sendAdvIndPacket(void);

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
void initAdvertiser(uint8_t *advAddr, uint8_t advIndPldLen, uint8_t scanRspPldLen, PalBbBleChan_t *pChan)
{   
    int i;
    for (i=0; i<ADDR_LEN; i++) {
        addrA[i] = advAddr[i];
    }

    advInd_pktLen = HEADER_LEN + ADDR_LEN + advIndPldLen;
    scanRsp_pktLen = HEADER_LEN + ADDR_LEN + scanRspPldLen;

    PalBbBleSetChannelParam(pChan);
    fillAdvIndPacket(&advIndBuf);
    fillScanRspPacket(&scanRspBuf);

    advState = ADV_SEND_ADV_IND;
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
void fillAdvIndPacket(PalBbBleTxBufDesc_t *bufDesc)
{   
    uint8_t pBuf[advInd_pktLen];
    uint8_t i;

    for (i=0; i<advInd_pktLen; i++)
    {
        pBuf[i] = bbTxData[i];
    }

    pBuf[0] = advInd_pktLen - (HEADER_LEN + ADDR_LEN);
    pBuf[1] = ADV_IND_HDR_MSB;
    for (i=0; i<ADDR_LEN; i++)
    {
        pBuf[i+HEADER_LEN] = addrA[(ADDR_LEN - 1) - i];
    }
    bufDesc->pBuf = pBuf;
    bufDesc->len = advInd_pktLen;
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
void fillScanRspPacket(PalBbBleTxBufDesc_t *bufDesc)
{
    uint8_t pBuf[scanRsp_pktLen];
    uint8_t i;
    for (i=0; i<scanRsp_pktLen; i++)
    {
        pBuf[i] = bbTxData[i];
    }

    pBuf[0] = scanRsp_pktLen - (HEADER_LEN + ADDR_LEN);
    pBuf[1] = SCAN_RSP_HDR_MSB;
    for (i=0; i<ADDR_LEN; i++)
    {
        pBuf[i+HEADER_LEN] = addrA[(ADDR_LEN - 1) - i];
    }

    bufDesc->pBuf = pBuf;
    bufDesc->len = scanRsp_pktLen;
}

/*************************************************************************************************/
/*!
 *  \brief      Send an advertising indicator.
 *
 *  \return     TRUE if a connection was established, else FALSE.
 */
/*************************************************************************************************/
bool_t sendAdvIndPacket(void)
{
    PalBbBleOpParam_t opParams = { 0 };
    PalBbBleSetOpParams(&opParams);
    advEvtFinished = FALSE;
    advState = ADV_LISTEN_SCAN_REQ;
    uint32_t dueTime;
    PalBbGetTimestamp(&dueTime);
    dueTime += DUE_OFFSET;
    PalBbBleDataParam_t dataParams = {
        .txCback = advTxCallback,
        .rxCback = advRxCallback,
        .dueUsec = dueTime,
        .rxTimeoutUsec = RX_TIMEOUT
    };
    PalBbBleSetDataParams(&dataParams);
    PalBbBleTxData(&advIndBuf, 1);
    while (!advEvtFinished) {}
    if (advEvtStatus != BB_STATUS_SUCCESS) {
        return FALSE;
    }
    return TRUE;
}

/*************************************************************************************************/
void advRxCallback(uint8_t status, int8_t rssi, uint32_t crc, uint32_t timestamp, uint8_t rxPhyOptions)
{
    if (status != BB_STATUS_SUCCESS)
    {
        DEBUG_PRINTF("** RX FAILED WITH STATUS %i **\n", status);
        advEvtStatus = status;
        advState = ADV_SEND_ADV_IND;
        PalBbBleCancelTifs();
        advEvtFinished = TRUE;
        return;
    }

    switch (advState) {
        case ADV_SEND_SCAN_RSP:
            // DEBUG_PRINTF("--> Scan Request Received\n");
            DEBUG_PRINTF("x");
            advState = ADV_LISTEN_CONN_IND;
            PalBbBleTxTifsData(&scanRspBuf, 1);
            break;
        case ADV_CONNECTED:
            advEvtStatus = BB_STATUS_SUCCESS;
            DEBUG_PRINTF("--> Connection Established\n");
            PalBbBleCancelTifs();
            advEvtFinished = TRUE;
            break;
        default:
            DEBUG_PRINTF("STATE ERROR. RESTARTING.");
            advState = ADV_SEND_ADV_IND;
    }
}

/*************************************************************************************************/
void advTxCallback(uint8_t status)
{
    if (status != BB_STATUS_SUCCESS) {
        DEBUG_PRINTF("** TX FAILED WITH STATUS %i **\n", status);
        advEvtStatus = status;
        advState = ADV_SEND_ADV_IND;
        PalBbBleCancelTifs();
        advEvtFinished = TRUE;
        return;
    }

    switch (advState) {
        case ADV_LISTEN_SCAN_REQ:
            memset(advRxData, 0, sizeof(advRxData));
            advState = ADV_SEND_SCAN_RSP;
            PalBbBleOpParam_t opParams = {
                .ifsMode = PAL_BB_IFS_MODE_TOGGLE_TIFS,
                .ifsTime = 0,
                .pIfsChan = NULL
            };
            PalBbBleSetOpParams(&opParams);
            PalBbBleRxTifsData(advRxData, MAX_RX_LEN);
            break;
        case ADV_LISTEN_CONN_IND:
            memset(advRxData, 0, sizeof(advRxData));
            advState = ADV_CONNECTED;
            PalBbBleRxTifsData(advRxData, MAX_RX_LEN);
            break;
        default:
            DEBUG_PRINTF("STATE ERROR. RESTARTING.");
            advState = ADV_SEND_ADV_IND;
    }
}

