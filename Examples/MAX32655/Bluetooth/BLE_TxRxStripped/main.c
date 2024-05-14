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
#include "scn_stripped.h"

/*******************************************************************************
  Definitions
*******************************************************************************/
#define MAX_RX_LEN          256

/** \brief  Role Options*/
#define SCANNING_ROLE       0
#define ADVERTISING_ROLE    1

/* Role setup */
#ifndef ROLE
#define ROLE                ADVERTISING_ROLE
#endif

/* Parameter Definition */
#define BB_PHY              BB_PHY_BLE_1M
#define BB_PHY_OPTS         BB_PHY_OPTIONS_DEFAULT
#define TX_POWER            0
#define ADV_INTERVAL_US     60000U
#define SCAN_INTERVAL_US    160000U
#define SCAN_WINDOW_US      150000U
#define CHANNEL             0
#define ADV_ADDR            0x102030405060U
#define SCAN_ADDR           0x010203040506U

#define ADV_IND_PLD_LEN     0x1FU
#define SCAN_RSP_PLD_LEN    0x1FU

/* Connection Parameters */
#define ACCESS_ADDRESS      0x71764129U
#define CRC_INIT            0x555555U
#define WIN_SIZE            0xFFU
#define WIN_OFFSET          0x100U
#define CONN_INTERVAL       0x06U
#define PERIPH_LATENCY      0x00
#define SUP_TIMEOUT         0x64U
#define CHANNEL_MAP         0x1FFFFFFFFFU
#define HOP                 0xAU
#define SCA                 0x0U

#define DUE_OFFSET          1000
#define MAX_RX_LEN          256

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
volatile uint8_t procStatus;
volatile bool_t procDone;
volatile bool_t tstRxDone;
volatile uint8_t tstRxStatus;
volatile bool_t tstTxDone;
volatile uint8_t tstTxStatus;
uint8_t bbRxData[MAX_RX_LEN];
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
// static uint8_t bbTxData[] = {0};
/*******************************************************************************
  Functions
*******************************************************************************/
void connRxCallback(uint8_t status, int8_t rssi, uint32_t crc, uint32_t timestamp, uint8_t rxPhyOptions);
void connTxCallback(uint8_t status);
void setConnTxDataParams(void);
void setConnRxDataParams(uint16_t connInterval);
void testScan(void);
void testTxCallback(uint8_t status);
void testRxCallback(uint8_t status, int8_t rssi, uint32_t crc, uint32_t timestamp, uint8_t rxPhyOptions);

/******************************************************************************/
void testTxCallback(uint8_t status) {
    tstTxStatus = status;
    tstTxDone = TRUE;
}

void testRxCallback(uint8_t status, int8_t rssi, uint32_t crc, uint32_t timestamp, uint8_t rxPhyOptions) {
    tstRxStatus = status;
    tstRxDone = TRUE;
}

void connTxCallback(uint8_t status)
{
    if (status != BB_STATUS_SUCCESS)
    {
        DEBUG_PRINTF("** CONNECTION: ...transmit ERROR, Status=%i **\n", status);
    }
    else{
        DEBUG_PRINTF("--> CONNECTION: ...transmit done.\n");
    }
    procStatus = status;
    procDone = TRUE;

}

/******************************************************************************/
void connRxCallback(uint8_t status, int8_t rssi, uint32_t crc, uint32_t timestamp, uint8_t rxPhyOptions)
{
    if (status != BB_STATUS_SUCCESS) {
        DEBUG_PRINTF("** CONNECTION: ...packet ERROR, Status=%i **\n", status);
    }
    else
    {
        DEBUG_PRINTF("--> CONNECTION: ...packet received.\n");
    }
    procStatus = status;
    procDone = TRUE;
}

/******************************************************************************/

void testScan(void) {
    uint8_t tstTxPktData[14] = {
        0
    };
    uint8_t tstRxBuf[14];
    PalBbBleChan_t chanParams = {
        .opType = 0,
        .chanIdx = CHANNEL,
        .txPower = TX_POWER,
        .accAddr = ACCESS_ADDRESS,
        .crcInit = CRC_INIT,
        .txPhy = BB_PHY,
        .rxPhy = BB_PHY,
        .initTxPhyOptions = BB_PHY_OPTS,
        .tifsTxPhyOptions = BB_PHY_OPTS,
        .peerTxStableModIdx = FALSE,
        .peerRxStableModIdx = FALSE
    };
    PalBbBleSetChannelParam(&chanParams);
    uint32_t dueTime;
    PalBbBleOpParam_t opParams = { 0 };
    PalBbBleSetOpParams(&opParams);
    tstRxDone = FALSE;
    do {
        PalBbGetTimestamp(&dueTime);
        dueTime += DUE_OFFSET;
        PalBbBleDataParam_t dataParams = {
            .txCback = testTxCallback,
            .rxCback = testRxCallback,
            .dueUsec = dueTime,
            .rxTimeoutUsec = 100000
        };
        PalBbBleSetDataParams(&dataParams);
        PalBbBleRxData(tstRxBuf, 14);
        while (!tstRxDone) {}
    } while (tstRxStatus != 0);
    opParams.ifsMode = PAL_BB_IFS_MODE_TOGGLE_TIFS;
    
    PalBbBleSetOpParams(&opParams);
    PalBbBleTxBufDesc_t testTxPkt = {
        .len = 14,
        .pBuf = tstTxPktData
    };
    while (1) {
        tstRxDone = FALSE;
        tstTxDone = FALSE;
        PalBbBleTxTifsData(&testTxPkt, 1);
        while (!tstTxDone) {}
        PalBbBleRxTifsData(tstRxBuf, 14);
        while (!tstRxDone) {}
    }
}
void enterScanLoop(void)
{
    uint32_t scanTime = SCAN_WINDOW_US;
    uint32_t connInterval_uS = (CONN_INTERVAL*10)/8; // Multiply by 1.25 to convert to uS.
    uint8_t advAddress[6];
    uint8_t scanAddress[6];
    uint8_t channelMap[5];
    uint8_t i; 
    for (i=0; i<6; i++) {
        advAddress[i] = (ADV_ADDR & (0xFF << (8*i))) >> (8*i);
        scanAddress[i] = (SCAN_ADDR & (0xFF << (8*i))) >> (8*i);
        if (i < 5) {
            channelMap[i] = (CHANNEL_MAP & (0xFF << (8*i))) >> (8*i);
        }
    }
    PalBbBleChan_t chanParams = {
        .opType = 0,
        .chanIdx = CHANNEL,
        .txPower = TX_POWER,
        .accAddr = ACCESS_ADDRESS,
        .crcInit = CRC_INIT,
        .txPhy = BB_PHY,
        .rxPhy = BB_PHY,
        .initTxPhyOptions = BB_PHY_OPTS,
        .tifsTxPhyOptions = BB_PHY_OPTS,
        .peerTxStableModIdx = FALSE,
        .peerRxStableModIdx = FALSE
    };
    ConnDataParams_t connParams = {
        .accessAddress = ACCESS_ADDRESS,
        .crcInit = CRC_INIT,
        .winSize = WIN_SIZE,
        .winOffset = WIN_OFFSET,
        .interval = CONN_INTERVAL,
        .latency = PERIPH_LATENCY,
        .timeout = SUP_TIMEOUT,
        .chM = channelMap,
        .hop = HOP,
        .sca = SCA
    };
    PalBbBleTxBufDesc_t connTxPkt = {
        .pBuf = bbTxData,
        .len = 251
    };

    initScanner(advAddress, scanAddress, &chanParams, &connParams);
    bool_t connEstablished;
    uint32_t now;
    uint32_t intervalEnd;
    do {
        PalBbGetTimestamp(&now);
        intervalEnd = now + SCAN_INTERVAL_US;
        connEstablished = startScanning(scanTime);
        printf("%u\n", connEstablished);
        PalBbGetTimestamp(&now);
        while (now < intervalEnd) { PalBbGetTimestamp(&now); }
    } while (!connEstablished);
    // while (!startScanning(scanInterval)) {}
    while (1) {
        DEBUG_PRINTF("--> CONNECTION: transmitting...\n");
        setConnTxDataParams();
        PalBbBleTxData(&connTxPkt, 1);
        while (!procDone) {}
        // MXC_Delay(connInterval_uS);
        MXC_Delay(1000000);
    }
}

/******************************************************************************/
void enterAdvLoop(void)
{
    uint8_t advIndPldLength = ADV_IND_PLD_LEN;
    uint8_t scanRspPldLength = SCAN_RSP_PLD_LEN;
    uint16_t connWindow = (WIN_SIZE*10)/8; // Multiply by 1.25 to convert to uS.
    uint8_t advAddress[6];
    // uint8_t scanAddress[6];
    // uint8_t channelMap[5];
    uint8_t i; 
    for (i=0; i<6; i++) {
        advAddress[i] = (ADV_ADDR & (0xFF << (8*i))) >> (8*i);
        // scanAddress[i] = (SCAN_ADDR & (0xFF << (8*i))) >> (8*i);
        // if (i < 5) {
        //     channelMap[i] = (CHANNEL_MAP & (0xFF << (8*i))) >> (8*i);
        // }
    }

    PalBbBleChan_t chanParams = {
        .opType = 0,
        .chanIdx = CHANNEL,
        .txPower = TX_POWER,
        .accAddr = ACCESS_ADDRESS,
        .crcInit = CRC_INIT,
        .txPhy = BB_PHY,
        .rxPhy = BB_PHY,
        .initTxPhyOptions = BB_PHY_OPTS,
        .tifsTxPhyOptions = BB_PHY_OPTS,
        .peerTxStableModIdx = FALSE,
        .peerRxStableModIdx = FALSE
    };

    initAdvertiser(advAddress, advIndPldLength, scanRspPldLength, &chanParams);
    uint32_t now;
    uint32_t intervalEnd;
    while (1) {
        PalBbGetTimestamp(&now);
        intervalEnd = now + ADV_INTERVAL_US;
        DEBUG_PRINTF("ENTER");
        if (sendAdvIndPacket()) {
            break;
        }
        if (sendAdvIndPacket()) {
            break;
        }
        if (sendAdvIndPacket()) {
            break;
        }
        PalBbGetTimestamp(&now);
        while (now < intervalEnd) { PalBbGetTimestamp(&now); }
    }
    while (1) {
        memset(bbRxData, 0, sizeof(bbRxData));
        DEBUG_PRINTF("--> CONNECTION: waiting for packet...\n");
        setConnRxDataParams(connWindow);
        PalBbBleRxData(bbRxData, MAX_RX_LEN);
    }
}

/*************************************************************************************************/
/*!
 *  \brief      Set connection RX data params.
 *
 *  \param      connInterval        Connection interval.
 * 
 *  \return     TRUE if a connection was established, else FALSE.
 */
/*************************************************************************************************/
void setConnRxDataParams(uint16_t connInterval) {
    uint32_t dueTime;
    PalBbGetTimestamp(&dueTime);
    dueTime += DUE_OFFSET;
    PalBbBleDataParam_t dataParams = {
        .txCback = connTxCallback,
        .rxCback = connRxCallback,
        .dueUsec = dueTime,
        .rxTimeoutUsec = connInterval
    };
    PalBbBleSetDataParams(&dataParams);
}

/*************************************************************************************************/
/*!
 *  \brief      Set connection TX data params.
 * 
 *  \return     TRUE if a connection was established, else FALSE.
 */
/*************************************************************************************************/
void setConnTxDataParams(void) {
    uint32_t dueTime;
    PalBbGetTimestamp(&dueTime);
    dueTime += DUE_OFFSET;
    PalBbBleDataParam_t dataParams = {
        .txCback = connTxCallback,
        .rxCback = connRxCallback,
        .dueUsec = dueTime,
        .rxTimeoutUsec = 2000
    };
    PalBbBleSetDataParams(&dataParams);
}

/******************************************************************************/
int main(void)
{
#if (ROLE == SCANNING_ROLE)
    DEBUG_PRINTF("\n****** STRIPPED TXRX -- SCANNER ******\n");
#endif
#if (ROLE == ADVERTISING_ROLE)
    DEBUG_PRINTF("\n****** STRIPPED TXRX -- ADVERTISER ******\n");
#endif
    LED_Off(1);

    /* Initialize baseband */
    PalBbInit();
    PalBbEnable();

    PalBbBleInit();
    PalBbBleEnable();

#if (ROLE == SCANNING_ROLE)
    testScan();
    // enterScanLoop();
#endif
#if (ROLE == ADVERTISING_ROLE)
    enterAdvLoop();
#endif

    return 0;
}