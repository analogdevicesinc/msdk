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
#include "cordioless_scan.h"
#include "common.h"

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
#define PHY                 BB_PHY_BLE_1M
#define PHY_OPTS            BB_PHY_OPTIONS_DEFAULT
#define TX_POWER            0
#define ADV_INTERVAL_US     60000u
#define SCAN_INTERVAL_US    160000u
#define SCAN_WINDOW_US      150000u
#define CHANNEL             0

#define ADV_IND_PLD_LEN     0x1Fu
#define SCAN_RSP_PLD_LEN    0x1Fu

/* Connection Parameters */
#define ACCESS_ADDR         0x71764129u
#define CRC_INIT            0x555555u
#define WIN_SIZE            0xFFu
#define WIN_OFFSET          0x100u
#define CONN_INTERVAL       0x06u
#define PERIPH_LATENCY      0x00
#define SUP_TIMEOUT         0x64u
#define HOP                 0xA
#define SCA                 0x0

/*******************************************************************************
  Global Variables
*******************************************************************************/
volatile uint8_t conn_commStatus;
volatile uint8_t conn_commFinished;
uint8_t connRxData[MAX_RX_LEN];
PalBbBleTxBufDesc_t connTxPkt;

uint8_t advAddress[ADDR_LEN] = {0x60, 0x50, 0x40, 0x30, 0x20, 0x10}; // Adv Address in transmit order.
uint8_t scanAddress[ADDR_LEN] = {0x06, 0x05, 0x04, 0x03, 0x02, 0x01}; // Scan Address in transmit order.
uint8_t chanMap[CHM_LEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0x1F}; // Channel Map in transmit order.

/*******************************************************************************
  Functions
*******************************************************************************/
void connRxCallback(uint8_t status, int8_t rssi, uint32_t crc, uint32_t timestamp, uint8_t rxPhyOptions);
void connTxCallback(uint8_t status);

/******************************************************************************/
void connTxCallback(uint8_t status)
{
    conn_commStatus = status;
    conn_commFinished = TRUE;
}

/******************************************************************************/
void connRxCallback(uint8_t status, int8_t rssi, uint32_t crc, uint32_t timestamp, uint8_t rxPhyOptions)
{
    // UNUSED
    (void)rssi;
    (void)crc;
    (void)timestamp;
    (void)rxPhyOptions;

    conn_commStatus = status;
    conn_commFinished = TRUE;
}

/*************************************************************************************************/
/*!
 *  \brief      Start scanning.
 * 
 *  \return     None.
 */
/*************************************************************************************************/
void enterScanLoop(void)
{
    uint32_t scanWin = SCAN_WINDOW_US;
    PalBbBleChan_t chanParams = {
        .opType = 0,
        .chanIdx = CHANNEL,
        .txPower = TX_POWER,
        .accAddr = ACCESS_ADDR,
        .crcInit = CRC_INIT,
        .txPhy = PHY,
        .rxPhy = PHY,
        .initTxPhyOptions = PHY_OPTS,
        .tifsTxPhyOptions = PHY_OPTS,
        .peerTxStableModIdx = FALSE,
        .peerRxStableModIdx = FALSE
    };
    ConnDataParams_t connParams = {
        .accessAddress = ACCESS_ADDR,
        .crcInit = CRC_INIT,
        .winSize = WIN_SIZE,
        .winOffset = WIN_OFFSET,
        .interval = CONN_INTERVAL,
        .latency = PERIPH_LATENCY,
        .timeout = SUP_TIMEOUT,
        .chM = chanMap,
        .hop = HOP,
        .sca = SCA
    };
    
    initScanner(advAddress, scanAddress, &chanParams, &connParams);
    bool_t connEstablished;
    uint32_t now;
    uint32_t intervalEnd;

    do {
        PalBbGetTimestamp(&now);
        intervalEnd = now + SCAN_INTERVAL_US;
        connEstablished = startScanning(scanWin);
        if (!connEstablished) {
            PalBbGetTimestamp(&now);
            while(now < intervalEnd) { PalBbGetTimestamp(&now); }
        }
    } while (!connEstablished);
    DEBUG_PRINTF("CONN_ESTABLISHED");
    MXC_Delay(500000);
}

/*************************************************************************************************/
/*!
 *  \brief      Start advertising.
 * 
 *  \return     None.
 */
/*************************************************************************************************/
void enterAdvLoop(void)
{
    uint8_t advInd_payloadLen = ADV_IND_PLD_LEN;
    uint8_t scanRsp_payloadLen = SCAN_RSP_PLD_LEN;
    PalBbBleChan_t chanParams = {
        .opType = 0,
        .chanIdx = CHANNEL,
        .txPower = TX_POWER,
        .accAddr = ACCESS_ADDR,
        .crcInit = CRC_INIT,
        .txPhy = PHY,
        .rxPhy = PHY,
        .initTxPhyOptions = PHY_OPTS,
        .tifsTxPhyOptions = PHY_OPTS,
        .peerTxStableModIdx = FALSE,
        .peerRxStableModIdx = FALSE
    };

    initAdvertiser(advAddress, advInd_payloadLen, scanRsp_payloadLen, &chanParams);
    uint32_t now;
    uint32_t intervalEnd;

    while (1) {
        PalBbGetTimestamp(&now);
        intervalEnd = now + ADV_INTERVAL_US;
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
    DEBUG_PRINTF("CONN ESTABLISHED");
    MXC_Delay(500000);
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
    while(1)
    {
        enterScanLoop();
    }
#endif
#if (ROLE == ADVERTISING_ROLE)
    while(1)
    {
        enterAdvLoop();
    }
#endif

    return 0;
}