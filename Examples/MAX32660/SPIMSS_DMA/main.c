/******************************************************************************
 *
 * Copyright (C) 2023-2024 Analog Devices, Inc. All Rights Reserved. This
 * software is proprietary to Analog Devices, Inc. and its licensors.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

/**
 * @file    main.c
 * @brief   SPI Master DMA Demo
 * @details Shows Master DMA loopback demo for SPI1 (AKA: SPIMSS)
 *          Read the printf() for instructions
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "board.h"
#include "led.h"
#include "mxc_pins.h"
#include "spi.h"
#include "spimss.h"
#include "uart.h"
#include "dma.h"
#include "mxc_delay.h"
#include "nvic_table.h"
#include "spimss_regs.h"
#include "mxc_device.h"

// Enable this macro to enable auto dma channel selection
// and transaction handling.
#define SPIMSS_AUTO_DMA_HANDLING 1

#define SPIMSS MXC_SPIMSS
#define SPI_SPEED 10000000
#define NUMBER_OF_UNIT_SIZE 512
#define TRANSFER_UNIT_SIZE 2
/*
* Transfer unit size is defined according to the bit
* count.(BitCount/8) is defined as transfer unit size.
* tx_data and rx_data is define as uint16_t. So, unit
* size is 16/8 = 2.
*/
#define DATA_VALUE 0x0102
#define TEST_COUNT 100 // Transaction count.

/**
 * Transfer unit size is 2. So, we define tx_data and rx_data as uint16_t.
*/
uint16_t tx_data[NUMBER_OF_UNIT_SIZE] = { 0 };
uint16_t rx_data[NUMBER_OF_UNIT_SIZE] = { 0 };
mxc_spimss_req_t spimss_dma_req;

static uint8_t dma_tx_completed = 0;
static uint8_t dma_rx_completed = 0;

#if !SPIMSS_AUTO_DMA_HANDLING
int txChannel = -1;
int rxChannel = -1;
#endif

void spimss_callback(mxc_spimss_req_t *req, int error_code)
{
    if (NUMBER_OF_UNIT_SIZE * TRANSFER_UNIT_SIZE == req->rx_num) {
        dma_rx_completed = 1;
#if !SPIMSS_AUTO_DMA_HANDLING
        MXC_DMA_DisableInt(rxChannel);
        MXC_DMA_SetChannelInterruptEn(rxChannel, false, false);
        MXC_DMA_ReleaseChannel(rxChannel);
        MXC_SPIMSS_SetRXDMAChannel(SPIMSS, -1);
        SPIMSS->dma &= ~MXC_F_SPIMSS_DMA_RX_DMA_EN; // Disable SPIMSS RX DMA requests
#endif
    } else if (NUMBER_OF_UNIT_SIZE * TRANSFER_UNIT_SIZE == req->tx_num) {
        dma_tx_completed = 1;
#if !SPIMSS_AUTO_DMA_HANDLING
        MXC_DMA_DisableInt(txChannel);
        MXC_DMA_SetChannelInterruptEn(txChannel, false, false); // Disable ctz interrupt.
        MXC_DMA_ReleaseChannel(txChannel);
        MXC_SPIMSS_SetTXDMAChannel(SPIMSS, -1);
        MXC_SPIMSS->dma &= ~MXC_F_SPIMSS_DMA_TX_DMA_EN; // Disable SPIMSS TX DMA requests

        if (req->deass) {
            MXC_SPIMSS->mode |= MXC_F_SPIMSS_MODE_SSV; // Set Slave Select to HIGH.
        }
        MXC_SPIMSS->ctrl &= ~MXC_F_SPIMSS_CTRL_ENABLE; // Disable SPIMSS transaction.
#endif
    }
}

int main()
{
    int i = 0;
    int ret_val = 0;

    printf(
        "\n\n************** SPIMSS-DMA Master Demo **************** \nThis example configures the SPIMSS to send data between the MISO (P0.10) and \nMOSI (P0.11) pins over dma channel 0 and dma channel 1.  Connect these two pins\ntogether. This demo shows 1024 byte data transfer for 100 times using dma.\n During this demo you may see junk data printed to the serial port because the\nconsole UART shares the same pins as the SPIMSS.One DMA channel is used as tx from\nmemory to SPIMSS TX FIFO and another dma channel is used for reading data from\nSPIMSS RX FIFO to memory.\n\n");

    // Preparation of the transmission data and receive data buffer.
    for (i = 0; i < NUMBER_OF_UNIT_SIZE; i++) {
        tx_data[i] = DATA_VALUE + i;
        rx_data[i] = 0;
    }

    // Initialize SPIMSS with MAP_A.
    // For MAP_A P0.10:MISO, P0.11:MOSI, P0.12:SCK, P0.13:SSEL
    ret_val = MXC_SPIMSS_Init(SPIMSS, 0, SPI_SPEED, MAP_A);
    if (E_NO_ERROR != ret_val) {
        return ret_val;
    }

    // Prepare the SPIMSS data transaction request.
    spimss_dma_req.tx_data = (void *)tx_data;
    spimss_dma_req.rx_data = (void *)rx_data;
    spimss_dma_req.len = NUMBER_OF_UNIT_SIZE;
    spimss_dma_req.bits = 16; // This must be >= 8
    spimss_dma_req.deass = 1;
    spimss_dma_req.tx_num = 0;
    spimss_dma_req.rx_num = 0;
    spimss_dma_req.callback = spimss_callback;

    for (i = 0; i < TEST_COUNT; i++) {
// Setting auto dma handlers enable acquiring the tx and rx channels
// automatically in the background.
#if SPIMSS_AUTO_DMA_HANDLING
        MXC_SPIMSS_SetAutoDMAHandlers((SPIMSS), true);
#else
        MXC_DMA_Init();
        txChannel = MXC_DMA_AcquireChannel();
        rxChannel = MXC_DMA_AcquireChannel();
        if (0 > txChannel || 0 > rxChannel) {
            break;
        }
        MXC_SPIMSS_SetTXDMAChannel(SPIMSS, txChannel);
        MXC_SPIMSS_SetRXDMAChannel(SPIMSS, rxChannel);
#endif

        // Calling SPIM transaction function for communication over DMA.
        ret_val = MXC_SPIMSS_MasterTransDMA(SPIMSS, &spimss_dma_req);
        if (E_NO_ERROR != ret_val) {
            printf("Test STOPPED! Err Code -> %d\r\n", ret_val);
            return ret_val;
        }

        // Wait until we complete the transaction.
        while (dma_rx_completed != 1 || dma_tx_completed != 1) {}
        dma_rx_completed = 0;
        dma_tx_completed = 0;
        spimss_dma_req.rx_num = 0; // Setting read byte count to 0.
        spimss_dma_req.tx_num = 0; // Setting sent byte count to 0.

        // Wait for 10 ms to see the transaction on analyser clearly.
        MXC_Delay(MXC_DELAY_MSEC(10));
        if (0 ==
            memcmp((void *)rx_data, (void *)tx_data, TRANSFER_UNIT_SIZE * NUMBER_OF_UNIT_SIZE)) {
            printf("Test %d -> Successful\n\r", i);
        } else {
            printf("Test %d -> Failed!\n", i);
        }
    }

    MXC_SPIMSS_Shutdown(SPIMSS);

    while (1) {}

    return 0;
}
