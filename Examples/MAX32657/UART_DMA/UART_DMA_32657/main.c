/******************************************************************************
 *
 * Copyright (C) 2026 Analog Devices, Inc.
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
 * @brief   UART for DMA TX and RX of MAX32657 EVK
 * @details This example demo how MAX32657 send and receive bytes via UART with DMA
 *          - Connect the GND of MAX32670 EVK to the GND of MAX32657 EVK.
 *          - Connect the P0.15 (UART 2 TX) of MAX32670 EVK to the UART RX pin of MAX32657 EVK.
 *          - Connect the a logic analyzer channel to UART TX pin of MAX32657 EVK.
 *          - Ensure MAX32670 test code and MAX32657 (this file) setup UART in same baud-rate.
 *          - Run the logic analyzer and reset the MAX32657 to check the UART DMA TX waveform.
 *          - After MAX32657 finish to send UART TX, press the button P0_21 on MAX32670 EVK
 *          - In debug mode of MAX32657 check test_result == 0 and rx_index == 512
 *          The baud-rate can be change depending on the UART_BAUD macro definition from (115200 to 921600).
 *          The MAX32657 can success to receive with baud-rate from 115200 to 460800 (NG with 921600)
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "board.h"
#include "mxc_delay.h"
#include "uart.h"
#include "dma.h"

/***** Definitions *****/
#define UART_BAUD 115200 // OK for TX
//#define UART_BAUD 230400   // OK for TX
//#define UART_BAUD 460800   // Some delay (47,5us) between 2 TX transactions
//#define UART_BAUD 921600   // Some delay (79us) between 2 TX transactions
#define BUFF_SIZE 1024
#define TX_DMA_LENGTH 10
#define RX_TOTAL_LENGTH 512
#define RX_DMA_LENGTH 128

/***** Globals *****/
static mxc_uart_req_t read_req;
static mxc_uart_req_t write_req;
static uint8_t TxData[BUFF_SIZE];
static uint8_t RxData[BUFF_SIZE];
static int rx_index = 0;
volatile uint8_t test_result;

volatile uint8_t dma_busy;

/***** Functions *****/
void writeCallback(mxc_uart_req_t *req, int error)
{
    dma_busy = 0;
}

void readCallback(mxc_uart_req_t *req, int error)
{
    dma_busy = 0;
}

void exampleDMAAuto_TX(void)
{
    write_req.uart = MXC_UART;
    write_req.txData = TxData;
    write_req.txLen = TX_DMA_LENGTH;
    write_req.rxLen = 0;
    write_req.callback = writeCallback;

    MXC_UART_SetAutoDMAHandlers(MXC_UART, true);

    dma_busy = 1;
    MXC_UART_TransactionDMA(&write_req, MXC_DMA1);
    while (dma_busy == 1) {}

    dma_busy = 1;
    MXC_UART_TransactionDMA(&write_req, MXC_DMA1);
    while (dma_busy == 1) {}

    dma_busy = 1;
    MXC_UART_TransactionDMA(&write_req, MXC_DMA1);
    while (dma_busy == 1) {}

    dma_busy = 1;
    MXC_UART_TransactionDMA(&write_req, MXC_DMA1);
    while (dma_busy == 1) {}

    return;
}

void exampleDMAAuto_RX(void)
{
    int i;

    for (i = 0; i < BUFF_SIZE; i++) {
        RxData[i] = 0;
    }

    read_req.uart = MXC_UART;
    read_req.rxLen = 128;
    read_req.txLen = 0;
    read_req.callback = readCallback;

    while (rx_index < RX_TOTAL_LENGTH) {
        read_req.rxData = &(RxData[rx_index]);

        if ((RX_TOTAL_LENGTH - rx_index) > RX_DMA_LENGTH) {
            read_req.rxLen = RX_DMA_LENGTH;
        } else {
            read_req.rxLen = RX_TOTAL_LENGTH - rx_index;
        }

        dma_busy = 1;
        MXC_UART_TransactionDMA(&read_req, MXC_DMA1);
        while (dma_busy == 1) {}

        rx_index += read_req.rxLen;
    }

    return;
}

// *****************************************************************************
int main(void)
{
    int i;

    for (i = 0; i < BUFF_SIZE; i++) {
        TxData[i] = i % 256;
    }

    MXC_UART_Shutdown(MXC_UART);
    MXC_UART_Init(MXC_UART, UART_BAUD, MXC_UART_APB_CLK);

    exampleDMAAuto_TX();
    exampleDMAAuto_RX();

    test_result = 0;

    for (i = 0; i < RX_TOTAL_LENGTH; i++) {
        if (RxData[i] != (i % 256))
            test_result = 1;
    }

    while (1) {}
}
