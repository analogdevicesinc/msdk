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
 * @brief   UART with DMA TX test code on the MAX32657 EVK.
 * @details This example use DMA with UART of MAX32657 EVK to send a long transfer (512 bytes) to test the DMA RX of MAX32657 (DUT).
 *          - Connect the GND of 2 MAX32657 EVKs.
 *          - Connect the UART TX of MAX32657 EVK (UART_DMA_TX_32657) to the UART RX pin of MAX32657 EVK (UART_DMA_32657).
 *          - Power on and Press the button RESET to send 512 bytes via DMA from UART_DMA_TX_32657 EVK to UART_DMA_32657 EVK.
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
#define TX_DMA_LENGTH 512

/***** Globals *****/
static mxc_uart_req_t write_req;
static uint8_t TxData[BUFF_SIZE];

volatile uint8_t dma_busy;

/***** Functions *****/
void writeCallback(mxc_uart_req_t *req, int error)
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

    while (1) {}
}
