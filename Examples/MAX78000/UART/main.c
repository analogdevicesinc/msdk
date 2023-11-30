/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc., All Rights Reserved.
 * (now owned by Analog Devices, Inc.)
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
 ******************************************************************************
 *
 * Copyright 2023 Analog Devices, Inc.
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
 * @brief   UART!
 * @details This example demonstrates the UART Loopback Test.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "led.h"
#include "board.h"
#include "mxc_delay.h"
#include "uart.h"
#include "dma.h"
#include "nvic_table.h"

/***** Definitions *****/
#define DMA

#define UART_BAUD 115200
#define BUFF_SIZE 1024

/***** Globals *****/
volatile int READ_FLAG;

#if defined(BOARD_EVKIT_V1)
#define READING_UART MXC_UART1
#define WRITING_UART MXC_UART2
#elif defined(BOARD_FTHR_REVA)
#define READING_UART MXC_UART2
#define WRITING_UART MXC_UART3
#else
#warning "This example has been written for the MAX78000 Ev Kit or FTHR board."
#endif

/***** Functions *****/
#ifndef DMA
void Reading_UART_Handler(void)
{
    MXC_UART_AsyncHandler(READING_UART);
}
#endif

void readCallback(mxc_uart_req_t *req, int error)
{
    READ_FLAG = error;
}
/******************************************************************************/
int main(void)
{
    int error, i, fail = 0;
    uint8_t TxData[BUFF_SIZE];
    uint8_t RxData[BUFF_SIZE];

    printf("\n\n**************** UART Example ******************\n");
    printf("This example sends data from one UART to another.\n");
    printf("\nConnect the TX pin of UART%d to the RX pin of UART%d for this example.\n",
           WRITING_UART, READING_UART);

    printf("\n-->UART Baud \t: %d Hz\n", UART_BAUD);
    printf("\n-->Test Length \t: %d bytes\n", BUFF_SIZE);

    // Initialize the data buffers
    for (i = 0; i < BUFF_SIZE; i++) {
        TxData[i] = i;
    }

    memset(RxData, 0x0, BUFF_SIZE);

#ifndef DMA
    NVIC_ClearPendingIRQ(MXC_UART_GET_IRQ(READING_UART_IDX));
    NVIC_DisableIRQ(MXC_UART_GET_IRQ(READING_UART_IDX));
    MXC_NVIC_SetVector(MXC_UART_GET_IRQ(READING_UART_IDX), Reading_UART_Handler);
    NVIC_EnableIRQ(MXC_UART_GET_IRQ(READING_UART_IDX));
#endif

    // Initialize the UART
    if ((error = MXC_UART_Init(READING_UART, UART_BAUD, MXC_UART_APB_CLK)) != E_NO_ERROR) {
        printf("-->Error initializing UART: %d\n", error);
        printf("-->Example Failed\n");
        return error;
    }

    if ((error = MXC_UART_Init(WRITING_UART, UART_BAUD, MXC_UART_APB_CLK)) != E_NO_ERROR) {
        printf("-->Error initializing UART: %d\n", error);
        printf("-->Example Failed\n");
        return error;
    }

    printf("-->UART Initialized\n\n");

#ifdef DMA
    // Automatically set up DMA handlers/ISRs
    MXC_UART_SetAutoDMAHandlers(READING_UART, true);
    MXC_UART_SetAutoDMAHandlers(WRITING_UART, true);
#endif

    mxc_uart_req_t read_req;
    read_req.uart = READING_UART;
    read_req.rxData = RxData;
    read_req.rxLen = BUFF_SIZE;
    read_req.txLen = 0;
    read_req.callback = readCallback;

    mxc_uart_req_t write_req;
    write_req.uart = WRITING_UART;
    write_req.txData = TxData;
    write_req.txLen = BUFF_SIZE;
    write_req.rxLen = 0;
    write_req.callback = NULL;

    READ_FLAG = 1;
    MXC_UART_ClearRXFIFO(READING_UART); // Clear any previously pending data
#ifdef DMA
    error = MXC_UART_TransactionDMA(&read_req);
#else
    error = MXC_UART_TransactionAsync(&read_req);
#endif

    if (error != E_NO_ERROR) {
        printf("-->Error starting read: %d\n", error);
        printf("-->Example Failed\n");
        return error;
    }

    error = MXC_UART_Transaction(&write_req);

    if (error != E_NO_ERROR) {
        printf("-->Error starting sync write: %d\n", error);
        printf("-->Example Failed\n");
        return error;
    }

    // Wait for read transaction to complete
    while (READ_FLAG) {}

    printf("-->Transaction complete\n\n");

    if (READ_FLAG != E_NO_ERROR) {
        printf("-->Error from UART read callback; %d\n", READ_FLAG);
        fail++;
    }

    if ((error = memcmp(RxData, TxData, BUFF_SIZE)) != 0) {
        printf("-->Error verifying Data: %d\n", error);
        fail++;
    } else {
        printf("-->Data verified\n");
    }

    if (fail != 0) {
        printf("\n-->Example Failed\n");
        return E_FAIL;
    }

    LED_On(LED1); // indicates SUCCESS
    printf("\n-->Example Succeeded\n");
    return E_NO_ERROR;
}
