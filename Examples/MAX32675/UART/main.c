/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2026 Analog Devices, Inc.
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
#include "pb.h"
#include "board.h"
#include "mxc_delay.h"
#include "uart.h"
#include "dma.h"
#include "nvic_table.h"

/***** Definitions *****/
// #define DMA

#define UART_BAUD CONSOLE_BAUD
#define BUFF_SIZE 256
#define READING_UART MXC_UART0
#define READING_UART_IDX MXC_UART_GET_IDX(READING_UART)
#define WRITING_UART MXC_UART2
#define WRITING_UART_IDX MXC_UART_GET_IDX(WRITING_UART)

/***** Globals *****/
volatile int READ_FLAG;

/***** Functions *****/
#ifndef DMA
void UART0_Handler(void)
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
    printf("This example sends data from one UART to another\n");
    printf("\nThe green LED1 will illuminate for successful transaction.\n");
    printf("The red LED0 will illuminate if transaction failed.\n");
    printf("\nUnplug the Jumper at (JP5 - RX0_EN) above the Port 0 headers.\n");
    printf("\n\nConnect UART0 to UART2 for this example.\n");
    printf("P0.8(UART0_RX) -> P1.9(UART2_TX)\n\n");

    printf("\n-->UART Baud \t: %d Hz\n", UART_BAUD);
    printf("\n-->Test Length \t: %d bytes\n", BUFF_SIZE);
    while (MXC_UART_GetActive(READING_UART)) {}

    // Initialize the data buffers
    for (i = 0; i < BUFF_SIZE; i++) {
        TxData[i] = i;
    }

    memset(RxData, 0x0, BUFF_SIZE);

#ifdef DMA
    MXC_DMA_ReleaseChannel(0);
#else
    NVIC_ClearPendingIRQ(MXC_UART_GET_IRQ(READING_UART_IDX));
    NVIC_DisableIRQ(MXC_UART_GET_IRQ(READING_UART_IDX));
    MXC_NVIC_SetVector(MXC_UART_GET_IRQ(READING_UART_IDX), UART0_Handler);
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

    // Max baud rate for most serial ports is 115200
    if (UART_BAUD <= CONSOLE_BAUD) {
        printf("\n-->UARTs Initialized\n\n");
    }

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
    MXC_UART_SetAutoDMAHandlers(READING_UART, true);
    error = MXC_UART_TransactionDMA(&read_req);
#else
    error = MXC_UART_TransactionAsync(&read_req);
#endif

    if (error != E_NO_ERROR) {
        printf("-->Error starting async read: %d\n", error);
        printf("-->Example Failed\n");
        return error;
    }

    error = MXC_UART_Transaction(&write_req);

    if (error != E_NO_ERROR) {
        printf("-->Error starting sync write: %d\n", error);
        printf("-->Example Failed\n");
        return error;
    }

    while (READ_FLAG) {}

    if (READ_FLAG != E_NO_ERROR) {
        printf("-->Error with UART Read callback; %d\n", READ_FLAG);
        fail++;
    }

    if ((error = memcmp(RxData, TxData, BUFF_SIZE)) != 0) {
        printf("-->Error verifying Data: %d\n", error);
        fail++;
    } else {
        printf("-->Data verified\n");
    }

    if (fail != 0) {
        LED_On(0); // indicates FAIL
        printf("\n-->Example Failed\n");
        return E_FAIL;
    }

    LED_On(1); // indicates SUCCESS
    printf("\n-->Example Succeeded\n");
    return E_NO_ERROR;
}
