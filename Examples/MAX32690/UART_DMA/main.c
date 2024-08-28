/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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
 * @brief   Main for UART with DMA example.
 * @details This example demonstrates how to use DMA with UART for data transfer.
 *          The example sets up DMA channels to handle UART transmissions and receptions,
 *          looping back the TX to the RX on UART0 with DMA. A jumper must be
 *          connected between P2.11 (UART 0 RX) -> P2.12 (UART 0 TX).
 *          The example uses either automatic or manual DMA handler configuration
 *          depending on the AUTOHANDLERS macro definition.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "pb.h"
#include "board.h"
#include "mxc_delay.h"
#include "uart.h"
#include "dma.h"
#include "nvic_table.h"
#include "led.h"

/***** Definitions *****/
#define AUTOHANDLERS

#define UART_BAUD 115200
#define BUFF_SIZE 512

#define UART MXC_UART0

/***** Globals *****/
volatile int READ_FLAG;
volatile int buttonPressed;
static mxc_uart_req_t loop_req;

/***** Functions *****/
#ifndef AUTOHANDLERS
void DMA_RX_Handler(void)
{
    MXC_DMA_Handler();
}

void DMA_TX_Handler(void)
{
    MXC_DMA_Handler();
}
#endif

void readCallback(mxc_uart_req_t *req, int error)
{
    READ_FLAG = error;
}

void buttonHandler(void)
{
    buttonPressed = 1;
}

#ifdef AUTOHANDLERS
int exampleDMAAutoHandlers(void)
{
    int error = 0;

    // Auto DMA handlers will automatically initialize DMA, acquire & assign channels,
    // and guarantee that each transaction's callback function is executed when
    // the transaction is complete.
    MXC_UART_SetAutoDMAHandlers(UART, true);

    // "READ_FLAG" is set in the read transaction's callback.  It will be set to 0 when
    // the read request completes successfully.  We use it to wait for the DMA transaction
    // to complete, since the DMA APIs are asynchronous (non-blocking)
    READ_FLAG = 1;

    error = MXC_UART_TransactionDMA(&loop_req);
    if (error) {
        printf("-->Error starting DMA read: %d\n", error);
        printf("-->Example Failed\n");
        return error;
    }

    while (READ_FLAG) {}

    printf("-->Transaction completed\n");
    return READ_FLAG;
}
#else
int exampleDMAManualHandlers(void)
{
    int error = 0;

    // Manally initialize DMA
    MXC_DMA_Init();

    // Manually acquire a channel for the read request and assign it to the drivers.
    int rx_channel = MXC_DMA_AcquireChannel();
    if (rx_channel >= 0) {
        printf("Acquired DMA channel %i for RX transaction\n", rx_channel);
    } else {
        printf("Failed to acquire RX DMA channel with error %i\n", rx_channel);
        return rx_channel;
    }
    MXC_UART_SetRXDMAChannel(UART, rx_channel);

    // Additionally, assign the NVIC IRQ to a function that calls "MXC_DMA_Handler()".
    // This is required for any assigned callbacks to work.
    NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(rx_channel));
    MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(rx_channel), DMA_RX_Handler);

    // Do the same for the write request.
    int tx_channel = MXC_DMA_AcquireChannel();
    if (tx_channel >= 0) {
        printf("Acquired DMA channel %i for RX transaction\n", tx_channel);
    } else {
        printf("Failed to acquire RX DMA channel with error %i\n", tx_channel);
        return tx_channel;
    }
    MXC_UART_SetTXDMAChannel(UART, tx_channel);
    NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(tx_channel));
    MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(tx_channel), DMA_TX_Handler);

    // Initialize flags.  We will use these to monitor when the read/write requests
    // have completed, since the DMA APIs are asynchronous.
    READ_FLAG = 1;

    error = MXC_UART_TransactionDMA(&loop_req);
    if (error) {
        printf("-->Error starting DMA read: %d\n", error);
        printf("-->Example Failed\n");
        return error;
    }

    while (READ_FLAG) {}

    printf("-->Transaction completed\n");
    return READ_FLAG;
}
#endif // AUTOHANDLERS

/******************************************************************************/
int main(void)
{
    int error, i, fail = 0;
    uint8_t TxData[BUFF_SIZE];
    uint8_t RxData[BUFF_SIZE];

    printf("\n\n**************** UART Example ******************\n");
    printf("This example sends data from one UART to another\n");
    printf("\nConnect RX(P2.11) of UART1 and TX(P2.12) of UART0.\n\n");
    printf("To indicate a successful UART transfer, LED1 will illuminate.\n");
    printf("\nPush SW2 to continue\n");

    buttonPressed = 0;
    PB_RegisterCallback(0, buttonHandler);
    while (!buttonPressed) {}

    printf("\nUART Baud \t: %d Hz\n", UART_BAUD);
    printf("Test Length \t: %d bytes\n\n", BUFF_SIZE);

    // Initialize the data buffers
    for (i = 0; i < BUFF_SIZE; i++) {
        TxData[i] = i;
    }
    memset(RxData, 0x0, BUFF_SIZE);

    // Initialize the UART
    error = MXC_UART_Init(UART, UART_BAUD, MXC_UART_APB_CLK);
    if (error < E_NO_ERROR) {
        printf("-->Error initializing UART: %d\n", error);
        printf("-->Example Failed\n");
        return error;
    }

    // Setup request structs describing the transactions.
    // Request structs are placed in the global scope so they
    // don't go out of context.  This can happen when a req struct
    // is declared inside a function and the function completes.
    // The memory would be freed, invalidating the UART driver's
    // pointers to it.
    loop_req.uart = UART;
    loop_req.rxData = RxData;
    loop_req.rxLen = BUFF_SIZE;
    loop_req.txData = TxData;
    loop_req.txLen = BUFF_SIZE;
    loop_req.callback = readCallback;

    printf("-->UART Initialized\n\n");

#ifdef AUTOHANDLERS
    error = exampleDMAAutoHandlers();
#else
    error = exampleDMAManualHandlers();
#endif

    if (READ_FLAG != E_NO_ERROR) {
        printf("-->Error with read callback; %d\n", READ_FLAG);
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
        LED_On(0); // indicates FAIL
        return E_FAIL;
    }

    LED_On(1); // indicates SUCCESS
    printf("\n-->Example Succeeded\n");
    return E_NO_ERROR;
}
