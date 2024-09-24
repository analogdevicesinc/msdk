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
 *          looping back the TX of one UART to the RX of another UART. A jumper must be
 *          connected between P0.20 (RX of UART1) and P0.1 (TX of UART2).
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

#define RX_UART MXC_UART1
#define TX_UART MXC_UART2

#define DMA MXC_DMA0

/***** Globals *****/
volatile int READ_FLAG;
volatile int WRITE_FLAG;
volatile int buttonPressed;
static mxc_uart_req_t read_req;
static mxc_uart_req_t write_req;

#ifndef BOARD_FTHR2
#warning "This example has been written for the MAX32665 FTHR2 board."
#endif

/***** Functions *****/
#ifndef AUTOHANDLERS
void DMA_RX_Handler(void)
{
    MXC_DMA_Handler(DMA);
}

void DMA_TX_Handler(void)
{
    MXC_DMA_Handler(DMA);
}
#endif

void readCallback(mxc_uart_req_t *req, int error)
{
    READ_FLAG = error;
}

void writeCallback(mxc_uart_req_t *req, int error)
{
    WRITE_FLAG = error;
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
    MXC_UART_SetAutoDMAHandlers(RX_UART, true);
    MXC_UART_SetAutoDMAHandlers(TX_UART, true);

    // "READ_FLAG" is set in the read transaction's callback.  It will be set to 0 when
    // the read request completes successfully.  We use it to wait for the DMA transaction
    // to complete, since the DMA APIs are asynchronous (non-blocking)
    READ_FLAG = 1;

    error = MXC_UART_TransactionDMA(&read_req, DMA);
    if (error) {
        printf("-->Error starting DMA read: %d\n", error);
        printf("-->Example Failed\n");
        return error;
    }

    error = MXC_UART_TransactionDMA(&write_req, DMA);
    if (error) {
        printf("-->Error starting DMA write: %d\n", error);
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
    MXC_DMA_Init(DMA);

    // Manually acquire a channel for the read request and assign it to the drivers.
    int rx_channel = MXC_DMA_AcquireChannel(DMA);
    if (rx_channel >= 0) {
        printf("Acquired DMA channel %i for RX transaction\n", rx_channel);
    } else {
        printf("Failed to acquire RX DMA channel with error %i\n", rx_channel);
        return rx_channel;
    }
    MXC_UART_SetRXDMAChannel(RX_UART, rx_channel);

    // Additionally, assign the NVIC IRQ to a function that calls "MXC_DMA_Handler()".
    // This is required for any assigned callbacks to work.
    NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(rx_channel));
    MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(rx_channel), DMA_RX_Handler);

    // Do the same for the write request.
    int tx_channel = MXC_DMA_AcquireChannel(DMA);
    if (tx_channel >= 0) {
        printf("Acquired DMA channel %i for RX transaction\n", tx_channel);
    } else {
        printf("Failed to acquire RX DMA channel with error %i\n", tx_channel);
        return tx_channel;
    }
    MXC_UART_SetTXDMAChannel(TX_UART, tx_channel);
    NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(tx_channel));
    MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(tx_channel), DMA_TX_Handler);

    // Initialize flags.  We will use these to monitor when the read/write requests
    // have completed, since the DMA APIs are asynchronous.
    WRITE_FLAG = 1;
    READ_FLAG = 1;

    error = MXC_UART_TransactionDMA(&read_req, DMA);
    if (error) {
        printf("-->Error starting DMA read: %d\n", error);
        printf("-->Example Failed\n");
        return error;
    }

    error = MXC_UART_TransactionDMA(&write_req, DMA);
    if (error) {
        printf("-->Error starting DMA write: %d\n", error);
        printf("-->Example Failed\n");
        return error;
    }

    while (WRITE_FLAG) {}
    while (READ_FLAG) {}

    printf("-->Transaction completed\n");
    return WRITE_FLAG;
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
    printf("\nConnect P0.20 (RX of UART1) and P0.1 (TX of UART2).\n\n");
    printf("To indicate a successful UART transfer, LED1 will illuminate.\n");
    printf("\nPush SW2 to continue\n");

    buttonPressed = 0;
    PB_RegisterCallback(0, (pb_callback)buttonHandler);
    while (!buttonPressed) {}

    printf("\nUART Baud \t: %d Hz\n", UART_BAUD);
    printf("Test Length \t: %d bytes\n\n", BUFF_SIZE);

    // Initialize the data buffers
    for (i = 0; i < BUFF_SIZE; i++) {
        TxData[i] = i;
    }
    memset(RxData, 0x0, BUFF_SIZE);

    // Initialize the UART
    error = MXC_UART_Init(TX_UART, UART_BAUD, MAP_A);
    if (error < E_NO_ERROR) {
        printf("-->Error initializing UART: %d\n", error);
        printf("-->Example Failed\n");
        return error;
    }

    error = MXC_UART_Init(RX_UART, UART_BAUD, MAP_A);
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
    read_req.uart = RX_UART;
    read_req.rxData = RxData;
    read_req.rxLen = BUFF_SIZE;
    read_req.txLen = 0;
    read_req.callback = readCallback;

    write_req.uart = TX_UART;
    write_req.txData = TxData;
    write_req.txLen = BUFF_SIZE;
    write_req.rxLen = 0;
    write_req.callback = writeCallback;

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
