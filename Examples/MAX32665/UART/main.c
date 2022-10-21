/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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

/**
 * @file    main.c
 * @brief   Main for UART example.
 * @details 
 * On FTH2 board
 *          This example sends data from one UART to another
 *          You will need to connect RX1 (P0_20) of UART1 and TX2 (P0_1) of UART2
 * On other boards
 *          This example loops back the TX to the RX on UART1. For this example
 *          you must connect a jumper across P0.20 to P0.21. UART_BAUD and the BUFF_SIZE
 *          can be changed in this example.
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
// #define DMA

#if defined(BOARD_FTHR2)
#define MXC_UART_SENDER MXC_UART2
#else
#define MXC_UART_SENDER MXC_UART0
#endif

#define UART_BAUD 115200
#define BUFF_SIZE 512

/***** Globals *****/
volatile int READ_FLAG;
volatile int DMA_FLAG;
volatile int buttonPressed;

/***** Functions *****/
#ifdef DMA
void DMA_Handler(void)
{
    MXC_DMA_Handler(MXC_DMA0);
    DMA_FLAG = 0;
}
#else
void UART1_Handler(void)
{
    MXC_UART_AsyncHandler(MXC_UART1);
}
#endif

/******************************************************************************/
void readCallback(mxc_uart_req_t *req, int error)
{
    READ_FLAG = error;
}

void buttonHandler()
{
    buttonPressed = 1;
}

/******************************************************************************/
int main(void)
{
    int error, i, fail = 0;
    uint8_t TxData[BUFF_SIZE];
    uint8_t RxData[BUFF_SIZE];

    printf("\n\n**************** UART Example ******************\n");
#if defined(BOARD_FTHR2)
    printf("This example sends data from one UART to another\n");
    printf("\nYou will need to connect RX1 (P0_20) of UART1 and TX2 (P0_1) of UART2\n");
    printf("Because UART1 is used to receive data,\n");
    printf("not all printf statements will be printed to the terminal.\n\n");
    printf("To indicate a successful UART transfer, the green LED (P0_31) will illuminate.\n");
    printf("The red LED (P0_29) will illuminate when the transaction fails.\n\n");
    printf("Push SW2 to continue\n\n");
#else
    printf("This example sends data from one UART to another\n");
    printf("\nYou will need to connect RX1 of UART1 and TX0 of UART0\n");
    printf("Because UART1 is used to receive data,\n");
    printf("not all printf statements will be printed to the terminal.\n\n");
    printf("To indicate a successful UART transfer, the green LED (P1_15) will illuminate.\n");
    printf("The red LED (P1_14) will illuminate when the transaction fails.\n\n");
    printf("Push SW2 to continue\n\n");
#endif

    buttonPressed = 0;
    PB_RegisterCallback(0, (pb_callback)buttonHandler);
    while (!buttonPressed) {}

    printf("UART Baud \t: %d Hz\n", UART_BAUD);
    printf("Test Length \t: %d bytes\n\n", BUFF_SIZE);

    // Initialize the data buffers
    for (i = 0; i < BUFF_SIZE; i++) {
        TxData[i] = i;
    }
    memset(RxData, 0x0, BUFF_SIZE);

#ifdef DMA
    MXC_DMA_ReleaseChannel(0);
    MXC_NVIC_SetVector(DMA0_IRQn, DMA_Handler);
    NVIC_EnableIRQ(DMA0_IRQn);
#else
    NVIC_ClearPendingIRQ(UART1_IRQn);
    NVIC_DisableIRQ(UART1_IRQn);
    MXC_NVIC_SetVector(UART1_IRQn, UART1_Handler);
    NVIC_EnableIRQ(UART1_IRQn);
#endif

    // Initialize the UART
    error = MXC_UART_Init(MXC_UART_SENDER, UART_BAUD, MAP_A);
    if (error < E_NO_ERROR) {
        printf("-->Error initializing UART: %d\n", error);
        printf("-->Example Failed\n");
        while (1) {}
    }

    error = MXC_UART_Init(MXC_UART1, UART_BAUD, MAP_A);
    if (error < E_NO_ERROR) {
        printf("-->Error initializing UART: %d\n", error);
        printf("-->Example Failed\n");
        while (1) {}
    }

    printf("-->UART Initialized\n\n");

    // Setup the asynchronous request
    mxc_uart_req_t read_req;
    read_req.uart = MXC_UART1;
    read_req.rxData = RxData;
    read_req.rxLen = BUFF_SIZE;
    read_req.txLen = 0;
    read_req.callback = readCallback;

    mxc_uart_req_t write_req;
    write_req.uart = MXC_UART_SENDER;
    write_req.txData = TxData;
    write_req.txLen = BUFF_SIZE;
    write_req.rxLen = 0;
    write_req.callback = NULL;

    READ_FLAG = 1;
    DMA_FLAG = 1;

#ifdef DMA
    error = MXC_UART_TransactionDMA(&read_req, MXC_DMA0);
#else
    error = MXC_UART_TransactionAsync(&read_req);
#endif

    if (error != E_NO_ERROR) {
        printf("-->Error starting async read: %d\n", error);
        printf("-->Example Failed\n");

        while (1) {}
    }

#ifdef DMA
    error = MXC_UART_TransactionDMA(&write_req, MXC_DMA0);
#else
    error = MXC_UART_Transaction(&write_req);
#endif

    if (error != E_NO_ERROR) {
        printf("-->Error starting sync write: %d\n", error);
        printf("-->Example Failed\n");

        while (1) {}
    }

#ifdef DMA

    while (DMA_FLAG) {}

#else

    while (READ_FLAG) {}

    if (READ_FLAG != E_NO_ERROR) {
        printf("-->Error with UART_ReadAsync callback; %d\n", READ_FLAG);
        fail++;
    }

#endif

    if ((error = memcmp(RxData, TxData, BUFF_SIZE)) != 0) {
        printf("-->Error verifying Data: %d\n", error);
        fail++;
    } else {
        printf("-->Data verified\n");
    }

    printf("\n");

    if (fail == 0) {
        printf("-->EXAMPLE SUCCEEDED\n");
        LED_On(1);
    } else {
        printf("-->EXAMPLE FAILED\n");
        LED_On(0);
    }

    while (1) {}
}
