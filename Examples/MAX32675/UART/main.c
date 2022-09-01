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
 * @brief   UART!
 * @details This example demonstrates the UART Loopback Test.
 */

/***** Includes *****/
#include "board.h"
#include "dma.h"
#include "led.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "nvic_table.h"
#include "pb.h"
#include "uart.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

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
volatile int WRITE_FLAG;
volatile int DMA_FLAG;

/***** Functions *****/
#ifdef DMA
void DMA_Handler(void)
{
    MXC_DMA_Handler();
    DMA_FLAG = 0;
}
#else
void UART0_Handler(void)
{
    MXC_UART_AsyncHandler(READING_UART);
}
#endif

void readCallback(mxc_uart_req_t* req, int error)
{
    READ_FLAG = error;
}

void writeCallback(mxc_uart_req_t* req, int error)
{
    WRITE_FLAG = error;
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
    printf("P0.8 -> P0.15 and P0.9 -> P0.14\n\n");

    printf("\n-->UART Baud \t: %d Hz\n", UART_BAUD);
    printf("\n-->Test Length \t: %d bytes\n", BUFF_SIZE);
    while (MXC_UART_GetActive(READING_UART)) { }

    // Initialize the data buffers
    for (i = 0; i < BUFF_SIZE; i++) { TxData[i] = i; }

    memset(RxData, 0x0, BUFF_SIZE);

#ifdef DMA
    MXC_DMA_ReleaseChannel(0);
    MXC_NVIC_SetVector(DMA0_IRQn, DMA_Handler);
    NVIC_EnableIRQ(DMA0_IRQn);
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
        while (1) { }
    }

    if ((error = MXC_UART_Init(WRITING_UART, UART_BAUD, MXC_UART_APB_CLK)) != E_NO_ERROR) {
        printf("-->Error initializing UART: %d\n", error);
        printf("-->Example Failed\n");
        while (1) { }
    }

    // Max baud rate for most serial ports is 115200
    if (UART_BAUD <= CONSOLE_BAUD) {
        printf("-->UARTs Initialized\n\n");
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
    DMA_FLAG = 1;

#ifdef DMA
    error = MXC_UART_TransactionDMA(&read_req);
#else
    error = MXC_UART_TransactionAsync(&read_req);
#endif

    if (error != E_NO_ERROR) {
        printf("-->Error starting async read: %d\n", error);
        printf("-->Example Failed\n");
        while (1) { }
    }

    error = MXC_UART_Transaction(&write_req);

    if (error != E_NO_ERROR) {
        printf("-->Error starting sync write: %d\n", error);
        printf("-->Example Failed\n");
        while (1) { }
    }

#ifdef DMA

    while (DMA_FLAG) { }

#else

    while (READ_FLAG) { }

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
        LED_On(1);
        printf("-->EXAMPLE SUCCEEDED\n");
    } else {
        LED_On(0);
        printf("-->EXAMPLE FAILED\n");
    }

    return 0;
}
