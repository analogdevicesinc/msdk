/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
 *
 * This software is protected by copyright laws of the United States and
 * of foreign countries. This material may also be protected by patent laws
 * and technology transfer regulations of the United States and of foreign
 * countries. This software is furnished under a license agreement and/or a
 * nondisclosure agreement and may only be used or reproduced in accordance
 * with the terms of those agreements. Dissemination of this information to
 * any party or parties not specified in the license agreement and/or
 * nondisclosure agreement is expressly prohibited.
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
 * @details This example sends data from TX on UART1 to the RX on UART3. For this example
 *          you must connect a jumper across P1.20 to P1.17 and P1.21 to P1.16. UART_BAUD
 *          and the BUFF_SIZE can be changed in this example.
 */

/***** Includes *****/
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <MAX32xxx.h>

/***** Definitions *****/
// #define DMA

#define UART_BAUD 115200
#define BUFF_SIZE 256

/***** Globals *****/
volatile int READ_FLAG;
volatile int DMA_FLAG;

/***** Functions *****/

/******************************************************************************/
void readCallback(mxc_uart_req_t* req, int error)
{
    READ_FLAG = error;
}

/******************************************************************************/
void UART3_Handler(void)
{
    MXC_UART_AsyncHandler(MXC_UART3);
}

#ifdef DMA
void DMA_Handler(void)
{
    MXC_DMA_Handler();
    DMA_FLAG = 0;
}
#else
void UART1_Handler(void)
{
    MXC_UART_AsyncHandler(MXC_UART1);
}
#endif
/******************************************************************************/
int main(void)
{
    int error, i, fail = 0;
    uint8_t TxData[BUFF_SIZE];
    uint8_t RxData[BUFF_SIZE];

    printf("\n\n**************** UART Example ******************\n");
    printf("This example sends data from one UART to another\n");
    printf("\nThe LED (P2.17) is used to indicate the success of the test.\nLED ON -> Success\n");
    printf("\n\nConnect UART1 to UART3 for this example.\n");
    printf("P1.12 -> P1.21 and P1.13 -> P1.20\n\n");

    printf("\n-->UART Baud \t: %d Hz\n", UART_BAUD);
    printf("\n-->Test Length \t: %d bytes\n\n", BUFF_SIZE);

    // Initialize the data buffers
    for (i = 0; i < BUFF_SIZE; i++) {
        TxData[i] = i;
    }

    memset(RxData, 0x0, BUFF_SIZE);

    // Setup the interrupt
    NVIC_ClearPendingIRQ(UART3_IRQn);
    NVIC_DisableIRQ(UART3_IRQn);
    MXC_NVIC_SetVector(UART3_IRQn, UART3_Handler);
    NVIC_EnableIRQ(UART3_IRQn);

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
    if ((error = MXC_UART_Init(MXC_UART1, UART_BAUD)) != E_NO_ERROR) {
        printf("-->Error initializing UART: %d\n", error);
        printf("-->Example Failed\n");
        while (1) { }
    }

    if ((error = MXC_UART_Init(MXC_UART3, UART_BAUD)) != E_NO_ERROR) {
        printf("-->Error initializing UART: %d\n", error);
        printf("-->Example Failed\n");
        while (1) { }
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
    write_req.uart = MXC_UART3;
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
        LED_On(0);
        printf("-->EXAMPLE SUCCEEDED\n");
    } else {
        printf("-->EXAMPLE FAILED\n");
    }

    while (1) { }
}
