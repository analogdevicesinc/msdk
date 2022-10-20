/**
 * @file    main.c
 * @brief   SPI Master Demo
 * @details Shows Master loopback demo for QSPI0
 *          Read the printf() for instructions
 */

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

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc_pins.h"
#include "nvic_table.h"
#include "uart.h"
#include "spi.h"
#include "dma.h"

/***** Preprocessors *****/
#define MASTERSYNC // 1. MASTERSYNC
// 2. MASTERASYNC
// 3. MASTERDMA

/***** Definitions *****/
#define DATA_LEN 100 // Words
#define DATA_VALUE 0xA5A5 // This is for master mode only...
#define VALUE 0xFFFF
#define SPI_SPEED 100000 // Bit Rate

#define SPI MXC_SPI0
#define SPI_IRQ SPI0_IRQn

/***** Globals *****/
uint16_t rx_data[DATA_LEN];
uint16_t tx_data[DATA_LEN];
volatile int SPI_FLAG;
volatile int DMA0_FLAG = 0;
volatile int DMA1_FLAG = 0;

/***** Functions *****/
void SPI0_IRQHandler(void)
{
    MXC_SPI_AsyncHandler(SPI);
}

void DMA0_Handler(void)
{
    MXC_DMA_Handler();
    DMA0_FLAG = 1;
}

void DMA1_Handler(void)
{
    MXC_DMA_Handler();
    DMA1_FLAG = 1;
}

void SPI_Callback(mxc_spi_req_t *req, int error)
{
    SPI_FLAG = error;
}

int main(void)
{
    int i, j, retVal;
    uint16_t temp;
    mxc_spi_req_t req;

    printf("\n**************************** SPI MASTER TEST *************************\n");
    printf("This example configures the SPI to send data between the MISO (P0.4) and\n");
    printf("MOSI (P0.5) pins.  Connect these two pins together.  This demo shows SPI\n");
    printf("sending different bit sizes each run through. \n");

    printf("\nThis demo can be configured to show Asynchronous, Synchronous or DMA transactions "
           "for SPI0\n");

    for (i = 1; i < 17; i++) {
        if (i == 1) { // Sending out 2 to 16 bits
            continue;
        }

        memset(tx_data, 0x0, DATA_LEN * sizeof(uint16_t));

        for (j = 0; j < DATA_LEN; j++) {
            tx_data[j] = DATA_VALUE;
        }

        // Configure the peripheral
        if (MXC_SPI_Init(SPI, 1, 0, 1, 0, SPI_SPEED) != E_NO_ERROR) {
            printf("\nSPI INITIALIZATION ERROR\n");
            while (1) {}
        }

        memset(rx_data, 0x0, DATA_LEN * sizeof(uint16_t));

        //SPI Request
        req.spi = SPI;
        req.txData = (uint8_t *)tx_data;
        req.rxData = (uint8_t *)rx_data;
        req.txLen = DATA_LEN;
        req.rxLen = DATA_LEN;
        req.ssIdx = 0;
        req.ssDeassert = 1;
        req.txCnt = 0;
        req.rxCnt = 0;
        req.completeCB = (spi_complete_cb_t)SPI_Callback;
        SPI_FLAG = 1;

        retVal = MXC_SPI_SetDataSize(SPI, i);
        if (retVal != E_NO_ERROR) {
            printf("\nSPI SET DATASIZE ERROR: %d\n", retVal);
            return retVal;
        }

        retVal = MXC_SPI_SetWidth(SPI, SPI_WIDTH_STANDARD);
        if (retVal != E_NO_ERROR) {
            printf("\nSPI SET WIDTH ERROR: %d\n", retVal);
            return retVal;
        }

#ifdef MASTERSYNC
        MXC_SPI_MasterTransaction(&req);
#endif

#ifdef MASTERASYNC
        NVIC_EnableIRQ(SPI_IRQ);
        MXC_SPI_MasterTransactionAsync(&req);

        while (SPI_FLAG == 1) {}
#endif

#ifdef MASTERDMA
        MXC_DMA_ReleaseChannel(0);
        MXC_NVIC_SetVector(DMA0_IRQn, DMA0_Handler);
        NVIC_EnableIRQ(DMA0_IRQn);

        MXC_DMA_ReleaseChannel(1);
        MXC_NVIC_SetVector(DMA1_IRQn, DMA1_Handler);
        NVIC_EnableIRQ(DMA1_IRQn);

        DMA0_FLAG = 0;
        DMA1_FLAG = 0;

        MXC_SPI_MasterTransactionDMA(&req);

        while (DMA1_FLAG == 0 || DMA0_FLAG == 0) {}
        if (MXC_DMA->int_fl & MXC_F_DMA_INT_FL_IPEND) {
            while (MXC_DMA->int_fl & MXC_F_DMA_INT_FL_IPEND) {}
        }
#endif

        uint8_t bits = MXC_SPI_GetDataSize(SPI);

        for (j = 0; j < DATA_LEN; j++) {
            if (bits <= 8) {
                if (j < (DATA_LEN / 2)) {
                    temp = VALUE >> (16 - bits);
                    temp = (temp << 8) | temp;
                    temp &= DATA_VALUE;
                    tx_data[j] = temp;
                } else if (j == (DATA_LEN / 2) && DATA_LEN % 2 == 1) {
                    temp = VALUE >> (16 - bits);
                    temp &= DATA_VALUE;
                    tx_data[j] = temp;
                } else {
                    tx_data[j] = 0x0000;
                }
            } else {
                temp = VALUE >> (16 - bits);
                temp &= DATA_VALUE;
                tx_data[j] = temp;
            }
        }

        // Compare Sent data vs Received data
        // Printf needs the Uart turned on since they share the same pins
        if (memcmp(rx_data, tx_data, sizeof(tx_data)) != 0) {
            printf("\n-->%2d Bits Transaction Failed\n", i);
            return E_BAD_STATE;
        } else {
            printf("\n-->%2d Bits Transaction Successful\n", i);
        }

        retVal = MXC_SPI_Shutdown(SPI);
        if (retVal != E_NO_ERROR) {
            printf("\n-->SPI SHUTDOWN ERROR: %d\n", retVal);
            return retVal;
        }
    }

    return E_NO_ERROR;
}
