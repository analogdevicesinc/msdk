/**
 * @file    main.c
 * @brief   SPI Master Demo
 * @details Shows Master loopback demo for QSPI0
 *          Read the printf() for instructions
 */

/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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
#include <stdbool.h>
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
#define MASTERSYNC 0
#define MASTERASYNC 0
#define MASTERDMA 1

#define MXC_SPI_V2

#if (!(MASTERSYNC || MASTERASYNC || MASTERDMA))
#error "You must set either MASTERSYNC or MASTERASYNC or MASTERDMA to 1."
#endif
#if ((MASTERSYNC && MASTERASYNC) || (MASTERASYNC && MASTERDMA) || (MASTERDMA && MASTERSYNC))
#error "You must select either MASTERSYNC or MASTERASYNC or MASTERDMA, not all 3."
#endif

/***** Definitions *****/
#define DATA_LEN 100 // Words
#define DATA_VALUE 0xA5AB // This is for master mode only...
#define VALUE 0xFFFF
#define SPI_SPEED 100000 // Bit Rate

#define SPI_INSTANCE_NUM 1

/***** Globals *****/
uint16_t rx_data[DATA_LEN];
uint16_t tx_data[DATA_LEN];
volatile int SPI_FLAG;

/***** Functions *****/
#if (SPI_INSTANCE_NUM == 0)
#define SPI MXC_SPI0
#define SPI_IRQ SPI0_IRQn
void SPI0_IRQHandler(void)
{
    MXC_SPI_AsyncHandler(SPI);
}
#elif (SPI_INSTANCE_NUM == 1)
#define SPI MXC_SPI1
#define SPI_IRQ SPI1_IRQn
void SPI1_IRQHandler(void)
{
    MXC_SPI_AsyncHandler(SPI);
}
#endif

void DMA0_IRQHandler(void)
{
    MXC_SPI_DMA_TX_Handler(SPI);
}

void DMA1_IRQHandler(void)
{
    MXC_SPI_DMA_RX_Handler(SPI);
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
    printf("This example configures the SPI to send data between the MISO (P0.22) and\n");
    printf("MOSI (P0.21) pins.  Connect these two pins together.  \n\n");
    printf("Multiple word sizes (2 through 16 bits) are demonstrated.\n\n");

#if MASTERSYNC
    printf("Performing blocking (synchronous) transactions...\n");
#endif
#if MASTERASYNC
    printf("Performing non-blocking (asynchronous) transactions...\n");
#endif
#if MASTERDMA
    printf("Performing transactions with DMA...\n");
#endif

    for (i = 2; i < 17; i++) {
        // Sending out 2 to 16 bits

        for (j = 0; j < DATA_LEN; j++) {
            tx_data[j] = DATA_VALUE;
        }

// #if MASTERDMA
//         // Helper function for SPI v2 drivers for DMA transactions.
//         // Must be called before MXC_SPI_Init if using SPI DMA.
//         MXC_SPI_UseDMA(1); // Use DMA (1)
// #else
//         // Not necessary when not using SPI DMA.
//         MXC_SPI_UseDMA(0); // Don't use DMA (0).
// #endif

//         // Configure the peripheral
//         retVal = MXC_SPI_Init(SPI, 1, 0, 1, 0, SPI_SPEED, spi_pins);

        mxc_spi_init_t init;
        init.spi = SPI;
        init.freq = SPI_SPEED;
        init.spi_pins = NULL;
        init.mode = MXC_SPI_INTERFACE_STANDARD;
        init.type = MXC_SPI_TYPE_CONTROLLER;
        init.clk_mode = MXC_SPI_CLKMODE_0;
        init.frame_size = i;
        init.ts_control = MXC_SPI_TSCONTROL_HW_AUTO;
        init.target.active_polarity = 0;
        init.vssel = MXC_GPIO_VSSEL_VDDIO;
        init.ts_mask = 0x01;
    
#if MASTERDMA
        init.use_dma = true;
        init.dma = MXC_DMA;
#else
        init.use_dma = false;
#endif

        retVal = MXC_SPI_Init_v2(&init);
        if (retVal != E_NO_ERROR) {
            printf("\nSPI INITIALIZATION ERROR\n");
            return retVal;
        }

        memset(rx_data, 0x0, DATA_LEN * sizeof(uint16_t));

        // SPI Request
        req.spi = SPI;
        req.txData = (uint8_t *)tx_data;
        req.rxData = (uint8_t *)rx_data;
        req.txLen = DATA_LEN;
        req.rxLen = DATA_LEN;
        req.ssIdx = 0;
        req.ssDeassert = 1;
        req.txCnt = 0;
        req.rxCnt = 0;
        req.completeCB = (mxc_spi_callback_t)SPI_Callback;
        SPI_FLAG = 1;

#if MASTERSYNC
        // Blocking SPI v2 Implementation is Interrupt driven.
        NVIC_EnableIRQ(SPI_IRQ);
        MXC_SPI_MasterTransaction(&req);
#endif

#if MASTERASYNC
        NVIC_EnableIRQ(SPI_IRQ);
        MXC_SPI_MasterTransactionAsync(&req);

        while (SPI_FLAG == 1) {}
#endif

#if MASTERDMA
        NVIC_EnableIRQ(DMA0_IRQn);
        NVIC_EnableIRQ(DMA1_IRQn);
        MXC_SPI_MasterTransactionDMA(&req);

        while (SPI_FLAG == 1) {}
#endif

        uint8_t bits = MXC_SPI_GetFrameSize(SPI);

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
            return E_COMM_ERR;
        } else {
            printf("-->%2d Bits Transaction Successful\n", i);
        }

        retVal = MXC_SPI_Shutdown(SPI);

        if (retVal != E_NO_ERROR) {
            printf("\n-->SPI SHUTDOWN ERROR: %d\n", retVal);
            return retVal;
        }
    }

    printf("\nExample Complete.\n");
    return E_NO_ERROR;
}
