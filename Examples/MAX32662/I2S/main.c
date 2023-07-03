/**
 * @file        main.c
 * @brief       I2S Loopback Example 
 * @details     
 * @note        
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
#include <stdint.h>
#include <string.h>
#include "board.h"
#include "dma.h"
#include "i2s.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "nvic_table.h"
#include "tmr.h"

/***** Global Data *****/
uint16_t tone[64] = { 0x8000, 0x8c8b, 0x98f8, 0xa527, 0xb0fb, 0xbc56, 0xc71c, 0xd133,
                      0xda82, 0xe2f1, 0xea6d, 0xf0e2, 0xf641, 0xfa7c, 0xfd89, 0xff61,
                      0xffff, 0xff61, 0xfd89, 0xfa7c, 0xf641, 0xf0e2, 0xea6d, 0xe2f1,
                      0xda82, 0xd133, 0xc71c, 0xbc56, 0xb0fb, 0xa527, 0x98f8, 0x8c8b,
                      0x8000, 0x7374, 0x6707, 0x5ad8, 0x4f04, 0x43a9, 0x38e3, 0x2ecc,
                      0x257d, 0x1d0e, 0x1592, 0x0f1d, 0x09be, 0x0583, 0x0276, 0x009e,
                      0x0000, 0x009e, 0x0276, 0x0583, 0x09be, 0x0f1d, 0x1592, 0x1d0e,
                      0x257d, 0x2ecc, 0x38e3, 0x43a9, 0x4f04, 0x5ad8, 0x6707, 0x7374 };

mxc_i2s_req_t req;
uint32_t tx_complete = 1;

/***** Functions *****/
void DMA0_IRQHandler(void)
{
    MXC_DMA_Handler();
}

void tx_complete_cb(int ch, int err)
{
    tx_complete = err;
}

/*****************************************************************/
int main()
{
    int err;
    printf("\n\n****************** I2S Transmission Example ******************\n");
    printf("This example demonstrates how to transmit a set of audio samples\n");
    printf("over the I2S bus. The transmission is configured to send 16-bit\n");
    printf("samples in stereo mode at a 16kHz sampling rate. The I2S signals\n");
    printf("are available to view on pins P0.2-P0.5.\n");

    req.channelMode = MXC_I2S_INTERNAL_SCK_WS_0; // Set I2S configurations
    req.stereoMode = MXC_I2S_STEREO;
    req.wordSize = MXC_I2S_DATASIZE_HALFWORD;
    req.justify = MXC_I2S_MSB_JUSTIFY;
    req.bitOrder = MXC_I2S_MSB_FIRST;
    req.wsPolarity = MXC_I2S_POL_NORMAL;
    req.sampleSize = MXC_I2S_SAMPLESIZE_SIXTEEN;
    req.clkdiv =
        ERFO_FREQ / (2 * 16 * 16000); // SRC_CLK / (2 * BITS_PER_SAMPLE * DESIRED_WSCLK_FREQ)
    req.rawData = tone;
    req.txData = tone;
    req.rxData = NULL;
    req.length = 64;

    if ((err = MXC_I2S_Init(&req)) != E_NO_ERROR) { // Initialize/Configure I2S
        printf("\nError in I2S_Init: %d\n", err);
        while (1) {}
    }

    printf("\nI2S Initialized.\n");

    MXC_DMA_ReleaseChannel(0);
    NVIC_EnableIRQ(DMA0_IRQn);

    MXC_I2S_RegisterDMACallback(tx_complete_cb);

    printf("Transmitting audio samples...\n");
    MXC_I2S_TXDMAConfig(
        req.txData,
        2 * req.length); // Start transmission (DMA_Length = Num_Samples * Num_Bytes_Per_Sample)

    while (tx_complete == 1) {}
    // Wait for DMA to finish
    if (tx_complete != E_NO_ERROR) {
        printf("\nError transmitting I2S data.\n");
    }

    while (MXC_I2S->dmach0 & MXC_F_I2S_DMACH0_TX_LVL) {}
    // Wait for final audio sample to finish transmitting
    MXC_Delay(50);

    if ((err = MXC_I2S_Shutdown()) != E_NO_ERROR) {
        Console_Init();
        printf("\nCould not shut down I2S driver: %d\n", err);
        while (1) {}
    }

    printf("I2S Transmission Complete.\n");

    return E_NO_ERROR;
}
