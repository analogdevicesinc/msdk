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
#include "mxc_device.h"
#include "mxc_delay.h"
#include "nvic_table.h"
#include "spimss_regs.h"
#include "board.h"
#include "i2s.h"
#include "tmr.h"
#include "dma.h"

/***** Global Data *****/
uint16_t tone[64] = { 0x8000, 0x8c8b, 0x98f8, 0xa527, 0xb0fb, 0xbc56, 0xc71c, 0xd133,
                      0xda82, 0xe2f1, 0xea6d, 0xf0e2, 0xf641, 0xfa7c, 0xfd89, 0xff61,
                      0xffff, 0xff61, 0xfd89, 0xfa7c, 0xf641, 0xf0e2, 0xea6d, 0xe2f1,
                      0xda82, 0xd133, 0xc71c, 0xbc56, 0xb0fb, 0xa527, 0x98f8, 0x8c8b,
                      0x8000, 0x7374, 0x6707, 0x5ad8, 0x4f04, 0x43a9, 0x38e3, 0x2ecc,
                      0x257d, 0x1d0e, 0x1592, 0x0f1d, 0x09be, 0x0583, 0x0276, 0x009e,
                      0x0000, 0x009e, 0x0276, 0x0583, 0x09be, 0x0f1d, 0x1592, 0x1d0e,
                      0x257d, 0x2ecc, 0x38e3, 0x43a9, 0x4f04, 0x5ad8, 0x6707, 0x7374 };

mxc_i2s_config_t config;

/***** Functions *****/
void DMA0_IRQHandler(void)
{
    MXC_DMA_Handler();
}

/* Reset DMA Reload so we loop the audio sample forever */
void dma_ctz_cb(int ch, int err)
{
    MXC_I2S_DMA_SetReload(config.src_addr, config.dst_addr, config.length);
    MXC_I2S_DMA_ClearFlags();
}

/*****************************************************************/
int main()
{
    int err;
    printf("\nI2S Transmission Example\n\n");
    printf("\n\n\n\n");

    config.map = 0;
    config.justify = 0;
    config.audio_mode = 1;
    config.audio_direction = AUDIO_OUT;
    config.sample_rate = 16000;
    config.start_immediately = 0;
    config.src_addr = tone;
    config.dst_addr = NULL;
    config.length = 128;
    config.dma_reload_en = 1;

    Console_Shutdown();
    if ((err = MXC_I2S_Init(&config, dma_ctz_cb)) != E_NO_ERROR) {
        Console_Init();
        printf("\nError in I2S_Init: %d\n", err);
        while (1) {}
    }

    NVIC_EnableIRQ(DMA0_IRQn);

    MXC_Delay(MXC_DELAY_MSEC(1000));

    // Starting I2S Output
    MXC_I2S_Start();
    MXC_Delay(MXC_DELAY_MSEC(1000));

    // Muting I2S Output
    MXC_I2S_Mute();
    MXC_Delay(MXC_DELAY_MSEC(1000));

    // Unmuting I2S Output
    MXC_I2S_Unmute();
    MXC_Delay(MXC_DELAY_MSEC(1000));

    // Pausing I2S Output
    MXC_I2S_Pause();
    MXC_Delay(MXC_DELAY_MSEC(1000));

    // Resuming I2S Output
    MXC_I2S_Unpause();
    MXC_Delay(MXC_DELAY_MSEC(1000));

    // Stopping I2S Output.
    MXC_I2S_Stop();

    if ((err = MXC_I2S_Shutdown()) != E_NO_ERROR) {
        Console_Init();
        printf("\nCould not shut down I2S driver: %d\n", err);
        while (1) {}
    }

    Console_Init();
    printf("\nI2S Transaction Complete. Ignore any random characters previously");
    printf("\ndisplayed. The I2S and UART are sharing the same pins.\n");

    return 0;
}
