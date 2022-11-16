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
 * @brief   Main for I2S CODEC loopback
 * @details I2S CODEC loopback using DMA
 */

#include <stdio.h>
#include <stdint.h>
#include "mxc.h"
#include "max9867.h"

#undef USE_I2S_INTERRUPTS

#define CODEC_I2C MXC_I2C0
#define CODEC_I2C_FREQ 100000

#define CODEC_MCLOCK 12288000
#define BITS_PER_CHANNEL 16
#define CHANNELS_PER_FRAME 2
#define SAMPLE_RATE 24000

#define BIT_CLK (SAMPLE_RATE * CHANNELS_PER_FRAME * BITS_PER_CHANNEL)
#define CLK_DIV (((CODEC_MCLOCK / 2) / BIT_CLK) - 1)

#define I2S_DMA_BUFFER_SIZE 64

volatile int dma_flag;
uint32_t i2s_rx_buffer[I2S_DMA_BUFFER_SIZE * 2];
int dma_ch_tx, dma_ch_rx;
uint32_t *rxBufPtr = i2s_rx_buffer;

void blink_halt(const char *msg)
{
    if (msg && *msg)
        puts(msg);

    for (;;) {
        LED_On(LED1);
        LED_Off(LED2);
        MXC_Delay(MXC_DELAY_MSEC(500));
        LED_On(LED2);
        LED_Off(LED1);
        MXC_Delay(MXC_DELAY_MSEC(500));
    }
}

void dma_handler(void)
{
    dma_flag = 1;
    MXC_DMA_Handler();
}

void dma_init(void)
{
    MXC_NVIC_SetVector(DMA0_IRQn, dma_handler);
    MXC_NVIC_SetVector(DMA1_IRQn, dma_handler);
    NVIC_EnableIRQ(DMA0_IRQn);
    NVIC_EnableIRQ(DMA1_IRQn);
}

void dma_callback(int channel, int result)
{
    static uint32_t *tx_buf = i2s_rx_buffer + I2S_DMA_BUFFER_SIZE;

    if (channel == dma_ch_tx) {
        MXC_DMA_ReleaseChannel(dma_ch_tx);
        dma_ch_tx = MXC_I2S_TXDMAConfig(tx_buf, I2S_DMA_BUFFER_SIZE * sizeof(i2s_rx_buffer[0]));

    } else if (channel == dma_ch_rx) {
        tx_buf = rxBufPtr;

        if (rxBufPtr == i2s_rx_buffer) {
            rxBufPtr = i2s_rx_buffer + I2S_DMA_BUFFER_SIZE;
        } else {
            rxBufPtr = i2s_rx_buffer;
        }
        MXC_DMA_ReleaseChannel(dma_ch_rx);
        dma_ch_rx = MXC_I2S_RXDMAConfig(rxBufPtr, I2S_DMA_BUFFER_SIZE * sizeof(i2s_rx_buffer[0]));
    }
}

void dma_work_loop(void)
{
    int trig = 0;

    dma_init();
    MXC_I2S_RegisterDMACallback(dma_callback);
    dma_ch_tx = MXC_I2S_TXDMAConfig(i2s_rx_buffer + I2S_DMA_BUFFER_SIZE,
                                    I2S_DMA_BUFFER_SIZE * sizeof(i2s_rx_buffer[0]));
    dma_ch_rx = MXC_I2S_RXDMAConfig(i2s_rx_buffer, I2S_DMA_BUFFER_SIZE * sizeof(i2s_rx_buffer[0]));

    for (;;) {
        if (dma_flag) {
            dma_flag = 0;
            /*
        dma activity triggered work
      */
            if (++trig == SAMPLE_RATE / I2S_DMA_BUFFER_SIZE) {
                trig = 0;
                LED_Toggle(LED2);
            }
        }
        /*
      non-dma activity triggered work
    */
    }
}

void i2c_init(void)
{
    if (MXC_I2C_Init(CODEC_I2C, 1, 0))
        blink_halt("Error initializing I2C controller");

    MXC_I2C_SetFrequency(CODEC_I2C, CODEC_I2C_FREQ);
}

void codec_init(void)
{
    if (max9867_init(CODEC_I2C, CODEC_MCLOCK, 1))
        blink_halt("Error initializing MAX9867 CODEC");

    if (max9867_enable_playback(1))
        blink_halt("Error enabling playback path");

    if (max9867_playback_volume(-6, -6))
        blink_halt("Error setting playback volume");

    if (max9867_enable_record(1))
        blink_halt("Error enabling record path");

    if (max9867_adc_level(-12, -12))
        blink_halt("Error setting ADC level");

    if (max9867_linein_gain(-6, -6))
        blink_halt("Error setting Line-In gain");
}

void i2s_init(void)
{
    mxc_i2s_req_t req;

#define I2S_CRUFT_PTR (void *)UINT32_MAX
#define I2S_CRUFT_LEN UINT32_MAX

    req.wordSize = MXC_I2S_DATASIZE_HALFWORD;
    req.sampleSize = MXC_I2S_SAMPLESIZE_SIXTEEN;
    req.justify = MXC_I2S_MSB_JUSTIFY;
    req.wsPolarity = MXC_I2S_POL_NORMAL;
    req.channelMode = MXC_I2S_INTERNAL_SCK_WS_0;
    req.stereoMode = MXC_I2S_STEREO;

    req.bitOrder = MXC_I2S_MSB_FIRST;
    req.clkdiv = CLK_DIV;

    req.rawData = NULL;
    req.txData = I2S_CRUFT_PTR;
    req.rxData = I2S_CRUFT_PTR;
    req.length = I2S_CRUFT_LEN;

    if (MXC_I2S_Init(&req))
        blink_halt("Error initializing I2S");

    MXC_I2S_SetFrequency(MXC_I2S_EXTERNAL_SCK_EXTERNAL_WS, 0);
}

int main(void)
{
#if defined(BOARD_FTHR_REVA)
    /* Wait for PMIC 1.8V to become available, about 180ms after power up. */
    MXC_Delay(MXC_DELAY_MSEC(200));
#endif

    /* Switch to 100 MHz clock */
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();

    printf("***** MAX9867 CODEC DMA Loopback Example *****\n");

    printf("Waiting...\n");

    /* DO NOT DELETE THIS LINE: */
    MXC_Delay(MXC_DELAY_SEC(2)); /* Let debugger interrupt if needed */

    printf("Running...\n");

    i2c_init();
    codec_init();
    i2s_init();

    dma_work_loop();
}
