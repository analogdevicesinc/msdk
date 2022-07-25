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

// cifar-10-auto-test

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "mxc.h"
#include "cnn.h"
#include "spi.h"

#define FT4222_CLK 60000000
#define DATA_LEN   3072
#define SPI_REGS   MXC_SPI1
#define SPI_IRQ    SPI1_IRQn
#define SPI_SPEED  FT4222_CLK / 8 // Must match the SPI clock in python script

uint8_t rx_data[DATA_LEN];
uint8_t tx_data[1];
volatile bool SPI_FLAG = false;

volatile uint32_t cnn_time; // Stopwatch

void fail(void)
{
    printf("\n*** FAIL ***\n\n");
    while (1)
        ;
}

int8_t unsigned_to_signed(uint8_t val)
{
    return val - 128;
}

// 3-channel 32x32 data input (3072 bytes total / 1024 bytes per channel):
// HWC 32x32, channels 0 to 2
void memcpy_input(uint32_t* dst, uint8_t* src, size_t n)
{
    uint32_t val;

    while (n--) {
        val = *src++;          // R
        val += (*src++) << 8;  // G
        val += (*src++) << 16; // B
        *dst++ = val;
    }
}

void load_input(void)
{
    // This function loads the input data from SPI

    int retVal;
    int i;
    mxc_spi_req_t req;

    // SPI transaction request
    req.spi    = SPI_REGS;
    req.txData = (uint8_t*)tx_data;
    req.rxData = (uint8_t*)rx_data;
    req.txLen  = 0;
    req.rxLen  = DATA_LEN;
    req.ssIdx  = 0;

    // Blocking SPI transaction
    retVal = MXC_SPI_SlaveTransaction(&req);

    if (retVal != E_NO_ERROR) {
        printf("SPI Transaction failed!\n");
    } else {
        for (i = 0; i < DATA_LEN; i++) { rx_data[i] = unsigned_to_signed(rx_data[i]); }
        memcpy_input((uint32_t*)0x50400000, rx_data, 1024);
    }
}

// Classification layer:
static int32_t ml_data[CNN_NUM_OUTPUTS];
static q15_t ml_softmax[CNN_NUM_OUTPUTS];

void softmax_layer(void)
{
    cnn_unload((uint32_t*)ml_data);
    softmax_q17p14_q15((const q31_t*)ml_data, CNN_NUM_OUTPUTS, ml_softmax);
}

static void spi_enable_interrupts(uint32_t mask)
{
    MXC_SPI_EnableInt(SPI_REGS, mask);
    NVIC_EnableIRQ(SPI_IRQ);
}

static void spi_clear_interrupts(uint32_t mask)
{
    SPI_REGS->intfl |= mask;
}

static void spi_init(void)
{
    int retVal;
    mxc_spi_pins_t spi_pins;

    spi_pins.clock = TRUE;
    spi_pins.miso  = TRUE;
    spi_pins.mosi  = TRUE;
    spi_pins.ss0   = TRUE;
    spi_pins.ss1   = FALSE;
    spi_pins.ss2   = FALSE;
    spi_pins.sdio2 = FALSE;
    spi_pins.sdio3 = FALSE;

    retVal = MXC_SPI_Init(SPI_REGS, 1, 0, 1, 0, SPI_SPEED, spi_pins);
    if (retVal != E_NO_ERROR) {
        printf("SPI Slave Initialization Error\n");
        while (1)
            ;
    }
    retVal = MXC_SPI_SetDataSize(SPI_REGS, 8); // 8 bits per character

    if (retVal != E_NO_ERROR) {
        printf("SPI SET DATASIZE ERROR: %d\n", retVal);
        while (1)
            ;
    }

    retVal = MXC_SPI_SetWidth(SPI_REGS, SPI_WIDTH_STANDARD);

    if (retVal != E_NO_ERROR) {
        printf("SPI SET WIDTH ERROR: %d\n", retVal);
        while (1)
            ;
    }

    spi_enable_interrupts(MXC_F_SPI_INTEN_SSA);
}

void SPI1_IRQHandler(void)
{
    if (MXC_GPIO_InGet(MXC_GPIO0, MXC_GPIO_PIN_20) == 1) {
        return;
    }

    if (MXC_SPI_GetFlags(SPI_REGS) & MXC_F_SPI_INTFL_SSA) {
        spi_clear_interrupts(MXC_F_SPI_INTFL_SSA);
        SPI_FLAG = true;
    }
}

int main(void)
{
    int32_t max_ml;   // ml before going to soft_max
    int16_t class_id; // Identified class

    MXC_ICC_Enable(MXC_ICC0); // Enable cache

    // Switch to 100 MHz clock
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();

    // DO NOT DELETE THIS LINE:
    MXC_Delay(SEC(2)); // Let debugger interrupt if needed

    // Enable peripheral, enable CNN interrupt, turn on CNN clock
    // CNN clock: 50 MHz div 1
    cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);

    spi_init();
    cnn_init();         // Bring state machine into consistent state
    cnn_load_weights(); // Load kernels
    cnn_load_bias();
    cnn_configure(); // Configure state machine

    while (1) {
        if (SPI_FLAG) {
            load_input(); // Load data input
            cnn_start();  // Start CNN processing
            while ((*((volatile uint32_t*)0x50100000) & (1 << 12)) != 1 << 12)
                ; // Wait for CNN to complete processing
            softmax_layer();

            max_ml   = 1 << 31;
            class_id = -1;
            // Classification of the results
            for (int i = 0; i < CNN_NUM_OUTPUTS; i++) {
                if (ml_data[i] > max_ml) {
                    max_ml   = ml_data[i];
                    class_id = i;
                }
            }

            SPI_FLAG = false;
            spi_clear_interrupts(MXC_F_SPI_INTFL_SSA);
            spi_enable_interrupts(MXC_F_SPI_INTEN_SSA);

            printf("Class:%d\n", class_id);
        }
    }

    return 0;
}
