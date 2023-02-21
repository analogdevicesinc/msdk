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
/**
 * @file    main_riscv.c
 * @brief   MNIST streaming example with the OV7692.
 *
 * @details This example uses MNIST dataset. UART1 is used to stream out the image captured from the camera.
 *          The image is prepended with a header that is interpreted by the grab_image.py
 *          python script.  The data from this example through the UART is in a binary format.
 *          Instructions: 1) Load and execute this example. The example will initialize the camera
 *                        and start capturing binary output of camera frame data on PB1 press.
 *                        2) To display image run 'sudo python3 grab_image.py /dev/ttyUSB0 115200'
 *                           Substitute the /dev/ttyUSB0 string for the serial port on your system.
 *                           The python program will read in the binary data from this example and
 *                           output a png image.
 */

// mnist-stream_0
// Created using ./ai8xize.py -e --verbose --top-level cnn -L --test-dir demos --prefix mnist-stream_0 --checkpoint-file trained/ai85-mnist.pth.tar --config-file networks/mnist-chw-ai85.yaml --device MAX78000 --compact-data --mexpress --softmax --display-checkpoint --riscv --riscv-flash --riscv-cache --riscv-debug --fast-fifo --streaming-layer 0

// Configuring 5 layers:
// Layer 0: 1x28x28 (streaming CHW/big data), no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, 60x28x28 output
// Layer 1: 60x28x28 (HWC/little data), 2x2 max pool with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 2/2, 60x16x16 output
// Layer 2: 60x16x16 (HWC/little data), 2x2 max pool with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, 56x8x8 output
// Layer 3: 56x8x8 (HWC/little data), 2x2 avg pool with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, 12x4x4 output
// Layer 4: 12x4x4 (flattened HWC/little data), no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, 10x1x1 output

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "mxc_device.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "gcfr_regs.h"
#include "fcr_regs.h"
#include "icc.h"
#include "led.h"
#include "tmr.h"
#include "tornadocnn.h"
#include "weights.h"
#include "sampledata.h"
#include "pb.h"
#include "camera.h"
#include "utils.h"
#include "cameraif_regs.h"
#include "cameraif.h"
#include "uart.h"
#include "board.h"

// Comment out USE_SAMPLEDATA to use Camera module
//#define USE_SAMPLEDATA

#define IMAGE_SIZE_X (28)
#define IMAGE_SIZE_Y (28)
#define CAMERA_FREQ (10 * 1000 * 1000)

uint32_t camera_sampledata[196];
uint32_t cnn_time; // Stopwatch
static uint8_t *rxdata = NULL;

void fail(void)
{
    printf("\n*** FAIL ***\n\n");
    while (1) {}
}

void cnn_wait(void)
{
    while ((*((volatile uint32_t *)0x50100000) & (1 << 12)) != 1 << 12) {}
    CNN_COMPLETE; // Signal that processing is complete
    cnn_time = MXC_TMR_SW_Stop(MXC_TMR0);
}

void memcpy32(uint32_t *dst, const uint32_t *src, int n)
{
    while (n-- > 0) {
        *dst++ = *src++;
    }
}

#ifdef USE_SAMPLEDATA
// Data input: CHW (big data): 1x28x28
__attribute__((section(".rvflash_section"))) static const uint32_t input_0[] = INPUT_0;
#endif

void load_input(void)
{
    int i;
#ifdef USE_SAMPLEDATA
    const uint32_t *in0 = input_0;
#else
    const uint32_t *in0 = camera_sampledata;
#endif

    for (i = 0; i < 196; i++) {
        while (((*((volatile uint32_t *)0x400c0404) & 2)) != 0) {}
        // Wait for FIFO
        *((volatile uint32_t *)0x400c0410) = *in0++; // Write FIFO 0
    }
}

// Kernels:
static const uint32_t kernels_0[] = KERNELS_0;
static const uint32_t kernels_1[] = KERNELS_1;
static const uint32_t kernels_2[] = KERNELS_2;
static const uint32_t kernels_3[] = KERNELS_3;
static const uint32_t kernels_4[] = KERNELS_4;
static const uint32_t kernels_5[] = KERNELS_5;
static const uint32_t kernels_6[] = KERNELS_6;
static const uint32_t kernels_7[] = KERNELS_7;
static const uint32_t kernels_8[] = KERNELS_8;
static const uint32_t kernels_9[] = KERNELS_9;
static const uint32_t kernels_10[] = KERNELS_10;
static const uint32_t kernels_11[] = KERNELS_11;
static const uint32_t kernels_12[] = KERNELS_12;
static const uint32_t kernels_13[] = KERNELS_13;
static const uint32_t kernels_14[] = KERNELS_14;
static const uint32_t kernels_15[] = KERNELS_15;
static const uint32_t kernels_16[] = KERNELS_16;
static const uint32_t kernels_17[] = KERNELS_17;
static const uint32_t kernels_18[] = KERNELS_18;
static const uint32_t kernels_19[] = KERNELS_19;
static const uint32_t kernels_20[] = KERNELS_20;
static const uint32_t kernels_21[] = KERNELS_21;
static const uint32_t kernels_22[] = KERNELS_22;
static const uint32_t kernels_23[] = KERNELS_23;
static const uint32_t kernels_24[] = KERNELS_24;
static const uint32_t kernels_25[] = KERNELS_25;
static const uint32_t kernels_26[] = KERNELS_26;
static const uint32_t kernels_27[] = KERNELS_27;
static const uint32_t kernels_28[] = KERNELS_28;
static const uint32_t kernels_29[] = KERNELS_29;
static const uint32_t kernels_30[] = KERNELS_30;
static const uint32_t kernels_31[] = KERNELS_31;
static const uint32_t kernels_32[] = KERNELS_32;
static const uint32_t kernels_33[] = KERNELS_33;
static const uint32_t kernels_34[] = KERNELS_34;
static const uint32_t kernels_35[] = KERNELS_35;
static const uint32_t kernels_36[] = KERNELS_36;
static const uint32_t kernels_37[] = KERNELS_37;
static const uint32_t kernels_38[] = KERNELS_38;
static const uint32_t kernels_39[] = KERNELS_39;
static const uint32_t kernels_40[] = KERNELS_40;
static const uint32_t kernels_41[] = KERNELS_41;
static const uint32_t kernels_42[] = KERNELS_42;
static const uint32_t kernels_43[] = KERNELS_43;
static const uint32_t kernels_44[] = KERNELS_44;
static const uint32_t kernels_45[] = KERNELS_45;
static const uint32_t kernels_46[] = KERNELS_46;
static const uint32_t kernels_47[] = KERNELS_47;
static const uint32_t kernels_48[] = KERNELS_48;
static const uint32_t kernels_49[] = KERNELS_49;
static const uint32_t kernels_50[] = KERNELS_50;
static const uint32_t kernels_51[] = KERNELS_51;
static const uint32_t kernels_52[] = KERNELS_52;
static const uint32_t kernels_53[] = KERNELS_53;
static const uint32_t kernels_54[] = KERNELS_54;
static const uint32_t kernels_55[] = KERNELS_55;
static const uint32_t kernels_56[] = KERNELS_56;
static const uint32_t kernels_57[] = KERNELS_57;
static const uint32_t kernels_58[] = KERNELS_58;
static const uint32_t kernels_59[] = KERNELS_59;
static const uint32_t kernels_60[] = KERNELS_60;
static const uint32_t kernels_61[] = KERNELS_61;
static const uint32_t kernels_62[] = KERNELS_62;
static const uint32_t kernels_63[] = KERNELS_63;

void load_kernels(void)
{
    *((volatile uint8_t *)0x50180001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50180000, kernels_0, 329);
    *((volatile uint8_t *)0x50184001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50184000, kernels_1, 329);
    *((volatile uint8_t *)0x50188001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50188000, kernels_2, 329);
    *((volatile uint8_t *)0x5018c001) = 0x01; // Set address
    memcpy32((uint32_t *)0x5018c000, kernels_3, 329);
    *((volatile uint8_t *)0x50190001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50190000, kernels_4, 329);
    *((volatile uint8_t *)0x50194001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50194000, kernels_5, 329);
    *((volatile uint8_t *)0x50198001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50198000, kernels_6, 329);
    *((volatile uint8_t *)0x5019c001) = 0x01; // Set address
    memcpy32((uint32_t *)0x5019c000, kernels_7, 329);
    *((volatile uint8_t *)0x501a0001) = 0x01; // Set address
    memcpy32((uint32_t *)0x501a0000, kernels_8, 329);
    *((volatile uint8_t *)0x501a4001) = 0x01; // Set address
    memcpy32((uint32_t *)0x501a4000, kernels_9, 329);
    *((volatile uint8_t *)0x501a8001) = 0x01; // Set address
    memcpy32((uint32_t *)0x501a8000, kernels_10, 329);
    *((volatile uint8_t *)0x501ac001) = 0x01; // Set address
    memcpy32((uint32_t *)0x501ac000, kernels_11, 329);
    *((volatile uint8_t *)0x501b0001) = 0x01; // Set address
    memcpy32((uint32_t *)0x501b0000, kernels_12, 288);
    *((volatile uint8_t *)0x501b4001) = 0x01; // Set address
    memcpy32((uint32_t *)0x501b4000, kernels_13, 288);
    *((volatile uint8_t *)0x501b8001) = 0x01; // Set address
    memcpy32((uint32_t *)0x501b8000, kernels_14, 288);
    *((volatile uint8_t *)0x501bc001) = 0x01; // Set address
    memcpy32((uint32_t *)0x501bc000, kernels_15, 288);
    *((volatile uint8_t *)0x50580001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50580000, kernels_16, 288);
    *((volatile uint8_t *)0x50584001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50584000, kernels_17, 288);
    *((volatile uint8_t *)0x50588001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50588000, kernels_18, 288);
    *((volatile uint8_t *)0x5058c001) = 0x01; // Set address
    memcpy32((uint32_t *)0x5058c000, kernels_19, 288);
    *((volatile uint8_t *)0x50590001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50590000, kernels_20, 288);
    *((volatile uint8_t *)0x50594001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50594000, kernels_21, 288);
    *((volatile uint8_t *)0x50598001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50598000, kernels_22, 288);
    *((volatile uint8_t *)0x5059c001) = 0x01; // Set address
    memcpy32((uint32_t *)0x5059c000, kernels_23, 288);
    *((volatile uint8_t *)0x505a0001) = 0x01; // Set address
    memcpy32((uint32_t *)0x505a0000, kernels_24, 288);
    *((volatile uint8_t *)0x505a4001) = 0x01; // Set address
    memcpy32((uint32_t *)0x505a4000, kernels_25, 288);
    *((volatile uint8_t *)0x505a8001) = 0x01; // Set address
    memcpy32((uint32_t *)0x505a8000, kernels_26, 288);
    *((volatile uint8_t *)0x505ac001) = 0x01; // Set address
    memcpy32((uint32_t *)0x505ac000, kernels_27, 288);
    *((volatile uint8_t *)0x505b0001) = 0x01; // Set address
    memcpy32((uint32_t *)0x505b0000, kernels_28, 288);
    *((volatile uint8_t *)0x505b4001) = 0x01; // Set address
    memcpy32((uint32_t *)0x505b4000, kernels_29, 288);
    *((volatile uint8_t *)0x505b8001) = 0x01; // Set address
    memcpy32((uint32_t *)0x505b8000, kernels_30, 288);
    *((volatile uint8_t *)0x505bc001) = 0x01; // Set address
    memcpy32((uint32_t *)0x505bc000, kernels_31, 288);
    *((volatile uint8_t *)0x50980001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50980000, kernels_32, 288);
    *((volatile uint8_t *)0x50984001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50984000, kernels_33, 288);
    *((volatile uint8_t *)0x50988001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50988000, kernels_34, 288);
    *((volatile uint8_t *)0x5098c001) = 0x01; // Set address
    memcpy32((uint32_t *)0x5098c000, kernels_35, 288);
    *((volatile uint8_t *)0x50990001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50990000, kernels_36, 288);
    *((volatile uint8_t *)0x50994001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50994000, kernels_37, 288);
    *((volatile uint8_t *)0x50998001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50998000, kernels_38, 288);
    *((volatile uint8_t *)0x5099c001) = 0x01; // Set address
    memcpy32((uint32_t *)0x5099c000, kernels_39, 288);
    *((volatile uint8_t *)0x509a0001) = 0x01; // Set address
    memcpy32((uint32_t *)0x509a0000, kernels_40, 288);
    *((volatile uint8_t *)0x509a4001) = 0x01; // Set address
    memcpy32((uint32_t *)0x509a4000, kernels_41, 288);
    *((volatile uint8_t *)0x509a8001) = 0x01; // Set address
    memcpy32((uint32_t *)0x509a8000, kernels_42, 288);
    *((volatile uint8_t *)0x509ac001) = 0x01; // Set address
    memcpy32((uint32_t *)0x509ac000, kernels_43, 288);
    *((volatile uint8_t *)0x509b0001) = 0x01; // Set address
    memcpy32((uint32_t *)0x509b0000, kernels_44, 288);
    *((volatile uint8_t *)0x509b4001) = 0x01; // Set address
    memcpy32((uint32_t *)0x509b4000, kernels_45, 288);
    *((volatile uint8_t *)0x509b8001) = 0x01; // Set address
    memcpy32((uint32_t *)0x509b8000, kernels_46, 288);
    *((volatile uint8_t *)0x509bc001) = 0x01; // Set address
    memcpy32((uint32_t *)0x509bc000, kernels_47, 288);
    *((volatile uint8_t *)0x50d80001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50d80000, kernels_48, 288);
    *((volatile uint8_t *)0x50d84001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50d84000, kernels_49, 288);
    *((volatile uint8_t *)0x50d88001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50d88000, kernels_50, 288);
    *((volatile uint8_t *)0x50d8c001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50d8c000, kernels_51, 288);
    *((volatile uint8_t *)0x50d90001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50d90000, kernels_52, 288);
    *((volatile uint8_t *)0x50d94001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50d94000, kernels_53, 288);
    *((volatile uint8_t *)0x50d98001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50d98000, kernels_54, 288);
    *((volatile uint8_t *)0x50d9c001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50d9c000, kernels_55, 288);
    *((volatile uint8_t *)0x50da0001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50da0000, kernels_56, 288);
    *((volatile uint8_t *)0x50da4001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50da4000, kernels_57, 288);
    *((volatile uint8_t *)0x50da8001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50da8000, kernels_58, 288);
    *((volatile uint8_t *)0x50dac001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50dac000, kernels_59, 288);
    *((volatile uint8_t *)0x50db0001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50db0000, kernels_60, 261);
    *((volatile uint8_t *)0x50db4001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50db4000, kernels_61, 261);
    *((volatile uint8_t *)0x50db8001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50db8000, kernels_62, 261);
    *((volatile uint8_t *)0x50dbc001) = 0x01; // Set address
    memcpy32((uint32_t *)0x50dbc000, kernels_63, 261);
}

static const uint8_t bias_0[] = BIAS_0;

void memcpy_8to32(uint32_t *dst, const uint8_t *src, size_t n)
{
    while (n-- > 0) {
        *dst++ = *src++;
    }
}

void load_bias(void)
{
    memcpy_8to32((uint32_t *)0x50108000, bias_0, sizeof(uint8_t) * 10);
}

int cnn_load(void)
{
    *((volatile uint32_t *)0x50001000) = 0x00000000; // AON control
    *((volatile uint32_t *)0x50100000) = 0x00108008; // Stop SM
    *((volatile uint32_t *)0x50100004) = 0x0000040e; // SRAM control
    *((volatile uint32_t *)0x50100008) = 0x00000004; // Layer count

    *((volatile uint32_t *)0x50500000) = 0x00108008; // Stop SM
    *((volatile uint32_t *)0x50500004) = 0x0000040e; // SRAM control
    *((volatile uint32_t *)0x50500008) = 0x00000004; // Layer count

    *((volatile uint32_t *)0x50900000) = 0x00108008; // Stop SM
    *((volatile uint32_t *)0x50900004) = 0x0000040e; // SRAM control
    *((volatile uint32_t *)0x50900008) = 0x00000004; // Layer count

    *((volatile uint32_t *)0x50d00000) = 0x00108008; // Stop SM
    *((volatile uint32_t *)0x50d00004) = 0x0000040e; // SRAM control
    *((volatile uint32_t *)0x50d00008) = 0x00000004; // Layer count

    load_kernels();
    load_bias();

    // Layer 0 group 0
    *((volatile uint32_t *)0x50100010) = 0x0001001d; // Rows
    *((volatile uint32_t *)0x50100090) = 0x0001001d; // Columns
    *((volatile uint32_t *)0x50100310) = 0x00002800; // SRAM write ptr
    *((volatile uint32_t *)0x50100410) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50100490) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x50100590) = 0x00000b60; // Layer control
    *((volatile uint32_t *)0x50100a10) = 0x0001d800; // Layer control 2
    *((volatile uint32_t *)0x50100610) = 0x000001d8; // Mask offset and count
    *((volatile uint32_t *)0x50100690) = 0x0000001b; // TRAM ptr max
    *((volatile uint32_t *)0x50100710) = 0x00010001; // Mask and processor enables
    *((volatile uint32_t *)0x50100810) = 0x00000001; // Stream processing start
    *((volatile uint32_t *)0x50100910) = 0x0000000c; // Rollover
    *((volatile uint32_t *)0x50100990) = 0x000000c4; // Input frame size

    // Layer 0 group 1
    *((volatile uint32_t *)0x50500010) = 0x0001001d; // Rows
    *((volatile uint32_t *)0x50500090) = 0x0001001d; // Columns
    *((volatile uint32_t *)0x50500310) = 0x00002800; // SRAM write ptr
    *((volatile uint32_t *)0x50500410) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50500490) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x50500590) = 0x00000b60; // Layer control
    *((volatile uint32_t *)0x50500a10) = 0x0001d800; // Layer control 2
    *((volatile uint32_t *)0x50500610) = 0x000001d8; // Mask offset and count
    *((volatile uint32_t *)0x50500690) = 0x0000001b; // TRAM ptr max
    *((volatile uint32_t *)0x50500810) = 0x00000001; // Stream processing start
    *((volatile uint32_t *)0x50500910) = 0x0000000c; // Rollover
    *((volatile uint32_t *)0x50500990) = 0x000000c4; // Input frame size

    // Layer 0 group 2
    *((volatile uint32_t *)0x50900010) = 0x0001001d; // Rows
    *((volatile uint32_t *)0x50900090) = 0x0001001d; // Columns
    *((volatile uint32_t *)0x50900310) = 0x00002800; // SRAM write ptr
    *((volatile uint32_t *)0x50900410) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50900490) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x50900590) = 0x00000b60; // Layer control
    *((volatile uint32_t *)0x50900a10) = 0x0001d800; // Layer control 2
    *((volatile uint32_t *)0x50900610) = 0x000001d8; // Mask offset and count
    *((volatile uint32_t *)0x50900690) = 0x0000001b; // TRAM ptr max
    *((volatile uint32_t *)0x50900810) = 0x00000001; // Stream processing start
    *((volatile uint32_t *)0x50900910) = 0x0000000c; // Rollover
    *((volatile uint32_t *)0x50900990) = 0x000000c4; // Input frame size

    // Layer 0 group 3
    *((volatile uint32_t *)0x50d00010) = 0x0001001d; // Rows
    *((volatile uint32_t *)0x50d00090) = 0x0001001d; // Columns
    *((volatile uint32_t *)0x50d00310) = 0x00002800; // SRAM write ptr
    *((volatile uint32_t *)0x50d00410) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50d00490) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x50d00590) = 0x00000b60; // Layer control
    *((volatile uint32_t *)0x50d00a10) = 0x0001d800; // Layer control 2
    *((volatile uint32_t *)0x50d00610) = 0x000001d8; // Mask offset and count
    *((volatile uint32_t *)0x50d00690) = 0x0000001b; // TRAM ptr max
    *((volatile uint32_t *)0x50d00810) = 0x00000001; // Stream processing start
    *((volatile uint32_t *)0x50d00910) = 0x0000000c; // Rollover
    *((volatile uint32_t *)0x50d00990) = 0x000000c4; // Input frame size

    // Layer 1 group 0
    *((volatile uint32_t *)0x50100014) = 0x0002001f; // Rows
    *((volatile uint32_t *)0x50100094) = 0x0002001f; // Columns
    *((volatile uint32_t *)0x50100194) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50100214) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50100294) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50100314) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x50100414) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50100494) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x50100514) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x50100594) = 0x0000eba0; // Layer control
    *((volatile uint32_t *)0x50100a14) = 0x0001d800; // Layer control 2
    *((volatile uint32_t *)0x50100614) = 0x000001d8; // Mask offset and count
    *((volatile uint32_t *)0x50100694) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t *)0x50100714) = 0xfff0fff0; // Mask and processor enables

    // Layer 1 group 1
    *((volatile uint32_t *)0x50500014) = 0x0002001f; // Rows
    *((volatile uint32_t *)0x50500094) = 0x0002001f; // Columns
    *((volatile uint32_t *)0x50500194) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50500214) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50500294) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50500314) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x50500414) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50500494) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x50500514) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x50500594) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x50500a14) = 0x0001d800; // Layer control 2
    *((volatile uint32_t *)0x50500614) = 0x000001d8; // Mask offset and count
    *((volatile uint32_t *)0x50500694) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t *)0x50500714) = 0xffffffff; // Mask and processor enables

    // Layer 1 group 2
    *((volatile uint32_t *)0x50900014) = 0x0002001f; // Rows
    *((volatile uint32_t *)0x50900094) = 0x0002001f; // Columns
    *((volatile uint32_t *)0x50900194) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50900214) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50900294) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50900314) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x50900414) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50900494) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x50900514) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x50900594) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x50900a14) = 0x0001d800; // Layer control 2
    *((volatile uint32_t *)0x50900614) = 0x000001d8; // Mask offset and count
    *((volatile uint32_t *)0x50900694) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t *)0x50900714) = 0xffffffff; // Mask and processor enables

    // Layer 1 group 3
    *((volatile uint32_t *)0x50d00014) = 0x0002001f; // Rows
    *((volatile uint32_t *)0x50d00094) = 0x0002001f; // Columns
    *((volatile uint32_t *)0x50d00194) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50d00214) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50d00294) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50d00314) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x50d00414) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50d00494) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x50d00514) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x50d00594) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x50d00a14) = 0x0001d800; // Layer control 2
    *((volatile uint32_t *)0x50d00614) = 0x000001d8; // Mask offset and count
    *((volatile uint32_t *)0x50d00694) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t *)0x50d00714) = 0xffffffff; // Mask and processor enables

    // Layer 2 group 0
    *((volatile uint32_t *)0x50100018) = 0x00010011; // Rows
    *((volatile uint32_t *)0x50100098) = 0x00010011; // Columns
    *((volatile uint32_t *)0x50100198) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50100218) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50100298) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50100318) = 0x00002800; // SRAM write ptr
    *((volatile uint32_t *)0x50100418) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50100498) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x50100598) = 0x0000eba0; // Layer control
    *((volatile uint32_t *)0x50100a18) = 0x0001b800; // Layer control 2
    *((volatile uint32_t *)0x50100618) = 0x01e00398; // Mask offset and count
    *((volatile uint32_t *)0x50100698) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t *)0x50100718) = 0xfff0fff0; // Mask and processor enables

    // Layer 2 group 1
    *((volatile uint32_t *)0x50500018) = 0x00010011; // Rows
    *((volatile uint32_t *)0x50500098) = 0x00010011; // Columns
    *((volatile uint32_t *)0x50500198) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50500218) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50500298) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50500318) = 0x00002800; // SRAM write ptr
    *((volatile uint32_t *)0x50500418) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50500498) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x50500598) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x50500a18) = 0x0001b800; // Layer control 2
    *((volatile uint32_t *)0x50500618) = 0x01e00398; // Mask offset and count
    *((volatile uint32_t *)0x50500698) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t *)0x50500718) = 0xffffffff; // Mask and processor enables

    // Layer 2 group 2
    *((volatile uint32_t *)0x50900018) = 0x00010011; // Rows
    *((volatile uint32_t *)0x50900098) = 0x00010011; // Columns
    *((volatile uint32_t *)0x50900198) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50900218) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50900298) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50900318) = 0x00002800; // SRAM write ptr
    *((volatile uint32_t *)0x50900418) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50900498) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x50900598) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x50900a18) = 0x0001b800; // Layer control 2
    *((volatile uint32_t *)0x50900618) = 0x01e00398; // Mask offset and count
    *((volatile uint32_t *)0x50900698) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t *)0x50900718) = 0xffffffff; // Mask and processor enables

    // Layer 2 group 3
    *((volatile uint32_t *)0x50d00018) = 0x00010011; // Rows
    *((volatile uint32_t *)0x50d00098) = 0x00010011; // Columns
    *((volatile uint32_t *)0x50d00198) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50d00218) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50d00298) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50d00318) = 0x00002800; // SRAM write ptr
    *((volatile uint32_t *)0x50d00418) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50d00498) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x50d00598) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x50d00a18) = 0x0001b800; // Layer control 2
    *((volatile uint32_t *)0x50d00618) = 0x01e00398; // Mask offset and count
    *((volatile uint32_t *)0x50d00698) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t *)0x50d00718) = 0xffffffff; // Mask and processor enables

    // Layer 3 group 0
    *((volatile uint32_t *)0x5010001c) = 0x00010009; // Rows
    *((volatile uint32_t *)0x5010009c) = 0x00010009; // Columns
    *((volatile uint32_t *)0x5010019c) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x5010021c) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x5010029c) = 0x00000001; // Stride
    *((volatile uint32_t *)0x5010041c) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x5010049c) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5010051c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x5010059c) = 0x0000eaa0; // Layer control
    *((volatile uint32_t *)0x50100a1c) = 0x00005800; // Layer control 2
    *((volatile uint32_t *)0x5010061c) = 0x03a003f8; // Mask offset and count
    *((volatile uint32_t *)0x5010069c) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5010071c) = 0xfff0fff0; // Mask and processor enables

    // Layer 3 group 1
    *((volatile uint32_t *)0x5050001c) = 0x00010009; // Rows
    *((volatile uint32_t *)0x5050009c) = 0x00010009; // Columns
    *((volatile uint32_t *)0x5050019c) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x5050021c) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x5050029c) = 0x00000001; // Stride
    *((volatile uint32_t *)0x5050041c) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x5050049c) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5050051c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x5050059c) = 0x00000aa0; // Layer control
    *((volatile uint32_t *)0x50500a1c) = 0x00005800; // Layer control 2
    *((volatile uint32_t *)0x5050061c) = 0x03a003f8; // Mask offset and count
    *((volatile uint32_t *)0x5050069c) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5050071c) = 0xffffffff; // Mask and processor enables

    // Layer 3 group 2
    *((volatile uint32_t *)0x5090001c) = 0x00010009; // Rows
    *((volatile uint32_t *)0x5090009c) = 0x00010009; // Columns
    *((volatile uint32_t *)0x5090019c) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x5090021c) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x5090029c) = 0x00000001; // Stride
    *((volatile uint32_t *)0x5090041c) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x5090049c) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5090051c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x5090059c) = 0x00000aa0; // Layer control
    *((volatile uint32_t *)0x50900a1c) = 0x00005800; // Layer control 2
    *((volatile uint32_t *)0x5090061c) = 0x03a003f8; // Mask offset and count
    *((volatile uint32_t *)0x5090069c) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5090071c) = 0xffffffff; // Mask and processor enables

    // Layer 3 group 3
    *((volatile uint32_t *)0x50d0001c) = 0x00010009; // Rows
    *((volatile uint32_t *)0x50d0009c) = 0x00010009; // Columns
    *((volatile uint32_t *)0x50d0019c) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50d0021c) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50d0029c) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50d0041c) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50d0049c) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x50d0051c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x50d0059c) = 0x00000aa0; // Layer control
    *((volatile uint32_t *)0x50d00a1c) = 0x00005800; // Layer control 2
    *((volatile uint32_t *)0x50d0061c) = 0x03a003f8; // Mask offset and count
    *((volatile uint32_t *)0x50d0069c) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x50d0071c) = 0x0fff0fff; // Mask and processor enables

    // Layer 4 group 0
    *((volatile uint32_t *)0x50100320) = 0x00000400; // SRAM write ptr
    *((volatile uint32_t *)0x501003a0) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x50100420) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x501004a0) = 0x00000004; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x501005a0) = 0x00010920; // Layer control
    *((volatile uint32_t *)0x50100a20) = 0x0000480f; // Layer control 2
    *((volatile uint32_t *)0x50100620) = 0x240028f8; // Mask offset and count
    *((volatile uint32_t *)0x50100120) = 0x00000100; // 1D
    *((volatile uint32_t *)0x501007a0) = 0x00001000; // Post processing register
    *((volatile uint32_t *)0x50100720) = 0x0fff0fff; // Mask and processor enables

    // Layer 4 group 1
    *((volatile uint32_t *)0x50500320) = 0x00000400; // SRAM write ptr
    *((volatile uint32_t *)0x505003a0) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x50500420) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x505004a0) = 0x00000004; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x505005a0) = 0x00010920; // Layer control
    *((volatile uint32_t *)0x50500a20) = 0x0000480f; // Layer control 2
    *((volatile uint32_t *)0x50500620) = 0x240028f8; // Mask offset and count
    *((volatile uint32_t *)0x50500120) = 0x00000100; // 1D

    // Layer 4 group 2
    *((volatile uint32_t *)0x50900320) = 0x00000400; // SRAM write ptr
    *((volatile uint32_t *)0x509003a0) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x50900420) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x509004a0) = 0x00000004; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x509005a0) = 0x00010920; // Layer control
    *((volatile uint32_t *)0x50900a20) = 0x0000480f; // Layer control 2
    *((volatile uint32_t *)0x50900620) = 0x240028f8; // Mask offset and count
    *((volatile uint32_t *)0x50900120) = 0x00000100; // 1D

    // Layer 4 group 3
    *((volatile uint32_t *)0x50d00320) = 0x00000400; // SRAM write ptr
    *((volatile uint32_t *)0x50d003a0) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x50d00420) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50d004a0) = 0x00000004; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x50d005a0) = 0x00010920; // Layer control
    *((volatile uint32_t *)0x50d00a20) = 0x0000480f; // Layer control 2
    *((volatile uint32_t *)0x50d00620) = 0x240028f8; // Mask offset and count
    *((volatile uint32_t *)0x50d00120) = 0x00000100; // 1D

    *((volatile uint32_t *)0x400c0400) = 0x000000a0; // Fast FIFO control
    *((volatile uint32_t *)0x50100000) = 0x00d8c808; // Enable group 0
    *((volatile uint32_t *)0x50500000) = 0x0018c809; // Enable group 1
    *((volatile uint32_t *)0x50900000) = 0x0018c809; // Enable group 2
    *((volatile uint32_t *)0x50d00000) = 0x0018c809; // Enable group 3

#ifdef USE_SAMPLEDATA
    CNN_START; // Allow capture of processing time
    *((volatile uint32_t *)0x50100000) = 0x00d8c809; // Master enable group 0

    load_input(); // Load data input
#endif

    return 1;
}

// mnist-stream_0
// Expected output of layer 4
int cnn_check(void)
{
    int rv = 1;
    if ((*((volatile uint32_t *)0x50401000)) != 0xffff8c2f)
        return 0; // 0,0,0
    if ((*((volatile uint32_t *)0x50401004)) != 0xffff9c34)
        return 0; // 0,0,1
    if ((*((volatile uint32_t *)0x50401008)) != 0xffff94db)
        return 0; // 0,0,2
    if ((*((volatile uint32_t *)0x5040100c)) != 0xffff9b4d)
        return 0; // 0,0,3
    if ((*((volatile uint32_t *)0x50409000)) != 0xffff9ac6)
        return 0; // 0,0,4
    if ((*((volatile uint32_t *)0x50409004)) != 0xffff67dc)
        return 0; // 0,0,5
    if ((*((volatile uint32_t *)0x50409008)) != 0xfffef60d)
        return 0; // 0,0,6
    if ((*((volatile uint32_t *)0x5040900c)) != 0x0000999b)
        return 0; // 0,0,7
    if ((*((volatile uint32_t *)0x50411000)) != 0xffff86c1)
        return 0; // 0,0,8
    if ((*((volatile uint32_t *)0x50411004)) != 0xffff90a7)
        return 0; // 0,0,9
    return rv;
}

// Custom unload for this network:
// 32-bit data, shape: [10, 1, 1]
void cnn_unload(uint32_t *out_buf)
{
    volatile uint32_t *addr;

    addr = (volatile uint32_t *)0x50401000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x50409000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x50411000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
}

// Classification layer:
#define NUM_OUTPUTS 10
static int32_t ml_data[NUM_OUTPUTS];
static q15_t ml_softmax[NUM_OUTPUTS];

int softmax_layer(void)
{
    cnn_unload((uint32_t *)ml_data);
    softmax_q17p14_q15((const q31_t *)ml_data, NUM_OUTPUTS, ml_softmax);

    return 1;
}

/* **************************************************************************** */
void start_streaming(void)
{
    unsigned int fifocnt = 0, i, word_cnt = 0;
    uint32_t data, flags, rx_data_index = 0;
    uint32_t imgLen = IMAGE_SIZE_X * IMAGE_SIZE_Y * 3;

    uint8_t *rxbuf = rxdata;
    memset(rxdata, 0, imgLen);
    int8_t *ptr8;
    ptr8 = (int8_t *)camera_sampledata;
    __disable_irq();
    NVIC_DisableIRQ(PCIF_IRQn);
    MXC_PCIF_DisableInt(0xf);
    // Set camera fifo threshold
    MXC_PCIF_SetThreshold(8);
    camera_start_capture_image();

    CNN_START; // Allow capture of processing time
    *((volatile uint32_t *)0x50100000) = 0x00d8c809; // Master enable group 0

    while (!(MXC_PCIF->int_fl & MXC_F_CAMERAIF_INT_FL_FIFO_THRESH)) {
    } // Wait for camera fifo threshold
    while (fifocnt < (196)) {
        if (!(MXC_PCIF->int_fl & MXC_F_CAMERAIF_INT_FL_IMG_DONE)) {
            if (MXC_PCIF->int_fl & MXC_F_CAMERAIF_INT_FL_FIFO_THRESH) {
                for (i = 0; i < 8; i++) {
                    data = MXC_PCIF->fifo_data;
                    rxdata[rx_data_index + 0] = data >> 0;
                    rxdata[rx_data_index + 1] = data >> 8;
                    rxdata[rx_data_index + 2] = data >> 16;
                    *ptr8 = (~(((data & 0xff) + (data >> 8 & 0xff) + (data >> 16)) / 3)) -
                            128; // Convert rgb to monochrome signed data
                    ptr8++;
                    word_cnt++;
                    rx_data_index += 3;
                }
                flags = MXC_PCIF->int_fl;
                MXC_PCIF->int_fl = flags; // clear flags
            }
        } else {
            while (MXC_PCIF->int_fl & MXC_F_CAMERAIF_INT_FL_FIFO_NOT_EMPTY) {
                data = MXC_PCIF->fifo_data;
                MXC_PCIF->int_fl = MXC_F_CAMERAIF_INT_FL_FIFO_NOT_EMPTY;
                rxdata[rx_data_index + 0] = data >> 0;
                rxdata[rx_data_index + 1] = data >> 8;
                rxdata[rx_data_index + 2] = data >> 16;
                *ptr8 = (~(((data & 0xff) + (data >> 8 & 0xff) + (data >> 16)) / 3)) - 128;
                ptr8++;
                word_cnt++;
                rx_data_index += 3;
            }
        }
        // Load input data
        if ((((*((volatile uint32_t *)0x400c0404) & 2)) == 0) &&
            (word_cnt >= (fifocnt * 4 + 4))) { // Wait for FIFO
            *((volatile uint32_t *)0x400c0410) = camera_sampledata[fifocnt]; // Write FIFO
            fifocnt++;
        }
    }

    flags = MXC_PCIF->int_fl;
    MXC_PCIF->int_fl = flags;

    printf("Image captured.\n");
    // Use uart1 to display image
    utils_send_img_to_pc(rxbuf, imgLen, IMAGE_SIZE_X, IMAGE_SIZE_Y, camera_get_pixel_format());
}

/* **************************************************************************** */
int main(void)
{
    int i;
    int digs, tens;
    Debug_Init(); // Set up RISCV JTAG
    MXC_ICC_Enable(MXC_ICC1); // Enable cache

    // Reset all domains, restore power to CNN
    MXC_GCFR->reg3 = 0xf; // Reset
    MXC_GCFR->reg1 = 0xf; // Mask memory
    MXC_GCFR->reg0 = 0xf; // Power
    MXC_GCFR->reg2 = 0x0; // Iso
    MXC_GCFR->reg3 = 0x0; // Reset

    printf("\n*** CNN Test ***\n");

#ifndef USE_SAMPLEDATA
    int slaveAddress, id, ret = 0;
    // Initialize camera.
    printf("Init Camera.\n");

#ifdef BOARD_FTHR_REVA
    if (Camera_Power(POWER_ON) != E_NO_ERROR) {
        printf("Failed to turn on camera.\n");
        while (1) {}
    }
#endif

    camera_init(CAMERA_FREQ);
    // Set camera clock prescaler
    camera_write_reg(0x11, 2);
    // Obtain the I2C slave address of the camera.
    slaveAddress = camera_get_slave_address();
    printf("Camera I2C slave address is %02x\n", slaveAddress);

    // Obtain the product ID of the camera.
    ret = camera_get_product_id(&id);

    if (ret != STATUS_OK) {
        printf("Error returned from reading camera id. Error %d\n", ret);
        return -1;
    }

    printf("Camera Product ID is %04x\n", id);
    ret = camera_setup(IMAGE_SIZE_X, IMAGE_SIZE_Y, PIXFORMAT_RGB888, FIFO_THREE_BYTE, NO_DMA, 0);
    if (ret != STATUS_OK) {
        printf("Error returned from setting up camera. Error %d\n", ret);
        return -1;
    }

    if ((ret = MXC_UART_Init(MXC_UART1, CONSOLE_BAUD, MXC_UART_IBRO_CLK)) < E_NO_ERROR) {
        return ret;
    }

    rxdata = (uint8_t *)malloc(IMAGE_SIZE_X * IMAGE_SIZE_Y * 3);
    int frame = 0;
#else
    memcpy32(camera_sampledata, input_0, 196);
#endif

    while (1) {
        printf("********** Press PB1 to capture an image **********\r\n");
        while (!PB_Get(0)) {}
        if (!cnn_load())
            fail();
#ifndef USE_SAMPLEDATA
        // Capture a single camera frame.
        printf("\nCapture a camera frame %d\n", ++frame);
        start_streaming();
#endif
        MXC_TMR_SW_Start(MXC_TMR0);
        cnn_wait();

        if (!softmax_layer())
            fail();

        printf("\n*** PASS ***\n\n");
        printf("Time for CNN: %d us\n\n", cnn_time);

        printf("Classification results:\n");
        for (i = 0; i < NUM_OUTPUTS; i++) {
            digs = (1000 * ml_softmax[i] + 0x4000) >> 15;
            tens = digs % 10;
            digs = digs / 10;
            printf("[%7d] -> Class %d: %d.%d%%\n", ml_data[i], i, digs, tens);
        }
    }

    return 0;
}
