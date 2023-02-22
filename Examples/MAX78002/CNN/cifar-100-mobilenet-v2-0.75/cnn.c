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

// cifar-100-mobilenet-v2-0.75
// Created using ai8xize.py --test-dir sdk/Examples/MAX78002/CNN --prefix cifar-100-mobilenet-v2-0.75 --checkpoint-file trained/ai87-cifar100-mobilenet-v2-0.75-qat8-q.pth.tar --config-file networks/ai87-cifar100-mobilenet-v2-0.75.yaml --softmax --device MAX78002 --timer 0 --display-checkpoint --verbose --overwrite

// DO NOT EDIT - regenerate this file instead!

// Configuring 73 layers
// Input data: HWC
// Layer 0: 3x32x32, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 24x32x32 output
// Layer 1: 24x32x32, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 24x32x32 output
// Layer 2: 24x32x32, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 12x32x32 output
// Layer 3: 12x32x32, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, ReLU, 72x32x32 output
// Layer 4: 72x32x32, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 72x16x16 output
// Layer 5: 72x16x16, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 20x16x16 output
// Layer 6: 20x16x16, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, ReLU, 120x16x16 output
// Layer 7: 120x16x16, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 120x16x16 output
// Layer 8: 120x16x16, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 20x16x16 output
// Layer 9: 20x16x16, no pooling, no convolution, 20x16x16 output
// Layer 10: 2x20x16x16, no pooling, 2-element add, no convolution, 20x16x16 output
// Layer 11: 20x16x16, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, ReLU, 120x16x16 output
// Layer 12: 120x16x16, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 120x8x8 output
// Layer 13: 120x8x8, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 24x8x8 output
// Layer 14: 24x8x8, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, ReLU, 144x8x8 output
// Layer 15: 144x8x8, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 144x8x8 output
// Layer 16: 144x8x8, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 24x8x8 output
// Layer 17: 24x8x8, no pooling, no convolution, 24x8x8 output
// Layer 18: 2x24x8x8, no pooling, 2-element add, no convolution, 24x8x8 output
// Layer 19: 24x8x8, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, ReLU, 144x8x8 output
// Layer 20: 144x8x8, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 144x8x8 output
// Layer 21: 144x8x8, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 24x8x8 output
// Layer 22: 24x8x8, no pooling, no convolution, 24x8x8 output
// Layer 23: 2x24x8x8, no pooling, 2-element add, no convolution, 24x8x8 output
// Layer 24: 24x8x8, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, ReLU, 144x8x8 output
// Layer 25: 144x8x8, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 144x4x4 output
// Layer 26: 144x4x4, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 48x4x4 output
// Layer 27: 48x4x4, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, ReLU, 288x4x4 output
// Layer 28: 288x4x4, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 288x4x4 output
// Layer 29: 288x4x4, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 48x4x4 output
// Layer 30: 48x4x4, no pooling, no convolution, 48x4x4 output
// Layer 31: 2x48x4x4, no pooling, 2-element add, no convolution, 48x4x4 output
// Layer 32: 48x4x4, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, ReLU, 288x4x4 output
// Layer 33: 288x4x4, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 288x4x4 output
// Layer 34: 288x4x4, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 48x4x4 output
// Layer 35: 48x4x4, no pooling, no convolution, 48x4x4 output
// Layer 36: 2x48x4x4, no pooling, 2-element add, no convolution, 48x4x4 output
// Layer 37: 48x4x4, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, ReLU, 288x4x4 output
// Layer 38: 288x4x4, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 288x4x4 output
// Layer 39: 288x4x4, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 48x4x4 output
// Layer 40: 48x4x4, no pooling, no convolution, 48x4x4 output
// Layer 41: 2x48x4x4, no pooling, 2-element add, no convolution, 48x4x4 output
// Layer 42: 48x4x4, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, ReLU, 288x4x4 output
// Layer 43: 288x4x4, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 288x4x4 output
// Layer 44: 288x4x4, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 72x4x4 output
// Layer 45: 72x4x4, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, ReLU, 432x4x4 output
// Layer 46: 432x4x4, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 432x4x4 output
// Layer 47: 432x4x4, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 72x4x4 output
// Layer 48: 72x4x4, no pooling, no convolution, 72x4x4 output
// Layer 49: 2x72x4x4, no pooling, 2-element add, no convolution, 72x4x4 output
// Layer 50: 72x4x4, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, ReLU, 432x4x4 output
// Layer 51: 432x4x4, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 432x4x4 output
// Layer 52: 432x4x4, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 72x4x4 output
// Layer 53: 72x4x4, no pooling, no convolution, 72x4x4 output
// Layer 54: 2x72x4x4, no pooling, 2-element add, no convolution, 72x4x4 output
// Layer 55: 72x4x4, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, ReLU, 432x4x4 output
// Layer 56: 432x4x4, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 432x4x4 output
// Layer 57: 432x4x4, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 120x4x4 output
// Layer 58: 120x4x4, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, ReLU, 720x4x4 output
// Layer 59: 720x4x4, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 720x4x4 output
// Layer 60: 720x4x4, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 120x4x4 output
// Layer 61: 120x4x4, no pooling, no convolution, 120x4x4 output
// Layer 62: 2x120x4x4, no pooling, 2-element add, no convolution, 120x4x4 output
// Layer 63: 120x4x4, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, ReLU, 720x4x4 output
// Layer 64: 720x4x4, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 720x4x4 output
// Layer 65: 720x4x4, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 120x4x4 output
// Layer 66: 120x4x4, no pooling, no convolution, 120x4x4 output
// Layer 67: 2x120x4x4, no pooling, 2-element add, no convolution, 120x4x4 output
// Layer 68: 120x4x4, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, ReLU, 720x4x4 output
// Layer 69: 720x4x4, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 720x4x4 output
// Layer 70: 720x4x4, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 240x4x4 output
// Layer 71: 240x4x4, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, ReLU, 960x4x4 output
// Layer 72: 960x4x4, avg pool 4x4 with stride 4/4, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 100x1x1 output

#include <assert.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "mxc.h"
#include "gcfr_regs.h"
#include "cnn.h"
#include "weights.h"

void CNN_ISR(void)
{
    // Acknowledge interrupt to all quadrants
    *((volatile uint32_t *)0x51000000) &= ~((1 << 12) | 1);
    *((volatile uint32_t *)0x52000000) &= ~((1 << 12) | 1);
    *((volatile uint32_t *)0x53000000) &= ~((1 << 12) | 1);
    *((volatile uint32_t *)0x54000000) &= ~((1 << 12) | 1);

    CNN_COMPLETE; // Signal that processing is complete
#ifdef CNN_INFERENCE_TIMER
    cnn_time = MXC_TMR_SW_Stop(CNN_INFERENCE_TIMER);
#else
    cnn_time = 1;
#endif
}

int cnn_continue(void)
{
    cnn_time = 0;

    *((volatile uint32_t *)0x51000000) |= 1; // Re-enable quadrant 0

    return CNN_OK;
}

int cnn_stop(void)
{
    *((volatile uint32_t *)0x51000000) &= ~1; // Disable quadrant 0

    return CNN_OK;
}

void memcpy32(uint32_t *dst, const uint32_t *src, int n)
{
    while (n-- > 0) {
        *dst++ = *src++;
    }
}

static const uint32_t kernels[] = KERNELS;

int cnn_load_weights(void)
{
    uint32_t len;
    volatile uint32_t *addr;
    const uint32_t *ptr = kernels;

    while ((addr = (volatile uint32_t *)*ptr++) != 0) {
        *((volatile uint8_t *)((uint32_t)addr | 1)) = 0x01; // Set address
        len = *ptr++;
        while (len-- > 0) *addr++ = *ptr++;
    }

    return CNN_OK;
}

static const uint8_t bias_0[] = BIAS_0;
static const uint8_t bias_1[] = BIAS_1;
static const uint8_t bias_2[] = BIAS_2;
static const uint8_t bias_3[] = BIAS_3;

static void memcpy_8to32(uint32_t *dst, const uint8_t *src, int n)
{
    while (n-- > 0) {
        *dst++ = *src++;
    }
}

int cnn_load_bias(void)
{
    memcpy_8to32((uint32_t *)0x51180000, bias_0, sizeof(uint8_t) * 1632);
    memcpy_8to32((uint32_t *)0x52180000, bias_1, sizeof(uint8_t) * 1628);
    memcpy_8to32((uint32_t *)0x53180000, bias_2, sizeof(uint8_t) * 1628);
    memcpy_8to32((uint32_t *)0x54180000, bias_3, sizeof(uint8_t) * 1620);

    return CNN_OK;
}

int cnn_init(void)
{
    *((volatile uint32_t *)0x51000000) = 0x00000008; // Enable clocks
    *((volatile uint32_t *)0x52000000) = 0x00000008; // Enable clocks
    *((volatile uint32_t *)0x53000000) = 0x00000008; // Enable clocks
    *((volatile uint32_t *)0x54000000) = 0x00000008; // Enable clocks
    *((volatile uint32_t *)0x50001000) = 0x00000000; // AON control
    *((volatile uint32_t *)0x51000004) = 0x0000040e; // SRAM control
    *((volatile uint32_t *)0x52000004) = 0x0000040e; // SRAM control
    *((volatile uint32_t *)0x53000004) = 0x0000040e; // SRAM control
    *((volatile uint32_t *)0x54000004) = 0x0000040e; // SRAM control
    *((volatile uint32_t *)0x5100000c) = 0x00001c80; // Clear registers
    *((volatile uint32_t *)0x5200000c) = 0x00001c80; // Clear registers
    *((volatile uint32_t *)0x5300000c) = 0x00001c80; // Clear registers
    *((volatile uint32_t *)0x5400000c) = 0x00001c80; // Clear registers
    while ((*((volatile uint32_t *)0x5100000c) & 0x2000000) != 0x2000000) {}
    // Wait for clear
    while ((*((volatile uint32_t *)0x5200000c) & 0x2000000) != 0x2000000) {}
    // Wait for clear
    while ((*((volatile uint32_t *)0x5300000c) & 0x2000000) != 0x2000000) {}
    // Wait for clear
    while ((*((volatile uint32_t *)0x5400000c) & 0x2000000) != 0x2000000) {}
    // Wait for clear
    *((volatile uint32_t *)0x5100000c) = 0x00000000; // Reset BIST
    *((volatile uint32_t *)0x5200000c) = 0x00000000; // Reset BIST
    *((volatile uint32_t *)0x5300000c) = 0x00000000; // Reset BIST
    *((volatile uint32_t *)0x5400000c) = 0x00000000; // Reset BIST

    *((volatile uint32_t *)0x51000000) = 0x00100008; // Stop SM
    *((volatile uint32_t *)0x51000008) = 0x00000048; // Layer count
    *((volatile uint32_t *)0x52000000) = 0x00100008; // Stop SM
    *((volatile uint32_t *)0x52000008) = 0x00000048; // Layer count
    *((volatile uint32_t *)0x53000000) = 0x00100008; // Stop SM
    *((volatile uint32_t *)0x53000008) = 0x00000048; // Layer count
    *((volatile uint32_t *)0x54000000) = 0x00100008; // Stop SM
    *((volatile uint32_t *)0x54000008) = 0x00000048; // Layer count

    return CNN_OK;
}

int cnn_configure(void)
{
    // Layer 0 quadrant 0
    *((volatile uint32_t *)0x51100004) = 0x0001801f; // Rows
    *((volatile uint32_t *)0x51100008) = 0x0001801f; // Columns
    *((volatile uint32_t *)0x51100018) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5110001c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x51100024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51100030) = 0x00888b20; // Layer control
    *((volatile uint32_t *)0x51100034) = 0x000b8000; // Layer control 2
    *((volatile uint32_t *)0x51100038) = 0x000000b8; // Mask count
    *((volatile uint32_t *)0x51100040) = 0x00000017; // Output channel count
    *((volatile uint32_t *)0x51100044) = 0x0000001f; // TRAM ptr max
    *((volatile uint32_t *)0x51100048) = 0x00070007; // Mask and processor enables

    // Layer 0 quadrant 1
    *((volatile uint32_t *)0x52100004) = 0x0001801f; // Rows
    *((volatile uint32_t *)0x52100008) = 0x0001801f; // Columns
    *((volatile uint32_t *)0x52100018) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5210001c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x52100024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52100030) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x52100034) = 0x000b8000; // Layer control 2
    *((volatile uint32_t *)0x52100038) = 0x000000b8; // Mask count
    *((volatile uint32_t *)0x52100040) = 0x00000017; // Output channel count
    *((volatile uint32_t *)0x52100044) = 0x0000001f; // TRAM ptr max

    // Layer 0 quadrant 2
    *((volatile uint32_t *)0x53100004) = 0x0001801f; // Rows
    *((volatile uint32_t *)0x53100008) = 0x0001801f; // Columns
    *((volatile uint32_t *)0x53100018) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5310001c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x53100024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53100030) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x53100034) = 0x000b8000; // Layer control 2
    *((volatile uint32_t *)0x53100038) = 0x000000b8; // Mask count
    *((volatile uint32_t *)0x53100040) = 0x00000017; // Output channel count
    *((volatile uint32_t *)0x53100044) = 0x0000001f; // TRAM ptr max

    // Layer 0 quadrant 3
    *((volatile uint32_t *)0x54100004) = 0x0001801f; // Rows
    *((volatile uint32_t *)0x54100008) = 0x0001801f; // Columns
    *((volatile uint32_t *)0x54100018) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5410001c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x54100024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54100030) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x54100034) = 0x000b8000; // Layer control 2
    *((volatile uint32_t *)0x54100038) = 0x000000b8; // Mask count
    *((volatile uint32_t *)0x54100040) = 0x00000017; // Output channel count
    *((volatile uint32_t *)0x54100044) = 0x0000001f; // TRAM ptr max
    *((volatile uint32_t *)0x5410004c) = 0x00001618; // Post processing register

    // Layer 1 quadrant 0
    *((volatile uint32_t *)0x51100104) = 0x0001801f; // Rows
    *((volatile uint32_t *)0x51100108) = 0x0001801f; // Columns
    *((volatile uint32_t *)0x51100118) = 0x00000010; // Stride
    *((volatile uint32_t *)0x51100124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5110012c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51100130) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x51100134) = 0x000b8000; // Layer control 2
    *((volatile uint32_t *)0x51100138) = 0x000000c0; // Mask count
    *((volatile uint32_t *)0x5110013c) = 0x000000c0; // Mask offset
    *((volatile uint32_t *)0x51100140) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5110010c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x51100144) = 0x0000001f; // TRAM ptr max
    *((volatile uint32_t *)0x5110014c) = 0x41004000; // Post processing register
    *((volatile uint32_t *)0x51100148) = 0xffffffff; // Mask and processor enables

    // Layer 1 quadrant 1
    *((volatile uint32_t *)0x52100104) = 0x0001801f; // Rows
    *((volatile uint32_t *)0x52100108) = 0x0001801f; // Columns
    *((volatile uint32_t *)0x52100118) = 0x00000010; // Stride
    *((volatile uint32_t *)0x52100124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5210012c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52100130) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x52100134) = 0x000b8000; // Layer control 2
    *((volatile uint32_t *)0x52100138) = 0x000000c0; // Mask count
    *((volatile uint32_t *)0x5210013c) = 0x000000c0; // Mask offset
    *((volatile uint32_t *)0x52100140) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5210010c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x52100144) = 0x0000001f; // TRAM ptr max
    *((volatile uint32_t *)0x5210014c) = 0x41004000; // Post processing register
    *((volatile uint32_t *)0x52100148) = 0x00ff00ff; // Mask and processor enables

    // Layer 1 quadrant 2
    *((volatile uint32_t *)0x53100104) = 0x0001801f; // Rows
    *((volatile uint32_t *)0x53100108) = 0x0001801f; // Columns
    *((volatile uint32_t *)0x53100118) = 0x00000010; // Stride
    *((volatile uint32_t *)0x53100124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5310012c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53100130) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x53100134) = 0x000b8000; // Layer control 2
    *((volatile uint32_t *)0x53100138) = 0x000000c0; // Mask count
    *((volatile uint32_t *)0x5310013c) = 0x000000c0; // Mask offset
    *((volatile uint32_t *)0x53100140) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5310010c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x53100144) = 0x0000001f; // TRAM ptr max
    *((volatile uint32_t *)0x5310014c) = 0x41004000; // Post processing register

    // Layer 1 quadrant 3
    *((volatile uint32_t *)0x54100104) = 0x0001801f; // Rows
    *((volatile uint32_t *)0x54100108) = 0x0001801f; // Columns
    *((volatile uint32_t *)0x54100118) = 0x00000010; // Stride
    *((volatile uint32_t *)0x54100124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5410012c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54100130) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x54100134) = 0x000b8000; // Layer control 2
    *((volatile uint32_t *)0x54100138) = 0x000000c0; // Mask count
    *((volatile uint32_t *)0x5410013c) = 0x000000c0; // Mask offset
    *((volatile uint32_t *)0x54100140) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5410010c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x54100144) = 0x0000001f; // TRAM ptr max
    *((volatile uint32_t *)0x5410014c) = 0x41004000; // Post processing register

    // Layer 2 quadrant 0
    *((volatile uint32_t *)0x51100204) = 0x0001001f; // Rows
    *((volatile uint32_t *)0x51100208) = 0x0001001f; // Columns
    *((volatile uint32_t *)0x51100218) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5110021c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x51100220) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51100224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51100230) = 0x0000a920; // Layer control
    *((volatile uint32_t *)0x51100234) = 0x00058000; // Layer control 2
    *((volatile uint32_t *)0x51100238) = 0x00000838; // Mask count
    *((volatile uint32_t *)0x5110023c) = 0x000007e0; // Mask offset
    *((volatile uint32_t *)0x51100240) = 0x0000000b; // Output channel count
    *((volatile uint32_t *)0x5110020c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5110024c) = 0x00002000; // Post processing register
    *((volatile uint32_t *)0x51100248) = 0xffffffff; // Mask and processor enables

    // Layer 2 quadrant 1
    *((volatile uint32_t *)0x52100204) = 0x0001001f; // Rows
    *((volatile uint32_t *)0x52100208) = 0x0001001f; // Columns
    *((volatile uint32_t *)0x52100218) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5210021c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x52100220) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52100224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52100230) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52100234) = 0x00058000; // Layer control 2
    *((volatile uint32_t *)0x52100238) = 0x00000838; // Mask count
    *((volatile uint32_t *)0x5210023c) = 0x000007e0; // Mask offset
    *((volatile uint32_t *)0x52100240) = 0x0000000b; // Output channel count
    *((volatile uint32_t *)0x5210020c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210024c) = 0x00002000; // Post processing register
    *((volatile uint32_t *)0x52100248) = 0x00ff00ff; // Mask and processor enables

    // Layer 2 quadrant 2
    *((volatile uint32_t *)0x53100204) = 0x0001001f; // Rows
    *((volatile uint32_t *)0x53100208) = 0x0001001f; // Columns
    *((volatile uint32_t *)0x53100218) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5310021c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x53100220) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53100224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53100230) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53100234) = 0x00058000; // Layer control 2
    *((volatile uint32_t *)0x53100238) = 0x00000838; // Mask count
    *((volatile uint32_t *)0x5310023c) = 0x000007e0; // Mask offset
    *((volatile uint32_t *)0x53100240) = 0x0000000b; // Output channel count
    *((volatile uint32_t *)0x5310020c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310024c) = 0x00002000; // Post processing register

    // Layer 2 quadrant 3
    *((volatile uint32_t *)0x54100204) = 0x0001001f; // Rows
    *((volatile uint32_t *)0x54100208) = 0x0001001f; // Columns
    *((volatile uint32_t *)0x54100218) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5410021c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x54100220) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54100224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54100230) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54100234) = 0x00058000; // Layer control 2
    *((volatile uint32_t *)0x54100238) = 0x00000838; // Mask count
    *((volatile uint32_t *)0x5410023c) = 0x000007e0; // Mask offset
    *((volatile uint32_t *)0x54100240) = 0x0000000b; // Output channel count
    *((volatile uint32_t *)0x5410020c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410024c) = 0x00003648; // Post processing register

    // Layer 3 quadrant 0
    *((volatile uint32_t *)0x51100304) = 0x0001001f; // Rows
    *((volatile uint32_t *)0x51100308) = 0x0001001f; // Columns
    *((volatile uint32_t *)0x51100318) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5110031c) = 0x00018800; // SRAM write ptr
    *((volatile uint32_t *)0x51100320) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51100324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51100328) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110032c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51100330) = 0x00002b20; // Layer control
    *((volatile uint32_t *)0x51100334) = 0x00118010; // Layer control 2
    *((volatile uint32_t *)0x51100338) = 0x00000b38; // Mask count
    *((volatile uint32_t *)0x5110033c) = 0x00000900; // Mask offset
    *((volatile uint32_t *)0x51100340) = 0x00000047; // Output channel count
    *((volatile uint32_t *)0x5110030c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51100348) = 0x0fff0fff; // Mask and processor enables

    // Layer 3 quadrant 1
    *((volatile uint32_t *)0x52100304) = 0x0001001f; // Rows
    *((volatile uint32_t *)0x52100308) = 0x0001001f; // Columns
    *((volatile uint32_t *)0x52100318) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5210031c) = 0x00018800; // SRAM write ptr
    *((volatile uint32_t *)0x52100320) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52100324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52100328) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210032c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52100330) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x52100334) = 0x00118010; // Layer control 2
    *((volatile uint32_t *)0x52100338) = 0x00000b38; // Mask count
    *((volatile uint32_t *)0x5210033c) = 0x00000900; // Mask offset
    *((volatile uint32_t *)0x52100340) = 0x00000047; // Output channel count
    *((volatile uint32_t *)0x5210030c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210034c) = 0x00001588; // Post processing register

    // Layer 3 quadrant 2
    *((volatile uint32_t *)0x53100304) = 0x0001001f; // Rows
    *((volatile uint32_t *)0x53100308) = 0x0001001f; // Columns
    *((volatile uint32_t *)0x53100318) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5310031c) = 0x00018800; // SRAM write ptr
    *((volatile uint32_t *)0x53100320) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53100324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53100328) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310032c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53100330) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x53100334) = 0x00118010; // Layer control 2
    *((volatile uint32_t *)0x53100338) = 0x00000b38; // Mask count
    *((volatile uint32_t *)0x5310033c) = 0x00000900; // Mask offset
    *((volatile uint32_t *)0x53100340) = 0x00000047; // Output channel count
    *((volatile uint32_t *)0x5310030c) = 0x00000100; // 1D

    // Layer 3 quadrant 3
    *((volatile uint32_t *)0x54100304) = 0x0001001f; // Rows
    *((volatile uint32_t *)0x54100308) = 0x0001001f; // Columns
    *((volatile uint32_t *)0x54100318) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5410031c) = 0x00018800; // SRAM write ptr
    *((volatile uint32_t *)0x54100320) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54100324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54100328) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410032c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54100330) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x54100334) = 0x00118010; // Layer control 2
    *((volatile uint32_t *)0x54100338) = 0x00000b38; // Mask count
    *((volatile uint32_t *)0x5410033c) = 0x00000900; // Mask offset
    *((volatile uint32_t *)0x54100340) = 0x00000047; // Output channel count
    *((volatile uint32_t *)0x5410030c) = 0x00000100; // 1D

    // Layer 4 quadrant 0
    *((volatile uint32_t *)0x51100404) = 0x0044801e; // Rows
    *((volatile uint32_t *)0x51100408) = 0x0002801e; // Columns
    *((volatile uint32_t *)0x51100410) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x51100414) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x51100418) = 0x00000041; // Stride
    *((volatile uint32_t *)0x5110041c) = 0x00018000; // SRAM write ptr
    *((volatile uint32_t *)0x51100424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51100428) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110042c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x51100430) = 0x20880ba0; // Layer control
    *((volatile uint32_t *)0x51100434) = 0x00118011; // Layer control 2
    *((volatile uint32_t *)0x51100438) = 0x00000008; // Mask count
    *((volatile uint32_t *)0x51100440) = 0x00000007; // Output channel count
    *((volatile uint32_t *)0x5110040c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x51100444) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t *)0x5110044c) = 0x41002000; // Post processing register
    *((volatile uint32_t *)0x51100448) = 0xf000f000; // Mask and processor enables

    // Layer 4 quadrant 1
    *((volatile uint32_t *)0x52100404) = 0x0044801e; // Rows
    *((volatile uint32_t *)0x52100408) = 0x0002801e; // Columns
    *((volatile uint32_t *)0x52100410) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x52100414) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x52100418) = 0x00000041; // Stride
    *((volatile uint32_t *)0x5210041c) = 0x00018000; // SRAM write ptr
    *((volatile uint32_t *)0x52100424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52100428) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210042c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x52100430) = 0x20880ba0; // Layer control
    *((volatile uint32_t *)0x52100434) = 0x00118011; // Layer control 2
    *((volatile uint32_t *)0x52100438) = 0x00000008; // Mask count
    *((volatile uint32_t *)0x52100440) = 0x00000007; // Output channel count
    *((volatile uint32_t *)0x5210040c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x52100444) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t *)0x5210044c) = 0x41002000; // Post processing register
    *((volatile uint32_t *)0x52100448) = 0xffffffff; // Mask and processor enables

    // Layer 4 quadrant 2
    *((volatile uint32_t *)0x53100404) = 0x0044801e; // Rows
    *((volatile uint32_t *)0x53100408) = 0x0002801e; // Columns
    *((volatile uint32_t *)0x53100410) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x53100414) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x53100418) = 0x00000041; // Stride
    *((volatile uint32_t *)0x5310041c) = 0x00018000; // SRAM write ptr
    *((volatile uint32_t *)0x53100424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53100428) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310042c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x53100430) = 0x20880ba0; // Layer control
    *((volatile uint32_t *)0x53100434) = 0x00118011; // Layer control 2
    *((volatile uint32_t *)0x53100438) = 0x00000008; // Mask count
    *((volatile uint32_t *)0x53100440) = 0x00000007; // Output channel count
    *((volatile uint32_t *)0x5310040c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x53100444) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t *)0x5310044c) = 0x41002000; // Post processing register
    *((volatile uint32_t *)0x53100448) = 0xffffffff; // Mask and processor enables

    // Layer 4 quadrant 3
    *((volatile uint32_t *)0x54100404) = 0x0044801e; // Rows
    *((volatile uint32_t *)0x54100408) = 0x0002801e; // Columns
    *((volatile uint32_t *)0x54100410) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x54100414) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x54100418) = 0x00000041; // Stride
    *((volatile uint32_t *)0x5410041c) = 0x00018000; // SRAM write ptr
    *((volatile uint32_t *)0x54100424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54100428) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410042c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x54100430) = 0x20880ba0; // Layer control
    *((volatile uint32_t *)0x54100434) = 0x00118011; // Layer control 2
    *((volatile uint32_t *)0x54100438) = 0x00000008; // Mask count
    *((volatile uint32_t *)0x54100440) = 0x00000007; // Output channel count
    *((volatile uint32_t *)0x5410040c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x54100444) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t *)0x5410044c) = 0x41002000; // Post processing register

    // Layer 5 quadrant 0
    *((volatile uint32_t *)0x51100504) = 0x0002000f; // Rows
    *((volatile uint32_t *)0x51100508) = 0x0001000f; // Columns
    *((volatile uint32_t *)0x51100518) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5110051c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x51100520) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51100524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51100530) = 0x00006920; // Layer control
    *((volatile uint32_t *)0x51100534) = 0x00098001; // Layer control 2
    *((volatile uint32_t *)0x51100538) = 0x00000258; // Mask count
    *((volatile uint32_t *)0x5110053c) = 0x00000120; // Mask offset
    *((volatile uint32_t *)0x51100540) = 0x00000027; // Output channel count
    *((volatile uint32_t *)0x5110050c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51100548) = 0xf000f000; // Mask and processor enables

    // Layer 5 quadrant 1
    *((volatile uint32_t *)0x52100504) = 0x0002000f; // Rows
    *((volatile uint32_t *)0x52100508) = 0x0001000f; // Columns
    *((volatile uint32_t *)0x52100518) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5210051c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x52100520) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52100524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52100530) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52100534) = 0x00098001; // Layer control 2
    *((volatile uint32_t *)0x52100538) = 0x00000258; // Mask count
    *((volatile uint32_t *)0x5210053c) = 0x00000120; // Mask offset
    *((volatile uint32_t *)0x52100540) = 0x00000027; // Output channel count
    *((volatile uint32_t *)0x5210050c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210054c) = 0x00001648; // Post processing register
    *((volatile uint32_t *)0x52100548) = 0xffffffff; // Mask and processor enables

    // Layer 5 quadrant 2
    *((volatile uint32_t *)0x53100504) = 0x0002000f; // Rows
    *((volatile uint32_t *)0x53100508) = 0x0001000f; // Columns
    *((volatile uint32_t *)0x53100518) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5310051c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x53100520) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53100524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53100530) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53100534) = 0x00098001; // Layer control 2
    *((volatile uint32_t *)0x53100538) = 0x00000258; // Mask count
    *((volatile uint32_t *)0x5310053c) = 0x00000120; // Mask offset
    *((volatile uint32_t *)0x53100540) = 0x00000027; // Output channel count
    *((volatile uint32_t *)0x5310050c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x53100548) = 0xffffffff; // Mask and processor enables

    // Layer 5 quadrant 3
    *((volatile uint32_t *)0x54100504) = 0x0002000f; // Rows
    *((volatile uint32_t *)0x54100508) = 0x0001000f; // Columns
    *((volatile uint32_t *)0x54100518) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5410051c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x54100520) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54100524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54100530) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54100534) = 0x00098001; // Layer control 2
    *((volatile uint32_t *)0x54100538) = 0x00000258; // Mask count
    *((volatile uint32_t *)0x5410053c) = 0x00000120; // Mask offset
    *((volatile uint32_t *)0x54100540) = 0x00000027; // Output channel count
    *((volatile uint32_t *)0x5410050c) = 0x00000100; // 1D

    // Layer 6 quadrant 0
    *((volatile uint32_t *)0x51100604) = 0x0001000f; // Rows
    *((volatile uint32_t *)0x51100608) = 0x0001000f; // Columns
    *((volatile uint32_t *)0x51100618) = 0x00000010; // Stride
    *((volatile uint32_t *)0x51100620) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51100624) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51100628) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110062c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51100630) = 0x00002b20; // Layer control
    *((volatile uint32_t *)0x51100634) = 0x001d8010; // Layer control 2
    *((volatile uint32_t *)0x51100638) = 0x00000ef8; // Mask count
    *((volatile uint32_t *)0x5110063c) = 0x00000b40; // Mask offset
    *((volatile uint32_t *)0x51100640) = 0x00000077; // Output channel count
    *((volatile uint32_t *)0x5110060c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5110064c) = 0x00023510; // Post processing register
    *((volatile uint32_t *)0x51100648) = 0xffffffff; // Mask and processor enables

    // Layer 6 quadrant 1
    *((volatile uint32_t *)0x52100604) = 0x0001000f; // Rows
    *((volatile uint32_t *)0x52100608) = 0x0001000f; // Columns
    *((volatile uint32_t *)0x52100618) = 0x00000010; // Stride
    *((volatile uint32_t *)0x52100620) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52100624) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52100628) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210062c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52100630) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x52100634) = 0x001d8010; // Layer control 2
    *((volatile uint32_t *)0x52100638) = 0x00000ef8; // Mask count
    *((volatile uint32_t *)0x5210063c) = 0x00000b40; // Mask offset
    *((volatile uint32_t *)0x52100640) = 0x00000077; // Output channel count
    *((volatile uint32_t *)0x5210060c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210064c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x52100648) = 0x000f000f; // Mask and processor enables

    // Layer 6 quadrant 2
    *((volatile uint32_t *)0x53100604) = 0x0001000f; // Rows
    *((volatile uint32_t *)0x53100608) = 0x0001000f; // Columns
    *((volatile uint32_t *)0x53100618) = 0x00000010; // Stride
    *((volatile uint32_t *)0x53100620) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53100624) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53100628) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310062c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53100630) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x53100634) = 0x001d8010; // Layer control 2
    *((volatile uint32_t *)0x53100638) = 0x00000ef8; // Mask count
    *((volatile uint32_t *)0x5310063c) = 0x00000b40; // Mask offset
    *((volatile uint32_t *)0x53100640) = 0x00000077; // Output channel count
    *((volatile uint32_t *)0x5310060c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310064c) = 0x00022000; // Post processing register

    // Layer 6 quadrant 3
    *((volatile uint32_t *)0x54100604) = 0x0001000f; // Rows
    *((volatile uint32_t *)0x54100608) = 0x0001000f; // Columns
    *((volatile uint32_t *)0x54100618) = 0x00000010; // Stride
    *((volatile uint32_t *)0x54100620) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54100624) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54100628) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410062c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54100630) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x54100634) = 0x001d8010; // Layer control 2
    *((volatile uint32_t *)0x54100638) = 0x00000ef8; // Mask count
    *((volatile uint32_t *)0x5410063c) = 0x00000b40; // Mask offset
    *((volatile uint32_t *)0x54100640) = 0x00000077; // Output channel count
    *((volatile uint32_t *)0x5410060c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410064c) = 0x00022000; // Post processing register

    // Layer 7 quadrant 0
    *((volatile uint32_t *)0x51100704) = 0x0002800f; // Rows
    *((volatile uint32_t *)0x51100708) = 0x0001800f; // Columns
    *((volatile uint32_t *)0x51100718) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5110071c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x51100724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51100728) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51100730) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x51100734) = 0x001d8011; // Layer control 2
    *((volatile uint32_t *)0x51100738) = 0x000001c8; // Mask count
    *((volatile uint32_t *)0x5110073c) = 0x000001c0; // Mask offset
    *((volatile uint32_t *)0x51100740) = 0x00000007; // Output channel count
    *((volatile uint32_t *)0x5110070c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x51100744) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t *)0x5110074c) = 0x41002000; // Post processing register
    *((volatile uint32_t *)0x51100748) = 0xffffffff; // Mask and processor enables

    // Layer 7 quadrant 1
    *((volatile uint32_t *)0x52100704) = 0x0002800f; // Rows
    *((volatile uint32_t *)0x52100708) = 0x0001800f; // Columns
    *((volatile uint32_t *)0x52100718) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5210071c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x52100724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52100728) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52100730) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x52100734) = 0x001d8011; // Layer control 2
    *((volatile uint32_t *)0x52100738) = 0x000001c8; // Mask count
    *((volatile uint32_t *)0x5210073c) = 0x000001c0; // Mask offset
    *((volatile uint32_t *)0x52100740) = 0x00000007; // Output channel count
    *((volatile uint32_t *)0x5210070c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x52100744) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t *)0x5210074c) = 0x41002000; // Post processing register
    *((volatile uint32_t *)0x52100748) = 0xffffffff; // Mask and processor enables

    // Layer 7 quadrant 2
    *((volatile uint32_t *)0x53100704) = 0x0002800f; // Rows
    *((volatile uint32_t *)0x53100708) = 0x0001800f; // Columns
    *((volatile uint32_t *)0x53100718) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5310071c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x53100724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53100728) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53100730) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x53100734) = 0x001d8011; // Layer control 2
    *((volatile uint32_t *)0x53100738) = 0x000001c8; // Mask count
    *((volatile uint32_t *)0x5310073c) = 0x000001c0; // Mask offset
    *((volatile uint32_t *)0x53100740) = 0x00000007; // Output channel count
    *((volatile uint32_t *)0x5310070c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x53100744) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t *)0x5310074c) = 0x41002000; // Post processing register
    *((volatile uint32_t *)0x53100748) = 0xffffffff; // Mask and processor enables

    // Layer 7 quadrant 3
    *((volatile uint32_t *)0x54100704) = 0x0002800f; // Rows
    *((volatile uint32_t *)0x54100708) = 0x0001800f; // Columns
    *((volatile uint32_t *)0x54100718) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5410071c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x54100724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54100728) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54100730) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x54100734) = 0x001d8011; // Layer control 2
    *((volatile uint32_t *)0x54100738) = 0x000001c8; // Mask count
    *((volatile uint32_t *)0x5410073c) = 0x000001c0; // Mask offset
    *((volatile uint32_t *)0x54100740) = 0x00000007; // Output channel count
    *((volatile uint32_t *)0x5410070c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x54100744) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t *)0x5410074c) = 0x41002000; // Post processing register
    *((volatile uint32_t *)0x54100748) = 0x0fff0fff; // Mask and processor enables

    // Layer 8 quadrant 0
    *((volatile uint32_t *)0x51100804) = 0x0002000f; // Rows
    *((volatile uint32_t *)0x51100808) = 0x0001000f; // Columns
    *((volatile uint32_t *)0x51100818) = 0x00000020; // Stride
    *((volatile uint32_t *)0x51100820) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51100824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5110082c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x51100830) = 0x0000e920; // Layer control
    *((volatile uint32_t *)0x51100834) = 0x00098011; // Layer control 2
    *((volatile uint32_t *)0x51100838) = 0x00001218; // Mask count
    *((volatile uint32_t *)0x5110083c) = 0x000010e0; // Mask offset
    *((volatile uint32_t *)0x51100840) = 0x00000027; // Output channel count
    *((volatile uint32_t *)0x5110080c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5110084c) = 0x00002000; // Post processing register
    *((volatile uint32_t *)0x51100848) = 0xffffffff; // Mask and processor enables

    // Layer 8 quadrant 1
    *((volatile uint32_t *)0x52100804) = 0x0002000f; // Rows
    *((volatile uint32_t *)0x52100808) = 0x0001000f; // Columns
    *((volatile uint32_t *)0x52100818) = 0x00000020; // Stride
    *((volatile uint32_t *)0x52100820) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52100824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5210082c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x52100830) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52100834) = 0x00098011; // Layer control 2
    *((volatile uint32_t *)0x52100838) = 0x00001218; // Mask count
    *((volatile uint32_t *)0x5210083c) = 0x000010e0; // Mask offset
    *((volatile uint32_t *)0x52100840) = 0x00000027; // Output channel count
    *((volatile uint32_t *)0x5210080c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210084c) = 0x00002000; // Post processing register
    *((volatile uint32_t *)0x52100848) = 0xffffffff; // Mask and processor enables

    // Layer 8 quadrant 2
    *((volatile uint32_t *)0x53100804) = 0x0002000f; // Rows
    *((volatile uint32_t *)0x53100808) = 0x0001000f; // Columns
    *((volatile uint32_t *)0x53100818) = 0x00000020; // Stride
    *((volatile uint32_t *)0x53100820) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53100824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5310082c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x53100830) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53100834) = 0x00098011; // Layer control 2
    *((volatile uint32_t *)0x53100838) = 0x00001218; // Mask count
    *((volatile uint32_t *)0x5310083c) = 0x000010e0; // Mask offset
    *((volatile uint32_t *)0x53100840) = 0x00000027; // Output channel count
    *((volatile uint32_t *)0x5310080c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310084c) = 0x00003648; // Post processing register
    *((volatile uint32_t *)0x53100848) = 0xffffffff; // Mask and processor enables

    // Layer 8 quadrant 3
    *((volatile uint32_t *)0x54100804) = 0x0002000f; // Rows
    *((volatile uint32_t *)0x54100808) = 0x0001000f; // Columns
    *((volatile uint32_t *)0x54100818) = 0x00000020; // Stride
    *((volatile uint32_t *)0x54100820) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54100824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5410082c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x54100830) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54100834) = 0x00098011; // Layer control 2
    *((volatile uint32_t *)0x54100838) = 0x00001218; // Mask count
    *((volatile uint32_t *)0x5410083c) = 0x000010e0; // Mask offset
    *((volatile uint32_t *)0x54100840) = 0x00000027; // Output channel count
    *((volatile uint32_t *)0x5410080c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410084c) = 0x00002000; // Post processing register
    *((volatile uint32_t *)0x54100848) = 0x0fff0fff; // Mask and processor enables

    // Layer 9 quadrant 0
    *((volatile uint32_t *)0x51100904) = 0x0001000f; // Rows
    *((volatile uint32_t *)0x51100908) = 0x0001000f; // Columns
    *((volatile uint32_t *)0x51100918) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5110091c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x51100920) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x5110092c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51100930) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51100934) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x51100940) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5110090c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5110094c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x51100948) = 0x0000ffff; // Mask and processor enables

    // Layer 9 quadrant 1
    *((volatile uint32_t *)0x52100904) = 0x0001000f; // Rows
    *((volatile uint32_t *)0x52100908) = 0x0001000f; // Columns
    *((volatile uint32_t *)0x52100918) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5210091c) = 0x00020001; // SRAM write ptr
    *((volatile uint32_t *)0x52100920) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x5210092c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52100930) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52100934) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x52100940) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5210090c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5210094c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x52100948) = 0x0000000f; // Mask and processor enables

    // Layer 9 quadrant 2
    *((volatile uint32_t *)0x53100904) = 0x0001000f; // Rows
    *((volatile uint32_t *)0x53100908) = 0x0001000f; // Columns
    *((volatile uint32_t *)0x53100918) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5310091c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x53100920) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x5310092c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53100930) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53100934) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x53100940) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5310090c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5310094c) = 0x03000000; // Post processing register

    // Layer 9 quadrant 3
    *((volatile uint32_t *)0x54100904) = 0x0001000f; // Rows
    *((volatile uint32_t *)0x54100908) = 0x0001000f; // Columns
    *((volatile uint32_t *)0x54100918) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5410091c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x54100920) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x5410092c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54100930) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54100934) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x54100940) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5410090c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5410094c) = 0x03000000; // Post processing register

    // Layer 10 quadrant 0
    *((volatile uint32_t *)0x51100a04) = 0x0002000f; // Rows
    *((volatile uint32_t *)0x51100a08) = 0x0001000f; // Columns
    *((volatile uint32_t *)0x51100a18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x51100a1c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x51100a20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x51100a30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51100a40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x51100a0c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x51100a4c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x51100a48) = 0x0000ffff; // Mask and processor enables

    // Layer 10 quadrant 1
    *((volatile uint32_t *)0x52100a04) = 0x0002000f; // Rows
    *((volatile uint32_t *)0x52100a08) = 0x0001000f; // Columns
    *((volatile uint32_t *)0x52100a18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x52100a1c) = 0x00021000; // SRAM write ptr
    *((volatile uint32_t *)0x52100a20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x52100a30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52100a40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x52100a0c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x52100a4c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x52100a48) = 0x0000000f; // Mask and processor enables

    // Layer 10 quadrant 2
    *((volatile uint32_t *)0x53100a04) = 0x0002000f; // Rows
    *((volatile uint32_t *)0x53100a08) = 0x0001000f; // Columns
    *((volatile uint32_t *)0x53100a18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x53100a1c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x53100a20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x53100a30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53100a40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x53100a0c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x53100a4c) = 0x01000000; // Post processing register

    // Layer 10 quadrant 3
    *((volatile uint32_t *)0x54100a04) = 0x0002000f; // Rows
    *((volatile uint32_t *)0x54100a08) = 0x0001000f; // Columns
    *((volatile uint32_t *)0x54100a18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x54100a1c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x54100a20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x54100a30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54100a40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x54100a0c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x54100a4c) = 0x01000000; // Post processing register

    // Layer 11 quadrant 0
    *((volatile uint32_t *)0x51100b04) = 0x0001000f; // Rows
    *((volatile uint32_t *)0x51100b08) = 0x0001000f; // Columns
    *((volatile uint32_t *)0x51100b18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x51100b1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x51100b20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51100b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51100b28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51100b2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51100b30) = 0x00002b20; // Layer control
    *((volatile uint32_t *)0x51100b34) = 0x001d8010; // Layer control 2
    *((volatile uint32_t *)0x51100b38) = 0x000016d8; // Mask count
    *((volatile uint32_t *)0x51100b3c) = 0x00001320; // Mask offset
    *((volatile uint32_t *)0x51100b40) = 0x00000077; // Output channel count
    *((volatile uint32_t *)0x51100b0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51100b4c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x51100b48) = 0xffffffff; // Mask and processor enables

    // Layer 11 quadrant 1
    *((volatile uint32_t *)0x52100b04) = 0x0001000f; // Rows
    *((volatile uint32_t *)0x52100b08) = 0x0001000f; // Columns
    *((volatile uint32_t *)0x52100b18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x52100b1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x52100b20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52100b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52100b28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52100b2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52100b30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x52100b34) = 0x001d8010; // Layer control 2
    *((volatile uint32_t *)0x52100b38) = 0x000016d8; // Mask count
    *((volatile uint32_t *)0x52100b3c) = 0x00001320; // Mask offset
    *((volatile uint32_t *)0x52100b40) = 0x00000077; // Output channel count
    *((volatile uint32_t *)0x52100b0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x52100b4c) = 0x00023510; // Post processing register
    *((volatile uint32_t *)0x52100b48) = 0x000f000f; // Mask and processor enables

    // Layer 11 quadrant 2
    *((volatile uint32_t *)0x53100b04) = 0x0001000f; // Rows
    *((volatile uint32_t *)0x53100b08) = 0x0001000f; // Columns
    *((volatile uint32_t *)0x53100b18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x53100b1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x53100b20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53100b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53100b28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53100b2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53100b30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x53100b34) = 0x001d8010; // Layer control 2
    *((volatile uint32_t *)0x53100b38) = 0x000016d8; // Mask count
    *((volatile uint32_t *)0x53100b3c) = 0x00001320; // Mask offset
    *((volatile uint32_t *)0x53100b40) = 0x00000077; // Output channel count
    *((volatile uint32_t *)0x53100b0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x53100b4c) = 0x00022000; // Post processing register

    // Layer 11 quadrant 3
    *((volatile uint32_t *)0x54100b04) = 0x0001000f; // Rows
    *((volatile uint32_t *)0x54100b08) = 0x0001000f; // Columns
    *((volatile uint32_t *)0x54100b18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x54100b1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x54100b20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54100b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54100b28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54100b2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54100b30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x54100b34) = 0x001d8010; // Layer control 2
    *((volatile uint32_t *)0x54100b38) = 0x000016d8; // Mask count
    *((volatile uint32_t *)0x54100b3c) = 0x00001320; // Mask offset
    *((volatile uint32_t *)0x54100b40) = 0x00000077; // Output channel count
    *((volatile uint32_t *)0x54100b0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x54100b4c) = 0x00022000; // Post processing register

    // Layer 12 quadrant 0
    *((volatile uint32_t *)0x51100c04) = 0x0024800e; // Rows
    *((volatile uint32_t *)0x51100c08) = 0x0002800e; // Columns
    *((volatile uint32_t *)0x51100c10) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x51100c14) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x51100c18) = 0x00000041; // Stride
    *((volatile uint32_t *)0x51100c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51100c28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51100c2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x51100c30) = 0x20880ba0; // Layer control
    *((volatile uint32_t *)0x51100c34) = 0x001d8011; // Layer control 2
    *((volatile uint32_t *)0x51100c38) = 0x000002a8; // Mask count
    *((volatile uint32_t *)0x51100c3c) = 0x000002a0; // Mask offset
    *((volatile uint32_t *)0x51100c40) = 0x00000007; // Output channel count
    *((volatile uint32_t *)0x51100c0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x51100c44) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t *)0x51100c4c) = 0x41002000; // Post processing register
    *((volatile uint32_t *)0x51100c48) = 0xffffffff; // Mask and processor enables

    // Layer 12 quadrant 1
    *((volatile uint32_t *)0x52100c04) = 0x0024800e; // Rows
    *((volatile uint32_t *)0x52100c08) = 0x0002800e; // Columns
    *((volatile uint32_t *)0x52100c10) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x52100c14) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x52100c18) = 0x00000041; // Stride
    *((volatile uint32_t *)0x52100c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52100c28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52100c2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x52100c30) = 0x20880ba0; // Layer control
    *((volatile uint32_t *)0x52100c34) = 0x001d8011; // Layer control 2
    *((volatile uint32_t *)0x52100c38) = 0x000002a8; // Mask count
    *((volatile uint32_t *)0x52100c3c) = 0x000002a0; // Mask offset
    *((volatile uint32_t *)0x52100c40) = 0x00000007; // Output channel count
    *((volatile uint32_t *)0x52100c0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x52100c44) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t *)0x52100c4c) = 0x41002000; // Post processing register
    *((volatile uint32_t *)0x52100c48) = 0xffffffff; // Mask and processor enables

    // Layer 12 quadrant 2
    *((volatile uint32_t *)0x53100c04) = 0x0024800e; // Rows
    *((volatile uint32_t *)0x53100c08) = 0x0002800e; // Columns
    *((volatile uint32_t *)0x53100c10) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x53100c14) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x53100c18) = 0x00000041; // Stride
    *((volatile uint32_t *)0x53100c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53100c28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53100c2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x53100c30) = 0x20880ba0; // Layer control
    *((volatile uint32_t *)0x53100c34) = 0x001d8011; // Layer control 2
    *((volatile uint32_t *)0x53100c38) = 0x000002a8; // Mask count
    *((volatile uint32_t *)0x53100c3c) = 0x000002a0; // Mask offset
    *((volatile uint32_t *)0x53100c40) = 0x00000007; // Output channel count
    *((volatile uint32_t *)0x53100c0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x53100c44) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t *)0x53100c4c) = 0x41002000; // Post processing register
    *((volatile uint32_t *)0x53100c48) = 0xffffffff; // Mask and processor enables

    // Layer 12 quadrant 3
    *((volatile uint32_t *)0x54100c04) = 0x0024800e; // Rows
    *((volatile uint32_t *)0x54100c08) = 0x0002800e; // Columns
    *((volatile uint32_t *)0x54100c10) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x54100c14) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x54100c18) = 0x00000041; // Stride
    *((volatile uint32_t *)0x54100c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54100c28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54100c2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x54100c30) = 0x20880ba0; // Layer control
    *((volatile uint32_t *)0x54100c34) = 0x001d8011; // Layer control 2
    *((volatile uint32_t *)0x54100c38) = 0x000002a8; // Mask count
    *((volatile uint32_t *)0x54100c3c) = 0x000002a0; // Mask offset
    *((volatile uint32_t *)0x54100c40) = 0x00000007; // Output channel count
    *((volatile uint32_t *)0x54100c0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x54100c44) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t *)0x54100c4c) = 0x41002000; // Post processing register
    *((volatile uint32_t *)0x54100c48) = 0x0fff0fff; // Mask and processor enables

    // Layer 13 quadrant 0
    *((volatile uint32_t *)0x51100d04) = 0x00020007; // Rows
    *((volatile uint32_t *)0x51100d08) = 0x00010007; // Columns
    *((volatile uint32_t *)0x51100d18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x51100d1c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x51100d20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51100d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51100d30) = 0x0000e920; // Layer control
    *((volatile uint32_t *)0x51100d34) = 0x000b8001; // Layer control 2
    *((volatile uint32_t *)0x51100d38) = 0x00001a38; // Mask count
    *((volatile uint32_t *)0x51100d3c) = 0x000018c0; // Mask offset
    *((volatile uint32_t *)0x51100d40) = 0x0000002f; // Output channel count
    *((volatile uint32_t *)0x51100d0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51100d4c) = 0x00001630; // Post processing register
    *((volatile uint32_t *)0x51100d48) = 0xffffffff; // Mask and processor enables

    // Layer 13 quadrant 1
    *((volatile uint32_t *)0x52100d04) = 0x00020007; // Rows
    *((volatile uint32_t *)0x52100d08) = 0x00010007; // Columns
    *((volatile uint32_t *)0x52100d18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x52100d1c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x52100d20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52100d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52100d30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52100d34) = 0x000b8001; // Layer control 2
    *((volatile uint32_t *)0x52100d38) = 0x00001a38; // Mask count
    *((volatile uint32_t *)0x52100d3c) = 0x000018c0; // Mask offset
    *((volatile uint32_t *)0x52100d40) = 0x0000002f; // Output channel count
    *((volatile uint32_t *)0x52100d0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x52100d48) = 0xffffffff; // Mask and processor enables

    // Layer 13 quadrant 2
    *((volatile uint32_t *)0x53100d04) = 0x00020007; // Rows
    *((volatile uint32_t *)0x53100d08) = 0x00010007; // Columns
    *((volatile uint32_t *)0x53100d18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x53100d1c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x53100d20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53100d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53100d30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53100d34) = 0x000b8001; // Layer control 2
    *((volatile uint32_t *)0x53100d38) = 0x00001a38; // Mask count
    *((volatile uint32_t *)0x53100d3c) = 0x000018c0; // Mask offset
    *((volatile uint32_t *)0x53100d40) = 0x0000002f; // Output channel count
    *((volatile uint32_t *)0x53100d0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x53100d48) = 0xffffffff; // Mask and processor enables

    // Layer 13 quadrant 3
    *((volatile uint32_t *)0x54100d04) = 0x00020007; // Rows
    *((volatile uint32_t *)0x54100d08) = 0x00010007; // Columns
    *((volatile uint32_t *)0x54100d18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x54100d1c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x54100d20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54100d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54100d30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54100d34) = 0x000b8001; // Layer control 2
    *((volatile uint32_t *)0x54100d38) = 0x00001a38; // Mask count
    *((volatile uint32_t *)0x54100d3c) = 0x000018c0; // Mask offset
    *((volatile uint32_t *)0x54100d40) = 0x0000002f; // Output channel count
    *((volatile uint32_t *)0x54100d0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x54100d48) = 0x0fff0fff; // Mask and processor enables

    // Layer 14 quadrant 0
    *((volatile uint32_t *)0x51100e04) = 0x00010007; // Rows
    *((volatile uint32_t *)0x51100e08) = 0x00010007; // Columns
    *((volatile uint32_t *)0x51100e18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x51100e20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51100e24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51100e28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51100e2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51100e30) = 0x00002b20; // Layer control
    *((volatile uint32_t *)0x51100e34) = 0x00178020; // Layer control 2
    *((volatile uint32_t *)0x51100e38) = 0x00001f78; // Mask count
    *((volatile uint32_t *)0x51100e3c) = 0x00001b00; // Mask offset
    *((volatile uint32_t *)0x51100e40) = 0x0000008f; // Output channel count
    *((volatile uint32_t *)0x51100e0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51100e4c) = 0x00023480; // Post processing register
    *((volatile uint32_t *)0x51100e48) = 0xffffffff; // Mask and processor enables

    // Layer 14 quadrant 1
    *((volatile uint32_t *)0x52100e04) = 0x00010007; // Rows
    *((volatile uint32_t *)0x52100e08) = 0x00010007; // Columns
    *((volatile uint32_t *)0x52100e18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x52100e20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52100e24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52100e28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52100e2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52100e30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x52100e34) = 0x00178020; // Layer control 2
    *((volatile uint32_t *)0x52100e38) = 0x00001f78; // Mask count
    *((volatile uint32_t *)0x52100e3c) = 0x00001b00; // Mask offset
    *((volatile uint32_t *)0x52100e40) = 0x0000008f; // Output channel count
    *((volatile uint32_t *)0x52100e0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x52100e4c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x52100e48) = 0x00ff00ff; // Mask and processor enables

    // Layer 14 quadrant 2
    *((volatile uint32_t *)0x53100e04) = 0x00010007; // Rows
    *((volatile uint32_t *)0x53100e08) = 0x00010007; // Columns
    *((volatile uint32_t *)0x53100e18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x53100e20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53100e24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53100e28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53100e2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53100e30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x53100e34) = 0x00178020; // Layer control 2
    *((volatile uint32_t *)0x53100e38) = 0x00001f78; // Mask count
    *((volatile uint32_t *)0x53100e3c) = 0x00001b00; // Mask offset
    *((volatile uint32_t *)0x53100e40) = 0x0000008f; // Output channel count
    *((volatile uint32_t *)0x53100e0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x53100e4c) = 0x00022000; // Post processing register

    // Layer 14 quadrant 3
    *((volatile uint32_t *)0x54100e04) = 0x00010007; // Rows
    *((volatile uint32_t *)0x54100e08) = 0x00010007; // Columns
    *((volatile uint32_t *)0x54100e18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x54100e20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54100e24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54100e28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54100e2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54100e30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x54100e34) = 0x00178020; // Layer control 2
    *((volatile uint32_t *)0x54100e38) = 0x00001f78; // Mask count
    *((volatile uint32_t *)0x54100e3c) = 0x00001b00; // Mask offset
    *((volatile uint32_t *)0x54100e40) = 0x0000008f; // Output channel count
    *((volatile uint32_t *)0x54100e0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x54100e4c) = 0x00022000; // Post processing register

    // Layer 15 quadrant 0
    *((volatile uint32_t *)0x51100f04) = 0x00038007; // Rows
    *((volatile uint32_t *)0x51100f08) = 0x00018007; // Columns
    *((volatile uint32_t *)0x51100f18) = 0x00000030; // Stride
    *((volatile uint32_t *)0x51100f1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x51100f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51100f28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51100f30) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x51100f34) = 0x00178022; // Layer control 2
    *((volatile uint32_t *)0x51100f38) = 0x00000390; // Mask count
    *((volatile uint32_t *)0x51100f3c) = 0x00000380; // Mask offset
    *((volatile uint32_t *)0x51100f40) = 0x0000000b; // Output channel count
    *((volatile uint32_t *)0x51100f0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x51100f44) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t *)0x51100f4c) = 0x41000000; // Post processing register
    *((volatile uint32_t *)0x51100f48) = 0xffffffff; // Mask and processor enables

    // Layer 15 quadrant 1
    *((volatile uint32_t *)0x52100f04) = 0x00038007; // Rows
    *((volatile uint32_t *)0x52100f08) = 0x00018007; // Columns
    *((volatile uint32_t *)0x52100f18) = 0x00000030; // Stride
    *((volatile uint32_t *)0x52100f1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x52100f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52100f28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52100f30) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x52100f34) = 0x00178022; // Layer control 2
    *((volatile uint32_t *)0x52100f38) = 0x00000390; // Mask count
    *((volatile uint32_t *)0x52100f3c) = 0x00000380; // Mask offset
    *((volatile uint32_t *)0x52100f40) = 0x0000000b; // Output channel count
    *((volatile uint32_t *)0x52100f0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x52100f44) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t *)0x52100f4c) = 0x41000000; // Post processing register
    *((volatile uint32_t *)0x52100f48) = 0xffffffff; // Mask and processor enables

    // Layer 15 quadrant 2
    *((volatile uint32_t *)0x53100f04) = 0x00038007; // Rows
    *((volatile uint32_t *)0x53100f08) = 0x00018007; // Columns
    *((volatile uint32_t *)0x53100f18) = 0x00000030; // Stride
    *((volatile uint32_t *)0x53100f1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x53100f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53100f28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53100f30) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x53100f34) = 0x00178022; // Layer control 2
    *((volatile uint32_t *)0x53100f38) = 0x00000390; // Mask count
    *((volatile uint32_t *)0x53100f3c) = 0x00000380; // Mask offset
    *((volatile uint32_t *)0x53100f40) = 0x0000000b; // Output channel count
    *((volatile uint32_t *)0x53100f0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x53100f44) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t *)0x53100f4c) = 0x41000000; // Post processing register
    *((volatile uint32_t *)0x53100f48) = 0xffffffff; // Mask and processor enables

    // Layer 15 quadrant 3
    *((volatile uint32_t *)0x54100f04) = 0x00038007; // Rows
    *((volatile uint32_t *)0x54100f08) = 0x00018007; // Columns
    *((volatile uint32_t *)0x54100f18) = 0x00000030; // Stride
    *((volatile uint32_t *)0x54100f1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x54100f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54100f28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54100f30) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x54100f34) = 0x00178022; // Layer control 2
    *((volatile uint32_t *)0x54100f38) = 0x00000390; // Mask count
    *((volatile uint32_t *)0x54100f3c) = 0x00000380; // Mask offset
    *((volatile uint32_t *)0x54100f40) = 0x0000000b; // Output channel count
    *((volatile uint32_t *)0x54100f0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x54100f44) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t *)0x54100f4c) = 0x41000000; // Post processing register

    // Layer 16 quadrant 0
    *((volatile uint32_t *)0x51101004) = 0x00030007; // Rows
    *((volatile uint32_t *)0x51101008) = 0x00010007; // Columns
    *((volatile uint32_t *)0x51101018) = 0x00000030; // Stride
    *((volatile uint32_t *)0x51101020) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51101024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5110102c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x51101030) = 0x0000e920; // Layer control
    *((volatile uint32_t *)0x51101034) = 0x000b8012; // Layer control 2
    *((volatile uint32_t *)0x51101038) = 0x000022d8; // Mask count
    *((volatile uint32_t *)0x5110103c) = 0x000020a0; // Mask offset
    *((volatile uint32_t *)0x51101040) = 0x00000047; // Output channel count
    *((volatile uint32_t *)0x5110100c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5110104c) = 0x00002000; // Post processing register
    *((volatile uint32_t *)0x51101048) = 0xffffffff; // Mask and processor enables

    // Layer 16 quadrant 1
    *((volatile uint32_t *)0x52101004) = 0x00030007; // Rows
    *((volatile uint32_t *)0x52101008) = 0x00010007; // Columns
    *((volatile uint32_t *)0x52101018) = 0x00000030; // Stride
    *((volatile uint32_t *)0x52101020) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52101024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5210102c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x52101030) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52101034) = 0x000b8012; // Layer control 2
    *((volatile uint32_t *)0x52101038) = 0x000022d8; // Mask count
    *((volatile uint32_t *)0x5210103c) = 0x000020a0; // Mask offset
    *((volatile uint32_t *)0x52101040) = 0x00000047; // Output channel count
    *((volatile uint32_t *)0x5210100c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210104c) = 0x00002000; // Post processing register
    *((volatile uint32_t *)0x52101048) = 0xffffffff; // Mask and processor enables

    // Layer 16 quadrant 2
    *((volatile uint32_t *)0x53101004) = 0x00030007; // Rows
    *((volatile uint32_t *)0x53101008) = 0x00010007; // Columns
    *((volatile uint32_t *)0x53101018) = 0x00000030; // Stride
    *((volatile uint32_t *)0x53101020) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53101024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5310102c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x53101030) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53101034) = 0x000b8012; // Layer control 2
    *((volatile uint32_t *)0x53101038) = 0x000022d8; // Mask count
    *((volatile uint32_t *)0x5310103c) = 0x000020a0; // Mask offset
    *((volatile uint32_t *)0x53101040) = 0x00000047; // Output channel count
    *((volatile uint32_t *)0x5310100c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310104c) = 0x00002000; // Post processing register
    *((volatile uint32_t *)0x53101048) = 0xffffffff; // Mask and processor enables

    // Layer 16 quadrant 3
    *((volatile uint32_t *)0x54101004) = 0x00030007; // Rows
    *((volatile uint32_t *)0x54101008) = 0x00010007; // Columns
    *((volatile uint32_t *)0x54101018) = 0x00000030; // Stride
    *((volatile uint32_t *)0x54101020) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54101024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5410102c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x54101030) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54101034) = 0x000b8012; // Layer control 2
    *((volatile uint32_t *)0x54101038) = 0x000022d8; // Mask count
    *((volatile uint32_t *)0x5410103c) = 0x000020a0; // Mask offset
    *((volatile uint32_t *)0x54101040) = 0x00000047; // Output channel count
    *((volatile uint32_t *)0x5410100c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410104c) = 0x00003630; // Post processing register

    // Layer 17 quadrant 0
    *((volatile uint32_t *)0x51101104) = 0x00010007; // Rows
    *((volatile uint32_t *)0x51101108) = 0x00010007; // Columns
    *((volatile uint32_t *)0x51101118) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5110111c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x51101120) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x5110112c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51101130) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51101134) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x51101140) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5110110c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5110114c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x51101148) = 0x0000ffff; // Mask and processor enables

    // Layer 17 quadrant 1
    *((volatile uint32_t *)0x52101104) = 0x00010007; // Rows
    *((volatile uint32_t *)0x52101108) = 0x00010007; // Columns
    *((volatile uint32_t *)0x52101118) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5210111c) = 0x00020001; // SRAM write ptr
    *((volatile uint32_t *)0x52101120) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x5210112c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52101130) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52101134) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x52101140) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5210110c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5210114c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x52101148) = 0x000000ff; // Mask and processor enables

    // Layer 17 quadrant 2
    *((volatile uint32_t *)0x53101104) = 0x00010007; // Rows
    *((volatile uint32_t *)0x53101108) = 0x00010007; // Columns
    *((volatile uint32_t *)0x53101118) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5310111c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x53101120) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x5310112c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53101130) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53101134) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x53101140) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5310110c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5310114c) = 0x03000000; // Post processing register

    // Layer 17 quadrant 3
    *((volatile uint32_t *)0x54101104) = 0x00010007; // Rows
    *((volatile uint32_t *)0x54101108) = 0x00010007; // Columns
    *((volatile uint32_t *)0x54101118) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5410111c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x54101120) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x5410112c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54101130) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54101134) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x54101140) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5410110c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5410114c) = 0x03000000; // Post processing register

    // Layer 18 quadrant 0
    *((volatile uint32_t *)0x51101204) = 0x00020007; // Rows
    *((volatile uint32_t *)0x51101208) = 0x00010007; // Columns
    *((volatile uint32_t *)0x51101218) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5110121c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x51101220) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x51101230) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51101240) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5110120c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5110124c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x51101248) = 0x0000ffff; // Mask and processor enables

    // Layer 18 quadrant 1
    *((volatile uint32_t *)0x52101204) = 0x00020007; // Rows
    *((volatile uint32_t *)0x52101208) = 0x00010007; // Columns
    *((volatile uint32_t *)0x52101218) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5210121c) = 0x00021000; // SRAM write ptr
    *((volatile uint32_t *)0x52101220) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x52101230) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52101240) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5210120c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5210124c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x52101248) = 0x000000ff; // Mask and processor enables

    // Layer 18 quadrant 2
    *((volatile uint32_t *)0x53101204) = 0x00020007; // Rows
    *((volatile uint32_t *)0x53101208) = 0x00010007; // Columns
    *((volatile uint32_t *)0x53101218) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5310121c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x53101220) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x53101230) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53101240) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5310120c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5310124c) = 0x01000000; // Post processing register

    // Layer 18 quadrant 3
    *((volatile uint32_t *)0x54101204) = 0x00020007; // Rows
    *((volatile uint32_t *)0x54101208) = 0x00010007; // Columns
    *((volatile uint32_t *)0x54101218) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5410121c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x54101220) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x54101230) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54101240) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5410120c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5410124c) = 0x01000000; // Post processing register

    // Layer 19 quadrant 0
    *((volatile uint32_t *)0x51101304) = 0x00010007; // Rows
    *((volatile uint32_t *)0x51101308) = 0x00010007; // Columns
    *((volatile uint32_t *)0x51101318) = 0x00000010; // Stride
    *((volatile uint32_t *)0x51101320) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51101324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51101328) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110132c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51101330) = 0x0000ab20; // Layer control
    *((volatile uint32_t *)0x51101334) = 0x00178020; // Layer control 2
    *((volatile uint32_t *)0x51101338) = 0x00002758; // Mask count
    *((volatile uint32_t *)0x5110133c) = 0x000022e0; // Mask offset
    *((volatile uint32_t *)0x51101340) = 0x0000008f; // Output channel count
    *((volatile uint32_t *)0x5110130c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5110134c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x51101348) = 0xffffffff; // Mask and processor enables

    // Layer 19 quadrant 1
    *((volatile uint32_t *)0x52101304) = 0x00010007; // Rows
    *((volatile uint32_t *)0x52101308) = 0x00010007; // Columns
    *((volatile uint32_t *)0x52101318) = 0x00000010; // Stride
    *((volatile uint32_t *)0x52101320) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52101324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52101328) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210132c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52101330) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x52101334) = 0x00178020; // Layer control 2
    *((volatile uint32_t *)0x52101338) = 0x00002758; // Mask count
    *((volatile uint32_t *)0x5210133c) = 0x000022e0; // Mask offset
    *((volatile uint32_t *)0x52101340) = 0x0000008f; // Output channel count
    *((volatile uint32_t *)0x5210130c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210134c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x52101348) = 0x00ff00ff; // Mask and processor enables

    // Layer 19 quadrant 2
    *((volatile uint32_t *)0x53101304) = 0x00010007; // Rows
    *((volatile uint32_t *)0x53101308) = 0x00010007; // Columns
    *((volatile uint32_t *)0x53101318) = 0x00000010; // Stride
    *((volatile uint32_t *)0x53101320) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53101324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53101328) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310132c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53101330) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x53101334) = 0x00178020; // Layer control 2
    *((volatile uint32_t *)0x53101338) = 0x00002758; // Mask count
    *((volatile uint32_t *)0x5310133c) = 0x000022e0; // Mask offset
    *((volatile uint32_t *)0x53101340) = 0x0000008f; // Output channel count
    *((volatile uint32_t *)0x5310130c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310134c) = 0x00022000; // Post processing register

    // Layer 19 quadrant 3
    *((volatile uint32_t *)0x54101304) = 0x00010007; // Rows
    *((volatile uint32_t *)0x54101308) = 0x00010007; // Columns
    *((volatile uint32_t *)0x54101318) = 0x00000010; // Stride
    *((volatile uint32_t *)0x54101320) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54101324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54101328) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410132c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54101330) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x54101334) = 0x00178020; // Layer control 2
    *((volatile uint32_t *)0x54101338) = 0x00002758; // Mask count
    *((volatile uint32_t *)0x5410133c) = 0x000022e0; // Mask offset
    *((volatile uint32_t *)0x54101340) = 0x0000008f; // Output channel count
    *((volatile uint32_t *)0x5410130c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410134c) = 0x00023480; // Post processing register

    // Layer 20 quadrant 0
    *((volatile uint32_t *)0x51101404) = 0x00038007; // Rows
    *((volatile uint32_t *)0x51101408) = 0x00018007; // Columns
    *((volatile uint32_t *)0x51101418) = 0x00000030; // Stride
    *((volatile uint32_t *)0x5110141c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x51101424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51101428) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51101430) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x51101434) = 0x00178022; // Layer control 2
    *((volatile uint32_t *)0x51101438) = 0x00000470; // Mask count
    *((volatile uint32_t *)0x5110143c) = 0x00000460; // Mask offset
    *((volatile uint32_t *)0x51101440) = 0x0000000b; // Output channel count
    *((volatile uint32_t *)0x5110140c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x51101444) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t *)0x5110144c) = 0x41000000; // Post processing register
    *((volatile uint32_t *)0x51101448) = 0xffffffff; // Mask and processor enables

    // Layer 20 quadrant 1
    *((volatile uint32_t *)0x52101404) = 0x00038007; // Rows
    *((volatile uint32_t *)0x52101408) = 0x00018007; // Columns
    *((volatile uint32_t *)0x52101418) = 0x00000030; // Stride
    *((volatile uint32_t *)0x5210141c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x52101424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52101428) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52101430) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x52101434) = 0x00178022; // Layer control 2
    *((volatile uint32_t *)0x52101438) = 0x00000470; // Mask count
    *((volatile uint32_t *)0x5210143c) = 0x00000460; // Mask offset
    *((volatile uint32_t *)0x52101440) = 0x0000000b; // Output channel count
    *((volatile uint32_t *)0x5210140c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x52101444) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t *)0x5210144c) = 0x41000000; // Post processing register
    *((volatile uint32_t *)0x52101448) = 0xffffffff; // Mask and processor enables

    // Layer 20 quadrant 2
    *((volatile uint32_t *)0x53101404) = 0x00038007; // Rows
    *((volatile uint32_t *)0x53101408) = 0x00018007; // Columns
    *((volatile uint32_t *)0x53101418) = 0x00000030; // Stride
    *((volatile uint32_t *)0x5310141c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x53101424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53101428) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53101430) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x53101434) = 0x00178022; // Layer control 2
    *((volatile uint32_t *)0x53101438) = 0x00000470; // Mask count
    *((volatile uint32_t *)0x5310143c) = 0x00000460; // Mask offset
    *((volatile uint32_t *)0x53101440) = 0x0000000b; // Output channel count
    *((volatile uint32_t *)0x5310140c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x53101444) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t *)0x5310144c) = 0x41000000; // Post processing register
    *((volatile uint32_t *)0x53101448) = 0xffffffff; // Mask and processor enables

    // Layer 20 quadrant 3
    *((volatile uint32_t *)0x54101404) = 0x00038007; // Rows
    *((volatile uint32_t *)0x54101408) = 0x00018007; // Columns
    *((volatile uint32_t *)0x54101418) = 0x00000030; // Stride
    *((volatile uint32_t *)0x5410141c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x54101424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54101428) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54101430) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x54101434) = 0x00178022; // Layer control 2
    *((volatile uint32_t *)0x54101438) = 0x00000470; // Mask count
    *((volatile uint32_t *)0x5410143c) = 0x00000460; // Mask offset
    *((volatile uint32_t *)0x54101440) = 0x0000000b; // Output channel count
    *((volatile uint32_t *)0x5410140c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x54101444) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t *)0x5410144c) = 0x41000000; // Post processing register

    // Layer 21 quadrant 0
    *((volatile uint32_t *)0x51101504) = 0x00030007; // Rows
    *((volatile uint32_t *)0x51101508) = 0x00010007; // Columns
    *((volatile uint32_t *)0x51101518) = 0x00000030; // Stride
    *((volatile uint32_t *)0x51101520) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51101524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5110152c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x51101530) = 0x00006920; // Layer control
    *((volatile uint32_t *)0x51101534) = 0x000b8012; // Layer control 2
    *((volatile uint32_t *)0x51101538) = 0x00002ab8; // Mask count
    *((volatile uint32_t *)0x5110153c) = 0x00002880; // Mask offset
    *((volatile uint32_t *)0x51101540) = 0x00000047; // Output channel count
    *((volatile uint32_t *)0x5110150c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5110154c) = 0x00003648; // Post processing register
    *((volatile uint32_t *)0x51101548) = 0xffffffff; // Mask and processor enables

    // Layer 21 quadrant 1
    *((volatile uint32_t *)0x52101504) = 0x00030007; // Rows
    *((volatile uint32_t *)0x52101508) = 0x00010007; // Columns
    *((volatile uint32_t *)0x52101518) = 0x00000030; // Stride
    *((volatile uint32_t *)0x52101520) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52101524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5210152c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x52101530) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52101534) = 0x000b8012; // Layer control 2
    *((volatile uint32_t *)0x52101538) = 0x00002ab8; // Mask count
    *((volatile uint32_t *)0x5210153c) = 0x00002880; // Mask offset
    *((volatile uint32_t *)0x52101540) = 0x00000047; // Output channel count
    *((volatile uint32_t *)0x5210150c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210154c) = 0x00002000; // Post processing register
    *((volatile uint32_t *)0x52101548) = 0xffffffff; // Mask and processor enables

    // Layer 21 quadrant 2
    *((volatile uint32_t *)0x53101504) = 0x00030007; // Rows
    *((volatile uint32_t *)0x53101508) = 0x00010007; // Columns
    *((volatile uint32_t *)0x53101518) = 0x00000030; // Stride
    *((volatile uint32_t *)0x53101520) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53101524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5310152c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x53101530) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53101534) = 0x000b8012; // Layer control 2
    *((volatile uint32_t *)0x53101538) = 0x00002ab8; // Mask count
    *((volatile uint32_t *)0x5310153c) = 0x00002880; // Mask offset
    *((volatile uint32_t *)0x53101540) = 0x00000047; // Output channel count
    *((volatile uint32_t *)0x5310150c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310154c) = 0x00002000; // Post processing register
    *((volatile uint32_t *)0x53101548) = 0xffffffff; // Mask and processor enables

    // Layer 21 quadrant 3
    *((volatile uint32_t *)0x54101504) = 0x00030007; // Rows
    *((volatile uint32_t *)0x54101508) = 0x00010007; // Columns
    *((volatile uint32_t *)0x54101518) = 0x00000030; // Stride
    *((volatile uint32_t *)0x54101520) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54101524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5410152c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x54101530) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54101534) = 0x000b8012; // Layer control 2
    *((volatile uint32_t *)0x54101538) = 0x00002ab8; // Mask count
    *((volatile uint32_t *)0x5410153c) = 0x00002880; // Mask offset
    *((volatile uint32_t *)0x54101540) = 0x00000047; // Output channel count
    *((volatile uint32_t *)0x5410150c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410154c) = 0x00002000; // Post processing register

    // Layer 22 quadrant 0
    *((volatile uint32_t *)0x51101604) = 0x00010007; // Rows
    *((volatile uint32_t *)0x51101608) = 0x00010007; // Columns
    *((volatile uint32_t *)0x51101618) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5110161c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x51101620) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x5110162c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51101630) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51101634) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x51101640) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5110160c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5110164c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x51101648) = 0x0000ffff; // Mask and processor enables

    // Layer 22 quadrant 1
    *((volatile uint32_t *)0x52101604) = 0x00010007; // Rows
    *((volatile uint32_t *)0x52101608) = 0x00010007; // Columns
    *((volatile uint32_t *)0x52101618) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5210161c) = 0x00020001; // SRAM write ptr
    *((volatile uint32_t *)0x52101620) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x5210162c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52101630) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52101634) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x52101640) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5210160c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5210164c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x52101648) = 0x000000ff; // Mask and processor enables

    // Layer 22 quadrant 2
    *((volatile uint32_t *)0x53101604) = 0x00010007; // Rows
    *((volatile uint32_t *)0x53101608) = 0x00010007; // Columns
    *((volatile uint32_t *)0x53101618) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5310161c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x53101620) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x5310162c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53101630) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53101634) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x53101640) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5310160c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5310164c) = 0x03000000; // Post processing register

    // Layer 22 quadrant 3
    *((volatile uint32_t *)0x54101604) = 0x00010007; // Rows
    *((volatile uint32_t *)0x54101608) = 0x00010007; // Columns
    *((volatile uint32_t *)0x54101618) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5410161c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x54101620) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x5410162c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54101630) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54101634) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x54101640) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5410160c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5410164c) = 0x03000000; // Post processing register

    // Layer 23 quadrant 0
    *((volatile uint32_t *)0x51101704) = 0x00020007; // Rows
    *((volatile uint32_t *)0x51101708) = 0x00010007; // Columns
    *((volatile uint32_t *)0x51101718) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5110171c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x51101720) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x51101730) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51101740) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5110170c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5110174c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x51101748) = 0x0000ffff; // Mask and processor enables

    // Layer 23 quadrant 1
    *((volatile uint32_t *)0x52101704) = 0x00020007; // Rows
    *((volatile uint32_t *)0x52101708) = 0x00010007; // Columns
    *((volatile uint32_t *)0x52101718) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5210171c) = 0x00021000; // SRAM write ptr
    *((volatile uint32_t *)0x52101720) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x52101730) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52101740) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5210170c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5210174c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x52101748) = 0x000000ff; // Mask and processor enables

    // Layer 23 quadrant 2
    *((volatile uint32_t *)0x53101704) = 0x00020007; // Rows
    *((volatile uint32_t *)0x53101708) = 0x00010007; // Columns
    *((volatile uint32_t *)0x53101718) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5310171c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x53101720) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x53101730) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53101740) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5310170c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5310174c) = 0x01000000; // Post processing register

    // Layer 23 quadrant 3
    *((volatile uint32_t *)0x54101704) = 0x00020007; // Rows
    *((volatile uint32_t *)0x54101708) = 0x00010007; // Columns
    *((volatile uint32_t *)0x54101718) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5410171c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x54101720) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x54101730) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54101740) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5410170c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5410174c) = 0x01000000; // Post processing register

    // Layer 24 quadrant 0
    *((volatile uint32_t *)0x51101804) = 0x00010007; // Rows
    *((volatile uint32_t *)0x51101808) = 0x00010007; // Columns
    *((volatile uint32_t *)0x51101818) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5110181c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x51101820) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51101824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51101828) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110182c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51101830) = 0x00006b20; // Layer control
    *((volatile uint32_t *)0x51101834) = 0x00178020; // Layer control 2
    *((volatile uint32_t *)0x51101838) = 0x00002f38; // Mask count
    *((volatile uint32_t *)0x5110183c) = 0x00002ac0; // Mask offset
    *((volatile uint32_t *)0x51101840) = 0x0000008f; // Output channel count
    *((volatile uint32_t *)0x5110180c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5110184c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x51101848) = 0xffffffff; // Mask and processor enables

    // Layer 24 quadrant 1
    *((volatile uint32_t *)0x52101804) = 0x00010007; // Rows
    *((volatile uint32_t *)0x52101808) = 0x00010007; // Columns
    *((volatile uint32_t *)0x52101818) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5210181c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x52101820) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52101824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52101828) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210182c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52101830) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x52101834) = 0x00178020; // Layer control 2
    *((volatile uint32_t *)0x52101838) = 0x00002f38; // Mask count
    *((volatile uint32_t *)0x5210183c) = 0x00002ac0; // Mask offset
    *((volatile uint32_t *)0x52101840) = 0x0000008f; // Output channel count
    *((volatile uint32_t *)0x5210180c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210184c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x52101848) = 0x00ff00ff; // Mask and processor enables

    // Layer 24 quadrant 2
    *((volatile uint32_t *)0x53101804) = 0x00010007; // Rows
    *((volatile uint32_t *)0x53101808) = 0x00010007; // Columns
    *((volatile uint32_t *)0x53101818) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5310181c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x53101820) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53101824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53101828) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310182c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53101830) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x53101834) = 0x00178020; // Layer control 2
    *((volatile uint32_t *)0x53101838) = 0x00002f38; // Mask count
    *((volatile uint32_t *)0x5310183c) = 0x00002ac0; // Mask offset
    *((volatile uint32_t *)0x53101840) = 0x0000008f; // Output channel count
    *((volatile uint32_t *)0x5310180c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310184c) = 0x000234e0; // Post processing register

    // Layer 24 quadrant 3
    *((volatile uint32_t *)0x54101804) = 0x00010007; // Rows
    *((volatile uint32_t *)0x54101808) = 0x00010007; // Columns
    *((volatile uint32_t *)0x54101818) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5410181c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x54101820) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54101824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54101828) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410182c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54101830) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x54101834) = 0x00178020; // Layer control 2
    *((volatile uint32_t *)0x54101838) = 0x00002f38; // Mask count
    *((volatile uint32_t *)0x5410183c) = 0x00002ac0; // Mask offset
    *((volatile uint32_t *)0x54101840) = 0x0000008f; // Output channel count
    *((volatile uint32_t *)0x5410180c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410184c) = 0x00022000; // Post processing register

    // Layer 25 quadrant 0
    *((volatile uint32_t *)0x51101904) = 0x001e8006; // Rows
    *((volatile uint32_t *)0x51101908) = 0x00028006; // Columns
    *((volatile uint32_t *)0x51101910) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x51101914) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x51101918) = 0x00000061; // Stride
    *((volatile uint32_t *)0x51101924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51101928) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110192c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x51101930) = 0x20880ba0; // Layer control
    *((volatile uint32_t *)0x51101934) = 0x00178022; // Layer control 2
    *((volatile uint32_t *)0x51101938) = 0x00000550; // Mask count
    *((volatile uint32_t *)0x5110193c) = 0x00000540; // Mask offset
    *((volatile uint32_t *)0x51101940) = 0x0000000b; // Output channel count
    *((volatile uint32_t *)0x5110190c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x51101944) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5110194c) = 0x41002000; // Post processing register
    *((volatile uint32_t *)0x51101948) = 0xffffffff; // Mask and processor enables

    // Layer 25 quadrant 1
    *((volatile uint32_t *)0x52101904) = 0x001e8006; // Rows
    *((volatile uint32_t *)0x52101908) = 0x00028006; // Columns
    *((volatile uint32_t *)0x52101910) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x52101914) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x52101918) = 0x00000061; // Stride
    *((volatile uint32_t *)0x52101924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52101928) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210192c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x52101930) = 0x20880ba0; // Layer control
    *((volatile uint32_t *)0x52101934) = 0x00178022; // Layer control 2
    *((volatile uint32_t *)0x52101938) = 0x00000550; // Mask count
    *((volatile uint32_t *)0x5210193c) = 0x00000540; // Mask offset
    *((volatile uint32_t *)0x52101940) = 0x0000000b; // Output channel count
    *((volatile uint32_t *)0x5210190c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x52101944) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5210194c) = 0x41002000; // Post processing register
    *((volatile uint32_t *)0x52101948) = 0xffffffff; // Mask and processor enables

    // Layer 25 quadrant 2
    *((volatile uint32_t *)0x53101904) = 0x001e8006; // Rows
    *((volatile uint32_t *)0x53101908) = 0x00028006; // Columns
    *((volatile uint32_t *)0x53101910) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x53101914) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x53101918) = 0x00000061; // Stride
    *((volatile uint32_t *)0x53101924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53101928) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310192c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x53101930) = 0x20880ba0; // Layer control
    *((volatile uint32_t *)0x53101934) = 0x00178022; // Layer control 2
    *((volatile uint32_t *)0x53101938) = 0x00000550; // Mask count
    *((volatile uint32_t *)0x5310193c) = 0x00000540; // Mask offset
    *((volatile uint32_t *)0x53101940) = 0x0000000b; // Output channel count
    *((volatile uint32_t *)0x5310190c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x53101944) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5310194c) = 0x41002000; // Post processing register
    *((volatile uint32_t *)0x53101948) = 0xffffffff; // Mask and processor enables

    // Layer 25 quadrant 3
    *((volatile uint32_t *)0x54101904) = 0x001e8006; // Rows
    *((volatile uint32_t *)0x54101908) = 0x00028006; // Columns
    *((volatile uint32_t *)0x54101910) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x54101914) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x54101918) = 0x00000061; // Stride
    *((volatile uint32_t *)0x54101924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54101928) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410192c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x54101930) = 0x20880ba0; // Layer control
    *((volatile uint32_t *)0x54101934) = 0x00178022; // Layer control 2
    *((volatile uint32_t *)0x54101938) = 0x00000550; // Mask count
    *((volatile uint32_t *)0x5410193c) = 0x00000540; // Mask offset
    *((volatile uint32_t *)0x54101940) = 0x0000000b; // Output channel count
    *((volatile uint32_t *)0x5410190c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x54101944) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5410194c) = 0x41002000; // Post processing register

    // Layer 26 quadrant 0
    *((volatile uint32_t *)0x51101a04) = 0x00030003; // Rows
    *((volatile uint32_t *)0x51101a08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51101a18) = 0x00000030; // Stride
    *((volatile uint32_t *)0x51101a1c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x51101a20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51101a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51101a30) = 0x00006920; // Layer control
    *((volatile uint32_t *)0x51101a34) = 0x00178002; // Layer control 2
    *((volatile uint32_t *)0x51101a38) = 0x000034d8; // Mask count
    *((volatile uint32_t *)0x51101a3c) = 0x00003060; // Mask offset
    *((volatile uint32_t *)0x51101a40) = 0x0000008f; // Output channel count
    *((volatile uint32_t *)0x51101a0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51101a48) = 0xffffffff; // Mask and processor enables

    // Layer 26 quadrant 1
    *((volatile uint32_t *)0x52101a04) = 0x00030003; // Rows
    *((volatile uint32_t *)0x52101a08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52101a18) = 0x00000030; // Stride
    *((volatile uint32_t *)0x52101a1c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x52101a20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52101a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52101a30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52101a34) = 0x00178002; // Layer control 2
    *((volatile uint32_t *)0x52101a38) = 0x000034d8; // Mask count
    *((volatile uint32_t *)0x52101a3c) = 0x00003060; // Mask offset
    *((volatile uint32_t *)0x52101a40) = 0x0000008f; // Output channel count
    *((volatile uint32_t *)0x52101a0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x52101a48) = 0xffffffff; // Mask and processor enables

    // Layer 26 quadrant 2
    *((volatile uint32_t *)0x53101a04) = 0x00030003; // Rows
    *((volatile uint32_t *)0x53101a08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53101a18) = 0x00000030; // Stride
    *((volatile uint32_t *)0x53101a1c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x53101a20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53101a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53101a30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53101a34) = 0x00178002; // Layer control 2
    *((volatile uint32_t *)0x53101a38) = 0x000034d8; // Mask count
    *((volatile uint32_t *)0x53101a3c) = 0x00003060; // Mask offset
    *((volatile uint32_t *)0x53101a40) = 0x0000008f; // Output channel count
    *((volatile uint32_t *)0x53101a0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x53101a4c) = 0x000015e8; // Post processing register
    *((volatile uint32_t *)0x53101a48) = 0xffffffff; // Mask and processor enables

    // Layer 26 quadrant 3
    *((volatile uint32_t *)0x54101a04) = 0x00030003; // Rows
    *((volatile uint32_t *)0x54101a08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54101a18) = 0x00000030; // Stride
    *((volatile uint32_t *)0x54101a1c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x54101a20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54101a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54101a30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54101a34) = 0x00178002; // Layer control 2
    *((volatile uint32_t *)0x54101a38) = 0x000034d8; // Mask count
    *((volatile uint32_t *)0x54101a3c) = 0x00003060; // Mask offset
    *((volatile uint32_t *)0x54101a40) = 0x0000008f; // Output channel count
    *((volatile uint32_t *)0x54101a0c) = 0x00000100; // 1D

    // Layer 27 quadrant 0
    *((volatile uint32_t *)0x51101b04) = 0x00010003; // Rows
    *((volatile uint32_t *)0x51101b08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51101b18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x51101b20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51101b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51101b28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51101b2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51101b30) = 0x00006b20; // Layer control
    *((volatile uint32_t *)0x51101b34) = 0x001d8040; // Layer control 2
    *((volatile uint32_t *)0x51101b38) = 0x00003e38; // Mask count
    *((volatile uint32_t *)0x51101b3c) = 0x000034e0; // Mask offset
    *((volatile uint32_t *)0x51101b40) = 0x0000012b; // Output channel count
    *((volatile uint32_t *)0x51101b0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51101b4c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x51101b48) = 0xffffffff; // Mask and processor enables

    // Layer 27 quadrant 1
    *((volatile uint32_t *)0x52101b04) = 0x00010003; // Rows
    *((volatile uint32_t *)0x52101b08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52101b18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x52101b20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52101b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52101b28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52101b2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52101b30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x52101b34) = 0x001d8040; // Layer control 2
    *((volatile uint32_t *)0x52101b38) = 0x00003e38; // Mask count
    *((volatile uint32_t *)0x52101b3c) = 0x000034e0; // Mask offset
    *((volatile uint32_t *)0x52101b40) = 0x0000012b; // Output channel count
    *((volatile uint32_t *)0x52101b0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x52101b4c) = 0x000232d0; // Post processing register
    *((volatile uint32_t *)0x52101b48) = 0xffffffff; // Mask and processor enables

    // Layer 27 quadrant 2
    *((volatile uint32_t *)0x53101b04) = 0x00010003; // Rows
    *((volatile uint32_t *)0x53101b08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53101b18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x53101b20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53101b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53101b28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53101b2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53101b30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x53101b34) = 0x001d8040; // Layer control 2
    *((volatile uint32_t *)0x53101b38) = 0x00003e38; // Mask count
    *((volatile uint32_t *)0x53101b3c) = 0x000034e0; // Mask offset
    *((volatile uint32_t *)0x53101b40) = 0x0000012b; // Output channel count
    *((volatile uint32_t *)0x53101b0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x53101b4c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x53101b48) = 0xffffffff; // Mask and processor enables

    // Layer 27 quadrant 3
    *((volatile uint32_t *)0x54101b04) = 0x00010003; // Rows
    *((volatile uint32_t *)0x54101b08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54101b18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x54101b20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54101b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54101b28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54101b2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54101b30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x54101b34) = 0x001d8040; // Layer control 2
    *((volatile uint32_t *)0x54101b38) = 0x00003e38; // Mask count
    *((volatile uint32_t *)0x54101b3c) = 0x000034e0; // Mask offset
    *((volatile uint32_t *)0x54101b40) = 0x0000012b; // Output channel count
    *((volatile uint32_t *)0x54101b0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x54101b4c) = 0x00022000; // Post processing register

    // Layer 28 quadrant 0
    *((volatile uint32_t *)0x51101c04) = 0x00058003; // Rows
    *((volatile uint32_t *)0x51101c08) = 0x00018003; // Columns
    *((volatile uint32_t *)0x51101c18) = 0x00000050; // Stride
    *((volatile uint32_t *)0x51101c1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x51101c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51101c28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51101c30) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x51101c34) = 0x001d8044; // Layer control 2
    *((volatile uint32_t *)0x51101c38) = 0x00000720; // Mask count
    *((volatile uint32_t *)0x51101c3c) = 0x00000700; // Mask offset
    *((volatile uint32_t *)0x51101c40) = 0x00000013; // Output channel count
    *((volatile uint32_t *)0x51101c0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x51101c44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x51101c4c) = 0x41000000; // Post processing register
    *((volatile uint32_t *)0x51101c48) = 0xffffffff; // Mask and processor enables

    // Layer 28 quadrant 1
    *((volatile uint32_t *)0x52101c04) = 0x00058003; // Rows
    *((volatile uint32_t *)0x52101c08) = 0x00018003; // Columns
    *((volatile uint32_t *)0x52101c18) = 0x00000050; // Stride
    *((volatile uint32_t *)0x52101c1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x52101c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52101c28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52101c30) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x52101c34) = 0x001d8044; // Layer control 2
    *((volatile uint32_t *)0x52101c38) = 0x00000720; // Mask count
    *((volatile uint32_t *)0x52101c3c) = 0x00000700; // Mask offset
    *((volatile uint32_t *)0x52101c40) = 0x00000013; // Output channel count
    *((volatile uint32_t *)0x52101c0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x52101c44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x52101c4c) = 0x41000000; // Post processing register
    *((volatile uint32_t *)0x52101c48) = 0xffffffff; // Mask and processor enables

    // Layer 28 quadrant 2
    *((volatile uint32_t *)0x53101c04) = 0x00058003; // Rows
    *((volatile uint32_t *)0x53101c08) = 0x00018003; // Columns
    *((volatile uint32_t *)0x53101c18) = 0x00000050; // Stride
    *((volatile uint32_t *)0x53101c1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x53101c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53101c28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53101c30) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x53101c34) = 0x001d8044; // Layer control 2
    *((volatile uint32_t *)0x53101c38) = 0x00000720; // Mask count
    *((volatile uint32_t *)0x53101c3c) = 0x00000700; // Mask offset
    *((volatile uint32_t *)0x53101c40) = 0x00000013; // Output channel count
    *((volatile uint32_t *)0x53101c0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x53101c44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x53101c4c) = 0x41000000; // Post processing register
    *((volatile uint32_t *)0x53101c48) = 0xffffffff; // Mask and processor enables

    // Layer 28 quadrant 3
    *((volatile uint32_t *)0x54101c04) = 0x00058003; // Rows
    *((volatile uint32_t *)0x54101c08) = 0x00018003; // Columns
    *((volatile uint32_t *)0x54101c18) = 0x00000050; // Stride
    *((volatile uint32_t *)0x54101c1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x54101c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54101c28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54101c30) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x54101c34) = 0x001d8044; // Layer control 2
    *((volatile uint32_t *)0x54101c38) = 0x00000720; // Mask count
    *((volatile uint32_t *)0x54101c3c) = 0x00000700; // Mask offset
    *((volatile uint32_t *)0x54101c40) = 0x00000013; // Output channel count
    *((volatile uint32_t *)0x54101c0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x54101c44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x54101c4c) = 0x41000000; // Post processing register
    *((volatile uint32_t *)0x54101c48) = 0x0fff0fff; // Mask and processor enables

    // Layer 29 quadrant 0
    *((volatile uint32_t *)0x51101d04) = 0x00050003; // Rows
    *((volatile uint32_t *)0x51101d08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51101d18) = 0x00000050; // Stride
    *((volatile uint32_t *)0x51101d20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51101d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51101d2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x51101d30) = 0x0000e920; // Layer control
    *((volatile uint32_t *)0x51101d34) = 0x00178014; // Layer control 2
    *((volatile uint32_t *)0x51101d38) = 0x000048b8; // Mask count
    *((volatile uint32_t *)0x51101d3c) = 0x00004140; // Mask offset
    *((volatile uint32_t *)0x51101d40) = 0x000000ef; // Output channel count
    *((volatile uint32_t *)0x51101d0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51101d4c) = 0x00005600; // Post processing register
    *((volatile uint32_t *)0x51101d48) = 0xffffffff; // Mask and processor enables

    // Layer 29 quadrant 1
    *((volatile uint32_t *)0x52101d04) = 0x00050003; // Rows
    *((volatile uint32_t *)0x52101d08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52101d18) = 0x00000050; // Stride
    *((volatile uint32_t *)0x52101d20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52101d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52101d2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x52101d30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52101d34) = 0x00178014; // Layer control 2
    *((volatile uint32_t *)0x52101d38) = 0x000048b8; // Mask count
    *((volatile uint32_t *)0x52101d3c) = 0x00004140; // Mask offset
    *((volatile uint32_t *)0x52101d40) = 0x000000ef; // Output channel count
    *((volatile uint32_t *)0x52101d0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x52101d4c) = 0x00004000; // Post processing register
    *((volatile uint32_t *)0x52101d48) = 0xffffffff; // Mask and processor enables

    // Layer 29 quadrant 2
    *((volatile uint32_t *)0x53101d04) = 0x00050003; // Rows
    *((volatile uint32_t *)0x53101d08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53101d18) = 0x00000050; // Stride
    *((volatile uint32_t *)0x53101d20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53101d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53101d2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x53101d30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53101d34) = 0x00178014; // Layer control 2
    *((volatile uint32_t *)0x53101d38) = 0x000048b8; // Mask count
    *((volatile uint32_t *)0x53101d3c) = 0x00004140; // Mask offset
    *((volatile uint32_t *)0x53101d40) = 0x000000ef; // Output channel count
    *((volatile uint32_t *)0x53101d0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x53101d4c) = 0x00004000; // Post processing register
    *((volatile uint32_t *)0x53101d48) = 0xffffffff; // Mask and processor enables

    // Layer 29 quadrant 3
    *((volatile uint32_t *)0x54101d04) = 0x00050003; // Rows
    *((volatile uint32_t *)0x54101d08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54101d18) = 0x00000050; // Stride
    *((volatile uint32_t *)0x54101d20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54101d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54101d2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x54101d30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54101d34) = 0x00178014; // Layer control 2
    *((volatile uint32_t *)0x54101d38) = 0x000048b8; // Mask count
    *((volatile uint32_t *)0x54101d3c) = 0x00004140; // Mask offset
    *((volatile uint32_t *)0x54101d40) = 0x000000ef; // Output channel count
    *((volatile uint32_t *)0x54101d0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x54101d4c) = 0x00004000; // Post processing register
    *((volatile uint32_t *)0x54101d48) = 0x0fff0fff; // Mask and processor enables

    // Layer 30 quadrant 0
    *((volatile uint32_t *)0x51101e04) = 0x00010003; // Rows
    *((volatile uint32_t *)0x51101e08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51101e18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x51101e1c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x51101e20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x51101e2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51101e30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51101e34) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x51101e40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x51101e0c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x51101e4c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x51101e48) = 0x0000ffff; // Mask and processor enables

    // Layer 30 quadrant 1
    *((volatile uint32_t *)0x52101e04) = 0x00010003; // Rows
    *((volatile uint32_t *)0x52101e08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52101e18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x52101e1c) = 0x00020001; // SRAM write ptr
    *((volatile uint32_t *)0x52101e20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x52101e2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52101e30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52101e34) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x52101e40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x52101e0c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x52101e4c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x52101e48) = 0x0000ffff; // Mask and processor enables

    // Layer 30 quadrant 2
    *((volatile uint32_t *)0x53101e04) = 0x00010003; // Rows
    *((volatile uint32_t *)0x53101e08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53101e18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x53101e1c) = 0x00040001; // SRAM write ptr
    *((volatile uint32_t *)0x53101e20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x53101e2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53101e30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53101e34) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x53101e40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x53101e0c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x53101e4c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x53101e48) = 0x0000ffff; // Mask and processor enables

    // Layer 30 quadrant 3
    *((volatile uint32_t *)0x54101e04) = 0x00010003; // Rows
    *((volatile uint32_t *)0x54101e08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54101e18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x54101e1c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x54101e20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x54101e2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54101e30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54101e34) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x54101e40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x54101e0c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x54101e4c) = 0x03000000; // Post processing register

    // Layer 31 quadrant 0
    *((volatile uint32_t *)0x51101f04) = 0x00020003; // Rows
    *((volatile uint32_t *)0x51101f08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51101f18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x51101f1c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x51101f20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x51101f30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51101f40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x51101f0c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x51101f4c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x51101f48) = 0x0000ffff; // Mask and processor enables

    // Layer 31 quadrant 1
    *((volatile uint32_t *)0x52101f04) = 0x00020003; // Rows
    *((volatile uint32_t *)0x52101f08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52101f18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x52101f1c) = 0x00021000; // SRAM write ptr
    *((volatile uint32_t *)0x52101f20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x52101f30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52101f40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x52101f0c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x52101f4c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x52101f48) = 0x0000ffff; // Mask and processor enables

    // Layer 31 quadrant 2
    *((volatile uint32_t *)0x53101f04) = 0x00020003; // Rows
    *((volatile uint32_t *)0x53101f08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53101f18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x53101f1c) = 0x00041000; // SRAM write ptr
    *((volatile uint32_t *)0x53101f20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x53101f30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53101f40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x53101f0c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x53101f4c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x53101f48) = 0x0000ffff; // Mask and processor enables

    // Layer 31 quadrant 3
    *((volatile uint32_t *)0x54101f04) = 0x00020003; // Rows
    *((volatile uint32_t *)0x54101f08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54101f18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x54101f1c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x54101f20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x54101f30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54101f40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x54101f0c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x54101f4c) = 0x01000000; // Post processing register

    // Layer 32 quadrant 0
    *((volatile uint32_t *)0x51102004) = 0x00010003; // Rows
    *((volatile uint32_t *)0x51102008) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51102018) = 0x00000010; // Stride
    *((volatile uint32_t *)0x51102020) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51102024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51102028) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110202c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51102030) = 0x00006b20; // Layer control
    *((volatile uint32_t *)0x51102034) = 0x001d8040; // Layer control 2
    *((volatile uint32_t *)0x51102038) = 0x00005278; // Mask count
    *((volatile uint32_t *)0x5110203c) = 0x00004920; // Mask offset
    *((volatile uint32_t *)0x51102040) = 0x0000012b; // Output channel count
    *((volatile uint32_t *)0x5110200c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5110204c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x51102048) = 0xffffffff; // Mask and processor enables

    // Layer 32 quadrant 1
    *((volatile uint32_t *)0x52102004) = 0x00010003; // Rows
    *((volatile uint32_t *)0x52102008) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52102018) = 0x00000010; // Stride
    *((volatile uint32_t *)0x52102020) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52102024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52102028) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210202c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52102030) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x52102034) = 0x001d8040; // Layer control 2
    *((volatile uint32_t *)0x52102038) = 0x00005278; // Mask count
    *((volatile uint32_t *)0x5210203c) = 0x00004920; // Mask offset
    *((volatile uint32_t *)0x52102040) = 0x0000012b; // Output channel count
    *((volatile uint32_t *)0x5210200c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210204c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x52102048) = 0xffffffff; // Mask and processor enables

    // Layer 32 quadrant 2
    *((volatile uint32_t *)0x53102004) = 0x00010003; // Rows
    *((volatile uint32_t *)0x53102008) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53102018) = 0x00000010; // Stride
    *((volatile uint32_t *)0x53102020) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53102024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53102028) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310202c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53102030) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x53102034) = 0x001d8040; // Layer control 2
    *((volatile uint32_t *)0x53102038) = 0x00005278; // Mask count
    *((volatile uint32_t *)0x5310203c) = 0x00004920; // Mask offset
    *((volatile uint32_t *)0x53102040) = 0x0000012b; // Output channel count
    *((volatile uint32_t *)0x5310200c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310204c) = 0x000232d0; // Post processing register
    *((volatile uint32_t *)0x53102048) = 0xffffffff; // Mask and processor enables

    // Layer 32 quadrant 3
    *((volatile uint32_t *)0x54102004) = 0x00010003; // Rows
    *((volatile uint32_t *)0x54102008) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54102018) = 0x00000010; // Stride
    *((volatile uint32_t *)0x54102020) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54102024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54102028) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410202c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54102030) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x54102034) = 0x001d8040; // Layer control 2
    *((volatile uint32_t *)0x54102038) = 0x00005278; // Mask count
    *((volatile uint32_t *)0x5410203c) = 0x00004920; // Mask offset
    *((volatile uint32_t *)0x54102040) = 0x0000012b; // Output channel count
    *((volatile uint32_t *)0x5410200c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410204c) = 0x00022000; // Post processing register

    // Layer 33 quadrant 0
    *((volatile uint32_t *)0x51102104) = 0x00058003; // Rows
    *((volatile uint32_t *)0x51102108) = 0x00018003; // Columns
    *((volatile uint32_t *)0x51102118) = 0x00000050; // Stride
    *((volatile uint32_t *)0x5110211c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x51102124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51102128) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51102130) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x51102134) = 0x001d8044; // Layer control 2
    *((volatile uint32_t *)0x51102138) = 0x00000960; // Mask count
    *((volatile uint32_t *)0x5110213c) = 0x00000940; // Mask offset
    *((volatile uint32_t *)0x51102140) = 0x00000013; // Output channel count
    *((volatile uint32_t *)0x5110210c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x51102144) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5110214c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x51102148) = 0xffffffff; // Mask and processor enables

    // Layer 33 quadrant 1
    *((volatile uint32_t *)0x52102104) = 0x00058003; // Rows
    *((volatile uint32_t *)0x52102108) = 0x00018003; // Columns
    *((volatile uint32_t *)0x52102118) = 0x00000050; // Stride
    *((volatile uint32_t *)0x5210211c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x52102124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52102128) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52102130) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x52102134) = 0x001d8044; // Layer control 2
    *((volatile uint32_t *)0x52102138) = 0x00000960; // Mask count
    *((volatile uint32_t *)0x5210213c) = 0x00000940; // Mask offset
    *((volatile uint32_t *)0x52102140) = 0x00000013; // Output channel count
    *((volatile uint32_t *)0x5210210c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x52102144) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5210214c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x52102148) = 0xffffffff; // Mask and processor enables

    // Layer 33 quadrant 2
    *((volatile uint32_t *)0x53102104) = 0x00058003; // Rows
    *((volatile uint32_t *)0x53102108) = 0x00018003; // Columns
    *((volatile uint32_t *)0x53102118) = 0x00000050; // Stride
    *((volatile uint32_t *)0x5310211c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x53102124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53102128) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53102130) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x53102134) = 0x001d8044; // Layer control 2
    *((volatile uint32_t *)0x53102138) = 0x00000960; // Mask count
    *((volatile uint32_t *)0x5310213c) = 0x00000940; // Mask offset
    *((volatile uint32_t *)0x53102140) = 0x00000013; // Output channel count
    *((volatile uint32_t *)0x5310210c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x53102144) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5310214c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x53102148) = 0xffffffff; // Mask and processor enables

    // Layer 33 quadrant 3
    *((volatile uint32_t *)0x54102104) = 0x00058003; // Rows
    *((volatile uint32_t *)0x54102108) = 0x00018003; // Columns
    *((volatile uint32_t *)0x54102118) = 0x00000050; // Stride
    *((volatile uint32_t *)0x5410211c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x54102124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54102128) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54102130) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x54102134) = 0x001d8044; // Layer control 2
    *((volatile uint32_t *)0x54102138) = 0x00000960; // Mask count
    *((volatile uint32_t *)0x5410213c) = 0x00000940; // Mask offset
    *((volatile uint32_t *)0x54102140) = 0x00000013; // Output channel count
    *((volatile uint32_t *)0x5410210c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x54102144) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5410214c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x54102148) = 0x0fff0fff; // Mask and processor enables

    // Layer 34 quadrant 0
    *((volatile uint32_t *)0x51102204) = 0x00050003; // Rows
    *((volatile uint32_t *)0x51102208) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51102218) = 0x00000050; // Stride
    *((volatile uint32_t *)0x51102220) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51102224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5110222c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x51102230) = 0x0000e920; // Layer control
    *((volatile uint32_t *)0x51102234) = 0x00178014; // Layer control 2
    *((volatile uint32_t *)0x51102238) = 0x00005cf8; // Mask count
    *((volatile uint32_t *)0x5110223c) = 0x00005580; // Mask offset
    *((volatile uint32_t *)0x51102240) = 0x000000ef; // Output channel count
    *((volatile uint32_t *)0x5110220c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5110224c) = 0x00004000; // Post processing register
    *((volatile uint32_t *)0x51102248) = 0xffffffff; // Mask and processor enables

    // Layer 34 quadrant 1
    *((volatile uint32_t *)0x52102204) = 0x00050003; // Rows
    *((volatile uint32_t *)0x52102208) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52102218) = 0x00000050; // Stride
    *((volatile uint32_t *)0x52102220) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52102224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5210222c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x52102230) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52102234) = 0x00178014; // Layer control 2
    *((volatile uint32_t *)0x52102238) = 0x00005cf8; // Mask count
    *((volatile uint32_t *)0x5210223c) = 0x00005580; // Mask offset
    *((volatile uint32_t *)0x52102240) = 0x000000ef; // Output channel count
    *((volatile uint32_t *)0x5210220c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210224c) = 0x00005618; // Post processing register
    *((volatile uint32_t *)0x52102248) = 0xffffffff; // Mask and processor enables

    // Layer 34 quadrant 2
    *((volatile uint32_t *)0x53102204) = 0x00050003; // Rows
    *((volatile uint32_t *)0x53102208) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53102218) = 0x00000050; // Stride
    *((volatile uint32_t *)0x53102220) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53102224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5310222c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x53102230) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53102234) = 0x00178014; // Layer control 2
    *((volatile uint32_t *)0x53102238) = 0x00005cf8; // Mask count
    *((volatile uint32_t *)0x5310223c) = 0x00005580; // Mask offset
    *((volatile uint32_t *)0x53102240) = 0x000000ef; // Output channel count
    *((volatile uint32_t *)0x5310220c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310224c) = 0x00004000; // Post processing register
    *((volatile uint32_t *)0x53102248) = 0xffffffff; // Mask and processor enables

    // Layer 34 quadrant 3
    *((volatile uint32_t *)0x54102204) = 0x00050003; // Rows
    *((volatile uint32_t *)0x54102208) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54102218) = 0x00000050; // Stride
    *((volatile uint32_t *)0x54102220) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54102224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5410222c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x54102230) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54102234) = 0x00178014; // Layer control 2
    *((volatile uint32_t *)0x54102238) = 0x00005cf8; // Mask count
    *((volatile uint32_t *)0x5410223c) = 0x00005580; // Mask offset
    *((volatile uint32_t *)0x54102240) = 0x000000ef; // Output channel count
    *((volatile uint32_t *)0x5410220c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410224c) = 0x00004000; // Post processing register
    *((volatile uint32_t *)0x54102248) = 0x0fff0fff; // Mask and processor enables

    // Layer 35 quadrant 0
    *((volatile uint32_t *)0x51102304) = 0x00010003; // Rows
    *((volatile uint32_t *)0x51102308) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51102318) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5110231c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x51102320) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x5110232c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51102330) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51102334) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x51102340) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5110230c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5110234c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x51102348) = 0x0000ffff; // Mask and processor enables

    // Layer 35 quadrant 1
    *((volatile uint32_t *)0x52102304) = 0x00010003; // Rows
    *((volatile uint32_t *)0x52102308) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52102318) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5210231c) = 0x00020001; // SRAM write ptr
    *((volatile uint32_t *)0x52102320) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x5210232c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52102330) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52102334) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x52102340) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5210230c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5210234c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x52102348) = 0x0000ffff; // Mask and processor enables

    // Layer 35 quadrant 2
    *((volatile uint32_t *)0x53102304) = 0x00010003; // Rows
    *((volatile uint32_t *)0x53102308) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53102318) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5310231c) = 0x00040001; // SRAM write ptr
    *((volatile uint32_t *)0x53102320) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x5310232c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53102330) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53102334) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x53102340) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5310230c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5310234c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x53102348) = 0x0000ffff; // Mask and processor enables

    // Layer 35 quadrant 3
    *((volatile uint32_t *)0x54102304) = 0x00010003; // Rows
    *((volatile uint32_t *)0x54102308) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54102318) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5410231c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x54102320) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x5410232c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54102330) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54102334) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x54102340) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5410230c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5410234c) = 0x03000000; // Post processing register

    // Layer 36 quadrant 0
    *((volatile uint32_t *)0x51102404) = 0x00020003; // Rows
    *((volatile uint32_t *)0x51102408) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51102418) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5110241c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x51102420) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x51102430) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51102440) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5110240c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5110244c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x51102448) = 0x0000ffff; // Mask and processor enables

    // Layer 36 quadrant 1
    *((volatile uint32_t *)0x52102404) = 0x00020003; // Rows
    *((volatile uint32_t *)0x52102408) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52102418) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5210241c) = 0x00021000; // SRAM write ptr
    *((volatile uint32_t *)0x52102420) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x52102430) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52102440) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5210240c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5210244c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x52102448) = 0x0000ffff; // Mask and processor enables

    // Layer 36 quadrant 2
    *((volatile uint32_t *)0x53102404) = 0x00020003; // Rows
    *((volatile uint32_t *)0x53102408) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53102418) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5310241c) = 0x00041000; // SRAM write ptr
    *((volatile uint32_t *)0x53102420) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x53102430) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53102440) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5310240c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5310244c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x53102448) = 0x0000ffff; // Mask and processor enables

    // Layer 36 quadrant 3
    *((volatile uint32_t *)0x54102404) = 0x00020003; // Rows
    *((volatile uint32_t *)0x54102408) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54102418) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5410241c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x54102420) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x54102430) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54102440) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5410240c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5410244c) = 0x01000000; // Post processing register

    // Layer 37 quadrant 0
    *((volatile uint32_t *)0x51102504) = 0x00010003; // Rows
    *((volatile uint32_t *)0x51102508) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51102518) = 0x00000010; // Stride
    *((volatile uint32_t *)0x51102520) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51102524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51102528) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110252c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51102530) = 0x0000eb20; // Layer control
    *((volatile uint32_t *)0x51102534) = 0x001d8040; // Layer control 2
    *((volatile uint32_t *)0x51102538) = 0x000066b8; // Mask count
    *((volatile uint32_t *)0x5110253c) = 0x00005d60; // Mask offset
    *((volatile uint32_t *)0x51102540) = 0x0000012b; // Output channel count
    *((volatile uint32_t *)0x5110250c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5110254c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x51102548) = 0xffffffff; // Mask and processor enables

    // Layer 37 quadrant 1
    *((volatile uint32_t *)0x52102504) = 0x00010003; // Rows
    *((volatile uint32_t *)0x52102508) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52102518) = 0x00000010; // Stride
    *((volatile uint32_t *)0x52102520) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52102524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52102528) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210252c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52102530) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x52102534) = 0x001d8040; // Layer control 2
    *((volatile uint32_t *)0x52102538) = 0x000066b8; // Mask count
    *((volatile uint32_t *)0x5210253c) = 0x00005d60; // Mask offset
    *((volatile uint32_t *)0x52102540) = 0x0000012b; // Output channel count
    *((volatile uint32_t *)0x5210250c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210254c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x52102548) = 0xffffffff; // Mask and processor enables

    // Layer 37 quadrant 2
    *((volatile uint32_t *)0x53102504) = 0x00010003; // Rows
    *((volatile uint32_t *)0x53102508) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53102518) = 0x00000010; // Stride
    *((volatile uint32_t *)0x53102520) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53102524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53102528) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310252c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53102530) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x53102534) = 0x001d8040; // Layer control 2
    *((volatile uint32_t *)0x53102538) = 0x000066b8; // Mask count
    *((volatile uint32_t *)0x5310253c) = 0x00005d60; // Mask offset
    *((volatile uint32_t *)0x53102540) = 0x0000012b; // Output channel count
    *((volatile uint32_t *)0x5310250c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310254c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x53102548) = 0xffffffff; // Mask and processor enables

    // Layer 37 quadrant 3
    *((volatile uint32_t *)0x54102504) = 0x00010003; // Rows
    *((volatile uint32_t *)0x54102508) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54102518) = 0x00000010; // Stride
    *((volatile uint32_t *)0x54102520) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54102524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54102528) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410252c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54102530) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x54102534) = 0x001d8040; // Layer control 2
    *((volatile uint32_t *)0x54102538) = 0x000066b8; // Mask count
    *((volatile uint32_t *)0x5410253c) = 0x00005d60; // Mask offset
    *((volatile uint32_t *)0x54102540) = 0x0000012b; // Output channel count
    *((volatile uint32_t *)0x5410250c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410254c) = 0x00023360; // Post processing register

    // Layer 38 quadrant 0
    *((volatile uint32_t *)0x51102604) = 0x00058003; // Rows
    *((volatile uint32_t *)0x51102608) = 0x00018003; // Columns
    *((volatile uint32_t *)0x51102618) = 0x00000050; // Stride
    *((volatile uint32_t *)0x5110261c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x51102624) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51102628) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51102630) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x51102634) = 0x001d8044; // Layer control 2
    *((volatile uint32_t *)0x51102638) = 0x00000ba0; // Mask count
    *((volatile uint32_t *)0x5110263c) = 0x00000b80; // Mask offset
    *((volatile uint32_t *)0x51102640) = 0x00000013; // Output channel count
    *((volatile uint32_t *)0x5110260c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x51102644) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5110264c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x51102648) = 0xffffffff; // Mask and processor enables

    // Layer 38 quadrant 1
    *((volatile uint32_t *)0x52102604) = 0x00058003; // Rows
    *((volatile uint32_t *)0x52102608) = 0x00018003; // Columns
    *((volatile uint32_t *)0x52102618) = 0x00000050; // Stride
    *((volatile uint32_t *)0x5210261c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x52102624) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52102628) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52102630) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x52102634) = 0x001d8044; // Layer control 2
    *((volatile uint32_t *)0x52102638) = 0x00000ba0; // Mask count
    *((volatile uint32_t *)0x5210263c) = 0x00000b80; // Mask offset
    *((volatile uint32_t *)0x52102640) = 0x00000013; // Output channel count
    *((volatile uint32_t *)0x5210260c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x52102644) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5210264c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x52102648) = 0xffffffff; // Mask and processor enables

    // Layer 38 quadrant 2
    *((volatile uint32_t *)0x53102604) = 0x00058003; // Rows
    *((volatile uint32_t *)0x53102608) = 0x00018003; // Columns
    *((volatile uint32_t *)0x53102618) = 0x00000050; // Stride
    *((volatile uint32_t *)0x5310261c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x53102624) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53102628) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53102630) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x53102634) = 0x001d8044; // Layer control 2
    *((volatile uint32_t *)0x53102638) = 0x00000ba0; // Mask count
    *((volatile uint32_t *)0x5310263c) = 0x00000b80; // Mask offset
    *((volatile uint32_t *)0x53102640) = 0x00000013; // Output channel count
    *((volatile uint32_t *)0x5310260c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x53102644) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5310264c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x53102648) = 0xffffffff; // Mask and processor enables

    // Layer 38 quadrant 3
    *((volatile uint32_t *)0x54102604) = 0x00058003; // Rows
    *((volatile uint32_t *)0x54102608) = 0x00018003; // Columns
    *((volatile uint32_t *)0x54102618) = 0x00000050; // Stride
    *((volatile uint32_t *)0x5410261c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x54102624) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54102628) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54102630) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x54102634) = 0x001d8044; // Layer control 2
    *((volatile uint32_t *)0x54102638) = 0x00000ba0; // Mask count
    *((volatile uint32_t *)0x5410263c) = 0x00000b80; // Mask offset
    *((volatile uint32_t *)0x54102640) = 0x00000013; // Output channel count
    *((volatile uint32_t *)0x5410260c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x54102644) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5410264c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x54102648) = 0x0fff0fff; // Mask and processor enables

    // Layer 39 quadrant 0
    *((volatile uint32_t *)0x51102704) = 0x00050003; // Rows
    *((volatile uint32_t *)0x51102708) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51102718) = 0x00000050; // Stride
    *((volatile uint32_t *)0x51102720) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51102724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5110272c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x51102730) = 0x0000e920; // Layer control
    *((volatile uint32_t *)0x51102734) = 0x00178014; // Layer control 2
    *((volatile uint32_t *)0x51102738) = 0x00007138; // Mask count
    *((volatile uint32_t *)0x5110273c) = 0x000069c0; // Mask offset
    *((volatile uint32_t *)0x51102740) = 0x000000ef; // Output channel count
    *((volatile uint32_t *)0x5110270c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5110274c) = 0x00006000; // Post processing register
    *((volatile uint32_t *)0x51102748) = 0xffffffff; // Mask and processor enables

    // Layer 39 quadrant 1
    *((volatile uint32_t *)0x52102704) = 0x00050003; // Rows
    *((volatile uint32_t *)0x52102708) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52102718) = 0x00000050; // Stride
    *((volatile uint32_t *)0x52102720) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52102724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5210272c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x52102730) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52102734) = 0x00178014; // Layer control 2
    *((volatile uint32_t *)0x52102738) = 0x00007138; // Mask count
    *((volatile uint32_t *)0x5210273c) = 0x000069c0; // Mask offset
    *((volatile uint32_t *)0x52102740) = 0x000000ef; // Output channel count
    *((volatile uint32_t *)0x5210270c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210274c) = 0x00006000; // Post processing register
    *((volatile uint32_t *)0x52102748) = 0xffffffff; // Mask and processor enables

    // Layer 39 quadrant 2
    *((volatile uint32_t *)0x53102704) = 0x00050003; // Rows
    *((volatile uint32_t *)0x53102708) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53102718) = 0x00000050; // Stride
    *((volatile uint32_t *)0x53102720) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53102724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5310272c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x53102730) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53102734) = 0x00178014; // Layer control 2
    *((volatile uint32_t *)0x53102738) = 0x00007138; // Mask count
    *((volatile uint32_t *)0x5310273c) = 0x000069c0; // Mask offset
    *((volatile uint32_t *)0x53102740) = 0x000000ef; // Output channel count
    *((volatile uint32_t *)0x5310270c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310274c) = 0x00007618; // Post processing register
    *((volatile uint32_t *)0x53102748) = 0xffffffff; // Mask and processor enables

    // Layer 39 quadrant 3
    *((volatile uint32_t *)0x54102704) = 0x00050003; // Rows
    *((volatile uint32_t *)0x54102708) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54102718) = 0x00000050; // Stride
    *((volatile uint32_t *)0x54102720) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54102724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5410272c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x54102730) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54102734) = 0x00178014; // Layer control 2
    *((volatile uint32_t *)0x54102738) = 0x00007138; // Mask count
    *((volatile uint32_t *)0x5410273c) = 0x000069c0; // Mask offset
    *((volatile uint32_t *)0x54102740) = 0x000000ef; // Output channel count
    *((volatile uint32_t *)0x5410270c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410274c) = 0x00006000; // Post processing register
    *((volatile uint32_t *)0x54102748) = 0x0fff0fff; // Mask and processor enables

    // Layer 40 quadrant 0
    *((volatile uint32_t *)0x51102804) = 0x00010003; // Rows
    *((volatile uint32_t *)0x51102808) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51102818) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5110281c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x51102820) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x5110282c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51102830) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51102834) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x51102840) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5110280c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5110284c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x51102848) = 0x0000ffff; // Mask and processor enables

    // Layer 40 quadrant 1
    *((volatile uint32_t *)0x52102804) = 0x00010003; // Rows
    *((volatile uint32_t *)0x52102808) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52102818) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5210281c) = 0x00020001; // SRAM write ptr
    *((volatile uint32_t *)0x52102820) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x5210282c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52102830) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52102834) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x52102840) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5210280c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5210284c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x52102848) = 0x0000ffff; // Mask and processor enables

    // Layer 40 quadrant 2
    *((volatile uint32_t *)0x53102804) = 0x00010003; // Rows
    *((volatile uint32_t *)0x53102808) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53102818) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5310281c) = 0x00040001; // SRAM write ptr
    *((volatile uint32_t *)0x53102820) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x5310282c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53102830) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53102834) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x53102840) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5310280c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5310284c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x53102848) = 0x0000ffff; // Mask and processor enables

    // Layer 40 quadrant 3
    *((volatile uint32_t *)0x54102804) = 0x00010003; // Rows
    *((volatile uint32_t *)0x54102808) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54102818) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5410281c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x54102820) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x5410282c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54102830) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54102834) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x54102840) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5410280c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5410284c) = 0x03000000; // Post processing register

    // Layer 41 quadrant 0
    *((volatile uint32_t *)0x51102904) = 0x00020003; // Rows
    *((volatile uint32_t *)0x51102908) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51102918) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5110291c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x51102920) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x51102930) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51102940) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5110290c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5110294c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x51102948) = 0x0000ffff; // Mask and processor enables

    // Layer 41 quadrant 1
    *((volatile uint32_t *)0x52102904) = 0x00020003; // Rows
    *((volatile uint32_t *)0x52102908) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52102918) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5210291c) = 0x00021000; // SRAM write ptr
    *((volatile uint32_t *)0x52102920) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x52102930) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52102940) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5210290c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5210294c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x52102948) = 0x0000ffff; // Mask and processor enables

    // Layer 41 quadrant 2
    *((volatile uint32_t *)0x53102904) = 0x00020003; // Rows
    *((volatile uint32_t *)0x53102908) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53102918) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5310291c) = 0x00041000; // SRAM write ptr
    *((volatile uint32_t *)0x53102920) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x53102930) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53102940) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5310290c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5310294c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x53102948) = 0x0000ffff; // Mask and processor enables

    // Layer 41 quadrant 3
    *((volatile uint32_t *)0x54102904) = 0x00020003; // Rows
    *((volatile uint32_t *)0x54102908) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54102918) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5410291c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x54102920) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x54102930) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54102940) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5410290c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5410294c) = 0x01000000; // Post processing register

    // Layer 42 quadrant 0
    *((volatile uint32_t *)0x51102a04) = 0x00010003; // Rows
    *((volatile uint32_t *)0x51102a08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51102a18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x51102a1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x51102a20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51102a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51102a28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51102a2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51102a30) = 0x00006b20; // Layer control
    *((volatile uint32_t *)0x51102a34) = 0x001d8040; // Layer control 2
    *((volatile uint32_t *)0x51102a38) = 0x00007af8; // Mask count
    *((volatile uint32_t *)0x51102a3c) = 0x000071a0; // Mask offset
    *((volatile uint32_t *)0x51102a40) = 0x0000012b; // Output channel count
    *((volatile uint32_t *)0x51102a0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51102a4c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x51102a48) = 0xffffffff; // Mask and processor enables

    // Layer 42 quadrant 1
    *((volatile uint32_t *)0x52102a04) = 0x00010003; // Rows
    *((volatile uint32_t *)0x52102a08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52102a18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x52102a1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x52102a20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52102a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52102a28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52102a2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52102a30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x52102a34) = 0x001d8040; // Layer control 2
    *((volatile uint32_t *)0x52102a38) = 0x00007af8; // Mask count
    *((volatile uint32_t *)0x52102a3c) = 0x000071a0; // Mask offset
    *((volatile uint32_t *)0x52102a40) = 0x0000012b; // Output channel count
    *((volatile uint32_t *)0x52102a0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x52102a4c) = 0x000233f0; // Post processing register
    *((volatile uint32_t *)0x52102a48) = 0xffffffff; // Mask and processor enables

    // Layer 42 quadrant 2
    *((volatile uint32_t *)0x53102a04) = 0x00010003; // Rows
    *((volatile uint32_t *)0x53102a08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53102a18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x53102a1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x53102a20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53102a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53102a28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53102a2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53102a30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x53102a34) = 0x001d8040; // Layer control 2
    *((volatile uint32_t *)0x53102a38) = 0x00007af8; // Mask count
    *((volatile uint32_t *)0x53102a3c) = 0x000071a0; // Mask offset
    *((volatile uint32_t *)0x53102a40) = 0x0000012b; // Output channel count
    *((volatile uint32_t *)0x53102a0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x53102a4c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x53102a48) = 0xffffffff; // Mask and processor enables

    // Layer 42 quadrant 3
    *((volatile uint32_t *)0x54102a04) = 0x00010003; // Rows
    *((volatile uint32_t *)0x54102a08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54102a18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x54102a1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x54102a20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54102a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54102a28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54102a2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54102a30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x54102a34) = 0x001d8040; // Layer control 2
    *((volatile uint32_t *)0x54102a38) = 0x00007af8; // Mask count
    *((volatile uint32_t *)0x54102a3c) = 0x000071a0; // Mask offset
    *((volatile uint32_t *)0x54102a40) = 0x0000012b; // Output channel count
    *((volatile uint32_t *)0x54102a0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x54102a4c) = 0x00022000; // Post processing register

    // Layer 43 quadrant 0
    *((volatile uint32_t *)0x51102b04) = 0x00058003; // Rows
    *((volatile uint32_t *)0x51102b08) = 0x00018003; // Columns
    *((volatile uint32_t *)0x51102b18) = 0x00000050; // Stride
    *((volatile uint32_t *)0x51102b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51102b28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51102b2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x51102b30) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x51102b34) = 0x001d8044; // Layer control 2
    *((volatile uint32_t *)0x51102b38) = 0x00000de0; // Mask count
    *((volatile uint32_t *)0x51102b3c) = 0x00000dc0; // Mask offset
    *((volatile uint32_t *)0x51102b40) = 0x00000013; // Output channel count
    *((volatile uint32_t *)0x51102b0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x51102b44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x51102b4c) = 0x41000000; // Post processing register
    *((volatile uint32_t *)0x51102b48) = 0xffffffff; // Mask and processor enables

    // Layer 43 quadrant 1
    *((volatile uint32_t *)0x52102b04) = 0x00058003; // Rows
    *((volatile uint32_t *)0x52102b08) = 0x00018003; // Columns
    *((volatile uint32_t *)0x52102b18) = 0x00000050; // Stride
    *((volatile uint32_t *)0x52102b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52102b28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52102b2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x52102b30) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x52102b34) = 0x001d8044; // Layer control 2
    *((volatile uint32_t *)0x52102b38) = 0x00000de0; // Mask count
    *((volatile uint32_t *)0x52102b3c) = 0x00000dc0; // Mask offset
    *((volatile uint32_t *)0x52102b40) = 0x00000013; // Output channel count
    *((volatile uint32_t *)0x52102b0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x52102b44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x52102b4c) = 0x41000000; // Post processing register
    *((volatile uint32_t *)0x52102b48) = 0xffffffff; // Mask and processor enables

    // Layer 43 quadrant 2
    *((volatile uint32_t *)0x53102b04) = 0x00058003; // Rows
    *((volatile uint32_t *)0x53102b08) = 0x00018003; // Columns
    *((volatile uint32_t *)0x53102b18) = 0x00000050; // Stride
    *((volatile uint32_t *)0x53102b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53102b28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53102b2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x53102b30) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x53102b34) = 0x001d8044; // Layer control 2
    *((volatile uint32_t *)0x53102b38) = 0x00000de0; // Mask count
    *((volatile uint32_t *)0x53102b3c) = 0x00000dc0; // Mask offset
    *((volatile uint32_t *)0x53102b40) = 0x00000013; // Output channel count
    *((volatile uint32_t *)0x53102b0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x53102b44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x53102b4c) = 0x41000000; // Post processing register
    *((volatile uint32_t *)0x53102b48) = 0xffffffff; // Mask and processor enables

    // Layer 43 quadrant 3
    *((volatile uint32_t *)0x54102b04) = 0x00058003; // Rows
    *((volatile uint32_t *)0x54102b08) = 0x00018003; // Columns
    *((volatile uint32_t *)0x54102b18) = 0x00000050; // Stride
    *((volatile uint32_t *)0x54102b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54102b28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54102b2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x54102b30) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x54102b34) = 0x001d8044; // Layer control 2
    *((volatile uint32_t *)0x54102b38) = 0x00000de0; // Mask count
    *((volatile uint32_t *)0x54102b3c) = 0x00000dc0; // Mask offset
    *((volatile uint32_t *)0x54102b40) = 0x00000013; // Output channel count
    *((volatile uint32_t *)0x54102b0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x54102b44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x54102b4c) = 0x41000000; // Post processing register
    *((volatile uint32_t *)0x54102b48) = 0x0fff0fff; // Mask and processor enables

    // Layer 44 quadrant 0
    *((volatile uint32_t *)0x51102c04) = 0x00050003; // Rows
    *((volatile uint32_t *)0x51102c08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51102c18) = 0x00000050; // Stride
    *((volatile uint32_t *)0x51102c1c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x51102c20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51102c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51102c28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51102c30) = 0x0000e920; // Layer control
    *((volatile uint32_t *)0x51102c34) = 0x00118014; // Layer control 2
    *((volatile uint32_t *)0x51102c38) = 0x00008938; // Mask count
    *((volatile uint32_t *)0x51102c3c) = 0x00007e00; // Mask offset
    *((volatile uint32_t *)0x51102c40) = 0x00000167; // Output channel count
    *((volatile uint32_t *)0x51102c0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51102c48) = 0xffffffff; // Mask and processor enables

    // Layer 44 quadrant 1
    *((volatile uint32_t *)0x52102c04) = 0x00050003; // Rows
    *((volatile uint32_t *)0x52102c08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52102c18) = 0x00000050; // Stride
    *((volatile uint32_t *)0x52102c1c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x52102c20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52102c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52102c28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52102c30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52102c34) = 0x00118014; // Layer control 2
    *((volatile uint32_t *)0x52102c38) = 0x00008938; // Mask count
    *((volatile uint32_t *)0x52102c3c) = 0x00007e00; // Mask offset
    *((volatile uint32_t *)0x52102c40) = 0x00000167; // Output channel count
    *((volatile uint32_t *)0x52102c0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x52102c48) = 0xffffffff; // Mask and processor enables

    // Layer 44 quadrant 2
    *((volatile uint32_t *)0x53102c04) = 0x00050003; // Rows
    *((volatile uint32_t *)0x53102c08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53102c18) = 0x00000050; // Stride
    *((volatile uint32_t *)0x53102c1c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x53102c20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53102c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53102c28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53102c30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53102c34) = 0x00118014; // Layer control 2
    *((volatile uint32_t *)0x53102c38) = 0x00008938; // Mask count
    *((volatile uint32_t *)0x53102c3c) = 0x00007e00; // Mask offset
    *((volatile uint32_t *)0x53102c40) = 0x00000167; // Output channel count
    *((volatile uint32_t *)0x53102c0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x53102c48) = 0xffffffff; // Mask and processor enables

    // Layer 44 quadrant 3
    *((volatile uint32_t *)0x54102c04) = 0x00050003; // Rows
    *((volatile uint32_t *)0x54102c08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54102c18) = 0x00000050; // Stride
    *((volatile uint32_t *)0x54102c1c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x54102c20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54102c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54102c28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54102c30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54102c34) = 0x00118014; // Layer control 2
    *((volatile uint32_t *)0x54102c38) = 0x00008938; // Mask count
    *((volatile uint32_t *)0x54102c3c) = 0x00007e00; // Mask offset
    *((volatile uint32_t *)0x54102c40) = 0x00000167; // Output channel count
    *((volatile uint32_t *)0x54102c0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x54102c4c) = 0x00001588; // Post processing register
    *((volatile uint32_t *)0x54102c48) = 0x0fff0fff; // Mask and processor enables

    // Layer 45 quadrant 0
    *((volatile uint32_t *)0x51102d04) = 0x00020003; // Rows
    *((volatile uint32_t *)0x51102d08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51102d18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x51102d20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51102d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51102d28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51102d2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51102d30) = 0x0000eb20; // Layer control
    *((volatile uint32_t *)0x51102d34) = 0x001f8061; // Layer control 2
    *((volatile uint32_t *)0x51102d38) = 0x0000a538; // Mask count
    *((volatile uint32_t *)0x51102d3c) = 0x00008940; // Mask offset
    *((volatile uint32_t *)0x51102d40) = 0x0000037f; // Output channel count
    *((volatile uint32_t *)0x51102d0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51102d4c) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x51102d48) = 0xffffffff; // Mask and processor enables

    // Layer 45 quadrant 1
    *((volatile uint32_t *)0x52102d04) = 0x00020003; // Rows
    *((volatile uint32_t *)0x52102d08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52102d18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x52102d20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52102d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52102d28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52102d2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52102d30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x52102d34) = 0x001f8061; // Layer control 2
    *((volatile uint32_t *)0x52102d38) = 0x0000a538; // Mask count
    *((volatile uint32_t *)0x52102d3c) = 0x00008940; // Mask offset
    *((volatile uint32_t *)0x52102d40) = 0x0000037f; // Output channel count
    *((volatile uint32_t *)0x52102d0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x52102d4c) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x52102d48) = 0xffffffff; // Mask and processor enables

    // Layer 45 quadrant 2
    *((volatile uint32_t *)0x53102d04) = 0x00020003; // Rows
    *((volatile uint32_t *)0x53102d08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53102d18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x53102d20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53102d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53102d28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53102d2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53102d30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x53102d34) = 0x001f8061; // Layer control 2
    *((volatile uint32_t *)0x53102d38) = 0x0000a538; // Mask count
    *((volatile uint32_t *)0x53102d3c) = 0x00008940; // Mask offset
    *((volatile uint32_t *)0x53102d40) = 0x0000037f; // Output channel count
    *((volatile uint32_t *)0x53102d0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x53102d4c) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x53102d48) = 0x000f000f; // Mask and processor enables

    // Layer 45 quadrant 3
    *((volatile uint32_t *)0x54102d04) = 0x00020003; // Rows
    *((volatile uint32_t *)0x54102d08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54102d18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x54102d20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54102d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54102d28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54102d2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54102d30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x54102d34) = 0x001f8061; // Layer control 2
    *((volatile uint32_t *)0x54102d38) = 0x0000a538; // Mask count
    *((volatile uint32_t *)0x54102d3c) = 0x00008940; // Mask offset
    *((volatile uint32_t *)0x54102d40) = 0x0000037f; // Output channel count
    *((volatile uint32_t *)0x54102d0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x54102d4c) = 0x00025000; // Post processing register

    // Layer 46 quadrant 0
    *((volatile uint32_t *)0x51102e04) = 0x00078003; // Rows
    *((volatile uint32_t *)0x51102e08) = 0x00018003; // Columns
    *((volatile uint32_t *)0x51102e18) = 0x00000070; // Stride
    *((volatile uint32_t *)0x51102e1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x51102e24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51102e28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51102e30) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x51102e34) = 0x001f8066; // Layer control 2
    *((volatile uint32_t *)0x51102e38) = 0x00001290; // Mask count
    *((volatile uint32_t *)0x51102e3c) = 0x00001260; // Mask offset
    *((volatile uint32_t *)0x51102e40) = 0x0000001b; // Output channel count
    *((volatile uint32_t *)0x51102e0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x51102e44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x51102e4c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x51102e48) = 0xffffffff; // Mask and processor enables

    // Layer 46 quadrant 1
    *((volatile uint32_t *)0x52102e04) = 0x00078003; // Rows
    *((volatile uint32_t *)0x52102e08) = 0x00018003; // Columns
    *((volatile uint32_t *)0x52102e18) = 0x00000070; // Stride
    *((volatile uint32_t *)0x52102e1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x52102e24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52102e28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52102e30) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x52102e34) = 0x001f8066; // Layer control 2
    *((volatile uint32_t *)0x52102e38) = 0x00001290; // Mask count
    *((volatile uint32_t *)0x52102e3c) = 0x00001260; // Mask offset
    *((volatile uint32_t *)0x52102e40) = 0x0000001b; // Output channel count
    *((volatile uint32_t *)0x52102e0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x52102e44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x52102e4c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x52102e48) = 0xffffffff; // Mask and processor enables

    // Layer 46 quadrant 2
    *((volatile uint32_t *)0x53102e04) = 0x00078003; // Rows
    *((volatile uint32_t *)0x53102e08) = 0x00018003; // Columns
    *((volatile uint32_t *)0x53102e18) = 0x00000070; // Stride
    *((volatile uint32_t *)0x53102e1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x53102e24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53102e28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53102e30) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x53102e34) = 0x001f8066; // Layer control 2
    *((volatile uint32_t *)0x53102e38) = 0x00001290; // Mask count
    *((volatile uint32_t *)0x53102e3c) = 0x00001260; // Mask offset
    *((volatile uint32_t *)0x53102e40) = 0x0000001b; // Output channel count
    *((volatile uint32_t *)0x53102e0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x53102e44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x53102e4c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x53102e48) = 0xffffffff; // Mask and processor enables

    // Layer 46 quadrant 3
    *((volatile uint32_t *)0x54102e04) = 0x00078003; // Rows
    *((volatile uint32_t *)0x54102e08) = 0x00018003; // Columns
    *((volatile uint32_t *)0x54102e18) = 0x00000070; // Stride
    *((volatile uint32_t *)0x54102e1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x54102e24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54102e28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54102e30) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x54102e34) = 0x001f8066; // Layer control 2
    *((volatile uint32_t *)0x54102e38) = 0x00001290; // Mask count
    *((volatile uint32_t *)0x54102e3c) = 0x00001260; // Mask offset
    *((volatile uint32_t *)0x54102e40) = 0x0000001b; // Output channel count
    *((volatile uint32_t *)0x54102e0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x54102e44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x54102e4c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x54102e48) = 0xffffffff; // Mask and processor enables

    // Layer 47 quadrant 0
    *((volatile uint32_t *)0x51102f04) = 0x00070003; // Rows
    *((volatile uint32_t *)0x51102f08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51102f18) = 0x00000070; // Stride
    *((volatile uint32_t *)0x51102f20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51102f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51102f28) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51102f2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x51102f30) = 0x0000e920; // Layer control
    *((volatile uint32_t *)0x51102f34) = 0x00118036; // Layer control 2
    *((volatile uint32_t *)0x51102f38) = 0x0000b758; // Mask count
    *((volatile uint32_t *)0x51102f3c) = 0x0000a7a0; // Mask offset
    *((volatile uint32_t *)0x51102f40) = 0x000001f7; // Output channel count
    *((volatile uint32_t *)0x51102f0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51102f4c) = 0x00006000; // Post processing register
    *((volatile uint32_t *)0x51102f48) = 0xffffffff; // Mask and processor enables

    // Layer 47 quadrant 1
    *((volatile uint32_t *)0x52102f04) = 0x00070003; // Rows
    *((volatile uint32_t *)0x52102f08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52102f18) = 0x00000070; // Stride
    *((volatile uint32_t *)0x52102f20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52102f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52102f28) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52102f2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x52102f30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52102f34) = 0x00118036; // Layer control 2
    *((volatile uint32_t *)0x52102f38) = 0x0000b758; // Mask count
    *((volatile uint32_t *)0x52102f3c) = 0x0000a7a0; // Mask offset
    *((volatile uint32_t *)0x52102f40) = 0x000001f7; // Output channel count
    *((volatile uint32_t *)0x52102f0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x52102f4c) = 0x000075d0; // Post processing register
    *((volatile uint32_t *)0x52102f48) = 0xffffffff; // Mask and processor enables

    // Layer 47 quadrant 2
    *((volatile uint32_t *)0x53102f04) = 0x00070003; // Rows
    *((volatile uint32_t *)0x53102f08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53102f18) = 0x00000070; // Stride
    *((volatile uint32_t *)0x53102f20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53102f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53102f28) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53102f2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x53102f30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53102f34) = 0x00118036; // Layer control 2
    *((volatile uint32_t *)0x53102f38) = 0x0000b758; // Mask count
    *((volatile uint32_t *)0x53102f3c) = 0x0000a7a0; // Mask offset
    *((volatile uint32_t *)0x53102f40) = 0x000001f7; // Output channel count
    *((volatile uint32_t *)0x53102f0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x53102f4c) = 0x00006000; // Post processing register
    *((volatile uint32_t *)0x53102f48) = 0xffffffff; // Mask and processor enables

    // Layer 47 quadrant 3
    *((volatile uint32_t *)0x54102f04) = 0x00070003; // Rows
    *((volatile uint32_t *)0x54102f08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54102f18) = 0x00000070; // Stride
    *((volatile uint32_t *)0x54102f20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54102f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54102f28) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54102f2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x54102f30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54102f34) = 0x00118036; // Layer control 2
    *((volatile uint32_t *)0x54102f38) = 0x0000b758; // Mask count
    *((volatile uint32_t *)0x54102f3c) = 0x0000a7a0; // Mask offset
    *((volatile uint32_t *)0x54102f40) = 0x000001f7; // Output channel count
    *((volatile uint32_t *)0x54102f0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x54102f4c) = 0x00006000; // Post processing register
    *((volatile uint32_t *)0x54102f48) = 0xffffffff; // Mask and processor enables

    // Layer 48 quadrant 0
    *((volatile uint32_t *)0x51103004) = 0x00020003; // Rows
    *((volatile uint32_t *)0x51103008) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51103018) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5110301c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x51103020) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x51103028) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110302c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51103030) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51103034) = 0x00000011; // Layer control 2
    *((volatile uint32_t *)0x5110303c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x51103040) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5110300c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5110304c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x51103048) = 0x0000ffff; // Mask and processor enables

    // Layer 48 quadrant 1
    *((volatile uint32_t *)0x52103004) = 0x00020003; // Rows
    *((volatile uint32_t *)0x52103008) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52103018) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5210301c) = 0x00020001; // SRAM write ptr
    *((volatile uint32_t *)0x52103020) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x52103028) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210302c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52103030) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52103034) = 0x00000011; // Layer control 2
    *((volatile uint32_t *)0x5210303c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x52103040) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5210300c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5210304c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x52103048) = 0x0000ffff; // Mask and processor enables

    // Layer 48 quadrant 2
    *((volatile uint32_t *)0x53103004) = 0x00020003; // Rows
    *((volatile uint32_t *)0x53103008) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53103018) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5310301c) = 0x00040001; // SRAM write ptr
    *((volatile uint32_t *)0x53103020) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x53103028) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310302c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53103030) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53103034) = 0x00000011; // Layer control 2
    *((volatile uint32_t *)0x5310303c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x53103040) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5310300c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5310304c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x53103048) = 0x0000000f; // Mask and processor enables

    // Layer 48 quadrant 3
    *((volatile uint32_t *)0x54103004) = 0x00020003; // Rows
    *((volatile uint32_t *)0x54103008) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54103018) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5410301c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x54103020) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x54103028) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410302c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54103030) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54103034) = 0x00000011; // Layer control 2
    *((volatile uint32_t *)0x5410303c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x54103040) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5410300c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5410304c) = 0x03000000; // Post processing register

    // Layer 49 quadrant 0
    *((volatile uint32_t *)0x51103104) = 0x00040003; // Rows
    *((volatile uint32_t *)0x51103108) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51103118) = 0x00000040; // Stride
    *((volatile uint32_t *)0x5110311c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x51103120) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x51103128) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51103130) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51103134) = 0x00000001; // Layer control 2
    *((volatile uint32_t *)0x5110313c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x51103140) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5110310c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5110314c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x51103148) = 0x0000ffff; // Mask and processor enables

    // Layer 49 quadrant 1
    *((volatile uint32_t *)0x52103104) = 0x00040003; // Rows
    *((volatile uint32_t *)0x52103108) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52103118) = 0x00000040; // Stride
    *((volatile uint32_t *)0x5210311c) = 0x00021000; // SRAM write ptr
    *((volatile uint32_t *)0x52103120) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x52103128) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52103130) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52103134) = 0x00000001; // Layer control 2
    *((volatile uint32_t *)0x5210313c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x52103140) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5210310c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5210314c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x52103148) = 0x0000ffff; // Mask and processor enables

    // Layer 49 quadrant 2
    *((volatile uint32_t *)0x53103104) = 0x00040003; // Rows
    *((volatile uint32_t *)0x53103108) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53103118) = 0x00000040; // Stride
    *((volatile uint32_t *)0x5310311c) = 0x00041000; // SRAM write ptr
    *((volatile uint32_t *)0x53103120) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x53103128) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53103130) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53103134) = 0x00000001; // Layer control 2
    *((volatile uint32_t *)0x5310313c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x53103140) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5310310c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5310314c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x53103148) = 0x0000000f; // Mask and processor enables

    // Layer 49 quadrant 3
    *((volatile uint32_t *)0x54103104) = 0x00040003; // Rows
    *((volatile uint32_t *)0x54103108) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54103118) = 0x00000040; // Stride
    *((volatile uint32_t *)0x5410311c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x54103120) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x54103128) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54103130) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54103134) = 0x00000001; // Layer control 2
    *((volatile uint32_t *)0x5410313c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x54103140) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5410310c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5410314c) = 0x01000000; // Post processing register

    // Layer 50 quadrant 0
    *((volatile uint32_t *)0x51103204) = 0x00020003; // Rows
    *((volatile uint32_t *)0x51103208) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51103218) = 0x00000020; // Stride
    *((volatile uint32_t *)0x51103220) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51103224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51103228) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110322c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51103230) = 0x0000eb20; // Layer control
    *((volatile uint32_t *)0x51103234) = 0x001f8061; // Layer control 2
    *((volatile uint32_t *)0x51103238) = 0x0000d358; // Mask count
    *((volatile uint32_t *)0x5110323c) = 0x0000b760; // Mask offset
    *((volatile uint32_t *)0x51103240) = 0x0000037f; // Output channel count
    *((volatile uint32_t *)0x5110320c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5110324c) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x51103248) = 0xffffffff; // Mask and processor enables

    // Layer 50 quadrant 1
    *((volatile uint32_t *)0x52103204) = 0x00020003; // Rows
    *((volatile uint32_t *)0x52103208) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52103218) = 0x00000020; // Stride
    *((volatile uint32_t *)0x52103220) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52103224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52103228) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210322c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52103230) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x52103234) = 0x001f8061; // Layer control 2
    *((volatile uint32_t *)0x52103238) = 0x0000d358; // Mask count
    *((volatile uint32_t *)0x5210323c) = 0x0000b760; // Mask offset
    *((volatile uint32_t *)0x52103240) = 0x0000037f; // Output channel count
    *((volatile uint32_t *)0x5210320c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210324c) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x52103248) = 0xffffffff; // Mask and processor enables

    // Layer 50 quadrant 2
    *((volatile uint32_t *)0x53103204) = 0x00020003; // Rows
    *((volatile uint32_t *)0x53103208) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53103218) = 0x00000020; // Stride
    *((volatile uint32_t *)0x53103220) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53103224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53103228) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310322c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53103230) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x53103234) = 0x001f8061; // Layer control 2
    *((volatile uint32_t *)0x53103238) = 0x0000d358; // Mask count
    *((volatile uint32_t *)0x5310323c) = 0x0000b760; // Mask offset
    *((volatile uint32_t *)0x53103240) = 0x0000037f; // Output channel count
    *((volatile uint32_t *)0x5310320c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310324c) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x53103248) = 0x000f000f; // Mask and processor enables

    // Layer 50 quadrant 3
    *((volatile uint32_t *)0x54103204) = 0x00020003; // Rows
    *((volatile uint32_t *)0x54103208) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54103218) = 0x00000020; // Stride
    *((volatile uint32_t *)0x54103220) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54103224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54103228) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410322c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54103230) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x54103234) = 0x001f8061; // Layer control 2
    *((volatile uint32_t *)0x54103238) = 0x0000d358; // Mask count
    *((volatile uint32_t *)0x5410323c) = 0x0000b760; // Mask offset
    *((volatile uint32_t *)0x54103240) = 0x0000037f; // Output channel count
    *((volatile uint32_t *)0x5410320c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410324c) = 0x000251b0; // Post processing register

    // Layer 51 quadrant 0
    *((volatile uint32_t *)0x51103304) = 0x00078003; // Rows
    *((volatile uint32_t *)0x51103308) = 0x00018003; // Columns
    *((volatile uint32_t *)0x51103318) = 0x00000070; // Stride
    *((volatile uint32_t *)0x5110331c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x51103324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51103328) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51103330) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x51103334) = 0x001f8066; // Layer control 2
    *((volatile uint32_t *)0x51103338) = 0x000017b0; // Mask count
    *((volatile uint32_t *)0x5110333c) = 0x00001780; // Mask offset
    *((volatile uint32_t *)0x51103340) = 0x0000001b; // Output channel count
    *((volatile uint32_t *)0x5110330c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x51103344) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5110334c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x51103348) = 0xffffffff; // Mask and processor enables

    // Layer 51 quadrant 1
    *((volatile uint32_t *)0x52103304) = 0x00078003; // Rows
    *((volatile uint32_t *)0x52103308) = 0x00018003; // Columns
    *((volatile uint32_t *)0x52103318) = 0x00000070; // Stride
    *((volatile uint32_t *)0x5210331c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x52103324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52103328) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52103330) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x52103334) = 0x001f8066; // Layer control 2
    *((volatile uint32_t *)0x52103338) = 0x000017b0; // Mask count
    *((volatile uint32_t *)0x5210333c) = 0x00001780; // Mask offset
    *((volatile uint32_t *)0x52103340) = 0x0000001b; // Output channel count
    *((volatile uint32_t *)0x5210330c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x52103344) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5210334c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x52103348) = 0xffffffff; // Mask and processor enables

    // Layer 51 quadrant 2
    *((volatile uint32_t *)0x53103304) = 0x00078003; // Rows
    *((volatile uint32_t *)0x53103308) = 0x00018003; // Columns
    *((volatile uint32_t *)0x53103318) = 0x00000070; // Stride
    *((volatile uint32_t *)0x5310331c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x53103324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53103328) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53103330) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x53103334) = 0x001f8066; // Layer control 2
    *((volatile uint32_t *)0x53103338) = 0x000017b0; // Mask count
    *((volatile uint32_t *)0x5310333c) = 0x00001780; // Mask offset
    *((volatile uint32_t *)0x53103340) = 0x0000001b; // Output channel count
    *((volatile uint32_t *)0x5310330c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x53103344) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5310334c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x53103348) = 0xffffffff; // Mask and processor enables

    // Layer 51 quadrant 3
    *((volatile uint32_t *)0x54103304) = 0x00078003; // Rows
    *((volatile uint32_t *)0x54103308) = 0x00018003; // Columns
    *((volatile uint32_t *)0x54103318) = 0x00000070; // Stride
    *((volatile uint32_t *)0x5410331c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x54103324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54103328) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54103330) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x54103334) = 0x001f8066; // Layer control 2
    *((volatile uint32_t *)0x54103338) = 0x000017b0; // Mask count
    *((volatile uint32_t *)0x5410333c) = 0x00001780; // Mask offset
    *((volatile uint32_t *)0x54103340) = 0x0000001b; // Output channel count
    *((volatile uint32_t *)0x5410330c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x54103344) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5410334c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x54103348) = 0xffffffff; // Mask and processor enables

    // Layer 52 quadrant 0
    *((volatile uint32_t *)0x51103404) = 0x00070003; // Rows
    *((volatile uint32_t *)0x51103408) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51103418) = 0x00000070; // Stride
    *((volatile uint32_t *)0x51103420) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51103424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51103428) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110342c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x51103430) = 0x0000e920; // Layer control
    *((volatile uint32_t *)0x51103434) = 0x00118036; // Layer control 2
    *((volatile uint32_t *)0x51103438) = 0x0000e578; // Mask count
    *((volatile uint32_t *)0x5110343c) = 0x0000d5c0; // Mask offset
    *((volatile uint32_t *)0x51103440) = 0x000001f7; // Output channel count
    *((volatile uint32_t *)0x5110340c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5110344c) = 0x00006000; // Post processing register
    *((volatile uint32_t *)0x51103448) = 0xffffffff; // Mask and processor enables

    // Layer 52 quadrant 1
    *((volatile uint32_t *)0x52103404) = 0x00070003; // Rows
    *((volatile uint32_t *)0x52103408) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52103418) = 0x00000070; // Stride
    *((volatile uint32_t *)0x52103420) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52103424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52103428) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210342c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x52103430) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52103434) = 0x00118036; // Layer control 2
    *((volatile uint32_t *)0x52103438) = 0x0000e578; // Mask count
    *((volatile uint32_t *)0x5210343c) = 0x0000d5c0; // Mask offset
    *((volatile uint32_t *)0x52103440) = 0x000001f7; // Output channel count
    *((volatile uint32_t *)0x5210340c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210344c) = 0x00006000; // Post processing register
    *((volatile uint32_t *)0x52103448) = 0xffffffff; // Mask and processor enables

    // Layer 52 quadrant 2
    *((volatile uint32_t *)0x53103404) = 0x00070003; // Rows
    *((volatile uint32_t *)0x53103408) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53103418) = 0x00000070; // Stride
    *((volatile uint32_t *)0x53103420) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53103424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53103428) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310342c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x53103430) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53103434) = 0x00118036; // Layer control 2
    *((volatile uint32_t *)0x53103438) = 0x0000e578; // Mask count
    *((volatile uint32_t *)0x5310343c) = 0x0000d5c0; // Mask offset
    *((volatile uint32_t *)0x53103440) = 0x000001f7; // Output channel count
    *((volatile uint32_t *)0x5310340c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310344c) = 0x00006000; // Post processing register
    *((volatile uint32_t *)0x53103448) = 0xffffffff; // Mask and processor enables

    // Layer 52 quadrant 3
    *((volatile uint32_t *)0x54103404) = 0x00070003; // Rows
    *((volatile uint32_t *)0x54103408) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54103418) = 0x00000070; // Stride
    *((volatile uint32_t *)0x54103420) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54103424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54103428) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410342c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x54103430) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54103434) = 0x00118036; // Layer control 2
    *((volatile uint32_t *)0x54103438) = 0x0000e578; // Mask count
    *((volatile uint32_t *)0x5410343c) = 0x0000d5c0; // Mask offset
    *((volatile uint32_t *)0x54103440) = 0x000001f7; // Output channel count
    *((volatile uint32_t *)0x5410340c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410344c) = 0x000075d0; // Post processing register
    *((volatile uint32_t *)0x54103448) = 0xffffffff; // Mask and processor enables

    // Layer 53 quadrant 0
    *((volatile uint32_t *)0x51103504) = 0x00020003; // Rows
    *((volatile uint32_t *)0x51103508) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51103518) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5110351c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x51103520) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x51103528) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110352c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51103530) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51103534) = 0x00000011; // Layer control 2
    *((volatile uint32_t *)0x5110353c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x51103540) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5110350c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5110354c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x51103548) = 0x0000ffff; // Mask and processor enables

    // Layer 53 quadrant 1
    *((volatile uint32_t *)0x52103504) = 0x00020003; // Rows
    *((volatile uint32_t *)0x52103508) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52103518) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5210351c) = 0x00020001; // SRAM write ptr
    *((volatile uint32_t *)0x52103520) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x52103528) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210352c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52103530) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52103534) = 0x00000011; // Layer control 2
    *((volatile uint32_t *)0x5210353c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x52103540) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5210350c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5210354c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x52103548) = 0x0000ffff; // Mask and processor enables

    // Layer 53 quadrant 2
    *((volatile uint32_t *)0x53103504) = 0x00020003; // Rows
    *((volatile uint32_t *)0x53103508) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53103518) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5310351c) = 0x00040001; // SRAM write ptr
    *((volatile uint32_t *)0x53103520) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x53103528) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310352c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53103530) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53103534) = 0x00000011; // Layer control 2
    *((volatile uint32_t *)0x5310353c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x53103540) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5310350c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5310354c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x53103548) = 0x0000000f; // Mask and processor enables

    // Layer 53 quadrant 3
    *((volatile uint32_t *)0x54103504) = 0x00020003; // Rows
    *((volatile uint32_t *)0x54103508) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54103518) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5410351c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x54103520) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x54103528) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410352c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54103530) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54103534) = 0x00000011; // Layer control 2
    *((volatile uint32_t *)0x5410353c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x54103540) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5410350c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5410354c) = 0x03000000; // Post processing register

    // Layer 54 quadrant 0
    *((volatile uint32_t *)0x51103604) = 0x00040003; // Rows
    *((volatile uint32_t *)0x51103608) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51103618) = 0x00000040; // Stride
    *((volatile uint32_t *)0x5110361c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x51103620) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x51103628) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51103630) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51103634) = 0x00000001; // Layer control 2
    *((volatile uint32_t *)0x5110363c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x51103640) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5110360c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5110364c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x51103648) = 0x0000ffff; // Mask and processor enables

    // Layer 54 quadrant 1
    *((volatile uint32_t *)0x52103604) = 0x00040003; // Rows
    *((volatile uint32_t *)0x52103608) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52103618) = 0x00000040; // Stride
    *((volatile uint32_t *)0x5210361c) = 0x00021000; // SRAM write ptr
    *((volatile uint32_t *)0x52103620) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x52103628) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52103630) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52103634) = 0x00000001; // Layer control 2
    *((volatile uint32_t *)0x5210363c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x52103640) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5210360c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5210364c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x52103648) = 0x0000ffff; // Mask and processor enables

    // Layer 54 quadrant 2
    *((volatile uint32_t *)0x53103604) = 0x00040003; // Rows
    *((volatile uint32_t *)0x53103608) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53103618) = 0x00000040; // Stride
    *((volatile uint32_t *)0x5310361c) = 0x00041000; // SRAM write ptr
    *((volatile uint32_t *)0x53103620) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x53103628) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53103630) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53103634) = 0x00000001; // Layer control 2
    *((volatile uint32_t *)0x5310363c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x53103640) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5310360c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5310364c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x53103648) = 0x0000000f; // Mask and processor enables

    // Layer 54 quadrant 3
    *((volatile uint32_t *)0x54103604) = 0x00040003; // Rows
    *((volatile uint32_t *)0x54103608) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54103618) = 0x00000040; // Stride
    *((volatile uint32_t *)0x5410361c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x54103620) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x54103628) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54103630) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54103634) = 0x00000001; // Layer control 2
    *((volatile uint32_t *)0x5410363c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x54103640) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5410360c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5410364c) = 0x01000000; // Post processing register

    // Layer 55 quadrant 0
    *((volatile uint32_t *)0x51103704) = 0x00020003; // Rows
    *((volatile uint32_t *)0x51103708) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51103718) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5110371c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x51103720) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51103724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51103728) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110372c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51103730) = 0x00006b20; // Layer control
    *((volatile uint32_t *)0x51103734) = 0x001f8061; // Layer control 2
    *((volatile uint32_t *)0x51103738) = 0x00010178; // Mask count
    *((volatile uint32_t *)0x5110373c) = 0x0000e580; // Mask offset
    *((volatile uint32_t *)0x51103740) = 0x0000037f; // Output channel count
    *((volatile uint32_t *)0x5110370c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5110374c) = 0x000252d0; // Post processing register
    *((volatile uint32_t *)0x51103748) = 0xffffffff; // Mask and processor enables

    // Layer 55 quadrant 1
    *((volatile uint32_t *)0x52103704) = 0x00020003; // Rows
    *((volatile uint32_t *)0x52103708) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52103718) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5210371c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x52103720) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52103724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52103728) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210372c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52103730) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x52103734) = 0x001f8061; // Layer control 2
    *((volatile uint32_t *)0x52103738) = 0x00010178; // Mask count
    *((volatile uint32_t *)0x5210373c) = 0x0000e580; // Mask offset
    *((volatile uint32_t *)0x52103740) = 0x0000037f; // Output channel count
    *((volatile uint32_t *)0x5210370c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210374c) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x52103748) = 0xffffffff; // Mask and processor enables

    // Layer 55 quadrant 2
    *((volatile uint32_t *)0x53103704) = 0x00020003; // Rows
    *((volatile uint32_t *)0x53103708) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53103718) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5310371c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x53103720) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53103724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53103728) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310372c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53103730) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x53103734) = 0x001f8061; // Layer control 2
    *((volatile uint32_t *)0x53103738) = 0x00010178; // Mask count
    *((volatile uint32_t *)0x5310373c) = 0x0000e580; // Mask offset
    *((volatile uint32_t *)0x53103740) = 0x0000037f; // Output channel count
    *((volatile uint32_t *)0x5310370c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310374c) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x53103748) = 0x000f000f; // Mask and processor enables

    // Layer 55 quadrant 3
    *((volatile uint32_t *)0x54103704) = 0x00020003; // Rows
    *((volatile uint32_t *)0x54103708) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54103718) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5410371c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x54103720) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54103724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54103728) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410372c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54103730) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x54103734) = 0x001f8061; // Layer control 2
    *((volatile uint32_t *)0x54103738) = 0x00010178; // Mask count
    *((volatile uint32_t *)0x5410373c) = 0x0000e580; // Mask offset
    *((volatile uint32_t *)0x54103740) = 0x0000037f; // Output channel count
    *((volatile uint32_t *)0x5410370c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410374c) = 0x00024000; // Post processing register

    // Layer 56 quadrant 0
    *((volatile uint32_t *)0x51103804) = 0x00078003; // Rows
    *((volatile uint32_t *)0x51103808) = 0x00018003; // Columns
    *((volatile uint32_t *)0x51103818) = 0x00000070; // Stride
    *((volatile uint32_t *)0x51103824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51103828) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110382c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x51103830) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x51103834) = 0x001f8066; // Layer control 2
    *((volatile uint32_t *)0x51103838) = 0x00001cd0; // Mask count
    *((volatile uint32_t *)0x5110383c) = 0x00001ca0; // Mask offset
    *((volatile uint32_t *)0x51103840) = 0x0000001b; // Output channel count
    *((volatile uint32_t *)0x5110380c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x51103844) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5110384c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x51103848) = 0xffffffff; // Mask and processor enables

    // Layer 56 quadrant 1
    *((volatile uint32_t *)0x52103804) = 0x00078003; // Rows
    *((volatile uint32_t *)0x52103808) = 0x00018003; // Columns
    *((volatile uint32_t *)0x52103818) = 0x00000070; // Stride
    *((volatile uint32_t *)0x52103824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52103828) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210382c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x52103830) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x52103834) = 0x001f8066; // Layer control 2
    *((volatile uint32_t *)0x52103838) = 0x00001cd0; // Mask count
    *((volatile uint32_t *)0x5210383c) = 0x00001ca0; // Mask offset
    *((volatile uint32_t *)0x52103840) = 0x0000001b; // Output channel count
    *((volatile uint32_t *)0x5210380c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x52103844) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5210384c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x52103848) = 0xffffffff; // Mask and processor enables

    // Layer 56 quadrant 2
    *((volatile uint32_t *)0x53103804) = 0x00078003; // Rows
    *((volatile uint32_t *)0x53103808) = 0x00018003; // Columns
    *((volatile uint32_t *)0x53103818) = 0x00000070; // Stride
    *((volatile uint32_t *)0x53103824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53103828) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310382c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x53103830) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x53103834) = 0x001f8066; // Layer control 2
    *((volatile uint32_t *)0x53103838) = 0x00001cd0; // Mask count
    *((volatile uint32_t *)0x5310383c) = 0x00001ca0; // Mask offset
    *((volatile uint32_t *)0x53103840) = 0x0000001b; // Output channel count
    *((volatile uint32_t *)0x5310380c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x53103844) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5310384c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x53103848) = 0xffffffff; // Mask and processor enables

    // Layer 56 quadrant 3
    *((volatile uint32_t *)0x54103804) = 0x00078003; // Rows
    *((volatile uint32_t *)0x54103808) = 0x00018003; // Columns
    *((volatile uint32_t *)0x54103818) = 0x00000070; // Stride
    *((volatile uint32_t *)0x54103824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54103828) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410382c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x54103830) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x54103834) = 0x001f8066; // Layer control 2
    *((volatile uint32_t *)0x54103838) = 0x00001cd0; // Mask count
    *((volatile uint32_t *)0x5410383c) = 0x00001ca0; // Mask offset
    *((volatile uint32_t *)0x54103840) = 0x0000001b; // Output channel count
    *((volatile uint32_t *)0x5410380c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x54103844) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5410384c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x54103848) = 0xffffffff; // Mask and processor enables

    // Layer 57 quadrant 0
    *((volatile uint32_t *)0x51103904) = 0x00070003; // Rows
    *((volatile uint32_t *)0x51103908) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51103918) = 0x00000070; // Stride
    *((volatile uint32_t *)0x5110391c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x51103920) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51103924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51103928) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51103930) = 0x0000e920; // Layer control
    *((volatile uint32_t *)0x51103934) = 0x001d8016; // Layer control 2
    *((volatile uint32_t *)0x51103938) = 0x00011e18; // Mask count
    *((volatile uint32_t *)0x5110393c) = 0x000103e0; // Mask offset
    *((volatile uint32_t *)0x51103940) = 0x00000347; // Output channel count
    *((volatile uint32_t *)0x5110390c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51103948) = 0xffffffff; // Mask and processor enables

    // Layer 57 quadrant 1
    *((volatile uint32_t *)0x52103904) = 0x00070003; // Rows
    *((volatile uint32_t *)0x52103908) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52103918) = 0x00000070; // Stride
    *((volatile uint32_t *)0x5210391c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x52103920) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52103924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52103928) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52103930) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52103934) = 0x001d8016; // Layer control 2
    *((volatile uint32_t *)0x52103938) = 0x00011e18; // Mask count
    *((volatile uint32_t *)0x5210393c) = 0x000103e0; // Mask offset
    *((volatile uint32_t *)0x52103940) = 0x00000347; // Output channel count
    *((volatile uint32_t *)0x5210390c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x52103948) = 0xffffffff; // Mask and processor enables

    // Layer 57 quadrant 2
    *((volatile uint32_t *)0x53103904) = 0x00070003; // Rows
    *((volatile uint32_t *)0x53103908) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53103918) = 0x00000070; // Stride
    *((volatile uint32_t *)0x5310391c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x53103920) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53103924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53103928) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53103930) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53103934) = 0x001d8016; // Layer control 2
    *((volatile uint32_t *)0x53103938) = 0x00011e18; // Mask count
    *((volatile uint32_t *)0x5310393c) = 0x000103e0; // Mask offset
    *((volatile uint32_t *)0x53103940) = 0x00000347; // Output channel count
    *((volatile uint32_t *)0x5310390c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x53103948) = 0xffffffff; // Mask and processor enables

    // Layer 57 quadrant 3
    *((volatile uint32_t *)0x54103904) = 0x00070003; // Rows
    *((volatile uint32_t *)0x54103908) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54103918) = 0x00000070; // Stride
    *((volatile uint32_t *)0x5410391c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x54103920) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54103924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54103928) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54103930) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54103934) = 0x001d8016; // Layer control 2
    *((volatile uint32_t *)0x54103938) = 0x00011e18; // Mask count
    *((volatile uint32_t *)0x5410393c) = 0x000103e0; // Mask offset
    *((volatile uint32_t *)0x54103940) = 0x00000347; // Output channel count
    *((volatile uint32_t *)0x5410390c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410394c) = 0x00001510; // Post processing register
    *((volatile uint32_t *)0x54103948) = 0xffffffff; // Mask and processor enables

    // Layer 58 quadrant 0
    *((volatile uint32_t *)0x51103a04) = 0x00020003; // Rows
    *((volatile uint32_t *)0x51103a08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51103a18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x51103a20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51103a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51103a28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51103a2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51103a30) = 0x0000eb20; // Layer control
    *((volatile uint32_t *)0x51103a34) = 0x001d80b1; // Layer control 2
    *((volatile uint32_t *)0x51103a38) = 0x00014bd8; // Mask count
    *((volatile uint32_t *)0x51103a3c) = 0x00011ee0; // Mask offset
    *((volatile uint32_t *)0x51103a40) = 0x0000059f; // Output channel count
    *((volatile uint32_t *)0x51103a0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51103a4c) = 0x00025000; // Post processing register
    *((volatile uint32_t *)0x51103a48) = 0xffffffff; // Mask and processor enables

    // Layer 58 quadrant 1
    *((volatile uint32_t *)0x52103a04) = 0x00020003; // Rows
    *((volatile uint32_t *)0x52103a08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52103a18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x52103a20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52103a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52103a28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52103a2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52103a30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x52103a34) = 0x001d80b1; // Layer control 2
    *((volatile uint32_t *)0x52103a38) = 0x00014bd8; // Mask count
    *((volatile uint32_t *)0x52103a3c) = 0x00011ee0; // Mask offset
    *((volatile uint32_t *)0x52103a40) = 0x0000059f; // Output channel count
    *((volatile uint32_t *)0x52103a0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x52103a4c) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x52103a48) = 0xffffffff; // Mask and processor enables

    // Layer 58 quadrant 2
    *((volatile uint32_t *)0x53103a04) = 0x00020003; // Rows
    *((volatile uint32_t *)0x53103a08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53103a18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x53103a20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53103a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53103a28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53103a2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53103a30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x53103a34) = 0x001d80b1; // Layer control 2
    *((volatile uint32_t *)0x53103a38) = 0x00014bd8; // Mask count
    *((volatile uint32_t *)0x53103a3c) = 0x00011ee0; // Mask offset
    *((volatile uint32_t *)0x53103a40) = 0x0000059f; // Output channel count
    *((volatile uint32_t *)0x53103a0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x53103a4c) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x53103a48) = 0xffffffff; // Mask and processor enables

    // Layer 58 quadrant 3
    *((volatile uint32_t *)0x54103a04) = 0x00020003; // Rows
    *((volatile uint32_t *)0x54103a08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54103a18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x54103a20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54103a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54103a28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54103a2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54103a30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x54103a34) = 0x001d80b1; // Layer control 2
    *((volatile uint32_t *)0x54103a38) = 0x00014bd8; // Mask count
    *((volatile uint32_t *)0x54103a3c) = 0x00011ee0; // Mask offset
    *((volatile uint32_t *)0x54103a40) = 0x0000059f; // Output channel count
    *((volatile uint32_t *)0x54103a0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x54103a4c) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x54103a48) = 0x0fff0fff; // Mask and processor enables

    // Layer 59 quadrant 0
    *((volatile uint32_t *)0x51103b04) = 0x000c8003; // Rows
    *((volatile uint32_t *)0x51103b08) = 0x00018003; // Columns
    *((volatile uint32_t *)0x51103b18) = 0x000000c0; // Stride
    *((volatile uint32_t *)0x51103b1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x51103b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51103b28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51103b30) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x51103b34) = 0x001d80bb; // Layer control 2
    *((volatile uint32_t *)0x51103b38) = 0x00002538; // Mask count
    *((volatile uint32_t *)0x51103b3c) = 0x000024e0; // Mask offset
    *((volatile uint32_t *)0x51103b40) = 0x0000002f; // Output channel count
    *((volatile uint32_t *)0x51103b0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x51103b44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x51103b4c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x51103b48) = 0xffffffff; // Mask and processor enables

    // Layer 59 quadrant 1
    *((volatile uint32_t *)0x52103b04) = 0x000c8003; // Rows
    *((volatile uint32_t *)0x52103b08) = 0x00018003; // Columns
    *((volatile uint32_t *)0x52103b18) = 0x000000c0; // Stride
    *((volatile uint32_t *)0x52103b1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x52103b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52103b28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52103b30) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x52103b34) = 0x001d80bb; // Layer control 2
    *((volatile uint32_t *)0x52103b38) = 0x00002538; // Mask count
    *((volatile uint32_t *)0x52103b3c) = 0x000024e0; // Mask offset
    *((volatile uint32_t *)0x52103b40) = 0x0000002f; // Output channel count
    *((volatile uint32_t *)0x52103b0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x52103b44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x52103b4c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x52103b48) = 0xffffffff; // Mask and processor enables

    // Layer 59 quadrant 2
    *((volatile uint32_t *)0x53103b04) = 0x000c8003; // Rows
    *((volatile uint32_t *)0x53103b08) = 0x00018003; // Columns
    *((volatile uint32_t *)0x53103b18) = 0x000000c0; // Stride
    *((volatile uint32_t *)0x53103b1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x53103b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53103b28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53103b30) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x53103b34) = 0x001d80bb; // Layer control 2
    *((volatile uint32_t *)0x53103b38) = 0x00002538; // Mask count
    *((volatile uint32_t *)0x53103b3c) = 0x000024e0; // Mask offset
    *((volatile uint32_t *)0x53103b40) = 0x0000002f; // Output channel count
    *((volatile uint32_t *)0x53103b0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x53103b44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x53103b4c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x53103b48) = 0xffffffff; // Mask and processor enables

    // Layer 59 quadrant 3
    *((volatile uint32_t *)0x54103b04) = 0x000c8003; // Rows
    *((volatile uint32_t *)0x54103b08) = 0x00018003; // Columns
    *((volatile uint32_t *)0x54103b18) = 0x000000c0; // Stride
    *((volatile uint32_t *)0x54103b1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x54103b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54103b28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54103b30) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x54103b34) = 0x001d80bb; // Layer control 2
    *((volatile uint32_t *)0x54103b38) = 0x00002538; // Mask count
    *((volatile uint32_t *)0x54103b3c) = 0x000024e0; // Mask offset
    *((volatile uint32_t *)0x54103b40) = 0x0000002f; // Output channel count
    *((volatile uint32_t *)0x54103b0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x54103b44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x54103b4c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x54103b48) = 0x0fff0fff; // Mask and processor enables

    // Layer 60 quadrant 0
    *((volatile uint32_t *)0x51103c04) = 0x000c0003; // Rows
    *((volatile uint32_t *)0x51103c08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51103c18) = 0x000000c0; // Stride
    *((volatile uint32_t *)0x51103c20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51103c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51103c28) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51103c2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x51103c30) = 0x0000e920; // Layer control
    *((volatile uint32_t *)0x51103c34) = 0x001d803b; // Layer control 2
    *((volatile uint32_t *)0x51103c38) = 0x00017c38; // Mask count
    *((volatile uint32_t *)0x51103c3c) = 0x00014f40; // Mask offset
    *((volatile uint32_t *)0x51103c40) = 0x0000059f; // Output channel count
    *((volatile uint32_t *)0x51103c0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51103c4c) = 0x00006000; // Post processing register
    *((volatile uint32_t *)0x51103c48) = 0xffffffff; // Mask and processor enables

    // Layer 60 quadrant 1
    *((volatile uint32_t *)0x52103c04) = 0x000c0003; // Rows
    *((volatile uint32_t *)0x52103c08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52103c18) = 0x000000c0; // Stride
    *((volatile uint32_t *)0x52103c20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52103c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52103c28) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52103c2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x52103c30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52103c34) = 0x001d803b; // Layer control 2
    *((volatile uint32_t *)0x52103c38) = 0x00017c38; // Mask count
    *((volatile uint32_t *)0x52103c3c) = 0x00014f40; // Mask offset
    *((volatile uint32_t *)0x52103c40) = 0x0000059f; // Output channel count
    *((volatile uint32_t *)0x52103c0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x52103c4c) = 0x00006000; // Post processing register
    *((volatile uint32_t *)0x52103c48) = 0xffffffff; // Mask and processor enables

    // Layer 60 quadrant 2
    *((volatile uint32_t *)0x53103c04) = 0x000c0003; // Rows
    *((volatile uint32_t *)0x53103c08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53103c18) = 0x000000c0; // Stride
    *((volatile uint32_t *)0x53103c20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53103c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53103c28) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53103c2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x53103c30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53103c34) = 0x001d803b; // Layer control 2
    *((volatile uint32_t *)0x53103c38) = 0x00017c38; // Mask count
    *((volatile uint32_t *)0x53103c3c) = 0x00014f40; // Mask offset
    *((volatile uint32_t *)0x53103c40) = 0x0000059f; // Output channel count
    *((volatile uint32_t *)0x53103c0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x53103c4c) = 0x00007570; // Post processing register
    *((volatile uint32_t *)0x53103c48) = 0xffffffff; // Mask and processor enables

    // Layer 60 quadrant 3
    *((volatile uint32_t *)0x54103c04) = 0x000c0003; // Rows
    *((volatile uint32_t *)0x54103c08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54103c18) = 0x000000c0; // Stride
    *((volatile uint32_t *)0x54103c20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54103c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54103c28) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54103c2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x54103c30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54103c34) = 0x001d803b; // Layer control 2
    *((volatile uint32_t *)0x54103c38) = 0x00017c38; // Mask count
    *((volatile uint32_t *)0x54103c3c) = 0x00014f40; // Mask offset
    *((volatile uint32_t *)0x54103c40) = 0x0000059f; // Output channel count
    *((volatile uint32_t *)0x54103c0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x54103c4c) = 0x00006000; // Post processing register
    *((volatile uint32_t *)0x54103c48) = 0x0fff0fff; // Mask and processor enables

    // Layer 61 quadrant 0
    *((volatile uint32_t *)0x51103d04) = 0x00020003; // Rows
    *((volatile uint32_t *)0x51103d08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51103d18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x51103d1c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x51103d20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x51103d28) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51103d2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51103d30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51103d34) = 0x00000011; // Layer control 2
    *((volatile uint32_t *)0x51103d3c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x51103d40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x51103d0c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x51103d4c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x51103d48) = 0x0000ffff; // Mask and processor enables

    // Layer 61 quadrant 1
    *((volatile uint32_t *)0x52103d04) = 0x00020003; // Rows
    *((volatile uint32_t *)0x52103d08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52103d18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x52103d1c) = 0x00020001; // SRAM write ptr
    *((volatile uint32_t *)0x52103d20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x52103d28) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52103d2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52103d30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52103d34) = 0x00000011; // Layer control 2
    *((volatile uint32_t *)0x52103d3c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x52103d40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x52103d0c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x52103d4c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x52103d48) = 0x0000ffff; // Mask and processor enables

    // Layer 61 quadrant 2
    *((volatile uint32_t *)0x53103d04) = 0x00020003; // Rows
    *((volatile uint32_t *)0x53103d08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53103d18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x53103d1c) = 0x00040001; // SRAM write ptr
    *((volatile uint32_t *)0x53103d20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x53103d28) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53103d2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53103d30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53103d34) = 0x00000011; // Layer control 2
    *((volatile uint32_t *)0x53103d3c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x53103d40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x53103d0c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x53103d4c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x53103d48) = 0x0000ffff; // Mask and processor enables

    // Layer 61 quadrant 3
    *((volatile uint32_t *)0x54103d04) = 0x00020003; // Rows
    *((volatile uint32_t *)0x54103d08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54103d18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x54103d1c) = 0x00060001; // SRAM write ptr
    *((volatile uint32_t *)0x54103d20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x54103d28) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54103d2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54103d30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54103d34) = 0x00000011; // Layer control 2
    *((volatile uint32_t *)0x54103d3c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x54103d40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x54103d0c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x54103d4c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x54103d48) = 0x00000fff; // Mask and processor enables

    // Layer 62 quadrant 0
    *((volatile uint32_t *)0x51103e04) = 0x00040003; // Rows
    *((volatile uint32_t *)0x51103e08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51103e18) = 0x00000040; // Stride
    *((volatile uint32_t *)0x51103e1c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x51103e20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x51103e28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51103e30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51103e34) = 0x00000001; // Layer control 2
    *((volatile uint32_t *)0x51103e3c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x51103e40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x51103e0c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x51103e4c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x51103e48) = 0x0000ffff; // Mask and processor enables

    // Layer 62 quadrant 1
    *((volatile uint32_t *)0x52103e04) = 0x00040003; // Rows
    *((volatile uint32_t *)0x52103e08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52103e18) = 0x00000040; // Stride
    *((volatile uint32_t *)0x52103e1c) = 0x00021000; // SRAM write ptr
    *((volatile uint32_t *)0x52103e20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x52103e28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52103e30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52103e34) = 0x00000001; // Layer control 2
    *((volatile uint32_t *)0x52103e3c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x52103e40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x52103e0c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x52103e4c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x52103e48) = 0x0000ffff; // Mask and processor enables

    // Layer 62 quadrant 2
    *((volatile uint32_t *)0x53103e04) = 0x00040003; // Rows
    *((volatile uint32_t *)0x53103e08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53103e18) = 0x00000040; // Stride
    *((volatile uint32_t *)0x53103e1c) = 0x00041000; // SRAM write ptr
    *((volatile uint32_t *)0x53103e20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x53103e28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53103e30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53103e34) = 0x00000001; // Layer control 2
    *((volatile uint32_t *)0x53103e3c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x53103e40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x53103e0c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x53103e4c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x53103e48) = 0x0000ffff; // Mask and processor enables

    // Layer 62 quadrant 3
    *((volatile uint32_t *)0x54103e04) = 0x00040003; // Rows
    *((volatile uint32_t *)0x54103e08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54103e18) = 0x00000040; // Stride
    *((volatile uint32_t *)0x54103e1c) = 0x00061000; // SRAM write ptr
    *((volatile uint32_t *)0x54103e20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x54103e28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54103e30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54103e34) = 0x00000001; // Layer control 2
    *((volatile uint32_t *)0x54103e3c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x54103e40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x54103e0c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x54103e4c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x54103e48) = 0x00000fff; // Mask and processor enables

    // Layer 63 quadrant 0
    *((volatile uint32_t *)0x51103f04) = 0x00020003; // Rows
    *((volatile uint32_t *)0x51103f08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51103f18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x51103f20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51103f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51103f28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51103f2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51103f30) = 0x0000eb20; // Layer control
    *((volatile uint32_t *)0x51103f34) = 0x001d80b1; // Layer control 2
    *((volatile uint32_t *)0x51103f38) = 0x0001a938; // Mask count
    *((volatile uint32_t *)0x51103f3c) = 0x00017c40; // Mask offset
    *((volatile uint32_t *)0x51103f40) = 0x0000059f; // Output channel count
    *((volatile uint32_t *)0x51103f0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51103f4c) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x51103f48) = 0xffffffff; // Mask and processor enables

    // Layer 63 quadrant 1
    *((volatile uint32_t *)0x52103f04) = 0x00020003; // Rows
    *((volatile uint32_t *)0x52103f08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52103f18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x52103f20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52103f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52103f28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52103f2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52103f30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x52103f34) = 0x001d80b1; // Layer control 2
    *((volatile uint32_t *)0x52103f38) = 0x0001a938; // Mask count
    *((volatile uint32_t *)0x52103f3c) = 0x00017c40; // Mask offset
    *((volatile uint32_t *)0x52103f40) = 0x0000059f; // Output channel count
    *((volatile uint32_t *)0x52103f0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x52103f4c) = 0x00025000; // Post processing register
    *((volatile uint32_t *)0x52103f48) = 0xffffffff; // Mask and processor enables

    // Layer 63 quadrant 2
    *((volatile uint32_t *)0x53103f04) = 0x00020003; // Rows
    *((volatile uint32_t *)0x53103f08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53103f18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x53103f20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53103f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53103f28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53103f2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53103f30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x53103f34) = 0x001d80b1; // Layer control 2
    *((volatile uint32_t *)0x53103f38) = 0x0001a938; // Mask count
    *((volatile uint32_t *)0x53103f3c) = 0x00017c40; // Mask offset
    *((volatile uint32_t *)0x53103f40) = 0x0000059f; // Output channel count
    *((volatile uint32_t *)0x53103f0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x53103f4c) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x53103f48) = 0xffffffff; // Mask and processor enables

    // Layer 63 quadrant 3
    *((volatile uint32_t *)0x54103f04) = 0x00020003; // Rows
    *((volatile uint32_t *)0x54103f08) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54103f18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x54103f20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54103f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54103f28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54103f2c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54103f30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x54103f34) = 0x001d80b1; // Layer control 2
    *((volatile uint32_t *)0x54103f38) = 0x0001a938; // Mask count
    *((volatile uint32_t *)0x54103f3c) = 0x00017c40; // Mask offset
    *((volatile uint32_t *)0x54103f40) = 0x0000059f; // Output channel count
    *((volatile uint32_t *)0x54103f0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x54103f4c) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x54103f48) = 0x0fff0fff; // Mask and processor enables

    // Layer 64 quadrant 0
    *((volatile uint32_t *)0x51104004) = 0x000c8003; // Rows
    *((volatile uint32_t *)0x51104008) = 0x00018003; // Columns
    *((volatile uint32_t *)0x51104018) = 0x000000c0; // Stride
    *((volatile uint32_t *)0x5110401c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x51104024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51104028) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51104030) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x51104034) = 0x001d80bb; // Layer control 2
    *((volatile uint32_t *)0x51104038) = 0x00002f98; // Mask count
    *((volatile uint32_t *)0x5110403c) = 0x00002f40; // Mask offset
    *((volatile uint32_t *)0x51104040) = 0x0000002f; // Output channel count
    *((volatile uint32_t *)0x5110400c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x51104044) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5110404c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x51104048) = 0xffffffff; // Mask and processor enables

    // Layer 64 quadrant 1
    *((volatile uint32_t *)0x52104004) = 0x000c8003; // Rows
    *((volatile uint32_t *)0x52104008) = 0x00018003; // Columns
    *((volatile uint32_t *)0x52104018) = 0x000000c0; // Stride
    *((volatile uint32_t *)0x5210401c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x52104024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52104028) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52104030) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x52104034) = 0x001d80bb; // Layer control 2
    *((volatile uint32_t *)0x52104038) = 0x00002f98; // Mask count
    *((volatile uint32_t *)0x5210403c) = 0x00002f40; // Mask offset
    *((volatile uint32_t *)0x52104040) = 0x0000002f; // Output channel count
    *((volatile uint32_t *)0x5210400c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x52104044) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5210404c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x52104048) = 0xffffffff; // Mask and processor enables

    // Layer 64 quadrant 2
    *((volatile uint32_t *)0x53104004) = 0x000c8003; // Rows
    *((volatile uint32_t *)0x53104008) = 0x00018003; // Columns
    *((volatile uint32_t *)0x53104018) = 0x000000c0; // Stride
    *((volatile uint32_t *)0x5310401c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x53104024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53104028) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53104030) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x53104034) = 0x001d80bb; // Layer control 2
    *((volatile uint32_t *)0x53104038) = 0x00002f98; // Mask count
    *((volatile uint32_t *)0x5310403c) = 0x00002f40; // Mask offset
    *((volatile uint32_t *)0x53104040) = 0x0000002f; // Output channel count
    *((volatile uint32_t *)0x5310400c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x53104044) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5310404c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x53104048) = 0xffffffff; // Mask and processor enables

    // Layer 64 quadrant 3
    *((volatile uint32_t *)0x54104004) = 0x000c8003; // Rows
    *((volatile uint32_t *)0x54104008) = 0x00018003; // Columns
    *((volatile uint32_t *)0x54104018) = 0x000000c0; // Stride
    *((volatile uint32_t *)0x5410401c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x54104024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54104028) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54104030) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x54104034) = 0x001d80bb; // Layer control 2
    *((volatile uint32_t *)0x54104038) = 0x00002f98; // Mask count
    *((volatile uint32_t *)0x5410403c) = 0x00002f40; // Mask offset
    *((volatile uint32_t *)0x54104040) = 0x0000002f; // Output channel count
    *((volatile uint32_t *)0x5410400c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x54104044) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5410404c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x54104048) = 0x0fff0fff; // Mask and processor enables

    // Layer 65 quadrant 0
    *((volatile uint32_t *)0x51104104) = 0x000c0003; // Rows
    *((volatile uint32_t *)0x51104108) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51104118) = 0x000000c0; // Stride
    *((volatile uint32_t *)0x51104120) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51104124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51104128) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110412c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x51104130) = 0x0000e920; // Layer control
    *((volatile uint32_t *)0x51104134) = 0x001d803b; // Layer control 2
    *((volatile uint32_t *)0x51104138) = 0x0001d998; // Mask count
    *((volatile uint32_t *)0x5110413c) = 0x0001aca0; // Mask offset
    *((volatile uint32_t *)0x51104140) = 0x0000059f; // Output channel count
    *((volatile uint32_t *)0x5110410c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5110414c) = 0x00007588; // Post processing register
    *((volatile uint32_t *)0x51104148) = 0xffffffff; // Mask and processor enables

    // Layer 65 quadrant 1
    *((volatile uint32_t *)0x52104104) = 0x000c0003; // Rows
    *((volatile uint32_t *)0x52104108) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52104118) = 0x000000c0; // Stride
    *((volatile uint32_t *)0x52104120) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52104124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52104128) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210412c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x52104130) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52104134) = 0x001d803b; // Layer control 2
    *((volatile uint32_t *)0x52104138) = 0x0001d998; // Mask count
    *((volatile uint32_t *)0x5210413c) = 0x0001aca0; // Mask offset
    *((volatile uint32_t *)0x52104140) = 0x0000059f; // Output channel count
    *((volatile uint32_t *)0x5210410c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210414c) = 0x00006000; // Post processing register
    *((volatile uint32_t *)0x52104148) = 0xffffffff; // Mask and processor enables

    // Layer 65 quadrant 2
    *((volatile uint32_t *)0x53104104) = 0x000c0003; // Rows
    *((volatile uint32_t *)0x53104108) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53104118) = 0x000000c0; // Stride
    *((volatile uint32_t *)0x53104120) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53104124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53104128) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310412c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x53104130) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53104134) = 0x001d803b; // Layer control 2
    *((volatile uint32_t *)0x53104138) = 0x0001d998; // Mask count
    *((volatile uint32_t *)0x5310413c) = 0x0001aca0; // Mask offset
    *((volatile uint32_t *)0x53104140) = 0x0000059f; // Output channel count
    *((volatile uint32_t *)0x5310410c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310414c) = 0x00006000; // Post processing register
    *((volatile uint32_t *)0x53104148) = 0xffffffff; // Mask and processor enables

    // Layer 65 quadrant 3
    *((volatile uint32_t *)0x54104104) = 0x000c0003; // Rows
    *((volatile uint32_t *)0x54104108) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54104118) = 0x000000c0; // Stride
    *((volatile uint32_t *)0x54104120) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54104124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54104128) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410412c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x54104130) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54104134) = 0x001d803b; // Layer control 2
    *((volatile uint32_t *)0x54104138) = 0x0001d998; // Mask count
    *((volatile uint32_t *)0x5410413c) = 0x0001aca0; // Mask offset
    *((volatile uint32_t *)0x54104140) = 0x0000059f; // Output channel count
    *((volatile uint32_t *)0x5410410c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410414c) = 0x00006000; // Post processing register
    *((volatile uint32_t *)0x54104148) = 0x0fff0fff; // Mask and processor enables

    // Layer 66 quadrant 0
    *((volatile uint32_t *)0x51104204) = 0x00020003; // Rows
    *((volatile uint32_t *)0x51104208) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51104218) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5110421c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x51104220) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x51104228) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110422c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51104230) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51104234) = 0x00000011; // Layer control 2
    *((volatile uint32_t *)0x5110423c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x51104240) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5110420c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5110424c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x51104248) = 0x0000ffff; // Mask and processor enables

    // Layer 66 quadrant 1
    *((volatile uint32_t *)0x52104204) = 0x00020003; // Rows
    *((volatile uint32_t *)0x52104208) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52104218) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5210421c) = 0x00020001; // SRAM write ptr
    *((volatile uint32_t *)0x52104220) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x52104228) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210422c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52104230) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52104234) = 0x00000011; // Layer control 2
    *((volatile uint32_t *)0x5210423c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x52104240) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5210420c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5210424c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x52104248) = 0x0000ffff; // Mask and processor enables

    // Layer 66 quadrant 2
    *((volatile uint32_t *)0x53104204) = 0x00020003; // Rows
    *((volatile uint32_t *)0x53104208) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53104218) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5310421c) = 0x00040001; // SRAM write ptr
    *((volatile uint32_t *)0x53104220) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x53104228) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310422c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53104230) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53104234) = 0x00000011; // Layer control 2
    *((volatile uint32_t *)0x5310423c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x53104240) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5310420c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5310424c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x53104248) = 0x0000ffff; // Mask and processor enables

    // Layer 66 quadrant 3
    *((volatile uint32_t *)0x54104204) = 0x00020003; // Rows
    *((volatile uint32_t *)0x54104208) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54104218) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5410421c) = 0x00060001; // SRAM write ptr
    *((volatile uint32_t *)0x54104220) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x54104228) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410422c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54104230) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54104234) = 0x00000011; // Layer control 2
    *((volatile uint32_t *)0x5410423c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x54104240) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5410420c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5410424c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x54104248) = 0x00000fff; // Mask and processor enables

    // Layer 67 quadrant 0
    *((volatile uint32_t *)0x51104304) = 0x00040003; // Rows
    *((volatile uint32_t *)0x51104308) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51104318) = 0x00000040; // Stride
    *((volatile uint32_t *)0x5110431c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x51104320) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x51104328) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51104330) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51104334) = 0x00000001; // Layer control 2
    *((volatile uint32_t *)0x5110433c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x51104340) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5110430c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5110434c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x51104348) = 0x0000ffff; // Mask and processor enables

    // Layer 67 quadrant 1
    *((volatile uint32_t *)0x52104304) = 0x00040003; // Rows
    *((volatile uint32_t *)0x52104308) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52104318) = 0x00000040; // Stride
    *((volatile uint32_t *)0x5210431c) = 0x00021000; // SRAM write ptr
    *((volatile uint32_t *)0x52104320) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x52104328) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52104330) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52104334) = 0x00000001; // Layer control 2
    *((volatile uint32_t *)0x5210433c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x52104340) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5210430c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5210434c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x52104348) = 0x0000ffff; // Mask and processor enables

    // Layer 67 quadrant 2
    *((volatile uint32_t *)0x53104304) = 0x00040003; // Rows
    *((volatile uint32_t *)0x53104308) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53104318) = 0x00000040; // Stride
    *((volatile uint32_t *)0x5310431c) = 0x00041000; // SRAM write ptr
    *((volatile uint32_t *)0x53104320) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x53104328) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53104330) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53104334) = 0x00000001; // Layer control 2
    *((volatile uint32_t *)0x5310433c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x53104340) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5310430c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5310434c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x53104348) = 0x0000ffff; // Mask and processor enables

    // Layer 67 quadrant 3
    *((volatile uint32_t *)0x54104304) = 0x00040003; // Rows
    *((volatile uint32_t *)0x54104308) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54104318) = 0x00000040; // Stride
    *((volatile uint32_t *)0x5410431c) = 0x00061000; // SRAM write ptr
    *((volatile uint32_t *)0x54104320) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x54104328) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54104330) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54104334) = 0x00000001; // Layer control 2
    *((volatile uint32_t *)0x5410433c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x54104340) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5410430c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5410434c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x54104348) = 0x00000fff; // Mask and processor enables

    // Layer 68 quadrant 0
    *((volatile uint32_t *)0x51104404) = 0x00020003; // Rows
    *((volatile uint32_t *)0x51104408) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51104418) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5110441c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x51104420) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51104424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51104428) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110442c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51104430) = 0x0000eb20; // Layer control
    *((volatile uint32_t *)0x51104434) = 0x001d80b1; // Layer control 2
    *((volatile uint32_t *)0x51104438) = 0x00020698; // Mask count
    *((volatile uint32_t *)0x5110443c) = 0x0001d9a0; // Mask offset
    *((volatile uint32_t *)0x51104440) = 0x0000059f; // Output channel count
    *((volatile uint32_t *)0x5110440c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5110444c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x51104448) = 0xffffffff; // Mask and processor enables

    // Layer 68 quadrant 1
    *((volatile uint32_t *)0x52104404) = 0x00020003; // Rows
    *((volatile uint32_t *)0x52104408) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52104418) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5210441c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x52104420) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52104424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52104428) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210442c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52104430) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x52104434) = 0x001d80b1; // Layer control 2
    *((volatile uint32_t *)0x52104438) = 0x00020698; // Mask count
    *((volatile uint32_t *)0x5210443c) = 0x0001d9a0; // Mask offset
    *((volatile uint32_t *)0x52104440) = 0x0000059f; // Output channel count
    *((volatile uint32_t *)0x5210440c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210444c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x52104448) = 0xffffffff; // Mask and processor enables

    // Layer 68 quadrant 2
    *((volatile uint32_t *)0x53104404) = 0x00020003; // Rows
    *((volatile uint32_t *)0x53104408) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53104418) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5310441c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x53104420) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53104424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53104428) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310442c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53104430) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x53104434) = 0x001d80b1; // Layer control 2
    *((volatile uint32_t *)0x53104438) = 0x00020698; // Mask count
    *((volatile uint32_t *)0x5310443c) = 0x0001d9a0; // Mask offset
    *((volatile uint32_t *)0x53104440) = 0x0000059f; // Output channel count
    *((volatile uint32_t *)0x5310440c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310444c) = 0x00023000; // Post processing register
    *((volatile uint32_t *)0x53104448) = 0xffffffff; // Mask and processor enables

    // Layer 68 quadrant 3
    *((volatile uint32_t *)0x54104404) = 0x00020003; // Rows
    *((volatile uint32_t *)0x54104408) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54104418) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5410441c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x54104420) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54104424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54104428) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410442c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54104430) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x54104434) = 0x001d80b1; // Layer control 2
    *((volatile uint32_t *)0x54104438) = 0x00020698; // Mask count
    *((volatile uint32_t *)0x5410443c) = 0x0001d9a0; // Mask offset
    *((volatile uint32_t *)0x54104440) = 0x0000059f; // Output channel count
    *((volatile uint32_t *)0x5410440c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410444c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x54104448) = 0x0fff0fff; // Mask and processor enables

    // Layer 69 quadrant 0
    *((volatile uint32_t *)0x51104504) = 0x000c8003; // Rows
    *((volatile uint32_t *)0x51104508) = 0x00018003; // Columns
    *((volatile uint32_t *)0x51104518) = 0x000000c0; // Stride
    *((volatile uint32_t *)0x51104524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51104528) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110452c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x51104530) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x51104534) = 0x001d80bb; // Layer control 2
    *((volatile uint32_t *)0x51104538) = 0x000039f8; // Mask count
    *((volatile uint32_t *)0x5110453c) = 0x000039a0; // Mask offset
    *((volatile uint32_t *)0x51104540) = 0x0000002f; // Output channel count
    *((volatile uint32_t *)0x5110450c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x51104544) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5110454c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x51104548) = 0xffffffff; // Mask and processor enables

    // Layer 69 quadrant 1
    *((volatile uint32_t *)0x52104504) = 0x000c8003; // Rows
    *((volatile uint32_t *)0x52104508) = 0x00018003; // Columns
    *((volatile uint32_t *)0x52104518) = 0x000000c0; // Stride
    *((volatile uint32_t *)0x52104524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52104528) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210452c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x52104530) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x52104534) = 0x001d80bb; // Layer control 2
    *((volatile uint32_t *)0x52104538) = 0x000039f8; // Mask count
    *((volatile uint32_t *)0x5210453c) = 0x000039a0; // Mask offset
    *((volatile uint32_t *)0x52104540) = 0x0000002f; // Output channel count
    *((volatile uint32_t *)0x5210450c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x52104544) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5210454c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x52104548) = 0xffffffff; // Mask and processor enables

    // Layer 69 quadrant 2
    *((volatile uint32_t *)0x53104504) = 0x000c8003; // Rows
    *((volatile uint32_t *)0x53104508) = 0x00018003; // Columns
    *((volatile uint32_t *)0x53104518) = 0x000000c0; // Stride
    *((volatile uint32_t *)0x53104524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53104528) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310452c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x53104530) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x53104534) = 0x001d80bb; // Layer control 2
    *((volatile uint32_t *)0x53104538) = 0x000039f8; // Mask count
    *((volatile uint32_t *)0x5310453c) = 0x000039a0; // Mask offset
    *((volatile uint32_t *)0x53104540) = 0x0000002f; // Output channel count
    *((volatile uint32_t *)0x5310450c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x53104544) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5310454c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x53104548) = 0xffffffff; // Mask and processor enables

    // Layer 69 quadrant 3
    *((volatile uint32_t *)0x54104504) = 0x000c8003; // Rows
    *((volatile uint32_t *)0x54104508) = 0x00018003; // Columns
    *((volatile uint32_t *)0x54104518) = 0x000000c0; // Stride
    *((volatile uint32_t *)0x54104524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54104528) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410452c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x54104530) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x54104534) = 0x001d80bb; // Layer control 2
    *((volatile uint32_t *)0x54104538) = 0x000039f8; // Mask count
    *((volatile uint32_t *)0x5410453c) = 0x000039a0; // Mask offset
    *((volatile uint32_t *)0x54104540) = 0x0000002f; // Output channel count
    *((volatile uint32_t *)0x5410450c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x54104544) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5410454c) = 0x41022000; // Post processing register
    *((volatile uint32_t *)0x54104548) = 0x0fff0fff; // Mask and processor enables

    // Layer 70 quadrant 0
    *((volatile uint32_t *)0x51104604) = 0x000c0003; // Rows
    *((volatile uint32_t *)0x51104608) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51104618) = 0x000000c0; // Stride
    *((volatile uint32_t *)0x5110461c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x51104620) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51104624) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51104628) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51104630) = 0x0000e920; // Layer control
    *((volatile uint32_t *)0x51104634) = 0x001d803b; // Layer control 2
    *((volatile uint32_t *)0x51104638) = 0x000263f8; // Mask count
    *((volatile uint32_t *)0x5110463c) = 0x00020a00; // Mask offset
    *((volatile uint32_t *)0x51104640) = 0x00000b3f; // Output channel count
    *((volatile uint32_t *)0x5110460c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5110464c) = 0x00006000; // Post processing register
    *((volatile uint32_t *)0x51104648) = 0xffffffff; // Mask and processor enables

    // Layer 70 quadrant 1
    *((volatile uint32_t *)0x52104604) = 0x000c0003; // Rows
    *((volatile uint32_t *)0x52104608) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52104618) = 0x000000c0; // Stride
    *((volatile uint32_t *)0x5210461c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x52104620) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52104624) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52104628) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52104630) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52104634) = 0x001d803b; // Layer control 2
    *((volatile uint32_t *)0x52104638) = 0x000263f8; // Mask count
    *((volatile uint32_t *)0x5210463c) = 0x00020a00; // Mask offset
    *((volatile uint32_t *)0x52104640) = 0x00000b3f; // Output channel count
    *((volatile uint32_t *)0x5210460c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210464c) = 0x00006000; // Post processing register
    *((volatile uint32_t *)0x52104648) = 0xffffffff; // Mask and processor enables

    // Layer 70 quadrant 2
    *((volatile uint32_t *)0x53104604) = 0x000c0003; // Rows
    *((volatile uint32_t *)0x53104608) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53104618) = 0x000000c0; // Stride
    *((volatile uint32_t *)0x5310461c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x53104620) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53104624) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53104628) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53104630) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53104634) = 0x001d803b; // Layer control 2
    *((volatile uint32_t *)0x53104638) = 0x000263f8; // Mask count
    *((volatile uint32_t *)0x5310463c) = 0x00020a00; // Mask offset
    *((volatile uint32_t *)0x53104640) = 0x00000b3f; // Output channel count
    *((volatile uint32_t *)0x5310460c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310464c) = 0x000073f0; // Post processing register
    *((volatile uint32_t *)0x53104648) = 0xffffffff; // Mask and processor enables

    // Layer 70 quadrant 3
    *((volatile uint32_t *)0x54104604) = 0x000c0003; // Rows
    *((volatile uint32_t *)0x54104608) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54104618) = 0x000000c0; // Stride
    *((volatile uint32_t *)0x5410461c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x54104620) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54104624) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54104628) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54104630) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54104634) = 0x001d803b; // Layer control 2
    *((volatile uint32_t *)0x54104638) = 0x000263f8; // Mask count
    *((volatile uint32_t *)0x5410463c) = 0x00020a00; // Mask offset
    *((volatile uint32_t *)0x54104640) = 0x00000b3f; // Output channel count
    *((volatile uint32_t *)0x5410460c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410464c) = 0x00006000; // Post processing register
    *((volatile uint32_t *)0x54104648) = 0x0fff0fff; // Mask and processor enables

    // Layer 71 quadrant 0
    *((volatile uint32_t *)0x51104704) = 0x00040003; // Rows
    *((volatile uint32_t *)0x51104708) = 0x00010003; // Columns
    *((volatile uint32_t *)0x51104718) = 0x00000040; // Stride
    *((volatile uint32_t *)0x5110471c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x51104720) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51104724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51104728) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110472c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51104730) = 0x0000eb20; // Layer control
    *((volatile uint32_t *)0x51104734) = 0x001f80e3; // Layer control 2
    *((volatile uint32_t *)0x51104738) = 0x0002dbf8; // Mask count
    *((volatile uint32_t *)0x5110473c) = 0x00026400; // Mask offset
    *((volatile uint32_t *)0x51104740) = 0x00000eff; // Output channel count
    *((volatile uint32_t *)0x5110470c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5110474c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x51104748) = 0xffffffff; // Mask and processor enables

    // Layer 71 quadrant 1
    *((volatile uint32_t *)0x52104704) = 0x00040003; // Rows
    *((volatile uint32_t *)0x52104708) = 0x00010003; // Columns
    *((volatile uint32_t *)0x52104718) = 0x00000040; // Stride
    *((volatile uint32_t *)0x5210471c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x52104720) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52104724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52104728) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210472c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52104730) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x52104734) = 0x001f80e3; // Layer control 2
    *((volatile uint32_t *)0x52104738) = 0x0002dbf8; // Mask count
    *((volatile uint32_t *)0x5210473c) = 0x00026400; // Mask offset
    *((volatile uint32_t *)0x52104740) = 0x00000eff; // Output channel count
    *((volatile uint32_t *)0x5210470c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210474c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x52104748) = 0xffffffff; // Mask and processor enables

    // Layer 71 quadrant 2
    *((volatile uint32_t *)0x53104704) = 0x00040003; // Rows
    *((volatile uint32_t *)0x53104708) = 0x00010003; // Columns
    *((volatile uint32_t *)0x53104718) = 0x00000040; // Stride
    *((volatile uint32_t *)0x5310471c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x53104720) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53104724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53104728) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310472c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53104730) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x53104734) = 0x001f80e3; // Layer control 2
    *((volatile uint32_t *)0x53104738) = 0x0002dbf8; // Mask count
    *((volatile uint32_t *)0x5310473c) = 0x00026400; // Mask offset
    *((volatile uint32_t *)0x53104740) = 0x00000eff; // Output channel count
    *((volatile uint32_t *)0x5310470c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310474c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x53104748) = 0xffffffff; // Mask and processor enables

    // Layer 71 quadrant 3
    *((volatile uint32_t *)0x54104704) = 0x00040003; // Rows
    *((volatile uint32_t *)0x54104708) = 0x00010003; // Columns
    *((volatile uint32_t *)0x54104718) = 0x00000040; // Stride
    *((volatile uint32_t *)0x5410471c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x54104720) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54104724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54104728) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410472c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54104730) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x54104734) = 0x001f80e3; // Layer control 2
    *((volatile uint32_t *)0x54104738) = 0x0002dbf8; // Mask count
    *((volatile uint32_t *)0x5410473c) = 0x00026400; // Mask offset
    *((volatile uint32_t *)0x54104740) = 0x00000eff; // Output channel count
    *((volatile uint32_t *)0x5410470c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410474c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x54104748) = 0x0fff0fff; // Mask and processor enables

    // Layer 72 quadrant 0
    *((volatile uint32_t *)0x51104804) = 0x00f00000; // Rows
    *((volatile uint32_t *)0x51104808) = 0x00040000; // Columns
    *((volatile uint32_t *)0x51104810) = 0x00000003; // Pooling rows
    *((volatile uint32_t *)0x51104814) = 0x00000003; // Pooling columns
    *((volatile uint32_t *)0x51104818) = 0x000003c3; // Stride
    *((volatile uint32_t *)0x51104820) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51104824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51104828) = 0x00000004; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110482c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x51104830) = 0x0001e8a0; // Layer control
    *((volatile uint32_t *)0x51104834) = 0x0019801e; // Layer control 2
    *((volatile uint32_t *)0x51104838) = 0x00030d18; // Mask count
    *((volatile uint32_t *)0x5110483c) = 0x0002dc60; // Mask offset
    *((volatile uint32_t *)0x51104840) = 0x00000617; // Output channel count
    *((volatile uint32_t *)0x5110480c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51104848) = 0xffffffff; // Mask and processor enables

    // Layer 72 quadrant 1
    *((volatile uint32_t *)0x52104804) = 0x00f00000; // Rows
    *((volatile uint32_t *)0x52104808) = 0x00040000; // Columns
    *((volatile uint32_t *)0x52104810) = 0x00000003; // Pooling rows
    *((volatile uint32_t *)0x52104814) = 0x00000003; // Pooling columns
    *((volatile uint32_t *)0x52104818) = 0x000003c3; // Stride
    *((volatile uint32_t *)0x52104820) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52104824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52104828) = 0x00000004; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210482c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x52104830) = 0x000108a0; // Layer control
    *((volatile uint32_t *)0x52104834) = 0x0019801e; // Layer control 2
    *((volatile uint32_t *)0x52104838) = 0x00030d18; // Mask count
    *((volatile uint32_t *)0x5210483c) = 0x0002dc60; // Mask offset
    *((volatile uint32_t *)0x52104840) = 0x00000617; // Output channel count
    *((volatile uint32_t *)0x5210480c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x52104848) = 0xffffffff; // Mask and processor enables

    // Layer 72 quadrant 2
    *((volatile uint32_t *)0x53104804) = 0x00f00000; // Rows
    *((volatile uint32_t *)0x53104808) = 0x00040000; // Columns
    *((volatile uint32_t *)0x53104810) = 0x00000003; // Pooling rows
    *((volatile uint32_t *)0x53104814) = 0x00000003; // Pooling columns
    *((volatile uint32_t *)0x53104818) = 0x000003c3; // Stride
    *((volatile uint32_t *)0x53104820) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53104824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53104828) = 0x00000004; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310482c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x53104830) = 0x000108a0; // Layer control
    *((volatile uint32_t *)0x53104834) = 0x0019801e; // Layer control 2
    *((volatile uint32_t *)0x53104838) = 0x00030d18; // Mask count
    *((volatile uint32_t *)0x5310483c) = 0x0002dc60; // Mask offset
    *((volatile uint32_t *)0x53104840) = 0x00000617; // Output channel count
    *((volatile uint32_t *)0x5310480c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x53104848) = 0xffffffff; // Mask and processor enables

    // Layer 72 quadrant 3
    *((volatile uint32_t *)0x54104804) = 0x00f00000; // Rows
    *((volatile uint32_t *)0x54104808) = 0x00040000; // Columns
    *((volatile uint32_t *)0x54104810) = 0x00000003; // Pooling rows
    *((volatile uint32_t *)0x54104814) = 0x00000003; // Pooling columns
    *((volatile uint32_t *)0x54104818) = 0x000003c3; // Stride
    *((volatile uint32_t *)0x54104820) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54104824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54104828) = 0x00000004; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410482c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x54104830) = 0x000108a0; // Layer control
    *((volatile uint32_t *)0x54104834) = 0x0019801e; // Layer control 2
    *((volatile uint32_t *)0x54104838) = 0x00030d18; // Mask count
    *((volatile uint32_t *)0x5410483c) = 0x0002dc60; // Mask offset
    *((volatile uint32_t *)0x54104840) = 0x00000617; // Output channel count
    *((volatile uint32_t *)0x5410480c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x54104848) = 0xffffffff; // Mask and processor enables

    return CNN_OK;
}

int cnn_start(void)
{
    cnn_time = 0;

    *((volatile uint32_t *)0x51000000) = 0x00100808; // Enable quadrant 0
    *((volatile uint32_t *)0x52000000) = 0x00100809; // Enable quadrant 1
    *((volatile uint32_t *)0x53000000) = 0x00100809; // Enable quadrant 2
    *((volatile uint32_t *)0x54000000) = 0x00100809; // Enable quadrant 3

#ifdef CNN_INFERENCE_TIMER
    MXC_TMR_SW_Start(CNN_INFERENCE_TIMER);
#endif

    CNN_START; // Allow capture of processing time
    *((volatile uint32_t *)0x51000000) = 0x00100009; // Master enable quadrant 0

    return CNN_OK;
}

int cnn_unload(uint32_t *out_buf)
{
    volatile uint32_t *addr;

    // Custom unload for this network, layer 72: 32-bit data, shape: (100, 1, 1)
    addr = (volatile uint32_t *)0x51800000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51820000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51840000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51860000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52800000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52820000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52840000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52860000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53800000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53820000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53840000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53860000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54800000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51800010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51820010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51840010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51860010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52800010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52820010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52840010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52860010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53800010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53820010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53840010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53860010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;

    return CNN_OK;
}

int cnn_enable(uint32_t clock_source, uint32_t clock_divider)
{
    // Reset all domains, restore power to CNN
    MXC_GCFR->reg3 = 0xf; // Reset
    MXC_GCFR->reg1 = 0xf; // Mask memory
    MXC_GCFR->reg0 = 0xf; // Power
    MXC_Delay(MSEC(10)); // Wait for load switches
    MXC_GCFR->reg2 = 0x0; // Iso
    MXC_GCFR->reg3 = 0x0; // Reset

    if (clock_source == MXC_S_GCR_PCLKDIV_CNNCLKSEL_IPLL)
        while ((MXC_GCR->ipll_ctrl & MXC_F_GCR_IPLL_CTRL_RDY) != MXC_F_GCR_IPLL_CTRL_RDY) {}
    // Wait for PLL

    MXC_GCR->pclkdiv =
        (MXC_GCR->pclkdiv & ~(MXC_F_GCR_PCLKDIV_CNNCLKDIV | MXC_F_GCR_PCLKDIV_CNNCLKSEL)) |
        clock_divider | clock_source;
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_CNN); // Enable CNN clock

    MXC_NVIC_SetVector(CNN_IRQn, CNN_ISR); // Set CNN complete vector

    return CNN_OK;
}

int cnn_boost_enable(mxc_gpio_regs_t *port, uint32_t pin)
{
    mxc_gpio_cfg_t gpio_out;
    gpio_out.port = port;
    gpio_out.mask = pin;
    gpio_out.pad = MXC_GPIO_PAD_NONE;
    gpio_out.func = MXC_GPIO_FUNC_OUT;
    MXC_GPIO_Config(&gpio_out);
    MXC_GPIO_OutSet(gpio_out.port, gpio_out.mask);

    return CNN_OK;
}

int cnn_boost_disable(mxc_gpio_regs_t *port, uint32_t pin)
{
    mxc_gpio_cfg_t gpio_out;
    gpio_out.port = port;
    gpio_out.mask = pin;
    gpio_out.pad = MXC_GPIO_PAD_NONE;
    gpio_out.func = MXC_GPIO_FUNC_OUT;
    MXC_GPIO_Config(&gpio_out);
    MXC_GPIO_OutClr(gpio_out.port, gpio_out.mask);

    return CNN_OK;
}

int cnn_disable(void)
{
    // Disable CNN clock
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_CNN);

    // Disable power to CNN
    MXC_GCFR->reg3 = 0xf; // Reset
    MXC_GCFR->reg2 = 0xf; // Iso
    MXC_GCFR->reg0 = 0x0; // Power
    MXC_GCFR->reg1 = 0x0; // Mask memory
    MXC_GCFR->reg3 = 0x0; // Reset

    return CNN_OK;
}
