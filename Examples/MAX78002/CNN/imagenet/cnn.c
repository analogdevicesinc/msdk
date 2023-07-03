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

// imagenet
// Created using ai8xize.py --test-dir sdk/Examples/MAX78002/CNN --prefix imagenet --checkpoint-file trained/ai87-imagenet-effnet2-q.pth.tar --config-file networks/ai87-imagenet-effnet2.yaml --device MAX78002 --timer 0 --display-checkpoint --verbose --overwrite

// DO NOT EDIT - regenerate this file instead!

// Configuring 34 layers
// Input data: HWC
// Layer 0: 3x112x112, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 32x56x56 output
// Layer 1: 32x56x56, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 16x56x56 output
// Layer 2: 16x56x56, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 64x56x56 output
// Layer 3: 64x56x56, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 32x56x56 output
// Layer 4: 32x56x56, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 128x56x56 output
// Layer 5: 128x56x56, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 32x56x56 output
// Layer 6: 2x32x56x56, no pooling, 2-element add, no convolution, 32x56x56 output
// Layer 7: 32x56x56, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 128x56x56 output
// Layer 8: 128x56x56, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 48x56x56 output
// Layer 9: 48x56x56, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 192x56x56 output
// Layer 10: 192x56x56, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 48x56x56 output
// Layer 11: 2x48x56x56, no pooling, 2-element add, no convolution, 48x56x56 output
// Layer 12: 48x56x56, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 192x56x56 output
// Layer 13: 192x56x56, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 48x56x56 output
// Layer 14: 2x48x56x56, max pool 2x2 with stride 2/2, 2-element add, no convolution, 48x28x28 output
// Layer 15: 48x28x28, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, ReLU, 96x28x28 output
// Layer 16: 96x28x28, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 96x28x28 output
// Layer 17: 96x28x28, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 96x28x28 output
// Layer 18: 96x28x28, no pooling, no convolution, 96x28x28 output
// Layer 19: 96x28x28, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, ReLU, 192x28x28 output
// Layer 20: 192x28x28, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 192x28x28 output
// Layer 21: 192x28x28, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 96x28x28 output
// Layer 22: 2x96x28x28, no pooling, 2-element add, no convolution, 96x28x28 output
// Layer 23: 96x28x28, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, ReLU, 192x28x28 output
// Layer 24: 192x28x28, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 192x28x28 output
// Layer 25: 192x28x28, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 128x28x28 output
// Layer 26: 128x28x28, max pool 2x2 with stride 2/2, conv2d with kernel size 1x1, stride 1/1, pad 0/0, ReLU, 128x14x14 output
// Layer 27: 128x14x14, no pooling, no convolution, 128x14x14 output
// Layer 28: 128x14x14, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, ReLU, 512x14x14 output
// Layer 29: 512x14x14, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 512x14x14 output
// Layer 30: 512x14x14, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 128x14x14 output
// Layer 31: 2x128x14x14, no pooling, 2-element add, no convolution, 128x14x14 output
// Layer 32: 128x14x14, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, ReLU, 1024x14x14 output
// Layer 33: 1024x14x14, avg pool 14x14 with stride 14/14, linear, no activation, 1000x1x1 output

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
    memcpy_8to32((uint32_t *)0x51180000, bias_0, sizeof(uint8_t) * 1384);
    memcpy_8to32((uint32_t *)0x52180000, bias_1, sizeof(uint8_t) * 1392);
    memcpy_8to32((uint32_t *)0x53180000, bias_2, sizeof(uint8_t) * 1392);
    memcpy_8to32((uint32_t *)0x54180000, bias_3, sizeof(uint8_t) * 1376);

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
    *((volatile uint32_t *)0x51000008) = 0x00000021; // Layer count
    *((volatile uint32_t *)0x52000000) = 0x00100008; // Stop SM
    *((volatile uint32_t *)0x52000008) = 0x00000021; // Layer count
    *((volatile uint32_t *)0x53000000) = 0x00100008; // Stop SM
    *((volatile uint32_t *)0x53000008) = 0x00000021; // Layer count
    *((volatile uint32_t *)0x54000000) = 0x00100008; // Stop SM
    *((volatile uint32_t *)0x54000008) = 0x00000021; // Layer count

    return CNN_OK;
}

int cnn_configure(void)
{
    // Layer 0 quadrant 0
    *((volatile uint32_t *)0x51100004) = 0x0072806e; // Rows
    *((volatile uint32_t *)0x51100008) = 0x0002806e; // Columns
    *((volatile uint32_t *)0x51100010) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x51100014) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x51100018) = 0x00000021; // Stride
    *((volatile uint32_t *)0x5110001c) = 0x00039000; // SRAM write ptr
    *((volatile uint32_t *)0x51100024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51100030) = 0x00888ba0; // Layer control
    *((volatile uint32_t *)0x51100034) = 0x000f8000; // Layer control 2
    *((volatile uint32_t *)0x51100038) = 0x000000f8; // Mask count
    *((volatile uint32_t *)0x51100040) = 0x0000001f; // Output channel count
    *((volatile uint32_t *)0x51100044) = 0x00000037; // TRAM ptr max
    *((volatile uint32_t *)0x5110004c) = 0x00022000; // Post processing register

    // Layer 0 quadrant 1
    *((volatile uint32_t *)0x52100004) = 0x0072806e; // Rows
    *((volatile uint32_t *)0x52100008) = 0x0002806e; // Columns
    *((volatile uint32_t *)0x52100010) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x52100014) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x52100018) = 0x00000021; // Stride
    *((volatile uint32_t *)0x5210001c) = 0x00039000; // SRAM write ptr
    *((volatile uint32_t *)0x52100024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52100030) = 0x00880ba0; // Layer control
    *((volatile uint32_t *)0x52100034) = 0x000f8000; // Layer control 2
    *((volatile uint32_t *)0x52100038) = 0x000000f8; // Mask count
    *((volatile uint32_t *)0x52100040) = 0x0000001f; // Output channel count
    *((volatile uint32_t *)0x52100044) = 0x00000037; // TRAM ptr max
    *((volatile uint32_t *)0x5210004c) = 0x00022000; // Post processing register

    // Layer 0 quadrant 2
    *((volatile uint32_t *)0x53100004) = 0x0072806e; // Rows
    *((volatile uint32_t *)0x53100008) = 0x0002806e; // Columns
    *((volatile uint32_t *)0x53100010) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x53100014) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x53100018) = 0x00000021; // Stride
    *((volatile uint32_t *)0x5310001c) = 0x00039000; // SRAM write ptr
    *((volatile uint32_t *)0x53100024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53100030) = 0x00880ba0; // Layer control
    *((volatile uint32_t *)0x53100034) = 0x000f8000; // Layer control 2
    *((volatile uint32_t *)0x53100038) = 0x000000f8; // Mask count
    *((volatile uint32_t *)0x53100040) = 0x0000001f; // Output channel count
    *((volatile uint32_t *)0x53100044) = 0x00000037; // TRAM ptr max
    *((volatile uint32_t *)0x5310004c) = 0x00022000; // Post processing register

    // Layer 0 quadrant 3
    *((volatile uint32_t *)0x54100004) = 0x0072806e; // Rows
    *((volatile uint32_t *)0x54100008) = 0x0002806e; // Columns
    *((volatile uint32_t *)0x54100010) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x54100014) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x54100018) = 0x00000021; // Stride
    *((volatile uint32_t *)0x5410001c) = 0x00039000; // SRAM write ptr
    *((volatile uint32_t *)0x54100024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54100030) = 0x00880ba0; // Layer control
    *((volatile uint32_t *)0x54100034) = 0x000f8000; // Layer control 2
    *((volatile uint32_t *)0x54100038) = 0x000000f8; // Mask count
    *((volatile uint32_t *)0x54100040) = 0x0000001f; // Output channel count
    *((volatile uint32_t *)0x54100044) = 0x00000037; // TRAM ptr max
    *((volatile uint32_t *)0x5410004c) = 0x00023540; // Post processing register
    *((volatile uint32_t *)0x54100048) = 0x70007000; // Mask and processor enables

    // Layer 1 quadrant 0
    *((volatile uint32_t *)0x51100104) = 0x00010037; // Rows
    *((volatile uint32_t *)0x51100108) = 0x00010037; // Columns
    *((volatile uint32_t *)0x51100118) = 0x00000010; // Stride
    *((volatile uint32_t *)0x51100120) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51100124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5110012c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51100130) = 0x0000e920; // Layer control
    *((volatile uint32_t *)0x51100134) = 0x00078000; // Layer control 2
    *((volatile uint32_t *)0x51100138) = 0x00000078; // Mask count
    *((volatile uint32_t *)0x51100140) = 0x0000000f; // Output channel count
    *((volatile uint32_t *)0x5110010c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5110014c) = 0x00003558; // Post processing register

    // Layer 1 quadrant 1
    *((volatile uint32_t *)0x52100104) = 0x00010037; // Rows
    *((volatile uint32_t *)0x52100108) = 0x00010037; // Columns
    *((volatile uint32_t *)0x52100118) = 0x00000010; // Stride
    *((volatile uint32_t *)0x52100120) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52100124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5210012c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52100130) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52100134) = 0x00078000; // Layer control 2
    *((volatile uint32_t *)0x52100138) = 0x00000078; // Mask count
    *((volatile uint32_t *)0x52100140) = 0x0000000f; // Output channel count
    *((volatile uint32_t *)0x5210010c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210014c) = 0x00002000; // Post processing register
    *((volatile uint32_t *)0x52100148) = 0xf000f000; // Mask and processor enables

    // Layer 1 quadrant 2
    *((volatile uint32_t *)0x53100104) = 0x00010037; // Rows
    *((volatile uint32_t *)0x53100108) = 0x00010037; // Columns
    *((volatile uint32_t *)0x53100118) = 0x00000010; // Stride
    *((volatile uint32_t *)0x53100120) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53100124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5310012c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53100130) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53100134) = 0x00078000; // Layer control 2
    *((volatile uint32_t *)0x53100138) = 0x00000078; // Mask count
    *((volatile uint32_t *)0x53100140) = 0x0000000f; // Output channel count
    *((volatile uint32_t *)0x5310010c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310014c) = 0x00002000; // Post processing register
    *((volatile uint32_t *)0x53100148) = 0xffffffff; // Mask and processor enables

    // Layer 1 quadrant 3
    *((volatile uint32_t *)0x54100104) = 0x00010037; // Rows
    *((volatile uint32_t *)0x54100108) = 0x00010037; // Columns
    *((volatile uint32_t *)0x54100118) = 0x00000010; // Stride
    *((volatile uint32_t *)0x54100120) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54100124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5410012c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54100130) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54100134) = 0x00078000; // Layer control 2
    *((volatile uint32_t *)0x54100138) = 0x00000078; // Mask count
    *((volatile uint32_t *)0x54100140) = 0x0000000f; // Output channel count
    *((volatile uint32_t *)0x5410010c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410014c) = 0x00002000; // Post processing register
    *((volatile uint32_t *)0x54100148) = 0x0fff0fff; // Mask and processor enables

    // Layer 2 quadrant 0
    *((volatile uint32_t *)0x51100204) = 0x00018037; // Rows
    *((volatile uint32_t *)0x51100208) = 0x00018037; // Columns
    *((volatile uint32_t *)0x51100218) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5110021c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x51100224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51100230) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x51100234) = 0x001f8000; // Layer control 2
    *((volatile uint32_t *)0x51100238) = 0x000001f8; // Mask count
    *((volatile uint32_t *)0x51100240) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x51100244) = 0x00000037; // TRAM ptr max
    *((volatile uint32_t *)0x5110024c) = 0x000234e8; // Post processing register
    *((volatile uint32_t *)0x51100248) = 0xffffffff; // Mask and processor enables

    // Layer 2 quadrant 1
    *((volatile uint32_t *)0x52100204) = 0x00018037; // Rows
    *((volatile uint32_t *)0x52100208) = 0x00018037; // Columns
    *((volatile uint32_t *)0x52100218) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5210021c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x52100224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52100230) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x52100234) = 0x001f8000; // Layer control 2
    *((volatile uint32_t *)0x52100238) = 0x000001f8; // Mask count
    *((volatile uint32_t *)0x52100240) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x52100244) = 0x00000037; // TRAM ptr max
    *((volatile uint32_t *)0x5210024c) = 0x00022000; // Post processing register

    // Layer 2 quadrant 2
    *((volatile uint32_t *)0x53100204) = 0x00018037; // Rows
    *((volatile uint32_t *)0x53100208) = 0x00018037; // Columns
    *((volatile uint32_t *)0x53100218) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5310021c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x53100224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53100230) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x53100234) = 0x001f8000; // Layer control 2
    *((volatile uint32_t *)0x53100238) = 0x000001f8; // Mask count
    *((volatile uint32_t *)0x53100240) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x53100244) = 0x00000037; // TRAM ptr max
    *((volatile uint32_t *)0x5310024c) = 0x00022000; // Post processing register

    // Layer 2 quadrant 3
    *((volatile uint32_t *)0x54100204) = 0x00018037; // Rows
    *((volatile uint32_t *)0x54100208) = 0x00018037; // Columns
    *((volatile uint32_t *)0x54100218) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5410021c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x54100224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54100230) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x54100234) = 0x001f8000; // Layer control 2
    *((volatile uint32_t *)0x54100238) = 0x000001f8; // Mask count
    *((volatile uint32_t *)0x54100240) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x54100244) = 0x00000037; // TRAM ptr max
    *((volatile uint32_t *)0x5410024c) = 0x00022000; // Post processing register

    // Layer 3 quadrant 0
    *((volatile uint32_t *)0x51100304) = 0x00010037; // Rows
    *((volatile uint32_t *)0x51100308) = 0x00010037; // Columns
    *((volatile uint32_t *)0x51100318) = 0x00000010; // Stride
    *((volatile uint32_t *)0x51100320) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51100324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5110032c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x51100330) = 0x0000e920; // Layer control
    *((volatile uint32_t *)0x51100334) = 0x000f8010; // Layer control 2
    *((volatile uint32_t *)0x51100338) = 0x000012f8; // Mask count
    *((volatile uint32_t *)0x5110033c) = 0x00001200; // Mask offset
    *((volatile uint32_t *)0x51100340) = 0x0000001f; // Output channel count
    *((volatile uint32_t *)0x5110030c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51100348) = 0xffffffff; // Mask and processor enables

    // Layer 3 quadrant 1
    *((volatile uint32_t *)0x52100304) = 0x00010037; // Rows
    *((volatile uint32_t *)0x52100308) = 0x00010037; // Columns
    *((volatile uint32_t *)0x52100318) = 0x00000010; // Stride
    *((volatile uint32_t *)0x52100320) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52100324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5210032c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x52100330) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52100334) = 0x000f8010; // Layer control 2
    *((volatile uint32_t *)0x52100338) = 0x000012f8; // Mask count
    *((volatile uint32_t *)0x5210033c) = 0x00001200; // Mask offset
    *((volatile uint32_t *)0x52100340) = 0x0000001f; // Output channel count
    *((volatile uint32_t *)0x5210030c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210034c) = 0x00001550; // Post processing register
    *((volatile uint32_t *)0x52100348) = 0xffffffff; // Mask and processor enables

    // Layer 3 quadrant 2
    *((volatile uint32_t *)0x53100304) = 0x00010037; // Rows
    *((volatile uint32_t *)0x53100308) = 0x00010037; // Columns
    *((volatile uint32_t *)0x53100318) = 0x00000010; // Stride
    *((volatile uint32_t *)0x53100320) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53100324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5310032c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x53100330) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53100334) = 0x000f8010; // Layer control 2
    *((volatile uint32_t *)0x53100338) = 0x000012f8; // Mask count
    *((volatile uint32_t *)0x5310033c) = 0x00001200; // Mask offset
    *((volatile uint32_t *)0x53100340) = 0x0000001f; // Output channel count
    *((volatile uint32_t *)0x5310030c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x53100348) = 0xffffffff; // Mask and processor enables

    // Layer 3 quadrant 3
    *((volatile uint32_t *)0x54100304) = 0x00010037; // Rows
    *((volatile uint32_t *)0x54100308) = 0x00010037; // Columns
    *((volatile uint32_t *)0x54100318) = 0x00000010; // Stride
    *((volatile uint32_t *)0x54100320) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54100324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5410032c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x54100330) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54100334) = 0x000f8010; // Layer control 2
    *((volatile uint32_t *)0x54100338) = 0x000012f8; // Mask count
    *((volatile uint32_t *)0x5410033c) = 0x00001200; // Mask offset
    *((volatile uint32_t *)0x54100340) = 0x0000001f; // Output channel count
    *((volatile uint32_t *)0x5410030c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x54100348) = 0xffffffff; // Mask and processor enables

    // Layer 4 quadrant 0
    *((volatile uint32_t *)0x51100404) = 0x00028037; // Rows
    *((volatile uint32_t *)0x51100408) = 0x00018037; // Columns
    *((volatile uint32_t *)0x51100418) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5110041c) = 0x00003000; // SRAM write ptr
    *((volatile uint32_t *)0x51100424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51100428) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51100430) = 0x00886b20; // Layer control
    *((volatile uint32_t *)0x51100434) = 0x001f8010; // Layer control 2
    *((volatile uint32_t *)0x51100438) = 0x00000618; // Mask count
    *((volatile uint32_t *)0x5110043c) = 0x00000220; // Mask offset
    *((volatile uint32_t *)0x51100440) = 0x0000007f; // Output channel count
    *((volatile uint32_t *)0x51100444) = 0x00000037; // TRAM ptr max
    *((volatile uint32_t *)0x5110044c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x51100448) = 0xffffffff; // Mask and processor enables

    // Layer 4 quadrant 1
    *((volatile uint32_t *)0x52100404) = 0x00028037; // Rows
    *((volatile uint32_t *)0x52100408) = 0x00018037; // Columns
    *((volatile uint32_t *)0x52100418) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5210041c) = 0x00003000; // SRAM write ptr
    *((volatile uint32_t *)0x52100424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52100428) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52100430) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x52100434) = 0x001f8010; // Layer control 2
    *((volatile uint32_t *)0x52100438) = 0x00000618; // Mask count
    *((volatile uint32_t *)0x5210043c) = 0x00000220; // Mask offset
    *((volatile uint32_t *)0x52100440) = 0x0000007f; // Output channel count
    *((volatile uint32_t *)0x52100444) = 0x00000037; // TRAM ptr max
    *((volatile uint32_t *)0x5210044c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x52100448) = 0xffffffff; // Mask and processor enables

    // Layer 4 quadrant 2
    *((volatile uint32_t *)0x53100404) = 0x00028037; // Rows
    *((volatile uint32_t *)0x53100408) = 0x00018037; // Columns
    *((volatile uint32_t *)0x53100418) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5310041c) = 0x00003000; // SRAM write ptr
    *((volatile uint32_t *)0x53100424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53100428) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53100430) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x53100434) = 0x001f8010; // Layer control 2
    *((volatile uint32_t *)0x53100438) = 0x00000618; // Mask count
    *((volatile uint32_t *)0x5310043c) = 0x00000220; // Mask offset
    *((volatile uint32_t *)0x53100440) = 0x0000007f; // Output channel count
    *((volatile uint32_t *)0x53100444) = 0x00000037; // TRAM ptr max
    *((volatile uint32_t *)0x5310044c) = 0x00023340; // Post processing register

    // Layer 4 quadrant 3
    *((volatile uint32_t *)0x54100404) = 0x00028037; // Rows
    *((volatile uint32_t *)0x54100408) = 0x00018037; // Columns
    *((volatile uint32_t *)0x54100418) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5410041c) = 0x00003000; // SRAM write ptr
    *((volatile uint32_t *)0x54100424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54100428) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54100430) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x54100434) = 0x001f8010; // Layer control 2
    *((volatile uint32_t *)0x54100438) = 0x00000618; // Mask count
    *((volatile uint32_t *)0x5410043c) = 0x00000220; // Mask offset
    *((volatile uint32_t *)0x54100440) = 0x0000007f; // Output channel count
    *((volatile uint32_t *)0x54100444) = 0x00000037; // TRAM ptr max
    *((volatile uint32_t *)0x5410044c) = 0x00022000; // Post processing register

    // Layer 5 quadrant 0
    *((volatile uint32_t *)0x51100504) = 0x00020037; // Rows
    *((volatile uint32_t *)0x51100508) = 0x00010037; // Columns
    *((volatile uint32_t *)0x51100518) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5110051c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x51100520) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51100524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5110052c) = 0x00003000; // SRAM read ptr
    *((volatile uint32_t *)0x51100530) = 0x0000e920; // Layer control
    *((volatile uint32_t *)0x51100534) = 0x000f8011; // Layer control 2
    *((volatile uint32_t *)0x51100538) = 0x00003918; // Mask count
    *((volatile uint32_t *)0x5110053c) = 0x00003720; // Mask offset
    *((volatile uint32_t *)0x51100540) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x5110050c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5110054c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x51100548) = 0xffffffff; // Mask and processor enables

    // Layer 5 quadrant 1
    *((volatile uint32_t *)0x52100504) = 0x00020037; // Rows
    *((volatile uint32_t *)0x52100508) = 0x00010037; // Columns
    *((volatile uint32_t *)0x52100518) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5210051c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x52100520) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52100524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5210052c) = 0x00003000; // SRAM read ptr
    *((volatile uint32_t *)0x52100530) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52100534) = 0x000f8011; // Layer control 2
    *((volatile uint32_t *)0x52100538) = 0x00003918; // Mask count
    *((volatile uint32_t *)0x5210053c) = 0x00003720; // Mask offset
    *((volatile uint32_t *)0x52100540) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x5210050c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210054c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x52100548) = 0xffffffff; // Mask and processor enables

    // Layer 5 quadrant 2
    *((volatile uint32_t *)0x53100504) = 0x00020037; // Rows
    *((volatile uint32_t *)0x53100508) = 0x00010037; // Columns
    *((volatile uint32_t *)0x53100518) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5310051c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x53100520) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53100524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5310052c) = 0x00003000; // SRAM read ptr
    *((volatile uint32_t *)0x53100530) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53100534) = 0x000f8011; // Layer control 2
    *((volatile uint32_t *)0x53100538) = 0x00003918; // Mask count
    *((volatile uint32_t *)0x5310053c) = 0x00003720; // Mask offset
    *((volatile uint32_t *)0x53100540) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x5310050c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310054c) = 0x00023550; // Post processing register
    *((volatile uint32_t *)0x53100548) = 0xffffffff; // Mask and processor enables

    // Layer 5 quadrant 3
    *((volatile uint32_t *)0x54100504) = 0x00020037; // Rows
    *((volatile uint32_t *)0x54100508) = 0x00010037; // Columns
    *((volatile uint32_t *)0x54100518) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5410051c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x54100520) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54100524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5410052c) = 0x00003000; // SRAM read ptr
    *((volatile uint32_t *)0x54100530) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54100534) = 0x000f8011; // Layer control 2
    *((volatile uint32_t *)0x54100538) = 0x00003918; // Mask count
    *((volatile uint32_t *)0x5410053c) = 0x00003720; // Mask offset
    *((volatile uint32_t *)0x54100540) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x5410050c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410054c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x54100548) = 0xffffffff; // Mask and processor enables

    // Layer 6 quadrant 0
    *((volatile uint32_t *)0x51100604) = 0x00020037; // Rows
    *((volatile uint32_t *)0x51100608) = 0x00010037; // Columns
    *((volatile uint32_t *)0x51100618) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5110061c) = 0x00001880; // SRAM write ptr
    *((volatile uint32_t *)0x51100620) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x51100630) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51100640) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5110060c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5110064c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x51100648) = 0x0000ffff; // Mask and processor enables

    // Layer 6 quadrant 1
    *((volatile uint32_t *)0x52100604) = 0x00020037; // Rows
    *((volatile uint32_t *)0x52100608) = 0x00010037; // Columns
    *((volatile uint32_t *)0x52100618) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5210061c) = 0x00021880; // SRAM write ptr
    *((volatile uint32_t *)0x52100620) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x52100630) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52100640) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5210060c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5210064c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x52100648) = 0x0000ffff; // Mask and processor enables

    // Layer 6 quadrant 2
    *((volatile uint32_t *)0x53100604) = 0x00020037; // Rows
    *((volatile uint32_t *)0x53100608) = 0x00010037; // Columns
    *((volatile uint32_t *)0x53100618) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5310061c) = 0x00001880; // SRAM write ptr
    *((volatile uint32_t *)0x53100620) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x53100630) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53100640) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5310060c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5310064c) = 0x01000000; // Post processing register

    // Layer 6 quadrant 3
    *((volatile uint32_t *)0x54100604) = 0x00020037; // Rows
    *((volatile uint32_t *)0x54100608) = 0x00010037; // Columns
    *((volatile uint32_t *)0x54100618) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5410061c) = 0x00001880; // SRAM write ptr
    *((volatile uint32_t *)0x54100620) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x54100630) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54100640) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5410060c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5410064c) = 0x01000000; // Post processing register

    // Layer 7 quadrant 0
    *((volatile uint32_t *)0x51100704) = 0x00018037; // Rows
    *((volatile uint32_t *)0x51100708) = 0x00018037; // Columns
    *((volatile uint32_t *)0x51100718) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5110071c) = 0x00003100; // SRAM write ptr
    *((volatile uint32_t *)0x51100724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51100728) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110072c) = 0x00001880; // SRAM read ptr
    *((volatile uint32_t *)0x51100730) = 0x00882b20; // Layer control
    *((volatile uint32_t *)0x51100734) = 0x001f8010; // Layer control 2
    *((volatile uint32_t *)0x51100738) = 0x00000a58; // Mask count
    *((volatile uint32_t *)0x5110073c) = 0x00000660; // Mask offset
    *((volatile uint32_t *)0x51100740) = 0x0000007f; // Output channel count
    *((volatile uint32_t *)0x51100744) = 0x00000037; // TRAM ptr max
    *((volatile uint32_t *)0x5110074c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x51100748) = 0xffffffff; // Mask and processor enables

    // Layer 7 quadrant 1
    *((volatile uint32_t *)0x52100704) = 0x00018037; // Rows
    *((volatile uint32_t *)0x52100708) = 0x00018037; // Columns
    *((volatile uint32_t *)0x52100718) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5210071c) = 0x00003100; // SRAM write ptr
    *((volatile uint32_t *)0x52100724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52100728) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210072c) = 0x00001880; // SRAM read ptr
    *((volatile uint32_t *)0x52100730) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x52100734) = 0x001f8010; // Layer control 2
    *((volatile uint32_t *)0x52100738) = 0x00000a58; // Mask count
    *((volatile uint32_t *)0x5210073c) = 0x00000660; // Mask offset
    *((volatile uint32_t *)0x52100740) = 0x0000007f; // Output channel count
    *((volatile uint32_t *)0x52100744) = 0x00000037; // TRAM ptr max
    *((volatile uint32_t *)0x5210074c) = 0x000233c0; // Post processing register
    *((volatile uint32_t *)0x52100748) = 0xffffffff; // Mask and processor enables

    // Layer 7 quadrant 2
    *((volatile uint32_t *)0x53100704) = 0x00018037; // Rows
    *((volatile uint32_t *)0x53100708) = 0x00018037; // Columns
    *((volatile uint32_t *)0x53100718) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5310071c) = 0x00003100; // SRAM write ptr
    *((volatile uint32_t *)0x53100724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53100728) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310072c) = 0x00001880; // SRAM read ptr
    *((volatile uint32_t *)0x53100730) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x53100734) = 0x001f8010; // Layer control 2
    *((volatile uint32_t *)0x53100738) = 0x00000a58; // Mask count
    *((volatile uint32_t *)0x5310073c) = 0x00000660; // Mask offset
    *((volatile uint32_t *)0x53100740) = 0x0000007f; // Output channel count
    *((volatile uint32_t *)0x53100744) = 0x00000037; // TRAM ptr max
    *((volatile uint32_t *)0x5310074c) = 0x00022000; // Post processing register

    // Layer 7 quadrant 3
    *((volatile uint32_t *)0x54100704) = 0x00018037; // Rows
    *((volatile uint32_t *)0x54100708) = 0x00018037; // Columns
    *((volatile uint32_t *)0x54100718) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5410071c) = 0x00003100; // SRAM write ptr
    *((volatile uint32_t *)0x54100724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54100728) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410072c) = 0x00001880; // SRAM read ptr
    *((volatile uint32_t *)0x54100730) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x54100734) = 0x001f8010; // Layer control 2
    *((volatile uint32_t *)0x54100738) = 0x00000a58; // Mask count
    *((volatile uint32_t *)0x5410073c) = 0x00000660; // Mask offset
    *((volatile uint32_t *)0x54100740) = 0x0000007f; // Output channel count
    *((volatile uint32_t *)0x54100744) = 0x00000037; // TRAM ptr max
    *((volatile uint32_t *)0x5410074c) = 0x00022000; // Post processing register

    // Layer 8 quadrant 0
    *((volatile uint32_t *)0x51100804) = 0x00020037; // Rows
    *((volatile uint32_t *)0x51100808) = 0x00010037; // Columns
    *((volatile uint32_t *)0x51100818) = 0x00000020; // Stride
    *((volatile uint32_t *)0x51100820) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51100824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5110082c) = 0x00003100; // SRAM read ptr
    *((volatile uint32_t *)0x51100830) = 0x0000e920; // Layer control
    *((volatile uint32_t *)0x51100834) = 0x00178011; // Layer control 2
    *((volatile uint32_t *)0x51100838) = 0x00006058; // Mask count
    *((volatile uint32_t *)0x5110083c) = 0x00005d60; // Mask offset
    *((volatile uint32_t *)0x51100840) = 0x0000005f; // Output channel count
    *((volatile uint32_t *)0x5110080c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5110084c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x51100848) = 0xffffffff; // Mask and processor enables

    // Layer 8 quadrant 1
    *((volatile uint32_t *)0x52100804) = 0x00020037; // Rows
    *((volatile uint32_t *)0x52100808) = 0x00010037; // Columns
    *((volatile uint32_t *)0x52100818) = 0x00000020; // Stride
    *((volatile uint32_t *)0x52100820) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52100824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5210082c) = 0x00003100; // SRAM read ptr
    *((volatile uint32_t *)0x52100830) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52100834) = 0x00178011; // Layer control 2
    *((volatile uint32_t *)0x52100838) = 0x00006058; // Mask count
    *((volatile uint32_t *)0x5210083c) = 0x00005d60; // Mask offset
    *((volatile uint32_t *)0x52100840) = 0x0000005f; // Output channel count
    *((volatile uint32_t *)0x5210080c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210084c) = 0x00023520; // Post processing register
    *((volatile uint32_t *)0x52100848) = 0xffffffff; // Mask and processor enables

    // Layer 8 quadrant 2
    *((volatile uint32_t *)0x53100804) = 0x00020037; // Rows
    *((volatile uint32_t *)0x53100808) = 0x00010037; // Columns
    *((volatile uint32_t *)0x53100818) = 0x00000020; // Stride
    *((volatile uint32_t *)0x53100820) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53100824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5310082c) = 0x00003100; // SRAM read ptr
    *((volatile uint32_t *)0x53100830) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53100834) = 0x00178011; // Layer control 2
    *((volatile uint32_t *)0x53100838) = 0x00006058; // Mask count
    *((volatile uint32_t *)0x5310083c) = 0x00005d60; // Mask offset
    *((volatile uint32_t *)0x53100840) = 0x0000005f; // Output channel count
    *((volatile uint32_t *)0x5310080c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310084c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x53100848) = 0xffffffff; // Mask and processor enables

    // Layer 8 quadrant 3
    *((volatile uint32_t *)0x54100804) = 0x00020037; // Rows
    *((volatile uint32_t *)0x54100808) = 0x00010037; // Columns
    *((volatile uint32_t *)0x54100818) = 0x00000020; // Stride
    *((volatile uint32_t *)0x54100820) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54100824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5410082c) = 0x00003100; // SRAM read ptr
    *((volatile uint32_t *)0x54100830) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54100834) = 0x00178011; // Layer control 2
    *((volatile uint32_t *)0x54100838) = 0x00006058; // Mask count
    *((volatile uint32_t *)0x5410083c) = 0x00005d60; // Mask offset
    *((volatile uint32_t *)0x54100840) = 0x0000005f; // Output channel count
    *((volatile uint32_t *)0x5410080c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410084c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x54100848) = 0xffffffff; // Mask and processor enables

    // Layer 9 quadrant 0
    *((volatile uint32_t *)0x51100904) = 0x00028037; // Rows
    *((volatile uint32_t *)0x51100908) = 0x00018037; // Columns
    *((volatile uint32_t *)0x51100918) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5110091c) = 0x00001880; // SRAM write ptr
    *((volatile uint32_t *)0x51100924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51100928) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51100930) = 0x00886b20; // Layer control
    *((volatile uint32_t *)0x51100934) = 0x001f8020; // Layer control 2
    *((volatile uint32_t *)0x51100938) = 0x000010b8; // Mask count
    *((volatile uint32_t *)0x5110093c) = 0x00000ac0; // Mask offset
    *((volatile uint32_t *)0x51100940) = 0x000000bf; // Output channel count
    *((volatile uint32_t *)0x51100944) = 0x00000037; // TRAM ptr max
    *((volatile uint32_t *)0x5110094c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x51100948) = 0xffffffff; // Mask and processor enables

    // Layer 9 quadrant 1
    *((volatile uint32_t *)0x52100904) = 0x00028037; // Rows
    *((volatile uint32_t *)0x52100908) = 0x00018037; // Columns
    *((volatile uint32_t *)0x52100918) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5210091c) = 0x00001880; // SRAM write ptr
    *((volatile uint32_t *)0x52100924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52100928) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52100930) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x52100934) = 0x001f8020; // Layer control 2
    *((volatile uint32_t *)0x52100938) = 0x000010b8; // Mask count
    *((volatile uint32_t *)0x5210093c) = 0x00000ac0; // Mask offset
    *((volatile uint32_t *)0x52100940) = 0x000000bf; // Output channel count
    *((volatile uint32_t *)0x52100944) = 0x00000037; // TRAM ptr max
    *((volatile uint32_t *)0x5210094c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x52100948) = 0xffffffff; // Mask and processor enables

    // Layer 9 quadrant 2
    *((volatile uint32_t *)0x53100904) = 0x00028037; // Rows
    *((volatile uint32_t *)0x53100908) = 0x00018037; // Columns
    *((volatile uint32_t *)0x53100918) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5310091c) = 0x00001880; // SRAM write ptr
    *((volatile uint32_t *)0x53100924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53100928) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53100930) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x53100934) = 0x001f8020; // Layer control 2
    *((volatile uint32_t *)0x53100938) = 0x000010b8; // Mask count
    *((volatile uint32_t *)0x5310093c) = 0x00000ac0; // Mask offset
    *((volatile uint32_t *)0x53100940) = 0x000000bf; // Output channel count
    *((volatile uint32_t *)0x53100944) = 0x00000037; // TRAM ptr max
    *((volatile uint32_t *)0x5310094c) = 0x00023100; // Post processing register
    *((volatile uint32_t *)0x53100948) = 0xffffffff; // Mask and processor enables

    // Layer 9 quadrant 3
    *((volatile uint32_t *)0x54100904) = 0x00028037; // Rows
    *((volatile uint32_t *)0x54100908) = 0x00018037; // Columns
    *((volatile uint32_t *)0x54100918) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5410091c) = 0x00001880; // SRAM write ptr
    *((volatile uint32_t *)0x54100924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54100928) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54100930) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x54100934) = 0x001f8020; // Layer control 2
    *((volatile uint32_t *)0x54100938) = 0x000010b8; // Mask count
    *((volatile uint32_t *)0x5410093c) = 0x00000ac0; // Mask offset
    *((volatile uint32_t *)0x54100940) = 0x000000bf; // Output channel count
    *((volatile uint32_t *)0x54100944) = 0x00000037; // TRAM ptr max
    *((volatile uint32_t *)0x5410094c) = 0x00022000; // Post processing register

    // Layer 10 quadrant 0
    *((volatile uint32_t *)0x51100a04) = 0x00030037; // Rows
    *((volatile uint32_t *)0x51100a08) = 0x00010037; // Columns
    *((volatile uint32_t *)0x51100a18) = 0x00000030; // Stride
    *((volatile uint32_t *)0x51100a1c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x51100a20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51100a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51100a2c) = 0x00001880; // SRAM read ptr
    *((volatile uint32_t *)0x51100a30) = 0x0000e920; // Layer control
    *((volatile uint32_t *)0x51100a34) = 0x00178012; // Layer control 2
    *((volatile uint32_t *)0x51100a38) = 0x00009b38; // Mask count
    *((volatile uint32_t *)0x51100a3c) = 0x000096c0; // Mask offset
    *((volatile uint32_t *)0x51100a40) = 0x0000008f; // Output channel count
    *((volatile uint32_t *)0x51100a0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51100a4c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x51100a48) = 0xffffffff; // Mask and processor enables

    // Layer 10 quadrant 1
    *((volatile uint32_t *)0x52100a04) = 0x00030037; // Rows
    *((volatile uint32_t *)0x52100a08) = 0x00010037; // Columns
    *((volatile uint32_t *)0x52100a18) = 0x00000030; // Stride
    *((volatile uint32_t *)0x52100a1c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x52100a20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52100a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52100a2c) = 0x00001880; // SRAM read ptr
    *((volatile uint32_t *)0x52100a30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52100a34) = 0x00178012; // Layer control 2
    *((volatile uint32_t *)0x52100a38) = 0x00009b38; // Mask count
    *((volatile uint32_t *)0x52100a3c) = 0x000096c0; // Mask offset
    *((volatile uint32_t *)0x52100a40) = 0x0000008f; // Output channel count
    *((volatile uint32_t *)0x52100a0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x52100a4c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x52100a48) = 0xffffffff; // Mask and processor enables

    // Layer 10 quadrant 2
    *((volatile uint32_t *)0x53100a04) = 0x00030037; // Rows
    *((volatile uint32_t *)0x53100a08) = 0x00010037; // Columns
    *((volatile uint32_t *)0x53100a18) = 0x00000030; // Stride
    *((volatile uint32_t *)0x53100a1c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x53100a20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53100a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53100a2c) = 0x00001880; // SRAM read ptr
    *((volatile uint32_t *)0x53100a30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53100a34) = 0x00178012; // Layer control 2
    *((volatile uint32_t *)0x53100a38) = 0x00009b38; // Mask count
    *((volatile uint32_t *)0x53100a3c) = 0x000096c0; // Mask offset
    *((volatile uint32_t *)0x53100a40) = 0x0000008f; // Output channel count
    *((volatile uint32_t *)0x53100a0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x53100a4c) = 0x00023520; // Post processing register
    *((volatile uint32_t *)0x53100a48) = 0xffffffff; // Mask and processor enables

    // Layer 10 quadrant 3
    *((volatile uint32_t *)0x54100a04) = 0x00030037; // Rows
    *((volatile uint32_t *)0x54100a08) = 0x00010037; // Columns
    *((volatile uint32_t *)0x54100a18) = 0x00000030; // Stride
    *((volatile uint32_t *)0x54100a1c) = 0x00000001; // SRAM write ptr
    *((volatile uint32_t *)0x54100a20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54100a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54100a2c) = 0x00001880; // SRAM read ptr
    *((volatile uint32_t *)0x54100a30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54100a34) = 0x00178012; // Layer control 2
    *((volatile uint32_t *)0x54100a38) = 0x00009b38; // Mask count
    *((volatile uint32_t *)0x54100a3c) = 0x000096c0; // Mask offset
    *((volatile uint32_t *)0x54100a40) = 0x0000008f; // Output channel count
    *((volatile uint32_t *)0x54100a0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x54100a4c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x54100a48) = 0xffffffff; // Mask and processor enables

    // Layer 11 quadrant 0
    *((volatile uint32_t *)0x51100b04) = 0x00020037; // Rows
    *((volatile uint32_t *)0x51100b08) = 0x00010037; // Columns
    *((volatile uint32_t *)0x51100b18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x51100b1c) = 0x00003400; // SRAM write ptr
    *((volatile uint32_t *)0x51100b20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x51100b30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51100b34) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x51100b40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x51100b0c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x51100b4c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x51100b48) = 0x0000ffff; // Mask and processor enables

    // Layer 11 quadrant 1
    *((volatile uint32_t *)0x52100b04) = 0x00020037; // Rows
    *((volatile uint32_t *)0x52100b08) = 0x00010037; // Columns
    *((volatile uint32_t *)0x52100b18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x52100b1c) = 0x00023400; // SRAM write ptr
    *((volatile uint32_t *)0x52100b20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x52100b30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52100b34) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x52100b40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x52100b0c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x52100b4c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x52100b48) = 0x0000ffff; // Mask and processor enables

    // Layer 11 quadrant 2
    *((volatile uint32_t *)0x53100b04) = 0x00020037; // Rows
    *((volatile uint32_t *)0x53100b08) = 0x00010037; // Columns
    *((volatile uint32_t *)0x53100b18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x53100b1c) = 0x00043400; // SRAM write ptr
    *((volatile uint32_t *)0x53100b20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x53100b30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53100b34) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x53100b40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x53100b0c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x53100b4c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x53100b48) = 0x0000ffff; // Mask and processor enables

    // Layer 11 quadrant 3
    *((volatile uint32_t *)0x54100b04) = 0x00020037; // Rows
    *((volatile uint32_t *)0x54100b08) = 0x00010037; // Columns
    *((volatile uint32_t *)0x54100b18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x54100b1c) = 0x00003400; // SRAM write ptr
    *((volatile uint32_t *)0x54100b20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x54100b30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54100b34) = 0x00000010; // Layer control 2
    *((volatile uint32_t *)0x54100b40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x54100b0c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x54100b4c) = 0x01000000; // Post processing register

    // Layer 12 quadrant 0
    *((volatile uint32_t *)0x51100c04) = 0x00028037; // Rows
    *((volatile uint32_t *)0x51100c08) = 0x00018037; // Columns
    *((volatile uint32_t *)0x51100c18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x51100c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51100c28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51100c2c) = 0x00003400; // SRAM read ptr
    *((volatile uint32_t *)0x51100c30) = 0x00886b20; // Layer control
    *((volatile uint32_t *)0x51100c34) = 0x001f8020; // Layer control 2
    *((volatile uint32_t *)0x51100c38) = 0x00001738; // Mask count
    *((volatile uint32_t *)0x51100c3c) = 0x00001140; // Mask offset
    *((volatile uint32_t *)0x51100c40) = 0x000000bf; // Output channel count
    *((volatile uint32_t *)0x51100c44) = 0x00000037; // TRAM ptr max
    *((volatile uint32_t *)0x51100c4c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x51100c48) = 0xffffffff; // Mask and processor enables

    // Layer 12 quadrant 1
    *((volatile uint32_t *)0x52100c04) = 0x00028037; // Rows
    *((volatile uint32_t *)0x52100c08) = 0x00018037; // Columns
    *((volatile uint32_t *)0x52100c18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x52100c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52100c28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52100c2c) = 0x00003400; // SRAM read ptr
    *((volatile uint32_t *)0x52100c30) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x52100c34) = 0x001f8020; // Layer control 2
    *((volatile uint32_t *)0x52100c38) = 0x00001738; // Mask count
    *((volatile uint32_t *)0x52100c3c) = 0x00001140; // Mask offset
    *((volatile uint32_t *)0x52100c40) = 0x000000bf; // Output channel count
    *((volatile uint32_t *)0x52100c44) = 0x00000037; // TRAM ptr max
    *((volatile uint32_t *)0x52100c4c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x52100c48) = 0xffffffff; // Mask and processor enables

    // Layer 12 quadrant 2
    *((volatile uint32_t *)0x53100c04) = 0x00028037; // Rows
    *((volatile uint32_t *)0x53100c08) = 0x00018037; // Columns
    *((volatile uint32_t *)0x53100c18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x53100c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53100c28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53100c2c) = 0x00003400; // SRAM read ptr
    *((volatile uint32_t *)0x53100c30) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x53100c34) = 0x001f8020; // Layer control 2
    *((volatile uint32_t *)0x53100c38) = 0x00001738; // Mask count
    *((volatile uint32_t *)0x53100c3c) = 0x00001140; // Mask offset
    *((volatile uint32_t *)0x53100c40) = 0x000000bf; // Output channel count
    *((volatile uint32_t *)0x53100c44) = 0x00000037; // TRAM ptr max
    *((volatile uint32_t *)0x53100c4c) = 0x000231c0; // Post processing register
    *((volatile uint32_t *)0x53100c48) = 0xffffffff; // Mask and processor enables

    // Layer 12 quadrant 3
    *((volatile uint32_t *)0x54100c04) = 0x00028037; // Rows
    *((volatile uint32_t *)0x54100c08) = 0x00018037; // Columns
    *((volatile uint32_t *)0x54100c18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x54100c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54100c28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54100c2c) = 0x00003400; // SRAM read ptr
    *((volatile uint32_t *)0x54100c30) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x54100c34) = 0x001f8020; // Layer control 2
    *((volatile uint32_t *)0x54100c38) = 0x00001738; // Mask count
    *((volatile uint32_t *)0x54100c3c) = 0x00001140; // Mask offset
    *((volatile uint32_t *)0x54100c40) = 0x000000bf; // Output channel count
    *((volatile uint32_t *)0x54100c44) = 0x00000037; // TRAM ptr max
    *((volatile uint32_t *)0x54100c4c) = 0x00022000; // Post processing register

    // Layer 13 quadrant 0
    *((volatile uint32_t *)0x51100d04) = 0x00030037; // Rows
    *((volatile uint32_t *)0x51100d08) = 0x00010037; // Columns
    *((volatile uint32_t *)0x51100d18) = 0x00000030; // Stride
    *((volatile uint32_t *)0x51100d1c) = 0x00003401; // SRAM write ptr
    *((volatile uint32_t *)0x51100d20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51100d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51100d30) = 0x0000e920; // Layer control
    *((volatile uint32_t *)0x51100d34) = 0x00178012; // Layer control 2
    *((volatile uint32_t *)0x51100d38) = 0x0000d5b8; // Mask count
    *((volatile uint32_t *)0x51100d3c) = 0x0000d140; // Mask offset
    *((volatile uint32_t *)0x51100d40) = 0x0000008f; // Output channel count
    *((volatile uint32_t *)0x51100d0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51100d4c) = 0x00023528; // Post processing register
    *((volatile uint32_t *)0x51100d48) = 0xffffffff; // Mask and processor enables

    // Layer 13 quadrant 1
    *((volatile uint32_t *)0x52100d04) = 0x00030037; // Rows
    *((volatile uint32_t *)0x52100d08) = 0x00010037; // Columns
    *((volatile uint32_t *)0x52100d18) = 0x00000030; // Stride
    *((volatile uint32_t *)0x52100d1c) = 0x00003401; // SRAM write ptr
    *((volatile uint32_t *)0x52100d20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52100d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52100d30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52100d34) = 0x00178012; // Layer control 2
    *((volatile uint32_t *)0x52100d38) = 0x0000d5b8; // Mask count
    *((volatile uint32_t *)0x52100d3c) = 0x0000d140; // Mask offset
    *((volatile uint32_t *)0x52100d40) = 0x0000008f; // Output channel count
    *((volatile uint32_t *)0x52100d0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x52100d4c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x52100d48) = 0xffffffff; // Mask and processor enables

    // Layer 13 quadrant 2
    *((volatile uint32_t *)0x53100d04) = 0x00030037; // Rows
    *((volatile uint32_t *)0x53100d08) = 0x00010037; // Columns
    *((volatile uint32_t *)0x53100d18) = 0x00000030; // Stride
    *((volatile uint32_t *)0x53100d1c) = 0x00003401; // SRAM write ptr
    *((volatile uint32_t *)0x53100d20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53100d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53100d30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53100d34) = 0x00178012; // Layer control 2
    *((volatile uint32_t *)0x53100d38) = 0x0000d5b8; // Mask count
    *((volatile uint32_t *)0x53100d3c) = 0x0000d140; // Mask offset
    *((volatile uint32_t *)0x53100d40) = 0x0000008f; // Output channel count
    *((volatile uint32_t *)0x53100d0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x53100d4c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x53100d48) = 0xffffffff; // Mask and processor enables

    // Layer 13 quadrant 3
    *((volatile uint32_t *)0x54100d04) = 0x00030037; // Rows
    *((volatile uint32_t *)0x54100d08) = 0x00010037; // Columns
    *((volatile uint32_t *)0x54100d18) = 0x00000030; // Stride
    *((volatile uint32_t *)0x54100d1c) = 0x00003401; // SRAM write ptr
    *((volatile uint32_t *)0x54100d20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54100d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54100d30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54100d34) = 0x00178012; // Layer control 2
    *((volatile uint32_t *)0x54100d38) = 0x0000d5b8; // Mask count
    *((volatile uint32_t *)0x54100d3c) = 0x0000d140; // Mask offset
    *((volatile uint32_t *)0x54100d40) = 0x0000008f; // Output channel count
    *((volatile uint32_t *)0x54100d0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x54100d4c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x54100d48) = 0xffffffff; // Mask and processor enables

    // Layer 14 quadrant 0
    *((volatile uint32_t *)0x51100e04) = 0x00740036; // Rows
    *((volatile uint32_t *)0x51100e08) = 0x00020036; // Columns
    *((volatile uint32_t *)0x51100e10) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x51100e14) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x51100e18) = 0x00000041; // Stride
    *((volatile uint32_t *)0x51100e20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x51100e2c) = 0x00003400; // SRAM read ptr
    *((volatile uint32_t *)0x51100e30) = 0x000009a0; // Layer control
    *((volatile uint32_t *)0x51100e40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x51100e0c) = 0x00056003; // 1D
    *((volatile uint32_t *)0x51100e4c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x51100e48) = 0x0000ffff; // Mask and processor enables

    // Layer 14 quadrant 1
    *((volatile uint32_t *)0x52100e04) = 0x00740036; // Rows
    *((volatile uint32_t *)0x52100e08) = 0x00020036; // Columns
    *((volatile uint32_t *)0x52100e10) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x52100e14) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x52100e18) = 0x00000041; // Stride
    *((volatile uint32_t *)0x52100e1c) = 0x00020000; // SRAM write ptr
    *((volatile uint32_t *)0x52100e20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x52100e2c) = 0x00003400; // SRAM read ptr
    *((volatile uint32_t *)0x52100e30) = 0x000009a0; // Layer control
    *((volatile uint32_t *)0x52100e40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x52100e0c) = 0x00056003; // 1D
    *((volatile uint32_t *)0x52100e4c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x52100e48) = 0x0000ffff; // Mask and processor enables

    // Layer 14 quadrant 2
    *((volatile uint32_t *)0x53100e04) = 0x00740036; // Rows
    *((volatile uint32_t *)0x53100e08) = 0x00020036; // Columns
    *((volatile uint32_t *)0x53100e10) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x53100e14) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x53100e18) = 0x00000041; // Stride
    *((volatile uint32_t *)0x53100e1c) = 0x00040000; // SRAM write ptr
    *((volatile uint32_t *)0x53100e20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x53100e2c) = 0x00003400; // SRAM read ptr
    *((volatile uint32_t *)0x53100e30) = 0x000009a0; // Layer control
    *((volatile uint32_t *)0x53100e40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x53100e0c) = 0x00056003; // 1D
    *((volatile uint32_t *)0x53100e4c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x53100e48) = 0x0000ffff; // Mask and processor enables

    // Layer 14 quadrant 3
    *((volatile uint32_t *)0x54100e04) = 0x00740036; // Rows
    *((volatile uint32_t *)0x54100e08) = 0x00020036; // Columns
    *((volatile uint32_t *)0x54100e10) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x54100e14) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x54100e18) = 0x00000041; // Stride
    *((volatile uint32_t *)0x54100e20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x54100e2c) = 0x00003400; // SRAM read ptr
    *((volatile uint32_t *)0x54100e30) = 0x000009a0; // Layer control
    *((volatile uint32_t *)0x54100e40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x54100e0c) = 0x00056003; // 1D
    *((volatile uint32_t *)0x54100e4c) = 0x01000000; // Post processing register

    // Layer 15 quadrant 0
    *((volatile uint32_t *)0x51100f04) = 0x0001001b; // Rows
    *((volatile uint32_t *)0x51100f08) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x51100f18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x51100f1c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x51100f20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51100f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51100f28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51100f30) = 0x00006b20; // Layer control
    *((volatile uint32_t *)0x51100f34) = 0x00178010; // Layer control 2
    *((volatile uint32_t *)0x51100f38) = 0x0000d8b8; // Mask count
    *((volatile uint32_t *)0x51100f3c) = 0x0000d5c0; // Mask offset
    *((volatile uint32_t *)0x51100f40) = 0x0000005f; // Output channel count
    *((volatile uint32_t *)0x51100f0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51100f4c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x51100f48) = 0xffffffff; // Mask and processor enables

    // Layer 15 quadrant 1
    *((volatile uint32_t *)0x52100f04) = 0x0001001b; // Rows
    *((volatile uint32_t *)0x52100f08) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x52100f18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x52100f1c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x52100f20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52100f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52100f28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52100f30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x52100f34) = 0x00178010; // Layer control 2
    *((volatile uint32_t *)0x52100f38) = 0x0000d8b8; // Mask count
    *((volatile uint32_t *)0x52100f3c) = 0x0000d5c0; // Mask offset
    *((volatile uint32_t *)0x52100f40) = 0x0000005f; // Output channel count
    *((volatile uint32_t *)0x52100f0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x52100f4c) = 0x000234c0; // Post processing register
    *((volatile uint32_t *)0x52100f48) = 0xffffffff; // Mask and processor enables

    // Layer 15 quadrant 2
    *((volatile uint32_t *)0x53100f04) = 0x0001001b; // Rows
    *((volatile uint32_t *)0x53100f08) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x53100f18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x53100f1c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x53100f20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53100f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53100f28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53100f30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x53100f34) = 0x00178010; // Layer control 2
    *((volatile uint32_t *)0x53100f38) = 0x0000d8b8; // Mask count
    *((volatile uint32_t *)0x53100f3c) = 0x0000d5c0; // Mask offset
    *((volatile uint32_t *)0x53100f40) = 0x0000005f; // Output channel count
    *((volatile uint32_t *)0x53100f0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x53100f4c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x53100f48) = 0xffffffff; // Mask and processor enables

    // Layer 15 quadrant 3
    *((volatile uint32_t *)0x54100f04) = 0x0001001b; // Rows
    *((volatile uint32_t *)0x54100f08) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x54100f18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x54100f1c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x54100f20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54100f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54100f28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54100f30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x54100f34) = 0x00178010; // Layer control 2
    *((volatile uint32_t *)0x54100f38) = 0x0000d8b8; // Mask count
    *((volatile uint32_t *)0x54100f3c) = 0x0000d5c0; // Mask offset
    *((volatile uint32_t *)0x54100f40) = 0x0000005f; // Output channel count
    *((volatile uint32_t *)0x54100f0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x54100f4c) = 0x00022000; // Post processing register

    // Layer 16 quadrant 0
    *((volatile uint32_t *)0x51101004) = 0x0002801b; // Rows
    *((volatile uint32_t *)0x51101008) = 0x0001801b; // Columns
    *((volatile uint32_t *)0x51101018) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5110101c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x51101024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51101028) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110102c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51101030) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x51101034) = 0x00178011; // Layer control 2
    *((volatile uint32_t *)0x51101038) = 0x00001828; // Mask count
    *((volatile uint32_t *)0x5110103c) = 0x00001820; // Mask offset
    *((volatile uint32_t *)0x51101040) = 0x00000007; // Output channel count
    *((volatile uint32_t *)0x5110100c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x51101044) = 0x0000001b; // TRAM ptr max
    *((volatile uint32_t *)0x5110104c) = 0x41003000; // Post processing register
    *((volatile uint32_t *)0x51101048) = 0xffffffff; // Mask and processor enables

    // Layer 16 quadrant 1
    *((volatile uint32_t *)0x52101004) = 0x0002801b; // Rows
    *((volatile uint32_t *)0x52101008) = 0x0001801b; // Columns
    *((volatile uint32_t *)0x52101018) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5210101c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x52101024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52101028) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210102c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52101030) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x52101034) = 0x00178011; // Layer control 2
    *((volatile uint32_t *)0x52101038) = 0x00001828; // Mask count
    *((volatile uint32_t *)0x5210103c) = 0x00001820; // Mask offset
    *((volatile uint32_t *)0x52101040) = 0x00000007; // Output channel count
    *((volatile uint32_t *)0x5210100c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x52101044) = 0x0000001b; // TRAM ptr max
    *((volatile uint32_t *)0x5210104c) = 0x41003000; // Post processing register
    *((volatile uint32_t *)0x52101048) = 0xffffffff; // Mask and processor enables

    // Layer 16 quadrant 2
    *((volatile uint32_t *)0x53101004) = 0x0002801b; // Rows
    *((volatile uint32_t *)0x53101008) = 0x0001801b; // Columns
    *((volatile uint32_t *)0x53101018) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5310101c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x53101024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53101028) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310102c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53101030) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x53101034) = 0x00178011; // Layer control 2
    *((volatile uint32_t *)0x53101038) = 0x00001828; // Mask count
    *((volatile uint32_t *)0x5310103c) = 0x00001820; // Mask offset
    *((volatile uint32_t *)0x53101040) = 0x00000007; // Output channel count
    *((volatile uint32_t *)0x5310100c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x53101044) = 0x0000001b; // TRAM ptr max
    *((volatile uint32_t *)0x5310104c) = 0x41003000; // Post processing register
    *((volatile uint32_t *)0x53101048) = 0xffffffff; // Mask and processor enables

    // Layer 16 quadrant 3
    *((volatile uint32_t *)0x54101004) = 0x0002801b; // Rows
    *((volatile uint32_t *)0x54101008) = 0x0001801b; // Columns
    *((volatile uint32_t *)0x54101018) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5410101c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x54101024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54101028) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410102c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54101030) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x54101034) = 0x00178011; // Layer control 2
    *((volatile uint32_t *)0x54101038) = 0x00001828; // Mask count
    *((volatile uint32_t *)0x5410103c) = 0x00001820; // Mask offset
    *((volatile uint32_t *)0x54101040) = 0x00000007; // Output channel count
    *((volatile uint32_t *)0x5410100c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x54101044) = 0x0000001b; // TRAM ptr max
    *((volatile uint32_t *)0x5410104c) = 0x41002000; // Post processing register

    // Layer 17 quadrant 0
    *((volatile uint32_t *)0x51101104) = 0x0002001b; // Rows
    *((volatile uint32_t *)0x51101108) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x51101118) = 0x00000020; // Stride
    *((volatile uint32_t *)0x51101120) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51101124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51101128) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110112c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x51101130) = 0x00006920; // Layer control
    *((volatile uint32_t *)0x51101134) = 0x00178011; // Layer control 2
    *((volatile uint32_t *)0x51101138) = 0x0000e038; // Mask count
    *((volatile uint32_t *)0x5110113c) = 0x0000da40; // Mask offset
    *((volatile uint32_t *)0x51101140) = 0x000000bf; // Output channel count
    *((volatile uint32_t *)0x5110110c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51101148) = 0xffffffff; // Mask and processor enables

    // Layer 17 quadrant 1
    *((volatile uint32_t *)0x52101104) = 0x0002001b; // Rows
    *((volatile uint32_t *)0x52101108) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x52101118) = 0x00000020; // Stride
    *((volatile uint32_t *)0x52101120) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52101124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52101128) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210112c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x52101130) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52101134) = 0x00178011; // Layer control 2
    *((volatile uint32_t *)0x52101138) = 0x0000e038; // Mask count
    *((volatile uint32_t *)0x5210113c) = 0x0000da40; // Mask offset
    *((volatile uint32_t *)0x52101140) = 0x000000bf; // Output channel count
    *((volatile uint32_t *)0x5210110c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x52101148) = 0xffffffff; // Mask and processor enables

    // Layer 17 quadrant 2
    *((volatile uint32_t *)0x53101104) = 0x0002001b; // Rows
    *((volatile uint32_t *)0x53101108) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x53101118) = 0x00000020; // Stride
    *((volatile uint32_t *)0x53101120) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53101124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53101128) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310112c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x53101130) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53101134) = 0x00178011; // Layer control 2
    *((volatile uint32_t *)0x53101138) = 0x0000e038; // Mask count
    *((volatile uint32_t *)0x5310113c) = 0x0000da40; // Mask offset
    *((volatile uint32_t *)0x53101140) = 0x000000bf; // Output channel count
    *((volatile uint32_t *)0x5310110c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310114c) = 0x000014c0; // Post processing register
    *((volatile uint32_t *)0x53101148) = 0xffffffff; // Mask and processor enables

    // Layer 17 quadrant 3
    *((volatile uint32_t *)0x54101104) = 0x0002001b; // Rows
    *((volatile uint32_t *)0x54101108) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x54101118) = 0x00000020; // Stride
    *((volatile uint32_t *)0x54101120) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54101124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54101128) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410112c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x54101130) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54101134) = 0x00178011; // Layer control 2
    *((volatile uint32_t *)0x54101138) = 0x0000e038; // Mask count
    *((volatile uint32_t *)0x5410113c) = 0x0000da40; // Mask offset
    *((volatile uint32_t *)0x54101140) = 0x000000bf; // Output channel count
    *((volatile uint32_t *)0x5410110c) = 0x00000100; // 1D

    // Layer 18 quadrant 0
    *((volatile uint32_t *)0x51101204) = 0x0002001b; // Rows
    *((volatile uint32_t *)0x51101208) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x51101218) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5110121c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x51101220) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x51101228) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51101230) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51101234) = 0x00000011; // Layer control 2
    *((volatile uint32_t *)0x5110123c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x51101240) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5110120c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5110124c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x51101248) = 0x0000ffff; // Mask and processor enables

    // Layer 18 quadrant 1
    *((volatile uint32_t *)0x52101204) = 0x0002001b; // Rows
    *((volatile uint32_t *)0x52101208) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x52101218) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5210121c) = 0x00022000; // SRAM write ptr
    *((volatile uint32_t *)0x52101220) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x52101228) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52101230) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52101234) = 0x00000011; // Layer control 2
    *((volatile uint32_t *)0x5210123c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x52101240) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5210120c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5210124c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x52101248) = 0x0000ffff; // Mask and processor enables

    // Layer 18 quadrant 2
    *((volatile uint32_t *)0x53101204) = 0x0002001b; // Rows
    *((volatile uint32_t *)0x53101208) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x53101218) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5310121c) = 0x00042000; // SRAM write ptr
    *((volatile uint32_t *)0x53101220) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x53101228) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53101230) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53101234) = 0x00000011; // Layer control 2
    *((volatile uint32_t *)0x5310123c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x53101240) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5310120c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5310124c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x53101248) = 0x0000ffff; // Mask and processor enables

    // Layer 18 quadrant 3
    *((volatile uint32_t *)0x54101204) = 0x0002001b; // Rows
    *((volatile uint32_t *)0x54101208) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x54101218) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5410121c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x54101220) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x54101228) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54101230) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54101234) = 0x00000011; // Layer control 2
    *((volatile uint32_t *)0x5410123c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x54101240) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5410120c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x5410124c) = 0x03000000; // Post processing register

    // Layer 19 quadrant 0
    *((volatile uint32_t *)0x51101304) = 0x0002001b; // Rows
    *((volatile uint32_t *)0x51101308) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x51101318) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5110131c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x51101320) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51101324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51101328) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51101330) = 0x00006b20; // Layer control
    *((volatile uint32_t *)0x51101334) = 0x001f8021; // Layer control 2
    *((volatile uint32_t *)0x51101338) = 0x0000ecf8; // Mask count
    *((volatile uint32_t *)0x5110133c) = 0x0000e100; // Mask offset
    *((volatile uint32_t *)0x51101340) = 0x0000017f; // Output channel count
    *((volatile uint32_t *)0x5110130c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5110134c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x51101348) = 0xffffffff; // Mask and processor enables

    // Layer 19 quadrant 1
    *((volatile uint32_t *)0x52101304) = 0x0002001b; // Rows
    *((volatile uint32_t *)0x52101308) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x52101318) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5210131c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x52101320) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52101324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52101328) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52101330) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x52101334) = 0x001f8021; // Layer control 2
    *((volatile uint32_t *)0x52101338) = 0x0000ecf8; // Mask count
    *((volatile uint32_t *)0x5210133c) = 0x0000e100; // Mask offset
    *((volatile uint32_t *)0x52101340) = 0x0000017f; // Output channel count
    *((volatile uint32_t *)0x5210130c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210134c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x52101348) = 0xffffffff; // Mask and processor enables

    // Layer 19 quadrant 2
    *((volatile uint32_t *)0x53101304) = 0x0002001b; // Rows
    *((volatile uint32_t *)0x53101308) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x53101318) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5310131c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x53101320) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53101324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53101328) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53101330) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x53101334) = 0x001f8021; // Layer control 2
    *((volatile uint32_t *)0x53101338) = 0x0000ecf8; // Mask count
    *((volatile uint32_t *)0x5310133c) = 0x0000e100; // Mask offset
    *((volatile uint32_t *)0x53101340) = 0x0000017f; // Output channel count
    *((volatile uint32_t *)0x5310130c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310134c) = 0x00023280; // Post processing register
    *((volatile uint32_t *)0x53101348) = 0xffffffff; // Mask and processor enables

    // Layer 19 quadrant 3
    *((volatile uint32_t *)0x54101304) = 0x0002001b; // Rows
    *((volatile uint32_t *)0x54101308) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x54101318) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5410131c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x54101320) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54101324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54101328) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54101330) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x54101334) = 0x001f8021; // Layer control 2
    *((volatile uint32_t *)0x54101338) = 0x0000ecf8; // Mask count
    *((volatile uint32_t *)0x5410133c) = 0x0000e100; // Mask offset
    *((volatile uint32_t *)0x54101340) = 0x0000017f; // Output channel count
    *((volatile uint32_t *)0x5410130c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410134c) = 0x00022000; // Post processing register

    // Layer 20 quadrant 0
    *((volatile uint32_t *)0x51101404) = 0x0003801b; // Rows
    *((volatile uint32_t *)0x51101408) = 0x0001801b; // Columns
    *((volatile uint32_t *)0x51101418) = 0x00000030; // Stride
    *((volatile uint32_t *)0x5110141c) = 0x00003000; // SRAM write ptr
    *((volatile uint32_t *)0x51101424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51101428) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110142c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51101430) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x51101434) = 0x001f8022; // Layer control 2
    *((volatile uint32_t *)0x51101438) = 0x00001a70; // Mask count
    *((volatile uint32_t *)0x5110143c) = 0x00001a60; // Mask offset
    *((volatile uint32_t *)0x51101440) = 0x0000000b; // Output channel count
    *((volatile uint32_t *)0x5110140c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x51101444) = 0x0000001b; // TRAM ptr max
    *((volatile uint32_t *)0x5110144c) = 0x41003008; // Post processing register
    *((volatile uint32_t *)0x51101448) = 0xffffffff; // Mask and processor enables

    // Layer 20 quadrant 1
    *((volatile uint32_t *)0x52101404) = 0x0003801b; // Rows
    *((volatile uint32_t *)0x52101408) = 0x0001801b; // Columns
    *((volatile uint32_t *)0x52101418) = 0x00000030; // Stride
    *((volatile uint32_t *)0x5210141c) = 0x00003000; // SRAM write ptr
    *((volatile uint32_t *)0x52101424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52101428) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210142c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52101430) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x52101434) = 0x001f8022; // Layer control 2
    *((volatile uint32_t *)0x52101438) = 0x00001a70; // Mask count
    *((volatile uint32_t *)0x5210143c) = 0x00001a60; // Mask offset
    *((volatile uint32_t *)0x52101440) = 0x0000000b; // Output channel count
    *((volatile uint32_t *)0x5210140c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x52101444) = 0x0000001b; // TRAM ptr max
    *((volatile uint32_t *)0x5210144c) = 0x41003008; // Post processing register
    *((volatile uint32_t *)0x52101448) = 0xffffffff; // Mask and processor enables

    // Layer 20 quadrant 2
    *((volatile uint32_t *)0x53101404) = 0x0003801b; // Rows
    *((volatile uint32_t *)0x53101408) = 0x0001801b; // Columns
    *((volatile uint32_t *)0x53101418) = 0x00000030; // Stride
    *((volatile uint32_t *)0x5310141c) = 0x00003000; // SRAM write ptr
    *((volatile uint32_t *)0x53101424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53101428) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310142c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53101430) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x53101434) = 0x001f8022; // Layer control 2
    *((volatile uint32_t *)0x53101438) = 0x00001a70; // Mask count
    *((volatile uint32_t *)0x5310143c) = 0x00001a60; // Mask offset
    *((volatile uint32_t *)0x53101440) = 0x0000000b; // Output channel count
    *((volatile uint32_t *)0x5310140c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x53101444) = 0x0000001b; // TRAM ptr max
    *((volatile uint32_t *)0x5310144c) = 0x41003008; // Post processing register
    *((volatile uint32_t *)0x53101448) = 0xffffffff; // Mask and processor enables

    // Layer 20 quadrant 3
    *((volatile uint32_t *)0x54101404) = 0x0003801b; // Rows
    *((volatile uint32_t *)0x54101408) = 0x0001801b; // Columns
    *((volatile uint32_t *)0x54101418) = 0x00000030; // Stride
    *((volatile uint32_t *)0x5410141c) = 0x00003000; // SRAM write ptr
    *((volatile uint32_t *)0x54101424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54101428) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410142c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54101430) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x54101434) = 0x001f8022; // Layer control 2
    *((volatile uint32_t *)0x54101438) = 0x00001a70; // Mask count
    *((volatile uint32_t *)0x5410143c) = 0x00001a60; // Mask offset
    *((volatile uint32_t *)0x54101440) = 0x0000000b; // Output channel count
    *((volatile uint32_t *)0x5410140c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x54101444) = 0x0000001b; // TRAM ptr max
    *((volatile uint32_t *)0x5410144c) = 0x41003000; // Post processing register
    *((volatile uint32_t *)0x54101448) = 0xffffffff; // Mask and processor enables

    // Layer 21 quadrant 0
    *((volatile uint32_t *)0x51101504) = 0x0003001b; // Rows
    *((volatile uint32_t *)0x51101508) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x51101518) = 0x00000030; // Stride
    *((volatile uint32_t *)0x5110151c) = 0x00002001; // SRAM write ptr
    *((volatile uint32_t *)0x51101520) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51101524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51101528) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110152c) = 0x00003000; // SRAM read ptr
    *((volatile uint32_t *)0x51101530) = 0x0000e920; // Layer control
    *((volatile uint32_t *)0x51101534) = 0x00178032; // Layer control 2
    *((volatile uint32_t *)0x51101538) = 0x0000f778; // Mask count
    *((volatile uint32_t *)0x5110153c) = 0x0000ee80; // Mask offset
    *((volatile uint32_t *)0x51101540) = 0x0000011f; // Output channel count
    *((volatile uint32_t *)0x5110150c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51101548) = 0xffffffff; // Mask and processor enables

    // Layer 21 quadrant 1
    *((volatile uint32_t *)0x52101504) = 0x0003001b; // Rows
    *((volatile uint32_t *)0x52101508) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x52101518) = 0x00000030; // Stride
    *((volatile uint32_t *)0x5210151c) = 0x00002001; // SRAM write ptr
    *((volatile uint32_t *)0x52101520) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52101524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52101528) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210152c) = 0x00003000; // SRAM read ptr
    *((volatile uint32_t *)0x52101530) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52101534) = 0x00178032; // Layer control 2
    *((volatile uint32_t *)0x52101538) = 0x0000f778; // Mask count
    *((volatile uint32_t *)0x5210153c) = 0x0000ee80; // Mask offset
    *((volatile uint32_t *)0x52101540) = 0x0000011f; // Output channel count
    *((volatile uint32_t *)0x5210150c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x52101548) = 0xffffffff; // Mask and processor enables

    // Layer 21 quadrant 2
    *((volatile uint32_t *)0x53101504) = 0x0003001b; // Rows
    *((volatile uint32_t *)0x53101508) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x53101518) = 0x00000030; // Stride
    *((volatile uint32_t *)0x5310151c) = 0x00002001; // SRAM write ptr
    *((volatile uint32_t *)0x53101520) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53101524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53101528) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310152c) = 0x00003000; // SRAM read ptr
    *((volatile uint32_t *)0x53101530) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53101534) = 0x00178032; // Layer control 2
    *((volatile uint32_t *)0x53101538) = 0x0000f778; // Mask count
    *((volatile uint32_t *)0x5310153c) = 0x0000ee80; // Mask offset
    *((volatile uint32_t *)0x53101540) = 0x0000011f; // Output channel count
    *((volatile uint32_t *)0x5310150c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x53101548) = 0xffffffff; // Mask and processor enables

    // Layer 21 quadrant 3
    *((volatile uint32_t *)0x54101504) = 0x0003001b; // Rows
    *((volatile uint32_t *)0x54101508) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x54101518) = 0x00000030; // Stride
    *((volatile uint32_t *)0x5410151c) = 0x00002001; // SRAM write ptr
    *((volatile uint32_t *)0x54101520) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54101524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54101528) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410152c) = 0x00003000; // SRAM read ptr
    *((volatile uint32_t *)0x54101530) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54101534) = 0x00178032; // Layer control 2
    *((volatile uint32_t *)0x54101538) = 0x0000f778; // Mask count
    *((volatile uint32_t *)0x5410153c) = 0x0000ee80; // Mask offset
    *((volatile uint32_t *)0x54101540) = 0x0000011f; // Output channel count
    *((volatile uint32_t *)0x5410150c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410154c) = 0x000014e0; // Post processing register
    *((volatile uint32_t *)0x54101548) = 0xffffffff; // Mask and processor enables

    // Layer 22 quadrant 0
    *((volatile uint32_t *)0x51101604) = 0x0004001b; // Rows
    *((volatile uint32_t *)0x51101608) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x51101618) = 0x00000040; // Stride
    *((volatile uint32_t *)0x51101620) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x51101628) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110162c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x51101630) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51101634) = 0x00000001; // Layer control 2
    *((volatile uint32_t *)0x5110163c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x51101640) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5110160c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5110164c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x51101648) = 0x0000ffff; // Mask and processor enables

    // Layer 22 quadrant 1
    *((volatile uint32_t *)0x52101604) = 0x0004001b; // Rows
    *((volatile uint32_t *)0x52101608) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x52101618) = 0x00000040; // Stride
    *((volatile uint32_t *)0x5210161c) = 0x00020000; // SRAM write ptr
    *((volatile uint32_t *)0x52101620) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x52101628) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210162c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x52101630) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52101634) = 0x00000001; // Layer control 2
    *((volatile uint32_t *)0x5210163c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x52101640) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5210160c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5210164c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x52101648) = 0x0000ffff; // Mask and processor enables

    // Layer 22 quadrant 2
    *((volatile uint32_t *)0x53101604) = 0x0004001b; // Rows
    *((volatile uint32_t *)0x53101608) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x53101618) = 0x00000040; // Stride
    *((volatile uint32_t *)0x5310161c) = 0x00040000; // SRAM write ptr
    *((volatile uint32_t *)0x53101620) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x53101628) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310162c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x53101630) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53101634) = 0x00000001; // Layer control 2
    *((volatile uint32_t *)0x5310163c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x53101640) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5310160c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5310164c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x53101648) = 0x0000ffff; // Mask and processor enables

    // Layer 22 quadrant 3
    *((volatile uint32_t *)0x54101604) = 0x0004001b; // Rows
    *((volatile uint32_t *)0x54101608) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x54101618) = 0x00000040; // Stride
    *((volatile uint32_t *)0x54101620) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x54101628) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410162c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x54101630) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54101634) = 0x00000001; // Layer control 2
    *((volatile uint32_t *)0x5410163c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x54101640) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x5410160c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x5410164c) = 0x01000000; // Post processing register

    // Layer 23 quadrant 0
    *((volatile uint32_t *)0x51101704) = 0x0002001b; // Rows
    *((volatile uint32_t *)0x51101708) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x51101718) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5110171c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x51101720) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51101724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51101728) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51101730) = 0x00006b20; // Layer control
    *((volatile uint32_t *)0x51101734) = 0x001f8021; // Layer control 2
    *((volatile uint32_t *)0x51101738) = 0x00010378; // Mask count
    *((volatile uint32_t *)0x5110173c) = 0x0000f780; // Mask offset
    *((volatile uint32_t *)0x51101740) = 0x0000017f; // Output channel count
    *((volatile uint32_t *)0x5110170c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5110174c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x51101748) = 0xffffffff; // Mask and processor enables

    // Layer 23 quadrant 1
    *((volatile uint32_t *)0x52101704) = 0x0002001b; // Rows
    *((volatile uint32_t *)0x52101708) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x52101718) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5210171c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x52101720) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52101724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52101728) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52101730) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x52101734) = 0x001f8021; // Layer control 2
    *((volatile uint32_t *)0x52101738) = 0x00010378; // Mask count
    *((volatile uint32_t *)0x5210173c) = 0x0000f780; // Mask offset
    *((volatile uint32_t *)0x52101740) = 0x0000017f; // Output channel count
    *((volatile uint32_t *)0x5210170c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210174c) = 0x00023300; // Post processing register
    *((volatile uint32_t *)0x52101748) = 0xffffffff; // Mask and processor enables

    // Layer 23 quadrant 2
    *((volatile uint32_t *)0x53101704) = 0x0002001b; // Rows
    *((volatile uint32_t *)0x53101708) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x53101718) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5310171c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x53101720) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53101724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53101728) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53101730) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x53101734) = 0x001f8021; // Layer control 2
    *((volatile uint32_t *)0x53101738) = 0x00010378; // Mask count
    *((volatile uint32_t *)0x5310173c) = 0x0000f780; // Mask offset
    *((volatile uint32_t *)0x53101740) = 0x0000017f; // Output channel count
    *((volatile uint32_t *)0x5310170c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310174c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x53101748) = 0xffffffff; // Mask and processor enables

    // Layer 23 quadrant 3
    *((volatile uint32_t *)0x54101704) = 0x0002001b; // Rows
    *((volatile uint32_t *)0x54101708) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x54101718) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5410171c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x54101720) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54101724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54101728) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54101730) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x54101734) = 0x001f8021; // Layer control 2
    *((volatile uint32_t *)0x54101738) = 0x00010378; // Mask count
    *((volatile uint32_t *)0x5410173c) = 0x0000f780; // Mask offset
    *((volatile uint32_t *)0x54101740) = 0x0000017f; // Output channel count
    *((volatile uint32_t *)0x5410170c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410174c) = 0x00022000; // Post processing register

    // Layer 24 quadrant 0
    *((volatile uint32_t *)0x51101804) = 0x0003801b; // Rows
    *((volatile uint32_t *)0x51101808) = 0x0001801b; // Columns
    *((volatile uint32_t *)0x51101818) = 0x00000030; // Stride
    *((volatile uint32_t *)0x5110181c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x51101824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51101828) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110182c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51101830) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x51101834) = 0x001f8022; // Layer control 2
    *((volatile uint32_t *)0x51101838) = 0x00001cf0; // Mask count
    *((volatile uint32_t *)0x5110183c) = 0x00001ce0; // Mask offset
    *((volatile uint32_t *)0x51101840) = 0x0000000b; // Output channel count
    *((volatile uint32_t *)0x5110180c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x51101844) = 0x0000001b; // TRAM ptr max
    *((volatile uint32_t *)0x5110184c) = 0x41003014; // Post processing register
    *((volatile uint32_t *)0x51101848) = 0xffffffff; // Mask and processor enables

    // Layer 24 quadrant 1
    *((volatile uint32_t *)0x52101804) = 0x0003801b; // Rows
    *((volatile uint32_t *)0x52101808) = 0x0001801b; // Columns
    *((volatile uint32_t *)0x52101818) = 0x00000030; // Stride
    *((volatile uint32_t *)0x5210181c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x52101824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52101828) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210182c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52101830) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x52101834) = 0x001f8022; // Layer control 2
    *((volatile uint32_t *)0x52101838) = 0x00001cf0; // Mask count
    *((volatile uint32_t *)0x5210183c) = 0x00001ce0; // Mask offset
    *((volatile uint32_t *)0x52101840) = 0x0000000b; // Output channel count
    *((volatile uint32_t *)0x5210180c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x52101844) = 0x0000001b; // TRAM ptr max
    *((volatile uint32_t *)0x5210184c) = 0x41003014; // Post processing register
    *((volatile uint32_t *)0x52101848) = 0xffffffff; // Mask and processor enables

    // Layer 24 quadrant 2
    *((volatile uint32_t *)0x53101804) = 0x0003801b; // Rows
    *((volatile uint32_t *)0x53101808) = 0x0001801b; // Columns
    *((volatile uint32_t *)0x53101818) = 0x00000030; // Stride
    *((volatile uint32_t *)0x5310181c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x53101824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53101828) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310182c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53101830) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x53101834) = 0x001f8022; // Layer control 2
    *((volatile uint32_t *)0x53101838) = 0x00001cf0; // Mask count
    *((volatile uint32_t *)0x5310183c) = 0x00001ce0; // Mask offset
    *((volatile uint32_t *)0x53101840) = 0x0000000b; // Output channel count
    *((volatile uint32_t *)0x5310180c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x53101844) = 0x0000001b; // TRAM ptr max
    *((volatile uint32_t *)0x5310184c) = 0x41003014; // Post processing register
    *((volatile uint32_t *)0x53101848) = 0xffffffff; // Mask and processor enables

    // Layer 24 quadrant 3
    *((volatile uint32_t *)0x54101804) = 0x0003801b; // Rows
    *((volatile uint32_t *)0x54101808) = 0x0001801b; // Columns
    *((volatile uint32_t *)0x54101818) = 0x00000030; // Stride
    *((volatile uint32_t *)0x5410181c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x54101824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54101828) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410182c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54101830) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x54101834) = 0x001f8022; // Layer control 2
    *((volatile uint32_t *)0x54101838) = 0x00001cf0; // Mask count
    *((volatile uint32_t *)0x5410183c) = 0x00001ce0; // Mask offset
    *((volatile uint32_t *)0x54101840) = 0x0000000b; // Output channel count
    *((volatile uint32_t *)0x5410180c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x54101844) = 0x0000001b; // TRAM ptr max
    *((volatile uint32_t *)0x5410184c) = 0x4100300c; // Post processing register
    *((volatile uint32_t *)0x54101848) = 0xffffffff; // Mask and processor enables

    // Layer 25 quadrant 0
    *((volatile uint32_t *)0x51101904) = 0x0003001b; // Rows
    *((volatile uint32_t *)0x51101908) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x51101918) = 0x00000030; // Stride
    *((volatile uint32_t *)0x51101920) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51101924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51101928) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110192c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x51101930) = 0x0000e920; // Layer control
    *((volatile uint32_t *)0x51101934) = 0x001f8012; // Layer control 2
    *((volatile uint32_t *)0x51101938) = 0x000110f8; // Mask count
    *((volatile uint32_t *)0x5110193c) = 0x00010500; // Mask offset
    *((volatile uint32_t *)0x51101940) = 0x0000017f; // Output channel count
    *((volatile uint32_t *)0x5110190c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51101948) = 0xffffffff; // Mask and processor enables

    // Layer 25 quadrant 1
    *((volatile uint32_t *)0x52101904) = 0x0003001b; // Rows
    *((volatile uint32_t *)0x52101908) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x52101918) = 0x00000030; // Stride
    *((volatile uint32_t *)0x52101920) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52101924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52101928) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210192c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x52101930) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52101934) = 0x001f8012; // Layer control 2
    *((volatile uint32_t *)0x52101938) = 0x000110f8; // Mask count
    *((volatile uint32_t *)0x5210193c) = 0x00010500; // Mask offset
    *((volatile uint32_t *)0x52101940) = 0x0000017f; // Output channel count
    *((volatile uint32_t *)0x5210190c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x52101948) = 0xffffffff; // Mask and processor enables

    // Layer 25 quadrant 2
    *((volatile uint32_t *)0x53101904) = 0x0003001b; // Rows
    *((volatile uint32_t *)0x53101908) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x53101918) = 0x00000030; // Stride
    *((volatile uint32_t *)0x53101920) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53101924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53101928) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310192c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x53101930) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53101934) = 0x001f8012; // Layer control 2
    *((volatile uint32_t *)0x53101938) = 0x000110f8; // Mask count
    *((volatile uint32_t *)0x5310193c) = 0x00010500; // Mask offset
    *((volatile uint32_t *)0x53101940) = 0x0000017f; // Output channel count
    *((volatile uint32_t *)0x5310190c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310194c) = 0x000013c0; // Post processing register
    *((volatile uint32_t *)0x53101948) = 0xffffffff; // Mask and processor enables

    // Layer 25 quadrant 3
    *((volatile uint32_t *)0x54101904) = 0x0003001b; // Rows
    *((volatile uint32_t *)0x54101908) = 0x0001001b; // Columns
    *((volatile uint32_t *)0x54101918) = 0x00000030; // Stride
    *((volatile uint32_t *)0x54101920) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54101924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54101928) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410192c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x54101930) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54101934) = 0x001f8012; // Layer control 2
    *((volatile uint32_t *)0x54101938) = 0x000110f8; // Mask count
    *((volatile uint32_t *)0x5410193c) = 0x00010500; // Mask offset
    *((volatile uint32_t *)0x54101940) = 0x0000017f; // Output channel count
    *((volatile uint32_t *)0x5410190c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x54101948) = 0xffffffff; // Mask and processor enables

    // Layer 26 quadrant 0
    *((volatile uint32_t *)0x51101a04) = 0x003c001a; // Rows
    *((volatile uint32_t *)0x51101a08) = 0x0002001a; // Columns
    *((volatile uint32_t *)0x51101a10) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x51101a14) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x51101a18) = 0x00000041; // Stride
    *((volatile uint32_t *)0x51101a1c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x51101a20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51101a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51101a28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51101a30) = 0x0000eba0; // Layer control
    *((volatile uint32_t *)0x51101a34) = 0x001f8011; // Layer control 2
    *((volatile uint32_t *)0x51101a38) = 0x00011958; // Mask count
    *((volatile uint32_t *)0x51101a3c) = 0x00011160; // Mask offset
    *((volatile uint32_t *)0x51101a40) = 0x000000ff; // Output channel count
    *((volatile uint32_t *)0x51101a0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51101a4c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x51101a48) = 0xffffffff; // Mask and processor enables

    // Layer 26 quadrant 1
    *((volatile uint32_t *)0x52101a04) = 0x003c001a; // Rows
    *((volatile uint32_t *)0x52101a08) = 0x0002001a; // Columns
    *((volatile uint32_t *)0x52101a10) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x52101a14) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x52101a18) = 0x00000041; // Stride
    *((volatile uint32_t *)0x52101a1c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x52101a20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52101a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52101a28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52101a30) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x52101a34) = 0x001f8011; // Layer control 2
    *((volatile uint32_t *)0x52101a38) = 0x00011958; // Mask count
    *((volatile uint32_t *)0x52101a3c) = 0x00011160; // Mask offset
    *((volatile uint32_t *)0x52101a40) = 0x000000ff; // Output channel count
    *((volatile uint32_t *)0x52101a0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x52101a4c) = 0x00023440; // Post processing register
    *((volatile uint32_t *)0x52101a48) = 0xffffffff; // Mask and processor enables

    // Layer 26 quadrant 2
    *((volatile uint32_t *)0x53101a04) = 0x003c001a; // Rows
    *((volatile uint32_t *)0x53101a08) = 0x0002001a; // Columns
    *((volatile uint32_t *)0x53101a10) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x53101a14) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x53101a18) = 0x00000041; // Stride
    *((volatile uint32_t *)0x53101a1c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x53101a20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53101a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53101a28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53101a30) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x53101a34) = 0x001f8011; // Layer control 2
    *((volatile uint32_t *)0x53101a38) = 0x00011958; // Mask count
    *((volatile uint32_t *)0x53101a3c) = 0x00011160; // Mask offset
    *((volatile uint32_t *)0x53101a40) = 0x000000ff; // Output channel count
    *((volatile uint32_t *)0x53101a0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x53101a4c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x53101a48) = 0xffffffff; // Mask and processor enables

    // Layer 26 quadrant 3
    *((volatile uint32_t *)0x54101a04) = 0x003c001a; // Rows
    *((volatile uint32_t *)0x54101a08) = 0x0002001a; // Columns
    *((volatile uint32_t *)0x54101a10) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x54101a14) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x54101a18) = 0x00000041; // Stride
    *((volatile uint32_t *)0x54101a1c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x54101a20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54101a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54101a28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54101a30) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x54101a34) = 0x001f8011; // Layer control 2
    *((volatile uint32_t *)0x54101a38) = 0x00011958; // Mask count
    *((volatile uint32_t *)0x54101a3c) = 0x00011160; // Mask offset
    *((volatile uint32_t *)0x54101a40) = 0x000000ff; // Output channel count
    *((volatile uint32_t *)0x54101a0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x54101a4c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x54101a48) = 0xffffffff; // Mask and processor enables

    // Layer 27 quadrant 0
    *((volatile uint32_t *)0x51101b04) = 0x0002000d; // Rows
    *((volatile uint32_t *)0x51101b08) = 0x0001000d; // Columns
    *((volatile uint32_t *)0x51101b18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x51101b1c) = 0x00003000; // SRAM write ptr
    *((volatile uint32_t *)0x51101b20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x51101b28) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51101b2c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x51101b30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51101b34) = 0x00000011; // Layer control 2
    *((volatile uint32_t *)0x51101b3c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x51101b40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x51101b0c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x51101b4c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x51101b48) = 0x0000ffff; // Mask and processor enables

    // Layer 27 quadrant 1
    *((volatile uint32_t *)0x52101b04) = 0x0002000d; // Rows
    *((volatile uint32_t *)0x52101b08) = 0x0001000d; // Columns
    *((volatile uint32_t *)0x52101b18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x52101b1c) = 0x00023000; // SRAM write ptr
    *((volatile uint32_t *)0x52101b20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x52101b28) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52101b2c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x52101b30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52101b34) = 0x00000011; // Layer control 2
    *((volatile uint32_t *)0x52101b3c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x52101b40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x52101b0c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x52101b4c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x52101b48) = 0x0000ffff; // Mask and processor enables

    // Layer 27 quadrant 2
    *((volatile uint32_t *)0x53101b04) = 0x0002000d; // Rows
    *((volatile uint32_t *)0x53101b08) = 0x0001000d; // Columns
    *((volatile uint32_t *)0x53101b18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x53101b1c) = 0x00043000; // SRAM write ptr
    *((volatile uint32_t *)0x53101b20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x53101b28) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53101b2c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x53101b30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53101b34) = 0x00000011; // Layer control 2
    *((volatile uint32_t *)0x53101b3c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x53101b40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x53101b0c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x53101b4c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x53101b48) = 0x0000ffff; // Mask and processor enables

    // Layer 27 quadrant 3
    *((volatile uint32_t *)0x54101b04) = 0x0002000d; // Rows
    *((volatile uint32_t *)0x54101b08) = 0x0001000d; // Columns
    *((volatile uint32_t *)0x54101b18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x54101b1c) = 0x00063000; // SRAM write ptr
    *((volatile uint32_t *)0x54101b20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x54101b28) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54101b2c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x54101b30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54101b34) = 0x00000011; // Layer control 2
    *((volatile uint32_t *)0x54101b3c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x54101b40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x54101b0c) = 0x00000103; // 1D
    *((volatile uint32_t *)0x54101b4c) = 0x03000000; // Post processing register
    *((volatile uint32_t *)0x54101b48) = 0x0000ffff; // Mask and processor enables

    // Layer 28 quadrant 0
    *((volatile uint32_t *)0x51101c04) = 0x0002000d; // Rows
    *((volatile uint32_t *)0x51101c08) = 0x0001000d; // Columns
    *((volatile uint32_t *)0x51101c18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x51101c20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51101c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51101c28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51101c2c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x51101c30) = 0x0000eb20; // Layer control
    *((volatile uint32_t *)0x51101c34) = 0x001f8071; // Layer control 2
    *((volatile uint32_t *)0x51101c38) = 0x00013a58; // Mask count
    *((volatile uint32_t *)0x51101c3c) = 0x00011a60; // Mask offset
    *((volatile uint32_t *)0x51101c40) = 0x000003ff; // Output channel count
    *((volatile uint32_t *)0x51101c0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51101c48) = 0xffffffff; // Mask and processor enables

    // Layer 28 quadrant 1
    *((volatile uint32_t *)0x52101c04) = 0x0002000d; // Rows
    *((volatile uint32_t *)0x52101c08) = 0x0001000d; // Columns
    *((volatile uint32_t *)0x52101c18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x52101c20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52101c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52101c28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52101c2c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x52101c30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x52101c34) = 0x001f8071; // Layer control 2
    *((volatile uint32_t *)0x52101c38) = 0x00013a58; // Mask count
    *((volatile uint32_t *)0x52101c3c) = 0x00011a60; // Mask offset
    *((volatile uint32_t *)0x52101c40) = 0x000003ff; // Output channel count
    *((volatile uint32_t *)0x52101c0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x52101c4c) = 0x00001100; // Post processing register
    *((volatile uint32_t *)0x52101c48) = 0xffffffff; // Mask and processor enables

    // Layer 28 quadrant 2
    *((volatile uint32_t *)0x53101c04) = 0x0002000d; // Rows
    *((volatile uint32_t *)0x53101c08) = 0x0001000d; // Columns
    *((volatile uint32_t *)0x53101c18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x53101c20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53101c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53101c28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53101c2c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x53101c30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x53101c34) = 0x001f8071; // Layer control 2
    *((volatile uint32_t *)0x53101c38) = 0x00013a58; // Mask count
    *((volatile uint32_t *)0x53101c3c) = 0x00011a60; // Mask offset
    *((volatile uint32_t *)0x53101c40) = 0x000003ff; // Output channel count
    *((volatile uint32_t *)0x53101c0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x53101c48) = 0xffffffff; // Mask and processor enables

    // Layer 28 quadrant 3
    *((volatile uint32_t *)0x54101c04) = 0x0002000d; // Rows
    *((volatile uint32_t *)0x54101c08) = 0x0001000d; // Columns
    *((volatile uint32_t *)0x54101c18) = 0x00000020; // Stride
    *((volatile uint32_t *)0x54101c20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54101c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54101c28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54101c2c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x54101c30) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x54101c34) = 0x001f8071; // Layer control 2
    *((volatile uint32_t *)0x54101c38) = 0x00013a58; // Mask count
    *((volatile uint32_t *)0x54101c3c) = 0x00011a60; // Mask offset
    *((volatile uint32_t *)0x54101c40) = 0x000003ff; // Output channel count
    *((volatile uint32_t *)0x54101c0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x54101c48) = 0xffffffff; // Mask and processor enables

    // Layer 29 quadrant 0
    *((volatile uint32_t *)0x51101d04) = 0x0008800d; // Rows
    *((volatile uint32_t *)0x51101d08) = 0x0001800d; // Columns
    *((volatile uint32_t *)0x51101d18) = 0x00000080; // Stride
    *((volatile uint32_t *)0x51101d1c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x51101d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51101d28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51101d30) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x51101d34) = 0x001f8077; // Layer control 2
    *((volatile uint32_t *)0x51101d38) = 0x00002338; // Mask count
    *((volatile uint32_t *)0x51101d3c) = 0x00002300; // Mask offset
    *((volatile uint32_t *)0x51101d40) = 0x0000001f; // Output channel count
    *((volatile uint32_t *)0x51101d0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x51101d44) = 0x0000000d; // TRAM ptr max
    *((volatile uint32_t *)0x51101d4c) = 0x41005020; // Post processing register
    *((volatile uint32_t *)0x51101d48) = 0xffffffff; // Mask and processor enables

    // Layer 29 quadrant 1
    *((volatile uint32_t *)0x52101d04) = 0x0008800d; // Rows
    *((volatile uint32_t *)0x52101d08) = 0x0001800d; // Columns
    *((volatile uint32_t *)0x52101d18) = 0x00000080; // Stride
    *((volatile uint32_t *)0x52101d1c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x52101d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52101d28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52101d30) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x52101d34) = 0x001f8077; // Layer control 2
    *((volatile uint32_t *)0x52101d38) = 0x00002338; // Mask count
    *((volatile uint32_t *)0x52101d3c) = 0x00002300; // Mask offset
    *((volatile uint32_t *)0x52101d40) = 0x0000001f; // Output channel count
    *((volatile uint32_t *)0x52101d0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x52101d44) = 0x0000000d; // TRAM ptr max
    *((volatile uint32_t *)0x52101d4c) = 0x41005020; // Post processing register
    *((volatile uint32_t *)0x52101d48) = 0xffffffff; // Mask and processor enables

    // Layer 29 quadrant 2
    *((volatile uint32_t *)0x53101d04) = 0x0008800d; // Rows
    *((volatile uint32_t *)0x53101d08) = 0x0001800d; // Columns
    *((volatile uint32_t *)0x53101d18) = 0x00000080; // Stride
    *((volatile uint32_t *)0x53101d1c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x53101d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53101d28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53101d30) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x53101d34) = 0x001f8077; // Layer control 2
    *((volatile uint32_t *)0x53101d38) = 0x00002338; // Mask count
    *((volatile uint32_t *)0x53101d3c) = 0x00002300; // Mask offset
    *((volatile uint32_t *)0x53101d40) = 0x0000001f; // Output channel count
    *((volatile uint32_t *)0x53101d0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x53101d44) = 0x0000000d; // TRAM ptr max
    *((volatile uint32_t *)0x53101d4c) = 0x41005020; // Post processing register
    *((volatile uint32_t *)0x53101d48) = 0xffffffff; // Mask and processor enables

    // Layer 29 quadrant 3
    *((volatile uint32_t *)0x54101d04) = 0x0008800d; // Rows
    *((volatile uint32_t *)0x54101d08) = 0x0001800d; // Columns
    *((volatile uint32_t *)0x54101d18) = 0x00000080; // Stride
    *((volatile uint32_t *)0x54101d1c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x54101d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54101d28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54101d30) = 0x20880b20; // Layer control
    *((volatile uint32_t *)0x54101d34) = 0x001f8077; // Layer control 2
    *((volatile uint32_t *)0x54101d38) = 0x00002338; // Mask count
    *((volatile uint32_t *)0x54101d3c) = 0x00002300; // Mask offset
    *((volatile uint32_t *)0x54101d40) = 0x0000001f; // Output channel count
    *((volatile uint32_t *)0x54101d0c) = 0x00000003; // 1D
    *((volatile uint32_t *)0x54101d44) = 0x0000000d; // TRAM ptr max
    *((volatile uint32_t *)0x54101d4c) = 0x41005018; // Post processing register
    *((volatile uint32_t *)0x54101d48) = 0xffffffff; // Mask and processor enables

    // Layer 30 quadrant 0
    *((volatile uint32_t *)0x51101e04) = 0x0008000d; // Rows
    *((volatile uint32_t *)0x51101e08) = 0x0001000d; // Columns
    *((volatile uint32_t *)0x51101e18) = 0x00000080; // Stride
    *((volatile uint32_t *)0x51101e1c) = 0x00003001; // SRAM write ptr
    *((volatile uint32_t *)0x51101e20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51101e24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51101e28) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51101e2c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x51101e30) = 0x0000e920; // Layer control
    *((volatile uint32_t *)0x51101e34) = 0x001f8037; // Layer control 2
    *((volatile uint32_t *)0x51101e38) = 0x00015d38; // Mask count
    *((volatile uint32_t *)0x51101e3c) = 0x00013d40; // Mask offset
    *((volatile uint32_t *)0x51101e40) = 0x000003ff; // Output channel count
    *((volatile uint32_t *)0x51101e0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51101e4c) = 0x00002000; // Post processing register
    *((volatile uint32_t *)0x51101e48) = 0xffffffff; // Mask and processor enables

    // Layer 30 quadrant 1
    *((volatile uint32_t *)0x52101e04) = 0x0008000d; // Rows
    *((volatile uint32_t *)0x52101e08) = 0x0001000d; // Columns
    *((volatile uint32_t *)0x52101e18) = 0x00000080; // Stride
    *((volatile uint32_t *)0x52101e1c) = 0x00003001; // SRAM write ptr
    *((volatile uint32_t *)0x52101e20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52101e24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52101e28) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52101e2c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x52101e30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52101e34) = 0x001f8037; // Layer control 2
    *((volatile uint32_t *)0x52101e38) = 0x00015d38; // Mask count
    *((volatile uint32_t *)0x52101e3c) = 0x00013d40; // Mask offset
    *((volatile uint32_t *)0x52101e40) = 0x000003ff; // Output channel count
    *((volatile uint32_t *)0x52101e0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x52101e4c) = 0x00002000; // Post processing register
    *((volatile uint32_t *)0x52101e48) = 0xffffffff; // Mask and processor enables

    // Layer 30 quadrant 2
    *((volatile uint32_t *)0x53101e04) = 0x0008000d; // Rows
    *((volatile uint32_t *)0x53101e08) = 0x0001000d; // Columns
    *((volatile uint32_t *)0x53101e18) = 0x00000080; // Stride
    *((volatile uint32_t *)0x53101e1c) = 0x00003001; // SRAM write ptr
    *((volatile uint32_t *)0x53101e20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53101e24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53101e28) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53101e2c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x53101e30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53101e34) = 0x001f8037; // Layer control 2
    *((volatile uint32_t *)0x53101e38) = 0x00015d38; // Mask count
    *((volatile uint32_t *)0x53101e3c) = 0x00013d40; // Mask offset
    *((volatile uint32_t *)0x53101e40) = 0x000003ff; // Output channel count
    *((volatile uint32_t *)0x53101e0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x53101e4c) = 0x00003440; // Post processing register
    *((volatile uint32_t *)0x53101e48) = 0xffffffff; // Mask and processor enables

    // Layer 30 quadrant 3
    *((volatile uint32_t *)0x54101e04) = 0x0008000d; // Rows
    *((volatile uint32_t *)0x54101e08) = 0x0001000d; // Columns
    *((volatile uint32_t *)0x54101e18) = 0x00000080; // Stride
    *((volatile uint32_t *)0x54101e1c) = 0x00003001; // SRAM write ptr
    *((volatile uint32_t *)0x54101e20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54101e24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54101e28) = 0x00000002; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54101e2c) = 0x00002000; // SRAM read ptr
    *((volatile uint32_t *)0x54101e30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54101e34) = 0x001f8037; // Layer control 2
    *((volatile uint32_t *)0x54101e38) = 0x00015d38; // Mask count
    *((volatile uint32_t *)0x54101e3c) = 0x00013d40; // Mask offset
    *((volatile uint32_t *)0x54101e40) = 0x000003ff; // Output channel count
    *((volatile uint32_t *)0x54101e0c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x54101e4c) = 0x00002000; // Post processing register
    *((volatile uint32_t *)0x54101e48) = 0xffffffff; // Mask and processor enables

    // Layer 31 quadrant 0
    *((volatile uint32_t *)0x51101f04) = 0x0004000d; // Rows
    *((volatile uint32_t *)0x51101f08) = 0x0001000d; // Columns
    *((volatile uint32_t *)0x51101f18) = 0x00000040; // Stride
    *((volatile uint32_t *)0x51101f20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x51101f28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51101f2c) = 0x00003000; // SRAM read ptr
    *((volatile uint32_t *)0x51101f30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x51101f34) = 0x00000001; // Layer control 2
    *((volatile uint32_t *)0x51101f3c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x51101f40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x51101f0c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x51101f4c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x51101f48) = 0x0000ffff; // Mask and processor enables

    // Layer 31 quadrant 1
    *((volatile uint32_t *)0x52101f04) = 0x0004000d; // Rows
    *((volatile uint32_t *)0x52101f08) = 0x0001000d; // Columns
    *((volatile uint32_t *)0x52101f18) = 0x00000040; // Stride
    *((volatile uint32_t *)0x52101f1c) = 0x00020000; // SRAM write ptr
    *((volatile uint32_t *)0x52101f20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x52101f28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52101f2c) = 0x00003000; // SRAM read ptr
    *((volatile uint32_t *)0x52101f30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x52101f34) = 0x00000001; // Layer control 2
    *((volatile uint32_t *)0x52101f3c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x52101f40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x52101f0c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x52101f4c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x52101f48) = 0x0000ffff; // Mask and processor enables

    // Layer 31 quadrant 2
    *((volatile uint32_t *)0x53101f04) = 0x0004000d; // Rows
    *((volatile uint32_t *)0x53101f08) = 0x0001000d; // Columns
    *((volatile uint32_t *)0x53101f18) = 0x00000040; // Stride
    *((volatile uint32_t *)0x53101f1c) = 0x00040000; // SRAM write ptr
    *((volatile uint32_t *)0x53101f20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x53101f28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53101f2c) = 0x00003000; // SRAM read ptr
    *((volatile uint32_t *)0x53101f30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x53101f34) = 0x00000001; // Layer control 2
    *((volatile uint32_t *)0x53101f3c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x53101f40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x53101f0c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x53101f4c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x53101f48) = 0x0000ffff; // Mask and processor enables

    // Layer 31 quadrant 3
    *((volatile uint32_t *)0x54101f04) = 0x0004000d; // Rows
    *((volatile uint32_t *)0x54101f08) = 0x0001000d; // Columns
    *((volatile uint32_t *)0x54101f18) = 0x00000040; // Stride
    *((volatile uint32_t *)0x54101f1c) = 0x00060000; // SRAM write ptr
    *((volatile uint32_t *)0x54101f20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t *)0x54101f28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54101f2c) = 0x00003000; // SRAM read ptr
    *((volatile uint32_t *)0x54101f30) = 0x00000920; // Layer control
    *((volatile uint32_t *)0x54101f34) = 0x00000001; // Layer control 2
    *((volatile uint32_t *)0x54101f3c) = 0x00000008; // Mask offset
    *((volatile uint32_t *)0x54101f40) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x54101f0c) = 0x00046003; // 1D
    *((volatile uint32_t *)0x54101f4c) = 0x01000000; // Post processing register
    *((volatile uint32_t *)0x54101f48) = 0x0000ffff; // Mask and processor enables

    // Layer 32 quadrant 0
    *((volatile uint32_t *)0x51102004) = 0x0002000d; // Rows
    *((volatile uint32_t *)0x51102008) = 0x0001000d; // Columns
    *((volatile uint32_t *)0x51102018) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5110201c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x51102020) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51102024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51102028) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51102030) = 0x0000eb20; // Layer control
    *((volatile uint32_t *)0x51102034) = 0x001f80f1; // Layer control 2
    *((volatile uint32_t *)0x51102038) = 0x00019dd8; // Mask count
    *((volatile uint32_t *)0x5110203c) = 0x00015de0; // Mask offset
    *((volatile uint32_t *)0x51102040) = 0x000007ff; // Output channel count
    *((volatile uint32_t *)0x5110200c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x51102048) = 0xffffffff; // Mask and processor enables

    // Layer 32 quadrant 1
    *((volatile uint32_t *)0x52102004) = 0x0002000d; // Rows
    *((volatile uint32_t *)0x52102008) = 0x0001000d; // Columns
    *((volatile uint32_t *)0x52102018) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5210201c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x52102020) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52102024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52102028) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52102030) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x52102034) = 0x001f80f1; // Layer control 2
    *((volatile uint32_t *)0x52102038) = 0x00019dd8; // Mask count
    *((volatile uint32_t *)0x5210203c) = 0x00015de0; // Mask offset
    *((volatile uint32_t *)0x52102040) = 0x000007ff; // Output channel count
    *((volatile uint32_t *)0x5210200c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x52102048) = 0xffffffff; // Mask and processor enables

    // Layer 32 quadrant 2
    *((volatile uint32_t *)0x53102004) = 0x0002000d; // Rows
    *((volatile uint32_t *)0x53102008) = 0x0001000d; // Columns
    *((volatile uint32_t *)0x53102018) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5310201c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x53102020) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53102024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53102028) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53102030) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x53102034) = 0x001f80f1; // Layer control 2
    *((volatile uint32_t *)0x53102038) = 0x00019dd8; // Mask count
    *((volatile uint32_t *)0x5310203c) = 0x00015de0; // Mask offset
    *((volatile uint32_t *)0x53102040) = 0x000007ff; // Output channel count
    *((volatile uint32_t *)0x5310200c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x53102048) = 0xffffffff; // Mask and processor enables

    // Layer 32 quadrant 3
    *((volatile uint32_t *)0x54102004) = 0x0002000d; // Rows
    *((volatile uint32_t *)0x54102008) = 0x0001000d; // Columns
    *((volatile uint32_t *)0x54102018) = 0x00000020; // Stride
    *((volatile uint32_t *)0x5410201c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t *)0x54102020) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54102024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54102028) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54102030) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x54102034) = 0x001f80f1; // Layer control 2
    *((volatile uint32_t *)0x54102038) = 0x00019dd8; // Mask count
    *((volatile uint32_t *)0x5410203c) = 0x00015de0; // Mask offset
    *((volatile uint32_t *)0x54102040) = 0x000007ff; // Output channel count
    *((volatile uint32_t *)0x5410200c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410204c) = 0x000010e0; // Post processing register
    *((volatile uint32_t *)0x54102048) = 0xffffffff; // Mask and processor enables

    // Layer 33 quadrant 0
    *((volatile uint32_t *)0x51102104) = 0x0c400000; // Rows
    *((volatile uint32_t *)0x51102108) = 0x000e0000; // Columns
    *((volatile uint32_t *)0x51102110) = 0x0000000d; // Pooling rows
    *((volatile uint32_t *)0x51102114) = 0x0000000d; // Pooling columns
    *((volatile uint32_t *)0x51102118) = 0x00000e0d; // Stride
    *((volatile uint32_t *)0x5110211c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x51102120) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x51102124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51102128) = 0x00000004; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5110212c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x51102130) = 0x0001e8a0; // Layer control
    *((volatile uint32_t *)0x51102134) = 0x001f80ff; // Layer control 2
    *((volatile uint32_t *)0x51102138) = 0x00039df8; // Mask count
    *((volatile uint32_t *)0x5110213c) = 0x00019e00; // Mask offset
    *((volatile uint32_t *)0x51102140) = 0x00003fff; // Output channel count
    *((volatile uint32_t *)0x5110210c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5110214c) = 0x00003100; // Post processing register
    *((volatile uint32_t *)0x51102148) = 0xffffffff; // Mask and processor enables

    // Layer 33 quadrant 1
    *((volatile uint32_t *)0x52102104) = 0x0c400000; // Rows
    *((volatile uint32_t *)0x52102108) = 0x000e0000; // Columns
    *((volatile uint32_t *)0x52102110) = 0x0000000d; // Pooling rows
    *((volatile uint32_t *)0x52102114) = 0x0000000d; // Pooling columns
    *((volatile uint32_t *)0x52102118) = 0x00000e0d; // Stride
    *((volatile uint32_t *)0x5210211c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x52102120) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x52102124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52102128) = 0x00000004; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5210212c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x52102130) = 0x000108a0; // Layer control
    *((volatile uint32_t *)0x52102134) = 0x001f80ff; // Layer control 2
    *((volatile uint32_t *)0x52102138) = 0x00039df8; // Mask count
    *((volatile uint32_t *)0x5210213c) = 0x00019e00; // Mask offset
    *((volatile uint32_t *)0x52102140) = 0x00003fff; // Output channel count
    *((volatile uint32_t *)0x5210210c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5210214c) = 0x00002000; // Post processing register
    *((volatile uint32_t *)0x52102148) = 0xffffffff; // Mask and processor enables

    // Layer 33 quadrant 2
    *((volatile uint32_t *)0x53102104) = 0x0c400000; // Rows
    *((volatile uint32_t *)0x53102108) = 0x000e0000; // Columns
    *((volatile uint32_t *)0x53102110) = 0x0000000d; // Pooling rows
    *((volatile uint32_t *)0x53102114) = 0x0000000d; // Pooling columns
    *((volatile uint32_t *)0x53102118) = 0x00000e0d; // Stride
    *((volatile uint32_t *)0x5310211c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x53102120) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x53102124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53102128) = 0x00000004; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5310212c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x53102130) = 0x000108a0; // Layer control
    *((volatile uint32_t *)0x53102134) = 0x001f80ff; // Layer control 2
    *((volatile uint32_t *)0x53102138) = 0x00039df8; // Mask count
    *((volatile uint32_t *)0x5310213c) = 0x00019e00; // Mask offset
    *((volatile uint32_t *)0x53102140) = 0x00003fff; // Output channel count
    *((volatile uint32_t *)0x5310210c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5310214c) = 0x00002000; // Post processing register
    *((volatile uint32_t *)0x53102148) = 0xffffffff; // Mask and processor enables

    // Layer 33 quadrant 3
    *((volatile uint32_t *)0x54102104) = 0x0c400000; // Rows
    *((volatile uint32_t *)0x54102108) = 0x000e0000; // Columns
    *((volatile uint32_t *)0x54102110) = 0x0000000d; // Pooling rows
    *((volatile uint32_t *)0x54102114) = 0x0000000d; // Pooling columns
    *((volatile uint32_t *)0x54102118) = 0x00000e0d; // Stride
    *((volatile uint32_t *)0x5410211c) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x54102120) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x54102124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54102128) = 0x00000004; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x5410212c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t *)0x54102130) = 0x000108a0; // Layer control
    *((volatile uint32_t *)0x54102134) = 0x001f80ff; // Layer control 2
    *((volatile uint32_t *)0x54102138) = 0x00039df8; // Mask count
    *((volatile uint32_t *)0x5410213c) = 0x00019e00; // Mask offset
    *((volatile uint32_t *)0x54102140) = 0x00003fff; // Output channel count
    *((volatile uint32_t *)0x5410210c) = 0x00000100; // 1D
    *((volatile uint32_t *)0x5410214c) = 0x00002000; // Post processing register
    *((volatile uint32_t *)0x54102148) = 0xffffffff; // Mask and processor enables

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

    // Custom unload for this network, layer 33 (l33): 32-bit data, shape: (1000, 1, 1)
    addr = (volatile uint32_t *)0x51808000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51828000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51848000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51868000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52808000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52828000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52848000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52868000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53808000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53828000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53848000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53868000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54808000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54828000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54848000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54868000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51808010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51828010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51848010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51868010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52808010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52828010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52848010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52868010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53808010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53828010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53848010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53868010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54808010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54828010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54848010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54868010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51808020;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51828020;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51848020;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51868020;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52808020;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52828020;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52848020;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52868020;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53808020;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53828020;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53848020;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53868020;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54808020;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54828020;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54848020;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54868020;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51808030;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51828030;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51848030;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51868030;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52808030;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52828030;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52848030;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52868030;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53808030;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53828030;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53848030;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53868030;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54808030;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54828030;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54848030;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54868030;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51808040;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51828040;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51848040;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51868040;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52808040;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52828040;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52848040;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52868040;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53808040;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53828040;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53848040;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53868040;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54808040;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54828040;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54848040;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54868040;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51808050;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51828050;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51848050;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51868050;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52808050;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52828050;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52848050;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52868050;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53808050;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53828050;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53848050;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53868050;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54808050;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54828050;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54848050;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54868050;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51808060;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51828060;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51848060;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51868060;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52808060;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52828060;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52848060;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52868060;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53808060;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53828060;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53848060;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53868060;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54808060;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54828060;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54848060;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54868060;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51808070;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51828070;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51848070;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51868070;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52808070;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52828070;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52848070;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52868070;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53808070;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53828070;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53848070;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53868070;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54808070;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54828070;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54848070;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54868070;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51808080;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51828080;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51848080;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51868080;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52808080;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52828080;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52848080;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52868080;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53808080;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53828080;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53848080;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53868080;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54808080;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54828080;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54848080;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54868080;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51808090;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51828090;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51848090;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x51868090;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52808090;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52828090;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52848090;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x52868090;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53808090;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53828090;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53848090;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x53868090;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54808090;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54828090;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54848090;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x54868090;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x518080a0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x518280a0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x518480a0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x518680a0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x528080a0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x528280a0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x528480a0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x528680a0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x538080a0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x538280a0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x538480a0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x538680a0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x548080a0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x548280a0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x548480a0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x548680a0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x518080b0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x518280b0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x518480b0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x518680b0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x528080b0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x528280b0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x528480b0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x528680b0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x538080b0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x538280b0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x538480b0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x538680b0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x548080b0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x548280b0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x548480b0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x548680b0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x518080c0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x518280c0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x518480c0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x518680c0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x528080c0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x528280c0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x528480c0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x528680c0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x538080c0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x538280c0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x538480c0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x538680c0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x548080c0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x548280c0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x548480c0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x548680c0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x518080d0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x518280d0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x518480d0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x518680d0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x528080d0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x528280d0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x528480d0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x528680d0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x538080d0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x538280d0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x538480d0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x538680d0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x548080d0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x548280d0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x548480d0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x548680d0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x518080e0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x518280e0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x518480e0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x518680e0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x528080e0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x528280e0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x528480e0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x528680e0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x538080e0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x538280e0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x538480e0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x538680e0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x548080e0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x548280e0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x548480e0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x548680e0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x518080f0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x518280f0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x518480f0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x518680f0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x528080f0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x528280f0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x528480f0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x528680f0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x538080f0;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr = (volatile uint32_t *)0x538280f0;
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
