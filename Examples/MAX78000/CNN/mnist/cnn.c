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

// mnist
// Created using ai8xize.py --test-dir sdk/Examples/MAX78000/CNN --prefix mnist --checkpoint-file trained/ai85-mnist-qat8-q.pth.tar --config-file networks/mnist-chw-ai85.yaml --softmax --device MAX78000 --timer 0 --display-checkpoint --verbose

// DO NOT EDIT - regenerate this file instead!

// Configuring 5 layers
// Input data: CHW
// Layer 0: 1x28x28, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 60x28x28 output
// Layer 1: 60x28x28, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 2/2, ReLU, 60x16x16 output
// Layer 2: 60x16x16, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 56x8x8 output
// Layer 3: 56x8x8, avg pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 12x4x4 output
// Layer 4: 12x4x4 flattened to 192x1x1, no pooling, linear, no activation, 10x1x1 output

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
    *((volatile uint32_t *)0x50100000) &= ~((1 << 12) | 1);
    *((volatile uint32_t *)0x50500000) &= ~((1 << 12) | 1);
    *((volatile uint32_t *)0x50900000) &= ~((1 << 12) | 1);
    *((volatile uint32_t *)0x50d00000) &= ~((1 << 12) | 1);

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

    *((volatile uint32_t *)0x50100000) |= 1; // Re-enable quadrant 0

    return CNN_OK;
}

int cnn_stop(void)
{
    *((volatile uint32_t *)0x50100000) &= ~1; // Disable quadrant 0

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

static void memcpy_8to32(uint32_t *dst, const uint8_t *src, int n)
{
    while (n-- > 0) {
        *dst++ = *src++;
    }
}

int cnn_load_bias(void)
{
    memcpy_8to32((uint32_t *)0x50108000, bias_0, sizeof(uint8_t) * 10);

    return CNN_OK;
}

int cnn_init(void)
{
    *((volatile uint32_t *)0x50001000) = 0x00000000; // AON control
    *((volatile uint32_t *)0x50100000) = 0x00100008; // Stop SM
    *((volatile uint32_t *)0x50100004) = 0x0000040e; // SRAM control
    *((volatile uint32_t *)0x50100008) = 0x00000004; // Layer count
    *((volatile uint32_t *)0x50500000) = 0x00100008; // Stop SM
    *((volatile uint32_t *)0x50500004) = 0x0000040e; // SRAM control
    *((volatile uint32_t *)0x50500008) = 0x00000004; // Layer count
    *((volatile uint32_t *)0x50900000) = 0x00100008; // Stop SM
    *((volatile uint32_t *)0x50900004) = 0x0000040e; // SRAM control
    *((volatile uint32_t *)0x50900008) = 0x00000004; // Layer count
    *((volatile uint32_t *)0x50d00000) = 0x00100008; // Stop SM
    *((volatile uint32_t *)0x50d00004) = 0x0000040e; // SRAM control
    *((volatile uint32_t *)0x50d00008) = 0x00000004; // Layer count

    return CNN_OK;
}

int cnn_configure(void)
{
    // Layer 0 quadrant 0
    *((volatile uint32_t *)0x50100010) = 0x0001001d; // Rows
    *((volatile uint32_t *)0x50100090) = 0x0001001d; // Columns
    *((volatile uint32_t *)0x50100310) = 0x00002800; // SRAM write ptr
    *((volatile uint32_t *)0x50100410) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50100590) = 0x00000b60; // Layer control
    *((volatile uint32_t *)0x50100a10) = 0x0001d800; // Layer control 2
    *((volatile uint32_t *)0x50100610) = 0x000001d8; // Mask offset and count
    *((volatile uint32_t *)0x50100690) = 0x0000001b; // TRAM ptr max
    *((volatile uint32_t *)0x50100790) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x50100710) = 0x00010001; // Mask and processor enables

    // Layer 0 quadrant 1
    *((volatile uint32_t *)0x50500010) = 0x0001001d; // Rows
    *((volatile uint32_t *)0x50500090) = 0x0001001d; // Columns
    *((volatile uint32_t *)0x50500310) = 0x00002800; // SRAM write ptr
    *((volatile uint32_t *)0x50500410) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50500590) = 0x00000b60; // Layer control
    *((volatile uint32_t *)0x50500a10) = 0x0001d800; // Layer control 2
    *((volatile uint32_t *)0x50500610) = 0x000001d8; // Mask offset and count
    *((volatile uint32_t *)0x50500690) = 0x0000001b; // TRAM ptr max
    *((volatile uint32_t *)0x50500790) = 0x00022000; // Post processing register

    // Layer 0 quadrant 2
    *((volatile uint32_t *)0x50900010) = 0x0001001d; // Rows
    *((volatile uint32_t *)0x50900090) = 0x0001001d; // Columns
    *((volatile uint32_t *)0x50900310) = 0x00002800; // SRAM write ptr
    *((volatile uint32_t *)0x50900410) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50900590) = 0x00000b60; // Layer control
    *((volatile uint32_t *)0x50900a10) = 0x0001d800; // Layer control 2
    *((volatile uint32_t *)0x50900610) = 0x000001d8; // Mask offset and count
    *((volatile uint32_t *)0x50900690) = 0x0000001b; // TRAM ptr max
    *((volatile uint32_t *)0x50900790) = 0x00022000; // Post processing register

    // Layer 0 quadrant 3
    *((volatile uint32_t *)0x50d00010) = 0x0001001d; // Rows
    *((volatile uint32_t *)0x50d00090) = 0x0001001d; // Columns
    *((volatile uint32_t *)0x50d00310) = 0x00002800; // SRAM write ptr
    *((volatile uint32_t *)0x50d00410) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50d00590) = 0x00000b60; // Layer control
    *((volatile uint32_t *)0x50d00a10) = 0x0001d800; // Layer control 2
    *((volatile uint32_t *)0x50d00610) = 0x000001d8; // Mask offset and count
    *((volatile uint32_t *)0x50d00690) = 0x0000001b; // TRAM ptr max
    *((volatile uint32_t *)0x50d00790) = 0x00022000; // Post processing register

    // Layer 1 quadrant 0
    *((volatile uint32_t *)0x50100014) = 0x0002001f; // Rows
    *((volatile uint32_t *)0x50100094) = 0x0002001f; // Columns
    *((volatile uint32_t *)0x50100194) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50100214) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50100294) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50100314) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x50100414) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50100514) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x50100594) = 0x0000eba0; // Layer control
    *((volatile uint32_t *)0x50100a14) = 0x0001d800; // Layer control 2
    *((volatile uint32_t *)0x50100614) = 0x000001d8; // Mask offset and count
    *((volatile uint32_t *)0x50100694) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t *)0x50100794) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x50100714) = 0xfff0fff0; // Mask and processor enables

    // Layer 1 quadrant 1
    *((volatile uint32_t *)0x50500014) = 0x0002001f; // Rows
    *((volatile uint32_t *)0x50500094) = 0x0002001f; // Columns
    *((volatile uint32_t *)0x50500194) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50500214) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50500294) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50500314) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x50500414) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50500514) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x50500594) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x50500a14) = 0x0001d800; // Layer control 2
    *((volatile uint32_t *)0x50500614) = 0x000001d8; // Mask offset and count
    *((volatile uint32_t *)0x50500694) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t *)0x50500794) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x50500714) = 0xffffffff; // Mask and processor enables

    // Layer 1 quadrant 2
    *((volatile uint32_t *)0x50900014) = 0x0002001f; // Rows
    *((volatile uint32_t *)0x50900094) = 0x0002001f; // Columns
    *((volatile uint32_t *)0x50900194) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50900214) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50900294) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50900314) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x50900414) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50900514) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x50900594) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x50900a14) = 0x0001d800; // Layer control 2
    *((volatile uint32_t *)0x50900614) = 0x000001d8; // Mask offset and count
    *((volatile uint32_t *)0x50900694) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t *)0x50900794) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x50900714) = 0xffffffff; // Mask and processor enables

    // Layer 1 quadrant 3
    *((volatile uint32_t *)0x50d00014) = 0x0002001f; // Rows
    *((volatile uint32_t *)0x50d00094) = 0x0002001f; // Columns
    *((volatile uint32_t *)0x50d00194) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50d00214) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50d00294) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50d00314) = 0x00002000; // SRAM write ptr
    *((volatile uint32_t *)0x50d00414) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50d00514) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x50d00594) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x50d00a14) = 0x0001d800; // Layer control 2
    *((volatile uint32_t *)0x50d00614) = 0x000001d8; // Mask offset and count
    *((volatile uint32_t *)0x50d00694) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t *)0x50d00794) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x50d00714) = 0xffffffff; // Mask and processor enables

    // Layer 2 quadrant 0
    *((volatile uint32_t *)0x50100018) = 0x00010011; // Rows
    *((volatile uint32_t *)0x50100098) = 0x00010011; // Columns
    *((volatile uint32_t *)0x50100198) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50100218) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50100298) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50100318) = 0x00002800; // SRAM write ptr
    *((volatile uint32_t *)0x50100418) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50100598) = 0x0000eba0; // Layer control
    *((volatile uint32_t *)0x50100a18) = 0x0001b800; // Layer control 2
    *((volatile uint32_t *)0x50100618) = 0x01e00398; // Mask offset and count
    *((volatile uint32_t *)0x50100698) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t *)0x50100798) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x50100718) = 0xfff0fff0; // Mask and processor enables

    // Layer 2 quadrant 1
    *((volatile uint32_t *)0x50500018) = 0x00010011; // Rows
    *((volatile uint32_t *)0x50500098) = 0x00010011; // Columns
    *((volatile uint32_t *)0x50500198) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50500218) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50500298) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50500318) = 0x00002800; // SRAM write ptr
    *((volatile uint32_t *)0x50500418) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50500598) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x50500a18) = 0x0001b800; // Layer control 2
    *((volatile uint32_t *)0x50500618) = 0x01e00398; // Mask offset and count
    *((volatile uint32_t *)0x50500698) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t *)0x50500798) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x50500718) = 0xffffffff; // Mask and processor enables

    // Layer 2 quadrant 2
    *((volatile uint32_t *)0x50900018) = 0x00010011; // Rows
    *((volatile uint32_t *)0x50900098) = 0x00010011; // Columns
    *((volatile uint32_t *)0x50900198) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50900218) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50900298) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50900318) = 0x00002800; // SRAM write ptr
    *((volatile uint32_t *)0x50900418) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50900598) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x50900a18) = 0x0001b800; // Layer control 2
    *((volatile uint32_t *)0x50900618) = 0x01e00398; // Mask offset and count
    *((volatile uint32_t *)0x50900698) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t *)0x50900798) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x50900718) = 0xffffffff; // Mask and processor enables

    // Layer 2 quadrant 3
    *((volatile uint32_t *)0x50d00018) = 0x00010011; // Rows
    *((volatile uint32_t *)0x50d00098) = 0x00010011; // Columns
    *((volatile uint32_t *)0x50d00198) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50d00218) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50d00298) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50d00318) = 0x00002800; // SRAM write ptr
    *((volatile uint32_t *)0x50d00418) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50d00598) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x50d00a18) = 0x0001b800; // Layer control 2
    *((volatile uint32_t *)0x50d00618) = 0x01e00398; // Mask offset and count
    *((volatile uint32_t *)0x50d00698) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t *)0x50d00798) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x50d00718) = 0xffffffff; // Mask and processor enables

    // Layer 3 quadrant 0
    *((volatile uint32_t *)0x5010001c) = 0x00010009; // Rows
    *((volatile uint32_t *)0x5010009c) = 0x00010009; // Columns
    *((volatile uint32_t *)0x5010019c) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x5010021c) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x5010029c) = 0x00000001; // Stride
    *((volatile uint32_t *)0x5010041c) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x5010051c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x5010059c) = 0x0000eaa0; // Layer control
    *((volatile uint32_t *)0x50100a1c) = 0x00005800; // Layer control 2
    *((volatile uint32_t *)0x5010061c) = 0x03a003f8; // Mask offset and count
    *((volatile uint32_t *)0x5010069c) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5010079c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x5010071c) = 0xfff0fff0; // Mask and processor enables

    // Layer 3 quadrant 1
    *((volatile uint32_t *)0x5050001c) = 0x00010009; // Rows
    *((volatile uint32_t *)0x5050009c) = 0x00010009; // Columns
    *((volatile uint32_t *)0x5050019c) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x5050021c) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x5050029c) = 0x00000001; // Stride
    *((volatile uint32_t *)0x5050041c) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x5050051c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x5050059c) = 0x00000aa0; // Layer control
    *((volatile uint32_t *)0x50500a1c) = 0x00005800; // Layer control 2
    *((volatile uint32_t *)0x5050061c) = 0x03a003f8; // Mask offset and count
    *((volatile uint32_t *)0x5050069c) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5050079c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x5050071c) = 0xffffffff; // Mask and processor enables

    // Layer 3 quadrant 2
    *((volatile uint32_t *)0x5090001c) = 0x00010009; // Rows
    *((volatile uint32_t *)0x5090009c) = 0x00010009; // Columns
    *((volatile uint32_t *)0x5090019c) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x5090021c) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x5090029c) = 0x00000001; // Stride
    *((volatile uint32_t *)0x5090041c) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x5090051c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x5090059c) = 0x00000aa0; // Layer control
    *((volatile uint32_t *)0x50900a1c) = 0x00005800; // Layer control 2
    *((volatile uint32_t *)0x5090061c) = 0x03a003f8; // Mask offset and count
    *((volatile uint32_t *)0x5090069c) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x5090079c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x5090071c) = 0xffffffff; // Mask and processor enables

    // Layer 3 quadrant 3
    *((volatile uint32_t *)0x50d0001c) = 0x00010009; // Rows
    *((volatile uint32_t *)0x50d0009c) = 0x00010009; // Columns
    *((volatile uint32_t *)0x50d0019c) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50d0021c) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50d0029c) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50d0041c) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50d0051c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x50d0059c) = 0x00000aa0; // Layer control
    *((volatile uint32_t *)0x50d00a1c) = 0x00005800; // Layer control 2
    *((volatile uint32_t *)0x50d0061c) = 0x03a003f8; // Mask offset and count
    *((volatile uint32_t *)0x50d0069c) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t *)0x50d0079c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x50d0071c) = 0x0fff0fff; // Mask and processor enables

    // Layer 4 quadrant 0
    *((volatile uint32_t *)0x50100320) = 0x00000400; // SRAM write ptr
    *((volatile uint32_t *)0x501003a0) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x50100420) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x501005a0) = 0x00010920; // Layer control
    *((volatile uint32_t *)0x50100a20) = 0x0000480f; // Layer control 2
    *((volatile uint32_t *)0x50100620) = 0x240028f8; // Mask offset and count
    *((volatile uint32_t *)0x50100120) = 0x00000100; // 1D
    *((volatile uint32_t *)0x501007a0) = 0x00001000; // Post processing register
    *((volatile uint32_t *)0x50100720) = 0x0fff0fff; // Mask and processor enables

    // Layer 4 quadrant 1
    *((volatile uint32_t *)0x50500320) = 0x00000400; // SRAM write ptr
    *((volatile uint32_t *)0x505003a0) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x50500420) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x505005a0) = 0x00010920; // Layer control
    *((volatile uint32_t *)0x50500a20) = 0x0000480f; // Layer control 2
    *((volatile uint32_t *)0x50500620) = 0x240028f8; // Mask offset and count
    *((volatile uint32_t *)0x50500120) = 0x00000100; // 1D

    // Layer 4 quadrant 2
    *((volatile uint32_t *)0x50900320) = 0x00000400; // SRAM write ptr
    *((volatile uint32_t *)0x509003a0) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x50900420) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x509005a0) = 0x00010920; // Layer control
    *((volatile uint32_t *)0x50900a20) = 0x0000480f; // Layer control 2
    *((volatile uint32_t *)0x50900620) = 0x240028f8; // Mask offset and count
    *((volatile uint32_t *)0x50900120) = 0x00000100; // 1D

    // Layer 4 quadrant 3
    *((volatile uint32_t *)0x50d00320) = 0x00000400; // SRAM write ptr
    *((volatile uint32_t *)0x50d003a0) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x50d00420) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50d005a0) = 0x00010920; // Layer control
    *((volatile uint32_t *)0x50d00a20) = 0x0000480f; // Layer control 2
    *((volatile uint32_t *)0x50d00620) = 0x240028f8; // Mask offset and count
    *((volatile uint32_t *)0x50d00120) = 0x00000100; // 1D

    return CNN_OK;
}

int cnn_start(void)
{
    cnn_time = 0;

    *((volatile uint32_t *)0x50100000) = 0x00100808; // Enable quadrant 0
    *((volatile uint32_t *)0x50500000) = 0x00100809; // Enable quadrant 1
    *((volatile uint32_t *)0x50900000) = 0x00100809; // Enable quadrant 2
    *((volatile uint32_t *)0x50d00000) = 0x00100809; // Enable quadrant 3

#ifdef CNN_INFERENCE_TIMER
    MXC_TMR_SW_Start(CNN_INFERENCE_TIMER);
#endif

    CNN_START; // Allow capture of processing time
    *((volatile uint32_t *)0x50100000) = 0x00100009; // Master enable quadrant 0

    return CNN_OK;
}

int cnn_unload(uint32_t *out_buf)
{
    volatile uint32_t *addr;

    // Custom unload for this network, layer 4: 32-bit data, shape: (10, 1, 1)
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

    return CNN_OK;
}

int cnn_enable(uint32_t clock_source, uint32_t clock_divider)
{
    // Reset all domains, restore power to CNN
    MXC_GCFR->reg3 = 0xf; // Reset
    MXC_GCFR->reg1 = 0xf; // Mask memory
    MXC_GCFR->reg0 = 0xf; // Power
    MXC_GCFR->reg2 = 0x0; // Iso
    MXC_GCFR->reg3 = 0x0; // Reset

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
