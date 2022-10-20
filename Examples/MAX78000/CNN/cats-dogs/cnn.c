/*******************************************************************************
* Copyright (C) 2019-2022 Maxim Integrated Products, Inc., All rights Reserved.
*
* This software is protected by copyright laws of the United States and
* of foreign countries. This material may also be protected by patent laws
* and technology transfer regulations of the United States and of foreign
* countries. This software is furnished under a license agreement and/or a
* nondisclosure agreement and may only be used or reproduced in accordance
* with the terms of those agreements. Dissemination of this information to
* any party or parties not specified in the license agreement and/or
* nondisclosure agreement is expressly prohibited.
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
*******************************************************************************/

// cats-dogs
// Created using ai8xize.py --test-dir sdk/Examples/MAX78000/CNN --prefix cats-dogs --checkpoint-file trained/ai85-catsdogs-qat8-q.pth.tar --config-file networks/cats-dogs-hwc.yaml --fifo --softmax --device MAX78000 --timer 0 --display-checkpoint --verbose

// DO NOT EDIT - regenerate this file instead!

// Configuring 7 layers
// Input data: HWC
// Layer 0: 3x128x128 streaming, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 16x128x128 output
// Layer 1: 16x128x128 streaming, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 32x64x64 output
// Layer 2: 32x64x64, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 64x32x32 output
// Layer 3: 64x32x32, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 32x16x16 output
// Layer 4: 32x16x16, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 32x8x8 output
// Layer 5: 32x8x8, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 16x8x8 output
// Layer 6: 16x8x8 flattened to 1024x1x1, no pooling, linear, no activation, 2x1x1 output

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
    *((volatile uint32_t *)0x50100000) &= ~((1 << 12) | (1 << 14) | 1);
    *((volatile uint32_t *)0x50500000) &= ~((1 << 12) | (1 << 14) | 1);
    *((volatile uint32_t *)0x50900000) &= ~((1 << 12) | (1 << 14) | 1);
    *((volatile uint32_t *)0x50d00000) &= ~((1 << 12) | (1 << 14) | 1);

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
    memcpy_8to32((uint32_t *)0x50108000, bias_0, sizeof(uint8_t) * 2);

    return CNN_OK;
}

int cnn_init(void)
{
    *((volatile uint32_t *)0x50001000) = 0x00000000; // AON control
    *((volatile uint32_t *)0x50100000) = 0x00108008; // Stop SM
    *((volatile uint32_t *)0x50100004) = 0x0000040e; // SRAM control
    *((volatile uint32_t *)0x50100008) = 0x00000006; // Layer count
    *((volatile uint32_t *)0x50500000) = 0x00108008; // Stop SM
    *((volatile uint32_t *)0x50500004) = 0x0000040e; // SRAM control
    *((volatile uint32_t *)0x50500008) = 0x00000006; // Layer count
    *((volatile uint32_t *)0x50900000) = 0x00108008; // Stop SM
    *((volatile uint32_t *)0x50900004) = 0x0000040e; // SRAM control
    *((volatile uint32_t *)0x50900008) = 0x00000006; // Layer count
    *((volatile uint32_t *)0x50d00000) = 0x00108008; // Stop SM
    *((volatile uint32_t *)0x50d00004) = 0x0000040e; // SRAM control
    *((volatile uint32_t *)0x50d00008) = 0x00000006; // Layer count

    return CNN_OK;
}

int cnn_configure(void)
{
    // Layer 0 quadrant 0
    *((volatile uint32_t *)0x50100010) = 0x00010081; // Rows
    *((volatile uint32_t *)0x50100090) = 0x00010081; // Columns
    *((volatile uint32_t *)0x50100310) = 0x00012400; // SRAM write ptr
    *((volatile uint32_t *)0x50100410) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50100590) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x50100a10) = 0x00007800; // Layer control 2
    *((volatile uint32_t *)0x50100610) = 0x00000078; // Mask offset and count
    *((volatile uint32_t *)0x50100690) = 0x0000007f; // TRAM ptr max
    *((volatile uint32_t *)0x50100710) = 0x00070007; // Mask and processor enables
    *((volatile uint32_t *)0x50100810) = 0x00000001; // Stream processing start
    *((volatile uint32_t *)0x50100910) = 0x00000002; // Rollover
    *((volatile uint32_t *)0x50100990) = 0x00004000; // Input frame size

    // Layer 0 quadrant 1
    *((volatile uint32_t *)0x50500010) = 0x00010081; // Rows
    *((volatile uint32_t *)0x50500090) = 0x00010081; // Columns
    *((volatile uint32_t *)0x50500310) = 0x00012400; // SRAM write ptr
    *((volatile uint32_t *)0x50500410) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50500590) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x50500a10) = 0x00007800; // Layer control 2
    *((volatile uint32_t *)0x50500610) = 0x00000078; // Mask offset and count
    *((volatile uint32_t *)0x50500690) = 0x0000007f; // TRAM ptr max
    *((volatile uint32_t *)0x50500810) = 0x00000001; // Stream processing start
    *((volatile uint32_t *)0x50500910) = 0x00000002; // Rollover
    *((volatile uint32_t *)0x50500990) = 0x00004000; // Input frame size

    // Layer 0 quadrant 2
    *((volatile uint32_t *)0x50900010) = 0x00010081; // Rows
    *((volatile uint32_t *)0x50900090) = 0x00010081; // Columns
    *((volatile uint32_t *)0x50900310) = 0x00012400; // SRAM write ptr
    *((volatile uint32_t *)0x50900410) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50900590) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x50900a10) = 0x00007800; // Layer control 2
    *((volatile uint32_t *)0x50900610) = 0x00000078; // Mask offset and count
    *((volatile uint32_t *)0x50900690) = 0x0000007f; // TRAM ptr max
    *((volatile uint32_t *)0x50900810) = 0x00000001; // Stream processing start
    *((volatile uint32_t *)0x50900910) = 0x00000002; // Rollover
    *((volatile uint32_t *)0x50900990) = 0x00004000; // Input frame size

    // Layer 0 quadrant 3
    *((volatile uint32_t *)0x50d00010) = 0x00010081; // Rows
    *((volatile uint32_t *)0x50d00090) = 0x00010081; // Columns
    *((volatile uint32_t *)0x50d00310) = 0x00012400; // SRAM write ptr
    *((volatile uint32_t *)0x50d00410) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50d00590) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x50d00a10) = 0x00007800; // Layer control 2
    *((volatile uint32_t *)0x50d00610) = 0x00000078; // Mask offset and count
    *((volatile uint32_t *)0x50d00690) = 0x0000007f; // TRAM ptr max
    *((volatile uint32_t *)0x50d00810) = 0x00000001; // Stream processing start
    *((volatile uint32_t *)0x50d00910) = 0x00000002; // Rollover
    *((volatile uint32_t *)0x50d00990) = 0x00004000; // Input frame size

    // Layer 1 quadrant 0
    *((volatile uint32_t *)0x50100014) = 0x00010081; // Rows
    *((volatile uint32_t *)0x50100094) = 0x00010081; // Columns
    *((volatile uint32_t *)0x50100194) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50100214) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50100294) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50100314) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x50100414) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50100514) = 0x00000400; // SRAM read ptr
    *((volatile uint32_t *)0x50100594) = 0x0000cba0; // Layer control
    *((volatile uint32_t *)0x50100a14) = 0x0000f800; // Layer control 2
    *((volatile uint32_t *)0x50100614) = 0x000000f8; // Mask offset and count
    *((volatile uint32_t *)0x50100694) = 0x008000bf; // TRAM ptr max
    *((volatile uint32_t *)0x50100814) = 0x0000018c; // Stream processing start
    *((volatile uint32_t *)0x50100894) = 0x00840021; // Stream processing delta
    *((volatile uint32_t *)0x50100914) = 0x0000020e; // Rollover

    // Layer 1 quadrant 1
    *((volatile uint32_t *)0x50500014) = 0x00010081; // Rows
    *((volatile uint32_t *)0x50500094) = 0x00010081; // Columns
    *((volatile uint32_t *)0x50500194) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50500214) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50500294) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50500314) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x50500414) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50500514) = 0x00000400; // SRAM read ptr
    *((volatile uint32_t *)0x50500594) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x50500a14) = 0x0000f800; // Layer control 2
    *((volatile uint32_t *)0x50500614) = 0x000000f8; // Mask offset and count
    *((volatile uint32_t *)0x50500694) = 0x008000bf; // TRAM ptr max
    *((volatile uint32_t *)0x50500814) = 0x0000018c; // Stream processing start
    *((volatile uint32_t *)0x50500894) = 0x00840021; // Stream processing delta
    *((volatile uint32_t *)0x50500914) = 0x0000020e; // Rollover

    // Layer 1 quadrant 2
    *((volatile uint32_t *)0x50900014) = 0x00010081; // Rows
    *((volatile uint32_t *)0x50900094) = 0x00010081; // Columns
    *((volatile uint32_t *)0x50900194) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50900214) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50900294) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50900314) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x50900414) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50900514) = 0x00000400; // SRAM read ptr
    *((volatile uint32_t *)0x50900594) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x50900a14) = 0x0000f800; // Layer control 2
    *((volatile uint32_t *)0x50900614) = 0x000000f8; // Mask offset and count
    *((volatile uint32_t *)0x50900694) = 0x008000bf; // TRAM ptr max
    *((volatile uint32_t *)0x50900714) = 0xfff0fff0; // Mask and processor enables
    *((volatile uint32_t *)0x50900814) = 0x0000018c; // Stream processing start
    *((volatile uint32_t *)0x50900894) = 0x00840021; // Stream processing delta
    *((volatile uint32_t *)0x50900914) = 0x0000020e; // Rollover

    // Layer 1 quadrant 3
    *((volatile uint32_t *)0x50d00014) = 0x00010081; // Rows
    *((volatile uint32_t *)0x50d00094) = 0x00010081; // Columns
    *((volatile uint32_t *)0x50d00194) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50d00214) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50d00294) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50d00314) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x50d00414) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50d00514) = 0x00000400; // SRAM read ptr
    *((volatile uint32_t *)0x50d00594) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x50d00a14) = 0x0000f800; // Layer control 2
    *((volatile uint32_t *)0x50d00614) = 0x000000f8; // Mask offset and count
    *((volatile uint32_t *)0x50d00694) = 0x008000bf; // TRAM ptr max
    *((volatile uint32_t *)0x50d00714) = 0x000f000f; // Mask and processor enables
    *((volatile uint32_t *)0x50d00814) = 0x0000018c; // Stream processing start
    *((volatile uint32_t *)0x50d00894) = 0x00840021; // Stream processing delta
    *((volatile uint32_t *)0x50d00914) = 0x0000020e; // Rollover

    // Layer 2 quadrant 0
    *((volatile uint32_t *)0x50100018) = 0x00010041; // Rows
    *((volatile uint32_t *)0x50100098) = 0x00010041; // Columns
    *((volatile uint32_t *)0x50100198) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50100218) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50100298) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50100418) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50100518) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x50100598) = 0x00002ba0; // Layer control
    *((volatile uint32_t *)0x50100a18) = 0x0001f800; // Layer control 2
    *((volatile uint32_t *)0x50100618) = 0x00800278; // Mask offset and count
    *((volatile uint32_t *)0x50100698) = 0x0000001f; // TRAM ptr max
    *((volatile uint32_t *)0x50100718) = 0xffffffff; // Mask and processor enables

    // Layer 2 quadrant 1
    *((volatile uint32_t *)0x50500018) = 0x00010041; // Rows
    *((volatile uint32_t *)0x50500098) = 0x00010041; // Columns
    *((volatile uint32_t *)0x50500198) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50500218) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50500298) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50500418) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50500518) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x50500598) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x50500a18) = 0x0001f800; // Layer control 2
    *((volatile uint32_t *)0x50500618) = 0x00800278; // Mask offset and count
    *((volatile uint32_t *)0x50500698) = 0x0000001f; // TRAM ptr max
    *((volatile uint32_t *)0x50500718) = 0xffffffff; // Mask and processor enables

    // Layer 2 quadrant 2
    *((volatile uint32_t *)0x50900018) = 0x00010041; // Rows
    *((volatile uint32_t *)0x50900098) = 0x00010041; // Columns
    *((volatile uint32_t *)0x50900198) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50900218) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50900298) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50900418) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50900518) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x50900598) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x50900a18) = 0x0001f800; // Layer control 2
    *((volatile uint32_t *)0x50900618) = 0x00800278; // Mask offset and count
    *((volatile uint32_t *)0x50900698) = 0x0000001f; // TRAM ptr max

    // Layer 2 quadrant 3
    *((volatile uint32_t *)0x50d00018) = 0x00010041; // Rows
    *((volatile uint32_t *)0x50d00098) = 0x00010041; // Columns
    *((volatile uint32_t *)0x50d00198) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50d00218) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50d00298) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50d00418) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50d00518) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x50d00598) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x50d00a18) = 0x0001f800; // Layer control 2
    *((volatile uint32_t *)0x50d00618) = 0x00800278; // Mask offset and count
    *((volatile uint32_t *)0x50d00698) = 0x0000001f; // TRAM ptr max

    // Layer 3 quadrant 0
    *((volatile uint32_t *)0x5010001c) = 0x00010021; // Rows
    *((volatile uint32_t *)0x5010009c) = 0x00010021; // Columns
    *((volatile uint32_t *)0x5010019c) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x5010021c) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x5010029c) = 0x00000001; // Stride
    *((volatile uint32_t *)0x5010031c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x5010041c) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x5010059c) = 0x0000eba0; // Layer control
    *((volatile uint32_t *)0x50100a1c) = 0x0000f800; // Layer control 2
    *((volatile uint32_t *)0x5010061c) = 0x02800378; // Mask offset and count
    *((volatile uint32_t *)0x5010069c) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t *)0x5010071c) = 0xffffffff; // Mask and processor enables

    // Layer 3 quadrant 1
    *((volatile uint32_t *)0x5050001c) = 0x00010021; // Rows
    *((volatile uint32_t *)0x5050009c) = 0x00010021; // Columns
    *((volatile uint32_t *)0x5050019c) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x5050021c) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x5050029c) = 0x00000001; // Stride
    *((volatile uint32_t *)0x5050031c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x5050041c) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x5050059c) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x50500a1c) = 0x0000f800; // Layer control 2
    *((volatile uint32_t *)0x5050061c) = 0x02800378; // Mask offset and count
    *((volatile uint32_t *)0x5050069c) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t *)0x5050071c) = 0xffffffff; // Mask and processor enables

    // Layer 3 quadrant 2
    *((volatile uint32_t *)0x5090001c) = 0x00010021; // Rows
    *((volatile uint32_t *)0x5090009c) = 0x00010021; // Columns
    *((volatile uint32_t *)0x5090019c) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x5090021c) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x5090029c) = 0x00000001; // Stride
    *((volatile uint32_t *)0x5090031c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x5090041c) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x5090059c) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x50900a1c) = 0x0000f800; // Layer control 2
    *((volatile uint32_t *)0x5090061c) = 0x02800378; // Mask offset and count
    *((volatile uint32_t *)0x5090069c) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t *)0x5090071c) = 0xffffffff; // Mask and processor enables

    // Layer 3 quadrant 3
    *((volatile uint32_t *)0x50d0001c) = 0x00010021; // Rows
    *((volatile uint32_t *)0x50d0009c) = 0x00010021; // Columns
    *((volatile uint32_t *)0x50d0019c) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50d0021c) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50d0029c) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50d0031c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x50d0041c) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50d0059c) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x50d00a1c) = 0x0000f800; // Layer control 2
    *((volatile uint32_t *)0x50d0061c) = 0x02800378; // Mask offset and count
    *((volatile uint32_t *)0x50d0069c) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t *)0x50d0071c) = 0xffffffff; // Mask and processor enables

    // Layer 4 quadrant 0
    *((volatile uint32_t *)0x50100020) = 0x00010011; // Rows
    *((volatile uint32_t *)0x501000a0) = 0x00010011; // Columns
    *((volatile uint32_t *)0x501001a0) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50100220) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x501002a0) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50100320) = 0x00010000; // SRAM write ptr
    *((volatile uint32_t *)0x50100420) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50100520) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x501005a0) = 0x00002ba0; // Layer control
    *((volatile uint32_t *)0x50100a20) = 0x0000f800; // Layer control 2
    *((volatile uint32_t *)0x50100620) = 0x03800478; // Mask offset and count
    *((volatile uint32_t *)0x501006a0) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t *)0x50100720) = 0xffffffff; // Mask and processor enables

    // Layer 4 quadrant 1
    *((volatile uint32_t *)0x50500020) = 0x00010011; // Rows
    *((volatile uint32_t *)0x505000a0) = 0x00010011; // Columns
    *((volatile uint32_t *)0x505001a0) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50500220) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x505002a0) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50500320) = 0x00010000; // SRAM write ptr
    *((volatile uint32_t *)0x50500420) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50500520) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x505005a0) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x50500a20) = 0x0000f800; // Layer control 2
    *((volatile uint32_t *)0x50500620) = 0x03800478; // Mask offset and count
    *((volatile uint32_t *)0x505006a0) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t *)0x50500720) = 0xffffffff; // Mask and processor enables

    // Layer 4 quadrant 2
    *((volatile uint32_t *)0x50900020) = 0x00010011; // Rows
    *((volatile uint32_t *)0x509000a0) = 0x00010011; // Columns
    *((volatile uint32_t *)0x509001a0) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50900220) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x509002a0) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50900320) = 0x00010000; // SRAM write ptr
    *((volatile uint32_t *)0x50900420) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50900520) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x509005a0) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x50900a20) = 0x0000f800; // Layer control 2
    *((volatile uint32_t *)0x50900620) = 0x03800478; // Mask offset and count
    *((volatile uint32_t *)0x509006a0) = 0x00000007; // TRAM ptr max

    // Layer 4 quadrant 3
    *((volatile uint32_t *)0x50d00020) = 0x00010011; // Rows
    *((volatile uint32_t *)0x50d000a0) = 0x00010011; // Columns
    *((volatile uint32_t *)0x50d001a0) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x50d00220) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x50d002a0) = 0x00000001; // Stride
    *((volatile uint32_t *)0x50d00320) = 0x00010000; // SRAM write ptr
    *((volatile uint32_t *)0x50d00420) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50d00520) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x50d005a0) = 0x00000ba0; // Layer control
    *((volatile uint32_t *)0x50d00a20) = 0x0000f800; // Layer control 2
    *((volatile uint32_t *)0x50d00620) = 0x03800478; // Mask offset and count
    *((volatile uint32_t *)0x50d006a0) = 0x00000007; // TRAM ptr max

    // Layer 5 quadrant 0
    *((volatile uint32_t *)0x50100024) = 0x00010009; // Rows
    *((volatile uint32_t *)0x501000a4) = 0x00010009; // Columns
    *((volatile uint32_t *)0x50100324) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x50100424) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x501005a4) = 0x0000cb20; // Layer control
    *((volatile uint32_t *)0x50100a24) = 0x00007800; // Layer control 2
    *((volatile uint32_t *)0x50100624) = 0x01000178; // Mask offset and count
    *((volatile uint32_t *)0x501006a4) = 0x00000007; // TRAM ptr max

    // Layer 5 quadrant 1
    *((volatile uint32_t *)0x50500024) = 0x00010009; // Rows
    *((volatile uint32_t *)0x505000a4) = 0x00010009; // Columns
    *((volatile uint32_t *)0x50500324) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x50500424) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x505005a4) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x50500a24) = 0x00007800; // Layer control 2
    *((volatile uint32_t *)0x50500624) = 0x01000178; // Mask offset and count
    *((volatile uint32_t *)0x505006a4) = 0x00000007; // TRAM ptr max

    // Layer 5 quadrant 2
    *((volatile uint32_t *)0x50900024) = 0x00010009; // Rows
    *((volatile uint32_t *)0x509000a4) = 0x00010009; // Columns
    *((volatile uint32_t *)0x50900324) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x50900424) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x509005a4) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x50900a24) = 0x00007800; // Layer control 2
    *((volatile uint32_t *)0x50900624) = 0x01000178; // Mask offset and count
    *((volatile uint32_t *)0x509006a4) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t *)0x50900724) = 0xffffffff; // Mask and processor enables

    // Layer 5 quadrant 3
    *((volatile uint32_t *)0x50d00024) = 0x00010009; // Rows
    *((volatile uint32_t *)0x50d000a4) = 0x00010009; // Columns
    *((volatile uint32_t *)0x50d00324) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t *)0x50d00424) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50d005a4) = 0x00000b20; // Layer control
    *((volatile uint32_t *)0x50d00a24) = 0x00007800; // Layer control 2
    *((volatile uint32_t *)0x50d00624) = 0x01000178; // Mask offset and count
    *((volatile uint32_t *)0x50d006a4) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t *)0x50d00724) = 0xffffffff; // Mask and processor enables

    // Layer 6 quadrant 0
    *((volatile uint32_t *)0x50100328) = 0x00000400; // SRAM write ptr
    *((volatile uint32_t *)0x501003a8) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x50100428) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50100528) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x501005a8) = 0x00010920; // Layer control
    *((volatile uint32_t *)0x50100a28) = 0x0000080f; // Layer control 2
    *((volatile uint32_t *)0x50100628) = 0x28802c78; // Mask offset and count
    *((volatile uint32_t *)0x50100128) = 0x00000100; // 1D
    *((volatile uint32_t *)0x501006a8) = 0x003f007f; // TRAM ptr max
    *((volatile uint32_t *)0x501007a8) = 0x080c3000; // Post processing register
    *((volatile uint32_t *)0x50100728) = 0xffffffff; // Mask and processor enables

    // Layer 6 quadrant 1
    *((volatile uint32_t *)0x50500328) = 0x00000400; // SRAM write ptr
    *((volatile uint32_t *)0x505003a8) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x50500428) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50500528) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x505005a8) = 0x00010920; // Layer control
    *((volatile uint32_t *)0x50500a28) = 0x0000080f; // Layer control 2
    *((volatile uint32_t *)0x50500628) = 0x28802c78; // Mask offset and count
    *((volatile uint32_t *)0x50500128) = 0x00000100; // 1D
    *((volatile uint32_t *)0x505006a8) = 0x003f007f; // TRAM ptr max
    *((volatile uint32_t *)0x505007a8) = 0x080c2000; // Post processing register

    // Layer 6 quadrant 2
    *((volatile uint32_t *)0x50900328) = 0x00000400; // SRAM write ptr
    *((volatile uint32_t *)0x509003a8) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x50900428) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50900528) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x509005a8) = 0x00010920; // Layer control
    *((volatile uint32_t *)0x50900a28) = 0x0000080f; // Layer control 2
    *((volatile uint32_t *)0x50900628) = 0x28802c78; // Mask offset and count
    *((volatile uint32_t *)0x50900128) = 0x00000100; // 1D
    *((volatile uint32_t *)0x509006a8) = 0x003f007f; // TRAM ptr max
    *((volatile uint32_t *)0x509007a8) = 0x080c2000; // Post processing register

    // Layer 6 quadrant 3
    *((volatile uint32_t *)0x50d00328) = 0x00000400; // SRAM write ptr
    *((volatile uint32_t *)0x50d003a8) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t *)0x50d00428) = 0x00002000; // Write ptr mask offs
    *((volatile uint32_t *)0x50d00528) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t *)0x50d005a8) = 0x00010920; // Layer control
    *((volatile uint32_t *)0x50d00a28) = 0x0000080f; // Layer control 2
    *((volatile uint32_t *)0x50d00628) = 0x28802c78; // Mask offset and count
    *((volatile uint32_t *)0x50d00128) = 0x00000100; // 1D
    *((volatile uint32_t *)0x50d006a8) = 0x003f007f; // TRAM ptr max
    *((volatile uint32_t *)0x50d007a8) = 0x080c2000; // Post processing register

    *((volatile uint32_t *)0x50000000) = 0x00001908; // FIFO control

    return CNN_OK;
}

int cnn_start(void)
{
    cnn_time = 0;

    *((volatile uint32_t *)0x50100000) = 0x0018c808; // Enable quadrant 0
    *((volatile uint32_t *)0x50500000) = 0x0018c809; // Enable quadrant 1
    *((volatile uint32_t *)0x50900000) = 0x0018c809; // Enable quadrant 2
    *((volatile uint32_t *)0x50d00000) = 0x0018c809; // Enable quadrant 3

#ifdef CNN_INFERENCE_TIMER
    MXC_TMR_SW_Start(CNN_INFERENCE_TIMER);
#endif

    CNN_START; // Allow capture of processing time
    *((volatile uint32_t *)0x50100000) = 0x0018c809; // Master enable quadrant 0

    return CNN_OK;
}

int cnn_unload(uint32_t *out_buf)
{
    volatile uint32_t *addr;

    // Custom unload for this network, layer 6: 32-bit data, shape: (2, 1, 1)
    addr = (volatile uint32_t *)0x50401000;
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
