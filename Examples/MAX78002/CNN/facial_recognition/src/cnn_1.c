/*******************************************************************************
* Copyright (C) 2019-2023 Maxim Integrated Products, Inc., All rights Reserved.
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

// facedetection-ai87-layer74
// Created using ai8xize.py --test-dir sdk/Examples/MAX78002/CNN --prefix facedetection-ai87-layer74 --checkpoint-file trained/ai87-facedetection3-qat8-q.pth.tar --config-file networks/ai87-face-tinierssd-layer74.yaml --sample-input tests/sample_facedetection.npy --overwrite --start-layer 74 --weight-start 2500 --device MAX78002 --timer 0 --display-checkpoint --verbose

// DO NOT EDIT - regenerate this file instead!

// Configuring 90 layers
// Input data: CHW
// Layer 74: 3x224x168, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 16x112x84 output
// Layer 75: 16x112x84, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 32x56x42 output
// Layer 76: 32x56x42, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 64x56x42 output
// Layer 77: 64x56x42, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 64x56x42 output
// Layer 78: 64x56x42, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 64x56x42 output
// Layer 79: 64x56x42, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 64x56x42 output
// Layer 80: 64x56x42, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 128x56x42 output
// Layer 81: 128x56x42, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 32x56x42 output
// Layer 82: 32x56x42, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 32x28x21 output
// Layer 83: 32x28x21, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 32x14x10 output
// Layer 84: 32x14x10, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 16x14x10 output
// Layer 85: 16x14x10, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 16x7x5 output
// Layer 86: 32x28x21, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, no activation, 8x28x21 output
// Layer 87: 16x7x5, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, no activation, 8x7x5 output
// Layer 88: 32x28x21, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, no activation, 4x28x21 output
// Layer 89: 16x7x5, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, no activation, 4x7x5 output

#include <assert.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "mxc.h"
#include "gcfr_regs.h"
#include "cnn_1.h"
#include "weights_1.h"

void CNN_1_ISR(void)
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

static const uint32_t kernels_1[] = KERNELS_1;

int cnn_1_load_weights(void)
{
    uint32_t len;
    volatile uint32_t *addr;
    const uint32_t *ptr = kernels_1;

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

int cnn_1_load_bias(void)
{
    memcpy_8to32((uint32_t *)0x51180000, bias_0, sizeof(uint8_t) * 136);
    memcpy_8to32((uint32_t *)0x52180000, bias_1, sizeof(uint8_t) * 136);
    memcpy_8to32((uint32_t *)0x53180000, bias_2, sizeof(uint8_t) * 132);
    memcpy_8to32((uint32_t *)0x54180000, bias_3, sizeof(uint8_t) * 132);

    return CNN_OK;
}

int cnn_1_init(void)
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
    *((volatile uint32_t *)0x51000008) = 0x00004a59; // Layer count
    *((volatile uint32_t *)0x52000000) = 0x00100008; // Stop SM
    *((volatile uint32_t *)0x52000008) = 0x00004a59; // Layer count
    *((volatile uint32_t *)0x53000000) = 0x00100008; // Stop SM
    *((volatile uint32_t *)0x53000008) = 0x00004a59; // Layer count
    *((volatile uint32_t *)0x54000000) = 0x00100008; // Stop SM
    *((volatile uint32_t *)0x54000008) = 0x00004a59; // Layer count

    return CNN_OK;
}

int cnn_1_configure(void)
{
    // Layer 74 quadrant 0
    *((volatile uint32_t *)0x51104a04) = 0x00aa80de; // Rows
    *((volatile uint32_t *)0x51104a08) = 0x000280a6; // Columns
    *((volatile uint32_t *)0x51104a10) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x51104a14) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x51104a18) = 0x00000021; // Stride
    *((volatile uint32_t *)0x51104a1c) = 0x000224c0; // SRAM write ptr
    *((volatile uint32_t *)0x51104a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51104a30) = 0x00886be0; // Layer control
    *((volatile uint32_t *)0x51104a34) = 0x00078000; // Layer control 2
    *((volatile uint32_t *)0x51104a38) = 0x00009ff8; // Mask count
    *((volatile uint32_t *)0x51104a3c) = 0x00009f80; // Mask offset
    *((volatile uint32_t *)0x51104a40) = 0x0000000f; // Output channel count
    *((volatile uint32_t *)0x51104a44) = 0x00000053; // TRAM ptr max
    *((volatile uint32_t *)0x51104a48) = 0x00010001; // Mask and processor enables

    // Layer 74 quadrant 1
    *((volatile uint32_t *)0x52104a04) = 0x00aa80de; // Rows
    *((volatile uint32_t *)0x52104a08) = 0x000280a6; // Columns
    *((volatile uint32_t *)0x52104a10) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x52104a14) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x52104a18) = 0x00000021; // Stride
    *((volatile uint32_t *)0x52104a1c) = 0x000224c0; // SRAM write ptr
    *((volatile uint32_t *)0x52104a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52104a30) = 0x00880be0; // Layer control
    *((volatile uint32_t *)0x52104a34) = 0x00078000; // Layer control 2
    *((volatile uint32_t *)0x52104a38) = 0x00009ff8; // Mask count
    *((volatile uint32_t *)0x52104a3c) = 0x00009f80; // Mask offset
    *((volatile uint32_t *)0x52104a40) = 0x0000000f; // Output channel count
    *((volatile uint32_t *)0x52104a44) = 0x00000053; // TRAM ptr max
    *((volatile uint32_t *)0x52104a48) = 0x00010001; // Mask and processor enables

    // Layer 74 quadrant 2
    *((volatile uint32_t *)0x53104a04) = 0x00aa80de; // Rows
    *((volatile uint32_t *)0x53104a08) = 0x000280a6; // Columns
    *((volatile uint32_t *)0x53104a10) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x53104a14) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x53104a18) = 0x00000021; // Stride
    *((volatile uint32_t *)0x53104a1c) = 0x000224c0; // SRAM write ptr
    *((volatile uint32_t *)0x53104a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53104a30) = 0x00880be0; // Layer control
    *((volatile uint32_t *)0x53104a34) = 0x00078000; // Layer control 2
    *((volatile uint32_t *)0x53104a38) = 0x00009ff8; // Mask count
    *((volatile uint32_t *)0x53104a3c) = 0x00009f80; // Mask offset
    *((volatile uint32_t *)0x53104a40) = 0x0000000f; // Output channel count
    *((volatile uint32_t *)0x53104a44) = 0x00000053; // TRAM ptr max
    *((volatile uint32_t *)0x53104a48) = 0x00010001; // Mask and processor enables

    // Layer 74 quadrant 3
    *((volatile uint32_t *)0x54104a04) = 0x00aa80de; // Rows
    *((volatile uint32_t *)0x54104a08) = 0x000280a6; // Columns
    *((volatile uint32_t *)0x54104a10) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x54104a14) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x54104a18) = 0x00000021; // Stride
    *((volatile uint32_t *)0x54104a1c) = 0x000224c0; // SRAM write ptr
    *((volatile uint32_t *)0x54104a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54104a30) = 0x00880be0; // Layer control
    *((volatile uint32_t *)0x54104a34) = 0x00078000; // Layer control 2
    *((volatile uint32_t *)0x54104a38) = 0x00009ff8; // Mask count
    *((volatile uint32_t *)0x54104a3c) = 0x00009f80; // Mask offset
    *((volatile uint32_t *)0x54104a40) = 0x0000000f; // Output channel count
    *((volatile uint32_t *)0x54104a44) = 0x00000053; // TRAM ptr max

    // Layer 75 quadrant 0
    *((volatile uint32_t *)0x51104b04) = 0x0056806e; // Rows
    *((volatile uint32_t *)0x51104b08) = 0x00028052; // Columns
    *((volatile uint32_t *)0x51104b10) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x51104b14) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x51104b18) = 0x00000021; // Stride
    *((volatile uint32_t *)0x51104b1c) = 0x00040000; // SRAM write ptr
    *((volatile uint32_t *)0x51104b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51104b2c) = 0x000024c0; // SRAM read ptr
    *((volatile uint32_t *)0x51104b30) = 0x00882ba0; // Layer control
    *((volatile uint32_t *)0x51104b34) = 0x000f8000; // Layer control 2
    *((volatile uint32_t *)0x51104b38) = 0x00004f18; // Mask count
    *((volatile uint32_t *)0x51104b3c) = 0x00004e20; // Mask offset
    *((volatile uint32_t *)0x51104b40) = 0x0000001f; // Output channel count
    *((volatile uint32_t *)0x51104b44) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x51104b4c) = 0x00022000; // Post processing register

    // Layer 75 quadrant 1
    *((volatile uint32_t *)0x52104b04) = 0x0056806e; // Rows
    *((volatile uint32_t *)0x52104b08) = 0x00028052; // Columns
    *((volatile uint32_t *)0x52104b10) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x52104b14) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x52104b18) = 0x00000021; // Stride
    *((volatile uint32_t *)0x52104b1c) = 0x00040000; // SRAM write ptr
    *((volatile uint32_t *)0x52104b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52104b2c) = 0x000024c0; // SRAM read ptr
    *((volatile uint32_t *)0x52104b30) = 0x00880ba0; // Layer control
    *((volatile uint32_t *)0x52104b34) = 0x000f8000; // Layer control 2
    *((volatile uint32_t *)0x52104b38) = 0x00004f18; // Mask count
    *((volatile uint32_t *)0x52104b3c) = 0x00004e20; // Mask offset
    *((volatile uint32_t *)0x52104b40) = 0x0000001f; // Output channel count
    *((volatile uint32_t *)0x52104b44) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x52104b4c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x52104b48) = 0xffffffff; // Mask and processor enables

    // Layer 75 quadrant 2
    *((volatile uint32_t *)0x53104b04) = 0x0056806e; // Rows
    *((volatile uint32_t *)0x53104b08) = 0x00028052; // Columns
    *((volatile uint32_t *)0x53104b10) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x53104b14) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x53104b18) = 0x00000021; // Stride
    *((volatile uint32_t *)0x53104b1c) = 0x00040000; // SRAM write ptr
    *((volatile uint32_t *)0x53104b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53104b2c) = 0x000024c0; // SRAM read ptr
    *((volatile uint32_t *)0x53104b30) = 0x00880ba0; // Layer control
    *((volatile uint32_t *)0x53104b34) = 0x000f8000; // Layer control 2
    *((volatile uint32_t *)0x53104b38) = 0x00004f18; // Mask count
    *((volatile uint32_t *)0x53104b3c) = 0x00004e20; // Mask offset
    *((volatile uint32_t *)0x53104b40) = 0x0000001f; // Output channel count
    *((volatile uint32_t *)0x53104b44) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x53104b4c) = 0x00022000; // Post processing register

    // Layer 75 quadrant 3
    *((volatile uint32_t *)0x54104b04) = 0x0056806e; // Rows
    *((volatile uint32_t *)0x54104b08) = 0x00028052; // Columns
    *((volatile uint32_t *)0x54104b10) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x54104b14) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x54104b18) = 0x00000021; // Stride
    *((volatile uint32_t *)0x54104b1c) = 0x00040000; // SRAM write ptr
    *((volatile uint32_t *)0x54104b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54104b2c) = 0x000024c0; // SRAM read ptr
    *((volatile uint32_t *)0x54104b30) = 0x00880ba0; // Layer control
    *((volatile uint32_t *)0x54104b34) = 0x000f8000; // Layer control 2
    *((volatile uint32_t *)0x54104b38) = 0x00004f18; // Mask count
    *((volatile uint32_t *)0x54104b3c) = 0x00004e20; // Mask offset
    *((volatile uint32_t *)0x54104b40) = 0x0000001f; // Output channel count
    *((volatile uint32_t *)0x54104b44) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x54104b4c) = 0x00022000; // Post processing register

    // Layer 76 quadrant 0
    *((volatile uint32_t *)0x51104c04) = 0x00018037; // Rows
    *((volatile uint32_t *)0x51104c08) = 0x00018029; // Columns
    *((volatile uint32_t *)0x51104c18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x51104c1c) = 0x00000c00; // SRAM write ptr
    *((volatile uint32_t *)0x51104c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51104c30) = 0x0088eb20; // Layer control
    *((volatile uint32_t *)0x51104c34) = 0x001f8000; // Layer control 2
    *((volatile uint32_t *)0x51104c38) = 0x00005018; // Mask count
    *((volatile uint32_t *)0x51104c3c) = 0x00004e20; // Mask offset
    *((volatile uint32_t *)0x51104c40) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x51104c44) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x51104c4c) = 0x00024000; // Post processing register

    // Layer 76 quadrant 1
    *((volatile uint32_t *)0x52104c04) = 0x00018037; // Rows
    *((volatile uint32_t *)0x52104c08) = 0x00018029; // Columns
    *((volatile uint32_t *)0x52104c18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x52104c1c) = 0x00000c00; // SRAM write ptr
    *((volatile uint32_t *)0x52104c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52104c30) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x52104c34) = 0x001f8000; // Layer control 2
    *((volatile uint32_t *)0x52104c38) = 0x00005018; // Mask count
    *((volatile uint32_t *)0x52104c3c) = 0x00004e20; // Mask offset
    *((volatile uint32_t *)0x52104c40) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x52104c44) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x52104c4c) = 0x00025000; // Post processing register

    // Layer 76 quadrant 2
    *((volatile uint32_t *)0x53104c04) = 0x00018037; // Rows
    *((volatile uint32_t *)0x53104c08) = 0x00018029; // Columns
    *((volatile uint32_t *)0x53104c18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x53104c1c) = 0x00000c00; // SRAM write ptr
    *((volatile uint32_t *)0x53104c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53104c30) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x53104c34) = 0x001f8000; // Layer control 2
    *((volatile uint32_t *)0x53104c38) = 0x00005018; // Mask count
    *((volatile uint32_t *)0x53104c3c) = 0x00004e20; // Mask offset
    *((volatile uint32_t *)0x53104c40) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x53104c44) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x53104c4c) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x53104c48) = 0xffffffff; // Mask and processor enables

    // Layer 76 quadrant 3
    *((volatile uint32_t *)0x54104c04) = 0x00018037; // Rows
    *((volatile uint32_t *)0x54104c08) = 0x00018029; // Columns
    *((volatile uint32_t *)0x54104c18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x54104c1c) = 0x00000c00; // SRAM write ptr
    *((volatile uint32_t *)0x54104c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54104c30) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x54104c34) = 0x001f8000; // Layer control 2
    *((volatile uint32_t *)0x54104c38) = 0x00005018; // Mask count
    *((volatile uint32_t *)0x54104c3c) = 0x00004e20; // Mask offset
    *((volatile uint32_t *)0x54104c40) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x54104c44) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x54104c4c) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x54104c48) = 0xffffffff; // Mask and processor enables

    // Layer 77 quadrant 0
    *((volatile uint32_t *)0x51104d04) = 0x00018037; // Rows
    *((volatile uint32_t *)0x51104d08) = 0x00018029; // Columns
    *((volatile uint32_t *)0x51104d18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x51104d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51104d2c) = 0x00000c00; // SRAM read ptr
    *((volatile uint32_t *)0x51104d30) = 0x0088eb20; // Layer control
    *((volatile uint32_t *)0x51104d34) = 0x001f8000; // Layer control 2
    *((volatile uint32_t *)0x51104d38) = 0x00005218; // Mask count
    *((volatile uint32_t *)0x51104d3c) = 0x00005020; // Mask offset
    *((volatile uint32_t *)0x51104d40) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x51104d44) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x51104d4c) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x51104d48) = 0xffffffff; // Mask and processor enables

    // Layer 77 quadrant 1
    *((volatile uint32_t *)0x52104d04) = 0x00018037; // Rows
    *((volatile uint32_t *)0x52104d08) = 0x00018029; // Columns
    *((volatile uint32_t *)0x52104d18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x52104d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52104d2c) = 0x00000c00; // SRAM read ptr
    *((volatile uint32_t *)0x52104d30) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x52104d34) = 0x001f8000; // Layer control 2
    *((volatile uint32_t *)0x52104d38) = 0x00005218; // Mask count
    *((volatile uint32_t *)0x52104d3c) = 0x00005020; // Mask offset
    *((volatile uint32_t *)0x52104d40) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x52104d44) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x52104d4c) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x52104d48) = 0xffffffff; // Mask and processor enables

    // Layer 77 quadrant 2
    *((volatile uint32_t *)0x53104d04) = 0x00018037; // Rows
    *((volatile uint32_t *)0x53104d08) = 0x00018029; // Columns
    *((volatile uint32_t *)0x53104d18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x53104d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53104d2c) = 0x00000c00; // SRAM read ptr
    *((volatile uint32_t *)0x53104d30) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x53104d34) = 0x001f8000; // Layer control 2
    *((volatile uint32_t *)0x53104d38) = 0x00005218; // Mask count
    *((volatile uint32_t *)0x53104d3c) = 0x00005020; // Mask offset
    *((volatile uint32_t *)0x53104d40) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x53104d44) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x53104d4c) = 0x00025000; // Post processing register
    *((volatile uint32_t *)0x53104d48) = 0xffffffff; // Mask and processor enables

    // Layer 77 quadrant 3
    *((volatile uint32_t *)0x54104d04) = 0x00018037; // Rows
    *((volatile uint32_t *)0x54104d08) = 0x00018029; // Columns
    *((volatile uint32_t *)0x54104d18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x54104d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54104d2c) = 0x00000c00; // SRAM read ptr
    *((volatile uint32_t *)0x54104d30) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x54104d34) = 0x001f8000; // Layer control 2
    *((volatile uint32_t *)0x54104d38) = 0x00005218; // Mask count
    *((volatile uint32_t *)0x54104d3c) = 0x00005020; // Mask offset
    *((volatile uint32_t *)0x54104d40) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x54104d44) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x54104d4c) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x54104d48) = 0xffffffff; // Mask and processor enables

    // Layer 78 quadrant 0
    *((volatile uint32_t *)0x51104e04) = 0x00018037; // Rows
    *((volatile uint32_t *)0x51104e08) = 0x00018029; // Columns
    *((volatile uint32_t *)0x51104e18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x51104e1c) = 0x00000c00; // SRAM write ptr
    *((volatile uint32_t *)0x51104e24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51104e30) = 0x0088eb20; // Layer control
    *((volatile uint32_t *)0x51104e34) = 0x001f8000; // Layer control 2
    *((volatile uint32_t *)0x51104e38) = 0x00005418; // Mask count
    *((volatile uint32_t *)0x51104e3c) = 0x00005220; // Mask offset
    *((volatile uint32_t *)0x51104e40) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x51104e44) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x51104e4c) = 0x00026000; // Post processing register
    *((volatile uint32_t *)0x51104e48) = 0xffffffff; // Mask and processor enables

    // Layer 78 quadrant 1
    *((volatile uint32_t *)0x52104e04) = 0x00018037; // Rows
    *((volatile uint32_t *)0x52104e08) = 0x00018029; // Columns
    *((volatile uint32_t *)0x52104e18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x52104e1c) = 0x00000c00; // SRAM write ptr
    *((volatile uint32_t *)0x52104e24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52104e30) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x52104e34) = 0x001f8000; // Layer control 2
    *((volatile uint32_t *)0x52104e38) = 0x00005418; // Mask count
    *((volatile uint32_t *)0x52104e3c) = 0x00005220; // Mask offset
    *((volatile uint32_t *)0x52104e40) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x52104e44) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x52104e4c) = 0x00026000; // Post processing register
    *((volatile uint32_t *)0x52104e48) = 0xffffffff; // Mask and processor enables

    // Layer 78 quadrant 2
    *((volatile uint32_t *)0x53104e04) = 0x00018037; // Rows
    *((volatile uint32_t *)0x53104e08) = 0x00018029; // Columns
    *((volatile uint32_t *)0x53104e18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x53104e1c) = 0x00000c00; // SRAM write ptr
    *((volatile uint32_t *)0x53104e24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53104e30) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x53104e34) = 0x001f8000; // Layer control 2
    *((volatile uint32_t *)0x53104e38) = 0x00005418; // Mask count
    *((volatile uint32_t *)0x53104e3c) = 0x00005220; // Mask offset
    *((volatile uint32_t *)0x53104e40) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x53104e44) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x53104e4c) = 0x00026000; // Post processing register
    *((volatile uint32_t *)0x53104e48) = 0xffffffff; // Mask and processor enables

    // Layer 78 quadrant 3
    *((volatile uint32_t *)0x54104e04) = 0x00018037; // Rows
    *((volatile uint32_t *)0x54104e08) = 0x00018029; // Columns
    *((volatile uint32_t *)0x54104e18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x54104e1c) = 0x00000c00; // SRAM write ptr
    *((volatile uint32_t *)0x54104e24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54104e30) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x54104e34) = 0x001f8000; // Layer control 2
    *((volatile uint32_t *)0x54104e38) = 0x00005418; // Mask count
    *((volatile uint32_t *)0x54104e3c) = 0x00005220; // Mask offset
    *((volatile uint32_t *)0x54104e40) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x54104e44) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x54104e4c) = 0x00027000; // Post processing register
    *((volatile uint32_t *)0x54104e48) = 0xffffffff; // Mask and processor enables

    // Layer 79 quadrant 0
    *((volatile uint32_t *)0x51104f04) = 0x00018037; // Rows
    *((volatile uint32_t *)0x51104f08) = 0x00018029; // Columns
    *((volatile uint32_t *)0x51104f18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x51104f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51104f2c) = 0x00000c00; // SRAM read ptr
    *((volatile uint32_t *)0x51104f30) = 0x0088eb20; // Layer control
    *((volatile uint32_t *)0x51104f34) = 0x001f8000; // Layer control 2
    *((volatile uint32_t *)0x51104f38) = 0x00005618; // Mask count
    *((volatile uint32_t *)0x51104f3c) = 0x00005420; // Mask offset
    *((volatile uint32_t *)0x51104f40) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x51104f44) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x51104f4c) = 0x00026000; // Post processing register
    *((volatile uint32_t *)0x51104f48) = 0xffffffff; // Mask and processor enables

    // Layer 79 quadrant 1
    *((volatile uint32_t *)0x52104f04) = 0x00018037; // Rows
    *((volatile uint32_t *)0x52104f08) = 0x00018029; // Columns
    *((volatile uint32_t *)0x52104f18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x52104f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52104f2c) = 0x00000c00; // SRAM read ptr
    *((volatile uint32_t *)0x52104f30) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x52104f34) = 0x001f8000; // Layer control 2
    *((volatile uint32_t *)0x52104f38) = 0x00005618; // Mask count
    *((volatile uint32_t *)0x52104f3c) = 0x00005420; // Mask offset
    *((volatile uint32_t *)0x52104f40) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x52104f44) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x52104f4c) = 0x00027040; // Post processing register
    *((volatile uint32_t *)0x52104f48) = 0xffffffff; // Mask and processor enables

    // Layer 79 quadrant 2
    *((volatile uint32_t *)0x53104f04) = 0x00018037; // Rows
    *((volatile uint32_t *)0x53104f08) = 0x00018029; // Columns
    *((volatile uint32_t *)0x53104f18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x53104f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53104f2c) = 0x00000c00; // SRAM read ptr
    *((volatile uint32_t *)0x53104f30) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x53104f34) = 0x001f8000; // Layer control 2
    *((volatile uint32_t *)0x53104f38) = 0x00005618; // Mask count
    *((volatile uint32_t *)0x53104f3c) = 0x00005420; // Mask offset
    *((volatile uint32_t *)0x53104f40) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x53104f44) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x53104f4c) = 0x00026000; // Post processing register
    *((volatile uint32_t *)0x53104f48) = 0xffffffff; // Mask and processor enables

    // Layer 79 quadrant 3
    *((volatile uint32_t *)0x54104f04) = 0x00018037; // Rows
    *((volatile uint32_t *)0x54104f08) = 0x00018029; // Columns
    *((volatile uint32_t *)0x54104f18) = 0x00000010; // Stride
    *((volatile uint32_t *)0x54104f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54104f2c) = 0x00000c00; // SRAM read ptr
    *((volatile uint32_t *)0x54104f30) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x54104f34) = 0x001f8000; // Layer control 2
    *((volatile uint32_t *)0x54104f38) = 0x00005618; // Mask count
    *((volatile uint32_t *)0x54104f3c) = 0x00005420; // Mask offset
    *((volatile uint32_t *)0x54104f40) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x54104f44) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x54104f4c) = 0x00026000; // Post processing register
    *((volatile uint32_t *)0x54104f48) = 0xffffffff; // Mask and processor enables

    // Layer 80 quadrant 0
    *((volatile uint32_t *)0x51105004) = 0x00018037; // Rows
    *((volatile uint32_t *)0x51105008) = 0x00018029; // Columns
    *((volatile uint32_t *)0x51105018) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5110501c) = 0x00000c00; // SRAM write ptr
    *((volatile uint32_t *)0x51105024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51105028) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x51105030) = 0x0088eb20; // Layer control
    *((volatile uint32_t *)0x51105034) = 0x001f8010; // Layer control 2
    *((volatile uint32_t *)0x51105038) = 0x00005a18; // Mask count
    *((volatile uint32_t *)0x5110503c) = 0x00005620; // Mask offset
    *((volatile uint32_t *)0x51105040) = 0x0000007f; // Output channel count
    *((volatile uint32_t *)0x51105044) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x5110504c) = 0x00027000; // Post processing register
    *((volatile uint32_t *)0x51105048) = 0xffffffff; // Mask and processor enables

    // Layer 80 quadrant 1
    *((volatile uint32_t *)0x52105004) = 0x00018037; // Rows
    *((volatile uint32_t *)0x52105008) = 0x00018029; // Columns
    *((volatile uint32_t *)0x52105018) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5210501c) = 0x00000c00; // SRAM write ptr
    *((volatile uint32_t *)0x52105024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52105028) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x52105030) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x52105034) = 0x001f8010; // Layer control 2
    *((volatile uint32_t *)0x52105038) = 0x00005a18; // Mask count
    *((volatile uint32_t *)0x5210503c) = 0x00005620; // Mask offset
    *((volatile uint32_t *)0x52105040) = 0x0000007f; // Output channel count
    *((volatile uint32_t *)0x52105044) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x5210504c) = 0x00026000; // Post processing register
    *((volatile uint32_t *)0x52105048) = 0xffffffff; // Mask and processor enables

    // Layer 80 quadrant 2
    *((volatile uint32_t *)0x53105004) = 0x00018037; // Rows
    *((volatile uint32_t *)0x53105008) = 0x00018029; // Columns
    *((volatile uint32_t *)0x53105018) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5310501c) = 0x00000c00; // SRAM write ptr
    *((volatile uint32_t *)0x53105024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53105028) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x53105030) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x53105034) = 0x001f8010; // Layer control 2
    *((volatile uint32_t *)0x53105038) = 0x00005a18; // Mask count
    *((volatile uint32_t *)0x5310503c) = 0x00005620; // Mask offset
    *((volatile uint32_t *)0x53105040) = 0x0000007f; // Output channel count
    *((volatile uint32_t *)0x53105044) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x5310504c) = 0x00026000; // Post processing register
    *((volatile uint32_t *)0x53105048) = 0xffffffff; // Mask and processor enables

    // Layer 80 quadrant 3
    *((volatile uint32_t *)0x54105004) = 0x00018037; // Rows
    *((volatile uint32_t *)0x54105008) = 0x00018029; // Columns
    *((volatile uint32_t *)0x54105018) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5410501c) = 0x00000c00; // SRAM write ptr
    *((volatile uint32_t *)0x54105024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54105028) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t *)0x54105030) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x54105034) = 0x001f8010; // Layer control 2
    *((volatile uint32_t *)0x54105038) = 0x00005a18; // Mask count
    *((volatile uint32_t *)0x5410503c) = 0x00005620; // Mask offset
    *((volatile uint32_t *)0x54105040) = 0x0000007f; // Output channel count
    *((volatile uint32_t *)0x54105044) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x5410504c) = 0x00026000; // Post processing register
    *((volatile uint32_t *)0x54105048) = 0xffffffff; // Mask and processor enables

    // Layer 81 quadrant 0
    *((volatile uint32_t *)0x51105104) = 0x00028037; // Rows
    *((volatile uint32_t *)0x51105108) = 0x00018029; // Columns
    *((volatile uint32_t *)0x51105118) = 0x00000020; // Stride
    *((volatile uint32_t *)0x51105124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5110512c) = 0x00000c00; // SRAM read ptr
    *((volatile uint32_t *)0x51105130) = 0x0088eb20; // Layer control
    *((volatile uint32_t *)0x51105134) = 0x000f8001; // Layer control 2
    *((volatile uint32_t *)0x51105138) = 0x00005c18; // Mask count
    *((volatile uint32_t *)0x5110513c) = 0x00005a20; // Mask offset
    *((volatile uint32_t *)0x51105140) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x51105144) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x5110514c) = 0x00026000; // Post processing register
    *((volatile uint32_t *)0x51105148) = 0xffffffff; // Mask and processor enables

    // Layer 81 quadrant 1
    *((volatile uint32_t *)0x52105104) = 0x00028037; // Rows
    *((volatile uint32_t *)0x52105108) = 0x00018029; // Columns
    *((volatile uint32_t *)0x52105118) = 0x00000020; // Stride
    *((volatile uint32_t *)0x52105124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5210512c) = 0x00000c00; // SRAM read ptr
    *((volatile uint32_t *)0x52105130) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x52105134) = 0x000f8001; // Layer control 2
    *((volatile uint32_t *)0x52105138) = 0x00005c18; // Mask count
    *((volatile uint32_t *)0x5210513c) = 0x00005a20; // Mask offset
    *((volatile uint32_t *)0x52105140) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x52105144) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x5210514c) = 0x00026000; // Post processing register
    *((volatile uint32_t *)0x52105148) = 0xffffffff; // Mask and processor enables

    // Layer 81 quadrant 2
    *((volatile uint32_t *)0x53105104) = 0x00028037; // Rows
    *((volatile uint32_t *)0x53105108) = 0x00018029; // Columns
    *((volatile uint32_t *)0x53105118) = 0x00000020; // Stride
    *((volatile uint32_t *)0x53105124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5310512c) = 0x00000c00; // SRAM read ptr
    *((volatile uint32_t *)0x53105130) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x53105134) = 0x000f8001; // Layer control 2
    *((volatile uint32_t *)0x53105138) = 0x00005c18; // Mask count
    *((volatile uint32_t *)0x5310513c) = 0x00005a20; // Mask offset
    *((volatile uint32_t *)0x53105140) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x53105144) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x5310514c) = 0x00027040; // Post processing register
    *((volatile uint32_t *)0x53105148) = 0xffffffff; // Mask and processor enables

    // Layer 81 quadrant 3
    *((volatile uint32_t *)0x54105104) = 0x00028037; // Rows
    *((volatile uint32_t *)0x54105108) = 0x00018029; // Columns
    *((volatile uint32_t *)0x54105118) = 0x00000020; // Stride
    *((volatile uint32_t *)0x54105124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5410512c) = 0x00000c00; // SRAM read ptr
    *((volatile uint32_t *)0x54105130) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x54105134) = 0x000f8001; // Layer control 2
    *((volatile uint32_t *)0x54105138) = 0x00005c18; // Mask count
    *((volatile uint32_t *)0x5410513c) = 0x00005a20; // Mask offset
    *((volatile uint32_t *)0x54105140) = 0x0000003f; // Output channel count
    *((volatile uint32_t *)0x54105144) = 0x00000029; // TRAM ptr max
    *((volatile uint32_t *)0x5410514c) = 0x00026000; // Post processing register
    *((volatile uint32_t *)0x54105148) = 0xffffffff; // Mask and processor enables

    // Layer 82 quadrant 0
    *((volatile uint32_t *)0x51105204) = 0x002c8036; // Rows
    *((volatile uint32_t *)0x51105208) = 0x00028028; // Columns
    *((volatile uint32_t *)0x51105210) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x51105214) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x51105218) = 0x00000021; // Stride
    *((volatile uint32_t *)0x5110521c) = 0x00040c00; // SRAM write ptr
    *((volatile uint32_t *)0x51105224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x51105230) = 0x0088aba0; // Layer control
    *((volatile uint32_t *)0x51105234) = 0x000f8000; // Layer control 2
    *((volatile uint32_t *)0x51105238) = 0x00005018; // Mask count
    *((volatile uint32_t *)0x5110523c) = 0x00004f20; // Mask offset
    *((volatile uint32_t *)0x51105240) = 0x0000001f; // Output channel count
    *((volatile uint32_t *)0x51105244) = 0x00000014; // TRAM ptr max
    *((volatile uint32_t *)0x5110524c) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x51105248) = 0xffffffff; // Mask and processor enables

    // Layer 82 quadrant 1
    *((volatile uint32_t *)0x52105204) = 0x002c8036; // Rows
    *((volatile uint32_t *)0x52105208) = 0x00028028; // Columns
    *((volatile uint32_t *)0x52105210) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x52105214) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x52105218) = 0x00000021; // Stride
    *((volatile uint32_t *)0x5210521c) = 0x00040c00; // SRAM write ptr
    *((volatile uint32_t *)0x52105224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x52105230) = 0x00880ba0; // Layer control
    *((volatile uint32_t *)0x52105234) = 0x000f8000; // Layer control 2
    *((volatile uint32_t *)0x52105238) = 0x00005018; // Mask count
    *((volatile uint32_t *)0x5210523c) = 0x00004f20; // Mask offset
    *((volatile uint32_t *)0x52105240) = 0x0000001f; // Output channel count
    *((volatile uint32_t *)0x52105244) = 0x00000014; // TRAM ptr max
    *((volatile uint32_t *)0x5210524c) = 0x00024000; // Post processing register
    *((volatile uint32_t *)0x52105248) = 0xffffffff; // Mask and processor enables

    // Layer 82 quadrant 2
    *((volatile uint32_t *)0x53105204) = 0x002c8036; // Rows
    *((volatile uint32_t *)0x53105208) = 0x00028028; // Columns
    *((volatile uint32_t *)0x53105210) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x53105214) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x53105218) = 0x00000021; // Stride
    *((volatile uint32_t *)0x5310521c) = 0x00040c00; // SRAM write ptr
    *((volatile uint32_t *)0x53105224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x53105230) = 0x00880ba0; // Layer control
    *((volatile uint32_t *)0x53105234) = 0x000f8000; // Layer control 2
    *((volatile uint32_t *)0x53105238) = 0x00005018; // Mask count
    *((volatile uint32_t *)0x5310523c) = 0x00004f20; // Mask offset
    *((volatile uint32_t *)0x53105240) = 0x0000001f; // Output channel count
    *((volatile uint32_t *)0x53105244) = 0x00000014; // TRAM ptr max
    *((volatile uint32_t *)0x5310524c) = 0x00024000; // Post processing register

    // Layer 82 quadrant 3
    *((volatile uint32_t *)0x54105204) = 0x002c8036; // Rows
    *((volatile uint32_t *)0x54105208) = 0x00028028; // Columns
    *((volatile uint32_t *)0x54105210) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x54105214) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x54105218) = 0x00000021; // Stride
    *((volatile uint32_t *)0x5410521c) = 0x00040c00; // SRAM write ptr
    *((volatile uint32_t *)0x54105224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x54105230) = 0x00880ba0; // Layer control
    *((volatile uint32_t *)0x54105234) = 0x000f8000; // Layer control 2
    *((volatile uint32_t *)0x54105238) = 0x00005018; // Mask count
    *((volatile uint32_t *)0x5410523c) = 0x00004f20; // Mask offset
    *((volatile uint32_t *)0x54105240) = 0x0000001f; // Output channel count
    *((volatile uint32_t *)0x54105244) = 0x00000014; // TRAM ptr max
    *((volatile uint32_t *)0x5410524c) = 0x00025040; // Post processing register

    // Layer 83 quadrant 0
    *((volatile uint32_t *)0x51105304) = 0x0018801a; // Rows
    *((volatile uint32_t *)0x51105308) = 0x00038012; // Columns
    *((volatile uint32_t *)0x51105310) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x51105314) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x51105318) = 0x00000021; // Stride
    *((volatile uint32_t *)0x5110531c) = 0x0004079c; // SRAM write ptr
    *((volatile uint32_t *)0x51105324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5110532c) = 0x00000c00; // SRAM read ptr
    *((volatile uint32_t *)0x51105330) = 0x0088cba0; // Layer control
    *((volatile uint32_t *)0x51105334) = 0x000f8000; // Layer control 2
    *((volatile uint32_t *)0x51105338) = 0x00005d18; // Mask count
    *((volatile uint32_t *)0x5110533c) = 0x00005c20; // Mask offset
    *((volatile uint32_t *)0x51105340) = 0x0000001f; // Output channel count
    *((volatile uint32_t *)0x51105344) = 0x00000009; // TRAM ptr max
    *((volatile uint32_t *)0x5110534c) = 0x00028000; // Post processing register

    // Layer 83 quadrant 1
    *((volatile uint32_t *)0x52105304) = 0x0018801a; // Rows
    *((volatile uint32_t *)0x52105308) = 0x00038012; // Columns
    *((volatile uint32_t *)0x52105310) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x52105314) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x52105318) = 0x00000021; // Stride
    *((volatile uint32_t *)0x5210531c) = 0x0004079c; // SRAM write ptr
    *((volatile uint32_t *)0x52105324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5210532c) = 0x00000c00; // SRAM read ptr
    *((volatile uint32_t *)0x52105330) = 0x00880ba0; // Layer control
    *((volatile uint32_t *)0x52105334) = 0x000f8000; // Layer control 2
    *((volatile uint32_t *)0x52105338) = 0x00005d18; // Mask count
    *((volatile uint32_t *)0x5210533c) = 0x00005c20; // Mask offset
    *((volatile uint32_t *)0x52105340) = 0x0000001f; // Output channel count
    *((volatile uint32_t *)0x52105344) = 0x00000009; // TRAM ptr max
    *((volatile uint32_t *)0x5210534c) = 0x00028000; // Post processing register

    // Layer 83 quadrant 2
    *((volatile uint32_t *)0x53105304) = 0x0018801a; // Rows
    *((volatile uint32_t *)0x53105308) = 0x00038012; // Columns
    *((volatile uint32_t *)0x53105310) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x53105314) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x53105318) = 0x00000021; // Stride
    *((volatile uint32_t *)0x5310531c) = 0x0004079c; // SRAM write ptr
    *((volatile uint32_t *)0x53105324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5310532c) = 0x00000c00; // SRAM read ptr
    *((volatile uint32_t *)0x53105330) = 0x00880ba0; // Layer control
    *((volatile uint32_t *)0x53105334) = 0x000f8000; // Layer control 2
    *((volatile uint32_t *)0x53105338) = 0x00005d18; // Mask count
    *((volatile uint32_t *)0x5310533c) = 0x00005c20; // Mask offset
    *((volatile uint32_t *)0x53105340) = 0x0000001f; // Output channel count
    *((volatile uint32_t *)0x53105344) = 0x00000009; // TRAM ptr max
    *((volatile uint32_t *)0x5310534c) = 0x00029060; // Post processing register
    *((volatile uint32_t *)0x53105348) = 0xffffffff; // Mask and processor enables

    // Layer 83 quadrant 3
    *((volatile uint32_t *)0x54105304) = 0x0018801a; // Rows
    *((volatile uint32_t *)0x54105308) = 0x00038012; // Columns
    *((volatile uint32_t *)0x54105310) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x54105314) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x54105318) = 0x00000021; // Stride
    *((volatile uint32_t *)0x5410531c) = 0x0004079c; // SRAM write ptr
    *((volatile uint32_t *)0x54105324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5410532c) = 0x00000c00; // SRAM read ptr
    *((volatile uint32_t *)0x54105330) = 0x00880ba0; // Layer control
    *((volatile uint32_t *)0x54105334) = 0x000f8000; // Layer control 2
    *((volatile uint32_t *)0x54105338) = 0x00005d18; // Mask count
    *((volatile uint32_t *)0x5410533c) = 0x00005c20; // Mask offset
    *((volatile uint32_t *)0x54105340) = 0x0000001f; // Output channel count
    *((volatile uint32_t *)0x54105344) = 0x00000009; // TRAM ptr max
    *((volatile uint32_t *)0x5410534c) = 0x00028000; // Post processing register
    *((volatile uint32_t *)0x54105348) = 0xffffffff; // Mask and processor enables

    // Layer 84 quadrant 0
    *((volatile uint32_t *)0x51105404) = 0x0001800d; // Rows
    *((volatile uint32_t *)0x51105408) = 0x00018009; // Columns
    *((volatile uint32_t *)0x51105418) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5110541c) = 0x00060020; // SRAM write ptr
    *((volatile uint32_t *)0x51105424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5110542c) = 0x0000079c; // SRAM read ptr
    *((volatile uint32_t *)0x51105430) = 0x0088cb20; // Layer control
    *((volatile uint32_t *)0x51105434) = 0x00078000; // Layer control 2
    *((volatile uint32_t *)0x51105438) = 0x00005d98; // Mask count
    *((volatile uint32_t *)0x5110543c) = 0x00005d20; // Mask offset
    *((volatile uint32_t *)0x51105440) = 0x0000000f; // Output channel count
    *((volatile uint32_t *)0x51105444) = 0x00000009; // TRAM ptr max
    *((volatile uint32_t *)0x5110544c) = 0x00026000; // Post processing register

    // Layer 84 quadrant 1
    *((volatile uint32_t *)0x52105404) = 0x0001800d; // Rows
    *((volatile uint32_t *)0x52105408) = 0x00018009; // Columns
    *((volatile uint32_t *)0x52105418) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5210541c) = 0x00060020; // SRAM write ptr
    *((volatile uint32_t *)0x52105424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5210542c) = 0x0000079c; // SRAM read ptr
    *((volatile uint32_t *)0x52105430) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x52105434) = 0x00078000; // Layer control 2
    *((volatile uint32_t *)0x52105438) = 0x00005d98; // Mask count
    *((volatile uint32_t *)0x5210543c) = 0x00005d20; // Mask offset
    *((volatile uint32_t *)0x52105440) = 0x0000000f; // Output channel count
    *((volatile uint32_t *)0x52105444) = 0x00000009; // TRAM ptr max
    *((volatile uint32_t *)0x5210544c) = 0x00026000; // Post processing register

    // Layer 84 quadrant 2
    *((volatile uint32_t *)0x53105404) = 0x0001800d; // Rows
    *((volatile uint32_t *)0x53105408) = 0x00018009; // Columns
    *((volatile uint32_t *)0x53105418) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5310541c) = 0x00060020; // SRAM write ptr
    *((volatile uint32_t *)0x53105424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5310542c) = 0x0000079c; // SRAM read ptr
    *((volatile uint32_t *)0x53105430) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x53105434) = 0x00078000; // Layer control 2
    *((volatile uint32_t *)0x53105438) = 0x00005d98; // Mask count
    *((volatile uint32_t *)0x5310543c) = 0x00005d20; // Mask offset
    *((volatile uint32_t *)0x53105440) = 0x0000000f; // Output channel count
    *((volatile uint32_t *)0x53105444) = 0x00000009; // TRAM ptr max
    *((volatile uint32_t *)0x5310544c) = 0x00026000; // Post processing register
    *((volatile uint32_t *)0x53105448) = 0xffffffff; // Mask and processor enables

    // Layer 84 quadrant 3
    *((volatile uint32_t *)0x54105404) = 0x0001800d; // Rows
    *((volatile uint32_t *)0x54105408) = 0x00018009; // Columns
    *((volatile uint32_t *)0x54105418) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5410541c) = 0x00060020; // SRAM write ptr
    *((volatile uint32_t *)0x54105424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5410542c) = 0x0000079c; // SRAM read ptr
    *((volatile uint32_t *)0x54105430) = 0x00880b20; // Layer control
    *((volatile uint32_t *)0x54105434) = 0x00078000; // Layer control 2
    *((volatile uint32_t *)0x54105438) = 0x00005d98; // Mask count
    *((volatile uint32_t *)0x5410543c) = 0x00005d20; // Mask offset
    *((volatile uint32_t *)0x54105440) = 0x0000000f; // Output channel count
    *((volatile uint32_t *)0x54105444) = 0x00000009; // TRAM ptr max
    *((volatile uint32_t *)0x5410544c) = 0x00027060; // Post processing register
    *((volatile uint32_t *)0x54105448) = 0xffffffff; // Mask and processor enables

    // Layer 85 quadrant 0
    *((volatile uint32_t *)0x51105504) = 0x000c800c; // Rows
    *((volatile uint32_t *)0x51105508) = 0x00028008; // Columns
    *((volatile uint32_t *)0x51105510) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x51105514) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x51105518) = 0x00000021; // Stride
    *((volatile uint32_t *)0x5110551c) = 0x00040820; // SRAM write ptr
    *((volatile uint32_t *)0x51105524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5110552c) = 0x00000020; // SRAM read ptr
    *((volatile uint32_t *)0x51105530) = 0x00888ba0; // Layer control
    *((volatile uint32_t *)0x51105534) = 0x00078000; // Layer control 2
    *((volatile uint32_t *)0x51105538) = 0x00005e18; // Mask count
    *((volatile uint32_t *)0x5110553c) = 0x00005da0; // Mask offset
    *((volatile uint32_t *)0x51105540) = 0x0000000f; // Output channel count
    *((volatile uint32_t *)0x51105544) = 0x00000004; // TRAM ptr max
    *((volatile uint32_t *)0x5110554c) = 0x00024000; // Post processing register

    // Layer 85 quadrant 1
    *((volatile uint32_t *)0x52105504) = 0x000c800c; // Rows
    *((volatile uint32_t *)0x52105508) = 0x00028008; // Columns
    *((volatile uint32_t *)0x52105510) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x52105514) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x52105518) = 0x00000021; // Stride
    *((volatile uint32_t *)0x5210551c) = 0x00040820; // SRAM write ptr
    *((volatile uint32_t *)0x52105524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5210552c) = 0x00000020; // SRAM read ptr
    *((volatile uint32_t *)0x52105530) = 0x00880ba0; // Layer control
    *((volatile uint32_t *)0x52105534) = 0x00078000; // Layer control 2
    *((volatile uint32_t *)0x52105538) = 0x00005e18; // Mask count
    *((volatile uint32_t *)0x5210553c) = 0x00005da0; // Mask offset
    *((volatile uint32_t *)0x52105540) = 0x0000000f; // Output channel count
    *((volatile uint32_t *)0x52105544) = 0x00000004; // TRAM ptr max
    *((volatile uint32_t *)0x5210554c) = 0x00024000; // Post processing register

    // Layer 85 quadrant 2
    *((volatile uint32_t *)0x53105504) = 0x000c800c; // Rows
    *((volatile uint32_t *)0x53105508) = 0x00028008; // Columns
    *((volatile uint32_t *)0x53105510) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x53105514) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x53105518) = 0x00000021; // Stride
    *((volatile uint32_t *)0x5310551c) = 0x00040820; // SRAM write ptr
    *((volatile uint32_t *)0x53105524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5310552c) = 0x00000020; // SRAM read ptr
    *((volatile uint32_t *)0x53105530) = 0x00880ba0; // Layer control
    *((volatile uint32_t *)0x53105534) = 0x00078000; // Layer control 2
    *((volatile uint32_t *)0x53105538) = 0x00005e18; // Mask count
    *((volatile uint32_t *)0x5310553c) = 0x00005da0; // Mask offset
    *((volatile uint32_t *)0x53105540) = 0x0000000f; // Output channel count
    *((volatile uint32_t *)0x53105544) = 0x00000004; // TRAM ptr max
    *((volatile uint32_t *)0x5310554c) = 0x00024000; // Post processing register

    // Layer 85 quadrant 3
    *((volatile uint32_t *)0x54105504) = 0x000c800c; // Rows
    *((volatile uint32_t *)0x54105508) = 0x00028008; // Columns
    *((volatile uint32_t *)0x54105510) = 0x00000001; // Pooling rows
    *((volatile uint32_t *)0x54105514) = 0x00000001; // Pooling columns
    *((volatile uint32_t *)0x54105518) = 0x00000021; // Stride
    *((volatile uint32_t *)0x5410551c) = 0x00040820; // SRAM write ptr
    *((volatile uint32_t *)0x54105524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5410552c) = 0x00000020; // SRAM read ptr
    *((volatile uint32_t *)0x54105530) = 0x00880ba0; // Layer control
    *((volatile uint32_t *)0x54105534) = 0x00078000; // Layer control 2
    *((volatile uint32_t *)0x54105538) = 0x00005e18; // Mask count
    *((volatile uint32_t *)0x5410553c) = 0x00005da0; // Mask offset
    *((volatile uint32_t *)0x54105540) = 0x0000000f; // Output channel count
    *((volatile uint32_t *)0x54105544) = 0x00000004; // TRAM ptr max
    *((volatile uint32_t *)0x5410554c) = 0x00025070; // Post processing register
    *((volatile uint32_t *)0x54105548) = 0xffffffff; // Mask and processor enables

    // Layer 86 quadrant 0
    *((volatile uint32_t *)0x51105604) = 0x0001801b; // Rows
    *((volatile uint32_t *)0x51105608) = 0x00018014; // Columns
    *((volatile uint32_t *)0x51105618) = 0x00000010; // Stride
    *((volatile uint32_t *)0x51105624) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5110562c) = 0x00000c00; // SRAM read ptr
    *((volatile uint32_t *)0x51105630) = 0x0088c920; // Layer control
    *((volatile uint32_t *)0x51105634) = 0x00038000; // Layer control 2
    *((volatile uint32_t *)0x51105638) = 0x00005e58; // Mask count
    *((volatile uint32_t *)0x5110563c) = 0x00005e20; // Mask offset
    *((volatile uint32_t *)0x51105640) = 0x00000007; // Output channel count
    *((volatile uint32_t *)0x51105644) = 0x00000014; // TRAM ptr max
    *((volatile uint32_t *)0x5110564c) = 0x00023080; // Post processing register

    // Layer 86 quadrant 1
    *((volatile uint32_t *)0x52105604) = 0x0001801b; // Rows
    *((volatile uint32_t *)0x52105608) = 0x00018014; // Columns
    *((volatile uint32_t *)0x52105618) = 0x00000010; // Stride
    *((volatile uint32_t *)0x52105624) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5210562c) = 0x00000c00; // SRAM read ptr
    *((volatile uint32_t *)0x52105630) = 0x00880920; // Layer control
    *((volatile uint32_t *)0x52105634) = 0x00038000; // Layer control 2
    *((volatile uint32_t *)0x52105638) = 0x00005e58; // Mask count
    *((volatile uint32_t *)0x5210563c) = 0x00005e20; // Mask offset
    *((volatile uint32_t *)0x52105640) = 0x00000007; // Output channel count
    *((volatile uint32_t *)0x52105644) = 0x00000014; // TRAM ptr max
    *((volatile uint32_t *)0x5210564c) = 0x00022000; // Post processing register

    // Layer 86 quadrant 2
    *((volatile uint32_t *)0x53105604) = 0x0001801b; // Rows
    *((volatile uint32_t *)0x53105608) = 0x00018014; // Columns
    *((volatile uint32_t *)0x53105618) = 0x00000010; // Stride
    *((volatile uint32_t *)0x53105624) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5310562c) = 0x00000c00; // SRAM read ptr
    *((volatile uint32_t *)0x53105630) = 0x00880920; // Layer control
    *((volatile uint32_t *)0x53105634) = 0x00038000; // Layer control 2
    *((volatile uint32_t *)0x53105638) = 0x00005e58; // Mask count
    *((volatile uint32_t *)0x5310563c) = 0x00005e20; // Mask offset
    *((volatile uint32_t *)0x53105640) = 0x00000007; // Output channel count
    *((volatile uint32_t *)0x53105644) = 0x00000014; // TRAM ptr max
    *((volatile uint32_t *)0x5310564c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x53105648) = 0xffffffff; // Mask and processor enables

    // Layer 86 quadrant 3
    *((volatile uint32_t *)0x54105604) = 0x0001801b; // Rows
    *((volatile uint32_t *)0x54105608) = 0x00018014; // Columns
    *((volatile uint32_t *)0x54105618) = 0x00000010; // Stride
    *((volatile uint32_t *)0x54105624) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5410562c) = 0x00000c00; // SRAM read ptr
    *((volatile uint32_t *)0x54105630) = 0x00880920; // Layer control
    *((volatile uint32_t *)0x54105634) = 0x00038000; // Layer control 2
    *((volatile uint32_t *)0x54105638) = 0x00005e58; // Mask count
    *((volatile uint32_t *)0x5410563c) = 0x00005e20; // Mask offset
    *((volatile uint32_t *)0x54105640) = 0x00000007; // Output channel count
    *((volatile uint32_t *)0x54105644) = 0x00000014; // TRAM ptr max
    *((volatile uint32_t *)0x5410564c) = 0x00022000; // Post processing register
    *((volatile uint32_t *)0x54105648) = 0xffffffff; // Mask and processor enables

    // Layer 87 quadrant 0
    *((volatile uint32_t *)0x51105704) = 0x00018006; // Rows
    *((volatile uint32_t *)0x51105708) = 0x00018004; // Columns
    *((volatile uint32_t *)0x51105718) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5110571c) = 0x0000024c; // SRAM write ptr
    *((volatile uint32_t *)0x51105724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5110572c) = 0x00000820; // SRAM read ptr
    *((volatile uint32_t *)0x51105730) = 0x00886920; // Layer control
    *((volatile uint32_t *)0x51105734) = 0x00038000; // Layer control 2
    *((volatile uint32_t *)0x51105738) = 0x00005dd8; // Mask count
    *((volatile uint32_t *)0x5110573c) = 0x00005da0; // Mask offset
    *((volatile uint32_t *)0x51105740) = 0x00000007; // Output channel count
    *((volatile uint32_t *)0x51105744) = 0x00000004; // TRAM ptr max
    *((volatile uint32_t *)0x5110574c) = 0x00026000; // Post processing register

    // Layer 87 quadrant 1
    *((volatile uint32_t *)0x52105704) = 0x00018006; // Rows
    *((volatile uint32_t *)0x52105708) = 0x00018004; // Columns
    *((volatile uint32_t *)0x52105718) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5210571c) = 0x0000024c; // SRAM write ptr
    *((volatile uint32_t *)0x52105724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5210572c) = 0x00000820; // SRAM read ptr
    *((volatile uint32_t *)0x52105730) = 0x00880920; // Layer control
    *((volatile uint32_t *)0x52105734) = 0x00038000; // Layer control 2
    *((volatile uint32_t *)0x52105738) = 0x00005dd8; // Mask count
    *((volatile uint32_t *)0x5210573c) = 0x00005da0; // Mask offset
    *((volatile uint32_t *)0x52105740) = 0x00000007; // Output channel count
    *((volatile uint32_t *)0x52105744) = 0x00000004; // TRAM ptr max
    *((volatile uint32_t *)0x5210574c) = 0x00027080; // Post processing register

    // Layer 87 quadrant 2
    *((volatile uint32_t *)0x53105704) = 0x00018006; // Rows
    *((volatile uint32_t *)0x53105708) = 0x00018004; // Columns
    *((volatile uint32_t *)0x53105718) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5310571c) = 0x0000024c; // SRAM write ptr
    *((volatile uint32_t *)0x53105724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5310572c) = 0x00000820; // SRAM read ptr
    *((volatile uint32_t *)0x53105730) = 0x00880920; // Layer control
    *((volatile uint32_t *)0x53105734) = 0x00038000; // Layer control 2
    *((volatile uint32_t *)0x53105738) = 0x00005dd8; // Mask count
    *((volatile uint32_t *)0x5310573c) = 0x00005da0; // Mask offset
    *((volatile uint32_t *)0x53105740) = 0x00000007; // Output channel count
    *((volatile uint32_t *)0x53105744) = 0x00000004; // TRAM ptr max
    *((volatile uint32_t *)0x5310574c) = 0x00026000; // Post processing register
    *((volatile uint32_t *)0x53105748) = 0xffffffff; // Mask and processor enables

    // Layer 87 quadrant 3
    *((volatile uint32_t *)0x54105704) = 0x00018006; // Rows
    *((volatile uint32_t *)0x54105708) = 0x00018004; // Columns
    *((volatile uint32_t *)0x54105718) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5410571c) = 0x0000024c; // SRAM write ptr
    *((volatile uint32_t *)0x54105724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5410572c) = 0x00000820; // SRAM read ptr
    *((volatile uint32_t *)0x54105730) = 0x00880920; // Layer control
    *((volatile uint32_t *)0x54105734) = 0x00038000; // Layer control 2
    *((volatile uint32_t *)0x54105738) = 0x00005dd8; // Mask count
    *((volatile uint32_t *)0x5410573c) = 0x00005da0; // Mask offset
    *((volatile uint32_t *)0x54105740) = 0x00000007; // Output channel count
    *((volatile uint32_t *)0x54105744) = 0x00000004; // TRAM ptr max
    *((volatile uint32_t *)0x5410574c) = 0x00026000; // Post processing register

    // Layer 88 quadrant 0
    *((volatile uint32_t *)0x51105804) = 0x0001801b; // Rows
    *((volatile uint32_t *)0x51105808) = 0x00018014; // Columns
    *((volatile uint32_t *)0x51105818) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5110581c) = 0x00010000; // SRAM write ptr
    *((volatile uint32_t *)0x51105824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5110582c) = 0x00000c00; // SRAM read ptr
    *((volatile uint32_t *)0x51105830) = 0x0088c920; // Layer control
    *((volatile uint32_t *)0x51105834) = 0x00018000; // Layer control 2
    *((volatile uint32_t *)0x51105838) = 0x00005e78; // Mask count
    *((volatile uint32_t *)0x5110583c) = 0x00005e60; // Mask offset
    *((volatile uint32_t *)0x51105840) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x51105844) = 0x00000014; // TRAM ptr max

    // Layer 88 quadrant 1
    *((volatile uint32_t *)0x52105804) = 0x0001801b; // Rows
    *((volatile uint32_t *)0x52105808) = 0x00018014; // Columns
    *((volatile uint32_t *)0x52105818) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5210581c) = 0x00010000; // SRAM write ptr
    *((volatile uint32_t *)0x52105824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5210582c) = 0x00000c00; // SRAM read ptr
    *((volatile uint32_t *)0x52105830) = 0x00880920; // Layer control
    *((volatile uint32_t *)0x52105834) = 0x00018000; // Layer control 2
    *((volatile uint32_t *)0x52105838) = 0x00005e78; // Mask count
    *((volatile uint32_t *)0x5210583c) = 0x00005e60; // Mask offset
    *((volatile uint32_t *)0x52105840) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x52105844) = 0x00000014; // TRAM ptr max

    // Layer 88 quadrant 2
    *((volatile uint32_t *)0x53105804) = 0x0001801b; // Rows
    *((volatile uint32_t *)0x53105808) = 0x00018014; // Columns
    *((volatile uint32_t *)0x53105818) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5310581c) = 0x00010000; // SRAM write ptr
    *((volatile uint32_t *)0x53105824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5310582c) = 0x00000c00; // SRAM read ptr
    *((volatile uint32_t *)0x53105830) = 0x00880920; // Layer control
    *((volatile uint32_t *)0x53105834) = 0x00018000; // Layer control 2
    *((volatile uint32_t *)0x53105838) = 0x00005e78; // Mask count
    *((volatile uint32_t *)0x5310583c) = 0x00005e60; // Mask offset
    *((volatile uint32_t *)0x53105840) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x53105844) = 0x00000014; // TRAM ptr max
    *((volatile uint32_t *)0x5310584c) = 0x00001080; // Post processing register
    *((volatile uint32_t *)0x53105848) = 0xffffffff; // Mask and processor enables

    // Layer 88 quadrant 3
    *((volatile uint32_t *)0x54105804) = 0x0001801b; // Rows
    *((volatile uint32_t *)0x54105808) = 0x00018014; // Columns
    *((volatile uint32_t *)0x54105818) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5410581c) = 0x00010000; // SRAM write ptr
    *((volatile uint32_t *)0x54105824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5410582c) = 0x00000c00; // SRAM read ptr
    *((volatile uint32_t *)0x54105830) = 0x00880920; // Layer control
    *((volatile uint32_t *)0x54105834) = 0x00018000; // Layer control 2
    *((volatile uint32_t *)0x54105838) = 0x00005e78; // Mask count
    *((volatile uint32_t *)0x5410583c) = 0x00005e60; // Mask offset
    *((volatile uint32_t *)0x54105840) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x54105844) = 0x00000014; // TRAM ptr max
    *((volatile uint32_t *)0x54105848) = 0xffffffff; // Mask and processor enables

    // Layer 89 quadrant 0
    *((volatile uint32_t *)0x51105904) = 0x00018006; // Rows
    *((volatile uint32_t *)0x51105908) = 0x00018004; // Columns
    *((volatile uint32_t *)0x51105918) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5110591c) = 0x0001024c; // SRAM write ptr
    *((volatile uint32_t *)0x51105924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5110592c) = 0x00000820; // SRAM read ptr
    *((volatile uint32_t *)0x51105930) = 0x0088c920; // Layer control
    *((volatile uint32_t *)0x51105934) = 0x00018000; // Layer control 2
    *((volatile uint32_t *)0x51105938) = 0x00005df8; // Mask count
    *((volatile uint32_t *)0x5110593c) = 0x00005de0; // Mask offset
    *((volatile uint32_t *)0x51105940) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x51105944) = 0x00000004; // TRAM ptr max

    // Layer 89 quadrant 1
    *((volatile uint32_t *)0x52105904) = 0x00018006; // Rows
    *((volatile uint32_t *)0x52105908) = 0x00018004; // Columns
    *((volatile uint32_t *)0x52105918) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5210591c) = 0x0001024c; // SRAM write ptr
    *((volatile uint32_t *)0x52105924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5210592c) = 0x00000820; // SRAM read ptr
    *((volatile uint32_t *)0x52105930) = 0x00880920; // Layer control
    *((volatile uint32_t *)0x52105934) = 0x00018000; // Layer control 2
    *((volatile uint32_t *)0x52105938) = 0x00005df8; // Mask count
    *((volatile uint32_t *)0x5210593c) = 0x00005de0; // Mask offset
    *((volatile uint32_t *)0x52105940) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x52105944) = 0x00000004; // TRAM ptr max

    // Layer 89 quadrant 2
    *((volatile uint32_t *)0x53105904) = 0x00018006; // Rows
    *((volatile uint32_t *)0x53105908) = 0x00018004; // Columns
    *((volatile uint32_t *)0x53105918) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5310591c) = 0x0001024c; // SRAM write ptr
    *((volatile uint32_t *)0x53105924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5310592c) = 0x00000820; // SRAM read ptr
    *((volatile uint32_t *)0x53105930) = 0x00880920; // Layer control
    *((volatile uint32_t *)0x53105934) = 0x00018000; // Layer control 2
    *((volatile uint32_t *)0x53105938) = 0x00005df8; // Mask count
    *((volatile uint32_t *)0x5310593c) = 0x00005de0; // Mask offset
    *((volatile uint32_t *)0x53105940) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x53105944) = 0x00000004; // TRAM ptr max
    *((volatile uint32_t *)0x53105948) = 0xffffffff; // Mask and processor enables

    // Layer 89 quadrant 3
    *((volatile uint32_t *)0x54105904) = 0x00018006; // Rows
    *((volatile uint32_t *)0x54105908) = 0x00018004; // Columns
    *((volatile uint32_t *)0x54105918) = 0x00000010; // Stride
    *((volatile uint32_t *)0x5410591c) = 0x0001024c; // SRAM write ptr
    *((volatile uint32_t *)0x54105924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t *)0x5410592c) = 0x00000820; // SRAM read ptr
    *((volatile uint32_t *)0x54105930) = 0x00880920; // Layer control
    *((volatile uint32_t *)0x54105934) = 0x00018000; // Layer control 2
    *((volatile uint32_t *)0x54105938) = 0x00005df8; // Mask count
    *((volatile uint32_t *)0x5410593c) = 0x00005de0; // Mask offset
    *((volatile uint32_t *)0x54105940) = 0x00000003; // Output channel count
    *((volatile uint32_t *)0x54105944) = 0x00000004; // TRAM ptr max
    *((volatile uint32_t *)0x5410594c) = 0x00001080; // Post processing register

    return CNN_OK;
}

int cnn_1_start(void)
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

int cnn_1_unload(uint32_t *out_buf32)
{
    uint8_t *out_buf = (uint8_t *)out_buf32;
    uint32_t val;
    volatile uint32_t *addr;
    int i;
    uint32_t offs;

    // Custom unload for this network, layer 86: 8-bit data, shape: (8, 28, 21)
    offs = 0x0000;
    addr = (volatile uint32_t *)0x51800000;
    for (i = 0; i < 588; i++) {
        val = *addr++;
        out_buf[offs] = val & 0xff;
        out_buf[offs + 0x24c] = (val >> 8) & 0xff;
        out_buf[offs + 0x498] = (val >> 16) & 0xff;
        out_buf[offs + 0x6e4] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x06e4;
    addr = (volatile uint32_t *)0x51820000;
    for (i = 0; i < 588; i++) {
        val = *addr++;
        out_buf[offs] = val & 0xff;
        out_buf[offs + 0x24c] = (val >> 8) & 0xff;
        out_buf[offs + 0x498] = (val >> 16) & 0xff;
        out_buf[offs + 0x6e4] = (val >> 24) & 0xff;
        offs++;
    }

    // Custom unload for this network, layer 87: 8-bit data, shape: (8, 7, 5)
    offs = 0x0000;
    out_buf = ((uint8_t *)out_buf32) + 0x1260;
    addr = (volatile uint32_t *)0x51800930;
    for (i = 0; i < 35; i++) {
        val = *addr++;
        out_buf[offs] = val & 0xff;
        out_buf[offs + 0x23] = (val >> 8) & 0xff;
        out_buf[offs + 0x46] = (val >> 16) & 0xff;
        out_buf[offs + 0x69] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x0069;
    addr = (volatile uint32_t *)0x51820930;
    for (i = 0; i < 35; i++) {
        val = *addr++;
        out_buf[offs] = val & 0xff;
        out_buf[offs + 0x23] = (val >> 8) & 0xff;
        out_buf[offs + 0x46] = (val >> 16) & 0xff;
        out_buf[offs + 0x69] = (val >> 24) & 0xff;
        offs++;
    }

    // Custom unload for this network, layer 88: 8-bit data, shape: (4, 28, 21)
    offs = 0x0000;
    out_buf = ((uint8_t *)out_buf32) + 0x1378;
    addr = (volatile uint32_t *)0x51840000;
    for (i = 0; i < 588; i++) {
        val = *addr++;
        out_buf[offs] = val & 0xff;
        out_buf[offs + 0x24c] = (val >> 8) & 0xff;
        out_buf[offs + 0x498] = (val >> 16) & 0xff;
        out_buf[offs + 0x6e4] = (val >> 24) & 0xff;
        offs++;
    }

    // Custom unload for this network, layer 89: 8-bit data, shape: (4, 7, 5)
    offs = 0x0000;
    out_buf = ((uint8_t *)out_buf32) + 0x1ca8;
    for (i = 0; i < 35; i++) {
        val = *addr++;
        out_buf[offs] = val & 0xff;
        out_buf[offs + 0x23] = (val >> 8) & 0xff;
        out_buf[offs + 0x46] = (val >> 16) & 0xff;
        out_buf[offs + 0x69] = (val >> 24) & 0xff;
        offs++;
    }

    return CNN_OK;
}

int cnn_1_enable(uint32_t clock_source, uint32_t clock_divider)
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

    MXC_NVIC_SetVector(CNN_IRQn, CNN_1_ISR); // Set CNN complete vector

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
    MXC_GCFR->reg2 |= 0xf; // Iso
    MXC_GCFR->reg0 = 0x0; // Power
    MXC_GCFR->reg1 = 0x0; // Mask memory
    MXC_GCFR->reg3 = 0x0; // Reset

    return CNN_OK;
}
