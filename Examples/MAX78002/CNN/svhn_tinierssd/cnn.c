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

// svhn_tinierssd
// Created using ai8xize.py --test-dir sdk/Examples/MAX78002/CNN --prefix svhn_tinierssd --checkpoint-file trained/ai85-svhn-tinierssd-qat8-q.pth.tar --config-file networks/svhn-tinierssd.yaml --overlap-data --device MAX78002 --timer 0 --display-checkpoint --verbose --overwrite

// DO NOT EDIT - regenerate this file instead!

// Configuring 20 layers
// Input data: HWC
// Layer 0: 3x74x74, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 32x74x74 output
// Layer 1: 32x74x74, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 32x74x74 output
// Layer 2: 32x74x74, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 64x37x37 output
// Layer 3: 64x37x37, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 64x37x37 output
// Layer 4: 64x37x37, max pool 3x3 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 64x18x18 output
// Layer 5: 64x18x18, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 64x18x18 output
// Layer 6: 64x18x18, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 128x18x18 output
// Layer 7: 128x18x18, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 32x18x18 output
// Layer 8: 32x18x18, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 32x9x9 output
// Layer 9: 32x9x9, max pool 3x3 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 32x4x4 output
// Layer 10: 32x4x4, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 16x4x4 output
// Layer 11: 16x4x4, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 16x2x2 output
// Layer 12: 32x18x18, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, no activation, 16x18x18 output
// Layer 13: 32x9x9, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, no activation, 16x9x9 output
// Layer 14: 32x4x4, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, no activation, 16x4x4 output
// Layer 15: 16x2x2, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, no activation, 16x2x2 output
// Layer 16: 32x18x18, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, no activation, 44x18x18 output
// Layer 17: 32x9x9, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, no activation, 44x9x9 output
// Layer 18: 32x4x4, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, no activation, 44x4x4 output
// Layer 19: 16x2x2, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, no activation, 44x2x2 output

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
    *((volatile uint32_t*)0x51000000) &= ~((1 << 12) | 1);
    *((volatile uint32_t*)0x52000000) &= ~((1 << 12) | 1);
    *((volatile uint32_t*)0x53000000) &= ~((1 << 12) | 1);
    *((volatile uint32_t*)0x54000000) &= ~((1 << 12) | 1);

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

    *((volatile uint32_t*)0x51000000) |= 1; // Re-enable quadrant 0

    return CNN_OK;
}

int cnn_stop(void)
{
    *((volatile uint32_t*)0x51000000) &= ~1; // Disable quadrant 0

    return CNN_OK;
}

void memcpy32(uint32_t* dst, const uint32_t* src, int n)
{
    while (n-- > 0) { *dst++ = *src++; }
}

static const uint32_t kernels[] = KERNELS;

int cnn_load_weights(void)
{
    uint32_t len;
    volatile uint32_t* addr;
    const uint32_t* ptr = kernels;

    while ((addr = (volatile uint32_t*)*ptr++) != 0) {
        *((volatile uint8_t*)((uint32_t)addr | 1)) = 0x01; // Set address
        len                                        = *ptr++;
        while (len-- > 0) *addr++ = *ptr++;
    }

    return CNN_OK;
}

static const uint8_t bias_0[] = BIAS_0;
static const uint8_t bias_1[] = BIAS_1;
static const uint8_t bias_2[] = BIAS_2;
static const uint8_t bias_3[] = BIAS_3;

static void memcpy_8to32(uint32_t* dst, const uint8_t* src, int n)
{
    while (n-- > 0) { *dst++ = *src++; }
}

int cnn_load_bias(void)
{
    memcpy_8to32((uint32_t*)0x51180000, bias_0, sizeof(uint8_t) * 208);
    memcpy_8to32((uint32_t*)0x52180000, bias_1, sizeof(uint8_t) * 208);
    memcpy_8to32((uint32_t*)0x53180000, bias_2, sizeof(uint8_t) * 200);
    memcpy_8to32((uint32_t*)0x54180000, bias_3, sizeof(uint8_t) * 200);

    return CNN_OK;
}

int cnn_init(void)
{
    *((volatile uint32_t*)0x51000000) = 0x00000008; // Enable clocks
    *((volatile uint32_t*)0x52000000) = 0x00000008; // Enable clocks
    *((volatile uint32_t*)0x53000000) = 0x00000008; // Enable clocks
    *((volatile uint32_t*)0x54000000) = 0x00000008; // Enable clocks
    *((volatile uint32_t*)0x50001000) = 0x00000000; // AON control
    *((volatile uint32_t*)0x51000004) = 0x0000040e; // SRAM control
    *((volatile uint32_t*)0x52000004) = 0x0000040e; // SRAM control
    *((volatile uint32_t*)0x53000004) = 0x0000040e; // SRAM control
    *((volatile uint32_t*)0x54000004) = 0x0000040e; // SRAM control
    *((volatile uint32_t*)0x5100000c) = 0x00001c80; // Clear registers
    *((volatile uint32_t*)0x5200000c) = 0x00001c80; // Clear registers
    *((volatile uint32_t*)0x5300000c) = 0x00001c80; // Clear registers
    *((volatile uint32_t*)0x5400000c) = 0x00001c80; // Clear registers
    while ((*((volatile uint32_t*)0x5100000c) & 0x2000000) != 0x2000000)
        ; // Wait for clear
    while ((*((volatile uint32_t*)0x5200000c) & 0x2000000) != 0x2000000)
        ; // Wait for clear
    while ((*((volatile uint32_t*)0x5300000c) & 0x2000000) != 0x2000000)
        ; // Wait for clear
    while ((*((volatile uint32_t*)0x5400000c) & 0x2000000) != 0x2000000)
        ;                                           // Wait for clear
    *((volatile uint32_t*)0x5100000c) = 0x00000000; // Reset BIST
    *((volatile uint32_t*)0x5200000c) = 0x00000000; // Reset BIST
    *((volatile uint32_t*)0x5300000c) = 0x00000000; // Reset BIST
    *((volatile uint32_t*)0x5400000c) = 0x00000000; // Reset BIST

    *((volatile uint32_t*)0x51000000) = 0x00100008; // Stop SM
    *((volatile uint32_t*)0x51000008) = 0x00000013; // Layer count
    *((volatile uint32_t*)0x52000000) = 0x00100008; // Stop SM
    *((volatile uint32_t*)0x52000008) = 0x00000013; // Layer count
    *((volatile uint32_t*)0x53000000) = 0x00100008; // Stop SM
    *((volatile uint32_t*)0x53000008) = 0x00000013; // Layer count
    *((volatile uint32_t*)0x54000000) = 0x00100008; // Stop SM
    *((volatile uint32_t*)0x54000008) = 0x00000013; // Layer count

    return CNN_OK;
}

int cnn_configure(void)
{
    // Layer 0 quadrant 0
    *((volatile uint32_t*)0x51100004) = 0x00018049; // Rows
    *((volatile uint32_t*)0x51100008) = 0x00018049; // Columns
    *((volatile uint32_t*)0x51100018) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5110001c) = 0x00040800; // SRAM write ptr
    *((volatile uint32_t*)0x51100024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5110002c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x51100030) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x51100034) = 0x000f8000; // Layer control 2
    *((volatile uint32_t*)0x51100038) = 0x000000f8; // Mask count
    *((volatile uint32_t*)0x51100040) = 0x0000001f; // Output channel count
    *((volatile uint32_t*)0x51100044) = 0x00000049; // TRAM ptr max
    *((volatile uint32_t*)0x5110004c) = 0x00023080; // Post processing register
    *((volatile uint32_t*)0x51100048) = 0x00070007; // Mask and processor enables

    // Layer 0 quadrant 1
    *((volatile uint32_t*)0x52100004) = 0x00018049; // Rows
    *((volatile uint32_t*)0x52100008) = 0x00018049; // Columns
    *((volatile uint32_t*)0x52100018) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5210001c) = 0x00040800; // SRAM write ptr
    *((volatile uint32_t*)0x52100024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5210002c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x52100030) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x52100034) = 0x000f8000; // Layer control 2
    *((volatile uint32_t*)0x52100038) = 0x000000f8; // Mask count
    *((volatile uint32_t*)0x52100040) = 0x0000001f; // Output channel count
    *((volatile uint32_t*)0x52100044) = 0x00000049; // TRAM ptr max
    *((volatile uint32_t*)0x5210004c) = 0x00022000; // Post processing register

    // Layer 0 quadrant 2
    *((volatile uint32_t*)0x53100004) = 0x00018049; // Rows
    *((volatile uint32_t*)0x53100008) = 0x00018049; // Columns
    *((volatile uint32_t*)0x53100018) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5310001c) = 0x00040800; // SRAM write ptr
    *((volatile uint32_t*)0x53100024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5310002c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x53100030) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x53100034) = 0x000f8000; // Layer control 2
    *((volatile uint32_t*)0x53100038) = 0x000000f8; // Mask count
    *((volatile uint32_t*)0x53100040) = 0x0000001f; // Output channel count
    *((volatile uint32_t*)0x53100044) = 0x00000049; // TRAM ptr max
    *((volatile uint32_t*)0x5310004c) = 0x00022000; // Post processing register

    // Layer 0 quadrant 3
    *((volatile uint32_t*)0x54100004) = 0x00018049; // Rows
    *((volatile uint32_t*)0x54100008) = 0x00018049; // Columns
    *((volatile uint32_t*)0x54100018) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5410001c) = 0x00040800; // SRAM write ptr
    *((volatile uint32_t*)0x54100024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5410002c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x54100030) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x54100034) = 0x000f8000; // Layer control 2
    *((volatile uint32_t*)0x54100038) = 0x000000f8; // Mask count
    *((volatile uint32_t*)0x54100040) = 0x0000001f; // Output channel count
    *((volatile uint32_t*)0x54100044) = 0x00000049; // TRAM ptr max
    *((volatile uint32_t*)0x5410004c) = 0x00022000; // Post processing register

    // Layer 1 quadrant 0
    *((volatile uint32_t*)0x51100104) = 0x00018049; // Rows
    *((volatile uint32_t*)0x51100108) = 0x00018049; // Columns
    *((volatile uint32_t*)0x51100118) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5110011c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t*)0x51100124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5110012c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x51100130) = 0x0088eb20; // Layer control
    *((volatile uint32_t*)0x51100134) = 0x000f8000; // Layer control 2
    *((volatile uint32_t*)0x51100138) = 0x000000f8; // Mask count
    *((volatile uint32_t*)0x51100140) = 0x0000001f; // Output channel count
    *((volatile uint32_t*)0x51100144) = 0x00000049; // TRAM ptr max
    *((volatile uint32_t*)0x5110014c) = 0x00024000; // Post processing register

    // Layer 1 quadrant 1
    *((volatile uint32_t*)0x52100104) = 0x00018049; // Rows
    *((volatile uint32_t*)0x52100108) = 0x00018049; // Columns
    *((volatile uint32_t*)0x52100118) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5210011c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t*)0x52100124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5210012c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x52100130) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x52100134) = 0x000f8000; // Layer control 2
    *((volatile uint32_t*)0x52100138) = 0x000000f8; // Mask count
    *((volatile uint32_t*)0x52100140) = 0x0000001f; // Output channel count
    *((volatile uint32_t*)0x52100144) = 0x00000049; // TRAM ptr max
    *((volatile uint32_t*)0x5210014c) = 0x00025080; // Post processing register

    // Layer 1 quadrant 2
    *((volatile uint32_t*)0x53100104) = 0x00018049; // Rows
    *((volatile uint32_t*)0x53100108) = 0x00018049; // Columns
    *((volatile uint32_t*)0x53100118) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5310011c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t*)0x53100124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5310012c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x53100130) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x53100134) = 0x000f8000; // Layer control 2
    *((volatile uint32_t*)0x53100138) = 0x000000f8; // Mask count
    *((volatile uint32_t*)0x53100140) = 0x0000001f; // Output channel count
    *((volatile uint32_t*)0x53100144) = 0x00000049; // TRAM ptr max
    *((volatile uint32_t*)0x5310014c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x53100148) = 0xffffffff; // Mask and processor enables

    // Layer 1 quadrant 3
    *((volatile uint32_t*)0x54100104) = 0x00018049; // Rows
    *((volatile uint32_t*)0x54100108) = 0x00018049; // Columns
    *((volatile uint32_t*)0x54100118) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5410011c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t*)0x54100124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5410012c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x54100130) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x54100134) = 0x000f8000; // Layer control 2
    *((volatile uint32_t*)0x54100138) = 0x000000f8; // Mask count
    *((volatile uint32_t*)0x54100140) = 0x0000001f; // Output channel count
    *((volatile uint32_t*)0x54100144) = 0x00000049; // TRAM ptr max
    *((volatile uint32_t*)0x5410014c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x54100148) = 0xffffffff; // Mask and processor enables

    // Layer 2 quadrant 0
    *((volatile uint32_t*)0x51100204) = 0x004c8048; // Rows
    *((volatile uint32_t*)0x51100208) = 0x00028048; // Columns
    *((volatile uint32_t*)0x51100210) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x51100214) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x51100218) = 0x00000021; // Stride
    *((volatile uint32_t*)0x5110021c) = 0x00000400; // SRAM write ptr
    *((volatile uint32_t*)0x51100224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5110022c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x51100230) = 0x00882ba0; // Layer control
    *((volatile uint32_t*)0x51100234) = 0x001f8000; // Layer control 2
    *((volatile uint32_t*)0x51100238) = 0x000002f8; // Mask count
    *((volatile uint32_t*)0x5110023c) = 0x00000100; // Mask offset
    *((volatile uint32_t*)0x51100240) = 0x0000003f; // Output channel count
    *((volatile uint32_t*)0x51100244) = 0x00000024; // TRAM ptr max
    *((volatile uint32_t*)0x5110024c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x51100248) = 0xffffffff; // Mask and processor enables

    // Layer 2 quadrant 1
    *((volatile uint32_t*)0x52100204) = 0x004c8048; // Rows
    *((volatile uint32_t*)0x52100208) = 0x00028048; // Columns
    *((volatile uint32_t*)0x52100210) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x52100214) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x52100218) = 0x00000021; // Stride
    *((volatile uint32_t*)0x5210021c) = 0x00000400; // SRAM write ptr
    *((volatile uint32_t*)0x52100224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5210022c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x52100230) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x52100234) = 0x001f8000; // Layer control 2
    *((volatile uint32_t*)0x52100238) = 0x000002f8; // Mask count
    *((volatile uint32_t*)0x5210023c) = 0x00000100; // Mask offset
    *((volatile uint32_t*)0x52100240) = 0x0000003f; // Output channel count
    *((volatile uint32_t*)0x52100244) = 0x00000024; // TRAM ptr max
    *((volatile uint32_t*)0x5210024c) = 0x00025000; // Post processing register
    *((volatile uint32_t*)0x52100248) = 0xffffffff; // Mask and processor enables

    // Layer 2 quadrant 2
    *((volatile uint32_t*)0x53100204) = 0x004c8048; // Rows
    *((volatile uint32_t*)0x53100208) = 0x00028048; // Columns
    *((volatile uint32_t*)0x53100210) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x53100214) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x53100218) = 0x00000021; // Stride
    *((volatile uint32_t*)0x5310021c) = 0x00000400; // SRAM write ptr
    *((volatile uint32_t*)0x53100224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5310022c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x53100230) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x53100234) = 0x001f8000; // Layer control 2
    *((volatile uint32_t*)0x53100238) = 0x000002f8; // Mask count
    *((volatile uint32_t*)0x5310023c) = 0x00000100; // Mask offset
    *((volatile uint32_t*)0x53100240) = 0x0000003f; // Output channel count
    *((volatile uint32_t*)0x53100244) = 0x00000024; // TRAM ptr max
    *((volatile uint32_t*)0x5310024c) = 0x00024000; // Post processing register

    // Layer 2 quadrant 3
    *((volatile uint32_t*)0x54100204) = 0x004c8048; // Rows
    *((volatile uint32_t*)0x54100208) = 0x00028048; // Columns
    *((volatile uint32_t*)0x54100210) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x54100214) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x54100218) = 0x00000021; // Stride
    *((volatile uint32_t*)0x5410021c) = 0x00000400; // SRAM write ptr
    *((volatile uint32_t*)0x54100224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5410022c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x54100230) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x54100234) = 0x001f8000; // Layer control 2
    *((volatile uint32_t*)0x54100238) = 0x000002f8; // Mask count
    *((volatile uint32_t*)0x5410023c) = 0x00000100; // Mask offset
    *((volatile uint32_t*)0x54100240) = 0x0000003f; // Output channel count
    *((volatile uint32_t*)0x54100244) = 0x00000024; // TRAM ptr max
    *((volatile uint32_t*)0x5410024c) = 0x00024000; // Post processing register

    // Layer 3 quadrant 0
    *((volatile uint32_t*)0x51100304) = 0x00018024; // Rows
    *((volatile uint32_t*)0x51100308) = 0x00018024; // Columns
    *((volatile uint32_t*)0x51100318) = 0x00000010; // Stride
    *((volatile uint32_t*)0x51100324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5110032c) = 0x00000400; // SRAM read ptr
    *((volatile uint32_t*)0x51100330) = 0x0088eb20; // Layer control
    *((volatile uint32_t*)0x51100334) = 0x001f8000; // Layer control 2
    *((volatile uint32_t*)0x51100338) = 0x000004f8; // Mask count
    *((volatile uint32_t*)0x5110033c) = 0x00000300; // Mask offset
    *((volatile uint32_t*)0x51100340) = 0x0000003f; // Output channel count
    *((volatile uint32_t*)0x51100344) = 0x00000024; // TRAM ptr max
    *((volatile uint32_t*)0x5110034c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x51100348) = 0xffffffff; // Mask and processor enables

    // Layer 3 quadrant 1
    *((volatile uint32_t*)0x52100304) = 0x00018024; // Rows
    *((volatile uint32_t*)0x52100308) = 0x00018024; // Columns
    *((volatile uint32_t*)0x52100318) = 0x00000010; // Stride
    *((volatile uint32_t*)0x52100324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5210032c) = 0x00000400; // SRAM read ptr
    *((volatile uint32_t*)0x52100330) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x52100334) = 0x001f8000; // Layer control 2
    *((volatile uint32_t*)0x52100338) = 0x000004f8; // Mask count
    *((volatile uint32_t*)0x5210033c) = 0x00000300; // Mask offset
    *((volatile uint32_t*)0x52100340) = 0x0000003f; // Output channel count
    *((volatile uint32_t*)0x52100344) = 0x00000024; // TRAM ptr max
    *((volatile uint32_t*)0x5210034c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x52100348) = 0xffffffff; // Mask and processor enables

    // Layer 3 quadrant 2
    *((volatile uint32_t*)0x53100304) = 0x00018024; // Rows
    *((volatile uint32_t*)0x53100308) = 0x00018024; // Columns
    *((volatile uint32_t*)0x53100318) = 0x00000010; // Stride
    *((volatile uint32_t*)0x53100324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5310032c) = 0x00000400; // SRAM read ptr
    *((volatile uint32_t*)0x53100330) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x53100334) = 0x001f8000; // Layer control 2
    *((volatile uint32_t*)0x53100338) = 0x000004f8; // Mask count
    *((volatile uint32_t*)0x5310033c) = 0x00000300; // Mask offset
    *((volatile uint32_t*)0x53100340) = 0x0000003f; // Output channel count
    *((volatile uint32_t*)0x53100344) = 0x00000024; // TRAM ptr max
    *((volatile uint32_t*)0x5310034c) = 0x00025000; // Post processing register
    *((volatile uint32_t*)0x53100348) = 0xffffffff; // Mask and processor enables

    // Layer 3 quadrant 3
    *((volatile uint32_t*)0x54100304) = 0x00018024; // Rows
    *((volatile uint32_t*)0x54100308) = 0x00018024; // Columns
    *((volatile uint32_t*)0x54100318) = 0x00000010; // Stride
    *((volatile uint32_t*)0x54100324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5410032c) = 0x00000400; // SRAM read ptr
    *((volatile uint32_t*)0x54100330) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x54100334) = 0x001f8000; // Layer control 2
    *((volatile uint32_t*)0x54100338) = 0x000004f8; // Mask count
    *((volatile uint32_t*)0x5410033c) = 0x00000300; // Mask offset
    *((volatile uint32_t*)0x54100340) = 0x0000003f; // Output channel count
    *((volatile uint32_t*)0x54100344) = 0x00000024; // TRAM ptr max
    *((volatile uint32_t*)0x5410034c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x54100348) = 0xffffffff; // Mask and processor enables

    // Layer 4 quadrant 0
    *((volatile uint32_t*)0x51100404) = 0x00288022; // Rows
    *((volatile uint32_t*)0x51100408) = 0x00038022; // Columns
    *((volatile uint32_t*)0x51100410) = 0x00000002; // Pooling rows
    *((volatile uint32_t*)0x51100414) = 0x00000002; // Pooling columns
    *((volatile uint32_t*)0x51100418) = 0x00000021; // Stride
    *((volatile uint32_t*)0x5110041c) = 0x00000580; // SRAM write ptr
    *((volatile uint32_t*)0x51100424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x51100430) = 0x0088eba0; // Layer control
    *((volatile uint32_t*)0x51100434) = 0x001f8000; // Layer control 2
    *((volatile uint32_t*)0x51100438) = 0x000006f8; // Mask count
    *((volatile uint32_t*)0x5110043c) = 0x00000500; // Mask offset
    *((volatile uint32_t*)0x51100440) = 0x0000003f; // Output channel count
    *((volatile uint32_t*)0x51100444) = 0x00000011; // TRAM ptr max
    *((volatile uint32_t*)0x5110044c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x51100448) = 0xffffffff; // Mask and processor enables

    // Layer 4 quadrant 1
    *((volatile uint32_t*)0x52100404) = 0x00288022; // Rows
    *((volatile uint32_t*)0x52100408) = 0x00038022; // Columns
    *((volatile uint32_t*)0x52100410) = 0x00000002; // Pooling rows
    *((volatile uint32_t*)0x52100414) = 0x00000002; // Pooling columns
    *((volatile uint32_t*)0x52100418) = 0x00000021; // Stride
    *((volatile uint32_t*)0x5210041c) = 0x00000580; // SRAM write ptr
    *((volatile uint32_t*)0x52100424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x52100430) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x52100434) = 0x001f8000; // Layer control 2
    *((volatile uint32_t*)0x52100438) = 0x000006f8; // Mask count
    *((volatile uint32_t*)0x5210043c) = 0x00000500; // Mask offset
    *((volatile uint32_t*)0x52100440) = 0x0000003f; // Output channel count
    *((volatile uint32_t*)0x52100444) = 0x00000011; // TRAM ptr max
    *((volatile uint32_t*)0x5210044c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x52100448) = 0xffffffff; // Mask and processor enables

    // Layer 4 quadrant 2
    *((volatile uint32_t*)0x53100404) = 0x00288022; // Rows
    *((volatile uint32_t*)0x53100408) = 0x00038022; // Columns
    *((volatile uint32_t*)0x53100410) = 0x00000002; // Pooling rows
    *((volatile uint32_t*)0x53100414) = 0x00000002; // Pooling columns
    *((volatile uint32_t*)0x53100418) = 0x00000021; // Stride
    *((volatile uint32_t*)0x5310041c) = 0x00000580; // SRAM write ptr
    *((volatile uint32_t*)0x53100424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x53100430) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x53100434) = 0x001f8000; // Layer control 2
    *((volatile uint32_t*)0x53100438) = 0x000006f8; // Mask count
    *((volatile uint32_t*)0x5310043c) = 0x00000500; // Mask offset
    *((volatile uint32_t*)0x53100440) = 0x0000003f; // Output channel count
    *((volatile uint32_t*)0x53100444) = 0x00000011; // TRAM ptr max
    *((volatile uint32_t*)0x5310044c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x53100448) = 0xffffffff; // Mask and processor enables

    // Layer 4 quadrant 3
    *((volatile uint32_t*)0x54100404) = 0x00288022; // Rows
    *((volatile uint32_t*)0x54100408) = 0x00038022; // Columns
    *((volatile uint32_t*)0x54100410) = 0x00000002; // Pooling rows
    *((volatile uint32_t*)0x54100414) = 0x00000002; // Pooling columns
    *((volatile uint32_t*)0x54100418) = 0x00000021; // Stride
    *((volatile uint32_t*)0x5410041c) = 0x00000580; // SRAM write ptr
    *((volatile uint32_t*)0x54100424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x54100430) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x54100434) = 0x001f8000; // Layer control 2
    *((volatile uint32_t*)0x54100438) = 0x000006f8; // Mask count
    *((volatile uint32_t*)0x5410043c) = 0x00000500; // Mask offset
    *((volatile uint32_t*)0x54100440) = 0x0000003f; // Output channel count
    *((volatile uint32_t*)0x54100444) = 0x00000011; // TRAM ptr max
    *((volatile uint32_t*)0x5410044c) = 0x00025000; // Post processing register
    *((volatile uint32_t*)0x54100448) = 0xffffffff; // Mask and processor enables

    // Layer 5 quadrant 0
    *((volatile uint32_t*)0x51100504) = 0x00018011; // Rows
    *((volatile uint32_t*)0x51100508) = 0x00018011; // Columns
    *((volatile uint32_t*)0x51100518) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5110051c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t*)0x51100524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5110052c) = 0x00000580; // SRAM read ptr
    *((volatile uint32_t*)0x51100530) = 0x0088eb20; // Layer control
    *((volatile uint32_t*)0x51100534) = 0x001f8000; // Layer control 2
    *((volatile uint32_t*)0x51100538) = 0x000008f8; // Mask count
    *((volatile uint32_t*)0x5110053c) = 0x00000700; // Mask offset
    *((volatile uint32_t*)0x51100540) = 0x0000003f; // Output channel count
    *((volatile uint32_t*)0x51100544) = 0x00000011; // TRAM ptr max
    *((volatile uint32_t*)0x5110054c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x51100548) = 0xffffffff; // Mask and processor enables

    // Layer 5 quadrant 1
    *((volatile uint32_t*)0x52100504) = 0x00018011; // Rows
    *((volatile uint32_t*)0x52100508) = 0x00018011; // Columns
    *((volatile uint32_t*)0x52100518) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5210051c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t*)0x52100524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5210052c) = 0x00000580; // SRAM read ptr
    *((volatile uint32_t*)0x52100530) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x52100534) = 0x001f8000; // Layer control 2
    *((volatile uint32_t*)0x52100538) = 0x000008f8; // Mask count
    *((volatile uint32_t*)0x5210053c) = 0x00000700; // Mask offset
    *((volatile uint32_t*)0x52100540) = 0x0000003f; // Output channel count
    *((volatile uint32_t*)0x52100544) = 0x00000011; // TRAM ptr max
    *((volatile uint32_t*)0x5210054c) = 0x00025040; // Post processing register
    *((volatile uint32_t*)0x52100548) = 0xffffffff; // Mask and processor enables

    // Layer 5 quadrant 2
    *((volatile uint32_t*)0x53100504) = 0x00018011; // Rows
    *((volatile uint32_t*)0x53100508) = 0x00018011; // Columns
    *((volatile uint32_t*)0x53100518) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5310051c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t*)0x53100524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5310052c) = 0x00000580; // SRAM read ptr
    *((volatile uint32_t*)0x53100530) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x53100534) = 0x001f8000; // Layer control 2
    *((volatile uint32_t*)0x53100538) = 0x000008f8; // Mask count
    *((volatile uint32_t*)0x5310053c) = 0x00000700; // Mask offset
    *((volatile uint32_t*)0x53100540) = 0x0000003f; // Output channel count
    *((volatile uint32_t*)0x53100544) = 0x00000011; // TRAM ptr max
    *((volatile uint32_t*)0x5310054c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x53100548) = 0xffffffff; // Mask and processor enables

    // Layer 5 quadrant 3
    *((volatile uint32_t*)0x54100504) = 0x00018011; // Rows
    *((volatile uint32_t*)0x54100508) = 0x00018011; // Columns
    *((volatile uint32_t*)0x54100518) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5410051c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t*)0x54100524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5410052c) = 0x00000580; // SRAM read ptr
    *((volatile uint32_t*)0x54100530) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x54100534) = 0x001f8000; // Layer control 2
    *((volatile uint32_t*)0x54100538) = 0x000008f8; // Mask count
    *((volatile uint32_t*)0x5410053c) = 0x00000700; // Mask offset
    *((volatile uint32_t*)0x54100540) = 0x0000003f; // Output channel count
    *((volatile uint32_t*)0x54100544) = 0x00000011; // TRAM ptr max
    *((volatile uint32_t*)0x5410054c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x54100548) = 0xffffffff; // Mask and processor enables

    // Layer 6 quadrant 0
    *((volatile uint32_t*)0x51100604) = 0x00018011; // Rows
    *((volatile uint32_t*)0x51100608) = 0x00018011; // Columns
    *((volatile uint32_t*)0x51100618) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5110061c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t*)0x51100624) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x51100628) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t*)0x5110062c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x51100630) = 0x0088eb20; // Layer control
    *((volatile uint32_t*)0x51100634) = 0x001f8010; // Layer control 2
    *((volatile uint32_t*)0x51100638) = 0x00000cf8; // Mask count
    *((volatile uint32_t*)0x5110063c) = 0x00000900; // Mask offset
    *((volatile uint32_t*)0x51100640) = 0x0000007f; // Output channel count
    *((volatile uint32_t*)0x51100644) = 0x00000011; // TRAM ptr max
    *((volatile uint32_t*)0x5110064c) = 0x00025000; // Post processing register
    *((volatile uint32_t*)0x51100648) = 0xffffffff; // Mask and processor enables

    // Layer 6 quadrant 1
    *((volatile uint32_t*)0x52100604) = 0x00018011; // Rows
    *((volatile uint32_t*)0x52100608) = 0x00018011; // Columns
    *((volatile uint32_t*)0x52100618) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5210061c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t*)0x52100624) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x52100628) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t*)0x5210062c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x52100630) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x52100634) = 0x001f8010; // Layer control 2
    *((volatile uint32_t*)0x52100638) = 0x00000cf8; // Mask count
    *((volatile uint32_t*)0x5210063c) = 0x00000900; // Mask offset
    *((volatile uint32_t*)0x52100640) = 0x0000007f; // Output channel count
    *((volatile uint32_t*)0x52100644) = 0x00000011; // TRAM ptr max
    *((volatile uint32_t*)0x5210064c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x52100648) = 0xffffffff; // Mask and processor enables

    // Layer 6 quadrant 2
    *((volatile uint32_t*)0x53100604) = 0x00018011; // Rows
    *((volatile uint32_t*)0x53100608) = 0x00018011; // Columns
    *((volatile uint32_t*)0x53100618) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5310061c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t*)0x53100624) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x53100628) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t*)0x5310062c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x53100630) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x53100634) = 0x001f8010; // Layer control 2
    *((volatile uint32_t*)0x53100638) = 0x00000cf8; // Mask count
    *((volatile uint32_t*)0x5310063c) = 0x00000900; // Mask offset
    *((volatile uint32_t*)0x53100640) = 0x0000007f; // Output channel count
    *((volatile uint32_t*)0x53100644) = 0x00000011; // TRAM ptr max
    *((volatile uint32_t*)0x5310064c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x53100648) = 0xffffffff; // Mask and processor enables

    // Layer 6 quadrant 3
    *((volatile uint32_t*)0x54100604) = 0x00018011; // Rows
    *((volatile uint32_t*)0x54100608) = 0x00018011; // Columns
    *((volatile uint32_t*)0x54100618) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5410061c) = 0x00001000; // SRAM write ptr
    *((volatile uint32_t*)0x54100624) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x54100628) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t*)0x5410062c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x54100630) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x54100634) = 0x001f8010; // Layer control 2
    *((volatile uint32_t*)0x54100638) = 0x00000cf8; // Mask count
    *((volatile uint32_t*)0x5410063c) = 0x00000900; // Mask offset
    *((volatile uint32_t*)0x54100640) = 0x0000007f; // Output channel count
    *((volatile uint32_t*)0x54100644) = 0x00000011; // TRAM ptr max
    *((volatile uint32_t*)0x5410064c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x54100648) = 0xffffffff; // Mask and processor enables

    // Layer 7 quadrant 0
    *((volatile uint32_t*)0x51100704) = 0x00028011; // Rows
    *((volatile uint32_t*)0x51100708) = 0x00018011; // Columns
    *((volatile uint32_t*)0x51100718) = 0x00000020; // Stride
    *((volatile uint32_t*)0x5110071c) = 0x00000580; // SRAM write ptr
    *((volatile uint32_t*)0x51100724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5110072c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t*)0x51100730) = 0x0088eb20; // Layer control
    *((volatile uint32_t*)0x51100734) = 0x000f8001; // Layer control 2
    *((volatile uint32_t*)0x51100738) = 0x00000ef8; // Mask count
    *((volatile uint32_t*)0x5110073c) = 0x00000d00; // Mask offset
    *((volatile uint32_t*)0x51100740) = 0x0000003f; // Output channel count
    *((volatile uint32_t*)0x51100744) = 0x00000011; // TRAM ptr max
    *((volatile uint32_t*)0x5110074c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x51100748) = 0xffffffff; // Mask and processor enables

    // Layer 7 quadrant 1
    *((volatile uint32_t*)0x52100704) = 0x00028011; // Rows
    *((volatile uint32_t*)0x52100708) = 0x00018011; // Columns
    *((volatile uint32_t*)0x52100718) = 0x00000020; // Stride
    *((volatile uint32_t*)0x5210071c) = 0x00000580; // SRAM write ptr
    *((volatile uint32_t*)0x52100724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5210072c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t*)0x52100730) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x52100734) = 0x000f8001; // Layer control 2
    *((volatile uint32_t*)0x52100738) = 0x00000ef8; // Mask count
    *((volatile uint32_t*)0x5210073c) = 0x00000d00; // Mask offset
    *((volatile uint32_t*)0x52100740) = 0x0000003f; // Output channel count
    *((volatile uint32_t*)0x52100744) = 0x00000011; // TRAM ptr max
    *((volatile uint32_t*)0x5210074c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x52100748) = 0xffffffff; // Mask and processor enables

    // Layer 7 quadrant 2
    *((volatile uint32_t*)0x53100704) = 0x00028011; // Rows
    *((volatile uint32_t*)0x53100708) = 0x00018011; // Columns
    *((volatile uint32_t*)0x53100718) = 0x00000020; // Stride
    *((volatile uint32_t*)0x5310071c) = 0x00000580; // SRAM write ptr
    *((volatile uint32_t*)0x53100724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5310072c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t*)0x53100730) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x53100734) = 0x000f8001; // Layer control 2
    *((volatile uint32_t*)0x53100738) = 0x00000ef8; // Mask count
    *((volatile uint32_t*)0x5310073c) = 0x00000d00; // Mask offset
    *((volatile uint32_t*)0x53100740) = 0x0000003f; // Output channel count
    *((volatile uint32_t*)0x53100744) = 0x00000011; // TRAM ptr max
    *((volatile uint32_t*)0x5310074c) = 0x00025098; // Post processing register
    *((volatile uint32_t*)0x53100748) = 0xffffffff; // Mask and processor enables

    // Layer 7 quadrant 3
    *((volatile uint32_t*)0x54100704) = 0x00028011; // Rows
    *((volatile uint32_t*)0x54100708) = 0x00018011; // Columns
    *((volatile uint32_t*)0x54100718) = 0x00000020; // Stride
    *((volatile uint32_t*)0x5410071c) = 0x00000580; // SRAM write ptr
    *((volatile uint32_t*)0x54100724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5410072c) = 0x00001000; // SRAM read ptr
    *((volatile uint32_t*)0x54100730) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x54100734) = 0x000f8001; // Layer control 2
    *((volatile uint32_t*)0x54100738) = 0x00000ef8; // Mask count
    *((volatile uint32_t*)0x5410073c) = 0x00000d00; // Mask offset
    *((volatile uint32_t*)0x54100740) = 0x0000003f; // Output channel count
    *((volatile uint32_t*)0x54100744) = 0x00000011; // TRAM ptr max
    *((volatile uint32_t*)0x5410074c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x54100748) = 0xffffffff; // Mask and processor enables

    // Layer 8 quadrant 0
    *((volatile uint32_t*)0x51100804) = 0x00148010; // Rows
    *((volatile uint32_t*)0x51100808) = 0x00028010; // Columns
    *((volatile uint32_t*)0x51100810) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x51100814) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x51100818) = 0x00000021; // Stride
    *((volatile uint32_t*)0x5110081c) = 0x00040700; // SRAM write ptr
    *((volatile uint32_t*)0x51100824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5110082c) = 0x00000580; // SRAM read ptr
    *((volatile uint32_t*)0x51100830) = 0x0088aba0; // Layer control
    *((volatile uint32_t*)0x51100834) = 0x000f8000; // Layer control 2
    *((volatile uint32_t*)0x51100838) = 0x00000ff8; // Mask count
    *((volatile uint32_t*)0x5110083c) = 0x00000f00; // Mask offset
    *((volatile uint32_t*)0x51100840) = 0x0000001f; // Output channel count
    *((volatile uint32_t*)0x51100844) = 0x00000008; // TRAM ptr max
    *((volatile uint32_t*)0x5110084c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x51100848) = 0xffffffff; // Mask and processor enables

    // Layer 8 quadrant 1
    *((volatile uint32_t*)0x52100804) = 0x00148010; // Rows
    *((volatile uint32_t*)0x52100808) = 0x00028010; // Columns
    *((volatile uint32_t*)0x52100810) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x52100814) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x52100818) = 0x00000021; // Stride
    *((volatile uint32_t*)0x5210081c) = 0x00040700; // SRAM write ptr
    *((volatile uint32_t*)0x52100824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5210082c) = 0x00000580; // SRAM read ptr
    *((volatile uint32_t*)0x52100830) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x52100834) = 0x000f8000; // Layer control 2
    *((volatile uint32_t*)0x52100838) = 0x00000ff8; // Mask count
    *((volatile uint32_t*)0x5210083c) = 0x00000f00; // Mask offset
    *((volatile uint32_t*)0x52100840) = 0x0000001f; // Output channel count
    *((volatile uint32_t*)0x52100844) = 0x00000008; // TRAM ptr max
    *((volatile uint32_t*)0x5210084c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x52100848) = 0xffffffff; // Mask and processor enables

    // Layer 8 quadrant 2
    *((volatile uint32_t*)0x53100804) = 0x00148010; // Rows
    *((volatile uint32_t*)0x53100808) = 0x00028010; // Columns
    *((volatile uint32_t*)0x53100810) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x53100814) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x53100818) = 0x00000021; // Stride
    *((volatile uint32_t*)0x5310081c) = 0x00040700; // SRAM write ptr
    *((volatile uint32_t*)0x53100824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5310082c) = 0x00000580; // SRAM read ptr
    *((volatile uint32_t*)0x53100830) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x53100834) = 0x000f8000; // Layer control 2
    *((volatile uint32_t*)0x53100838) = 0x00000ff8; // Mask count
    *((volatile uint32_t*)0x5310083c) = 0x00000f00; // Mask offset
    *((volatile uint32_t*)0x53100840) = 0x0000001f; // Output channel count
    *((volatile uint32_t*)0x53100844) = 0x00000008; // TRAM ptr max
    *((volatile uint32_t*)0x5310084c) = 0x00022000; // Post processing register

    // Layer 8 quadrant 3
    *((volatile uint32_t*)0x54100804) = 0x00148010; // Rows
    *((volatile uint32_t*)0x54100808) = 0x00028010; // Columns
    *((volatile uint32_t*)0x54100810) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x54100814) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x54100818) = 0x00000021; // Stride
    *((volatile uint32_t*)0x5410081c) = 0x00040700; // SRAM write ptr
    *((volatile uint32_t*)0x54100824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5410082c) = 0x00000580; // SRAM read ptr
    *((volatile uint32_t*)0x54100830) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x54100834) = 0x000f8000; // Layer control 2
    *((volatile uint32_t*)0x54100838) = 0x00000ff8; // Mask count
    *((volatile uint32_t*)0x5410083c) = 0x00000f00; // Mask offset
    *((volatile uint32_t*)0x54100840) = 0x0000001f; // Output channel count
    *((volatile uint32_t*)0x54100844) = 0x00000008; // TRAM ptr max
    *((volatile uint32_t*)0x5410084c) = 0x00023098; // Post processing register

    // Layer 9 quadrant 0
    *((volatile uint32_t*)0x51100904) = 0x000c8006; // Rows
    *((volatile uint32_t*)0x51100908) = 0x00038006; // Columns
    *((volatile uint32_t*)0x51100910) = 0x00000002; // Pooling rows
    *((volatile uint32_t*)0x51100914) = 0x00000002; // Pooling columns
    *((volatile uint32_t*)0x51100918) = 0x00000021; // Stride
    *((volatile uint32_t*)0x5110091c) = 0x00000780; // SRAM write ptr
    *((volatile uint32_t*)0x51100924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5110092c) = 0x00000700; // SRAM read ptr
    *((volatile uint32_t*)0x51100930) = 0x0088cba0; // Layer control
    *((volatile uint32_t*)0x51100934) = 0x000f8000; // Layer control 2
    *((volatile uint32_t*)0x51100938) = 0x000001f8; // Mask count
    *((volatile uint32_t*)0x5110093c) = 0x00000100; // Mask offset
    *((volatile uint32_t*)0x51100940) = 0x0000001f; // Output channel count
    *((volatile uint32_t*)0x51100944) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t*)0x5110094c) = 0x000230a0; // Post processing register

    // Layer 9 quadrant 1
    *((volatile uint32_t*)0x52100904) = 0x000c8006; // Rows
    *((volatile uint32_t*)0x52100908) = 0x00038006; // Columns
    *((volatile uint32_t*)0x52100910) = 0x00000002; // Pooling rows
    *((volatile uint32_t*)0x52100914) = 0x00000002; // Pooling columns
    *((volatile uint32_t*)0x52100918) = 0x00000021; // Stride
    *((volatile uint32_t*)0x5210091c) = 0x00000780; // SRAM write ptr
    *((volatile uint32_t*)0x52100924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5210092c) = 0x00000700; // SRAM read ptr
    *((volatile uint32_t*)0x52100930) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x52100934) = 0x000f8000; // Layer control 2
    *((volatile uint32_t*)0x52100938) = 0x000001f8; // Mask count
    *((volatile uint32_t*)0x5210093c) = 0x00000100; // Mask offset
    *((volatile uint32_t*)0x52100940) = 0x0000001f; // Output channel count
    *((volatile uint32_t*)0x52100944) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t*)0x5210094c) = 0x00022000; // Post processing register

    // Layer 9 quadrant 2
    *((volatile uint32_t*)0x53100904) = 0x000c8006; // Rows
    *((volatile uint32_t*)0x53100908) = 0x00038006; // Columns
    *((volatile uint32_t*)0x53100910) = 0x00000002; // Pooling rows
    *((volatile uint32_t*)0x53100914) = 0x00000002; // Pooling columns
    *((volatile uint32_t*)0x53100918) = 0x00000021; // Stride
    *((volatile uint32_t*)0x5310091c) = 0x00000780; // SRAM write ptr
    *((volatile uint32_t*)0x53100924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5310092c) = 0x00000700; // SRAM read ptr
    *((volatile uint32_t*)0x53100930) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x53100934) = 0x000f8000; // Layer control 2
    *((volatile uint32_t*)0x53100938) = 0x000001f8; // Mask count
    *((volatile uint32_t*)0x5310093c) = 0x00000100; // Mask offset
    *((volatile uint32_t*)0x53100940) = 0x0000001f; // Output channel count
    *((volatile uint32_t*)0x53100944) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t*)0x5310094c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x53100948) = 0xffffffff; // Mask and processor enables

    // Layer 9 quadrant 3
    *((volatile uint32_t*)0x54100904) = 0x000c8006; // Rows
    *((volatile uint32_t*)0x54100908) = 0x00038006; // Columns
    *((volatile uint32_t*)0x54100910) = 0x00000002; // Pooling rows
    *((volatile uint32_t*)0x54100914) = 0x00000002; // Pooling columns
    *((volatile uint32_t*)0x54100918) = 0x00000021; // Stride
    *((volatile uint32_t*)0x5410091c) = 0x00000780; // SRAM write ptr
    *((volatile uint32_t*)0x54100924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5410092c) = 0x00000700; // SRAM read ptr
    *((volatile uint32_t*)0x54100930) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x54100934) = 0x000f8000; // Layer control 2
    *((volatile uint32_t*)0x54100938) = 0x000001f8; // Mask count
    *((volatile uint32_t*)0x5410093c) = 0x00000100; // Mask offset
    *((volatile uint32_t*)0x54100940) = 0x0000001f; // Output channel count
    *((volatile uint32_t*)0x54100944) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t*)0x5410094c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x54100948) = 0xffffffff; // Mask and processor enables

    // Layer 10 quadrant 0
    *((volatile uint32_t*)0x51100a04) = 0x00018003; // Rows
    *((volatile uint32_t*)0x51100a08) = 0x00018003; // Columns
    *((volatile uint32_t*)0x51100a18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x51100a1c) = 0x000007c0; // SRAM write ptr
    *((volatile uint32_t*)0x51100a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x51100a2c) = 0x00000780; // SRAM read ptr
    *((volatile uint32_t*)0x51100a30) = 0x00882b20; // Layer control
    *((volatile uint32_t*)0x51100a34) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x51100a38) = 0x00001078; // Mask count
    *((volatile uint32_t*)0x51100a3c) = 0x00001000; // Mask offset
    *((volatile uint32_t*)0x51100a40) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x51100a44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t*)0x51100a4c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x51100a48) = 0xffffffff; // Mask and processor enables

    // Layer 10 quadrant 1
    *((volatile uint32_t*)0x52100a04) = 0x00018003; // Rows
    *((volatile uint32_t*)0x52100a08) = 0x00018003; // Columns
    *((volatile uint32_t*)0x52100a18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x52100a1c) = 0x000007c0; // SRAM write ptr
    *((volatile uint32_t*)0x52100a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x52100a2c) = 0x00000780; // SRAM read ptr
    *((volatile uint32_t*)0x52100a30) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x52100a34) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x52100a38) = 0x00001078; // Mask count
    *((volatile uint32_t*)0x52100a3c) = 0x00001000; // Mask offset
    *((volatile uint32_t*)0x52100a40) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x52100a44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t*)0x52100a4c) = 0x000250a0; // Post processing register
    *((volatile uint32_t*)0x52100a48) = 0xffffffff; // Mask and processor enables

    // Layer 10 quadrant 2
    *((volatile uint32_t*)0x53100a04) = 0x00018003; // Rows
    *((volatile uint32_t*)0x53100a08) = 0x00018003; // Columns
    *((volatile uint32_t*)0x53100a18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x53100a1c) = 0x000007c0; // SRAM write ptr
    *((volatile uint32_t*)0x53100a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x53100a2c) = 0x00000780; // SRAM read ptr
    *((volatile uint32_t*)0x53100a30) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x53100a34) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x53100a38) = 0x00001078; // Mask count
    *((volatile uint32_t*)0x53100a3c) = 0x00001000; // Mask offset
    *((volatile uint32_t*)0x53100a40) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x53100a44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t*)0x53100a4c) = 0x00024000; // Post processing register

    // Layer 10 quadrant 3
    *((volatile uint32_t*)0x54100a04) = 0x00018003; // Rows
    *((volatile uint32_t*)0x54100a08) = 0x00018003; // Columns
    *((volatile uint32_t*)0x54100a18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x54100a1c) = 0x000007c0; // SRAM write ptr
    *((volatile uint32_t*)0x54100a24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x54100a2c) = 0x00000780; // SRAM read ptr
    *((volatile uint32_t*)0x54100a30) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x54100a34) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x54100a38) = 0x00001078; // Mask count
    *((volatile uint32_t*)0x54100a3c) = 0x00001000; // Mask offset
    *((volatile uint32_t*)0x54100a40) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x54100a44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t*)0x54100a4c) = 0x00024000; // Post processing register

    // Layer 11 quadrant 0
    *((volatile uint32_t*)0x51100b04) = 0x00068002; // Rows
    *((volatile uint32_t*)0x51100b08) = 0x00028002; // Columns
    *((volatile uint32_t*)0x51100b10) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x51100b14) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x51100b18) = 0x00000021; // Stride
    *((volatile uint32_t*)0x51100b1c) = 0x00020800; // SRAM write ptr
    *((volatile uint32_t*)0x51100b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x51100b2c) = 0x000007c0; // SRAM read ptr
    *((volatile uint32_t*)0x51100b30) = 0x00882ba0; // Layer control
    *((volatile uint32_t*)0x51100b34) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x51100b38) = 0x000010f8; // Mask count
    *((volatile uint32_t*)0x51100b3c) = 0x00001080; // Mask offset
    *((volatile uint32_t*)0x51100b40) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x51100b44) = 0x00000001; // TRAM ptr max
    *((volatile uint32_t*)0x51100b4c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x51100b48) = 0xffffffff; // Mask and processor enables

    // Layer 11 quadrant 1
    *((volatile uint32_t*)0x52100b04) = 0x00068002; // Rows
    *((volatile uint32_t*)0x52100b08) = 0x00028002; // Columns
    *((volatile uint32_t*)0x52100b10) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x52100b14) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x52100b18) = 0x00000021; // Stride
    *((volatile uint32_t*)0x52100b1c) = 0x00020800; // SRAM write ptr
    *((volatile uint32_t*)0x52100b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x52100b2c) = 0x000007c0; // SRAM read ptr
    *((volatile uint32_t*)0x52100b30) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x52100b34) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x52100b38) = 0x000010f8; // Mask count
    *((volatile uint32_t*)0x52100b3c) = 0x00001080; // Mask offset
    *((volatile uint32_t*)0x52100b40) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x52100b44) = 0x00000001; // TRAM ptr max
    *((volatile uint32_t*)0x52100b4c) = 0x000230b0; // Post processing register

    // Layer 11 quadrant 2
    *((volatile uint32_t*)0x53100b04) = 0x00068002; // Rows
    *((volatile uint32_t*)0x53100b08) = 0x00028002; // Columns
    *((volatile uint32_t*)0x53100b10) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x53100b14) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x53100b18) = 0x00000021; // Stride
    *((volatile uint32_t*)0x53100b1c) = 0x00020800; // SRAM write ptr
    *((volatile uint32_t*)0x53100b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x53100b2c) = 0x000007c0; // SRAM read ptr
    *((volatile uint32_t*)0x53100b30) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x53100b34) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x53100b38) = 0x000010f8; // Mask count
    *((volatile uint32_t*)0x53100b3c) = 0x00001080; // Mask offset
    *((volatile uint32_t*)0x53100b40) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x53100b44) = 0x00000001; // TRAM ptr max
    *((volatile uint32_t*)0x53100b4c) = 0x00022000; // Post processing register

    // Layer 11 quadrant 3
    *((volatile uint32_t*)0x54100b04) = 0x00068002; // Rows
    *((volatile uint32_t*)0x54100b08) = 0x00028002; // Columns
    *((volatile uint32_t*)0x54100b10) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x54100b14) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x54100b18) = 0x00000021; // Stride
    *((volatile uint32_t*)0x54100b1c) = 0x00020800; // SRAM write ptr
    *((volatile uint32_t*)0x54100b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x54100b2c) = 0x000007c0; // SRAM read ptr
    *((volatile uint32_t*)0x54100b30) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x54100b34) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x54100b38) = 0x000010f8; // Mask count
    *((volatile uint32_t*)0x54100b3c) = 0x00001080; // Mask offset
    *((volatile uint32_t*)0x54100b40) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x54100b44) = 0x00000001; // TRAM ptr max
    *((volatile uint32_t*)0x54100b4c) = 0x00022000; // Post processing register

    // Layer 12 quadrant 0
    *((volatile uint32_t*)0x51100c04) = 0x00018011; // Rows
    *((volatile uint32_t*)0x51100c08) = 0x00018011; // Columns
    *((volatile uint32_t*)0x51100c18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x51100c1c) = 0x00000c00; // SRAM write ptr
    *((volatile uint32_t*)0x51100c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x51100c2c) = 0x00000580; // SRAM read ptr
    *((volatile uint32_t*)0x51100c30) = 0x00886920; // Layer control
    *((volatile uint32_t*)0x51100c34) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x51100c38) = 0x00001178; // Mask count
    *((volatile uint32_t*)0x51100c3c) = 0x00001100; // Mask offset
    *((volatile uint32_t*)0x51100c40) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x51100c44) = 0x00000011; // TRAM ptr max
    *((volatile uint32_t*)0x51100c4c) = 0x00028000; // Post processing register
    *((volatile uint32_t*)0x51100c48) = 0xffffffff; // Mask and processor enables

    // Layer 12 quadrant 1
    *((volatile uint32_t*)0x52100c04) = 0x00018011; // Rows
    *((volatile uint32_t*)0x52100c08) = 0x00018011; // Columns
    *((volatile uint32_t*)0x52100c18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x52100c1c) = 0x00000c00; // SRAM write ptr
    *((volatile uint32_t*)0x52100c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x52100c2c) = 0x00000580; // SRAM read ptr
    *((volatile uint32_t*)0x52100c30) = 0x00880920; // Layer control
    *((volatile uint32_t*)0x52100c34) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x52100c38) = 0x00001178; // Mask count
    *((volatile uint32_t*)0x52100c3c) = 0x00001100; // Mask offset
    *((volatile uint32_t*)0x52100c40) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x52100c44) = 0x00000011; // TRAM ptr max
    *((volatile uint32_t*)0x52100c4c) = 0x00028000; // Post processing register
    *((volatile uint32_t*)0x52100c48) = 0xffffffff; // Mask and processor enables

    // Layer 12 quadrant 2
    *((volatile uint32_t*)0x53100c04) = 0x00018011; // Rows
    *((volatile uint32_t*)0x53100c08) = 0x00018011; // Columns
    *((volatile uint32_t*)0x53100c18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x53100c1c) = 0x00000c00; // SRAM write ptr
    *((volatile uint32_t*)0x53100c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x53100c2c) = 0x00000580; // SRAM read ptr
    *((volatile uint32_t*)0x53100c30) = 0x00880920; // Layer control
    *((volatile uint32_t*)0x53100c34) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x53100c38) = 0x00001178; // Mask count
    *((volatile uint32_t*)0x53100c3c) = 0x00001100; // Mask offset
    *((volatile uint32_t*)0x53100c40) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x53100c44) = 0x00000011; // TRAM ptr max
    *((volatile uint32_t*)0x53100c4c) = 0x000290b8; // Post processing register

    // Layer 12 quadrant 3
    *((volatile uint32_t*)0x54100c04) = 0x00018011; // Rows
    *((volatile uint32_t*)0x54100c08) = 0x00018011; // Columns
    *((volatile uint32_t*)0x54100c18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x54100c1c) = 0x00000c00; // SRAM write ptr
    *((volatile uint32_t*)0x54100c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x54100c2c) = 0x00000580; // SRAM read ptr
    *((volatile uint32_t*)0x54100c30) = 0x00880920; // Layer control
    *((volatile uint32_t*)0x54100c34) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x54100c38) = 0x00001178; // Mask count
    *((volatile uint32_t*)0x54100c3c) = 0x00001100; // Mask offset
    *((volatile uint32_t*)0x54100c40) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x54100c44) = 0x00000011; // TRAM ptr max
    *((volatile uint32_t*)0x54100c4c) = 0x00028000; // Post processing register

    // Layer 13 quadrant 0
    *((volatile uint32_t*)0x51100d04) = 0x00018008; // Rows
    *((volatile uint32_t*)0x51100d08) = 0x00018008; // Columns
    *((volatile uint32_t*)0x51100d18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x51100d1c) = 0x00000d44; // SRAM write ptr
    *((volatile uint32_t*)0x51100d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x51100d2c) = 0x00000700; // SRAM read ptr
    *((volatile uint32_t*)0x51100d30) = 0x0088c920; // Layer control
    *((volatile uint32_t*)0x51100d34) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x51100d38) = 0x00000278; // Mask count
    *((volatile uint32_t*)0x51100d3c) = 0x00000200; // Mask offset
    *((volatile uint32_t*)0x51100d40) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x51100d44) = 0x00000008; // TRAM ptr max
    *((volatile uint32_t*)0x51100d4c) = 0x00022000; // Post processing register

    // Layer 13 quadrant 1
    *((volatile uint32_t*)0x52100d04) = 0x00018008; // Rows
    *((volatile uint32_t*)0x52100d08) = 0x00018008; // Columns
    *((volatile uint32_t*)0x52100d18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x52100d1c) = 0x00000d44; // SRAM write ptr
    *((volatile uint32_t*)0x52100d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x52100d2c) = 0x00000700; // SRAM read ptr
    *((volatile uint32_t*)0x52100d30) = 0x00880920; // Layer control
    *((volatile uint32_t*)0x52100d34) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x52100d38) = 0x00000278; // Mask count
    *((volatile uint32_t*)0x52100d3c) = 0x00000200; // Mask offset
    *((volatile uint32_t*)0x52100d40) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x52100d44) = 0x00000008; // TRAM ptr max
    *((volatile uint32_t*)0x52100d4c) = 0x00022000; // Post processing register

    // Layer 13 quadrant 2
    *((volatile uint32_t*)0x53100d04) = 0x00018008; // Rows
    *((volatile uint32_t*)0x53100d08) = 0x00018008; // Columns
    *((volatile uint32_t*)0x53100d18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x53100d1c) = 0x00000d44; // SRAM write ptr
    *((volatile uint32_t*)0x53100d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x53100d2c) = 0x00000700; // SRAM read ptr
    *((volatile uint32_t*)0x53100d30) = 0x00880920; // Layer control
    *((volatile uint32_t*)0x53100d34) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x53100d38) = 0x00000278; // Mask count
    *((volatile uint32_t*)0x53100d3c) = 0x00000200; // Mask offset
    *((volatile uint32_t*)0x53100d40) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x53100d44) = 0x00000008; // TRAM ptr max
    *((volatile uint32_t*)0x53100d4c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x53100d48) = 0xffffffff; // Mask and processor enables

    // Layer 13 quadrant 3
    *((volatile uint32_t*)0x54100d04) = 0x00018008; // Rows
    *((volatile uint32_t*)0x54100d08) = 0x00018008; // Columns
    *((volatile uint32_t*)0x54100d18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x54100d1c) = 0x00000d44; // SRAM write ptr
    *((volatile uint32_t*)0x54100d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x54100d2c) = 0x00000700; // SRAM read ptr
    *((volatile uint32_t*)0x54100d30) = 0x00880920; // Layer control
    *((volatile uint32_t*)0x54100d34) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x54100d38) = 0x00000278; // Mask count
    *((volatile uint32_t*)0x54100d3c) = 0x00000200; // Mask offset
    *((volatile uint32_t*)0x54100d40) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x54100d44) = 0x00000008; // TRAM ptr max
    *((volatile uint32_t*)0x54100d4c) = 0x000230b8; // Post processing register
    *((volatile uint32_t*)0x54100d48) = 0xffffffff; // Mask and processor enables

    // Layer 14 quadrant 0
    *((volatile uint32_t*)0x51100e04) = 0x00018003; // Rows
    *((volatile uint32_t*)0x51100e08) = 0x00018003; // Columns
    *((volatile uint32_t*)0x51100e18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x51100e1c) = 0x00000d95; // SRAM write ptr
    *((volatile uint32_t*)0x51100e24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x51100e2c) = 0x00000780; // SRAM read ptr
    *((volatile uint32_t*)0x51100e30) = 0x00882920; // Layer control
    *((volatile uint32_t*)0x51100e34) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x51100e38) = 0x000011f8; // Mask count
    *((volatile uint32_t*)0x51100e3c) = 0x00001180; // Mask offset
    *((volatile uint32_t*)0x51100e40) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x51100e44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t*)0x51100e4c) = 0x000230c0; // Post processing register
    *((volatile uint32_t*)0x51100e48) = 0xffffffff; // Mask and processor enables

    // Layer 14 quadrant 1
    *((volatile uint32_t*)0x52100e04) = 0x00018003; // Rows
    *((volatile uint32_t*)0x52100e08) = 0x00018003; // Columns
    *((volatile uint32_t*)0x52100e18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x52100e1c) = 0x00000d95; // SRAM write ptr
    *((volatile uint32_t*)0x52100e24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x52100e2c) = 0x00000780; // SRAM read ptr
    *((volatile uint32_t*)0x52100e30) = 0x00880920; // Layer control
    *((volatile uint32_t*)0x52100e34) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x52100e38) = 0x000011f8; // Mask count
    *((volatile uint32_t*)0x52100e3c) = 0x00001180; // Mask offset
    *((volatile uint32_t*)0x52100e40) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x52100e44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t*)0x52100e4c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x52100e48) = 0xffffffff; // Mask and processor enables

    // Layer 14 quadrant 2
    *((volatile uint32_t*)0x53100e04) = 0x00018003; // Rows
    *((volatile uint32_t*)0x53100e08) = 0x00018003; // Columns
    *((volatile uint32_t*)0x53100e18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x53100e1c) = 0x00000d95; // SRAM write ptr
    *((volatile uint32_t*)0x53100e24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x53100e2c) = 0x00000780; // SRAM read ptr
    *((volatile uint32_t*)0x53100e30) = 0x00880920; // Layer control
    *((volatile uint32_t*)0x53100e34) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x53100e38) = 0x000011f8; // Mask count
    *((volatile uint32_t*)0x53100e3c) = 0x00001180; // Mask offset
    *((volatile uint32_t*)0x53100e40) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x53100e44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t*)0x53100e4c) = 0x00022000; // Post processing register

    // Layer 14 quadrant 3
    *((volatile uint32_t*)0x54100e04) = 0x00018003; // Rows
    *((volatile uint32_t*)0x54100e08) = 0x00018003; // Columns
    *((volatile uint32_t*)0x54100e18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x54100e1c) = 0x00000d95; // SRAM write ptr
    *((volatile uint32_t*)0x54100e24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x54100e2c) = 0x00000780; // SRAM read ptr
    *((volatile uint32_t*)0x54100e30) = 0x00880920; // Layer control
    *((volatile uint32_t*)0x54100e34) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x54100e38) = 0x000011f8; // Mask count
    *((volatile uint32_t*)0x54100e3c) = 0x00001180; // Mask offset
    *((volatile uint32_t*)0x54100e40) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x54100e44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t*)0x54100e4c) = 0x00022000; // Post processing register

    // Layer 15 quadrant 0
    *((volatile uint32_t*)0x51100f04) = 0x00018001; // Rows
    *((volatile uint32_t*)0x51100f08) = 0x00018001; // Columns
    *((volatile uint32_t*)0x51100f18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x51100f1c) = 0x00000da5; // SRAM write ptr
    *((volatile uint32_t*)0x51100f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x51100f2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x51100f30) = 0x00882920; // Layer control
    *((volatile uint32_t*)0x51100f34) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x51100f38) = 0x00000078; // Mask count
    *((volatile uint32_t*)0x51100f40) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x51100f44) = 0x00000001; // TRAM ptr max
    *((volatile uint32_t*)0x51100f4c) = 0x00022000; // Post processing register

    // Layer 15 quadrant 1
    *((volatile uint32_t*)0x52100f04) = 0x00018001; // Rows
    *((volatile uint32_t*)0x52100f08) = 0x00018001; // Columns
    *((volatile uint32_t*)0x52100f18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x52100f1c) = 0x00000da5; // SRAM write ptr
    *((volatile uint32_t*)0x52100f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x52100f2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x52100f30) = 0x00880920; // Layer control
    *((volatile uint32_t*)0x52100f34) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x52100f38) = 0x00000078; // Mask count
    *((volatile uint32_t*)0x52100f40) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x52100f44) = 0x00000001; // TRAM ptr max
    *((volatile uint32_t*)0x52100f4c) = 0x000230c0; // Post processing register
    *((volatile uint32_t*)0x52100f48) = 0xffffffff; // Mask and processor enables

    // Layer 15 quadrant 2
    *((volatile uint32_t*)0x53100f04) = 0x00018001; // Rows
    *((volatile uint32_t*)0x53100f08) = 0x00018001; // Columns
    *((volatile uint32_t*)0x53100f18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x53100f1c) = 0x00000da5; // SRAM write ptr
    *((volatile uint32_t*)0x53100f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x53100f2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x53100f30) = 0x00880920; // Layer control
    *((volatile uint32_t*)0x53100f34) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x53100f38) = 0x00000078; // Mask count
    *((volatile uint32_t*)0x53100f40) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x53100f44) = 0x00000001; // TRAM ptr max
    *((volatile uint32_t*)0x53100f4c) = 0x00022000; // Post processing register

    // Layer 15 quadrant 3
    *((volatile uint32_t*)0x54100f04) = 0x00018001; // Rows
    *((volatile uint32_t*)0x54100f08) = 0x00018001; // Columns
    *((volatile uint32_t*)0x54100f18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x54100f1c) = 0x00000da5; // SRAM write ptr
    *((volatile uint32_t*)0x54100f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x54100f2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x54100f30) = 0x00880920; // Layer control
    *((volatile uint32_t*)0x54100f34) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x54100f38) = 0x00000078; // Mask count
    *((volatile uint32_t*)0x54100f40) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x54100f44) = 0x00000001; // TRAM ptr max
    *((volatile uint32_t*)0x54100f4c) = 0x00022000; // Post processing register

    // Layer 16 quadrant 0
    *((volatile uint32_t*)0x51101004) = 0x00018011; // Rows
    *((volatile uint32_t*)0x51101008) = 0x00018011; // Columns
    *((volatile uint32_t*)0x51101018) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5110101c) = 0x00028c00; // SRAM write ptr
    *((volatile uint32_t*)0x51101024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5110102c) = 0x00000580; // SRAM read ptr
    *((volatile uint32_t*)0x51101030) = 0x00886920; // Layer control
    *((volatile uint32_t*)0x51101034) = 0x00158000; // Layer control 2
    *((volatile uint32_t*)0x51101038) = 0x00001358; // Mask count
    *((volatile uint32_t*)0x5110103c) = 0x00001200; // Mask offset
    *((volatile uint32_t*)0x51101040) = 0x0000002b; // Output channel count
    *((volatile uint32_t*)0x51101044) = 0x00000011; // TRAM ptr max
    *((volatile uint32_t*)0x5110104c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x51101048) = 0xffffffff; // Mask and processor enables

    // Layer 16 quadrant 1
    *((volatile uint32_t*)0x52101004) = 0x00018011; // Rows
    *((volatile uint32_t*)0x52101008) = 0x00018011; // Columns
    *((volatile uint32_t*)0x52101018) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5210101c) = 0x00028c00; // SRAM write ptr
    *((volatile uint32_t*)0x52101024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5210102c) = 0x00000580; // SRAM read ptr
    *((volatile uint32_t*)0x52101030) = 0x00880920; // Layer control
    *((volatile uint32_t*)0x52101034) = 0x00158000; // Layer control 2
    *((volatile uint32_t*)0x52101038) = 0x00001358; // Mask count
    *((volatile uint32_t*)0x5210103c) = 0x00001200; // Mask offset
    *((volatile uint32_t*)0x52101040) = 0x0000002b; // Output channel count
    *((volatile uint32_t*)0x52101044) = 0x00000011; // TRAM ptr max
    *((volatile uint32_t*)0x5210104c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x52101048) = 0xffffffff; // Mask and processor enables

    // Layer 16 quadrant 2
    *((volatile uint32_t*)0x53101004) = 0x00018011; // Rows
    *((volatile uint32_t*)0x53101008) = 0x00018011; // Columns
    *((volatile uint32_t*)0x53101018) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5310101c) = 0x00028c00; // SRAM write ptr
    *((volatile uint32_t*)0x53101024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5310102c) = 0x00000580; // SRAM read ptr
    *((volatile uint32_t*)0x53101030) = 0x00880920; // Layer control
    *((volatile uint32_t*)0x53101034) = 0x00158000; // Layer control 2
    *((volatile uint32_t*)0x53101038) = 0x00001358; // Mask count
    *((volatile uint32_t*)0x5310103c) = 0x00001200; // Mask offset
    *((volatile uint32_t*)0x53101040) = 0x0000002b; // Output channel count
    *((volatile uint32_t*)0x53101044) = 0x00000011; // TRAM ptr max
    *((volatile uint32_t*)0x5310104c) = 0x00025040; // Post processing register

    // Layer 16 quadrant 3
    *((volatile uint32_t*)0x54101004) = 0x00018011; // Rows
    *((volatile uint32_t*)0x54101008) = 0x00018011; // Columns
    *((volatile uint32_t*)0x54101018) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5410101c) = 0x00028c00; // SRAM write ptr
    *((volatile uint32_t*)0x54101024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5410102c) = 0x00000580; // SRAM read ptr
    *((volatile uint32_t*)0x54101030) = 0x00880920; // Layer control
    *((volatile uint32_t*)0x54101034) = 0x00158000; // Layer control 2
    *((volatile uint32_t*)0x54101038) = 0x00001358; // Mask count
    *((volatile uint32_t*)0x5410103c) = 0x00001200; // Mask offset
    *((volatile uint32_t*)0x54101040) = 0x0000002b; // Output channel count
    *((volatile uint32_t*)0x54101044) = 0x00000011; // TRAM ptr max
    *((volatile uint32_t*)0x5410104c) = 0x00024000; // Post processing register

    // Layer 17 quadrant 0
    *((volatile uint32_t*)0x51101104) = 0x00018008; // Rows
    *((volatile uint32_t*)0x51101108) = 0x00018008; // Columns
    *((volatile uint32_t*)0x51101118) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5110111c) = 0x00028d44; // SRAM write ptr
    *((volatile uint32_t*)0x51101124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5110112c) = 0x00000700; // SRAM read ptr
    *((volatile uint32_t*)0x51101130) = 0x0088c920; // Layer control
    *((volatile uint32_t*)0x51101134) = 0x00158000; // Layer control 2
    *((volatile uint32_t*)0x51101138) = 0x00001058; // Mask count
    *((volatile uint32_t*)0x5110113c) = 0x00000f00; // Mask offset
    *((volatile uint32_t*)0x51101140) = 0x0000002b; // Output channel count
    *((volatile uint32_t*)0x51101144) = 0x00000008; // TRAM ptr max
    *((volatile uint32_t*)0x5110114c) = 0x00022000; // Post processing register

    // Layer 17 quadrant 1
    *((volatile uint32_t*)0x52101104) = 0x00018008; // Rows
    *((volatile uint32_t*)0x52101108) = 0x00018008; // Columns
    *((volatile uint32_t*)0x52101118) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5210111c) = 0x00028d44; // SRAM write ptr
    *((volatile uint32_t*)0x52101124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5210112c) = 0x00000700; // SRAM read ptr
    *((volatile uint32_t*)0x52101130) = 0x00880920; // Layer control
    *((volatile uint32_t*)0x52101134) = 0x00158000; // Layer control 2
    *((volatile uint32_t*)0x52101138) = 0x00001058; // Mask count
    *((volatile uint32_t*)0x5210113c) = 0x00000f00; // Mask offset
    *((volatile uint32_t*)0x52101140) = 0x0000002b; // Output channel count
    *((volatile uint32_t*)0x52101144) = 0x00000008; // TRAM ptr max
    *((volatile uint32_t*)0x5210114c) = 0x00022000; // Post processing register

    // Layer 17 quadrant 2
    *((volatile uint32_t*)0x53101104) = 0x00018008; // Rows
    *((volatile uint32_t*)0x53101108) = 0x00018008; // Columns
    *((volatile uint32_t*)0x53101118) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5310111c) = 0x00028d44; // SRAM write ptr
    *((volatile uint32_t*)0x53101124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5310112c) = 0x00000700; // SRAM read ptr
    *((volatile uint32_t*)0x53101130) = 0x00880920; // Layer control
    *((volatile uint32_t*)0x53101134) = 0x00158000; // Layer control 2
    *((volatile uint32_t*)0x53101138) = 0x00001058; // Mask count
    *((volatile uint32_t*)0x5310113c) = 0x00000f00; // Mask offset
    *((volatile uint32_t*)0x53101140) = 0x0000002b; // Output channel count
    *((volatile uint32_t*)0x53101144) = 0x00000008; // TRAM ptr max
    *((volatile uint32_t*)0x5310114c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x53101148) = 0xffffffff; // Mask and processor enables

    // Layer 17 quadrant 3
    *((volatile uint32_t*)0x54101104) = 0x00018008; // Rows
    *((volatile uint32_t*)0x54101108) = 0x00018008; // Columns
    *((volatile uint32_t*)0x54101118) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5410111c) = 0x00028d44; // SRAM write ptr
    *((volatile uint32_t*)0x54101124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5410112c) = 0x00000700; // SRAM read ptr
    *((volatile uint32_t*)0x54101130) = 0x00880920; // Layer control
    *((volatile uint32_t*)0x54101134) = 0x00158000; // Layer control 2
    *((volatile uint32_t*)0x54101138) = 0x00001058; // Mask count
    *((volatile uint32_t*)0x5410113c) = 0x00000f00; // Mask offset
    *((volatile uint32_t*)0x54101140) = 0x0000002b; // Output channel count
    *((volatile uint32_t*)0x54101144) = 0x00000008; // TRAM ptr max
    *((volatile uint32_t*)0x5410114c) = 0x00023040; // Post processing register
    *((volatile uint32_t*)0x54101148) = 0xffffffff; // Mask and processor enables

    // Layer 18 quadrant 0
    *((volatile uint32_t*)0x51101204) = 0x00018003; // Rows
    *((volatile uint32_t*)0x51101208) = 0x00018003; // Columns
    *((volatile uint32_t*)0x51101218) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5110121c) = 0x00028d95; // SRAM write ptr
    *((volatile uint32_t*)0x51101224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5110122c) = 0x00000780; // SRAM read ptr
    *((volatile uint32_t*)0x51101230) = 0x00886920; // Layer control
    *((volatile uint32_t*)0x51101234) = 0x00158000; // Layer control 2
    *((volatile uint32_t*)0x51101238) = 0x000014b8; // Mask count
    *((volatile uint32_t*)0x5110123c) = 0x00001360; // Mask offset
    *((volatile uint32_t*)0x51101240) = 0x0000002b; // Output channel count
    *((volatile uint32_t*)0x51101244) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t*)0x5110124c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x51101248) = 0xffffffff; // Mask and processor enables

    // Layer 18 quadrant 1
    *((volatile uint32_t*)0x52101204) = 0x00018003; // Rows
    *((volatile uint32_t*)0x52101208) = 0x00018003; // Columns
    *((volatile uint32_t*)0x52101218) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5210121c) = 0x00028d95; // SRAM write ptr
    *((volatile uint32_t*)0x52101224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5210122c) = 0x00000780; // SRAM read ptr
    *((volatile uint32_t*)0x52101230) = 0x00880920; // Layer control
    *((volatile uint32_t*)0x52101234) = 0x00158000; // Layer control 2
    *((volatile uint32_t*)0x52101238) = 0x000014b8; // Mask count
    *((volatile uint32_t*)0x5210123c) = 0x00001360; // Mask offset
    *((volatile uint32_t*)0x52101240) = 0x0000002b; // Output channel count
    *((volatile uint32_t*)0x52101244) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t*)0x5210124c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x52101248) = 0xffffffff; // Mask and processor enables

    // Layer 18 quadrant 2
    *((volatile uint32_t*)0x53101204) = 0x00018003; // Rows
    *((volatile uint32_t*)0x53101208) = 0x00018003; // Columns
    *((volatile uint32_t*)0x53101218) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5310121c) = 0x00028d95; // SRAM write ptr
    *((volatile uint32_t*)0x53101224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5310122c) = 0x00000780; // SRAM read ptr
    *((volatile uint32_t*)0x53101230) = 0x00880920; // Layer control
    *((volatile uint32_t*)0x53101234) = 0x00158000; // Layer control 2
    *((volatile uint32_t*)0x53101238) = 0x000014b8; // Mask count
    *((volatile uint32_t*)0x5310123c) = 0x00001360; // Mask offset
    *((volatile uint32_t*)0x53101240) = 0x0000002b; // Output channel count
    *((volatile uint32_t*)0x53101244) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t*)0x5310124c) = 0x0002306c; // Post processing register

    // Layer 18 quadrant 3
    *((volatile uint32_t*)0x54101204) = 0x00018003; // Rows
    *((volatile uint32_t*)0x54101208) = 0x00018003; // Columns
    *((volatile uint32_t*)0x54101218) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5410121c) = 0x00028d95; // SRAM write ptr
    *((volatile uint32_t*)0x54101224) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5410122c) = 0x00000780; // SRAM read ptr
    *((volatile uint32_t*)0x54101230) = 0x00880920; // Layer control
    *((volatile uint32_t*)0x54101234) = 0x00158000; // Layer control 2
    *((volatile uint32_t*)0x54101238) = 0x000014b8; // Mask count
    *((volatile uint32_t*)0x5410123c) = 0x00001360; // Mask offset
    *((volatile uint32_t*)0x54101240) = 0x0000002b; // Output channel count
    *((volatile uint32_t*)0x54101244) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t*)0x5410124c) = 0x00022000; // Post processing register

    // Layer 19 quadrant 0
    *((volatile uint32_t*)0x51101304) = 0x00018001; // Rows
    *((volatile uint32_t*)0x51101308) = 0x00018001; // Columns
    *((volatile uint32_t*)0x51101318) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5110131c) = 0x00028da5; // SRAM write ptr
    *((volatile uint32_t*)0x51101324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5110132c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x51101330) = 0x0088a920; // Layer control
    *((volatile uint32_t*)0x51101334) = 0x00158000; // Layer control 2
    *((volatile uint32_t*)0x51101338) = 0x00001618; // Mask count
    *((volatile uint32_t*)0x5110133c) = 0x000014c0; // Mask offset
    *((volatile uint32_t*)0x51101340) = 0x0000002b; // Output channel count
    *((volatile uint32_t*)0x51101344) = 0x00000001; // TRAM ptr max
    *((volatile uint32_t*)0x5110134c) = 0x00022000; // Post processing register

    // Layer 19 quadrant 1
    *((volatile uint32_t*)0x52101304) = 0x00018001; // Rows
    *((volatile uint32_t*)0x52101308) = 0x00018001; // Columns
    *((volatile uint32_t*)0x52101318) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5210131c) = 0x00028da5; // SRAM write ptr
    *((volatile uint32_t*)0x52101324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5210132c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x52101330) = 0x00880920; // Layer control
    *((volatile uint32_t*)0x52101334) = 0x00158000; // Layer control 2
    *((volatile uint32_t*)0x52101338) = 0x00001618; // Mask count
    *((volatile uint32_t*)0x5210133c) = 0x000014c0; // Mask offset
    *((volatile uint32_t*)0x52101340) = 0x0000002b; // Output channel count
    *((volatile uint32_t*)0x52101344) = 0x00000001; // TRAM ptr max
    *((volatile uint32_t*)0x5210134c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x52101348) = 0xffffffff; // Mask and processor enables

    // Layer 19 quadrant 2
    *((volatile uint32_t*)0x53101304) = 0x00018001; // Rows
    *((volatile uint32_t*)0x53101308) = 0x00018001; // Columns
    *((volatile uint32_t*)0x53101318) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5310131c) = 0x00028da5; // SRAM write ptr
    *((volatile uint32_t*)0x53101324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5310132c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x53101330) = 0x00880920; // Layer control
    *((volatile uint32_t*)0x53101334) = 0x00158000; // Layer control 2
    *((volatile uint32_t*)0x53101338) = 0x00001618; // Mask count
    *((volatile uint32_t*)0x5310133c) = 0x000014c0; // Mask offset
    *((volatile uint32_t*)0x53101340) = 0x0000002b; // Output channel count
    *((volatile uint32_t*)0x53101344) = 0x00000001; // TRAM ptr max
    *((volatile uint32_t*)0x5310134c) = 0x00022000; // Post processing register

    // Layer 19 quadrant 3
    *((volatile uint32_t*)0x54101304) = 0x00018001; // Rows
    *((volatile uint32_t*)0x54101308) = 0x00018001; // Columns
    *((volatile uint32_t*)0x54101318) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5410131c) = 0x00028da5; // SRAM write ptr
    *((volatile uint32_t*)0x54101324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5410132c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x54101330) = 0x00880920; // Layer control
    *((volatile uint32_t*)0x54101334) = 0x00158000; // Layer control 2
    *((volatile uint32_t*)0x54101338) = 0x00001618; // Mask count
    *((volatile uint32_t*)0x5410133c) = 0x000014c0; // Mask offset
    *((volatile uint32_t*)0x54101340) = 0x0000002b; // Output channel count
    *((volatile uint32_t*)0x54101344) = 0x00000001; // TRAM ptr max
    *((volatile uint32_t*)0x5410134c) = 0x0002306c; // Post processing register

    return CNN_OK;
}

int cnn_start(void)
{
    cnn_time = 0;

    *((volatile uint32_t*)0x51000000) = 0x00100808; // Enable quadrant 0
    *((volatile uint32_t*)0x52000000) = 0x00100809; // Enable quadrant 1
    *((volatile uint32_t*)0x53000000) = 0x00100809; // Enable quadrant 2
    *((volatile uint32_t*)0x54000000) = 0x00100809; // Enable quadrant 3

#ifdef CNN_INFERENCE_TIMER
    MXC_TMR_SW_Start(CNN_INFERENCE_TIMER);
#endif

    CNN_START;                                      // Allow capture of processing time
    *((volatile uint32_t*)0x51000000) = 0x00100009; // Master enable quadrant 0

    return CNN_OK;
}

int cnn_unload(uint32_t* out_buf32)
{
    uint8_t* out_buf = (uint8_t*)out_buf32;
    uint32_t val;
    volatile uint32_t* addr;
    int i;
    uint32_t offs;

    // Custom unload for this network, layer 12: 8-bit data, shape: (16, 18, 18)
    offs = 0x0000;
    addr = (volatile uint32_t*)0x51803000;
    for (i = 0; i < 324; i++) {
        val                   = *addr++;
        out_buf[offs]         = val & 0xff;
        out_buf[offs + 0x144] = (val >> 8) & 0xff;
        out_buf[offs + 0x288] = (val >> 16) & 0xff;
        out_buf[offs + 0x3cc] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x03cc;
    addr = (volatile uint32_t*)0x51823000;
    for (i = 0; i < 324; i++) {
        val                   = *addr++;
        out_buf[offs]         = val & 0xff;
        out_buf[offs + 0x144] = (val >> 8) & 0xff;
        out_buf[offs + 0x288] = (val >> 16) & 0xff;
        out_buf[offs + 0x3cc] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x03cc;
    addr = (volatile uint32_t*)0x51843000;
    for (i = 0; i < 324; i++) {
        val                   = *addr++;
        out_buf[offs]         = val & 0xff;
        out_buf[offs + 0x144] = (val >> 8) & 0xff;
        out_buf[offs + 0x288] = (val >> 16) & 0xff;
        out_buf[offs + 0x3cc] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x03cc;
    addr = (volatile uint32_t*)0x51863000;
    for (i = 0; i < 324; i++) {
        val                   = *addr++;
        out_buf[offs]         = val & 0xff;
        out_buf[offs + 0x144] = (val >> 8) & 0xff;
        out_buf[offs + 0x288] = (val >> 16) & 0xff;
        out_buf[offs + 0x3cc] = (val >> 24) & 0xff;
        offs++;
    }

    // Custom unload for this network, layer 13: 8-bit data, shape: (16, 9, 9)
    offs = 0x1440;
    addr = (volatile uint32_t*)0x51803510;
    for (i = 0; i < 81; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x51] = (val >> 8) & 0xff;
        out_buf[offs + 0xa2] = (val >> 16) & 0xff;
        out_buf[offs + 0xf3] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x00f3;
    addr = (volatile uint32_t*)0x51823510;
    for (i = 0; i < 81; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x51] = (val >> 8) & 0xff;
        out_buf[offs + 0xa2] = (val >> 16) & 0xff;
        out_buf[offs + 0xf3] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x00f3;
    addr = (volatile uint32_t*)0x51843510;
    for (i = 0; i < 81; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x51] = (val >> 8) & 0xff;
        out_buf[offs + 0xa2] = (val >> 16) & 0xff;
        out_buf[offs + 0xf3] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x00f3;
    addr = (volatile uint32_t*)0x51863510;
    for (i = 0; i < 81; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x51] = (val >> 8) & 0xff;
        out_buf[offs + 0xa2] = (val >> 16) & 0xff;
        out_buf[offs + 0xf3] = (val >> 24) & 0xff;
        offs++;
    }

    // Custom unload for this network, layer 14: 8-bit data, shape: (16, 4, 4)
    offs = 0x1950;
    addr = (volatile uint32_t*)0x51803654;
    for (i = 0; i < 16; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x10] = (val >> 8) & 0xff;
        out_buf[offs + 0x20] = (val >> 16) & 0xff;
        out_buf[offs + 0x30] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x0030;
    addr = (volatile uint32_t*)0x51823654;
    for (i = 0; i < 16; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x10] = (val >> 8) & 0xff;
        out_buf[offs + 0x20] = (val >> 16) & 0xff;
        out_buf[offs + 0x30] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x0030;
    addr = (volatile uint32_t*)0x51843654;
    for (i = 0; i < 16; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x10] = (val >> 8) & 0xff;
        out_buf[offs + 0x20] = (val >> 16) & 0xff;
        out_buf[offs + 0x30] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x0030;
    addr = (volatile uint32_t*)0x51863654;
    for (i = 0; i < 16; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x10] = (val >> 8) & 0xff;
        out_buf[offs + 0x20] = (val >> 16) & 0xff;
        out_buf[offs + 0x30] = (val >> 24) & 0xff;
        offs++;
    }

    // Custom unload for this network, layer 15: 8-bit data, shape: (16, 2, 2)
    offs = 0x1a50;
    addr = (volatile uint32_t*)0x51803694;
    for (i = 0; i < 4; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x04] = (val >> 8) & 0xff;
        out_buf[offs + 0x08] = (val >> 16) & 0xff;
        out_buf[offs + 0x0c] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x000c;
    addr = (volatile uint32_t*)0x51823694;
    for (i = 0; i < 4; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x04] = (val >> 8) & 0xff;
        out_buf[offs + 0x08] = (val >> 16) & 0xff;
        out_buf[offs + 0x0c] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x000c;
    addr = (volatile uint32_t*)0x51843694;
    for (i = 0; i < 4; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x04] = (val >> 8) & 0xff;
        out_buf[offs + 0x08] = (val >> 16) & 0xff;
        out_buf[offs + 0x0c] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x000c;
    addr = (volatile uint32_t*)0x51863694;
    for (i = 0; i < 4; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x04] = (val >> 8) & 0xff;
        out_buf[offs + 0x08] = (val >> 16) & 0xff;
        out_buf[offs + 0x0c] = (val >> 24) & 0xff;
        offs++;
    }

    // Custom unload for this network, layer 16: 8-bit data, shape: (44, 18, 18)
    offs = 0x1a90;
    addr = (volatile uint32_t*)0x52823000;
    for (i = 0; i < 324; i++) {
        val                   = *addr++;
        out_buf[offs]         = val & 0xff;
        out_buf[offs + 0x144] = (val >> 8) & 0xff;
        out_buf[offs + 0x288] = (val >> 16) & 0xff;
        out_buf[offs + 0x3cc] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x03cc;
    addr = (volatile uint32_t*)0x52843000;
    for (i = 0; i < 324; i++) {
        val                   = *addr++;
        out_buf[offs]         = val & 0xff;
        out_buf[offs + 0x144] = (val >> 8) & 0xff;
        out_buf[offs + 0x288] = (val >> 16) & 0xff;
        out_buf[offs + 0x3cc] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x03cc;
    addr = (volatile uint32_t*)0x52863000;
    for (i = 0; i < 324; i++) {
        val                   = *addr++;
        out_buf[offs]         = val & 0xff;
        out_buf[offs + 0x144] = (val >> 8) & 0xff;
        out_buf[offs + 0x288] = (val >> 16) & 0xff;
        out_buf[offs + 0x3cc] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x03cc;
    addr = (volatile uint32_t*)0x53803000;
    for (i = 0; i < 324; i++) {
        val                   = *addr++;
        out_buf[offs]         = val & 0xff;
        out_buf[offs + 0x144] = (val >> 8) & 0xff;
        out_buf[offs + 0x288] = (val >> 16) & 0xff;
        out_buf[offs + 0x3cc] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x03cc;
    addr = (volatile uint32_t*)0x53823000;
    for (i = 0; i < 324; i++) {
        val                   = *addr++;
        out_buf[offs]         = val & 0xff;
        out_buf[offs + 0x144] = (val >> 8) & 0xff;
        out_buf[offs + 0x288] = (val >> 16) & 0xff;
        out_buf[offs + 0x3cc] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x03cc;
    addr = (volatile uint32_t*)0x53843000;
    for (i = 0; i < 324; i++) {
        val                   = *addr++;
        out_buf[offs]         = val & 0xff;
        out_buf[offs + 0x144] = (val >> 8) & 0xff;
        out_buf[offs + 0x288] = (val >> 16) & 0xff;
        out_buf[offs + 0x3cc] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x03cc;
    addr = (volatile uint32_t*)0x53863000;
    for (i = 0; i < 324; i++) {
        val                   = *addr++;
        out_buf[offs]         = val & 0xff;
        out_buf[offs + 0x144] = (val >> 8) & 0xff;
        out_buf[offs + 0x288] = (val >> 16) & 0xff;
        out_buf[offs + 0x3cc] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x03cc;
    addr = (volatile uint32_t*)0x54803000;
    for (i = 0; i < 324; i++) {
        val                   = *addr++;
        out_buf[offs]         = val & 0xff;
        out_buf[offs + 0x144] = (val >> 8) & 0xff;
        out_buf[offs + 0x288] = (val >> 16) & 0xff;
        out_buf[offs + 0x3cc] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x03cc;
    addr = (volatile uint32_t*)0x54823000;
    for (i = 0; i < 324; i++) {
        val                   = *addr++;
        out_buf[offs]         = val & 0xff;
        out_buf[offs + 0x144] = (val >> 8) & 0xff;
        out_buf[offs + 0x288] = (val >> 16) & 0xff;
        out_buf[offs + 0x3cc] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x03cc;
    addr = (volatile uint32_t*)0x54843000;
    for (i = 0; i < 324; i++) {
        val                   = *addr++;
        out_buf[offs]         = val & 0xff;
        out_buf[offs + 0x144] = (val >> 8) & 0xff;
        out_buf[offs + 0x288] = (val >> 16) & 0xff;
        out_buf[offs + 0x3cc] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x03cc;
    addr = (volatile uint32_t*)0x54863000;
    for (i = 0; i < 324; i++) {
        val                   = *addr++;
        out_buf[offs]         = val & 0xff;
        out_buf[offs + 0x144] = (val >> 8) & 0xff;
        out_buf[offs + 0x288] = (val >> 16) & 0xff;
        out_buf[offs + 0x3cc] = (val >> 24) & 0xff;
        offs++;
    }

    // Custom unload for this network, layer 17: 8-bit data, shape: (44, 9, 9)
    offs = 0x5240;
    addr = (volatile uint32_t*)0x52823510;
    for (i = 0; i < 81; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x51] = (val >> 8) & 0xff;
        out_buf[offs + 0xa2] = (val >> 16) & 0xff;
        out_buf[offs + 0xf3] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x00f3;
    addr = (volatile uint32_t*)0x52843510;
    for (i = 0; i < 81; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x51] = (val >> 8) & 0xff;
        out_buf[offs + 0xa2] = (val >> 16) & 0xff;
        out_buf[offs + 0xf3] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x00f3;
    addr = (volatile uint32_t*)0x52863510;
    for (i = 0; i < 81; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x51] = (val >> 8) & 0xff;
        out_buf[offs + 0xa2] = (val >> 16) & 0xff;
        out_buf[offs + 0xf3] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x00f3;
    addr = (volatile uint32_t*)0x53803510;
    for (i = 0; i < 81; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x51] = (val >> 8) & 0xff;
        out_buf[offs + 0xa2] = (val >> 16) & 0xff;
        out_buf[offs + 0xf3] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x00f3;
    addr = (volatile uint32_t*)0x53823510;
    for (i = 0; i < 81; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x51] = (val >> 8) & 0xff;
        out_buf[offs + 0xa2] = (val >> 16) & 0xff;
        out_buf[offs + 0xf3] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x00f3;
    addr = (volatile uint32_t*)0x53843510;
    for (i = 0; i < 81; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x51] = (val >> 8) & 0xff;
        out_buf[offs + 0xa2] = (val >> 16) & 0xff;
        out_buf[offs + 0xf3] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x00f3;
    addr = (volatile uint32_t*)0x53863510;
    for (i = 0; i < 81; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x51] = (val >> 8) & 0xff;
        out_buf[offs + 0xa2] = (val >> 16) & 0xff;
        out_buf[offs + 0xf3] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x00f3;
    addr = (volatile uint32_t*)0x54803510;
    for (i = 0; i < 81; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x51] = (val >> 8) & 0xff;
        out_buf[offs + 0xa2] = (val >> 16) & 0xff;
        out_buf[offs + 0xf3] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x00f3;
    addr = (volatile uint32_t*)0x54823510;
    for (i = 0; i < 81; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x51] = (val >> 8) & 0xff;
        out_buf[offs + 0xa2] = (val >> 16) & 0xff;
        out_buf[offs + 0xf3] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x00f3;
    addr = (volatile uint32_t*)0x54843510;
    for (i = 0; i < 81; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x51] = (val >> 8) & 0xff;
        out_buf[offs + 0xa2] = (val >> 16) & 0xff;
        out_buf[offs + 0xf3] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x00f3;
    addr = (volatile uint32_t*)0x54863510;
    for (i = 0; i < 81; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x51] = (val >> 8) & 0xff;
        out_buf[offs + 0xa2] = (val >> 16) & 0xff;
        out_buf[offs + 0xf3] = (val >> 24) & 0xff;
        offs++;
    }

    // Custom unload for this network, layer 18: 8-bit data, shape: (44, 4, 4)
    offs = 0x602c;
    addr = (volatile uint32_t*)0x52823654;
    for (i = 0; i < 16; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x10] = (val >> 8) & 0xff;
        out_buf[offs + 0x20] = (val >> 16) & 0xff;
        out_buf[offs + 0x30] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x0030;
    addr = (volatile uint32_t*)0x52843654;
    for (i = 0; i < 16; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x10] = (val >> 8) & 0xff;
        out_buf[offs + 0x20] = (val >> 16) & 0xff;
        out_buf[offs + 0x30] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x0030;
    addr = (volatile uint32_t*)0x52863654;
    for (i = 0; i < 16; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x10] = (val >> 8) & 0xff;
        out_buf[offs + 0x20] = (val >> 16) & 0xff;
        out_buf[offs + 0x30] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x0030;
    addr = (volatile uint32_t*)0x53803654;
    for (i = 0; i < 16; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x10] = (val >> 8) & 0xff;
        out_buf[offs + 0x20] = (val >> 16) & 0xff;
        out_buf[offs + 0x30] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x0030;
    addr = (volatile uint32_t*)0x53823654;
    for (i = 0; i < 16; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x10] = (val >> 8) & 0xff;
        out_buf[offs + 0x20] = (val >> 16) & 0xff;
        out_buf[offs + 0x30] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x0030;
    addr = (volatile uint32_t*)0x53843654;
    for (i = 0; i < 16; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x10] = (val >> 8) & 0xff;
        out_buf[offs + 0x20] = (val >> 16) & 0xff;
        out_buf[offs + 0x30] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x0030;
    addr = (volatile uint32_t*)0x53863654;
    for (i = 0; i < 16; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x10] = (val >> 8) & 0xff;
        out_buf[offs + 0x20] = (val >> 16) & 0xff;
        out_buf[offs + 0x30] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x0030;
    addr = (volatile uint32_t*)0x54803654;
    for (i = 0; i < 16; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x10] = (val >> 8) & 0xff;
        out_buf[offs + 0x20] = (val >> 16) & 0xff;
        out_buf[offs + 0x30] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x0030;
    addr = (volatile uint32_t*)0x54823654;
    for (i = 0; i < 16; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x10] = (val >> 8) & 0xff;
        out_buf[offs + 0x20] = (val >> 16) & 0xff;
        out_buf[offs + 0x30] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x0030;
    addr = (volatile uint32_t*)0x54843654;
    for (i = 0; i < 16; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x10] = (val >> 8) & 0xff;
        out_buf[offs + 0x20] = (val >> 16) & 0xff;
        out_buf[offs + 0x30] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x0030;
    addr = (volatile uint32_t*)0x54863654;
    for (i = 0; i < 16; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x10] = (val >> 8) & 0xff;
        out_buf[offs + 0x20] = (val >> 16) & 0xff;
        out_buf[offs + 0x30] = (val >> 24) & 0xff;
        offs++;
    }

    // Custom unload for this network, layer 19: 8-bit data, shape: (44, 2, 2)
    offs = 0x62ec;
    addr = (volatile uint32_t*)0x52823694;
    for (i = 0; i < 4; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x04] = (val >> 8) & 0xff;
        out_buf[offs + 0x08] = (val >> 16) & 0xff;
        out_buf[offs + 0x0c] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x000c;
    addr = (volatile uint32_t*)0x52843694;
    for (i = 0; i < 4; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x04] = (val >> 8) & 0xff;
        out_buf[offs + 0x08] = (val >> 16) & 0xff;
        out_buf[offs + 0x0c] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x000c;
    addr = (volatile uint32_t*)0x52863694;
    for (i = 0; i < 4; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x04] = (val >> 8) & 0xff;
        out_buf[offs + 0x08] = (val >> 16) & 0xff;
        out_buf[offs + 0x0c] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x000c;
    addr = (volatile uint32_t*)0x53803694;
    for (i = 0; i < 4; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x04] = (val >> 8) & 0xff;
        out_buf[offs + 0x08] = (val >> 16) & 0xff;
        out_buf[offs + 0x0c] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x000c;
    addr = (volatile uint32_t*)0x53823694;
    for (i = 0; i < 4; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x04] = (val >> 8) & 0xff;
        out_buf[offs + 0x08] = (val >> 16) & 0xff;
        out_buf[offs + 0x0c] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x000c;
    addr = (volatile uint32_t*)0x53843694;
    for (i = 0; i < 4; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x04] = (val >> 8) & 0xff;
        out_buf[offs + 0x08] = (val >> 16) & 0xff;
        out_buf[offs + 0x0c] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x000c;
    addr = (volatile uint32_t*)0x53863694;
    for (i = 0; i < 4; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x04] = (val >> 8) & 0xff;
        out_buf[offs + 0x08] = (val >> 16) & 0xff;
        out_buf[offs + 0x0c] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x000c;
    addr = (volatile uint32_t*)0x54803694;
    for (i = 0; i < 4; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x04] = (val >> 8) & 0xff;
        out_buf[offs + 0x08] = (val >> 16) & 0xff;
        out_buf[offs + 0x0c] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x000c;
    addr = (volatile uint32_t*)0x54823694;
    for (i = 0; i < 4; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x04] = (val >> 8) & 0xff;
        out_buf[offs + 0x08] = (val >> 16) & 0xff;
        out_buf[offs + 0x0c] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x000c;
    addr = (volatile uint32_t*)0x54843694;
    for (i = 0; i < 4; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x04] = (val >> 8) & 0xff;
        out_buf[offs + 0x08] = (val >> 16) & 0xff;
        out_buf[offs + 0x0c] = (val >> 24) & 0xff;
        offs++;
    }
    offs += 0x000c;
    addr = (volatile uint32_t*)0x54863694;
    for (i = 0; i < 4; i++) {
        val                  = *addr++;
        out_buf[offs]        = val & 0xff;
        out_buf[offs + 0x04] = (val >> 8) & 0xff;
        out_buf[offs + 0x08] = (val >> 16) & 0xff;
        out_buf[offs + 0x0c] = (val >> 24) & 0xff;
        offs++;
    }

    return CNN_OK;
}

int cnn_enable(uint32_t clock_source, uint32_t clock_divider)
{
    // Reset all domains, restore power to CNN
    MXC_GCFR->reg3 = 0xf; // Reset
    MXC_GCFR->reg1 = 0xf; // Mask memory
    MXC_GCFR->reg0 = 0xf; // Power
    MXC_Delay(MSEC(10));  // Wait for load switches
    MXC_GCFR->reg2 = 0x0; // Iso
    MXC_GCFR->reg3 = 0x0; // Reset

    if (clock_source == MXC_S_GCR_PCLKDIV_CNNCLKSEL_IPLL)
        while ((MXC_GCR->ipll_ctrl & MXC_F_GCR_IPLL_CTRL_RDY) != MXC_F_GCR_IPLL_CTRL_RDY)
            ; // Wait for PLL

    MXC_GCR->pclkdiv =
        (MXC_GCR->pclkdiv & ~(MXC_F_GCR_PCLKDIV_CNNCLKDIV | MXC_F_GCR_PCLKDIV_CNNCLKSEL)) |
        clock_divider | clock_source;
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_CNN); // Enable CNN clock

    MXC_NVIC_SetVector(CNN_IRQn, CNN_ISR); // Set CNN complete vector

    return CNN_OK;
}

int cnn_boost_enable(mxc_gpio_regs_t* port, uint32_t pin)
{
    mxc_gpio_cfg_t gpio_out;
    gpio_out.port = port;
    gpio_out.mask = pin;
    gpio_out.pad  = MXC_GPIO_PAD_NONE;
    gpio_out.func = MXC_GPIO_FUNC_OUT;
    MXC_GPIO_Config(&gpio_out);
    MXC_GPIO_OutSet(gpio_out.port, gpio_out.mask);

    return CNN_OK;
}

int cnn_boost_disable(mxc_gpio_regs_t* port, uint32_t pin)
{
    mxc_gpio_cfg_t gpio_out;
    gpio_out.port = port;
    gpio_out.mask = pin;
    gpio_out.pad  = MXC_GPIO_PAD_NONE;
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
