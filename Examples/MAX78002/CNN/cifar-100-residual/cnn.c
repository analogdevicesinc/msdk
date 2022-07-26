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

// cifar-100-residual
// Created using ai8xize.py --test-dir sdk/Examples/MAX78002/CNN --prefix cifar-100-residual --checkpoint-file trained/ai85-cifar100-residual-qat8-q.pth.tar --config-file networks/cifar100-ressimplenet.yaml --softmax --device MAX78002 --timer 0 --display-checkpoint --verbose --overwrite

// DO NOT EDIT - regenerate this file instead!

// Configuring 17 layers
// Input data: HWC
// Layer 0: 3x32x32, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 16x32x32 output
// Layer 1: 16x32x32, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 20x32x32 output
// Layer 2: 20x32x32, no pooling, no convolution, 20x32x32 output
// Layer 3: 20x32x32, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 20x32x32 output
// Layer 4: 2x20x32x32, no pooling, 2-element add, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 20x32x32 output
// Layer 5: 20x32x32, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 20x16x16 output
// Layer 6: 20x16x16, no pooling, no convolution, 20x16x16 output
// Layer 7: 20x16x16, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 20x16x16 output
// Layer 8: 2x20x16x16, no pooling, 2-element add, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 44x16x16 output
// Layer 9: 44x16x16, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 48x8x8 output
// Layer 10: 48x8x8, no pooling, no convolution, 48x8x8 output
// Layer 11: 48x8x8, no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 48x8x8 output
// Layer 12: 2x48x8x8, 2-element add, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 96x4x4 output
// Layer 13: 96x4x4, max pool 2x2 with stride 2/2, conv2d with kernel size 1x1, stride 1/1, pad 0/0, ReLU, 512x2x2 output
// Layer 14: 512x2x2, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, ReLU, 128x2x2 output
// Layer 15: 128x2x2, max pool 2x2 with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, ReLU, 128x1x1 output
// Layer 16: 128x1x1, no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, no activation, 100x1x1 output

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
    while (n-- > 0) {
        *dst++ = *src++;
    }
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
        while (len-- > 0)
            *addr++ = *ptr++;
    }

    return CNN_OK;
}

int cnn_load_bias(void)
{
    // Not used in this network
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
    *((volatile uint32_t*)0x5100000c) = 0x00001880; // Clear registers
    *((volatile uint32_t*)0x5200000c) = 0x00001880; // Clear registers
    *((volatile uint32_t*)0x5300000c) = 0x00001880; // Clear registers
    *((volatile uint32_t*)0x5400000c) = 0x00001880; // Clear registers
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
    *((volatile uint32_t*)0x51000008) = 0x00000010; // Layer count
    *((volatile uint32_t*)0x52000000) = 0x00100008; // Stop SM
    *((volatile uint32_t*)0x52000008) = 0x00000010; // Layer count
    *((volatile uint32_t*)0x53000000) = 0x00100008; // Stop SM
    *((volatile uint32_t*)0x53000008) = 0x00000010; // Layer count
    *((volatile uint32_t*)0x54000000) = 0x00100008; // Stop SM
    *((volatile uint32_t*)0x54000008) = 0x00000010; // Layer count

    return CNN_OK;
}

int cnn_configure(void)
{
    // Layer 0 quadrant 0
    *((volatile uint32_t*)0x51100004) = 0x0001801f; // Rows
    *((volatile uint32_t*)0x51100008) = 0x0001801f; // Columns
    *((volatile uint32_t*)0x51100018) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5110001c) = 0x00058800; // SRAM write ptr
    *((volatile uint32_t*)0x51100024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x51100030) = 0x00888b20; // Layer control
    *((volatile uint32_t*)0x51100034) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x51100038) = 0x00000078; // Mask count
    *((volatile uint32_t*)0x51100040) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x51100044) = 0x0000001f; // TRAM ptr max
    *((volatile uint32_t*)0x5110004c) = 0x00024000; // Post processing register

    // Layer 0 quadrant 1
    *((volatile uint32_t*)0x52100004) = 0x0001801f; // Rows
    *((volatile uint32_t*)0x52100008) = 0x0001801f; // Columns
    *((volatile uint32_t*)0x52100018) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5210001c) = 0x00058800; // SRAM write ptr
    *((volatile uint32_t*)0x52100024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x52100030) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x52100034) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x52100038) = 0x00000078; // Mask count
    *((volatile uint32_t*)0x52100040) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x52100044) = 0x0000001f; // TRAM ptr max
    *((volatile uint32_t*)0x5210004c) = 0x00024000; // Post processing register

    // Layer 0 quadrant 2
    *((volatile uint32_t*)0x53100004) = 0x0001801f; // Rows
    *((volatile uint32_t*)0x53100008) = 0x0001801f; // Columns
    *((volatile uint32_t*)0x53100018) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5310001c) = 0x00058800; // SRAM write ptr
    *((volatile uint32_t*)0x53100024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x53100030) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x53100034) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x53100038) = 0x00000078; // Mask count
    *((volatile uint32_t*)0x53100040) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x53100044) = 0x0000001f; // TRAM ptr max
    *((volatile uint32_t*)0x5310004c) = 0x00024000; // Post processing register

    // Layer 0 quadrant 3
    *((volatile uint32_t*)0x54100004) = 0x0001801f; // Rows
    *((volatile uint32_t*)0x54100008) = 0x0001801f; // Columns
    *((volatile uint32_t*)0x54100018) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5410001c) = 0x00058800; // SRAM write ptr
    *((volatile uint32_t*)0x54100024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x54100030) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x54100034) = 0x00078000; // Layer control 2
    *((volatile uint32_t*)0x54100038) = 0x00000078; // Mask count
    *((volatile uint32_t*)0x54100040) = 0x0000000f; // Output channel count
    *((volatile uint32_t*)0x54100044) = 0x0000001f; // TRAM ptr max
    *((volatile uint32_t*)0x5410004c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x54100048) = 0x70007000; // Mask and processor enables

    // Layer 1 quadrant 0
    *((volatile uint32_t*)0x51100104) = 0x0001801f; // Rows
    *((volatile uint32_t*)0x51100108) = 0x0001801f; // Columns
    *((volatile uint32_t*)0x51100118) = 0x00000010; // Stride
    *((volatile uint32_t*)0x51100124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5110012c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x51100130) = 0x0088cb20; // Layer control
    *((volatile uint32_t*)0x51100134) = 0x00098000; // Layer control 2
    *((volatile uint32_t*)0x51100138) = 0x00000098; // Mask count
    *((volatile uint32_t*)0x51100140) = 0x00000013; // Output channel count
    *((volatile uint32_t*)0x51100144) = 0x0000001f; // TRAM ptr max
    *((volatile uint32_t*)0x5110014c) = 0x00024000; // Post processing register

    // Layer 1 quadrant 1
    *((volatile uint32_t*)0x52100104) = 0x0001801f; // Rows
    *((volatile uint32_t*)0x52100108) = 0x0001801f; // Columns
    *((volatile uint32_t*)0x52100118) = 0x00000010; // Stride
    *((volatile uint32_t*)0x52100124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5210012c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x52100130) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x52100134) = 0x00098000; // Layer control 2
    *((volatile uint32_t*)0x52100138) = 0x00000098; // Mask count
    *((volatile uint32_t*)0x52100140) = 0x00000013; // Output channel count
    *((volatile uint32_t*)0x52100144) = 0x0000001f; // TRAM ptr max
    *((volatile uint32_t*)0x5210014c) = 0x00024000; // Post processing register

    // Layer 1 quadrant 2
    *((volatile uint32_t*)0x53100104) = 0x0001801f; // Rows
    *((volatile uint32_t*)0x53100108) = 0x0001801f; // Columns
    *((volatile uint32_t*)0x53100118) = 0x00000010; // Stride
    *((volatile uint32_t*)0x53100124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5310012c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x53100130) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x53100134) = 0x00098000; // Layer control 2
    *((volatile uint32_t*)0x53100138) = 0x00000098; // Mask count
    *((volatile uint32_t*)0x53100140) = 0x00000013; // Output channel count
    *((volatile uint32_t*)0x53100144) = 0x0000001f; // TRAM ptr max
    *((volatile uint32_t*)0x5310014c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x53100148) = 0xf000f000; // Mask and processor enables

    // Layer 1 quadrant 3
    *((volatile uint32_t*)0x54100104) = 0x0001801f; // Rows
    *((volatile uint32_t*)0x54100108) = 0x0001801f; // Columns
    *((volatile uint32_t*)0x54100118) = 0x00000010; // Stride
    *((volatile uint32_t*)0x54100124) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5410012c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x54100130) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x54100134) = 0x00098000; // Layer control 2
    *((volatile uint32_t*)0x54100138) = 0x00000098; // Mask count
    *((volatile uint32_t*)0x54100140) = 0x00000013; // Output channel count
    *((volatile uint32_t*)0x54100144) = 0x0000001f; // TRAM ptr max
    *((volatile uint32_t*)0x5410014c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x54100148) = 0x0fff0fff; // Mask and processor enables

    // Layer 2 quadrant 0
    *((volatile uint32_t*)0x51100204) = 0x0001001f; // Rows
    *((volatile uint32_t*)0x51100208) = 0x0001001f; // Columns
    *((volatile uint32_t*)0x51100218) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5110021c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t*)0x51100220) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t*)0x51100230) = 0x00000920; // Layer control
    *((volatile uint32_t*)0x51100234) = 0x00000010; // Layer control 2
    *((volatile uint32_t*)0x51100240) = 0x00000003; // Output channel count
    *((volatile uint32_t*)0x5110020c) = 0x00000103; // 1D
    *((volatile uint32_t*)0x5110024c) = 0x03000000; // Post processing register
    *((volatile uint32_t*)0x51100248) = 0x0000ffff; // Mask and processor enables

    // Layer 2 quadrant 1
    *((volatile uint32_t*)0x52100204) = 0x0001001f; // Rows
    *((volatile uint32_t*)0x52100208) = 0x0001001f; // Columns
    *((volatile uint32_t*)0x52100218) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5210021c) = 0x00020800; // SRAM write ptr
    *((volatile uint32_t*)0x52100220) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t*)0x52100230) = 0x00000920; // Layer control
    *((volatile uint32_t*)0x52100234) = 0x00000010; // Layer control 2
    *((volatile uint32_t*)0x52100240) = 0x00000003; // Output channel count
    *((volatile uint32_t*)0x5210020c) = 0x00000103; // 1D
    *((volatile uint32_t*)0x5210024c) = 0x03000000; // Post processing register
    *((volatile uint32_t*)0x52100248) = 0x0000000f; // Mask and processor enables

    // Layer 2 quadrant 2
    *((volatile uint32_t*)0x53100204) = 0x0001001f; // Rows
    *((volatile uint32_t*)0x53100208) = 0x0001001f; // Columns
    *((volatile uint32_t*)0x53100218) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5310021c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t*)0x53100220) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t*)0x53100230) = 0x00000920; // Layer control
    *((volatile uint32_t*)0x53100234) = 0x00000010; // Layer control 2
    *((volatile uint32_t*)0x53100240) = 0x00000003; // Output channel count
    *((volatile uint32_t*)0x5310020c) = 0x00000103; // 1D
    *((volatile uint32_t*)0x5310024c) = 0x03000000; // Post processing register

    // Layer 2 quadrant 3
    *((volatile uint32_t*)0x54100204) = 0x0001001f; // Rows
    *((volatile uint32_t*)0x54100208) = 0x0001001f; // Columns
    *((volatile uint32_t*)0x54100218) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5410021c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t*)0x54100220) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t*)0x54100230) = 0x00000920; // Layer control
    *((volatile uint32_t*)0x54100234) = 0x00000010; // Layer control 2
    *((volatile uint32_t*)0x54100240) = 0x00000003; // Output channel count
    *((volatile uint32_t*)0x5410020c) = 0x00000103; // 1D
    *((volatile uint32_t*)0x5410024c) = 0x03000000; // Post processing register

    // Layer 3 quadrant 0
    *((volatile uint32_t*)0x51100304) = 0x0001801f; // Rows
    *((volatile uint32_t*)0x51100308) = 0x0001801f; // Columns
    *((volatile uint32_t*)0x51100318) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5110031c) = 0x00000801; // SRAM write ptr
    *((volatile uint32_t*)0x51100324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x51100330) = 0x00882b20; // Layer control
    *((volatile uint32_t*)0x51100334) = 0x00098010; // Layer control 2
    *((volatile uint32_t*)0x51100338) = 0x00000098; // Mask count
    *((volatile uint32_t*)0x51100340) = 0x00000013; // Output channel count
    *((volatile uint32_t*)0x51100344) = 0x0000001f; // TRAM ptr max
    *((volatile uint32_t*)0x5110034c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x51100348) = 0xffffffff; // Mask and processor enables

    // Layer 3 quadrant 1
    *((volatile uint32_t*)0x52100304) = 0x0001801f; // Rows
    *((volatile uint32_t*)0x52100308) = 0x0001801f; // Columns
    *((volatile uint32_t*)0x52100318) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5210031c) = 0x00000801; // SRAM write ptr
    *((volatile uint32_t*)0x52100324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x52100330) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x52100334) = 0x00098010; // Layer control 2
    *((volatile uint32_t*)0x52100338) = 0x00000098; // Mask count
    *((volatile uint32_t*)0x52100340) = 0x00000013; // Output channel count
    *((volatile uint32_t*)0x52100344) = 0x0000001f; // TRAM ptr max
    *((volatile uint32_t*)0x5210034c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x52100348) = 0x000f000f; // Mask and processor enables

    // Layer 3 quadrant 2
    *((volatile uint32_t*)0x53100304) = 0x0001801f; // Rows
    *((volatile uint32_t*)0x53100308) = 0x0001801f; // Columns
    *((volatile uint32_t*)0x53100318) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5310031c) = 0x00000801; // SRAM write ptr
    *((volatile uint32_t*)0x53100324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x53100330) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x53100334) = 0x00098010; // Layer control 2
    *((volatile uint32_t*)0x53100338) = 0x00000098; // Mask count
    *((volatile uint32_t*)0x53100340) = 0x00000013; // Output channel count
    *((volatile uint32_t*)0x53100344) = 0x0000001f; // TRAM ptr max
    *((volatile uint32_t*)0x5310034c) = 0x00024000; // Post processing register

    // Layer 3 quadrant 3
    *((volatile uint32_t*)0x54100304) = 0x0001801f; // Rows
    *((volatile uint32_t*)0x54100308) = 0x0001801f; // Columns
    *((volatile uint32_t*)0x54100318) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5410031c) = 0x00000801; // SRAM write ptr
    *((volatile uint32_t*)0x54100324) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x54100330) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x54100334) = 0x00098010; // Layer control 2
    *((volatile uint32_t*)0x54100338) = 0x00000098; // Mask count
    *((volatile uint32_t*)0x54100340) = 0x00000013; // Output channel count
    *((volatile uint32_t*)0x54100344) = 0x0000001f; // TRAM ptr max
    *((volatile uint32_t*)0x5410034c) = 0x00024000; // Post processing register

    // Layer 4 quadrant 0
    *((volatile uint32_t*)0x51100404) = 0x0002801f; // Rows
    *((volatile uint32_t*)0x51100408) = 0x0001801f; // Columns
    *((volatile uint32_t*)0x51100418) = 0x00000020; // Stride
    *((volatile uint32_t*)0x5110041c) = 0x00058000; // SRAM write ptr
    *((volatile uint32_t*)0x51100424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5110042c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x51100430) = 0x00882b20; // Layer control
    *((volatile uint32_t*)0x51100434) = 0x00098000; // Layer control 2
    *((volatile uint32_t*)0x51100438) = 0x00000138; // Mask count
    *((volatile uint32_t*)0x5110043c) = 0x000000a0; // Mask offset
    *((volatile uint32_t*)0x51100440) = 0x00000013; // Output channel count
    *((volatile uint32_t*)0x5110040c) = 0x00066000; // 1D
    *((volatile uint32_t*)0x51100444) = 0x0000001f; // TRAM ptr max
    *((volatile uint32_t*)0x5110044c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x51100448) = 0xffffffff; // Mask and processor enables

    // Layer 4 quadrant 1
    *((volatile uint32_t*)0x52100404) = 0x0002801f; // Rows
    *((volatile uint32_t*)0x52100408) = 0x0001801f; // Columns
    *((volatile uint32_t*)0x52100418) = 0x00000020; // Stride
    *((volatile uint32_t*)0x5210041c) = 0x00058000; // SRAM write ptr
    *((volatile uint32_t*)0x52100424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5210042c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x52100430) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x52100434) = 0x00098000; // Layer control 2
    *((volatile uint32_t*)0x52100438) = 0x00000138; // Mask count
    *((volatile uint32_t*)0x5210043c) = 0x000000a0; // Mask offset
    *((volatile uint32_t*)0x52100440) = 0x00000013; // Output channel count
    *((volatile uint32_t*)0x5210040c) = 0x00066000; // 1D
    *((volatile uint32_t*)0x52100444) = 0x0000001f; // TRAM ptr max
    *((volatile uint32_t*)0x5210044c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x52100448) = 0x000f000f; // Mask and processor enables

    // Layer 4 quadrant 2
    *((volatile uint32_t*)0x53100404) = 0x0002801f; // Rows
    *((volatile uint32_t*)0x53100408) = 0x0001801f; // Columns
    *((volatile uint32_t*)0x53100418) = 0x00000020; // Stride
    *((volatile uint32_t*)0x5310041c) = 0x00058000; // SRAM write ptr
    *((volatile uint32_t*)0x53100424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5310042c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x53100430) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x53100434) = 0x00098000; // Layer control 2
    *((volatile uint32_t*)0x53100438) = 0x00000138; // Mask count
    *((volatile uint32_t*)0x5310043c) = 0x000000a0; // Mask offset
    *((volatile uint32_t*)0x53100440) = 0x00000013; // Output channel count
    *((volatile uint32_t*)0x5310040c) = 0x00066000; // 1D
    *((volatile uint32_t*)0x53100444) = 0x0000001f; // TRAM ptr max
    *((volatile uint32_t*)0x5310044c) = 0x00024000; // Post processing register

    // Layer 4 quadrant 3
    *((volatile uint32_t*)0x54100404) = 0x0002801f; // Rows
    *((volatile uint32_t*)0x54100408) = 0x0001801f; // Columns
    *((volatile uint32_t*)0x54100418) = 0x00000020; // Stride
    *((volatile uint32_t*)0x5410041c) = 0x00058000; // SRAM write ptr
    *((volatile uint32_t*)0x54100424) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5410042c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x54100430) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x54100434) = 0x00098000; // Layer control 2
    *((volatile uint32_t*)0x54100438) = 0x00000138; // Mask count
    *((volatile uint32_t*)0x5410043c) = 0x000000a0; // Mask offset
    *((volatile uint32_t*)0x54100440) = 0x00000013; // Output channel count
    *((volatile uint32_t*)0x5410040c) = 0x00066000; // 1D
    *((volatile uint32_t*)0x54100444) = 0x0000001f; // TRAM ptr max
    *((volatile uint32_t*)0x5410044c) = 0x00024000; // Post processing register

    // Layer 5 quadrant 0
    *((volatile uint32_t*)0x51100504) = 0x0022801e; // Rows
    *((volatile uint32_t*)0x51100508) = 0x0002801e; // Columns
    *((volatile uint32_t*)0x51100510) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x51100514) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x51100518) = 0x00000021; // Stride
    *((volatile uint32_t*)0x5110051c) = 0x00028800; // SRAM write ptr
    *((volatile uint32_t*)0x51100524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x51100530) = 0x0088cba0; // Layer control
    *((volatile uint32_t*)0x51100534) = 0x00098000; // Layer control 2
    *((volatile uint32_t*)0x51100538) = 0x00000138; // Mask count
    *((volatile uint32_t*)0x5110053c) = 0x000000a0; // Mask offset
    *((volatile uint32_t*)0x51100540) = 0x00000013; // Output channel count
    *((volatile uint32_t*)0x51100544) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t*)0x5110054c) = 0x00022000; // Post processing register

    // Layer 5 quadrant 1
    *((volatile uint32_t*)0x52100504) = 0x0022801e; // Rows
    *((volatile uint32_t*)0x52100508) = 0x0002801e; // Columns
    *((volatile uint32_t*)0x52100510) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x52100514) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x52100518) = 0x00000021; // Stride
    *((volatile uint32_t*)0x5210051c) = 0x00028800; // SRAM write ptr
    *((volatile uint32_t*)0x52100524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x52100530) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x52100534) = 0x00098000; // Layer control 2
    *((volatile uint32_t*)0x52100538) = 0x00000138; // Mask count
    *((volatile uint32_t*)0x5210053c) = 0x000000a0; // Mask offset
    *((volatile uint32_t*)0x52100540) = 0x00000013; // Output channel count
    *((volatile uint32_t*)0x52100544) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t*)0x5210054c) = 0x00022000; // Post processing register

    // Layer 5 quadrant 2
    *((volatile uint32_t*)0x53100504) = 0x0022801e; // Rows
    *((volatile uint32_t*)0x53100508) = 0x0002801e; // Columns
    *((volatile uint32_t*)0x53100510) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x53100514) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x53100518) = 0x00000021; // Stride
    *((volatile uint32_t*)0x5310051c) = 0x00028800; // SRAM write ptr
    *((volatile uint32_t*)0x53100524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x53100530) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x53100534) = 0x00098000; // Layer control 2
    *((volatile uint32_t*)0x53100538) = 0x00000138; // Mask count
    *((volatile uint32_t*)0x5310053c) = 0x000000a0; // Mask offset
    *((volatile uint32_t*)0x53100540) = 0x00000013; // Output channel count
    *((volatile uint32_t*)0x53100544) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t*)0x5310054c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x53100548) = 0xf000f000; // Mask and processor enables

    // Layer 5 quadrant 3
    *((volatile uint32_t*)0x54100504) = 0x0022801e; // Rows
    *((volatile uint32_t*)0x54100508) = 0x0002801e; // Columns
    *((volatile uint32_t*)0x54100510) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x54100514) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x54100518) = 0x00000021; // Stride
    *((volatile uint32_t*)0x5410051c) = 0x00028800; // SRAM write ptr
    *((volatile uint32_t*)0x54100524) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x54100530) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x54100534) = 0x00098000; // Layer control 2
    *((volatile uint32_t*)0x54100538) = 0x00000138; // Mask count
    *((volatile uint32_t*)0x5410053c) = 0x000000a0; // Mask offset
    *((volatile uint32_t*)0x54100540) = 0x00000013; // Output channel count
    *((volatile uint32_t*)0x54100544) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t*)0x5410054c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x54100548) = 0xffffffff; // Mask and processor enables

    // Layer 6 quadrant 0
    *((volatile uint32_t*)0x51100604) = 0x0001000f; // Rows
    *((volatile uint32_t*)0x51100608) = 0x0001000f; // Columns
    *((volatile uint32_t*)0x51100618) = 0x00000010; // Stride
    *((volatile uint32_t*)0x51100620) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t*)0x5110062c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x51100630) = 0x00000920; // Layer control
    *((volatile uint32_t*)0x51100634) = 0x00000010; // Layer control 2
    *((volatile uint32_t*)0x51100640) = 0x00000002; // Output channel count
    *((volatile uint32_t*)0x5110060c) = 0x00000102; // 1D
    *((volatile uint32_t*)0x5110064c) = 0x03000000; // Post processing register

    // Layer 6 quadrant 1
    *((volatile uint32_t*)0x52100604) = 0x0001000f; // Rows
    *((volatile uint32_t*)0x52100608) = 0x0001000f; // Columns
    *((volatile uint32_t*)0x52100618) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5210061c) = 0x00028000; // SRAM write ptr
    *((volatile uint32_t*)0x52100620) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t*)0x5210062c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x52100630) = 0x00000920; // Layer control
    *((volatile uint32_t*)0x52100634) = 0x00000010; // Layer control 2
    *((volatile uint32_t*)0x52100640) = 0x00000002; // Output channel count
    *((volatile uint32_t*)0x5210060c) = 0x00000102; // 1D
    *((volatile uint32_t*)0x5210064c) = 0x03000000; // Post processing register
    *((volatile uint32_t*)0x52100648) = 0x0000fff0; // Mask and processor enables

    // Layer 6 quadrant 2
    *((volatile uint32_t*)0x53100604) = 0x0001000f; // Rows
    *((volatile uint32_t*)0x53100608) = 0x0001000f; // Columns
    *((volatile uint32_t*)0x53100618) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5310061c) = 0x00040000; // SRAM write ptr
    *((volatile uint32_t*)0x53100620) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t*)0x5310062c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x53100630) = 0x00000920; // Layer control
    *((volatile uint32_t*)0x53100634) = 0x00000010; // Layer control 2
    *((volatile uint32_t*)0x53100640) = 0x00000002; // Output channel count
    *((volatile uint32_t*)0x5310060c) = 0x00000102; // 1D
    *((volatile uint32_t*)0x5310064c) = 0x03000000; // Post processing register
    *((volatile uint32_t*)0x53100648) = 0x000000ff; // Mask and processor enables

    // Layer 6 quadrant 3
    *((volatile uint32_t*)0x54100604) = 0x0001000f; // Rows
    *((volatile uint32_t*)0x54100608) = 0x0001000f; // Columns
    *((volatile uint32_t*)0x54100618) = 0x00000010; // Stride
    *((volatile uint32_t*)0x54100620) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t*)0x5410062c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x54100630) = 0x00000920; // Layer control
    *((volatile uint32_t*)0x54100634) = 0x00000010; // Layer control 2
    *((volatile uint32_t*)0x54100640) = 0x00000002; // Output channel count
    *((volatile uint32_t*)0x5410060c) = 0x00000102; // 1D
    *((volatile uint32_t*)0x5410064c) = 0x03000000; // Post processing register

    // Layer 7 quadrant 0
    *((volatile uint32_t*)0x51100704) = 0x0001800f; // Rows
    *((volatile uint32_t*)0x51100708) = 0x0001800f; // Columns
    *((volatile uint32_t*)0x51100718) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5110071c) = 0x00028001; // SRAM write ptr
    *((volatile uint32_t*)0x51100724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5110072c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x51100730) = 0x00886b20; // Layer control
    *((volatile uint32_t*)0x51100734) = 0x00098010; // Layer control 2
    *((volatile uint32_t*)0x51100738) = 0x00000098; // Mask count
    *((volatile uint32_t*)0x51100740) = 0x00000013; // Output channel count
    *((volatile uint32_t*)0x51100744) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t*)0x5110074c) = 0x00024000; // Post processing register

    // Layer 7 quadrant 1
    *((volatile uint32_t*)0x52100704) = 0x0001800f; // Rows
    *((volatile uint32_t*)0x52100708) = 0x0001800f; // Columns
    *((volatile uint32_t*)0x52100718) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5210071c) = 0x00028001; // SRAM write ptr
    *((volatile uint32_t*)0x52100724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5210072c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x52100730) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x52100734) = 0x00098010; // Layer control 2
    *((volatile uint32_t*)0x52100738) = 0x00000098; // Mask count
    *((volatile uint32_t*)0x52100740) = 0x00000013; // Output channel count
    *((volatile uint32_t*)0x52100744) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t*)0x5210074c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x52100748) = 0xfff0fff0; // Mask and processor enables

    // Layer 7 quadrant 2
    *((volatile uint32_t*)0x53100704) = 0x0001800f; // Rows
    *((volatile uint32_t*)0x53100708) = 0x0001800f; // Columns
    *((volatile uint32_t*)0x53100718) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5310071c) = 0x00028001; // SRAM write ptr
    *((volatile uint32_t*)0x53100724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5310072c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x53100730) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x53100734) = 0x00098010; // Layer control 2
    *((volatile uint32_t*)0x53100738) = 0x00000098; // Mask count
    *((volatile uint32_t*)0x53100740) = 0x00000013; // Output channel count
    *((volatile uint32_t*)0x53100744) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t*)0x5310074c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x53100748) = 0x00ff00ff; // Mask and processor enables

    // Layer 7 quadrant 3
    *((volatile uint32_t*)0x54100704) = 0x0001800f; // Rows
    *((volatile uint32_t*)0x54100708) = 0x0001800f; // Columns
    *((volatile uint32_t*)0x54100718) = 0x00000010; // Stride
    *((volatile uint32_t*)0x5410071c) = 0x00028001; // SRAM write ptr
    *((volatile uint32_t*)0x54100724) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5410072c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x54100730) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x54100734) = 0x00098010; // Layer control 2
    *((volatile uint32_t*)0x54100738) = 0x00000098; // Mask count
    *((volatile uint32_t*)0x54100740) = 0x00000013; // Output channel count
    *((volatile uint32_t*)0x54100744) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t*)0x5410074c) = 0x00024000; // Post processing register

    // Layer 8 quadrant 0
    *((volatile uint32_t*)0x51100804) = 0x0002800f; // Rows
    *((volatile uint32_t*)0x51100808) = 0x0001800f; // Columns
    *((volatile uint32_t*)0x51100818) = 0x00000020; // Stride
    *((volatile uint32_t*)0x5110081c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t*)0x51100824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x51100830) = 0x00886b20; // Layer control
    *((volatile uint32_t*)0x51100834) = 0x00158000; // Layer control 2
    *((volatile uint32_t*)0x51100838) = 0x000001f8; // Mask count
    *((volatile uint32_t*)0x5110083c) = 0x000000a0; // Mask offset
    *((volatile uint32_t*)0x51100840) = 0x0000002b; // Output channel count
    *((volatile uint32_t*)0x5110080c) = 0x00066000; // 1D
    *((volatile uint32_t*)0x51100844) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t*)0x5110084c) = 0x00024000; // Post processing register

    // Layer 8 quadrant 1
    *((volatile uint32_t*)0x52100804) = 0x0002800f; // Rows
    *((volatile uint32_t*)0x52100808) = 0x0001800f; // Columns
    *((volatile uint32_t*)0x52100818) = 0x00000020; // Stride
    *((volatile uint32_t*)0x5210081c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t*)0x52100824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x52100830) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x52100834) = 0x00158000; // Layer control 2
    *((volatile uint32_t*)0x52100838) = 0x000001f8; // Mask count
    *((volatile uint32_t*)0x5210083c) = 0x000000a0; // Mask offset
    *((volatile uint32_t*)0x52100840) = 0x0000002b; // Output channel count
    *((volatile uint32_t*)0x5210080c) = 0x00066000; // 1D
    *((volatile uint32_t*)0x52100844) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t*)0x5210084c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x52100848) = 0xfff0fff0; // Mask and processor enables

    // Layer 8 quadrant 2
    *((volatile uint32_t*)0x53100804) = 0x0002800f; // Rows
    *((volatile uint32_t*)0x53100808) = 0x0001800f; // Columns
    *((volatile uint32_t*)0x53100818) = 0x00000020; // Stride
    *((volatile uint32_t*)0x5310081c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t*)0x53100824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x53100830) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x53100834) = 0x00158000; // Layer control 2
    *((volatile uint32_t*)0x53100838) = 0x000001f8; // Mask count
    *((volatile uint32_t*)0x5310083c) = 0x000000a0; // Mask offset
    *((volatile uint32_t*)0x53100840) = 0x0000002b; // Output channel count
    *((volatile uint32_t*)0x5310080c) = 0x00066000; // 1D
    *((volatile uint32_t*)0x53100844) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t*)0x5310084c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x53100848) = 0x00ff00ff; // Mask and processor enables

    // Layer 8 quadrant 3
    *((volatile uint32_t*)0x54100804) = 0x0002800f; // Rows
    *((volatile uint32_t*)0x54100808) = 0x0001800f; // Columns
    *((volatile uint32_t*)0x54100818) = 0x00000020; // Stride
    *((volatile uint32_t*)0x5410081c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t*)0x54100824) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x54100830) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x54100834) = 0x00158000; // Layer control 2
    *((volatile uint32_t*)0x54100838) = 0x000001f8; // Mask count
    *((volatile uint32_t*)0x5410083c) = 0x000000a0; // Mask offset
    *((volatile uint32_t*)0x54100840) = 0x0000002b; // Output channel count
    *((volatile uint32_t*)0x5410080c) = 0x00066000; // 1D
    *((volatile uint32_t*)0x54100844) = 0x0000000f; // TRAM ptr max
    *((volatile uint32_t*)0x5410084c) = 0x00024000; // Post processing register

    // Layer 9 quadrant 0
    *((volatile uint32_t*)0x51100904) = 0x0012800e; // Rows
    *((volatile uint32_t*)0x51100908) = 0x0002800e; // Columns
    *((volatile uint32_t*)0x51100910) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x51100914) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x51100918) = 0x00000021; // Stride
    *((volatile uint32_t*)0x51100924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5110092c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x51100930) = 0x00886ba0; // Layer control
    *((volatile uint32_t*)0x51100934) = 0x00178000; // Layer control 2
    *((volatile uint32_t*)0x51100938) = 0x00000378; // Mask count
    *((volatile uint32_t*)0x5110093c) = 0x00000200; // Mask offset
    *((volatile uint32_t*)0x51100940) = 0x0000002f; // Output channel count
    *((volatile uint32_t*)0x51100944) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t*)0x5110094c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x51100948) = 0xffffffff; // Mask and processor enables

    // Layer 9 quadrant 1
    *((volatile uint32_t*)0x52100904) = 0x0012800e; // Rows
    *((volatile uint32_t*)0x52100908) = 0x0002800e; // Columns
    *((volatile uint32_t*)0x52100910) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x52100914) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x52100918) = 0x00000021; // Stride
    *((volatile uint32_t*)0x52100924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5210092c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x52100930) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x52100934) = 0x00178000; // Layer control 2
    *((volatile uint32_t*)0x52100938) = 0x00000378; // Mask count
    *((volatile uint32_t*)0x5210093c) = 0x00000200; // Mask offset
    *((volatile uint32_t*)0x52100940) = 0x0000002f; // Output channel count
    *((volatile uint32_t*)0x52100944) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t*)0x5210094c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x52100948) = 0xffffffff; // Mask and processor enables

    // Layer 9 quadrant 2
    *((volatile uint32_t*)0x53100904) = 0x0012800e; // Rows
    *((volatile uint32_t*)0x53100908) = 0x0002800e; // Columns
    *((volatile uint32_t*)0x53100910) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x53100914) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x53100918) = 0x00000021; // Stride
    *((volatile uint32_t*)0x53100924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5310092c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x53100930) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x53100934) = 0x00178000; // Layer control 2
    *((volatile uint32_t*)0x53100938) = 0x00000378; // Mask count
    *((volatile uint32_t*)0x5310093c) = 0x00000200; // Mask offset
    *((volatile uint32_t*)0x53100940) = 0x0000002f; // Output channel count
    *((volatile uint32_t*)0x53100944) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t*)0x5310094c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x53100948) = 0x0fff0fff; // Mask and processor enables

    // Layer 9 quadrant 3
    *((volatile uint32_t*)0x54100904) = 0x0012800e; // Rows
    *((volatile uint32_t*)0x54100908) = 0x0002800e; // Columns
    *((volatile uint32_t*)0x54100910) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x54100914) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x54100918) = 0x00000021; // Stride
    *((volatile uint32_t*)0x54100924) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x5410092c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x54100930) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x54100934) = 0x00178000; // Layer control 2
    *((volatile uint32_t*)0x54100938) = 0x00000378; // Mask count
    *((volatile uint32_t*)0x5410093c) = 0x00000200; // Mask offset
    *((volatile uint32_t*)0x54100940) = 0x0000002f; // Output channel count
    *((volatile uint32_t*)0x54100944) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t*)0x5410094c) = 0x00024000; // Post processing register

    // Layer 10 quadrant 0
    *((volatile uint32_t*)0x51100a04) = 0x00010007; // Rows
    *((volatile uint32_t*)0x51100a08) = 0x00010007; // Columns
    *((volatile uint32_t*)0x51100a18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x51100a1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t*)0x51100a20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t*)0x51100a30) = 0x00000920; // Layer control
    *((volatile uint32_t*)0x51100a34) = 0x00000010; // Layer control 2
    *((volatile uint32_t*)0x51100a40) = 0x00000003; // Output channel count
    *((volatile uint32_t*)0x51100a0c) = 0x00000103; // 1D
    *((volatile uint32_t*)0x51100a4c) = 0x03000000; // Post processing register
    *((volatile uint32_t*)0x51100a48) = 0x0000ffff; // Mask and processor enables

    // Layer 10 quadrant 1
    *((volatile uint32_t*)0x52100a04) = 0x00010007; // Rows
    *((volatile uint32_t*)0x52100a08) = 0x00010007; // Columns
    *((volatile uint32_t*)0x52100a18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x52100a1c) = 0x00020800; // SRAM write ptr
    *((volatile uint32_t*)0x52100a20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t*)0x52100a30) = 0x00000920; // Layer control
    *((volatile uint32_t*)0x52100a34) = 0x00000010; // Layer control 2
    *((volatile uint32_t*)0x52100a40) = 0x00000003; // Output channel count
    *((volatile uint32_t*)0x52100a0c) = 0x00000103; // 1D
    *((volatile uint32_t*)0x52100a4c) = 0x03000000; // Post processing register
    *((volatile uint32_t*)0x52100a48) = 0x0000ffff; // Mask and processor enables

    // Layer 10 quadrant 2
    *((volatile uint32_t*)0x53100a04) = 0x00010007; // Rows
    *((volatile uint32_t*)0x53100a08) = 0x00010007; // Columns
    *((volatile uint32_t*)0x53100a18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x53100a1c) = 0x00040800; // SRAM write ptr
    *((volatile uint32_t*)0x53100a20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t*)0x53100a30) = 0x00000920; // Layer control
    *((volatile uint32_t*)0x53100a34) = 0x00000010; // Layer control 2
    *((volatile uint32_t*)0x53100a40) = 0x00000003; // Output channel count
    *((volatile uint32_t*)0x53100a0c) = 0x00000103; // 1D
    *((volatile uint32_t*)0x53100a4c) = 0x03000000; // Post processing register
    *((volatile uint32_t*)0x53100a48) = 0x0000ffff; // Mask and processor enables

    // Layer 10 quadrant 3
    *((volatile uint32_t*)0x54100a04) = 0x00010007; // Rows
    *((volatile uint32_t*)0x54100a08) = 0x00010007; // Columns
    *((volatile uint32_t*)0x54100a18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x54100a1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t*)0x54100a20) = 0x00008000; // Write ptr time slot offs
    *((volatile uint32_t*)0x54100a30) = 0x00000920; // Layer control
    *((volatile uint32_t*)0x54100a34) = 0x00000010; // Layer control 2
    *((volatile uint32_t*)0x54100a40) = 0x00000003; // Output channel count
    *((volatile uint32_t*)0x54100a0c) = 0x00000103; // 1D
    *((volatile uint32_t*)0x54100a4c) = 0x03000000; // Post processing register

    // Layer 11 quadrant 0
    *((volatile uint32_t*)0x51100b04) = 0x00018007; // Rows
    *((volatile uint32_t*)0x51100b08) = 0x00018007; // Columns
    *((volatile uint32_t*)0x51100b18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x51100b1c) = 0x00000801; // SRAM write ptr
    *((volatile uint32_t*)0x51100b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x51100b30) = 0x00886b20; // Layer control
    *((volatile uint32_t*)0x51100b34) = 0x00178010; // Layer control 2
    *((volatile uint32_t*)0x51100b38) = 0x000004f8; // Mask count
    *((volatile uint32_t*)0x51100b3c) = 0x00000380; // Mask offset
    *((volatile uint32_t*)0x51100b40) = 0x0000002f; // Output channel count
    *((volatile uint32_t*)0x51100b44) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t*)0x51100b4c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x51100b48) = 0xffffffff; // Mask and processor enables

    // Layer 11 quadrant 1
    *((volatile uint32_t*)0x52100b04) = 0x00018007; // Rows
    *((volatile uint32_t*)0x52100b08) = 0x00018007; // Columns
    *((volatile uint32_t*)0x52100b18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x52100b1c) = 0x00000801; // SRAM write ptr
    *((volatile uint32_t*)0x52100b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x52100b30) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x52100b34) = 0x00178010; // Layer control 2
    *((volatile uint32_t*)0x52100b38) = 0x000004f8; // Mask count
    *((volatile uint32_t*)0x52100b3c) = 0x00000380; // Mask offset
    *((volatile uint32_t*)0x52100b40) = 0x0000002f; // Output channel count
    *((volatile uint32_t*)0x52100b44) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t*)0x52100b4c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x52100b48) = 0xffffffff; // Mask and processor enables

    // Layer 11 quadrant 2
    *((volatile uint32_t*)0x53100b04) = 0x00018007; // Rows
    *((volatile uint32_t*)0x53100b08) = 0x00018007; // Columns
    *((volatile uint32_t*)0x53100b18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x53100b1c) = 0x00000801; // SRAM write ptr
    *((volatile uint32_t*)0x53100b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x53100b30) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x53100b34) = 0x00178010; // Layer control 2
    *((volatile uint32_t*)0x53100b38) = 0x000004f8; // Mask count
    *((volatile uint32_t*)0x53100b3c) = 0x00000380; // Mask offset
    *((volatile uint32_t*)0x53100b40) = 0x0000002f; // Output channel count
    *((volatile uint32_t*)0x53100b44) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t*)0x53100b4c) = 0x00024000; // Post processing register
    *((volatile uint32_t*)0x53100b48) = 0xffffffff; // Mask and processor enables

    // Layer 11 quadrant 3
    *((volatile uint32_t*)0x54100b04) = 0x00018007; // Rows
    *((volatile uint32_t*)0x54100b08) = 0x00018007; // Columns
    *((volatile uint32_t*)0x54100b18) = 0x00000010; // Stride
    *((volatile uint32_t*)0x54100b1c) = 0x00000801; // SRAM write ptr
    *((volatile uint32_t*)0x54100b24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x54100b30) = 0x00880b20; // Layer control
    *((volatile uint32_t*)0x54100b34) = 0x00178010; // Layer control 2
    *((volatile uint32_t*)0x54100b38) = 0x000004f8; // Mask count
    *((volatile uint32_t*)0x54100b3c) = 0x00000380; // Mask offset
    *((volatile uint32_t*)0x54100b40) = 0x0000002f; // Output channel count
    *((volatile uint32_t*)0x54100b44) = 0x00000007; // TRAM ptr max
    *((volatile uint32_t*)0x54100b4c) = 0x00024000; // Post processing register

    // Layer 12 quadrant 0
    *((volatile uint32_t*)0x51100c04) = 0x00148006; // Rows
    *((volatile uint32_t*)0x51100c08) = 0x00028006; // Columns
    *((volatile uint32_t*)0x51100c10) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x51100c14) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x51100c18) = 0x00000041; // Stride
    *((volatile uint32_t*)0x51100c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x51100c28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t*)0x51100c2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x51100c30) = 0x00886ba0; // Layer control
    *((volatile uint32_t*)0x51100c34) = 0x00178010; // Layer control 2
    *((volatile uint32_t*)0x51100c38) = 0x000007f8; // Mask count
    *((volatile uint32_t*)0x51100c3c) = 0x00000500; // Mask offset
    *((volatile uint32_t*)0x51100c40) = 0x0000005f; // Output channel count
    *((volatile uint32_t*)0x51100c0c) = 0x00066000; // 1D
    *((volatile uint32_t*)0x51100c44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t*)0x51100c4c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x51100c48) = 0xffffffff; // Mask and processor enables

    // Layer 12 quadrant 1
    *((volatile uint32_t*)0x52100c04) = 0x00148006; // Rows
    *((volatile uint32_t*)0x52100c08) = 0x00028006; // Columns
    *((volatile uint32_t*)0x52100c10) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x52100c14) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x52100c18) = 0x00000041; // Stride
    *((volatile uint32_t*)0x52100c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x52100c28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t*)0x52100c2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x52100c30) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x52100c34) = 0x00178010; // Layer control 2
    *((volatile uint32_t*)0x52100c38) = 0x000007f8; // Mask count
    *((volatile uint32_t*)0x52100c3c) = 0x00000500; // Mask offset
    *((volatile uint32_t*)0x52100c40) = 0x0000005f; // Output channel count
    *((volatile uint32_t*)0x52100c0c) = 0x00066000; // 1D
    *((volatile uint32_t*)0x52100c44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t*)0x52100c4c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x52100c48) = 0xffffffff; // Mask and processor enables

    // Layer 12 quadrant 2
    *((volatile uint32_t*)0x53100c04) = 0x00148006; // Rows
    *((volatile uint32_t*)0x53100c08) = 0x00028006; // Columns
    *((volatile uint32_t*)0x53100c10) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x53100c14) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x53100c18) = 0x00000041; // Stride
    *((volatile uint32_t*)0x53100c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x53100c28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t*)0x53100c2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x53100c30) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x53100c34) = 0x00178010; // Layer control 2
    *((volatile uint32_t*)0x53100c38) = 0x000007f8; // Mask count
    *((volatile uint32_t*)0x53100c3c) = 0x00000500; // Mask offset
    *((volatile uint32_t*)0x53100c40) = 0x0000005f; // Output channel count
    *((volatile uint32_t*)0x53100c0c) = 0x00066000; // 1D
    *((volatile uint32_t*)0x53100c44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t*)0x53100c4c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x53100c48) = 0xffffffff; // Mask and processor enables

    // Layer 12 quadrant 3
    *((volatile uint32_t*)0x54100c04) = 0x00148006; // Rows
    *((volatile uint32_t*)0x54100c08) = 0x00028006; // Columns
    *((volatile uint32_t*)0x54100c10) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x54100c14) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x54100c18) = 0x00000041; // Stride
    *((volatile uint32_t*)0x54100c24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x54100c28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t*)0x54100c2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x54100c30) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x54100c34) = 0x00178010; // Layer control 2
    *((volatile uint32_t*)0x54100c38) = 0x000007f8; // Mask count
    *((volatile uint32_t*)0x54100c3c) = 0x00000500; // Mask offset
    *((volatile uint32_t*)0x54100c40) = 0x0000005f; // Output channel count
    *((volatile uint32_t*)0x54100c0c) = 0x00066000; // 1D
    *((volatile uint32_t*)0x54100c44) = 0x00000003; // TRAM ptr max
    *((volatile uint32_t*)0x54100c4c) = 0x00022000; // Post processing register

    // Layer 13 quadrant 0
    *((volatile uint32_t*)0x51100d04) = 0x000c0002; // Rows
    *((volatile uint32_t*)0x51100d08) = 0x00020002; // Columns
    *((volatile uint32_t*)0x51100d10) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x51100d14) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x51100d18) = 0x00000041; // Stride
    *((volatile uint32_t*)0x51100d1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t*)0x51100d20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t*)0x51100d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x51100d28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t*)0x51100d30) = 0x00006ba0; // Layer control
    *((volatile uint32_t*)0x51100d34) = 0x001f8071; // Layer control 2
    *((volatile uint32_t*)0x51100d38) = 0x000067f8; // Mask count
    *((volatile uint32_t*)0x51100d3c) = 0x00004800; // Mask offset
    *((volatile uint32_t*)0x51100d40) = 0x000003ff; // Output channel count
    *((volatile uint32_t*)0x51100d0c) = 0x00000100; // 1D
    *((volatile uint32_t*)0x51100d4c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x51100d48) = 0xffffffff; // Mask and processor enables

    // Layer 13 quadrant 1
    *((volatile uint32_t*)0x52100d04) = 0x000c0002; // Rows
    *((volatile uint32_t*)0x52100d08) = 0x00020002; // Columns
    *((volatile uint32_t*)0x52100d10) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x52100d14) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x52100d18) = 0x00000041; // Stride
    *((volatile uint32_t*)0x52100d1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t*)0x52100d20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t*)0x52100d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x52100d28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t*)0x52100d30) = 0x00000ba0; // Layer control
    *((volatile uint32_t*)0x52100d34) = 0x001f8071; // Layer control 2
    *((volatile uint32_t*)0x52100d38) = 0x000067f8; // Mask count
    *((volatile uint32_t*)0x52100d3c) = 0x00004800; // Mask offset
    *((volatile uint32_t*)0x52100d40) = 0x000003ff; // Output channel count
    *((volatile uint32_t*)0x52100d0c) = 0x00000100; // 1D
    *((volatile uint32_t*)0x52100d4c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x52100d48) = 0xffffffff; // Mask and processor enables

    // Layer 13 quadrant 2
    *((volatile uint32_t*)0x53100d04) = 0x000c0002; // Rows
    *((volatile uint32_t*)0x53100d08) = 0x00020002; // Columns
    *((volatile uint32_t*)0x53100d10) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x53100d14) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x53100d18) = 0x00000041; // Stride
    *((volatile uint32_t*)0x53100d1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t*)0x53100d20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t*)0x53100d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x53100d28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t*)0x53100d30) = 0x00000ba0; // Layer control
    *((volatile uint32_t*)0x53100d34) = 0x001f8071; // Layer control 2
    *((volatile uint32_t*)0x53100d38) = 0x000067f8; // Mask count
    *((volatile uint32_t*)0x53100d3c) = 0x00004800; // Mask offset
    *((volatile uint32_t*)0x53100d40) = 0x000003ff; // Output channel count
    *((volatile uint32_t*)0x53100d0c) = 0x00000100; // 1D
    *((volatile uint32_t*)0x53100d4c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x53100d48) = 0xffffffff; // Mask and processor enables

    // Layer 13 quadrant 3
    *((volatile uint32_t*)0x54100d04) = 0x000c0002; // Rows
    *((volatile uint32_t*)0x54100d08) = 0x00020002; // Columns
    *((volatile uint32_t*)0x54100d10) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x54100d14) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x54100d18) = 0x00000041; // Stride
    *((volatile uint32_t*)0x54100d1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t*)0x54100d20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t*)0x54100d24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x54100d28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t*)0x54100d30) = 0x00000ba0; // Layer control
    *((volatile uint32_t*)0x54100d34) = 0x001f8071; // Layer control 2
    *((volatile uint32_t*)0x54100d38) = 0x000067f8; // Mask count
    *((volatile uint32_t*)0x54100d3c) = 0x00004800; // Mask offset
    *((volatile uint32_t*)0x54100d40) = 0x000003ff; // Output channel count
    *((volatile uint32_t*)0x54100d0c) = 0x00000100; // 1D
    *((volatile uint32_t*)0x54100d4c) = 0x00022000; // Post processing register

    // Layer 14 quadrant 0
    *((volatile uint32_t*)0x51100e04) = 0x00080001; // Rows
    *((volatile uint32_t*)0x51100e08) = 0x00010001; // Columns
    *((volatile uint32_t*)0x51100e18) = 0x00000080; // Stride
    *((volatile uint32_t*)0x51100e20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t*)0x51100e24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x51100e28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t*)0x51100e2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x51100e30) = 0x0000eb20; // Layer control
    *((volatile uint32_t*)0x51100e34) = 0x001f8017; // Layer control 2
    *((volatile uint32_t*)0x51100e38) = 0x00008898; // Mask count
    *((volatile uint32_t*)0x51100e3c) = 0x000068a0; // Mask offset
    *((volatile uint32_t*)0x51100e40) = 0x000003ff; // Output channel count
    *((volatile uint32_t*)0x51100e0c) = 0x00000100; // 1D
    *((volatile uint32_t*)0x51100e4c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x51100e48) = 0xffffffff; // Mask and processor enables

    // Layer 14 quadrant 1
    *((volatile uint32_t*)0x52100e04) = 0x00080001; // Rows
    *((volatile uint32_t*)0x52100e08) = 0x00010001; // Columns
    *((volatile uint32_t*)0x52100e18) = 0x00000080; // Stride
    *((volatile uint32_t*)0x52100e20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t*)0x52100e24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x52100e28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t*)0x52100e2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x52100e30) = 0x00000b20; // Layer control
    *((volatile uint32_t*)0x52100e34) = 0x001f8017; // Layer control 2
    *((volatile uint32_t*)0x52100e38) = 0x00008898; // Mask count
    *((volatile uint32_t*)0x52100e3c) = 0x000068a0; // Mask offset
    *((volatile uint32_t*)0x52100e40) = 0x000003ff; // Output channel count
    *((volatile uint32_t*)0x52100e0c) = 0x00000100; // 1D
    *((volatile uint32_t*)0x52100e4c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x52100e48) = 0xffffffff; // Mask and processor enables

    // Layer 14 quadrant 2
    *((volatile uint32_t*)0x53100e04) = 0x00080001; // Rows
    *((volatile uint32_t*)0x53100e08) = 0x00010001; // Columns
    *((volatile uint32_t*)0x53100e18) = 0x00000080; // Stride
    *((volatile uint32_t*)0x53100e20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t*)0x53100e24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x53100e28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t*)0x53100e2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x53100e30) = 0x00000b20; // Layer control
    *((volatile uint32_t*)0x53100e34) = 0x001f8017; // Layer control 2
    *((volatile uint32_t*)0x53100e38) = 0x00008898; // Mask count
    *((volatile uint32_t*)0x53100e3c) = 0x000068a0; // Mask offset
    *((volatile uint32_t*)0x53100e40) = 0x000003ff; // Output channel count
    *((volatile uint32_t*)0x53100e0c) = 0x00000100; // 1D
    *((volatile uint32_t*)0x53100e4c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x53100e48) = 0xffffffff; // Mask and processor enables

    // Layer 14 quadrant 3
    *((volatile uint32_t*)0x54100e04) = 0x00080001; // Rows
    *((volatile uint32_t*)0x54100e08) = 0x00010001; // Columns
    *((volatile uint32_t*)0x54100e18) = 0x00000080; // Stride
    *((volatile uint32_t*)0x54100e20) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t*)0x54100e24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x54100e28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t*)0x54100e2c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x54100e30) = 0x00000b20; // Layer control
    *((volatile uint32_t*)0x54100e34) = 0x001f8017; // Layer control 2
    *((volatile uint32_t*)0x54100e38) = 0x00008898; // Mask count
    *((volatile uint32_t*)0x54100e3c) = 0x000068a0; // Mask offset
    *((volatile uint32_t*)0x54100e40) = 0x000003ff; // Output channel count
    *((volatile uint32_t*)0x54100e0c) = 0x00000100; // 1D
    *((volatile uint32_t*)0x54100e4c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x54100e48) = 0xffffffff; // Mask and processor enables

    // Layer 15 quadrant 0
    *((volatile uint32_t*)0x51100f04) = 0x00088000; // Rows
    *((volatile uint32_t*)0x51100f08) = 0x00028000; // Columns
    *((volatile uint32_t*)0x51100f10) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x51100f14) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x51100f18) = 0x00000041; // Stride
    *((volatile uint32_t*)0x51100f1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t*)0x51100f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x51100f28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t*)0x51100f30) = 0x0088eba0; // Layer control
    *((volatile uint32_t*)0x51100f34) = 0x001f8011; // Layer control 2
    *((volatile uint32_t*)0x51100f38) = 0x00001738; // Mask count
    *((volatile uint32_t*)0x51100f3c) = 0x00000f40; // Mask offset
    *((volatile uint32_t*)0x51100f40) = 0x000000ff; // Output channel count
    *((volatile uint32_t*)0x51100f4c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x51100f48) = 0xffffffff; // Mask and processor enables

    // Layer 15 quadrant 1
    *((volatile uint32_t*)0x52100f04) = 0x00088000; // Rows
    *((volatile uint32_t*)0x52100f08) = 0x00028000; // Columns
    *((volatile uint32_t*)0x52100f10) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x52100f14) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x52100f18) = 0x00000041; // Stride
    *((volatile uint32_t*)0x52100f1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t*)0x52100f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x52100f28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t*)0x52100f30) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x52100f34) = 0x001f8011; // Layer control 2
    *((volatile uint32_t*)0x52100f38) = 0x00001738; // Mask count
    *((volatile uint32_t*)0x52100f3c) = 0x00000f40; // Mask offset
    *((volatile uint32_t*)0x52100f40) = 0x000000ff; // Output channel count
    *((volatile uint32_t*)0x52100f4c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x52100f48) = 0xffffffff; // Mask and processor enables

    // Layer 15 quadrant 2
    *((volatile uint32_t*)0x53100f04) = 0x00088000; // Rows
    *((volatile uint32_t*)0x53100f08) = 0x00028000; // Columns
    *((volatile uint32_t*)0x53100f10) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x53100f14) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x53100f18) = 0x00000041; // Stride
    *((volatile uint32_t*)0x53100f1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t*)0x53100f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x53100f28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t*)0x53100f30) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x53100f34) = 0x001f8011; // Layer control 2
    *((volatile uint32_t*)0x53100f38) = 0x00001738; // Mask count
    *((volatile uint32_t*)0x53100f3c) = 0x00000f40; // Mask offset
    *((volatile uint32_t*)0x53100f40) = 0x000000ff; // Output channel count
    *((volatile uint32_t*)0x53100f4c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x53100f48) = 0xffffffff; // Mask and processor enables

    // Layer 15 quadrant 3
    *((volatile uint32_t*)0x54100f04) = 0x00088000; // Rows
    *((volatile uint32_t*)0x54100f08) = 0x00028000; // Columns
    *((volatile uint32_t*)0x54100f10) = 0x00000001; // Pooling rows
    *((volatile uint32_t*)0x54100f14) = 0x00000001; // Pooling columns
    *((volatile uint32_t*)0x54100f18) = 0x00000041; // Stride
    *((volatile uint32_t*)0x54100f1c) = 0x00000800; // SRAM write ptr
    *((volatile uint32_t*)0x54100f24) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x54100f28) = 0x00000001; // Write ptr multi-pass channel offs
    *((volatile uint32_t*)0x54100f30) = 0x00880ba0; // Layer control
    *((volatile uint32_t*)0x54100f34) = 0x001f8011; // Layer control 2
    *((volatile uint32_t*)0x54100f38) = 0x00001738; // Mask count
    *((volatile uint32_t*)0x54100f3c) = 0x00000f40; // Mask offset
    *((volatile uint32_t*)0x54100f40) = 0x000000ff; // Output channel count
    *((volatile uint32_t*)0x54100f4c) = 0x00022000; // Post processing register
    *((volatile uint32_t*)0x54100f48) = 0xffffffff; // Mask and processor enables

    // Layer 16 quadrant 0
    *((volatile uint32_t*)0x51101004) = 0x00020000; // Rows
    *((volatile uint32_t*)0x51101008) = 0x00010000; // Columns
    *((volatile uint32_t*)0x51101018) = 0x00000020; // Stride
    *((volatile uint32_t*)0x51101020) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t*)0x51101024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x51101028) = 0x00000004; // Write ptr multi-pass channel offs
    *((volatile uint32_t*)0x5110102c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x51101030) = 0x0001e920; // Layer control
    *((volatile uint32_t*)0x51101034) = 0x00198011; // Layer control 2
    *((volatile uint32_t*)0x51101038) = 0x0000d7b8; // Mask count
    *((volatile uint32_t*)0x5110103c) = 0x0000d140; // Mask offset
    *((volatile uint32_t*)0x51101040) = 0x000000cf; // Output channel count
    *((volatile uint32_t*)0x5110100c) = 0x00000100; // 1D
    *((volatile uint32_t*)0x51101048) = 0xffffffff; // Mask and processor enables

    // Layer 16 quadrant 1
    *((volatile uint32_t*)0x52101004) = 0x00020000; // Rows
    *((volatile uint32_t*)0x52101008) = 0x00010000; // Columns
    *((volatile uint32_t*)0x52101018) = 0x00000020; // Stride
    *((volatile uint32_t*)0x52101020) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t*)0x52101024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x52101028) = 0x00000004; // Write ptr multi-pass channel offs
    *((volatile uint32_t*)0x5210102c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x52101030) = 0x00010920; // Layer control
    *((volatile uint32_t*)0x52101034) = 0x00198011; // Layer control 2
    *((volatile uint32_t*)0x52101038) = 0x0000d7b8; // Mask count
    *((volatile uint32_t*)0x5210103c) = 0x0000d140; // Mask offset
    *((volatile uint32_t*)0x52101040) = 0x000000cf; // Output channel count
    *((volatile uint32_t*)0x5210100c) = 0x00000100; // 1D
    *((volatile uint32_t*)0x52101048) = 0xffffffff; // Mask and processor enables

    // Layer 16 quadrant 2
    *((volatile uint32_t*)0x53101004) = 0x00020000; // Rows
    *((volatile uint32_t*)0x53101008) = 0x00010000; // Columns
    *((volatile uint32_t*)0x53101018) = 0x00000020; // Stride
    *((volatile uint32_t*)0x53101020) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t*)0x53101024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x53101028) = 0x00000004; // Write ptr multi-pass channel offs
    *((volatile uint32_t*)0x5310102c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x53101030) = 0x00010920; // Layer control
    *((volatile uint32_t*)0x53101034) = 0x00198011; // Layer control 2
    *((volatile uint32_t*)0x53101038) = 0x0000d7b8; // Mask count
    *((volatile uint32_t*)0x5310103c) = 0x0000d140; // Mask offset
    *((volatile uint32_t*)0x53101040) = 0x000000cf; // Output channel count
    *((volatile uint32_t*)0x5310100c) = 0x00000100; // 1D
    *((volatile uint32_t*)0x53101048) = 0xffffffff; // Mask and processor enables

    // Layer 16 quadrant 3
    *((volatile uint32_t*)0x54101004) = 0x00020000; // Rows
    *((volatile uint32_t*)0x54101008) = 0x00010000; // Columns
    *((volatile uint32_t*)0x54101018) = 0x00000020; // Stride
    *((volatile uint32_t*)0x54101020) = 0x00000001; // Write ptr time slot offs
    *((volatile uint32_t*)0x54101024) = 0x00008000; // Write ptr mask offs
    *((volatile uint32_t*)0x54101028) = 0x00000004; // Write ptr multi-pass channel offs
    *((volatile uint32_t*)0x5410102c) = 0x00000800; // SRAM read ptr
    *((volatile uint32_t*)0x54101030) = 0x00010920; // Layer control
    *((volatile uint32_t*)0x54101034) = 0x00198011; // Layer control 2
    *((volatile uint32_t*)0x54101038) = 0x0000d7b8; // Mask count
    *((volatile uint32_t*)0x5410103c) = 0x0000d140; // Mask offset
    *((volatile uint32_t*)0x54101040) = 0x000000cf; // Output channel count
    *((volatile uint32_t*)0x5410100c) = 0x00000100; // 1D
    *((volatile uint32_t*)0x54101048) = 0xffffffff; // Mask and processor enables

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

int cnn_unload(uint32_t* out_buf)
{
    volatile uint32_t* addr;

    // Custom unload for this network, layer 16: 32-bit data, shape: (100, 1, 1)
    addr       = (volatile uint32_t*)0x51800000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr       = (volatile uint32_t*)0x51820000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr       = (volatile uint32_t*)0x51840000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr       = (volatile uint32_t*)0x51860000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr       = (volatile uint32_t*)0x52800000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr       = (volatile uint32_t*)0x52820000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr       = (volatile uint32_t*)0x52840000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr       = (volatile uint32_t*)0x52860000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr       = (volatile uint32_t*)0x53800000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr       = (volatile uint32_t*)0x53820000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr       = (volatile uint32_t*)0x53840000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr       = (volatile uint32_t*)0x53860000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr       = (volatile uint32_t*)0x54800000;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr       = (volatile uint32_t*)0x51800010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr       = (volatile uint32_t*)0x51820010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr       = (volatile uint32_t*)0x51840010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr       = (volatile uint32_t*)0x51860010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr       = (volatile uint32_t*)0x52800010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr       = (volatile uint32_t*)0x52820010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr       = (volatile uint32_t*)0x52840010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr       = (volatile uint32_t*)0x52860010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr       = (volatile uint32_t*)0x53800010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr       = (volatile uint32_t*)0x53820010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr       = (volatile uint32_t*)0x53840010;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    *out_buf++ = *addr++;
    addr       = (volatile uint32_t*)0x53860010;
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
