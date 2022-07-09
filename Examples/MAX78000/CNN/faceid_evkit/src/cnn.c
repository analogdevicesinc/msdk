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

// faceid
// Created using ./ai8xize.py --verbose --log --test-dir sdk/Examples/MAX78000/CNN --prefix faceid --checkpoint-file trained/ai85-faceid-qat8-q.pth.tar --config-file networks/faceid.yaml --fifo --device MAX78000 --compact-data --mexpress --timer 0 --display-checkpoint

// DO NOT EDIT - regenerate this file instead!

// Configuring 9 layers:
// Layer 0: 3x160x120 (streaming HWC data), no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, 16x160x120 output
// Layer 1: 16x160x120 (streaming HWC data), 2x2 max pool with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, 32x80x60 output
// Layer 2: 32x80x60 (HWC data), 2x2 max pool with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, 32x40x30 output
// Layer 3: 32x40x30 (HWC data), 2x2 max pool with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, 64x20x15 output
// Layer 4: 64x20x15 (HWC data), 2x2 max pool with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, 64x10x7 output
// Layer 5: 64x10x7 (HWC data), no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, 64x10x7 output
// Layer 6: 64x10x7 (HWC data), no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, 64x10x7 output
// Layer 7: 64x10x7 (HWC data), 2x2 max pool with stride 2/2, conv2d with kernel size 1x1, stride 1/1, pad 0/0, 512x5x3 output
// Layer 8: 512x5x3 (HWC data), 5x3 avg pool with stride 1/1, no convolution, 512x1x1 output

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
  // Acknowledge interrupt to all groups
  *((volatile uint32_t *) 0x50100000) &= ~((1<<12) | 1);
  *((volatile uint32_t *) 0x50500000) &= ~((1<<12) | 1);
  *((volatile uint32_t *) 0x50900000) &= ~((1<<12) | 1);
  *((volatile uint32_t *) 0x50d00000) &= ~((1<<12) | 1);

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

  *((volatile uint32_t *) 0x50100000) |= 1; // Re-enable group 0

  return CNN_OK;
}

int cnn_stop(void)
{
  *((volatile uint32_t *) 0x50100000) &= ~1; // Disable group 0

  return CNN_OK;
}

void memcpy32(uint32_t *dst, const uint32_t *src, int n)
{
  while (n-- > 0) {
    *dst++ = *src++;
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

int cnn_load_weights(void)
{
  *((volatile uint8_t *) 0x50180001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50180000, kernels_0, 705);
  *((volatile uint8_t *) 0x50184001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50184000, kernels_1, 705);
  *((volatile uint8_t *) 0x50188001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50188000, kernels_2, 705);
  *((volatile uint8_t *) 0x5018c081) = 0x01; // Set address
  memcpy32((uint32_t *) 0x5018c000, kernels_3, 633);
  *((volatile uint8_t *) 0x50190001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50190000, kernels_4, 705);
  *((volatile uint8_t *) 0x50194001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50194000, kernels_5, 705);
  *((volatile uint8_t *) 0x50198001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50198000, kernels_6, 705);
  *((volatile uint8_t *) 0x5019c001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x5019c000, kernels_7, 705);
  *((volatile uint8_t *) 0x501a0001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x501a0000, kernels_8, 705);
  *((volatile uint8_t *) 0x501a4001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x501a4000, kernels_9, 705);
  *((volatile uint8_t *) 0x501a8001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x501a8000, kernels_10, 705);
  *((volatile uint8_t *) 0x501ac001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x501ac000, kernels_11, 705);
  *((volatile uint8_t *) 0x501b0001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x501b0000, kernels_12, 705);
  *((volatile uint8_t *) 0x501b4001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x501b4000, kernels_13, 705);
  *((volatile uint8_t *) 0x501b8001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x501b8000, kernels_14, 705);
  *((volatile uint8_t *) 0x501bc001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x501bc000, kernels_15, 705);
  *((volatile uint8_t *) 0x50580001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50580000, kernels_16, 705);
  *((volatile uint8_t *) 0x50584001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50584000, kernels_17, 705);
  *((volatile uint8_t *) 0x50588001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50588000, kernels_18, 705);
  *((volatile uint8_t *) 0x5058c001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x5058c000, kernels_19, 705);
  *((volatile uint8_t *) 0x50590081) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50590000, kernels_20, 633);
  *((volatile uint8_t *) 0x50594081) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50594000, kernels_21, 633);
  *((volatile uint8_t *) 0x50598081) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50598000, kernels_22, 633);
  *((volatile uint8_t *) 0x5059c081) = 0x01; // Set address
  memcpy32((uint32_t *) 0x5059c000, kernels_23, 633);
  *((volatile uint8_t *) 0x505a0081) = 0x01; // Set address
  memcpy32((uint32_t *) 0x505a0000, kernels_24, 633);
  *((volatile uint8_t *) 0x505a4081) = 0x01; // Set address
  memcpy32((uint32_t *) 0x505a4000, kernels_25, 633);
  *((volatile uint8_t *) 0x505a8081) = 0x01; // Set address
  memcpy32((uint32_t *) 0x505a8000, kernels_26, 633);
  *((volatile uint8_t *) 0x505ac081) = 0x01; // Set address
  memcpy32((uint32_t *) 0x505ac000, kernels_27, 633);
  *((volatile uint8_t *) 0x505b0081) = 0x01; // Set address
  memcpy32((uint32_t *) 0x505b0000, kernels_28, 633);
  *((volatile uint8_t *) 0x505b4081) = 0x01; // Set address
  memcpy32((uint32_t *) 0x505b4000, kernels_29, 633);
  *((volatile uint8_t *) 0x505b8081) = 0x01; // Set address
  memcpy32((uint32_t *) 0x505b8000, kernels_30, 633);
  *((volatile uint8_t *) 0x505bc081) = 0x01; // Set address
  memcpy32((uint32_t *) 0x505bc000, kernels_31, 633);
  *((volatile uint8_t *) 0x50980001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50980000, kernels_32, 705);
  *((volatile uint8_t *) 0x50984001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50984000, kernels_33, 705);
  *((volatile uint8_t *) 0x50988001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50988000, kernels_34, 705);
  *((volatile uint8_t *) 0x5098c001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x5098c000, kernels_35, 705);
  *((volatile uint8_t *) 0x50990001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50990000, kernels_36, 705);
  *((volatile uint8_t *) 0x50994001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50994000, kernels_37, 705);
  *((volatile uint8_t *) 0x50998001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50998000, kernels_38, 705);
  *((volatile uint8_t *) 0x5099c001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x5099c000, kernels_39, 705);
  *((volatile uint8_t *) 0x509a0001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x509a0000, kernels_40, 705);
  *((volatile uint8_t *) 0x509a4001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x509a4000, kernels_41, 705);
  *((volatile uint8_t *) 0x509a8001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x509a8000, kernels_42, 705);
  *((volatile uint8_t *) 0x509ac001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x509ac000, kernels_43, 705);
  *((volatile uint8_t *) 0x509b0001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x509b0000, kernels_44, 705);
  *((volatile uint8_t *) 0x509b4001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x509b4000, kernels_45, 705);
  *((volatile uint8_t *) 0x509b8001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x509b8000, kernels_46, 705);
  *((volatile uint8_t *) 0x509bc001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x509bc000, kernels_47, 705);
  *((volatile uint8_t *) 0x50d80001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50d80000, kernels_48, 705);
  *((volatile uint8_t *) 0x50d84001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50d84000, kernels_49, 705);
  *((volatile uint8_t *) 0x50d88001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50d88000, kernels_50, 705);
  *((volatile uint8_t *) 0x50d8c001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50d8c000, kernels_51, 705);
  *((volatile uint8_t *) 0x50d90001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50d90000, kernels_52, 705);
  *((volatile uint8_t *) 0x50d94001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50d94000, kernels_53, 705);
  *((volatile uint8_t *) 0x50d98001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50d98000, kernels_54, 705);
  *((volatile uint8_t *) 0x50d9c001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50d9c000, kernels_55, 705);
  *((volatile uint8_t *) 0x50da0001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50da0000, kernels_56, 705);
  *((volatile uint8_t *) 0x50da4001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50da4000, kernels_57, 705);
  *((volatile uint8_t *) 0x50da8001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50da8000, kernels_58, 705);
  *((volatile uint8_t *) 0x50dac001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50dac000, kernels_59, 705);
  *((volatile uint8_t *) 0x50db0001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50db0000, kernels_60, 705);
  *((volatile uint8_t *) 0x50db4001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50db4000, kernels_61, 705);
  *((volatile uint8_t *) 0x50db8001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50db8000, kernels_62, 705);
  *((volatile uint8_t *) 0x50dbc001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50dbc000, kernels_63, 705);

  return CNN_OK;
}

int cnn_load_bias(void)
{
  // Not used in this network
  return CNN_OK;
}

int cnn_init(void)
{
  *((volatile uint32_t *) 0x50001000) = 0x00000000; // AON control
  *((volatile uint32_t *) 0x50100000) = 0x00108008; // Stop SM
  *((volatile uint32_t *) 0x50100004) = 0x0000040e; // SRAM control
  *((volatile uint32_t *) 0x50100008) = 0x00000008; // Layer count
  *((volatile uint32_t *) 0x50500000) = 0x00108008; // Stop SM
  *((volatile uint32_t *) 0x50500004) = 0x0000040e; // SRAM control
  *((volatile uint32_t *) 0x50500008) = 0x00000008; // Layer count
  *((volatile uint32_t *) 0x50900000) = 0x00108008; // Stop SM
  *((volatile uint32_t *) 0x50900004) = 0x0000040e; // SRAM control
  *((volatile uint32_t *) 0x50900008) = 0x00000008; // Layer count
  *((volatile uint32_t *) 0x50d00000) = 0x00108008; // Stop SM
  *((volatile uint32_t *) 0x50d00004) = 0x0000040e; // SRAM control
  *((volatile uint32_t *) 0x50d00008) = 0x00000008; // Layer count

  return CNN_OK;
}

int cnn_configure(void)
{
  // Layer 0 group 0
  *((volatile uint32_t *) 0x50100010) = 0x000100a1; // Rows
  *((volatile uint32_t *) 0x50100090) = 0x00010079; // Columns
  *((volatile uint32_t *) 0x50100310) = 0x00002400; // SRAM write ptr
  *((volatile uint32_t *) 0x50100410) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50100490) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50100590) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50100a10) = 0x00007800; // Layer control 2
  *((volatile uint32_t *) 0x50100610) = 0x00000078; // Mask offset and count
  *((volatile uint32_t *) 0x50100690) = 0x00000077; // TRAM ptr max
  *((volatile uint32_t *) 0x50100790) = 0x00022000; // Post processing register
  *((volatile uint32_t *) 0x50100710) = 0x00070007; // Mask and processor enables
  *((volatile uint32_t *) 0x50100810) = 0x00000001; // Stream processing start
  *((volatile uint32_t *) 0x50100910) = 0x00000002; // Rollover
  *((volatile uint32_t *) 0x50100990) = 0x00004b00; // Input frame size

  // Layer 0 group 1
  *((volatile uint32_t *) 0x50500010) = 0x000100a1; // Rows
  *((volatile uint32_t *) 0x50500090) = 0x00010079; // Columns
  *((volatile uint32_t *) 0x50500310) = 0x00002400; // SRAM write ptr
  *((volatile uint32_t *) 0x50500410) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50500490) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50500590) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50500a10) = 0x00007800; // Layer control 2
  *((volatile uint32_t *) 0x50500610) = 0x00000078; // Mask offset and count
  *((volatile uint32_t *) 0x50500690) = 0x00000077; // TRAM ptr max
  *((volatile uint32_t *) 0x50500790) = 0x00022000; // Post processing register
  *((volatile uint32_t *) 0x50500810) = 0x00000001; // Stream processing start
  *((volatile uint32_t *) 0x50500910) = 0x00000002; // Rollover
  *((volatile uint32_t *) 0x50500990) = 0x00004b00; // Input frame size

  // Layer 0 group 2
  *((volatile uint32_t *) 0x50900010) = 0x000100a1; // Rows
  *((volatile uint32_t *) 0x50900090) = 0x00010079; // Columns
  *((volatile uint32_t *) 0x50900310) = 0x00002400; // SRAM write ptr
  *((volatile uint32_t *) 0x50900410) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50900490) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50900590) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50900a10) = 0x00007800; // Layer control 2
  *((volatile uint32_t *) 0x50900610) = 0x00000078; // Mask offset and count
  *((volatile uint32_t *) 0x50900690) = 0x00000077; // TRAM ptr max
  *((volatile uint32_t *) 0x50900790) = 0x00022000; // Post processing register
  *((volatile uint32_t *) 0x50900810) = 0x00000001; // Stream processing start
  *((volatile uint32_t *) 0x50900910) = 0x00000002; // Rollover
  *((volatile uint32_t *) 0x50900990) = 0x00004b00; // Input frame size

  // Layer 0 group 3
  *((volatile uint32_t *) 0x50d00010) = 0x000100a1; // Rows
  *((volatile uint32_t *) 0x50d00090) = 0x00010079; // Columns
  *((volatile uint32_t *) 0x50d00310) = 0x00002400; // SRAM write ptr
  *((volatile uint32_t *) 0x50d00410) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d00490) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d00590) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50d00a10) = 0x00007800; // Layer control 2
  *((volatile uint32_t *) 0x50d00610) = 0x00000078; // Mask offset and count
  *((volatile uint32_t *) 0x50d00690) = 0x00000077; // TRAM ptr max
  *((volatile uint32_t *) 0x50d00790) = 0x00022000; // Post processing register
  *((volatile uint32_t *) 0x50d00810) = 0x00000001; // Stream processing start
  *((volatile uint32_t *) 0x50d00910) = 0x00000002; // Rollover
  *((volatile uint32_t *) 0x50d00990) = 0x00004b00; // Input frame size

  // Layer 1 group 0
  *((volatile uint32_t *) 0x50100014) = 0x000100a1; // Rows
  *((volatile uint32_t *) 0x50100094) = 0x00010079; // Columns
  *((volatile uint32_t *) 0x50100194) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50100214) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x50100294) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50100314) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50100414) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50100494) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50100514) = 0x00000400; // SRAM read ptr
  *((volatile uint32_t *) 0x50100594) = 0x00002ba0; // Layer control
  *((volatile uint32_t *) 0x50100a14) = 0x0000f800; // Layer control 2
  *((volatile uint32_t *) 0x50100614) = 0x000000f8; // Mask offset and count
  *((volatile uint32_t *) 0x50100694) = 0x007800b3; // TRAM ptr max
  *((volatile uint32_t *) 0x50100714) = 0xfff0fff0; // Mask and processor enables
  *((volatile uint32_t *) 0x50100814) = 0x00000174; // Stream processing start
  *((volatile uint32_t *) 0x50100894) = 0x007c0021; // Stream processing delta
  *((volatile uint32_t *) 0x50100914) = 0x000001ee; // Rollover

  // Layer 1 group 1
  *((volatile uint32_t *) 0x50500014) = 0x000100a1; // Rows
  *((volatile uint32_t *) 0x50500094) = 0x00010079; // Columns
  *((volatile uint32_t *) 0x50500194) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50500214) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x50500294) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50500314) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50500414) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50500494) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50500514) = 0x00000400; // SRAM read ptr
  *((volatile uint32_t *) 0x50500594) = 0x00000ba0; // Layer control
  *((volatile uint32_t *) 0x50500a14) = 0x0000f800; // Layer control 2
  *((volatile uint32_t *) 0x50500614) = 0x000000f8; // Mask offset and count
  *((volatile uint32_t *) 0x50500694) = 0x007800b3; // TRAM ptr max
  *((volatile uint32_t *) 0x50500714) = 0x000f000f; // Mask and processor enables
  *((volatile uint32_t *) 0x50500814) = 0x00000174; // Stream processing start
  *((volatile uint32_t *) 0x50500894) = 0x007c0021; // Stream processing delta
  *((volatile uint32_t *) 0x50500914) = 0x000001ee; // Rollover

  // Layer 1 group 2
  *((volatile uint32_t *) 0x50900014) = 0x000100a1; // Rows
  *((volatile uint32_t *) 0x50900094) = 0x00010079; // Columns
  *((volatile uint32_t *) 0x50900194) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50900214) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x50900294) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50900314) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50900414) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50900494) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50900514) = 0x00000400; // SRAM read ptr
  *((volatile uint32_t *) 0x50900594) = 0x00000ba0; // Layer control
  *((volatile uint32_t *) 0x50900a14) = 0x0000f800; // Layer control 2
  *((volatile uint32_t *) 0x50900614) = 0x000000f8; // Mask offset and count
  *((volatile uint32_t *) 0x50900694) = 0x007800b3; // TRAM ptr max
  *((volatile uint32_t *) 0x50900814) = 0x00000174; // Stream processing start
  *((volatile uint32_t *) 0x50900894) = 0x007c0021; // Stream processing delta
  *((volatile uint32_t *) 0x50900914) = 0x000001ee; // Rollover

  // Layer 1 group 3
  *((volatile uint32_t *) 0x50d00014) = 0x000100a1; // Rows
  *((volatile uint32_t *) 0x50d00094) = 0x00010079; // Columns
  *((volatile uint32_t *) 0x50d00194) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50d00214) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x50d00294) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50d00314) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50d00414) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d00494) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d00514) = 0x00000400; // SRAM read ptr
  *((volatile uint32_t *) 0x50d00594) = 0x00000ba0; // Layer control
  *((volatile uint32_t *) 0x50d00a14) = 0x0000f800; // Layer control 2
  *((volatile uint32_t *) 0x50d00614) = 0x000000f8; // Mask offset and count
  *((volatile uint32_t *) 0x50d00694) = 0x007800b3; // TRAM ptr max
  *((volatile uint32_t *) 0x50d00814) = 0x00000174; // Stream processing start
  *((volatile uint32_t *) 0x50d00894) = 0x007c0021; // Stream processing delta
  *((volatile uint32_t *) 0x50d00914) = 0x000001ee; // Rollover

  // Layer 2 group 0
  *((volatile uint32_t *) 0x50100018) = 0x00010051; // Rows
  *((volatile uint32_t *) 0x50100098) = 0x0001003d; // Columns
  *((volatile uint32_t *) 0x50100198) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50100218) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x50100298) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50100318) = 0x00010000; // SRAM write ptr
  *((volatile uint32_t *) 0x50100418) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50100498) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50100518) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x50100598) = 0x00002ba0; // Layer control
  *((volatile uint32_t *) 0x50100a18) = 0x0000f800; // Layer control 2
  *((volatile uint32_t *) 0x50100618) = 0x010001f8; // Mask offset and count
  *((volatile uint32_t *) 0x50100698) = 0x0000001d; // TRAM ptr max
  *((volatile uint32_t *) 0x50100798) = 0x00022000; // Post processing register
  *((volatile uint32_t *) 0x50100718) = 0xffffffff; // Mask and processor enables

  // Layer 2 group 1
  *((volatile uint32_t *) 0x50500018) = 0x00010051; // Rows
  *((volatile uint32_t *) 0x50500098) = 0x0001003d; // Columns
  *((volatile uint32_t *) 0x50500198) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50500218) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x50500298) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50500318) = 0x00010000; // SRAM write ptr
  *((volatile uint32_t *) 0x50500418) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50500498) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50500518) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x50500598) = 0x00000ba0; // Layer control
  *((volatile uint32_t *) 0x50500a18) = 0x0000f800; // Layer control 2
  *((volatile uint32_t *) 0x50500618) = 0x010001f8; // Mask offset and count
  *((volatile uint32_t *) 0x50500698) = 0x0000001d; // TRAM ptr max
  *((volatile uint32_t *) 0x50500798) = 0x00022000; // Post processing register
  *((volatile uint32_t *) 0x50500718) = 0xffffffff; // Mask and processor enables

  // Layer 2 group 2
  *((volatile uint32_t *) 0x50900018) = 0x00010051; // Rows
  *((volatile uint32_t *) 0x50900098) = 0x0001003d; // Columns
  *((volatile uint32_t *) 0x50900198) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50900218) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x50900298) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50900318) = 0x00010000; // SRAM write ptr
  *((volatile uint32_t *) 0x50900418) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50900498) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50900518) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x50900598) = 0x00000ba0; // Layer control
  *((volatile uint32_t *) 0x50900a18) = 0x0000f800; // Layer control 2
  *((volatile uint32_t *) 0x50900618) = 0x010001f8; // Mask offset and count
  *((volatile uint32_t *) 0x50900698) = 0x0000001d; // TRAM ptr max
  *((volatile uint32_t *) 0x50900798) = 0x00022000; // Post processing register

  // Layer 2 group 3
  *((volatile uint32_t *) 0x50d00018) = 0x00010051; // Rows
  *((volatile uint32_t *) 0x50d00098) = 0x0001003d; // Columns
  *((volatile uint32_t *) 0x50d00198) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50d00218) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x50d00298) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50d00318) = 0x00010000; // SRAM write ptr
  *((volatile uint32_t *) 0x50d00418) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d00498) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d00518) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x50d00598) = 0x00000ba0; // Layer control
  *((volatile uint32_t *) 0x50d00a18) = 0x0000f800; // Layer control 2
  *((volatile uint32_t *) 0x50d00618) = 0x010001f8; // Mask offset and count
  *((volatile uint32_t *) 0x50d00698) = 0x0000001d; // TRAM ptr max
  *((volatile uint32_t *) 0x50d00798) = 0x00022000; // Post processing register

  // Layer 3 group 0
  *((volatile uint32_t *) 0x5010001c) = 0x00010029; // Rows
  *((volatile uint32_t *) 0x5010009c) = 0x0001001f; // Columns
  *((volatile uint32_t *) 0x5010019c) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x5010021c) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x5010029c) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x5010031c) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x5010041c) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x5010049c) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x5010059c) = 0x0000cba0; // Layer control
  *((volatile uint32_t *) 0x50100a1c) = 0x0001f800; // Layer control 2
  *((volatile uint32_t *) 0x5010061c) = 0x000001f8; // Mask offset and count
  *((volatile uint32_t *) 0x5010069c) = 0x0000000e; // TRAM ptr max

  // Layer 3 group 1
  *((volatile uint32_t *) 0x5050001c) = 0x00010029; // Rows
  *((volatile uint32_t *) 0x5050009c) = 0x0001001f; // Columns
  *((volatile uint32_t *) 0x5050019c) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x5050021c) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x5050029c) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x5050031c) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x5050041c) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x5050049c) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x5050059c) = 0x00000ba0; // Layer control
  *((volatile uint32_t *) 0x50500a1c) = 0x0001f800; // Layer control 2
  *((volatile uint32_t *) 0x5050061c) = 0x000001f8; // Mask offset and count
  *((volatile uint32_t *) 0x5050069c) = 0x0000000e; // TRAM ptr max

  // Layer 3 group 2
  *((volatile uint32_t *) 0x5090001c) = 0x00010029; // Rows
  *((volatile uint32_t *) 0x5090009c) = 0x0001001f; // Columns
  *((volatile uint32_t *) 0x5090019c) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x5090021c) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x5090029c) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x5090031c) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x5090041c) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x5090049c) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x5090059c) = 0x00000ba0; // Layer control
  *((volatile uint32_t *) 0x50900a1c) = 0x0001f800; // Layer control 2
  *((volatile uint32_t *) 0x5090061c) = 0x000001f8; // Mask offset and count
  *((volatile uint32_t *) 0x5090069c) = 0x0000000e; // TRAM ptr max
  *((volatile uint32_t *) 0x5090071c) = 0xffffffff; // Mask and processor enables

  // Layer 3 group 3
  *((volatile uint32_t *) 0x50d0001c) = 0x00010029; // Rows
  *((volatile uint32_t *) 0x50d0009c) = 0x0001001f; // Columns
  *((volatile uint32_t *) 0x50d0019c) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50d0021c) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x50d0029c) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50d0031c) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50d0041c) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d0049c) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d0059c) = 0x00000ba0; // Layer control
  *((volatile uint32_t *) 0x50d00a1c) = 0x0001f800; // Layer control 2
  *((volatile uint32_t *) 0x50d0061c) = 0x000001f8; // Mask offset and count
  *((volatile uint32_t *) 0x50d0069c) = 0x0000000e; // TRAM ptr max
  *((volatile uint32_t *) 0x50d0071c) = 0xffffffff; // Mask and processor enables

  // Layer 4 group 0
  *((volatile uint32_t *) 0x50100020) = 0x00010015; // Rows
  *((volatile uint32_t *) 0x501000a0) = 0x00010010; // Columns
  *((volatile uint32_t *) 0x501001a0) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50100220) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x501002a0) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50100420) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x501004a0) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50100520) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x501005a0) = 0x0000eba0; // Layer control
  *((volatile uint32_t *) 0x50100a20) = 0x0001f800; // Layer control 2
  *((volatile uint32_t *) 0x50100620) = 0x020003f8; // Mask offset and count
  *((volatile uint32_t *) 0x501006a0) = 0x00000006; // TRAM ptr max
  *((volatile uint32_t *) 0x50100720) = 0xffffffff; // Mask and processor enables

  // Layer 4 group 1
  *((volatile uint32_t *) 0x50500020) = 0x00010015; // Rows
  *((volatile uint32_t *) 0x505000a0) = 0x00010010; // Columns
  *((volatile uint32_t *) 0x505001a0) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50500220) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x505002a0) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50500420) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x505004a0) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50500520) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x505005a0) = 0x00000ba0; // Layer control
  *((volatile uint32_t *) 0x50500a20) = 0x0001f800; // Layer control 2
  *((volatile uint32_t *) 0x50500620) = 0x020003f8; // Mask offset and count
  *((volatile uint32_t *) 0x505006a0) = 0x00000006; // TRAM ptr max
  *((volatile uint32_t *) 0x50500720) = 0xffffffff; // Mask and processor enables

  // Layer 4 group 2
  *((volatile uint32_t *) 0x50900020) = 0x00010015; // Rows
  *((volatile uint32_t *) 0x509000a0) = 0x00010010; // Columns
  *((volatile uint32_t *) 0x509001a0) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50900220) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x509002a0) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50900420) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x509004a0) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50900520) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x509005a0) = 0x00000ba0; // Layer control
  *((volatile uint32_t *) 0x50900a20) = 0x0001f800; // Layer control 2
  *((volatile uint32_t *) 0x50900620) = 0x020003f8; // Mask offset and count
  *((volatile uint32_t *) 0x509006a0) = 0x00000006; // TRAM ptr max
  *((volatile uint32_t *) 0x50900720) = 0xffffffff; // Mask and processor enables

  // Layer 4 group 3
  *((volatile uint32_t *) 0x50d00020) = 0x00010015; // Rows
  *((volatile uint32_t *) 0x50d000a0) = 0x00010010; // Columns
  *((volatile uint32_t *) 0x50d001a0) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50d00220) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x50d002a0) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50d00420) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d004a0) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d00520) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x50d005a0) = 0x00000ba0; // Layer control
  *((volatile uint32_t *) 0x50d00a20) = 0x0001f800; // Layer control 2
  *((volatile uint32_t *) 0x50d00620) = 0x020003f8; // Mask offset and count
  *((volatile uint32_t *) 0x50d006a0) = 0x00000006; // TRAM ptr max
  *((volatile uint32_t *) 0x50d00720) = 0xffffffff; // Mask and processor enables

  // Layer 5 group 0
  *((volatile uint32_t *) 0x50100024) = 0x0001000b; // Rows
  *((volatile uint32_t *) 0x501000a4) = 0x00010008; // Columns
  *((volatile uint32_t *) 0x50100324) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50100424) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x501004a4) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x501005a4) = 0x0000eb20; // Layer control
  *((volatile uint32_t *) 0x50100a24) = 0x0001f800; // Layer control 2
  *((volatile uint32_t *) 0x50100624) = 0x040005f8; // Mask offset and count
  *((volatile uint32_t *) 0x501006a4) = 0x00000006; // TRAM ptr max
  *((volatile uint32_t *) 0x50100724) = 0xffffffff; // Mask and processor enables

  // Layer 5 group 1
  *((volatile uint32_t *) 0x50500024) = 0x0001000b; // Rows
  *((volatile uint32_t *) 0x505000a4) = 0x00010008; // Columns
  *((volatile uint32_t *) 0x50500324) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50500424) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x505004a4) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x505005a4) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50500a24) = 0x0001f800; // Layer control 2
  *((volatile uint32_t *) 0x50500624) = 0x040005f8; // Mask offset and count
  *((volatile uint32_t *) 0x505006a4) = 0x00000006; // TRAM ptr max
  *((volatile uint32_t *) 0x50500724) = 0xffffffff; // Mask and processor enables

  // Layer 5 group 2
  *((volatile uint32_t *) 0x50900024) = 0x0001000b; // Rows
  *((volatile uint32_t *) 0x509000a4) = 0x00010008; // Columns
  *((volatile uint32_t *) 0x50900324) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50900424) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x509004a4) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x509005a4) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50900a24) = 0x0001f800; // Layer control 2
  *((volatile uint32_t *) 0x50900624) = 0x040005f8; // Mask offset and count
  *((volatile uint32_t *) 0x509006a4) = 0x00000006; // TRAM ptr max
  *((volatile uint32_t *) 0x50900724) = 0xffffffff; // Mask and processor enables

  // Layer 5 group 3
  *((volatile uint32_t *) 0x50d00024) = 0x0001000b; // Rows
  *((volatile uint32_t *) 0x50d000a4) = 0x00010008; // Columns
  *((volatile uint32_t *) 0x50d00324) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50d00424) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d004a4) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d005a4) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50d00a24) = 0x0001f800; // Layer control 2
  *((volatile uint32_t *) 0x50d00624) = 0x040005f8; // Mask offset and count
  *((volatile uint32_t *) 0x50d006a4) = 0x00000006; // TRAM ptr max
  *((volatile uint32_t *) 0x50d00724) = 0xffffffff; // Mask and processor enables

  // Layer 6 group 0
  *((volatile uint32_t *) 0x50100028) = 0x0001000b; // Rows
  *((volatile uint32_t *) 0x501000a8) = 0x00010008; // Columns
  *((volatile uint32_t *) 0x50100428) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x501004a8) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50100528) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x501005a8) = 0x0000eb20; // Layer control
  *((volatile uint32_t *) 0x50100a28) = 0x0001f800; // Layer control 2
  *((volatile uint32_t *) 0x50100628) = 0x060007f8; // Mask offset and count
  *((volatile uint32_t *) 0x501006a8) = 0x00000006; // TRAM ptr max
  *((volatile uint32_t *) 0x50100728) = 0xffffffff; // Mask and processor enables

  // Layer 6 group 1
  *((volatile uint32_t *) 0x50500028) = 0x0001000b; // Rows
  *((volatile uint32_t *) 0x505000a8) = 0x00010008; // Columns
  *((volatile uint32_t *) 0x50500428) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x505004a8) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50500528) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x505005a8) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50500a28) = 0x0001f800; // Layer control 2
  *((volatile uint32_t *) 0x50500628) = 0x060007f8; // Mask offset and count
  *((volatile uint32_t *) 0x505006a8) = 0x00000006; // TRAM ptr max
  *((volatile uint32_t *) 0x50500728) = 0xffffffff; // Mask and processor enables

  // Layer 6 group 2
  *((volatile uint32_t *) 0x50900028) = 0x0001000b; // Rows
  *((volatile uint32_t *) 0x509000a8) = 0x00010008; // Columns
  *((volatile uint32_t *) 0x50900428) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x509004a8) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50900528) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x509005a8) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50900a28) = 0x0001f800; // Layer control 2
  *((volatile uint32_t *) 0x50900628) = 0x060007f8; // Mask offset and count
  *((volatile uint32_t *) 0x509006a8) = 0x00000006; // TRAM ptr max
  *((volatile uint32_t *) 0x50900728) = 0xffffffff; // Mask and processor enables

  // Layer 6 group 3
  *((volatile uint32_t *) 0x50d00028) = 0x0001000b; // Rows
  *((volatile uint32_t *) 0x50d000a8) = 0x00010008; // Columns
  *((volatile uint32_t *) 0x50d00428) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d004a8) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d00528) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x50d005a8) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50d00a28) = 0x0001f800; // Layer control 2
  *((volatile uint32_t *) 0x50d00628) = 0x060007f8; // Mask offset and count
  *((volatile uint32_t *) 0x50d006a8) = 0x00000006; // TRAM ptr max
  *((volatile uint32_t *) 0x50d00728) = 0xffffffff; // Mask and processor enables

  // Layer 7 group 0
  *((volatile uint32_t *) 0x5010002c) = 0x00000009; // Rows
  *((volatile uint32_t *) 0x501000ac) = 0x00000006; // Columns
  *((volatile uint32_t *) 0x501001ac) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x5010022c) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x501002ac) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x5010032c) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x501003ac) = 0x00000001; // Write ptr time slot offs
  *((volatile uint32_t *) 0x5010042c) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x501004ac) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x501005ac) = 0x0000e9a0; // Layer control
  *((volatile uint32_t *) 0x50100a2c) = 0x0001f870; // Layer control 2
  *((volatile uint32_t *) 0x5010062c) = 0x480057f8; // Mask offset and count
  *((volatile uint32_t *) 0x5010012c) = 0x00000100; // 1D
  *((volatile uint32_t *) 0x5010072c) = 0xffffffff; // Mask and processor enables

  // Layer 7 group 1
  *((volatile uint32_t *) 0x5050002c) = 0x00000009; // Rows
  *((volatile uint32_t *) 0x505000ac) = 0x00000006; // Columns
  *((volatile uint32_t *) 0x505001ac) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x5050022c) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x505002ac) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x5050032c) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x505003ac) = 0x00000001; // Write ptr time slot offs
  *((volatile uint32_t *) 0x5050042c) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x505004ac) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x505005ac) = 0x000009a0; // Layer control
  *((volatile uint32_t *) 0x50500a2c) = 0x0001f870; // Layer control 2
  *((volatile uint32_t *) 0x5050062c) = 0x480057f8; // Mask offset and count
  *((volatile uint32_t *) 0x5050012c) = 0x00000100; // 1D
  *((volatile uint32_t *) 0x5050072c) = 0xffffffff; // Mask and processor enables

  // Layer 7 group 2
  *((volatile uint32_t *) 0x5090002c) = 0x00000009; // Rows
  *((volatile uint32_t *) 0x509000ac) = 0x00000006; // Columns
  *((volatile uint32_t *) 0x509001ac) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x5090022c) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x509002ac) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x5090032c) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x509003ac) = 0x00000001; // Write ptr time slot offs
  *((volatile uint32_t *) 0x5090042c) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x509004ac) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x509005ac) = 0x000009a0; // Layer control
  *((volatile uint32_t *) 0x50900a2c) = 0x0001f870; // Layer control 2
  *((volatile uint32_t *) 0x5090062c) = 0x480057f8; // Mask offset and count
  *((volatile uint32_t *) 0x5090012c) = 0x00000100; // 1D
  *((volatile uint32_t *) 0x5090072c) = 0xffffffff; // Mask and processor enables

  // Layer 7 group 3
  *((volatile uint32_t *) 0x50d0002c) = 0x00000009; // Rows
  *((volatile uint32_t *) 0x50d000ac) = 0x00000006; // Columns
  *((volatile uint32_t *) 0x50d001ac) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50d0022c) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x50d002ac) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50d0032c) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50d003ac) = 0x00000001; // Write ptr time slot offs
  *((volatile uint32_t *) 0x50d0042c) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d004ac) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d005ac) = 0x000009a0; // Layer control
  *((volatile uint32_t *) 0x50d00a2c) = 0x0001f870; // Layer control 2
  *((volatile uint32_t *) 0x50d0062c) = 0x480057f8; // Mask offset and count
  *((volatile uint32_t *) 0x50d0012c) = 0x00000100; // 1D
  *((volatile uint32_t *) 0x50d0072c) = 0xffffffff; // Mask and processor enables

  // Layer 8 group 0
  *((volatile uint32_t *) 0x50100030) = 0x00000004; // Rows
  *((volatile uint32_t *) 0x501000b0) = 0x00000002; // Columns
  *((volatile uint32_t *) 0x501001b0) = 0x00000004; // Pooling rows
  *((volatile uint32_t *) 0x50100230) = 0x00000002; // Pooling columns
  *((volatile uint32_t *) 0x501003b0) = 0x00002000; // Write ptr time slot offs
  *((volatile uint32_t *) 0x50100430) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x501004b0) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50100530) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x501005b0) = 0x000008a0; // Layer control
  *((volatile uint32_t *) 0x50100a30) = 0x0001f807; // Layer control 2
  *((volatile uint32_t *) 0x50100630) = 0x00000038; // Mask offset and count
  *((volatile uint32_t *) 0x50100130) = 0x00000103; // 1D
  *((volatile uint32_t *) 0x501007b0) = 0x03000000; // Post processing register
  *((volatile uint32_t *) 0x50100730) = 0x0000ffff; // Mask and processor enables

  // Layer 8 group 1
  *((volatile uint32_t *) 0x50500030) = 0x00000004; // Rows
  *((volatile uint32_t *) 0x505000b0) = 0x00000002; // Columns
  *((volatile uint32_t *) 0x505001b0) = 0x00000004; // Pooling rows
  *((volatile uint32_t *) 0x50500230) = 0x00000002; // Pooling columns
  *((volatile uint32_t *) 0x50500330) = 0x00008000; // SRAM write ptr
  *((volatile uint32_t *) 0x505003b0) = 0x00002000; // Write ptr time slot offs
  *((volatile uint32_t *) 0x50500430) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x505004b0) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50500530) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x505005b0) = 0x000008a0; // Layer control
  *((volatile uint32_t *) 0x50500a30) = 0x0001f807; // Layer control 2
  *((volatile uint32_t *) 0x50500630) = 0x00000038; // Mask offset and count
  *((volatile uint32_t *) 0x50500130) = 0x00000103; // 1D
  *((volatile uint32_t *) 0x505007b0) = 0x03000000; // Post processing register
  *((volatile uint32_t *) 0x50500730) = 0x0000ffff; // Mask and processor enables

  // Layer 8 group 2
  *((volatile uint32_t *) 0x50900030) = 0x00000004; // Rows
  *((volatile uint32_t *) 0x509000b0) = 0x00000002; // Columns
  *((volatile uint32_t *) 0x509001b0) = 0x00000004; // Pooling rows
  *((volatile uint32_t *) 0x50900230) = 0x00000002; // Pooling columns
  *((volatile uint32_t *) 0x50900330) = 0x00010000; // SRAM write ptr
  *((volatile uint32_t *) 0x509003b0) = 0x00002000; // Write ptr time slot offs
  *((volatile uint32_t *) 0x50900430) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x509004b0) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50900530) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x509005b0) = 0x000008a0; // Layer control
  *((volatile uint32_t *) 0x50900a30) = 0x0001f807; // Layer control 2
  *((volatile uint32_t *) 0x50900630) = 0x00000038; // Mask offset and count
  *((volatile uint32_t *) 0x50900130) = 0x00000103; // 1D
  *((volatile uint32_t *) 0x509007b0) = 0x03000000; // Post processing register
  *((volatile uint32_t *) 0x50900730) = 0x0000ffff; // Mask and processor enables

  // Layer 8 group 3
  *((volatile uint32_t *) 0x50d00030) = 0x00000004; // Rows
  *((volatile uint32_t *) 0x50d000b0) = 0x00000002; // Columns
  *((volatile uint32_t *) 0x50d001b0) = 0x00000004; // Pooling rows
  *((volatile uint32_t *) 0x50d00230) = 0x00000002; // Pooling columns
  *((volatile uint32_t *) 0x50d00330) = 0x00018000; // SRAM write ptr
  *((volatile uint32_t *) 0x50d003b0) = 0x00002000; // Write ptr time slot offs
  *((volatile uint32_t *) 0x50d00430) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d004b0) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d00530) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x50d005b0) = 0x000008a0; // Layer control
  *((volatile uint32_t *) 0x50d00a30) = 0x0001f807; // Layer control 2
  *((volatile uint32_t *) 0x50d00630) = 0x00000038; // Mask offset and count
  *((volatile uint32_t *) 0x50d00130) = 0x00000103; // 1D
  *((volatile uint32_t *) 0x50d007b0) = 0x03000000; // Post processing register
  *((volatile uint32_t *) 0x50d00730) = 0x0000ffff; // Mask and processor enables


  *((volatile uint32_t *) 0x50000000) = 0x00001908; // FIFO control

  return CNN_OK;
}

int cnn_start(void)
{
  cnn_time = 0;

  *((volatile uint32_t *) 0x50100000) = 0x0018c808; // Enable group 0
  *((volatile uint32_t *) 0x50500000) = 0x0018c809; // Enable group 1
  *((volatile uint32_t *) 0x50900000) = 0x0018c809; // Enable group 2
  *((volatile uint32_t *) 0x50d00000) = 0x0018c809; // Enable group 3

#ifdef CNN_INFERENCE_TIMER
  MXC_TMR_SW_Start(CNN_INFERENCE_TIMER);
#endif

  CNN_START; // Allow capture of processing time
  *((volatile uint32_t *) 0x50100000) = 0x0018c809; // Master enable group 0

  return CNN_OK;
}

// Custom unload for this network: 8-bit data, shape: (512, 1, 1)
int cnn_unload(uint32_t *out_buf32)
{
  volatile uint32_t *addr;
  uint8_t *out_buf = (uint8_t *) out_buf32;
  uint32_t val;

  addr = (volatile uint32_t *) 0x50400000;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50408000;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50410000;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50418000;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50800000;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50808000;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50810000;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50818000;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c00000;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c08000;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c10000;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c18000;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x51000000;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x51008000;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x51010000;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x51018000;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50400004;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50408004;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50410004;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50418004;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50800004;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50808004;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50810004;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50818004;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c00004;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c08004;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c10004;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c18004;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x51000004;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x51008004;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x51010004;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x51018004;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50400008;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50408008;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50410008;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50418008;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50800008;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50808008;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50810008;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50818008;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c00008;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c08008;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c10008;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c18008;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x51000008;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x51008008;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x51010008;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x51018008;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x5040000c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x5040800c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x5041000c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x5041800c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x5080000c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x5080800c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x5081000c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x5081800c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c0000c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c0800c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c1000c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c1800c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x5100000c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x5100800c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x5101000c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x5101800c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50400010;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50408010;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50410010;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50418010;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50800010;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50808010;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50810010;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50818010;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c00010;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c08010;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c10010;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c18010;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x51000010;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x51008010;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x51010010;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x51018010;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50400014;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50408014;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50410014;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50418014;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50800014;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50808014;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50810014;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50818014;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c00014;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c08014;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c10014;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c18014;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x51000014;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x51008014;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x51010014;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x51018014;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50400018;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50408018;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50410018;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50418018;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50800018;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50808018;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50810018;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50818018;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c00018;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c08018;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c10018;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c18018;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x51000018;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x51008018;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x51010018;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x51018018;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x5040001c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x5040801c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x5041001c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x5041801c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x5080001c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x5080801c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x5081001c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x5081801c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c0001c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c0801c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c1001c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x50c1801c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x5100001c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x5100801c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x5101001c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;
  addr = (volatile uint32_t *) 0x5101801c;
  val = *addr++;
  *out_buf++ = val & 0xff;
  *out_buf++ = (val >> 8) & 0xff;
  *out_buf++ = (val >> 16) & 0xff;
  *out_buf++ = (val >> 24) & 0xff;

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

  MXC_GCR->pclkdiv = (MXC_GCR->pclkdiv & ~(MXC_F_GCR_PCLKDIV_CNNCLKDIV | MXC_F_GCR_PCLKDIV_CNNCLKSEL))
                     | clock_divider | clock_source;
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
  MXC_GPIO_OutSet(gpio_out.port, gpio_out.mask);

  return CNN_OK;
}

int cnn_disable(void)
{
  // Disable CNN clock
  MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_CNN);

  // Disable power to CNN
  MXC_GCFR->reg3 = 0xf; // Reset
  MXC_GCFR->reg1 = 0x0; // Mask memory
  MXC_GCFR->reg0 = 0x0; // Power
  MXC_GCFR->reg2 = 0xf; // Iso
  MXC_GCFR->reg3 = 0x0; // Reset

  return CNN_OK;
}
