/******************************************************************************
 *
 * Copyright (C) 2019-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

// Kinetics Action Recognition Known-Answer-Test for MAX78002
// Generated using ./ai8xize.py --test-dir sdk/Examples/MAX78002/CNN --prefix kinetics --checkpoint-file trained/ai85-kinetics-qat8-q.pth.tar --config-file networks/ai85-kinetics-actiontcn.yaml --overlap-data --softmax --zero-sram --device MAX78002 --timer 0 --display-checkpoint --verbose

#include <assert.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "mxc.h"
#include "cnn.h"
#include "sampledata.h"
#include "sampleoutput.h"

volatile uint32_t cnn_time; // Stopwatch

void fail(void)
{
    printf("\n*** FAIL ***\n\n");
    while (1) {}
}

// 96-channel 60x60 data input (345600 bytes total / 3600 bytes per channel):
// HWC 60x60, channels 0 to 3
// HWC 60x60, channels 48 to 51
static const uint32_t input_0[] = SAMPLE_INPUT_0;

// HWC 60x60, channels 4 to 7
// HWC 60x60, channels 52 to 55
static const uint32_t input_4[] = SAMPLE_INPUT_4;

// HWC 60x60, channels 8 to 11
// HWC 60x60, channels 56 to 59
static const uint32_t input_8[] = SAMPLE_INPUT_8;

// HWC 60x60, channels 12 to 15
// HWC 60x60, channels 60 to 63
static const uint32_t input_12[] = SAMPLE_INPUT_12;

// HWC 60x60, channels 16 to 19
// HWC 60x60, channels 64 to 67
static const uint32_t input_16[] = SAMPLE_INPUT_16;

// HWC 60x60, channels 20 to 23
// HWC 60x60, channels 68 to 71
static const uint32_t input_20[] = SAMPLE_INPUT_20;

// HWC 60x60, channels 24 to 27
// HWC 60x60, channels 72 to 75
static const uint32_t input_24[] = SAMPLE_INPUT_24;

// HWC 60x60, channels 28 to 31
// HWC 60x60, channels 76 to 79
static const uint32_t input_28[] = SAMPLE_INPUT_28;

// HWC 60x60, channels 32 to 35
// HWC 60x60, channels 80 to 83
static const uint32_t input_32[] = SAMPLE_INPUT_32;

// HWC 60x60, channels 36 to 39
// HWC 60x60, channels 84 to 87
static const uint32_t input_36[] = SAMPLE_INPUT_36;

// HWC 60x60, channels 40 to 43
// HWC 60x60, channels 88 to 91
static const uint32_t input_40[] = SAMPLE_INPUT_40;

// HWC 60x60, channels 44 to 47
// HWC 60x60, channels 92 to 95
static const uint32_t input_44[] = SAMPLE_INPUT_44;

void load_input(void)
{
    // This function loads the sample data input -- replace with actual data

    memcpy32((uint32_t *)0x51800f00, input_0, 7200);
    memcpy32((uint32_t *)0x51820f00, input_4, 7200);
    memcpy32((uint32_t *)0x51840f00, input_8, 7200);
    memcpy32((uint32_t *)0x51860f00, input_12, 7200);
    memcpy32((uint32_t *)0x52800f00, input_16, 7200);
    memcpy32((uint32_t *)0x52820f00, input_20, 7200);
    memcpy32((uint32_t *)0x52840f00, input_24, 7200);
    memcpy32((uint32_t *)0x52860f00, input_28, 7200);
    memcpy32((uint32_t *)0x53800f00, input_32, 7200);
    memcpy32((uint32_t *)0x53820f00, input_36, 7200);
    memcpy32((uint32_t *)0x53840f00, input_40, 7200);
    memcpy32((uint32_t *)0x53860f00, input_44, 7200);
}

// Expected output of layer 20 (tcn2) for kinetics given the sample input (known-answer test)
// Delete this function for production code
static const uint32_t sample_output[] = SAMPLE_OUTPUT;
int check_output(void)
{
    int i;
    uint32_t mask, len;
    volatile uint32_t *addr;
    const uint32_t *ptr = sample_output;

    while ((addr = (volatile uint32_t *)*ptr++) != 0) {
        mask = *ptr++;
        len = *ptr++;
        for (i = 0; i < len; i++)
            if ((*addr++ & mask) != *ptr++) {
                printf("Data mismatch (%d/%d) at address 0x%08x: Expected 0x%08x, read 0x%08x.\n",
                       i + 1, len, addr - 1, *(ptr - 1), *(addr - 1) & mask);
                return CNN_FAIL;
            }
    }

    return CNN_OK;
}

// Classification layer:
static int32_t ml_data[CNN_NUM_OUTPUTS];
static q15_t ml_softmax[CNN_NUM_OUTPUTS];

void softmax_layer(void)
{
    cnn_unload((uint32_t *)ml_data);
    softmax_q17p14_q15((const q31_t *)ml_data, CNN_NUM_OUTPUTS, ml_softmax);
}

int main(void)
{
    int i;
    int digs, tens;

    MXC_ICC_Enable(MXC_ICC0); // Enable cache

    // Switch to 120 MHz clock
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    MXC_GCR->ipll_ctrl |= MXC_F_GCR_IPLL_CTRL_EN; // Enable IPLL
    SystemCoreClockUpdate();

    printf("Waiting...\n");

    // DO NOT DELETE THIS LINE:
    MXC_Delay(SEC(2)); // Let debugger interrupt if needed

    // Enable peripheral, enable CNN interrupt, turn on CNN clock
    // CNN clock: PLL (200 MHz) div 4
    cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_IPLL, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV4);

    printf("\n*** CNN Inference Test kinetics ***\n");

    if (cnn_init() != CNN_OK)
        fail();
    cnn_load_weights(); // Load kernels
    cnn_load_bias();
    cnn_configure(); // Configure state machine
    load_input(); // Load data input
    // CNN clock: PLL (200 MHz) div 1
    MXC_GCR->pclkdiv =
        (MXC_GCR->pclkdiv & ~(MXC_F_GCR_PCLKDIV_CNNCLKDIV | MXC_F_GCR_PCLKDIV_CNNCLKSEL)) |
        MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1 | MXC_S_GCR_PCLKDIV_CNNCLKSEL_IPLL;
    cnn_start(); // Start CNN processing

    while (cnn_time == 0) MXC_LP_EnterSleepMode(); // Wait for CNN

    // Switch CNN clock to PLL (200 MHz) div 4

    MXC_GCR->pclkdiv =
        (MXC_GCR->pclkdiv & ~(MXC_F_GCR_PCLKDIV_CNNCLKDIV | MXC_F_GCR_PCLKDIV_CNNCLKSEL)) |
        MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV4 | MXC_S_GCR_PCLKDIV_CNNCLKSEL_IPLL;
    if (check_output() != CNN_OK)
        fail();
    softmax_layer();

    printf("\n*** PASS ***\n\n");

#ifdef CNN_INFERENCE_TIMER
    printf("Approximate inference time: %u us\n\n", cnn_time);
#endif

    cnn_disable(); // Shut down CNN clock, disable peripheral

    MXC_GCR->ipll_ctrl &= ~MXC_F_GCR_IPLL_CTRL_EN; // Disable IPLL

    printf("Classification results:\n");
    for (i = 0; i < CNN_NUM_OUTPUTS; i++) {
        digs = (1000 * ml_softmax[i] + 0x4000) >> 15;
        tens = digs % 10;
        digs = digs / 10;
        printf("[%7d] -> Class %d: %d.%d%%\n", ml_data[i], i, digs, tens);
    }

    return 0;
}

/*
  SUMMARY OF OPS
  Hardware: 344,024,768 ops (342,827,488 macc; 1,179,168 comp; 18,112 add; 0 mul; 0 bitwise)
    Layer 0: 22,348,800 ops (22,118,400 macc; 230,400 comp; 0 add; 0 mul; 0 bitwise)
    Layer 1: 132,940,800 ops (132,710,400 macc; 230,400 comp; 0 add; 0 mul; 0 bitwise)
    Layer 2: 132,940,800 ops (132,710,400 macc; 230,400 comp; 0 add; 0 mul; 0 bitwise)
    Layer 3 (res1_out): 33,465,600 ops (33,177,600 macc; 288,000 comp; 0 add; 0 mul; 0 bitwise)
    Layer 4 (conv2): 8,366,400 ops (8,294,400 macc; 72,000 comp; 0 add; 0 mul; 0 bitwise)
    Layer 5 (conv2_1): 936,000 ops (921,600 macc; 14,400 comp; 0 add; 0 mul; 0 bitwise)
    Layer 6 (conv2_p): 8,366,400 ops (8,294,400 macc; 72,000 comp; 0 add; 0 mul; 0 bitwise)
    Layer 7 (res2_out): 14,400 ops (0 macc; 0 comp; 14,400 add; 0 mul; 0 bitwise)
    Layer 8 (conv3): 1,822,016 ops (1,806,336 macc; 15,680 comp; 0 add; 0 mul; 0 bitwise)
    Layer 9 (conv3_1): 203,840 ops (200,704 macc; 3,136 comp; 0 add; 0 mul; 0 bitwise)
    Layer 10 (conv3_p): 1,822,016 ops (1,806,336 macc; 15,680 comp; 0 add; 0 mul; 0 bitwise)
    Layer 11 (res3_out): 3,136 ops (0 macc; 0 comp; 3,136 add; 0 mul; 0 bitwise)
    Layer 12 (conv4): 334,656 ops (331,776 macc; 2,880 comp; 0 add; 0 mul; 0 bitwise)
    Layer 13 (conv4_1): 37,440 ops (36,864 macc; 576 comp; 0 add; 0 mul; 0 bitwise)
    Layer 14 (conv4_p): 334,656 ops (331,776 macc; 2,880 comp; 0 add; 0 mul; 0 bitwise)
    Layer 15 (res4_out): 576 ops (0 macc; 0 comp; 576 add; 0 mul; 0 bitwise)
    Layer 16 (buffer_shift): 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 17 (conv5): 18,464 ops (18,432 macc; 32 comp; 0 add; 0 mul; 0 bitwise)
    Layer 18 (tcn0): 40,352 ops (39,936 macc; 416 comp; 0 add; 0 mul; 0 bitwise)
    Layer 19 (tcn1): 27,936 ops (27,648 macc; 288 comp; 0 add; 0 mul; 0 bitwise)
    Layer 20 (tcn2): 480 ops (480 macc; 0 comp; 0 add; 0 mul; 0 bitwise)

  RESOURCE USAGE
  Weight memory: 378,272 bytes out of 2,396,160 bytes total (15.8%)
  Bias memory:   933 bytes out of 8,192 bytes total (11.4%)
*/
