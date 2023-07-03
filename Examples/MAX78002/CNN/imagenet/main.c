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

// 3-channel 112x112 data input (37632 bytes total / 12544 bytes per channel):
// HWC 112x112, channels 0 to 2
static const uint32_t input_60[] = SAMPLE_INPUT_60;

void load_input(void)
{
    // This function loads the sample data input -- replace with actual data

    memcpy32((uint32_t *)0x54860000, input_60, 12544);
}

// Expected output of layer 33 (l33) for imagenet given the sample input (known-answer test)
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

static int32_t ml_data[CNN_NUM_OUTPUTS];

int main(void)
{
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

    printf("\n*** CNN Inference Test ***\n");

    cnn_init(); // Bring state machine into consistent state
    cnn_load_weights(); // Load kernels
    cnn_load_bias();
    cnn_configure(); // Configure state machine
    load_input(); // Load data input
    // CNN clock: PLL (200 MHz) div 1
    MXC_GCR->pclkdiv =
        (MXC_GCR->pclkdiv & ~(MXC_F_GCR_PCLKDIV_CNNCLKDIV | MXC_F_GCR_PCLKDIV_CNNCLKSEL)) |
        MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1 | MXC_S_GCR_PCLKDIV_CNNCLKSEL_IPLL;
    cnn_start(); // Start CNN processing

    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk; // SLEEPDEEP=0
    while (cnn_time == 0) __WFI(); // Wait for CNN

    // Switch CNN clock to PLL (200 MHz) div 4

    MXC_GCR->pclkdiv =
        (MXC_GCR->pclkdiv & ~(MXC_F_GCR_PCLKDIV_CNNCLKDIV | MXC_F_GCR_PCLKDIV_CNNCLKSEL)) |
        MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV4 | MXC_S_GCR_PCLKDIV_CNNCLKSEL_IPLL;
    if (check_output() != CNN_OK)
        fail();
    cnn_unload((uint32_t *)ml_data);

    printf("\n*** PASS ***\n\n");

#ifdef CNN_INFERENCE_TIMER
    printf("Approximate inference time: %u us\n\n", cnn_time);
#endif

    cnn_disable(); // Shut down CNN clock, disable peripheral

    MXC_GCR->ipll_ctrl &= ~MXC_F_GCR_IPLL_CTRL_EN; // Disable IPLL

    return 0;
}

/*
  SUMMARY OF OPS
  Hardware: 1,018,869,248 ops (1,014,353,408 macc; 3,926,272 comp; 589,568 add; 0 mul; 0 bitwise)
    Layer 0 (l0): 2,847,488 ops (2,709,504 macc; 137,984 comp; 0 add; 0 mul; 0 bitwise)
    Layer 1 (l1): 1,605,632 ops (1,605,632 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 2 (l2): 29,102,080 ops (28,901,376 macc; 200,704 comp; 0 add; 0 mul; 0 bitwise)
    Layer 3 (l3): 6,422,528 ops (6,422,528 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 4 (l4): 116,006,912 ops (115,605,504 macc; 401,408 comp; 0 add; 0 mul; 0 bitwise)
    Layer 5 (l5): 12,845,056 ops (12,845,056 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 6 (l6): 100,352 ops (0 macc; 0 comp; 100,352 add; 0 mul; 0 bitwise)
    Layer 7 (l7): 116,006,912 ops (115,605,504 macc; 401,408 comp; 0 add; 0 mul; 0 bitwise)
    Layer 8 (l8): 19,267,584 ops (19,267,584 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 9 (l9): 260,714,496 ops (260,112,384 macc; 602,112 comp; 0 add; 0 mul; 0 bitwise)
    Layer 10 (l10): 28,901,376 ops (28,901,376 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 11 (l11): 150,528 ops (0 macc; 0 comp; 150,528 add; 0 mul; 0 bitwise)
    Layer 12 (l12): 260,714,496 ops (260,112,384 macc; 602,112 comp; 0 add; 0 mul; 0 bitwise)
    Layer 13 (l13): 28,901,376 ops (28,901,376 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 14 (l14): 338,688 ops (0 macc; 301,056 comp; 37,632 add; 0 mul; 0 bitwise)
    Layer 15 (l15): 3,687,936 ops (3,612,672 macc; 75,264 comp; 0 add; 0 mul; 0 bitwise)
    Layer 16 (l16): 752,640 ops (677,376 macc; 75,264 comp; 0 add; 0 mul; 0 bitwise)
    Layer 17 (l17): 7,225,344 ops (7,225,344 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 18 (l18): 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 19 (l19): 14,601,216 ops (14,450,688 macc; 150,528 comp; 0 add; 0 mul; 0 bitwise)
    Layer 20 (l20): 1,505,280 ops (1,354,752 macc; 150,528 comp; 0 add; 0 mul; 0 bitwise)
    Layer 21 (l21): 14,450,688 ops (14,450,688 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 22 (l22): 75,264 ops (0 macc; 0 comp; 75,264 add; 0 mul; 0 bitwise)
    Layer 23 (l23): 14,601,216 ops (14,450,688 macc; 150,528 comp; 0 add; 0 mul; 0 bitwise)
    Layer 24 (l24): 1,505,280 ops (1,354,752 macc; 150,528 comp; 0 add; 0 mul; 0 bitwise)
    Layer 25 (l25): 19,267,584 ops (19,267,584 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 26 (l26): 3,336,704 ops (3,211,264 macc; 125,440 comp; 0 add; 0 mul; 0 bitwise)
    Layer 27 (l27): 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 28 (l28): 12,945,408 ops (12,845,056 macc; 100,352 comp; 0 add; 0 mul; 0 bitwise)
    Layer 29 (l29): 1,003,520 ops (903,168 macc; 100,352 comp; 0 add; 0 mul; 0 bitwise)
    Layer 30 (l30): 12,845,056 ops (12,845,056 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 31 (l31): 25,088 ops (0 macc; 0 comp; 25,088 add; 0 mul; 0 bitwise)
    Layer 32 (l32): 25,890,816 ops (25,690,112 macc; 200,704 comp; 0 add; 0 mul; 0 bitwise)
    Layer 33 (l33): 1,224,704 ops (1,024,000 macc; 0 comp; 200,704 add; 0 mul; 0 bitwise)

  RESOURCE USAGE
  Weight memory: 1,686,080 bytes out of 2,396,160 bytes total (70%)
  Bias memory:   5,544 bytes out of 8,192 bytes total (68%)
*/
