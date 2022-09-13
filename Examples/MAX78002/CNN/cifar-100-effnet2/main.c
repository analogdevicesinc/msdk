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

// cifar-100-effnet2
// Created using ai8xize.py --test-dir sdk/Examples/MAX78002/CNN --prefix cifar-100-effnet2 --checkpoint-file trained/ai87-cifar100-effnet2-qat8-q.pth.tar --config-file networks/ai87-cifar100-effnet2.yaml --softmax --device MAX78002 --timer 0 --display-checkpoint --verbose --overwrite

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

// 3-channel 32x32 data input (3072 bytes total / 1024 bytes per channel):
// HWC 32x32, channels 0 to 2
static const uint32_t input_60[] = SAMPLE_INPUT_60;

void load_input(void)
{
    // This function loads the sample data input -- replace with actual data

    memcpy32((uint32_t *)0x54860000, input_60, 1024);
}

// Expected output of layer 32 for cifar-100-effnet2 given the sample input (known-answer test)
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
  Hardware: 168,397,824 ops (166,899,712 macc; 1,158,144 comp; 339,968 add; 0 mul; 0 bitwise)
    Layer 0: 232,448 ops (221,184 macc; 11,264 comp; 0 add; 0 mul; 0 bitwise)
    Layer 1: 131,072 ops (131,072 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 2: 2,375,680 ops (2,359,296 macc; 16,384 comp; 0 add; 0 mul; 0 bitwise)
    Layer 3 (l3): 524,288 ops (524,288 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 4 (res00): 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 5: 9,469,952 ops (9,437,184 macc; 32,768 comp; 0 add; 0 mul; 0 bitwise)
    Layer 6 (res01): 1,048,576 ops (1,048,576 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 7: 8,192 ops (0 macc; 0 comp; 8,192 add; 0 mul; 0 bitwise)
    Layer 8: 9,469,952 ops (9,437,184 macc; 32,768 comp; 0 add; 0 mul; 0 bitwise)
    Layer 9 (l8): 1,572,864 ops (1,572,864 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 10 (res10): 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 11: 21,282,816 ops (21,233,664 macc; 49,152 comp; 0 add; 0 mul; 0 bitwise)
    Layer 12 (res11): 2,359,296 ops (2,359,296 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 13: 12,288 ops (0 macc; 0 comp; 12,288 add; 0 mul; 0 bitwise)
    Layer 14: 2,408,448 ops (2,359,296 macc; 49,152 comp; 0 add; 0 mul; 0 bitwise)
    Layer 15: 491,520 ops (442,368 macc; 49,152 comp; 0 add; 0 mul; 0 bitwise)
    Layer 16 (l14): 4,718,592 ops (4,718,592 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 17 (res20): 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 18: 9,535,488 ops (9,437,184 macc; 98,304 comp; 0 add; 0 mul; 0 bitwise)
    Layer 19: 983,040 ops (884,736 macc; 98,304 comp; 0 add; 0 mul; 0 bitwise)
    Layer 20 (res21): 9,437,184 ops (9,437,184 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 21: 24,576 ops (0 macc; 0 comp; 24,576 add; 0 mul; 0 bitwise)
    Layer 22: 9,535,488 ops (9,437,184 macc; 98,304 comp; 0 add; 0 mul; 0 bitwise)
    Layer 23: 983,040 ops (884,736 macc; 98,304 comp; 0 add; 0 mul; 0 bitwise)
    Layer 24 (l22): 12,582,912 ops (12,582,912 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 25 (res30): 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 26: 16,908,288 ops (16,777,216 macc; 131,072 comp; 0 add; 0 mul; 0 bitwise)
    Layer 27: 1,310,720 ops (1,179,648 macc; 131,072 comp; 0 add; 0 mul; 0 bitwise)
    Layer 28 (res31): 16,777,216 ops (16,777,216 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 29: 32,768 ops (0 macc; 0 comp; 32,768 add; 0 mul; 0 bitwise)
    Layer 30: 33,816,576 ops (33,554,432 macc; 262,144 comp; 0 add; 0 mul; 0 bitwise)
    Layer 31: 262,144 ops (0 macc; 0 comp; 262,144 add; 0 mul; 0 bitwise)
    Layer 32: 102,400 ops (102,400 macc; 0 comp; 0 add; 0 mul; 0 bitwise)

  RESOURCE USAGE
  Weight memory: 753,952 bytes out of 2,396,160 bytes total (31%)
  Bias memory:   5,236 bytes out of 8,192 bytes total (64%)
*/
