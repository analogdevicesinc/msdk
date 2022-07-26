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

// cifar-100-mobilenet-v2-0.75
// Created using ai8xize.py --test-dir sdk/Examples/MAX78002/CNN --prefix cifar-100-mobilenet-v2-0.75 --checkpoint-file trained/ai87-cifar100-mobilenet-v2-0.75-qat8-q.pth.tar --config-file networks/ai87-cifar100-mobilenet-v2-0.75.yaml --softmax --device MAX78002 --timer 0 --display-checkpoint --verbose --overwrite

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
    while (1)
        ;
}

// 3-channel 32x32 data input (3072 bytes total / 1024 bytes per channel):
// HWC 32x32, channels 0 to 2
static const uint32_t input_0[] = SAMPLE_INPUT_0;

void load_input(void)
{
    // This function loads the sample data input -- replace with actual data

    memcpy32((uint32_t*)0x51800000, input_0, 1024);
}

// Expected output of layer 72 for cifar-100-mobilenet-v2-0.75 given the sample input (known-answer test)
// Delete this function for production code
static const uint32_t sample_output[] = SAMPLE_OUTPUT;
int check_output(void)
{
    int i;
    uint32_t mask, len;
    volatile uint32_t* addr;
    const uint32_t* ptr = sample_output;

    while ((addr = (volatile uint32_t*)*ptr++) != 0) {
        mask = *ptr++;
        len  = *ptr++;
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
    cnn_unload((uint32_t*)ml_data);
    softmax_q17p14_q15((const q31_t*)ml_data, CNN_NUM_OUTPUTS, ml_softmax);
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

    cnn_init();         // Bring state machine into consistent state
    cnn_load_weights(); // Load kernels
    cnn_load_bias();
    cnn_configure(); // Configure state machine
    load_input();    // Load data input
    // CNN clock: PLL (200 MHz) div 1
    MXC_GCR->pclkdiv =
        (MXC_GCR->pclkdiv & ~(MXC_F_GCR_PCLKDIV_CNNCLKDIV | MXC_F_GCR_PCLKDIV_CNNCLKSEL)) |
        MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1 | MXC_S_GCR_PCLKDIV_CNNCLKSEL_IPLL;
    cnn_start(); // Start CNN processing

    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk; // SLEEPDEEP=0
    while (cnn_time == 0)
        __WFI(); // Wait for CNN

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
  Hardware: 26,293,760 ops (25,695,744 macc; 566,016 comp; 32,000 add; 0 mul; 0 bitwise)
    Layer 0: 688,128 ops (663,552 macc; 24,576 comp; 0 add; 0 mul; 0 bitwise)
    Layer 1: 245,760 ops (221,184 macc; 24,576 comp; 0 add; 0 mul; 0 bitwise)
    Layer 2: 294,912 ops (294,912 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 3: 958,464 ops (884,736 macc; 73,728 comp; 0 add; 0 mul; 0 bitwise)
    Layer 4: 258,048 ops (165,888 macc; 92,160 comp; 0 add; 0 mul; 0 bitwise)
    Layer 5: 368,640 ops (368,640 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 6: 645,120 ops (614,400 macc; 30,720 comp; 0 add; 0 mul; 0 bitwise)
    Layer 7: 307,200 ops (276,480 macc; 30,720 comp; 0 add; 0 mul; 0 bitwise)
    Layer 8: 614,400 ops (614,400 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 9: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 10: 5,120 ops (0 macc; 0 comp; 5,120 add; 0 mul; 0 bitwise)
    Layer 11: 645,120 ops (614,400 macc; 30,720 comp; 0 add; 0 mul; 0 bitwise)
    Layer 12: 107,520 ops (69,120 macc; 38,400 comp; 0 add; 0 mul; 0 bitwise)
    Layer 13: 184,320 ops (184,320 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 14: 230,400 ops (221,184 macc; 9,216 comp; 0 add; 0 mul; 0 bitwise)
    Layer 15: 92,160 ops (82,944 macc; 9,216 comp; 0 add; 0 mul; 0 bitwise)
    Layer 16: 221,184 ops (221,184 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 17: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 18: 1,536 ops (0 macc; 0 comp; 1,536 add; 0 mul; 0 bitwise)
    Layer 19: 230,400 ops (221,184 macc; 9,216 comp; 0 add; 0 mul; 0 bitwise)
    Layer 20: 92,160 ops (82,944 macc; 9,216 comp; 0 add; 0 mul; 0 bitwise)
    Layer 21: 221,184 ops (221,184 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 22: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 23: 1,536 ops (0 macc; 0 comp; 1,536 add; 0 mul; 0 bitwise)
    Layer 24: 230,400 ops (221,184 macc; 9,216 comp; 0 add; 0 mul; 0 bitwise)
    Layer 25: 32,256 ops (20,736 macc; 11,520 comp; 0 add; 0 mul; 0 bitwise)
    Layer 26: 110,592 ops (110,592 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 27: 225,792 ops (221,184 macc; 4,608 comp; 0 add; 0 mul; 0 bitwise)
    Layer 28: 46,080 ops (41,472 macc; 4,608 comp; 0 add; 0 mul; 0 bitwise)
    Layer 29: 221,184 ops (221,184 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 30: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 31: 768 ops (0 macc; 0 comp; 768 add; 0 mul; 0 bitwise)
    Layer 32: 225,792 ops (221,184 macc; 4,608 comp; 0 add; 0 mul; 0 bitwise)
    Layer 33: 46,080 ops (41,472 macc; 4,608 comp; 0 add; 0 mul; 0 bitwise)
    Layer 34: 221,184 ops (221,184 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 35: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 36: 768 ops (0 macc; 0 comp; 768 add; 0 mul; 0 bitwise)
    Layer 37: 225,792 ops (221,184 macc; 4,608 comp; 0 add; 0 mul; 0 bitwise)
    Layer 38: 46,080 ops (41,472 macc; 4,608 comp; 0 add; 0 mul; 0 bitwise)
    Layer 39: 221,184 ops (221,184 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 40: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 41: 768 ops (0 macc; 0 comp; 768 add; 0 mul; 0 bitwise)
    Layer 42: 225,792 ops (221,184 macc; 4,608 comp; 0 add; 0 mul; 0 bitwise)
    Layer 43: 46,080 ops (41,472 macc; 4,608 comp; 0 add; 0 mul; 0 bitwise)
    Layer 44: 331,776 ops (331,776 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 45: 504,576 ops (497,664 macc; 6,912 comp; 0 add; 0 mul; 0 bitwise)
    Layer 46: 69,120 ops (62,208 macc; 6,912 comp; 0 add; 0 mul; 0 bitwise)
    Layer 47: 497,664 ops (497,664 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 48: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 49: 1,152 ops (0 macc; 0 comp; 1,152 add; 0 mul; 0 bitwise)
    Layer 50: 504,576 ops (497,664 macc; 6,912 comp; 0 add; 0 mul; 0 bitwise)
    Layer 51: 69,120 ops (62,208 macc; 6,912 comp; 0 add; 0 mul; 0 bitwise)
    Layer 52: 497,664 ops (497,664 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 53: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 54: 1,152 ops (0 macc; 0 comp; 1,152 add; 0 mul; 0 bitwise)
    Layer 55: 504,576 ops (497,664 macc; 6,912 comp; 0 add; 0 mul; 0 bitwise)
    Layer 56: 69,120 ops (62,208 macc; 6,912 comp; 0 add; 0 mul; 0 bitwise)
    Layer 57: 829,440 ops (829,440 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 58: 1,393,920 ops (1,382,400 macc; 11,520 comp; 0 add; 0 mul; 0 bitwise)
    Layer 59: 115,200 ops (103,680 macc; 11,520 comp; 0 add; 0 mul; 0 bitwise)
    Layer 60: 1,382,400 ops (1,382,400 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 61: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 62: 1,920 ops (0 macc; 0 comp; 1,920 add; 0 mul; 0 bitwise)
    Layer 63: 1,393,920 ops (1,382,400 macc; 11,520 comp; 0 add; 0 mul; 0 bitwise)
    Layer 64: 115,200 ops (103,680 macc; 11,520 comp; 0 add; 0 mul; 0 bitwise)
    Layer 65: 1,382,400 ops (1,382,400 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 66: 0 ops (0 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 67: 1,920 ops (0 macc; 0 comp; 1,920 add; 0 mul; 0 bitwise)
    Layer 68: 1,393,920 ops (1,382,400 macc; 11,520 comp; 0 add; 0 mul; 0 bitwise)
    Layer 69: 115,200 ops (103,680 macc; 11,520 comp; 0 add; 0 mul; 0 bitwise)
    Layer 70: 2,764,800 ops (2,764,800 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 71: 3,701,760 ops (3,686,400 macc; 15,360 comp; 0 add; 0 mul; 0 bitwise)
    Layer 72: 111,360 ops (96,000 macc; 0 comp; 15,360 add; 0 mul; 0 bitwise)

  RESOURCE USAGE
  Weight memory: 1,341,960 bytes out of 2,396,160 bytes total (56%)
  Bias memory:   6,508 bytes out of 8,192 bytes total (79%)
*/
