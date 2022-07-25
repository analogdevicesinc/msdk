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
// Created using ai8xize.py --test-dir sdk/Examples/MAX78000/CNN --prefix svhn_tinierssd --checkpoint-file trained/ai85-svhn-tinierssd-qat8-q.pth.tar --config-file networks/svhn-tinierssd.yaml --overlap-data --device MAX78000 --timer 0 --display-checkpoint --verbose

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

// 3-channel 74x74 data input (16428 bytes total / 5476 bytes per channel):
// HWC 74x74, channels 0 to 2
static const uint32_t input_0[] = SAMPLE_INPUT_0;

void load_input(void)
{
    // This function loads the sample data input -- replace with actual data

    memcpy32((uint32_t*)0x50402000, input_0, 5476);
}

// Expected output of layer 12, 13, 14, 15, 16, 17, 18, 19 for svhn_tinierssd given the sample input (known-answer test)
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

static int32_t ml_data32[(CNN_NUM_OUTPUTS + 3) / 4];

int main(void)
{
    MXC_ICC_Enable(MXC_ICC0); // Enable cache

    // Switch to 100 MHz clock
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();

    printf("Waiting...\n");

    // DO NOT DELETE THIS LINE:
    MXC_Delay(SEC(2)); // Let debugger interrupt if needed

    // Enable peripheral, enable CNN interrupt, turn on CNN clock
    // CNN clock: APB (50 MHz) div 1
    cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);

    printf("\n*** CNN Inference Test ***\n");

    cnn_init();         // Bring state machine into consistent state
    cnn_load_weights(); // Load kernels
    cnn_load_bias();
    cnn_configure(); // Configure state machine
    load_input();    // Load data input
    cnn_start();     // Start CNN processing

    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk; // SLEEPDEEP=0
    while (cnn_time == 0) __WFI();      // Wait for CNN

    if (check_output() != CNN_OK)
        fail();
    cnn_unload((uint32_t*)ml_data32);

    printf("\n*** PASS ***\n\n");

#ifdef CNN_INFERENCE_TIMER
    printf("Approximate inference time: %u us\n\n", cnn_time);
#endif

    cnn_disable(); // Shut down CNN clock, disable peripheral

    return 0;
}

/*
  SUMMARY OF OPS
  Hardware: 199,903,840 ops (198,904,320 macc; 999,520 comp; 0 add; 0 mul; 0 bitwise)
    Layer 0: 4,906,496 ops (4,731,264 macc; 175,232 comp; 0 add; 0 mul; 0 bitwise)
    Layer 1: 50,642,048 ops (50,466,816 macc; 175,232 comp; 0 add; 0 mul; 0 bitwise)
    Layer 2: 25,496,256 ops (25,233,408 macc; 262,848 comp; 0 add; 0 mul; 0 bitwise)
    Layer 3: 50,554,432 ops (50,466,816 macc; 87,616 comp; 0 add; 0 mul; 0 bitwise)
    Layer 4: 12,151,296 ops (11,943,936 macc; 207,360 comp; 0 add; 0 mul; 0 bitwise)
    Layer 5: 11,964,672 ops (11,943,936 macc; 20,736 comp; 0 add; 0 mul; 0 bitwise)
    Layer 6: 23,929,344 ops (23,887,872 macc; 41,472 comp; 0 add; 0 mul; 0 bitwise)
    Layer 7 (backbone_conv8): 11,954,304 ops (11,943,936 macc; 10,368 comp; 0 add; 0 mul; 0 bitwise)
    Layer 8 (backbone_conv9): 759,456 ops (746,496 macc; 12,960 comp; 0 add; 0 mul; 0 bitwise)
    Layer 9 (backbone_conv10): 152,576 ops (147,456 macc; 5,120 comp; 0 add; 0 mul; 0 bitwise)
    Layer 10 (conv12_1): 73,984 ops (73,728 macc; 256 comp; 0 add; 0 mul; 0 bitwise)
    Layer 11 (conv12_2): 9,536 ops (9,216 macc; 320 comp; 0 add; 0 mul; 0 bitwise)
    Layer 12: 1,492,992 ops (1,492,992 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 13: 373,248 ops (373,248 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 14: 73,728 ops (73,728 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 15: 9,216 ops (9,216 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 16: 4,105,728 ops (4,105,728 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 17: 1,026,432 ops (1,026,432 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 18: 202,752 ops (202,752 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 19: 25,344 ops (25,344 macc; 0 comp; 0 add; 0 mul; 0 bitwise)

  RESOURCE USAGE
  Weight memory: 335,520 bytes out of 442,368 bytes total (76%)
  Bias memory:   816 bytes out of 2,048 bytes total (40%)
*/
