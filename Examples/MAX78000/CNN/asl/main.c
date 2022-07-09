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

// asl

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "mxc.h"
#include "cnn.h"
#include "sampledata.h"

volatile uint32_t cnn_time; // Stopwatch

void fail(void)
{
    printf("\n*** FAIL ***\n\n");
    
    while (1);
}

// 3-channel 64x64 data input (12288 bytes total / 4096 bytes per channel):
// CHW 64x64, channel 0
static const uint32_t input_0[] = SAMPLE_INPUT_0;

// CHW 64x64, channel 1
static const uint32_t input_16[] = SAMPLE_INPUT_1;

// CHW 64x64, channel 2
static const uint32_t input_32[] = SAMPLE_INPUT_2;

void load_input(void)
{
    // This function loads the sample data input -- replace with actual data
    
    memcpy32((uint32_t*) 0x50400000, input_0, 1024);
    memcpy32((uint32_t*) 0x50800000, input_16, 1024);
    memcpy32((uint32_t*) 0x50c00000, input_32, 1024);
}

// Expected output of layer 6 for asl given the sample input
int check_output(void)
{
    if ((*((volatile uint32_t*) 0x50401000)) != 0xfffec932) {
        return CNN_FAIL;    // 0,0,0
    }
    
    if ((*((volatile uint32_t*) 0x50401004)) != 0x0001355b) {
        return CNN_FAIL;    // 0,0,1
    }
    
    return CNN_OK;
}

// Classification layer:
static int32_t ml_data[CNN_NUM_OUTPUTS];
static q15_t ml_softmax[CNN_NUM_OUTPUTS];

void softmax_layer(void)
{
    cnn_unload((uint32_t*) ml_data);
    softmax_q17p14_q15((const q31_t*) ml_data, CNN_NUM_OUTPUTS, ml_softmax);
}

int main(void)
{
    int i;
    int digs, tens;
    
    MXC_ICC_Enable(MXC_ICC0); // Enable cache
    
    // Switch to 100 MHz clock
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();
    
    printf("Waiting...\n");
    
    // DO NOT DELETE THIS LINE:
    MXC_Delay(SEC(2)); // Let debugger interrupt if needed
    
    // Enable peripheral, enable CNN interrupt, turn on CNN clock
    // CNN clock: 50 MHz div 1
    cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);
    
    printf("\n*** CNN Inference Test ***\n");
    
    cnn_init(); // Bring state machine into consistent state
    cnn_load_weights(); // Load kernels
    cnn_load_bias();
    cnn_configure(); // Configure state machine
    load_input(); // Load data input
    cnn_start(); // Start CNN processing
    
    while (cnn_time == 0) {
        __WFI();    // Wait for CNN
    }
    
    if (check_output() != CNN_OK) {
        fail();
    }
    
    softmax_layer();
    
    printf("\n*** PASS ***\n\n");
    
#ifdef CNN_INFERENCE_TIMER
    printf("Approximate inference time: %d us\n\n", cnn_time);
#endif
    
    cnn_disable(); // Shut down CNN clock, disable peripheral
    
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
  Hardware: 11,470,080 ops (11,250,240 macc; 219,840 comp; 0 add; 0 mul; 0 bitwise)

  RESOURCE USAGE
  Weight memory: 54,015 bytes out of 442,368 bytes total (12%)
  Bias memory:   2 bytes out of 2,048 bytes total (0%)
*/
