/******************************************************************************
 *
 * Copyright (C) 2025 Analog Devices, Inc.
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

/**
 * @file        main.c
 * @brief       DMA Example
 * @details     This runs through two DMA examples, first being memory-to-memory,
 *              second being a transfer with reload and callback.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mxc.h"

/***** Definitions *****/

#define MAX_CHANNEL 16
#define MAX_SIZE 64

/***** Globals *****/
int mychannel = -1;
volatile int flag = 0;

/***** Functions *****/
void my_int_func(int a, int b)
{
    int flags;
    flags = MXC_DMA_ChannelGetFlags(mychannel);
    MXC_DMA_ChannelClearFlags(mychannel, flags);
}

void memCpyComplete(void *dest)
{
    flag++;
}

void DMA1_CH0_IRQHandler(void)
{
    MXC_DMA_Handler(MXC_DMA1);
    LED_On(0);
}

int example1(void)
{
    printf("Transfer from memory to memory.\n");
    int fail = 0;
    int retval;
    int i = 0;

    //Initialize data before transfer
    uint8_t *srcdata, *dstdata;
    srcdata = (uint8_t *)malloc(MAX_SIZE);
    dstdata = (uint8_t *)malloc(MAX_SIZE);

    for (i = 0; i < MAX_SIZE; ++i) {
        srcdata[i] = i;
        dstdata[i] = 0;
    }

    retval = MXC_DMA_Init(MXC_DMA1);
    NVIC_EnableIRQ(DMA1_CH0_IRQn);
    
    if (retval != E_NO_ERROR) {
        printf("Failed MXC_DMA_Init().\n");
        fail += 1;
    } else {
        flag = 0;
        MXC_DMA_MemCpy(MXC_DMA1, dstdata, srcdata, MAX_SIZE, memCpyComplete);

        while (flag == 0) {}

        //Validate
        if (memcmp(srcdata, dstdata, MAX_SIZE) != 0) {
            printf("Data mismatch.\n");
            fail += 1;
        } else {
            printf("Data verified.\n");
        }
    }

    free(srcdata);
    free(dstdata);

    return fail;
}

int example2(void)
{
    printf("\nTransfer with Reload and Callback.\n");
    int fail = 0;
    int i, retval;

    //Init data
    uint8_t *srcdata, *dstdata, *srcdata2, *dstdata2;
    srcdata = (uint8_t *)malloc(MAX_SIZE);
    dstdata = (uint8_t *)malloc(MAX_SIZE);
    srcdata2 = (uint8_t *)malloc(MAX_SIZE);
    dstdata2 = (uint8_t *)malloc(MAX_SIZE);

    for (i = 0; i < MAX_SIZE; ++i) {
        srcdata[i] = i;
        dstdata[i] = 0;
        //Different set of data
        srcdata2[i] = MAX_SIZE - 1 - i;
        dstdata2[i] = 0;
    }

    NVIC_EnableIRQ(DMA1_CH0_IRQn);
    __enable_irq();
    MXC_DMA_Init(MXC_DMA1);
    mychannel = MXC_DMA_AcquireChannel(MXC_DMA1);

    mxc_dma_srcdst_t firstTransfer;
    firstTransfer.ch = mychannel;
    firstTransfer.source = srcdata;
    firstTransfer.dest = dstdata;
    firstTransfer.len = MAX_SIZE;

    mxc_dma_config_t config;
    config.ch = mychannel;
    config.reqsel = MXC_DMA_REQUEST_MEMTOMEM;
    config.srcwd = MXC_DMA_WIDTH_WORD;
    config.dstwd = MXC_DMA_WIDTH_WORD;
    config.srcinc_en = 1;
    config.dstinc_en = 1;

    mxc_dma_adv_config_t advConfig;
    advConfig.ch = mychannel;
    advConfig.prio = MXC_DMA_PRIO_HIGH;
    advConfig.reqwait_en = 0;
    advConfig.tosel = MXC_DMA_TIMEOUT_4_CLK;
    advConfig.pssel = MXC_DMA_PRESCALE_DISABLE;
    advConfig.burst_size = 32;

    MXC_DMA_ConfigChannel(config, firstTransfer);
    MXC_DMA_AdvConfigChannel(advConfig);

    mxc_dma_srcdst_t secondTransfer;
    secondTransfer.ch = mychannel;
    secondTransfer.source = srcdata2;
    secondTransfer.dest = dstdata2;
    secondTransfer.len = MAX_SIZE;

    MXC_DMA_SetSrcDst(firstTransfer);

    retval = MXC_DMA_SetSrcReload(secondTransfer);

    if (retval != E_NO_ERROR) {
        printf("Failed MXC_DMA_SetReload.\n");
    }

    MXC_DMA_SetCallback(mychannel, my_int_func);
    MXC_DMA_EnableInt(MXC_DMA1, mychannel);
    MXC_DMA_Start(mychannel);

    // Validate
    if (memcmp(srcdata, dstdata, MAX_SIZE) != 0 || memcmp(srcdata2, dstdata2, MAX_SIZE) != 0) {
        printf("Data mismatch.\n");
        fail += 1;
    } else {
        printf("Data verified.\n");
    }

    if (MXC_DMA_ReleaseChannel(mychannel) != E_NO_ERROR) {
        printf("Failed to release channel %d\n", mychannel);
        fail += 1;
    }

    free(srcdata);
    free(dstdata);
    free(srcdata2);
    free(dstdata2);

    return fail;
}

// *****************************************************************************
int main(void)
{
    int fail = 0;

    // Clear screen (ANSI escape code supported terminals only)
    printf("\033[H\033[J");

    printf("***** DMA Example *****\n");

    NVIC_EnableIRQ(DMA1_CH0_IRQn);
    fail += example1();
    fail += example2();

    if (fail != 0) {
        printf("\nExample Failed\n");
        return E_FAIL;
    }

    printf("\nExample Succeeded\n");
    return E_NO_ERROR;
}
