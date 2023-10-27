/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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
#include <stdbool.h>

#include "dma.h"
#include "mxc_errors.h"
#include "nvic_table.h"

/***** Definitions *****/
#define MAX_CHANNEL 16
#define MAX_SIZE 64

/***** Globals *****/
int mychannel = -1;
volatile int dma_done = 0;

/***** Functions *****/
void dma_callback(int ch, int err)
{
    dma_done++;
}

void DMA_Handler()
{
    MXC_DMA_Handler();
}

int example1(void)
{
    printf("Transfer from memory to memory.\n");
    int fail = 0;
    int retval;

    //Initialize data before transfer
    uint8_t srcdata[MAX_SIZE];
    uint8_t dstdata[MAX_SIZE];

    // Initialize arrays
    for (int i = 0; i < MAX_SIZE; ++i) {
        srcdata[i] = i;
        dstdata[i] = 0;
    }

    // Initialize DMA
    retval = MXC_DMA_Init();
    if (retval != E_NO_ERROR) {
        printf("Failed MXC_DMA_Init().\n");
        fail += 1;
        return fail;
    }

    // Acquire DMA Channel
    mychannel = MXC_DMA_AcquireChannel();
    if (mychannel < E_NO_ERROR) {
    	fail += 1;
    	return fail;
    }

	// Set DMA transfer structures
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

	// Configure DMA Channel
	retval = MXC_DMA_ConfigChannel(config, firstTransfer);
	if (retval != E_NO_ERROR) {
		printf("Failed to config channel\n");
		fail += 1;
		return fail;
	}

	// Configure DMA Interrupts
	MXC_DMA_SetCallback(mychannel, dma_callback);
    NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(mychannel));
    MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(mychannel), DMA_Handler);
    MXC_DMA_SetChannelInterruptEn(mychannel, false, true);
    MXC_DMA_EnableInt(mychannel);

	// Initiate Mem-to-Mem DMA Transfer
    printf("Starting transfer\n");
	dma_done = 0;
	if (MXC_DMA_Start(mychannel) != E_NO_ERROR) {
		printf("Failed to start.\n");
		while(1);
	}

	// Wait for transfer to complete
	while(dma_done == 0) {}

	// Validate transfer was successful
	if (memcmp(srcdata, dstdata, MAX_SIZE) != 0) {
		printf("Data mismatch.\n");
		fail += 1;
	} else {
		printf("Data verified.\n");
	}

	// Release DMA channel
	if (MXC_DMA_ReleaseChannel(mychannel) != E_NO_ERROR) {
		printf("Failed to release channel 0\n");
		while(1);
	}

    return fail;
}

int example2(void)
{
    printf("\nTransfer with Reload and Callback.\n");
    int fail = 0;
    int retval;

    // Initialize data
    uint8_t srcdata[MAX_SIZE], dstdata[MAX_SIZE];
    uint8_t srcdata2[MAX_SIZE], dstdata2[MAX_SIZE];

    for (int i = 0; i < MAX_SIZE; ++i) {
        srcdata[i] = i;
        dstdata[i] = 0;
        srcdata2[i] = MAX_SIZE - 1 - i;
        dstdata2[i] = 0;
    }

    // Initialize DMA
    MXC_DMA_Init();

    // Acquire DMA Channel
    mychannel = MXC_DMA_AcquireChannel();
    if(mychannel < E_NO_ERROR) {
    	printf("Failed to acquire channel.\n");
    	fail += 1;
    	return fail;
    }

    // Set channel config structures
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

    // Initialize first transfer structure
    mxc_dma_srcdst_t firstTransfer;
    firstTransfer.ch = mychannel;
    firstTransfer.source = srcdata;
    firstTransfer.dest = dstdata;
    firstTransfer.len = MAX_SIZE;

    // Initialize second transfer structure
    mxc_dma_srcdst_t secondTransfer;
    secondTransfer.ch = mychannel;
    secondTransfer.source = srcdata2;
    secondTransfer.dest = dstdata2;
    secondTransfer.len = MAX_SIZE;

    // Configure DMA Channel
    MXC_DMA_ConfigChannel(config, firstTransfer);
    MXC_DMA_AdvConfigChannel(advConfig);

    // Configure back-to-back DMA transfers
    MXC_DMA_SetSrcDst(firstTransfer);
    retval = MXC_DMA_SetSrcReload(secondTransfer);
    if (retval != E_NO_ERROR) {
        printf("Failed MXC_DMA_SetReload.\n");
        fail += 1;
        return fail;
    }

    // Configure interrupts for the DMA channel
    MXC_DMA_SetCallback(mychannel, dma_callback);
    NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(mychannel));
    MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(mychannel), DMA_Handler);
    MXC_DMA_SetChannelInterruptEn(mychannel, true, true);
    MXC_DMA_EnableInt(mychannel);

    // Initiate mem-to-mem transfers
    dma_done = 0;
    MXC_DMA_Start(mychannel);

    // Wait for transfers to complete
    while (dma_done == 0) {}

    // Validate Transfers were successful
    if (memcmp(srcdata, dstdata, MAX_SIZE) != 0 || memcmp(srcdata2, dstdata2, MAX_SIZE) != 0) {
        printf("Data mismatch.\n");
        fail += 1;
    } else {
        printf("Data verified.\n");
    }

    if (MXC_DMA_ReleaseChannel(mychannel) != E_NO_ERROR) {
        printf("Failed to release channel 0\n");
        fail += 1;
    }

    return fail;
}

// *****************************************************************************
int main(void)
{
    int fail = 0;
    printf("\n********** DMA Example **********\n");

    fail += example1();
    fail += example2();

    if (fail != 0) {
        printf("\nExample failed\n");
        return E_FAIL;
    }

    printf("\nExample Succeeded\n");
    return E_NO_ERROR;
}
