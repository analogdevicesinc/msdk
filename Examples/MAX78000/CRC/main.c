/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
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

/**
 * @file        main.c
 * @brief       Example showing how to use the CRC module. Covers 32-bit CRC.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "nvic_table.h"
#include "board.h"
#include "crc.h"
#include "dma.h"

/***** Definitions *****/
#define POLY 0xEDB88320
#define CHECK 0xDEBB20E3

#define SYNC 0
#define ASYNC 1

#define DATA_LENGTH 100

/***** Globals *****/
volatile int wait;
int fail = 0;

/***** Functions *****/
void DMA0_IRQHandler(void)
{
    MXC_DMA_Handler();
    MXC_DMA_ReleaseChannel(0);
    wait = 0;
}

void Test_CRC(int asynchronous)
{
    uint32_t array[101];
    int i;

    printf(asynchronous ? "TEST CRC ASYNC\n" : "TEST CRC SYNC\n");

    for (i = 0; i < DATA_LENGTH; i++) {
        array[i] = i;
    }

    // define the CRC parameters
    mxc_crc_req_t crc_req = {
        array, // pointer to data
        DATA_LENGTH, // length of data
        0 // initial crc result
    };

    MXC_CRC_Init();

    MXC_CRC_SetPoly(POLY);

    if (asynchronous) {
        wait = 1;
        MXC_CRC_ComputeAsync(&crc_req);

        while (wait) {}
    } else {
        MXC_CRC_Compute(&crc_req);
    }

    printf("\nComputed CRC: %x", crc_req.resultCRC);

    array[DATA_LENGTH] = ~crc_req.resultCRC;

    crc_req.dataLen = DATA_LENGTH + 1;

    MXC_CRC_Init();

    MXC_CRC_SetPoly(POLY);

    if (asynchronous) {
        wait = 1;
        MXC_CRC_ComputeAsync(&crc_req);

        while (wait) {}
    } else {
        MXC_CRC_Compute(&crc_req);
    }

    printf("\nCRC Check Result: %x", crc_req.resultCRC);

    MXC_CRC_Shutdown();

    if (CHECK != crc_req.resultCRC) {
        printf(" \n**Test Failed**\n\n");
        fail++;
    } else {
        printf(" \n**Test Passed**\n\n");
    }
}

// *****************************************************************************
int main(void)
{
    printf("\n***** CRC Example *****\n\n");

    Test_CRC(SYNC);

    //Release DMA Channel 0 to use it for CRC transaction (Any DMA channel can be used)
    MXC_DMA_ReleaseChannel(0);
    MXC_NVIC_SetVector(DMA0_IRQn, DMA0_IRQHandler);
    NVIC_EnableIRQ(DMA0_IRQn);
    __enable_irq();
    Test_CRC(ASYNC);

    if (fail != 0) {
        printf("\nExample Failed\n");
        return E_FAIL;
    }

    printf("\nExample Succeeded\n");
    return E_NO_ERROR;
}
