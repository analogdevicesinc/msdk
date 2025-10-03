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
 * @brief       Example showing how to use the CRC module. Covers 16 and 32-bit CRC.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc.h"

/***** Definitions *****/
#define POLY 0xEDB88320
#define CHECK 0xDEBB20E3

/***** Globals *****/
volatile int wait;
volatile int callback_result;
volatile int counter;
volatile int fail = 0;

/***** Functions *****/
void DMA1_CH0_IRQHandler(void)
{
    MXC_DMA_Handler(MXC_DMA1);
    MXC_DMA_ReleaseChannel(4);
    wait = 0;
}

void Test_CRC(int asynchronous)
{
    uint32_t array[101];
    int i;

    printf(asynchronous ? "TEST CRC ASYNC\n" : "TEST CRC SYNC\n");

    for (i = 0; i < 100; i++) {
        array[i] = i;
    }

    mxc_crc_req_t crc_req = { array, 100, 0 };

    MXC_CRC_Init(MXC_DMA1);
    // Load CRC polynomial into crc polynomial register
    MXC_CRC_SetPoly(POLY);

    if (asynchronous) {
        wait = 1;
        MXC_CRC_ComputeAsync(&crc_req);

        while (wait) {}
    } else {
        MXC_CRC_Compute(&crc_req);
    }

    printf("\nCRC Poly Result: %x", crc_req.resultCRC);

    array[100] = ~(crc_req.resultCRC);

    crc_req.dataLen = 101;

    MXC_CRC_Init(MXC_DMA1);
    // Load CRC polynomial into crc polynomial register
    MXC_CRC_SetPoly(POLY);

    if (asynchronous) {
        wait = 1;
        MXC_CRC_ComputeAsync(&crc_req);

        while (wait) {}
    } else {
        MXC_CRC_Compute(&crc_req);
    }

    printf("\nCRC Check Result: %x", crc_req.resultCRC);

    if (CHECK != crc_req.resultCRC) {
        printf("\n\n**Test Failed**\n\n");
        fail++;
    } else {
        printf("\n\n**Test Passed**\n\n");
    }

    MXC_CRC_Shutdown();
}

// *****************************************************************************
int main(void)
{
    printf("\nCRC Sync and Async Example\n\n");

    Test_CRC(0);

    MXC_DMA_ReleaseChannel(4);
    MXC_NVIC_SetVector(DMA1_CH0_IRQn, DMA1_CH0_IRQHandler);
    NVIC_EnableIRQ(DMA1_CH0_IRQn);
    Test_CRC(1);

    if (fail) {
        printf("\nExample Failed\n");
        return E_FAIL;
    }

    printf("\nExample Succeeded\n");
    return E_NO_ERROR;
}
