/**
 * @file        main.c
 * @brief      True Random Number Generator (TRNG) example
 * @details
 */

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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "nvic_table.h"
#include "trng.h"

volatile int wait;
volatile int callback_result;

/***** Globals *****/
uint8_t var_rnd_no[16] = { 0 };

void TRNG_IRQHandler(void)
{
    MXC_TRNG_Handler();
}

void Test_Callback(void *req, int result)
{
    wait = 0;
    callback_result = result;
}

void print(char *stuff)
{
    int i, j, size = 4;

    for (i = 0; i < 4; ++i) {
        for (j = 0; j < 4; ++j) {
            printf("0x%02x ", stuff[i * size + j]);
        }

        printf("\n");
    }

    return;
}

void Test_TRNG(int asynchronous)
{
    printf(asynchronous ? "\nTest TRNG Async\n" : "\nTest TRNG Sync\n");

    int num_bytes = 16;

    memset(var_rnd_no, 0, sizeof(var_rnd_no));

    MXC_TRNG_Init();

    if (asynchronous) {
        wait = 1;
        NVIC_EnableIRQ(TRNG_IRQn);
        MXC_TRNG_RandomAsync(var_rnd_no, num_bytes, &Test_Callback);

        while (wait) {}
    } else {
        MXC_TRNG_Random(var_rnd_no, num_bytes);
    }

    print((char *)var_rnd_no);

    MXC_TRNG_Shutdown();
}

int main(void)
{
    printf("\n\n********** TRNG Example **********\n");

    Test_TRNG(0);
    Test_TRNG(1);

    printf("\n********** Test Complete **********\n");

    return 0;
}
