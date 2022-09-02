/**
 * @file
 * @brief      TRNG Example
 * @note       Generates random 32-bit number for first part followed by an
 *             application (AES) with TRNG
 */

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

/* **** Includes **** */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "nvic_table.h"
#include "mxc_errors.h"
#include "trng.h"

/* **** Definitions **** */
#define TRNG_32BIT_RND_NO 4  // Number of 32bit random numbers created
#define LEN               16 // Length of one random number (bytes)

/* **** Globals **** */
unsigned int rnd_no[TRNG_32BIT_RND_NO] = {0};
uint8_t var_rnd_no[LEN]                = {0};
volatile uint8_t TRNG_Async;

// *****************************************************************************
void trng_callback(void* req, int result)
{
    TRNG_Async = 0;
}

void TRNG_Handler(void)
{
    MXC_TRNG_Handler();
}

int main(void)
{
    printf("\n***** TRNG Example *****\n\n");

    MXC_TRNG_Init();

    //Reading and printing rnd 32-bit numbers
    printf("%i Random 32 Bit Integers\n", TRNG_32BIT_RND_NO);
    int i;
    for (i = 0; i < TRNG_32BIT_RND_NO; i++) {
        rnd_no[i] = MXC_TRNG_RandomInt();
        printf("0x%08x\n", rnd_no[i]);
    }

    // Reading and pring rnd number of length LEN
    printf("\nSynchronously Acquired Random %i-Bit Number", LEN * 8);

    MXC_TRNG_Random(var_rnd_no, LEN);

    for (i = 0; i < LEN; i++) {
        if (!(i % 4)) {
            printf("\n%01x-%01x: 0x", i, i + 3);
        }
        printf("%02x", var_rnd_no[i]);
    }

    // Acquire Random number, asynchronously
    printf("\n\nAsynchronusly Acquired Random %i-Bit Number", LEN * 8);
    TRNG_Async = 1;
    MXC_NVIC_SetVector(TRNG_IRQn, TRNG_Handler);
    NVIC_EnableIRQ(TRNG_IRQn);
    MXC_TRNG_RandomAsync(var_rnd_no, LEN, trng_callback);

    while (TRNG_Async)
        ;

    for (i = 0; i < LEN; i++) {
        if (!(i % 4)) {
            printf("\n%01x-%01x: 0x", i, i + 3);
        }
        printf("%02x", var_rnd_no[i]);
    }

    printf("\n\nExample complete.\n");

    while (1) {
    }
}
