/**
 * @file   main.c
 * @brief  MAA Example
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
#include "mxc_errors.h"
#include "max32650.h"
#include "board.h"
#include "tpu.h"

/* **** Definitions **** */
#define ARR_SIZE 512

/* **** Globals **** */
char temp[] = { 0x00, 0x00, 0x00 };

char multiplier_data[] = { 0x10, 0x32, 0x54, 0x76, 0x98, 0xba, 0xdc, 0xfe,
                           0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12 };

char multiplicand_data[] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

char exponent_data[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                         0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f };

char modulus_data[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f };

char result_data[] = { 0x10, 0x32, 0x54, 0x76, 0x98, 0xba, 0xdc, 0xfe,
                       0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12 };

/* **** Functions **** */

// *****************************************************************************
void ascii_to_byte(const char *src, char *dst, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        int val;
        temp[0] = *src;
        src++;
        temp[1] = *src;
        src++;
        sscanf(temp, "%0x", &val);
        dst[i] = val;
    }
    return;
}

// *****************************************************************************
unsigned int findLength(int num)
{
    unsigned int len = 0;
    len = num / 8;
    if (num % 8 != 0) {
        len++; //rounding up
    }
    return len;
}

// *****************************************************************************
int verify(int *result)
{
    printf("0x%08x\n", *(result + 0));
    printf("0x%08x\n", *(result + 1));
    printf("0x%08x\n", *(result + 2));
    printf("0x%08x\n", *(result + 3));

    return 0;
}

// *****************************************************************************
int main(void)
{
    printf("\n***** MAA Example *****\n");

    int result[ARR_SIZE];
    int len = 127;
    int retval, i, j;

    //Init
    retval = MXC_TPU_MAA_Init(len);
    if (retval != E_SUCCESS) {
        printf("Failed MAA_Init().\n");
        return -1;
    }

    //Need to know the length of data
    len = findLength(len);

    //Compute MAA
    retval = MXC_TPU_MAA_Compute(MXC_TPU_MAA_EXP, multiplier_data, multiplicand_data, exponent_data,
                                 modulus_data, result, len);
    if (retval != E_SUCCESS) {
        printf("Failed MAA_Compute().\n");
        return -1;
    }

    //Fit into four byte array
    printf("Computed:\n");
    for (i = 0; i < len / 4; ++i) { printf("result[%d] = 0x%x\n", i, *(result + i)); }

    //Print out expected array to compare to calculated result
    printf("Expected:\n");
    for (i = 1; i <= len / 4; ++i) {
        printf("expected[%d] = 0x", i);
        for (j = 0; j < 4; ++j) { printf("%02x", result_data[i * len / 4 - 1 - j]); }
        printf("\n");
    }

    printf("\nExample complete.\n");

    return 0;
}
