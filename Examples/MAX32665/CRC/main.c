/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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
 * @file    	main.c
 * @brief   	Example showing how to use the CRC module. Covers 16 and 32-bit CRC.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "mxc_errors.h"
#include "board.h"
#include "tpu.h"

/***** Definitions *****/

/***** Globals *****/

/***** Functions *****/
int crc_sw(uint8_t *data, uint8_t len, uint32_t polynomial)
{
    int i, j;
    uint32_t temp;
    uint32_t crcval = 0;

    for (i = 0; i < len; ++i) {
        temp = (data[i] & 0xFF);
        for (j = 0; j < 8; ++j) {
            if ((temp ^ crcval) & 1) {
                crcval >>= 1;
                crcval ^= polynomial;
            } else {
                crcval >>= 1;
            }
            temp >>= 1;
        }
    }

    return crcval;
}

// *****************************************************************************
int main(void)
{
    printf("\n***** CRC Example *****\n");

    uint8_t data[] = { 0x14, 0x78, 0x9C, 0xDE };
    uint8_t len = sizeof(data) / sizeof(uint8_t);
    int fail = 0;

    if (MXC_TPU_CRC_Config() != E_SUCCESS) {
        printf("Failed MXC_TPU_Crc_Config()\n");
        return E_FAIL;
    }

    printf("CRC16:\n");

    //Calculate correct result
    uint32_t sw_crc = crc_sw(data, len, MXC_TPU_CRC16);
    uint32_t hw_crc;

    //Generate hardware result
    if (MXC_TPU_CRC(data, len, MXC_TPU_CRC16, &hw_crc) != E_SUCCESS) {
        printf("Failed MXC_TPU_CRC()\n");
        return E_FAIL;
    }

    fail += memcmp(&hw_crc, &sw_crc, sizeof(hw_crc));
    printf("Calculated CRC = 0x%08x\n", hw_crc);
    printf("Expected CRC   = 0x%08x\n", sw_crc);
    printf("\n");

    //Call TPU_Crc_Config() again to reset
    if (MXC_TPU_CRC_Config() != E_SUCCESS) {
        printf("Failed MXC_TPU_Crc_Config()\n");
        return E_FAIL;
    }

    printf("CRC32:\n");

    //Calculate correct result
    sw_crc = crc_sw(data, len, MXC_TPU_CRC32_ETHERNET);

    //Generate hardware result
    if (MXC_TPU_CRC(data, len, MXC_TPU_CRC32_ETHERNET, &hw_crc) != E_SUCCESS) {
        printf("Failed MXC_TPU_CRC()\n");
        return E_FAIL;
    }

    fail += memcmp(&hw_crc, &sw_crc, sizeof(hw_crc));
    printf("Calculated CRC = 0x%08x\n", hw_crc);
    printf("Expected CRC   = 0x%08x\n", sw_crc);

    if (fail != 0) {
        printf("\nExample Failed\n");
        return E_FAIL;
    }

    printf("\nExample Succeeded\n");
    return E_NO_ERROR;
}
