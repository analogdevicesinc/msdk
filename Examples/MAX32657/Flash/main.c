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
 * @file    main.c
 * @brief   Flash example.
 * @details This example demonstrates flash read/write/erase functions.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "flc.h"
#include "icc.h"
#include "board.h"
#include "mxc_delay.h"

/***** Definitions *****/
#define FLASH_BASE MXC_FLASH_MEM_BASE
#define FLASH_TEST_OFFSET (FLASH_BASE + (MXC_FLASH_MEM_SIZE / 2) + 0x2000)
#define FLASH_PAGE_SIZE (MXC_FLASH_PAGE_SIZE)
#define FLASH_TEST_SIZE (FLASH_PAGE_SIZE / 4)

#define CHECK_ERASED(buf, len) printf("%s\n", check_erased(buf, len) ? "FAIL" : "PASS")
#define CHECK_ERASED_ECC(ret, buf, len) printf("%s\n", check_erased_ecc(ret, buf, len) ? "FAIL" : "PASS")
#define CHECK_ERASED_NON_ALIGNED(buf, len, pos) printf("%s\n", check_erased_non_aligned(buf, len, pos) ? "FAIL" : "PASS")
#define CHECK_NUMBERS(buf, len, offset) printf("%s\n", check_numbers_with_offset(0, buf, len, offset) ? "FAIL" : "PASS")
#define CHECK_NUMBERS_ECC(ret, buf, len, offset) printf("%s\n", check_numbers_with_offset(ret, buf, len, offset) ? "FAIL" : "PASS")

/***** Globals *****/
extern int MXC_FLC_ReadAligned32(uint32_t address, void *buffer);
static const uint8_t erased_line_pattern[16] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD
};

static const uint8_t erased_line[16] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

/***** Functions *****/
void hexdump(uint8_t *buf, uint32_t len)
{
    uint32_t i;

    for (i = 0; i < len; i++) {
        if ((i % 16) == 0) {
            printf("\n0x%08X: ", (&buf[i]));
        }
        printf("%02X ", buf[i]);
    }
    printf("\n");
}

static int check_erased(uint8_t *buf, uint32_t len)
{
    while (len) {
        if (memcmp(buf, erased_line_pattern, 16) != 0) {
            return -1;
        }
        len -= 16;
        buf += 16;
    }

    return 0;
}

static int check_erased_non_aligned(uint8_t *buf, uint32_t len, uint32_t pos)
{
    while (len) {
        for (int i = 0; i < 16; i++) {
            if (i == pos) {
                if (buf[i] != 0xFD) {
                    return -1;
                }
            } else {
                if (buf[i] != 0xFF) {
                    return -1;
                }
            }
        }
        len -= 16;
        buf += 16;
    }

    return 0;
}

static int check_erased_ecc(int ret, uint8_t *buf, uint32_t len)
{
    if (ret != E_NO_ERROR) {
        return -1;
    }

    while (len) {
        if (memcmp(buf, erased_line, 16) != 0) {
            return -1;
        }
        len -= 16;
        buf += 16;
    }

    return 0;
}

static int check_numbers_with_offset(int ret, uint8_t *buf, uint32_t len, uint8_t offset)
{
    if (ret) {
        return -1;
    }

    for (int i = 0; i < len; i++) {
        if (buf[i] != ((i + offset) % 0xFF)) {
            return -1;
        }
    }

    return 0;
}

// *****************************************************************************
int main(void)
{
    int ret;
    uint8_t buf[FLASH_TEST_SIZE];
    uint8_t buf_ecc[FLASH_TEST_SIZE];
    uint32_t address = FLASH_TEST_OFFSET;

    printf("Flash test!\n");

    // printf("Flash test parameters:\n");
    // printf("Flash base address: 0x%08X\n", FLASH_BASE);
    // printf("Flash test offset: 0x%08X\n", FLASH_TEST_OFFSET);
    // printf("Flash page size: %d bytes\n", FLASH_PAGE_SIZE);
    // printf("ICC is: %s\n", (MXC_ICC->ctrl & MXC_F_ICC_CTRL_EN) ? "enabled" : "disabled");
#if 0
    MXC_FLC_PageErase(FLASH_TEST_OFFSET);
    // Erase the flash page
    MXC_FLC_Read(FLASH_TEST_OFFSET, buf, FLASH_TEST_SIZE);
    printf("Flash page after erase:\n");
    hexdump(buf, FLASH_TEST_SIZE);
#endif
    MXC_ICC_Disable();
    
    /* Erase test page */
    MXC_FLC_PageErase(address);

    /* Read 32 bytes from test page, we should observe ECC flipped bit in the 16th byte */
    printf("Reading 32 bytes from 0x%08X: ", address);
    MXC_FLC_Read(address, buf, 32);
    CHECK_ERASED(buf, 32);

    /* Now read using ECC workaround */
    printf("Reading 32 bytes from 0x%08X with ECC workaround: ", address);
    ret = MXC_FLC_ReadECC(address, buf_ecc, 32);
    CHECK_ERASED_ECC(ret, buf_ecc, 32);

    /* Read first 16 bytes */
    printf("Reading 16 bytes from 0x%08X: ", address);
    MXC_FLC_Read(address, buf, 16);
    CHECK_ERASED(buf, 16);

    /* Now read using ECC workaround */
    printf("Reading 16 bytes from 0x%08X with ECC workaround: ", address);
    ret = MXC_FLC_ReadECC(address, buf_ecc, 16);
    CHECK_ERASED_ECC(ret, buf_ecc, 16);

    /* Read from the middle */
    address = FLASH_TEST_OFFSET + 8;
    printf("Reading 16 bytes from 0x%08X: ", address);
    MXC_FLC_Read(address, buf, 16);
    CHECK_ERASED_NON_ALIGNED(buf, 16, 7);

    /* Now read using ECC workaround */
    printf("Reading 16 bytes from 0x%08X with ECC workaround: ", address);
    ret = MXC_FLC_ReadECC(address, buf_ecc, 16);
    CHECK_ERASED_ECC(ret, buf_ecc, 16);

    /* Write 0xFF to even line */
    address = FLASH_TEST_OFFSET;
    MXC_FLC_Write(address, 16, (uint32_t *)erased_line);

    /* Read even line */
    printf("Reading 16 bytes from 0x%08X: ", address);
    MXC_FLC_Read(address, buf, 16);
    CHECK_ERASED_ECC(0, buf, 16);

    /* Read odd line */
    address = FLASH_TEST_OFFSET + 16;
    printf("Reading 16 bytes from 0x%08X: ", address);
    MXC_FLC_Read(address, buf, 16);
    CHECK_ERASED(buf, 16);

    /* Now read using ECC workaround */
    address = FLASH_TEST_OFFSET;
    printf("Reading 16 bytes from 0x%08X with ECC workaround: ", address);
    ret = MXC_FLC_ReadECC(address, buf_ecc, 16);
    CHECK_ERASED_ECC(ret, buf_ecc, 16);
    
    /* Read even line */
    address = FLASH_TEST_OFFSET + 16;
    printf("Reading 16 bytes from 0x%08X with ECC workaround: ", address);
    ret = MXC_FLC_ReadECC(address, buf_ecc, 16);
    CHECK_ERASED_ECC(ret, buf_ecc, 16);

    /* Go to next page */
    address = FLASH_TEST_OFFSET + FLASH_PAGE_SIZE;
    MXC_FLC_PageErase(address);

    /* Read 32 bytes from test page, we should observe ECC flipped bit in the 16th byte */
    printf("Reading 32 bytes from 0x%08X: ", address);
    MXC_FLC_Read(address, buf, 32);
    CHECK_ERASED(buf, 32);

    /* Now read using ECC workaround */
    printf("Reading 32 bytes from 0x%08X with ECC workaround: ", address);
    ret = MXC_FLC_ReadECC(address, buf_ecc, 32);
    CHECK_ERASED_ECC(ret, buf_ecc, 32);

    /* Fill buffer with data */
    for (int i = 0; i < 32; i++) {
        buf[i] = i;
    }

    /* Write 32 bytes to test page */
    printf("Writing 32 bytes to 0x%08X: ", address);
    ret = MXC_FLC_Write(address, 32, (uint32_t *)buf);
    printf("%s\n", ret == E_NO_ERROR ? "PASS" : "FAIL");

    /* Read 32 bytes from test page */
    printf("Reading 32 bytes from 0x%08X: ", address);
    MXC_FLC_Read(address, buf, 32);
    CHECK_NUMBERS(buf, 32, 0);

    /* Now read using ECC workaround */
    printf("Reading 32 bytes from 0x%08X with ECC workaround: ", address);
    ret = MXC_FLC_ReadECC(address, buf_ecc, 32);
    CHECK_NUMBERS_ECC(ret, buf_ecc, 32, 0);

    /* Read from an offset */
    printf("Reading 32 bytes from 0x%08X: ", address);
    address += 6;
    MXC_FLC_Read(address, buf, 32);
    CHECK_NUMBERS(buf, 26, 6);

    /* Now read using ECC workaround */
    printf("Reading 32 bytes from 0x%08X with ECC workaround: ", address);    
    ret = MXC_FLC_ReadECC(address, buf_ecc, 32);
    CHECK_NUMBERS_ECC(ret, buf_ecc, 26, 6);

    /* Read from odd line */
    address = FLASH_TEST_OFFSET + FLASH_PAGE_SIZE;
    address += 18;
    printf("Reading 32 bytes from 0x%08X: ", address);
    MXC_FLC_Read(address, buf, 32);
    CHECK_NUMBERS(buf, 14, 18);

    /* Now read using ECC workaround */
    printf("Reading 32 bytes from 0x%08X with ECC workaround: ", address);
    ret = MXC_FLC_ReadECC(address, buf_ecc, 32);
    CHECK_NUMBERS_ECC(ret, buf_ecc, 14, 18);


    return 0;
}
