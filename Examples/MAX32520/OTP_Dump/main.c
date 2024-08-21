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
 * @file    main.c
 * @brief   OTP Dump Example
 * @details This example shows how to read from and write to OTP memory.
 *
 */

/***** Includes *****/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "mxc_device.h"
#include "flc.h"

/***** Definitions *****/
/*
 *  Attention if you set WITH_WRITE_TEST flag
 *  This example will search OTP and try to find 0xffffffff data
 *  If it could be found 4 test bytes will be written in that address.
 */
#define WITH_WRITE_TEST 0 // set it to test write test

// OTP Address
#define OTP_MANUFACTURER_AREA MXC_INFO0_MEM_BASE
#define OTP_MANUFACTURER_AREA_SIZE 0x6000

#define OTP_USER_AREA MXC_INFO1_MEM_BASE
#define OTP_USER_AREA_SIZE 0x2000

/***** Static Functions *****/
static void dump_section(unsigned int address, unsigned int length)
{
    unsigned int i;
    volatile uint32_t *addr = (uint32_t *)address;

    // unlock otp to access it
    MXC_FLC_UnlockInfoBlock((uint32_t)address);

    length /= 4; // on each loop print 4 bytes

    for (i = 0; i < length; i++) {
        if (!(i % 4)) {
            printf("\n0x%08x:", (unsigned int)addr);
        }

        // add extra space
        if (!(i % 2)) {
            printf("   ");
        }

        printf(" %08x", *addr);
        addr++;
    }

    // lock otp
    MXC_FLC_LockInfoBlock((uint32_t)address);
}

#if WITH_WRITE_TEST
static int write_test(void)
{
    int ret = 0;
    uint32_t test_val = 0x11223344;
    volatile uint32_t *addr = (uint32_t *)OTP_USER_AREA;
    volatile uint32_t *end_addr = (uint32_t *)(OTP_USER_AREA + 1024);

    // unlock otp
    MXC_FLC_UnlockInfoBlock(OTP_USER_AREA);

    // find free slot
    while (addr < end_addr) {
        if (*addr == 0xffffffff) {
            printf("\n\nFree Addr: 0x%X\n", (uint32_t)addr);
            break;
        }

        addr++;
    }

    if (addr >= end_addr) {
        ret = -1; // means free slot not found
        printf("\nFree Index Not Found in OTP\n");
    }

    if (ret == 0) {
        ret = MXC_FLC_Write32((uint32_t)addr, test_val);

        if (ret) {
            printf("FLC Write Error: %d\n", ret);
        }
    }

    // lock otp
    MXC_FLC_LockInfoBlock(OTP_USER_AREA);

    if (ret == 0) {
        /* Dump user section */
        printf("\n\n***** After Write OTP Section *****\n");
        dump_section((unsigned int)addr, 32);
    }

    return ret;
}
#endif

//******************************************************************************
int main(void)
{
    printf("\n\n***** OTP Memory Read/Write Example *****\n");
    printf("***** This example demonstrates how to read/write OTP memory *****\n");

    /* Dump manufacturer section */
    printf("\n\n***** MANUFACTURER AREA *****\n");
    dump_section(OTP_MANUFACTURER_AREA, 1024);

    /* Dump user section */
    printf("\n\n***** USER AREA *****\n");
    dump_section(OTP_USER_AREA, 1024);

#if WITH_WRITE_TEST
    // run write test
    write_test();
#endif

    printf("\n\nExample End\n");

    return 0;
}
