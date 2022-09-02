/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
 * 
 * This software is protected by copyright laws of the United States and
 * of foreign countries. This material may also be protected by patent laws
 * and technology transfer regulations of the United States and of foreign
 * countries. This software is furnished under a license agreement and/or a
 * nondisclosure agreement and may only be used or reproduced in accordance
 * with the terms of those agreements. Dissemination of this information to
 * any party or parties not specified in the license agreement and/or
 * nondisclosure agreement is expressly prohibited.
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
 * @file    main.c
 * @brief   OTP Dump Example
 * @details This example demonstrate how to read/write data from OTP memory
 *
 */

/***** Includes *****/
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <MAX32xxx.h>

/***** Definitions *****/
/*
 *  Attention if you set WITH_WRITE_TEST flag
 *  This example will search OTP and try to find 0xffffffff data
 *  If it could be found 4 test bytes will be written in that address.
 */
#define WITH_WRITE_TEST 0 // set it to test write test

// OTP Address
#define OTP_MAXIM_AREA      MXC_INFO0_MEM_BASE
#define OTP_MAXIM_AREA_SIZE MXC_INFO_MEM_SIZE

#define OTP_USER_AREA      MXC_INFO1_MEM_BASE
#define OTP_USER_AREA_SIZE MXC_INFO_MEM_SIZE

/***** Static Functions *****/
static void dump_section(unsigned int address, unsigned int length)
{
    unsigned int i;
    volatile uint32_t* addr = (uint32_t*)address;

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
    int ret                     = 0;
    uint32_t test_val           = 0x11223344;
    volatile uint32_t* addr     = (uint32_t*)OTP_USER_AREA;
    volatile uint32_t* end_addr = (uint32_t*)(OTP_USER_AREA + 1024);

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
    printf("***** This example demonstrates how you can read/write OTP memory *****\n");

    /* Dump maxim section */
    printf("\n\n***** MAXIM AREA *****\n");
    dump_section(OTP_MAXIM_AREA, 1024);

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
