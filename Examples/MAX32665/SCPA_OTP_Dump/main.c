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
 * @brief   SCP Applet OTP Dump Example
 * @details This example demonstrate how to SCP Applet mechanism,
 * 			The example shall be build with SCPA configuration, "make scpa"
 *
 */

/***** Includes *****/
#include <stdio.h>
#include <string.h>

#include "mxc_device.h"
#include "board.h"
#include "uart.h"
#include "flc.h"

#include "utils.h"

/********************************  DEFINITIONS  ******************************/
#define UART_PORT MXC_UART_GET_UART(CONSOLE_UART)

#define OTP_MAXIM_AREA MXC_INFO0_MEM_BASE
#define OTP_MAXIM_AREA_SIZE MXC_INFO_MEM_SIZE

#define OTP_USER_AREA MXC_INFO1_MEM_BASE
#define OTP_USER_AREA_SIZE MXC_INFO_MEM_SIZE

/****************************  Static Functions  *****************************/
static int otp_dump(unsigned int address, unsigned int length)
{
    int ret = 0;
    unsigned int i;
    unsigned char buffer[512];
    unsigned char str[32];
    unsigned int tmp;

    MXC_FLC_UnlockInfoBlock(address);

    while (length) {
        unsigned int size = (length > sizeof(buffer)) ? sizeof(buffer) : length;

        memset(buffer, 0x00, sizeof(buffer));

        memcpy(buffer, (unsigned int *)address, size);

        for (i = 0; i < size; i++) {
            if (!(i % 8)) {
                tmp = (address + i);
                utils_byteArr2str(str, (unsigned char *)&tmp, 1, sizeof(int), "\n0x", ":");
                print_str((char *)str);
            }
            utils_byteArr2str(str, (unsigned char *)&buffer[i], 1, sizeof(char), " ", NULL);
            print_str((char *)str);
        }
        length -= size;
        address += size;
    }
    print_str("\n");

    MXC_FLC_LockInfoBlock(address);

    return ret;
}

/****************************  Public Functions  *****************************/
#if !defined(__SCPA_FWK__)
#error "This example shall be build with SCPA configuration"
#endif

/*
 *  This example can be executed on the Secure micro controller which has Secure ROM Bootloader,
 *  If part number you have is not secure one you can not execute it on the device
 */
int scpa_erase(unsigned int dest, unsigned int length)
{
    (void)dest;

    switch (length) {
    case 0x10:
        print_str((char *)"\n****  MAXIM AREA OTP DUMP  ****");
        otp_dump(OTP_MAXIM_AREA, 1024);
        print_str((char *)"\n****  END ****\n");

        /* Character ASCII for end transmission */
        MXC_UART_WriteCharacter(UART_PORT, 0x04);
        break;

    case 0x20:
        print_str((char *)"\n****  USER AREA OTP DUMP  ****");
        otp_dump(OTP_USER_AREA, 256);
        print_str((char *)"\n****  END ****\n");

        /* Character ASCII for end transmission */
        MXC_UART_WriteCharacter(UART_PORT, 0x04);
        break;
    }
    return 0;
}
