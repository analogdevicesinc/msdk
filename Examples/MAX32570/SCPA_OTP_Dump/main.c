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
 * @brief   SCP Applet OTP Dump Example
 * @details This example demonstrate how to SCP Applet mechanism,
 * 			The example shall be build with SCPA configuration, "make scpa"
 *
 */

/*********************************** Includes ********************************/
#include <stdio.h>
#include <string.h>

#include <MAX32xxx.h>
#include "utils.h"

/********************************  DEFINITIONS  ******************************/
#define UART_PORT MXC_UART_GET_UART(CONSOLE_UART)

#define OTP_MAXIM_AREA      MXC_INFO0_MEM_BASE
#define OTP_MAXIM_AREA_SIZE MXC_INFO_MEM_SIZE

#define OTP_USER_AREA      MXC_INFO1_MEM_BASE
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

        memcpy(buffer, (unsigned int*)address, size);

        for (i = 0; i < size; i++) {
            if (!(i % 8)) {
                tmp = (address + i);
                utils_byteArr2str(str, (unsigned char*)&tmp, 1, sizeof(int), "\n0x", ":");
                print_str((char*)str);
            }
            utils_byteArr2str(str, (unsigned char*)&buffer[i], 1, sizeof(char), " ", NULL);
            print_str((char*)str);
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

int scpa_erase(unsigned int dest, unsigned int length)
{
    (void)dest;

    switch (length) {
        case 0x10:
            print_str((char*)"\n****  MAXIM AREA OTP DUMP  ****");
            otp_dump(OTP_MAXIM_AREA, 1024);
            print_str((char*)"\n****  END ****\n");

            /* Character ASCII for end transmission */
            MXC_UART_WriteCharacter(UART_PORT, 0x04);
            break;

        case 0x20:
            print_str((char*)"\n****  USER AREA OTP DUMP  ****");
            otp_dump(OTP_USER_AREA, 256);
            print_str((char*)"\n****  END ****\n");

            /* Character ASCII for end transmission */
            MXC_UART_WriteCharacter(UART_PORT, 0x04);
            break;
    }
    return 0;
}
