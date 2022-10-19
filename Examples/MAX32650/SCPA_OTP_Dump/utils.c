/*
 ******************************************************************************
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
 ******************************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "utils.h"
#include "uart.h"
#include "board.h"

#define UART_PORT MXC_UART_GET_UART(CONSOLE_UART)

static const char *hex = "0123456789ABCDEF";
/****************************** Static Functions *****************************/
char *my_itoa(int value, char *str, int base)
{
    int i = 0;
    int tmpValue = value;

    // find number chrachter
    for (i = 0; tmpValue; i++) { tmpValue /= base; }

    if (i == 0) {
        i = 1; // 0 case
    }

    // end of string
    str[i] = '\0';

    if (base == 10) {
        while (--i >= 0) {
            str[i] = (value % 10) + '0';
            value /= 10;
        }
    } else {
        int tmpval;
        while (--i >= 0) {
            tmpval = (value % 16);
            if (tmpval < 10) {
                str[i] = (tmpval) + '0';
            } else {
                str[i] = (tmpval - 10) + 'A';
            }
            value /= 16;
        }
    }
    return str;
}

/****************************** Public Functions *****************************/
void print_str(const char *str)
{
    int len = strlen(str);
    MXC_UART_Write(UART_PORT, (unsigned char *)str, &len);
}

void print_hex(int value)
{
    char str[32];
    my_itoa(value, str, 16);
    print_str(str);
}

void print_decimal(int value)
{
    char str[32];
    my_itoa(value, str, 10);
    print_str(str);
}

void utils_byteArr2str(unsigned char *dst, unsigned char *arr, int numberOfItem, int itemsize,
                       const char *prefix, const char *postfix)
{
    int i, k;
    int len_prefix = 0;
    int len_postfix = 0;

    if (prefix) {
        len_prefix = strlen(prefix);
    }
    if (postfix) {
        len_postfix = strlen(postfix);
    }

    for (i = 0; i < numberOfItem; i++) {
        memcpy(dst, prefix, len_prefix);
        dst += len_prefix;

        for (k = itemsize - 1; k >= 0; k--) {
            *dst++ = hex[(arr[k] >> 4) & 0xF];
            *dst++ = hex[(arr[k] >> 0) & 0xF];
        }
        memcpy(dst, postfix, len_postfix);
        dst += len_postfix;
    }
    *dst = '\0';
}
