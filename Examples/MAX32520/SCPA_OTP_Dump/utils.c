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
    for (i = 0; tmpValue; i++) {
        tmpValue /= base;
    }

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
