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

/*******************************      INCLUDES    ****************************/
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "uart.h"
#include "terminal.h"
#include "board.h"

/*******************************      DEFINES     ****************************/
#if defined(BOARD_FTHR)
#define PC_COM_PORT MXC_UART1
#elif defined(BOARD_FTHR2)
#define PC_COM_PORT MXC_UART0
#else
#define PC_COM_PORT MXC_UART1
#endif

/******************************* Type Definitions ****************************/

/*******************************    Variables   ****************************/

/******************************* Static Functions ****************************/

/******************************* Public Functions ****************************/
int terminal_init(void)
{
    int ret = 0;
    sys_map_t uart_pin_map;

#if defined(BOARD_FTHR)
    uart_pin_map = MAP_B;
#elif defined(BOARD_FTHR2)
    uart_pin_map = MAP_A;
#else
    uart_pin_map = MAP_A;
#endif

    ret = MXC_UART_Init(PC_COM_PORT, 115200, uart_pin_map);

    return ret;
}

int terminal_read_num(unsigned int timeout)
{
    (void)timeout;
    int key;
    int num = 0;

    while (1) {
        key = MXC_UART_ReadCharacter(PC_COM_PORT);

        if (key > 0) {
            //echo
            MXC_UART_WriteCharacter(PC_COM_PORT, (unsigned char)key);

            if ((key >= '0') && (key <= '9')) {
                num = num * 10 + (key - '0');
            }

            if ((key == '\n') || (key == '\r')) {
                break;
            } else if (key == 0x1B) { // Escape char 0x1B = 27
                num = -1;
                break;
            }
        }
    }

    MXC_UART_ClearRXFIFO(PC_COM_PORT);

    return num;
}

int terminal_select_from_list(const char *title, const list_t *items, int nb_items, int nb_col)
{
    int i, k;
    int key = 0;
    int nb_row;
    char buf[512];
    char item_data[64];
    int index;

    if (title) {
        terminal_printf("\r\n\r\n%s\r\n", title);
    }
    terminal_printf("---------------------------------------------------------\r\n");

    if (nb_col == 0) {
        nb_col = 1;
    }

    nb_row = nb_items / nb_col;

    if (nb_items % 2) {
        nb_row++;
    }

    if (nb_items > 1) {
        for (i = 0; i < nb_row; i++) {
            buf[0] = '\0';

            for (k = 0; k < nb_col; k++) {
                index = i + (k * nb_row);

                if (index < nb_items) {
                    snprintf(item_data, sizeof(item_data), "%-3d- %-32s ", index + 1, items[index].name);
                    memcpy(buf, item_data, sizeof(item_data));
                }
            }
            //
            terminal_printf("%s\r\n", buf);
        }

        terminal_printf("\r\nPlease select: ");
        key = terminal_read_num(0);
        terminal_printf("\r\n");
    } else if (nb_items == 1) {
        key = 1;
    }

    if ((key > 0) && (key <= nb_items)) {
        if (items[key - 1].callback) {
            int ret;
            ret = items[key - 1].callback(items[key - 1].name); // index start from 0
            if (ret == 0) {
                terminal_printf("\r\n%s: SUCCESS\r\n\r\n", items[key - 1].name);
            } else {
                terminal_printf("\r\n%s: FAILURE\r\n\r\n", items[key - 1].name);
            }
        }
    } else if (key == -1) {
        key = KEY_CANCEL;
    } else {
        terminal_printf("ERR:Invalid input\r\n\r\n");
    }

    return key;
}

int terminal_printf(const char *format, ...)
{
    char buffer[512];
    int len;

    __gnuc_va_list args;
    va_start(args, format);
    len = vsnprintf(buffer, sizeof(buffer), format, args);
    if (len > 0) {
        MXC_UART_Write(PC_COM_PORT, (uint8_t *)buffer, &len);
    }
    va_end(args);

    return len;
}

void terminal_hexdump(const char *title, char *buf, unsigned int len)
{
    unsigned int i;

    if (title) {
        terminal_printf("%s", title);
    }

    /* Print buffer bytes */
    for (i = 0; i < len; i++) {
        if (!(i % 16)) {
            terminal_printf("\r\n");
        }
        terminal_printf("%02X ", buf[i]);
    }

    terminal_printf("\r\n");
}
