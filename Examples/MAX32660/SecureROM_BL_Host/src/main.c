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
 * @file    main.c
 * @brief   Secure ROM Bootloader Host Example
 *
 * @details This example demonstrate how to load image on secure microcontroller
 * by using host microcontroller.
 */

/*******************************      INCLUDES    ****************************/
#include <stdio.h>
#include <stdint.h>

#include "terminal.h"
#include "bootloader.h"
#include "platform.h"

#include "test_images.h"

/*******************************      DEFINES     ****************************/

/******************************* Type Definitions ****************************/

/*******************************     Variables    ****************************/

/******************************* Static Functions ****************************/
static int max32520_load_image1(const char *parentName)
{
    return sbl_load(scp_MAX32520KIT_blinkled_P1_6);
}

static int max32520_load_image2(const char *parentName)
{
    return sbl_load(scp_MAX32520KIT_blinkled_P1_7);
}

static int max32651_load_image1(const char *parentName)
{
    return sbl_load(scp_MAX32651EVKIT_blinkled_P2_25);
}

/******************************* Public Functions ****************************/
static list_t list[] = {
    { "MAX32520-EvKit load blinkled-1 (P1.6)", max32520_load_image1 },
    { "MAX32520-EvKit load blinkled-2 (P1.7)", max32520_load_image2 },
    { "MAX32651-EvKit load blinkled-1 (P2.25)", max32651_load_image1 },
};

int main(void)
{
    plt_uart_init();
    plt_gpio_init();

    terminal_init();

    terminal_printf("\r\n***********Secure ROM Bootloader Host Example***********\r\n");
    terminal_printf("This example demonstrate how image can be load on secure microcontroller\r\n");
    terminal_printf("by using second microcontroller as host\r\n");
    terminal_printf(
        "In this example host micro will be MAX32660 and target will be MAX32520/MAX32651\r\n");
    terminal_printf("Blinkled images will be load to the target device by MAX32660 host\r\n");

    terminal_printf("\r\nHW Connection:\r\n");
    terminal_printf("MAX32660 UART1 will be used as debug port, connect it to the PC\r\n");
    terminal_printf("    On PC side open a serial port monitor application, 115200 8-N-1 \r\n");
    terminal_printf(
        "MAX32660 UART0 will be used to communicate with target board, apply below connection\r\n");
    terminal_printf("    Connect MAX32660 P0.5 (RX) to the target UART TX\r\n");
    terminal_printf("    Connect MAX32660 P0.4 (TX) to the target UART RX\r\n");
    terminal_printf("    Connect MAX32660 P0.6  to the target RSTN\r\n"); // Used to restart target
    // stimulus pin is used to enable SCP communication, this pin can be enable/disable over secure ROM bootloader.
    // on default it is disable for MAX32520 secure ROM bootloader. That means SCP communication enable on each restart.
    terminal_printf(
        "    (If stimulus pin requires) Connect MAX32660 P0.7  to the target Stimulus PIN\r\n");
    terminal_printf("    Connect GND line between MAX32660 - target boards\r\n");

    terminal_printf("\r\nNote:\r\n");
    terminal_printf("1-This example can be ported on any platform\r\n");
    terminal_printf("    If you would like to port it on other platform\r\n");
    terminal_printf("    you need to update terminal.c and platform_max32660.c files\r\n");
    terminal_printf(
        "2- To convert SCP images to C array, check ./scripts/scp_to_c_array.py script\r\r");
    terminal_printf("\r\n");

    bl_conf_struct_t plt;

    plt.read = plt_uart_read;
    plt.write = plt_uart_write;
    plt.gpio_set = plt_gpio_set;
    plt.delay_ms = plt_delay_ms;
    plt.printf = terminal_printf;

    sbl_init(&plt);

    while (1) {
        terminal_select_from_list("Main Menu", list, sizeof(list) / sizeof(list[0]), 1);
    }
}
