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

/**
 * @file    main.c
 * @brief   Bootloader Host Example
 *
 * @details This example provide a menu on terminal, depend on the selection it run tests.
 */

/*******************************      INCLUDES    ****************************/
#include <stdio.h>
#include <stdint.h>

#include "terminal.h"
#include "bootloader.h"
#include "platform.h"
#include "board.h"

#include "test_images.h"

/*******************************      DEFINES     ****************************/

/******************************* Type Definitions ****************************/

/******************************* 	Variables 	  ****************************/
extern int bootloader_menu(const char* parentName);

/******************************* Static Functions ****************************/
static int select_interface(const char* parentName)
{
    int ret;

    list_t list[] = {
        {"I2C", NULL},
        {"SPI", NULL},
    };

    while (1) {
        ret = terminal_select_from_list(parentName, list, sizeof(list) / sizeof(list[0]), 1);
        if (ret == KEY_CANCEL) {
            break;
        }

        if (ret == 1) {
            bl_update_interface(plt_i2c_read, plt_i2c_write);
            terminal_printf("Host-<->Target interface configured as I2C\r\n");
            ret = 0; // means success
            break;
        } else if (ret == 2) {
            bl_update_interface(plt_spi_read, plt_spi_write);
            terminal_printf("Host-<->Target interface configured as SPI\r\n");
            ret = 0; // means success
            break;
        }
    }

    return ret;
}

static int update_fw_nonsecure_MAX32660_blinkled_slow_P0_13(const char* parentName)
{
    return bl_flash_image(nonsecure_MAX32660_blinkled_slow_P0_13);
}

static int update_fw_nonsecure_MAX32660_blinkled_fast_P0_13(const char* parentName)
{
    return bl_flash_image(nonsecure_MAX32660_blinkled_fast_P0_13);
}

static int update_fw_devkey_MAX32660_blinkled_fast_P0_13(const char* parentName)
{
    return bl_flash_image(devkey_MAX32660_blinkled_fast_P0_13);
}

static int update_fw_devkey_MAX32660_blinkled_slow_P0_13(const char* parentName)
{
    return bl_flash_image(devkey_MAX32660_blinkled_slow_P0_13);
}

static int update_fw_devkey_MAX32670_blinkled_P0_22(const char* parentName)
{
    return bl_flash_image(devkey_MAX32670_blinkled_P0_22);
}

static int update_fw_devkey_MAX32670_blinkled_P0_23(const char* parentName)
{
    return bl_flash_image(devkey_MAX32670_blinkled_P0_23);
}

/******************************* Public Functions ****************************/
static list_t list[] = {
    {"Select Interface", select_interface},
    {"Bootloader Test Menu", bootloader_menu},
    {"Load MSBL: Non-secure MAX32660 blinkled fast P0.13",
     update_fw_nonsecure_MAX32660_blinkled_fast_P0_13},
    {"Load MSBL: Non-secure MAX32660 blinkled slow P0.13",
     update_fw_nonsecure_MAX32660_blinkled_slow_P0_13},
    {"Load MSBL: MAX32660 blinkled fast P0.13 (development key)",
     update_fw_devkey_MAX32660_blinkled_fast_P0_13},
    {"Load MSBL: MAX32660 blinkled slow P0.13 (development key)",
     update_fw_devkey_MAX32660_blinkled_slow_P0_13},
    {"Load MSBL: MAX32670 blinkled P0.22 (development key)",
     update_fw_devkey_MAX32670_blinkled_P0_22},
    {"Load MSBL: MAX32670 blinkled P0.23 (development key)",
     update_fw_devkey_MAX32670_blinkled_P0_23},
};

int main(void)
{
    plt_i2c_init();
    plt_spi_init();
    plt_gpio_init();

    // Initialize terminal uart after SPI initialization because some pins (P0.20, P0.21) are overlaps
    terminal_init();
    terminal_printf("\r\n***********Bootloader Host Example***********\r\n");
    terminal_printf("The purpose of this example is:\r\n");
    terminal_printf("   1- Demonstrate how bootloader device can be program\r\n");
    terminal_printf("   2- Provide platform independent bootloader protocol (files under "
                    "bootloader folder)\r\n");
    terminal_printf("\r\nThis example can be ported on any platform\r\n");
    terminal_printf("If you would like to port it on other platform\r\n");
    terminal_printf("you need to update terminal.c and platform_max32665.c files\r\n");
    terminal_printf("\r\n");
    terminal_printf("\tHW Pins\r\n");
    terminal_printf("\tI2C:    SCL(P0.6),    SDA(P0.7)  (Note: I2C requires pullup resistor) \r\n");
    terminal_printf("\tSPI:    MISO(P0.17),  MOSI(P0.18),  SCK(P0.19),  SS(P0.16)\r\n");
#if defined(BOARD_FTHR) || defined(BOARD_FTHR2)
    terminal_printf("\tTarget: RESET(P0.20), MFIO(P0.21)\r\n");
#else
    terminal_printf("\tTarget: RESET(P0.14), MFIO(P0.15)\r\n");
#endif
    terminal_printf("\r\n");
#if defined(BOARD_FTHR2)
    terminal_printf("UART0 is used as terminal comport\r\n");
#else
    terminal_printf("UART1 is used as terminal comport\r\n");
#endif

    bl_conf_struct_t plt;

    plt.read     = plt_i2c_read;
    plt.write    = plt_i2c_write;
    plt.gpio_set = plt_gpio_set;
    plt.delay_ms = plt_delay_ms;
    plt.printf   = terminal_printf;

    bl_init(&plt);

    while (1) {
        terminal_select_from_list("Main Menu", list, sizeof(list) / sizeof(list[0]), 1);
    }
}
