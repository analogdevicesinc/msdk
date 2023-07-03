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

/*******************************      INCLUDES    ****************************/
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "terminal.h"
#include "platform.h"

#include "bootloader.h"

/*******************************      DEFINES     ****************************/

/******************************* Type Definitions ****************************/

/*******************************    Variables   ****************************/

/******************************* Static Functions ****************************/
static int hard_reset_then_enter_bl_mode(const char *parentName)
{
    return bl_hard_reset_then_enter_bl_mode();
}

static int get_target_partnumber(const char *parentName)
{
    int ret;
    char partNumber[16] = { 0 };

    ret = bl_get_partnumber(partNumber, sizeof(partNumber) - 1);
    if (ret == 0) {
        terminal_printf("PartNumber: %s\r\n", partNumber);
    }

    return ret;
}

static int get_target_bl_version(const char *parentName)
{
    int ret;
    char version[16] = { 0 };

    ret = bl_get_version(version, sizeof(version) - 1);
    if (ret == 0) {
        terminal_printf("Version: %s\r\n", version);
    }

    return ret;
}

static int enter_bl_mode(const char *parentName)
{
    return bl_enter_bl_mode();
}

static int exit_bl_mode(const char *parentName)
{
    return bl_exit_bl_mode();
}

static int get_page_size(const char *parentName)
{
    int ret;
    unsigned int page_size = 0;

    ret = bl_get_page_size(&page_size);
    if (ret == 0) {
        terminal_printf("Page Size: %d\r\n", page_size);
    }

    return ret;
}

static int get_usn(const char *parentName)
{
    int ret;
    char usn[24];

    ret = bl_get_usn(usn, sizeof(usn));
    if (ret == 0) {
        terminal_hexdump("USN:", usn, 24);
    }

    return ret;
}

static int erase_app(const char *parentName)
{
    return bl_erase_app();
}

static int set_timeout(const char *parentName)
{
    int ret = -1;
    uint16_t timeout = 0;

    terminal_printf("\r\nNew Timeout (ms): ");
    timeout = terminal_read_num(0);
    if (timeout >= 0) {
        ret = b_update_cfg_set_bl_exit_timeout(timeout);
    }

    return ret;
}

static int dump_configurations(const char *parentName)
{
    int ret;
    char version[16] = { 0 };

    ret = bl_get_version(version, sizeof(version) - 1);
    if (ret == 0) {
        terminal_printf("\r\nVersion: %s\r\n", version);
    } else {
        terminal_printf("\r\nbl_get_version: FAILURE: %d\r\n", ret);
    }

    if (ret == 0) {
        if (strcmp(version, "v3.4.1") <= 0) {
            boot_config_t_before_v342 cfg_old;

            ret = bl_get_target_configure(version, &cfg_old);

            terminal_printf("%-18s: %d\r\n", "enter_bl_check", cfg_old.enter_bl_check);
            terminal_printf("%-18s: %d\r\n", "ebl_pin", cfg_old.ebl_pin);
            terminal_printf("%-18s: %d\r\n", "ebl_polarity", cfg_old.ebl_polarity);
            terminal_printf("%-18s: %d\r\n", "uart_enable", cfg_old.uart_enable);
            terminal_printf("%-18s: %d\r\n", "i2c_enable", cfg_old.i2c_enable);
            terminal_printf("%-18s: %d\r\n", "spi_enable", cfg_old.spi_enable);
            terminal_printf("%-18s: %d ms\r\n", "ebl_timeout", 1 << cfg_old.ebl_timeout);
            terminal_printf("%-18s: %d\r\n", "exit_bl_mode", cfg_old.exit_bl_mode);
            terminal_printf("%-18s: %d\r\n", "crc_check", cfg_old.crc_check);
            terminal_printf("%-18s: %d\r\n", "valid_mark_check", cfg_old.valid_mark_check);
            terminal_printf("%-18s: %d\r\n", "lock_swd", cfg_old.lock_swd);

            switch (cfg_old.i2c_addr) {
            case 0:
                terminal_printf("%-18s: 0x58\r\n", "i2c_addr");
                break;
            case 1:
                terminal_printf("%-18s: 0x5A\r\n", "i2c_addr");
                break;
            case 2:
                terminal_printf("%-18s: 0x5C\r\n", "i2c_addr");
                break;
            case 3:
                terminal_printf("%-18s: 0xAA\r\n", "i2c_addr");
                break;
            }
        } else {
            boot_config_t cfg;

            ret = bl_get_target_configure(version, &cfg);

            terminal_printf("%-18s: %d\r\n", "enter_bl_check", cfg.enter_bl_check);
            terminal_printf("%-18s: %d\r\n", "ebl_pin", cfg.ebl_pin);
            terminal_printf("%-18s: %d\r\n", "ebl_polarity", cfg.ebl_polarity);
            terminal_printf("%-18s: %d\r\n", "uart_enable", cfg.uart_enable);
            terminal_printf("%-18s: %d\r\n", "i2c_enable", cfg.i2c_enable);
            terminal_printf("%-18s: %d\r\n", "spi_enable", cfg.spi_enable);
            terminal_printf("%-18s: %d ms\r\n", "ebl_timeout", 1 << cfg.ebl_timeout);
            terminal_printf("%-18s: %d\r\n", "exit_bl_mode", cfg.exit_bl_mode);
            terminal_printf("%-18s: %d\r\n", "crc_check", cfg.crc_check);
            terminal_printf("%-18s: %d\r\n", "valid_mark_check", cfg.valid_mark_check);
            terminal_printf("%-18s: %d\r\n", "lock_swd", cfg.lock_swd);
            terminal_printf("%-18s: 0x%02X\r\n", "i2c_addr", cfg.i2c_addr);
        }
        terminal_printf("\r\n");
    }

    return ret;
}

static int flash_configurations(const char *parentName)
{
    return bl_flash_bl_cfg();
}

/******************************* Public Functions ****************************/
static list_t list[] = {
    { "Hard Reset than Send Enter Bootloader Command", hard_reset_then_enter_bl_mode },
    { "Get Target PartNumber", get_target_partnumber },
    { "Get Target Bootloader Version", get_target_bl_version },
    { "Enter Bootloader", enter_bl_mode },
    { "Exit Bootloader", exit_bl_mode },
    { "Get Page Size", get_page_size },
    { "Get USN", get_usn },
    { "Erase Application", erase_app },
    { "Set Bootloader Timeout", set_timeout },
    { "Dump Bootloader Configurations", dump_configurations },
    { "Flash Bootloader Configurations", flash_configurations },
};

int bootloader_menu(const char *parentName)
{
    int ret = 0;

    while (1) {
        ret = terminal_select_from_list(parentName, list, sizeof(list) / sizeof(list[0]), 1);
        if (ret == KEY_CANCEL) {
            break;
        }
    }

    return 0;
}
