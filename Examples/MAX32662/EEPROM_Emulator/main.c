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
 * @brief   EEPROM_Emulator Example
 * @details This example can be used to emulate the behavior of
 *          an EEPROM chip with an I2C interface. See README for
 *          details on how to perform read and write operations
 *          with the device.
 */

/***** Includes *****/
#include <stdbool.h>
#include <stdio.h>
#include "gpio.h"
#include "i2c.h"
#include "include/eeprom.h"
#include "mxc_errors.h"

/***** Definitions *****/
#define EEPROM_I2C MXC_I2C1
#define SYNC_PIN_PORT MXC_GPIO0
#define SYNC_PIN_MASK MXC_GPIO_PIN_7

/***** Functions *****/
int main(void)
{
    int err;
    printf("\n********************  EEPROM Emulator Demo *******************\n");

    mxc_gpio_cfg_t sync_pin;
    sync_pin.port = SYNC_PIN_PORT;
    sync_pin.mask = SYNC_PIN_MASK;

    // Initialize EEPROM Emulator
    if ((err = eeprom_init(EEPROM_I2C, sync_pin)) != E_NO_ERROR) {
        printf("Failed to initialize EEPROM Emulator!\n");
        return err;
    }

    while (1) {
        // Start next slave transaction
        eeprom_prep_for_txn();

        // Wait for slave transaction to finish
        while (!eeprom_txn_done) {}
    }
}
