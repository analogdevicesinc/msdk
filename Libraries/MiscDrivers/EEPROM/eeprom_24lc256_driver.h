/**
 * @file    eeprom_24lc256_driver.h
 * @brief   24LC256 EEPROM driver header
 * @details Defines 24LC256 EEPROM
 *          Implements helper macros
 **/

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

#ifndef LIBRARIES_MISCDRIVERS_EEPROM_EEPROM_24LC256_DRIVER_H_
#define LIBRARIES_MISCDRIVERS_EEPROM_EEPROM_24LC256_DRIVER_H_

#include <stdint.h>
#include <stdio.h>
#include <float.h>
#include "mxc_delay.h"
#include "i2c.h"

/**@def _24LC256_EEPROM_PAGE_SIZE
 * @brief Page size of  24LC256 EEPROM
 **/
#define _24LC256_EEPROM_PAGE_SIZE 64

/**@def _24LC256_EEPROM_SIZE
 * @brief Total size of  24LC256 EEPROM = 256 Kbit
 **/
#define _24LC256_EEPROM_SIZE 0x8000 // 32 KByte = 256 Kbit

/**@def I2C
 * @brief I2C Max read size for single transaction
 **/
#define I2C_MAX_READ_SIZE 256

/**
 * @brief Structure with sensor function pointers
 */
typedef struct {
    int (*init)(mxc_i2c_regs_t *i2c, uint8_t addr); ///< Pointer to
    int (*read)(uint16_t addr, uint8_t *data_buffer, uint16_t length);
    int (*write_chunk)(uint16_t addr, uint8_t *data_buffer, uint16_t length);
    int (*write)(uint16_t addr, uint8_t *data_buffer, uint32_t length);
} eeprom_24LC256_driver_t;

/**
 * @brief Prepare EEPROM_24LC256_DRIVER function pointers
 *
 * @return eeprom_24LC256_driver_t instance
 */
eeprom_24LC256_driver_t eeprom_24LC256_Open();

#endif  // LIBRARIES_MISCDRIVERS_EEPROM_EEPROM_24LC256_DRIVER_H_
