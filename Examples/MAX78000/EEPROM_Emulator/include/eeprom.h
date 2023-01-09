/*******************************************************************************
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
*******************************************************************************
*/

#ifndef EXAMPLES_MAX78000_EEPROM_EMULATOR_EEPROM_H_
#define EXAMPLES_MAX78000_EEPROM_EMULATOR_EEPROM_H_

/***** Included Files *****/
#include <stdbool.h>
#include "board.h"
#include "gpio.h"
#include "i2c.h"
#include "mxc_device.h"

/***** Definitions *****/
#ifdef BOARD_EVKIT_V1
#define EEPROM_I2C MXC_I2C2
#else
#define EEPROM_I2C MXC_I2C1
#endif

#define EEPROM_ADDR 0x24
#define EEPROM_I2C_IRQN MXC_I2C_GET_IRQ(MXC_I2C_GET_IDX(EEPROM_I2C))
#define EEPROM_I2C_FREQ 100000
#define EEPROM_FIFO_DEPTH MXC_I2C_FIFO_DEPTH

#define EEPROM_NUM_FLASH_PG 4
#define EEPROM_FLASH_SZ (EEPROM_NUM_FLASH_PG * MXC_FLASH_PAGE_SIZE)
#define EEPROM_BASE_ADDR ((MXC_FLASH_MEM_BASE + MXC_FLASH_MEM_SIZE) -\
						  EEPROM_FLASH_SZ)
#define EEPROM_RAW_ADDR(read_addr) (EEPROM_BASE_ADDR + read_addr)

#define EEPROM_MAX_DATA_RX 64
#define EEPROM_ADDR_SIZE sizeof(uint16_t)
#define EEPROM_RX_BUF_SIZE (EEPROM_MAX_DATA_RX + EEPROM_ADDR_SIZE)

/***** Global Variables *****/
extern volatile bool eeprom_txn_done;

/***** Functions *****/
int eeprom_init(mxc_gpio_cfg_t rdy_pin);

void eeprom_prep_for_txn(void);


#endif // EXAMPLES_MAX78000_EEPROM_EMULATOR_EEPROM_H_
