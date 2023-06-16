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
#ifndef EXAMPLES_MAX78000_QSPI_N01S830HA_H_
#define EXAMPLES_MAX78000_QSPI_N01S830HA_H_

#include <stdint.h>
#include "gpio.h"
#include "max78000.h"

#define CMD_READ 0x03
#define CMD_WRITE 0x02
#define CMD_ENABLE_QUAD_IO 0x38
#define CMD_ENABLE_DUAL_IO 0x3B
#define CMD_RESET_IO 0xFF
#define CMD_READ_MODE_REG 0x05
#define CMD_WRITE_MODE_REG 0x01

// P0.9 on MAX78000FTHR is routed to HOLD pin
#define HOLD_PIN_PORT MXC_GPIO0
#define HOLD_PIN_MASK MXC_GPIO_PIN_9

static const mxc_gpio_cfg_t hold_pin = { .port = HOLD_PIN_PORT,
                                           .mask = HOLD_PIN_MASK,
                                           .func = MXC_GPIO_FUNC_OUT,
                                           .pad = MXC_GPIO_PAD_WEAK_PULL_UP,
                                           .vssel = MXC_GPIO_VSSEL_VDDIOH };

int ram_init();

int ram_enter_quadmode();

int ram_exit_quadmode();

int ram_read(uint32_t address, uint8_t *out, unsigned int len);

int ram_write(uint32_t address, uint8_t *data, unsigned int len);

int ram_write_mode_reg(uint8_t val);

int ram_read_mode_reg(uint8_t* out); 

#endif // EXAMPLES_MAX78000_QSPI_N01S830HA_H_
