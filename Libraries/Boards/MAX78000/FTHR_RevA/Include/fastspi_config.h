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
#ifndef LIBRARIES_BOARDS_MAX78000_FTHR_REVA_INCLUDE_FASTSPI_CONFIG_H_
#define LIBRARIES_BOARDS_MAX78000_FTHR_REVA_INCLUDE_FASTSPI_CONFIG_H_

/**
 * @file    fastspi_config.c
 * @brief   "fastspi" configuration file for MAX78000FTHR board
 */

#include "spi.h"
#include "gpio.h"

// (*) Required definitions:
#define SPI MXC_SPI0
#define SPI_SPEED 25000000

// Optional definitions to make GPIO creation easier:
#define SPI_PINS_PORT MXC_GPIO0
#define SPI_VSSEL MXC_GPIO_VSSEL_VDDIOH
#define SPI_PINS_MASK \
    (MXC_GPIO_PIN_5 | MXC_GPIO_PIN_6 | MXC_GPIO_PIN_7 | MXC_GPIO_PIN_8 | MXC_GPIO_PIN_9)
#define SPI_SS_PORT MXC_GPIO0
#define SPI_SS_PIN MXC_GPIO_PIN_10 // (SS2)

// (*) Required GPIO definitions:
static const mxc_gpio_cfg_t fastspi_ss_pin = { .port = SPI_SS_PORT,
                                               .mask = SPI_SS_PIN,
                                               .func = MXC_GPIO_FUNC_ALT2, // ALT2 for SS2
                                               .pad = MXC_GPIO_PAD_WEAK_PULL_UP,
                                               .vssel = SPI_VSSEL };

static const mxc_gpio_cfg_t fastspi_spi_pins = { .port = SPI_PINS_PORT,
                                                 .mask = SPI_PINS_MASK,
                                                 .func = MXC_GPIO_FUNC_ALT1,
                                                 .pad = MXC_GPIO_PAD_NONE,
                                                 .vssel = SPI_VSSEL };

#endif // LIBRARIES_BOARDS_MAX78000_FTHR_REVA_INCLUDE_FASTSPI_CONFIG_H_
