/**
 * @file mxc_pins.h
 * @brief      This file contains constant pin configurations for the peripherals.
 */

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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_MXC_PINS_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_MXC_PINS_H_

#include "gpio.h"

/***** Global Variables *****/

typedef enum { MAP_A, MAP_B, MAP_C } sys_map_t;

// Predefined GPIO Configurations
extern const mxc_gpio_cfg_t gpio_cfg_tmr0;
extern const mxc_gpio_cfg_t gpio_cfg_tmr1;
extern const mxc_gpio_cfg_t gpio_cfg_tmr2;
extern const mxc_gpio_cfg_t gpio_cfg_tmr3;
extern const mxc_gpio_cfg_t gpio_cfg_tmr4;
extern const mxc_gpio_cfg_t gpio_cfg_tmr5;

extern const mxc_gpio_cfg_t gpio_cfg_uart0a;
extern const mxc_gpio_cfg_t gpio_cfg_uart0a_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart1a;
extern const mxc_gpio_cfg_t gpio_cfg_uart1a_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart2a;
extern const mxc_gpio_cfg_t gpio_cfg_uart2a_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart0b;
extern const mxc_gpio_cfg_t gpio_cfg_uart0b_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart1b;
extern const mxc_gpio_cfg_t gpio_cfg_uart1b_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart2b;
extern const mxc_gpio_cfg_t gpio_cfg_uart2b_flow;

extern const mxc_gpio_cfg_t gpio_cfg_i2c0;
extern const mxc_gpio_cfg_t gpio_cfg_i2c1;
extern const mxc_gpio_cfg_t gpio_cfg_i2c2;

extern const mxc_gpio_cfg_t gpio_cfg_spi0a;
extern const mxc_gpio_cfg_t gpio_cfg_spi0b;
extern const mxc_gpio_cfg_t gpio_cfg_spi0_ss0a;
extern const mxc_gpio_cfg_t gpio_cfg_spi0_ss0b;
extern const mxc_gpio_cfg_t gpio_cfg_spi0_ss1;
extern const mxc_gpio_cfg_t gpio_cfg_spi0_ss2;
extern const mxc_gpio_cfg_t gpio_cfg_spi1;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_ss0;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_ss1;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_ss2;
extern const mxc_gpio_cfg_t gpio_cfg_spi2;
extern const mxc_gpio_cfg_t gpio_cfg_spi2_ss0;
extern const mxc_gpio_cfg_t gpio_cfg_spi2_ss1;
extern const mxc_gpio_cfg_t gpio_cfg_spi2_ss2;

extern const mxc_gpio_cfg_t gpio_cfg_pt0;
extern const mxc_gpio_cfg_t gpio_cfg_pt1;
extern const mxc_gpio_cfg_t gpio_cfg_pt2;
extern const mxc_gpio_cfg_t gpio_cfg_pt3;
extern const mxc_gpio_cfg_t gpio_cfg_pt4;
extern const mxc_gpio_cfg_t gpio_cfg_pt5;
extern const mxc_gpio_cfg_t gpio_cfg_pt6;
extern const mxc_gpio_cfg_t gpio_cfg_pt7;
extern const mxc_gpio_cfg_t gpio_cfg_pt8;
extern const mxc_gpio_cfg_t gpio_cfg_pt9;
extern const mxc_gpio_cfg_t gpio_cfg_pt10;
extern const mxc_gpio_cfg_t gpio_cfg_pt11;
extern const mxc_gpio_cfg_t gpio_cfg_pt12;
extern const mxc_gpio_cfg_t gpio_cfg_pt13;
extern const mxc_gpio_cfg_t gpio_cfg_pt14;
extern const mxc_gpio_cfg_t gpio_cfg_pt15;

extern const mxc_gpio_cfg_t gpio_cfg_rtcsqw;
extern const mxc_gpio_cfg_t gpio_cfg_spixfc;
extern const mxc_gpio_cfg_t gpio_cfg_spim0;
extern const mxc_gpio_cfg_t gpio_cfg_spim1;
extern const mxc_gpio_cfg_t gpio_cfg_spim2;
extern const mxc_gpio_cfg_t gpio_cfg_spis;
extern const mxc_gpio_cfg_t gpio_cfg_spixr;

extern const mxc_gpio_cfg_t gpio_cfg_sdhc;
extern const mxc_gpio_cfg_t gpio_cfg_owma;
extern const mxc_gpio_cfg_t gpio_cfg_owmb;
extern const mxc_gpio_cfg_t gpio_cfg_owmc;

extern const mxc_gpio_cfg_t gpio_cfg_adc0;
extern const mxc_gpio_cfg_t gpio_cfg_adc1;
extern const mxc_gpio_cfg_t gpio_cfg_adc2;
extern const mxc_gpio_cfg_t gpio_cfg_adc3;
extern const mxc_gpio_cfg_t gpio_cfg_adc4;
extern const mxc_gpio_cfg_t gpio_cfg_adc5;
extern const mxc_gpio_cfg_t gpio_cfg_adc6;
extern const mxc_gpio_cfg_t gpio_cfg_adc7;

extern const mxc_gpio_cfg_t gpio_cfg_sdma;

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_MXC_PINS_H_
