/**
 * @file    mxc_pins.h
 * @brief   This file contains constant pin configurations for the peripherals.
 */

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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32572_MXC_PINS_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32572_MXC_PINS_H_
 typedef enum { MAP_A, MAP_B } sys_map_t;
#include "gpio.h"

/***** Global Variables *****/
// Predefined GPIO Configurations
extern const mxc_gpio_cfg_t gpio_cfg_extclk;
extern const mxc_gpio_cfg_t gpio_cfg_i2c0;
extern const mxc_gpio_cfg_t gpio_cfg_i2c1;

extern const mxc_gpio_cfg_t gpio_cfg_uart0;
extern const mxc_gpio_cfg_t gpio_cfg_uart0_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart0_flow_disable;
extern const mxc_gpio_cfg_t gpio_cfg_uart1;
extern const mxc_gpio_cfg_t gpio_cfg_uart1_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart1_flow_disable;
extern const mxc_gpio_cfg_t gpio_cfg_uart2;
extern const mxc_gpio_cfg_t gpio_cfg_uart2_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart2_flow_disable;
extern const mxc_gpio_cfg_t gpio_cfg_uart3;
extern const mxc_gpio_cfg_t gpio_cfg_uart3_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart3_flow_disable;

// Timers are only defined once, depending on package, each timer could be mapped to other pins
extern const mxc_gpio_cfg_t gpio_cfg_tmr0;
extern const mxc_gpio_cfg_t gpio_cfg_tmr1;
extern const mxc_gpio_cfg_t gpio_cfg_tmr2;
extern const mxc_gpio_cfg_t gpio_cfg_tmr3;
extern const mxc_gpio_cfg_t gpio_cfg_tmr4;
extern const mxc_gpio_cfg_t gpio_cfg_tmr5;

extern const mxc_gpio_cfg_t gpio_cfg_rtcsqw;
extern const mxc_gpio_cfg_t gpio_cfg_rtcsqwb;

extern const mxc_gpio_cfg_t gpio_cfg_pt0;
extern const mxc_gpio_cfg_t gpio_cfg_pt1;
extern const mxc_gpio_cfg_t gpio_cfg_pt2;
extern const mxc_gpio_cfg_t gpio_cfg_pt3;
extern const mxc_gpio_cfg_t gpio_cfg_pt4;
extern const mxc_gpio_cfg_t gpio_cfg_pt5;
extern const mxc_gpio_cfg_t gpio_cfg_pt6;
extern const mxc_gpio_cfg_t gpio_cfg_pt7;

extern const mxc_gpio_cfg_t gpio_cfg_rv_jtag;

extern const mxc_gpio_cfg_t gpio_cfg_spixf;

// SPI v2 Pin Definitions
extern const mxc_gpio_cfg_t gpio_cfg_spi0_standard;
extern const mxc_gpio_cfg_t gpio_cfg_spi0_3wire;
extern const mxc_gpio_cfg_t gpio_cfg_spi0_dual;
extern const mxc_gpio_cfg_t gpio_cfg_spi0_quad;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_standard;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_3wire;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_dual;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_quad;
// SPI2 does not exist in the MAX32572 (to match instance addressing with MAX32570B)
extern const mxc_gpio_cfg_t gpio_cfg_spi3_standard;
extern const mxc_gpio_cfg_t gpio_cfg_spi3_3wire;
extern const mxc_gpio_cfg_t gpio_cfg_spi3_dual;
extern const mxc_gpio_cfg_t gpio_cfg_spi3_quad;

// SPI v2 Target Selects Pin Definitions
extern const mxc_gpio_cfg_t gpio_cfg_spi0_ts0;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_ts0;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_ts1;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_ts2;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_ts3;
// SPI2 does not exist in the MAX32572 (to match instance addressing with MAX32570B)
extern const mxc_gpio_cfg_t gpio_cfg_spi3_ts0;
extern const mxc_gpio_cfg_t gpio_cfg_spi3_ts1;
extern const mxc_gpio_cfg_t gpio_cfg_spi3_ts2;
extern const mxc_gpio_cfg_t gpio_cfg_spi3_ts3;

extern const mxc_gpio_cfg_t gpio_cfg_adc_ain0;
extern const mxc_gpio_cfg_t gpio_cfg_adc_ain1;
extern const mxc_gpio_cfg_t gpio_cfg_adc_ain2;
extern const mxc_gpio_cfg_t gpio_cfg_adc_ain3;
extern const mxc_gpio_cfg_t gpio_cfg_adc_ain4;
extern const mxc_gpio_cfg_t gpio_cfg_adc_ain5;
extern const mxc_gpio_cfg_t gpio_cfg_adc_ain6;
extern const mxc_gpio_cfg_t gpio_cfg_adc_ain7;

extern const mxc_gpio_cfg_t gpio_cfg_kbd_P2;
#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32572_MXC_PINS_H_
