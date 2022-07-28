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


#ifndef _MXC_PINS_H_
#define _MXC_PINS_H_

#include "gpio.h"

typedef enum {
    MAP_A,
    MAP_B,
    MAP_C
} sys_map_t;

/***** Global Variables *****/
// Predefined GPIO Configurations
extern const mxc_gpio_cfg_t gpio_cfg_lp_extclk;
extern const mxc_gpio_cfg_t gpio_cfg_hf_extclk;

extern const mxc_gpio_cfg_t gpio_cfg_i2c0;
extern const mxc_gpio_cfg_t gpio_cfg_i2c1;

extern const mxc_gpio_cfg_t gpio_cfg_uart0;
extern const mxc_gpio_cfg_t gpio_cfg_uart0_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart0_flow_disable;
extern const mxc_gpio_cfg_t gpio_cfg_uart0b;
extern const mxc_gpio_cfg_t gpio_cfg_uart0b_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart0b_flow_disable;
extern const mxc_gpio_cfg_t gpio_cfg_uart1;
extern const mxc_gpio_cfg_t gpio_cfg_uart1_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart1_flow_disable;

// Timers are only defined once, depending on package, each timer could be mapped to other pins
extern const mxc_gpio_cfg_t gpio_cfg_tmr0;
extern const mxc_gpio_cfg_t gpio_cfg_tmr0b;
extern const mxc_gpio_cfg_t gpio_cfg_tmr0c;
extern const mxc_gpio_cfg_t gpio_cfg_tmr1;
extern const mxc_gpio_cfg_t gpio_cfg_tmr1b;
extern const mxc_gpio_cfg_t gpio_cfg_tmr1c;
extern const mxc_gpio_cfg_t gpio_cfg_tmr2;
extern const mxc_gpio_cfg_t gpio_cfg_tmr2b;
extern const mxc_gpio_cfg_t gpio_cfg_tmr2c;
extern const mxc_gpio_cfg_t gpio_cfg_tmr3;

extern const mxc_gpio_cfg_t gpio_cfg_i2s;

extern const mxc_gpio_cfg_t gpio_cfg_spi0;
extern const mxc_gpio_cfg_t gpio_cfg_spi0_ss0;

extern const mxc_gpio_cfg_t gpio_cfg_spi1;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_ss0;
extern const mxc_gpio_cfg_t gpio_cfg_spi1b;
extern const mxc_gpio_cfg_t gpio_cfg_spi1b_ss0;

extern const mxc_gpio_cfg_t gpio_cfg_pt0;
extern const mxc_gpio_cfg_t gpio_cfg_pt1;
extern const mxc_gpio_cfg_t gpio_cfg_pt2;
extern const mxc_gpio_cfg_t gpio_cfg_pt3;

extern const mxc_gpio_cfg_t gpio_cfg_adc_ain0;
extern const mxc_gpio_cfg_t gpio_cfg_adc_ain1;
extern const mxc_gpio_cfg_t gpio_cfg_adc_ain2;
extern const mxc_gpio_cfg_t gpio_cfg_adc_ain3;

extern const mxc_gpio_cfg_t gpio_cfg_adc_trig_p0_9;
extern const mxc_gpio_cfg_t gpio_cfg_adc_trig_p0_0;

extern const mxc_gpio_cfg_t gpio_cfg_can;
extern const mxc_gpio_cfg_t gpio_cfg_canb;

#endif /* _MXC_PINS_H_ */
