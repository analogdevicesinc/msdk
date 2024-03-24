/**
 * @file    mxc_pins.h
 * @brief   This file contains constant pin configurations for the peripherals.
 */

/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32662_MXC_PINS_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32662_MXC_PINS_H_

#include "gpio.h"

typedef enum { MAP_A, MAP_B, MAP_C } sys_map_t;

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

// SPI v2 Pin Definitions
extern const mxc_gpio_cfg_t gpio_cfg_spi0_standard;
extern const mxc_gpio_cfg_t gpio_cfg_spi0_3wire;
// MXC_SPI0 does not support Dual or Quad mode.
extern const mxc_gpio_cfg_t gpio_cfg_spi1a_standard;
extern const mxc_gpio_cfg_t gpio_cfg_spi1a_3wire;
extern const mxc_gpio_cfg_t gpio_cfg_spi1b_standard;
extern const mxc_gpio_cfg_t gpio_cfg_spi1b_3wire;
// MXC_SPI1 does not support Dual or Quad mode.

// SPI v2 Target Selects Pin Definitions
extern const mxc_gpio_cfg_t gpio_cfg_spi0_ts0;
extern const mxc_gpio_cfg_t gpio_cfg_spi1a_ts0;
extern const mxc_gpio_cfg_t gpio_cfg_spi1b_ts0;

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32662_MXC_PINS_H_
