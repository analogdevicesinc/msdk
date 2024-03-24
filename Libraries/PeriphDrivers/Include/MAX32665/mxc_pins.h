/**
 * @file mxc_pins.h
 * @brief      This file contains constant pin configurations for the peripherals.
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

// SPI v2 Pin Definitions
extern const mxc_gpio_cfg_t gpio_cfg_spi0a_standard;
extern const mxc_gpio_cfg_t gpio_cfg_spi0a_dual;
extern const mxc_gpio_cfg_t gpio_cfg_spi0a_quad;
extern const mxc_gpio_cfg_t gpio_cfg_spi0b_standard;
extern const mxc_gpio_cfg_t gpio_cfg_spi0b_3wire;
extern const mxc_gpio_cfg_t gpio_cfg_spi0b_dual;
extern const mxc_gpio_cfg_t gpio_cfg_spi0b_qua;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_standard;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_3wire;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_dual;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_quad;
extern const mxc_gpio_cfg_t gpio_cfg_spi2_standard;
extern const mxc_gpio_cfg_t gpio_cfg_spi2_3wire;
extern const mxc_gpio_cfg_t gpio_cfg_spi2_dual;
extern const mxc_gpio_cfg_t gpio_cfg_spi2_quad;

// SPI v2 Target Selects Pin Definitions
extern const mxc_gpio_cfg_t gpio_cfg_spi0a_ts0;
extern const mxc_gpio_cfg_t gpio_cfg_spi0b_ts0;
extern const mxc_gpio_cfg_t gpio_cfg_spi0_ts1;
extern const mxc_gpio_cfg_t gpio_cfg_spi0_ts2;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_ts0;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_ts1;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_ts2;
extern const mxc_gpio_cfg_t gpio_cfg_spi2_ts0;
extern const mxc_gpio_cfg_t gpio_cfg_spi2_ts1;
extern const mxc_gpio_cfg_t gpio_cfg_spi2_ts2;

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_MXC_PINS_H_
