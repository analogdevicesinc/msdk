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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32572_MXC_PINS_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32572_MXC_PINS_H_

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

extern const mxc_gpio_cfg_t gpio_cfg_skbd_P2;

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32572_MXC_PINS_H_
