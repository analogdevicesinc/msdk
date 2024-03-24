/**
 * @file   mxc_pins.h
 * @brief  This file contains constant pin configurations for the peripherals.
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

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_MXC_PINS_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_MXC_PINS_H_

/* **** Includes **** */
#include "gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/***** Global Variables *****/

// Predefined GPIO Configurations
extern const mxc_gpio_cfg_t gpio_cfg_tmr0;
extern const mxc_gpio_cfg_t gpio_cfg_tmr1;
extern const mxc_gpio_cfg_t gpio_cfg_tmr2;
extern const mxc_gpio_cfg_t gpio_cfg_tmr3;
extern const mxc_gpio_cfg_t gpio_cfg_tmr4;
extern const mxc_gpio_cfg_t gpio_cfg_tmr5;
extern const mxc_gpio_cfg_t gpio_cfg_uart0;
extern const mxc_gpio_cfg_t gpio_cfg_uart0_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart1;
extern const mxc_gpio_cfg_t gpio_cfg_uart1_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart2;
extern const mxc_gpio_cfg_t gpio_cfg_uart2_flow;
extern const mxc_gpio_cfg_t gpio_cfg_i2c0;
extern const mxc_gpio_cfg_t gpio_cfg_i2c1;
extern const mxc_gpio_cfg_t gpio_cfg_spi0_0;
extern const mxc_gpio_cfg_t gpio_cfg_spi0_1;
extern const mxc_gpio_cfg_t gpio_cfg_spi1;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_ss0;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_ss1;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_ss2;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_ss3;
extern const mxc_gpio_cfg_t gpio_cfg_spi2;
extern const mxc_gpio_cfg_t gpio_cfg_spi2_ss0;
extern const mxc_gpio_cfg_t gpio_cfg_spi2_ss1;
extern const mxc_gpio_cfg_t gpio_cfg_spi2_ss2;
extern const mxc_gpio_cfg_t gpio_cfg_spi2_ss3;
extern const mxc_gpio_cfg_t gpio_cfg_spi3;
extern const mxc_gpio_cfg_t gpio_cfg_spi3_ss0;
extern const mxc_gpio_cfg_t gpio_cfg_spi3_ss1;
extern const mxc_gpio_cfg_t gpio_cfg_spi3_ss2;
extern const mxc_gpio_cfg_t gpio_cfg_spi3_ss3;
extern const mxc_gpio_cfg_t gpio_cfg_pt0_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt1_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt2_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt3_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt4_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt5_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt6_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt7_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt8_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt9_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt10_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt11_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt12_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt13_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt14_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt15_0;
extern const mxc_gpio_cfg_t gpio_cfg_pt0_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt1_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt2_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt3_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt4_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt5_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt6_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt7_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt8_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt9_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt10_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt11_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt12_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt13_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt14_1;
extern const mxc_gpio_cfg_t gpio_cfg_pt15_1;
extern const mxc_gpio_cfg_t gpio_cfg_rtcsqw;
extern const mxc_gpio_cfg_t gpio_cfg_spixfc;
extern const mxc_gpio_cfg_t gpio_cfg_spixr;
extern const mxc_gpio_cfg_t gpio_cfg_hyp;
extern const mxc_gpio_cfg_t gpio_cfg_hyp_cs0;
extern const mxc_gpio_cfg_t gpio_cfg_hyp_cs1;

extern const mxc_gpio_cfg_t gpio_cfg_sdhc_0;
extern const mxc_gpio_cfg_t gpio_cfg_sdhc_1;
extern const mxc_gpio_cfg_t gpio_cfg_owm;

extern const mxc_gpio_cfg_t gpio_cfg_clcd_0;
extern const mxc_gpio_cfg_t gpio_cfg_clcd_1;
extern const mxc_gpio_cfg_t gpio_cfg_clcd_2;

extern const mxc_gpio_cfg_t gpio_cfg_i2s;

// SPI v2 Pin Definitions
extern const mxc_gpio_cfg_t gpio_cfg_spi0_standard;
extern const mxc_gpio_cfg_t gpio_cfg_spi0_3wire;
// MXC_SPI0 does not support Dual or Quad mode.
extern const mxc_gpio_cfg_t gpio_cfg_spi1_standard;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_3wire;
// MXC_SPI1 does not support Dual or Quad mode.
extern const mxc_gpio_cfg_t gpio_cfg_spi2_standard;
extern const mxc_gpio_cfg_t gpio_cfg_spi2_3wire;
// MXC_SPI2 does not support Dual or Quad mode.
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
extern const mxc_gpio_cfg_t gpio_cfg_spi2_ts0;
extern const mxc_gpio_cfg_t gpio_cfg_spi2_ts1;
extern const mxc_gpio_cfg_t gpio_cfg_spi2_ts2;
extern const mxc_gpio_cfg_t gpio_cfg_spi2_ts3;
extern const mxc_gpio_cfg_t gpio_cfg_spi3_ts0;
extern const mxc_gpio_cfg_t gpio_cfg_spi3_ts1;
extern const mxc_gpio_cfg_t gpio_cfg_spi3_ts2;
extern const mxc_gpio_cfg_t gpio_cfg_spi3_ts3;

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_MXC_PINS_H_
