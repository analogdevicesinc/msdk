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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32675_MXC_PINS_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32675_MXC_PINS_H_

#include "gpio.h"

/***** Global Variables *****/
// Predefined GPIO Configurations
extern const mxc_gpio_cfg_t gpio_cfg_hfextclk;
#define gpio_cfg_extclk gpio_cfg_hfextclk
/* ^ Defined for backwards compatability after renaming
    gpio_cfg_extclk -> gpio_cfg_hfextclk
*/
extern const mxc_gpio_cfg_t gpio_cfg_i2c0;
extern const mxc_gpio_cfg_t gpio_cfg_i2c1;
extern const mxc_gpio_cfg_t gpio_cfg_i2c2;
extern const mxc_gpio_cfg_t gpio_cfg_i2c2b;
extern const mxc_gpio_cfg_t gpio_cfg_i2c2c;

extern const mxc_gpio_cfg_t gpio_cfg_uart0;
extern const mxc_gpio_cfg_t gpio_cfg_uart0_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart0_flow_disable;
//extern const mxc_gpio_cfg_t gpio_cfg_uart1;
//extern const mxc_gpio_cfg_t gpio_cfg_uart1_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart2;
extern const mxc_gpio_cfg_t gpio_cfg_uart2_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart2_flow_disable;
extern const mxc_gpio_cfg_t gpio_cfg_uart3;
extern const mxc_gpio_cfg_t gpio_cfg_uart3_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart3_flow_disable;

// extern const mxc_gpio_cfg_t gpio_cfg_spi0;
// NOTE: SPI1 definied here with SS1 only, SS0 is on port0 by itself.
extern const mxc_gpio_cfg_t gpio_cfg_spi1;
// NOTE: SPI2 defined here with SS0 only, and NOT SS1 and SS2
extern const mxc_gpio_cfg_t gpio_cfg_spi2;
// extern const mxc_gpio_cfg_t gpio_cfg_spi2b;
// NOTE: SPI3 defined here with SS0 only, and NOT SS1, SS2, or SS3
// extern const mxc_gpio_cfg_t gpio_cfg_spi3;

// Timers are only defined once, depending on package, each timer could be mapped to other pins
extern const mxc_gpio_cfg_t gpio_cfg_tmr0;
extern const mxc_gpio_cfg_t gpio_cfg_tmr1;
extern const mxc_gpio_cfg_t gpio_cfg_tmr2;
extern const mxc_gpio_cfg_t gpio_cfg_tmr3;
extern const mxc_gpio_cfg_t gpio_cfg_tmr4;
extern const mxc_gpio_cfg_t gpio_cfg_tmr5;

extern const mxc_gpio_cfg_t gpio_cfg_i2s0;

//extern const mxc_gpio_cfg_t gpio_cfg_rtcsqw;
//extern const mxc_gpio_cfg_t gpio_cfg_rtcsqwb;

extern const mxc_gpio_cfg_t gpio_cfg_lc1;
extern const mxc_gpio_cfg_t gpio_cfg_mon_lc1;
extern const mxc_gpio_cfg_t gpio_cfg_cmd_rs_lc1;
extern const mxc_gpio_cfg_t gpio_cfg_chrg_lc1;
extern const mxc_gpio_cfg_t gpio_cfg_lc2;
extern const mxc_gpio_cfg_t gpio_cfg_mon_lc2;
extern const mxc_gpio_cfg_t gpio_cfg_cmd_rs_lc2;
extern const mxc_gpio_cfg_t gpio_cfg_chrg_lc2;

// SPI v2 Pin Definitions
extern const mxc_gpio_cfg_t gpio_cfg_spi1_standard;
extern const mxc_gpio_cfg_t gpio_cfg_spi1_3wire;
// MXC_SPI1 does not support Dual or Quad modes.

// SPI v2 Target Selects Pin Definitions
extern const mxc_gpio_cfg_t gpio_cfg_spi1_ts0;

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32675_MXC_PINS_H_
