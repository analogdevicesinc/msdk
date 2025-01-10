/**
 * @file mxc_pins.h
 * @brief      This file contains constant pin configurations for the peripherals.
 */

/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_MXC_PINS_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_MXC_PINS_H_

#include "gpio.h"

/***** Global Variables *****/
// Predefined GPIO Configurations
extern const mxc_gpio_cfg_t gpio_cfg_extclk;
extern const mxc_gpio_cfg_t gpio_cfg_i3c;

extern const mxc_gpio_cfg_t gpio_cfg_uart;

// Timers are only defined once, depending on package, each timer could be mapped to other pins
extern const mxc_gpio_cfg_t gpio_cfg_tmr0;
extern const mxc_gpio_cfg_t gpio_cfg_tmr1;
extern const mxc_gpio_cfg_t gpio_cfg_tmr2;
extern const mxc_gpio_cfg_t gpio_cfg_tmr3;
extern const mxc_gpio_cfg_t gpio_cfg_tmr4;
extern const mxc_gpio_cfg_t gpio_cfg_tmr5;
extern const mxc_gpio_cfg_t gpio_cfg_tmr0b;
extern const mxc_gpio_cfg_t gpio_cfg_tmr1b;
extern const mxc_gpio_cfg_t gpio_cfg_tmr2b;
extern const mxc_gpio_cfg_t gpio_cfg_tmr3b;
extern const mxc_gpio_cfg_t gpio_cfg_tmr4b;
extern const mxc_gpio_cfg_t gpio_cfg_tmr5b;

// SPI v2 Pin Definitions
extern const mxc_gpio_cfg_t gpio_cfg_spi_standard;
extern const mxc_gpio_cfg_t gpio_cfg_spi_3wire;
extern const mxc_gpio_cfg_t gpio_cfg_spi_dual;
extern const mxc_gpio_cfg_t gpio_cfg_spi_quad;

// SPI v2 Target Selects Pin Definitions
extern const mxc_gpio_cfg_t gpio_cfg_spi_ts0;
extern const mxc_gpio_cfg_t gpio_cfg_spi_ts1;
extern const mxc_gpio_cfg_t gpio_cfg_spi_ts2;

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_MXC_PINS_H_
