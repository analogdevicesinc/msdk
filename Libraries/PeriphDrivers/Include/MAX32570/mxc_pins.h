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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_MXC_PINS_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_MXC_PINS_H_

#include "gpio.h"

/***** Global Variables *****/
typedef enum { MAP_A, MAP_B } sys_map_t;

// Predefined GPIO Configurations
extern const mxc_gpio_cfg_t gpio_cfg_i2c0;
extern const mxc_gpio_cfg_t gpio_cfg_i2c1;
extern const mxc_gpio_cfg_t gpio_cfg_i2c2;
extern const mxc_gpio_cfg_t gpio_cfg_i2c2b;
extern const mxc_gpio_cfg_t gpio_cfg_i2c2c;

extern const mxc_gpio_cfg_t gpio_cfg_uart0;
extern const mxc_gpio_cfg_t gpio_cfg_uart0_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart1;
extern const mxc_gpio_cfg_t gpio_cfg_uart1_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart2;
extern const mxc_gpio_cfg_t gpio_cfg_uart2_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart3;
extern const mxc_gpio_cfg_t gpio_cfg_uart3_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart4;
extern const mxc_gpio_cfg_t gpio_cfg_uart4b;
extern const mxc_gpio_cfg_t gpio_cfg_uart4c;
extern const mxc_gpio_cfg_t gpio_cfg_uart4_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart4b_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart4c_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart5;
extern const mxc_gpio_cfg_t gpio_cfg_uart5b;
extern const mxc_gpio_cfg_t gpio_cfg_uart5_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart5b_flow;
// NOTE: uart5b flow control is split across port pins 0.31 and 1.0
extern const mxc_gpio_cfg_t gpio_cfg_uart5c_P1_flow;
extern const mxc_gpio_cfg_t gpio_cfg_uart5c_P0_flow;

extern const mxc_gpio_cfg_t gpio_cfg_spi0;
// NOTE: SPI1 definied here with SS1 only, SS0 is on port0 by itself.
extern const mxc_gpio_cfg_t gpio_cfg_spi1;
// NOTE: SPI2 defined here with SS0 only, and NOT SS1 and SS2
extern const mxc_gpio_cfg_t gpio_cfg_spi2;
extern const mxc_gpio_cfg_t gpio_cfg_spi2b;
// NOTE: SPI3 defined here with SS0 only, and NOT SS1, SS2, or SS3
extern const mxc_gpio_cfg_t gpio_cfg_spi3;

// Timers are only defined once, depending on package, each timer could be mapped to other pins
extern const mxc_gpio_cfg_t gpio_cfg_tmr0;
extern const mxc_gpio_cfg_t gpio_cfg_tmr1;
extern const mxc_gpio_cfg_t gpio_cfg_tmr2;
extern const mxc_gpio_cfg_t gpio_cfg_tmr3;
extern const mxc_gpio_cfg_t gpio_cfg_tmr4;
extern const mxc_gpio_cfg_t gpio_cfg_tmr5;

// Pulse trains are only defined once, depending on package, each PT could be mapped to other pins
extern const mxc_gpio_cfg_t gpio_cfg_pt0;
extern const mxc_gpio_cfg_t gpio_cfg_pt1;
extern const mxc_gpio_cfg_t gpio_cfg_pt2;
extern const mxc_gpio_cfg_t gpio_cfg_pt3;
extern const mxc_gpio_cfg_t gpio_cfg_pt4;
extern const mxc_gpio_cfg_t gpio_cfg_pt5;
extern const mxc_gpio_cfg_t gpio_cfg_pt6;
extern const mxc_gpio_cfg_t gpio_cfg_pt7;

extern const mxc_gpio_cfg_t gpio_cfg_owm;
extern const mxc_gpio_cfg_t gpio_cfg_owmb;

// Port 0 Pins 6-14, Port 1 Pins 1-5 and 16-19, Port 2 Pins 10-19
// Other configurations are available, depending on package, to allow the use of EMAC or SDHC
// Note that both P1a and P1b must be configured for proper operation
extern const mxc_gpio_cfg_t gpio_cfg_P0_clcd;
extern const mxc_gpio_cfg_t gpio_cfg_P1a_clcd;
extern const mxc_gpio_cfg_t gpio_cfg_P1b_clcd;
extern const mxc_gpio_cfg_t gpio_cfg_P2_clcd;

extern const mxc_gpio_cfg_t gpio_cfg_rtcsqw;
extern const mxc_gpio_cfg_t gpio_cfg_rtcsqwb;

extern const mxc_gpio_cfg_t gpio_cfg_sdhc;
extern const mxc_gpio_cfg_t gpio_cfg_sdhcb;

extern const mxc_gpio_cfg_t gpio_cfg_sc0;
extern const mxc_gpio_cfg_t gpio_cfg_sc1;

// Note that both P0 and P1 must be configured for proper operation
extern const mxc_gpio_cfg_t gpio_cfg_spixf;
extern const mxc_gpio_cfg_t gpio_cfg_spixr_P0;
extern const mxc_gpio_cfg_t gpio_cfg_spixr_P1;

// Note that both P2a and P2b must be configured for proper operation
extern const mxc_gpio_cfg_t gpio_cfg_emac_P2a;
extern const mxc_gpio_cfg_t gpio_cfg_emac_P2b;

// Note that all of the following must be configured for proper operation
extern const mxc_gpio_cfg_t gpio_cfg_kbd_P2;

// Note that both P0 and P1 must be configured for proper operation
extern const mxc_gpio_cfg_t gpio_cfg_pcif_P0_BITS_0_7;
extern const mxc_gpio_cfg_t gpio_cfg_pcif_P0_BITS_8;
extern const mxc_gpio_cfg_t gpio_cfg_pcif_P1_BITS_9;
extern const mxc_gpio_cfg_t gpio_cfg_pcif_P1_BITS_10_11;
extern const mxc_gpio_cfg_t gpio_cfg_pcif_hsync;
extern const mxc_gpio_cfg_t gpio_cfg_pcif_vsync;
extern const mxc_gpio_cfg_t gpio_cfg_pcif_pclk;
extern const mxc_gpio_cfg_t gpio_cfg_pcif_pwrdwn;

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_MXC_PINS_H_
