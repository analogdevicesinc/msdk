/******************************************************************************
 *
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

#ifndef LIBRARIES_BOARDS_MAX78002_EVKIT_V1_INCLUDE_FASTSPI_CONFIG_H_
#define LIBRARIES_BOARDS_MAX78002_EVKIT_V1_INCLUDE_FASTSPI_CONFIG_H_

/**
 * @file    fastspi_config.c
 * @brief   "fastspi" configuration file for MAX78000FTHR board
 */

#include "spi.h"
#include "gpio.h"

// (*) Required definitions:
#define FASTSPI_INSTANCE MXC_SPI0
#define FASTSPI_SPEED 24000000

#define FASTSPI_PINS_PORT MXC_GPIO0
#define FASTSPI_VSSEL MXC_GPIO_VSSEL_VDDIOH
#define FASTSPI_PINS_MASK \
    (MXC_GPIO_PIN_5 | MXC_GPIO_PIN_6 | MXC_GPIO_PIN_7 | MXC_GPIO_PIN_8 | MXC_GPIO_PIN_9)

#define FASTSPI_SS_PORT MXC_GPIO0
#define FASTSPI_SS_PIN MXC_GPIO_PIN_4
#define FASTSPI_SS_NUM 1

// (*) Required GPIO definitions:
static const mxc_gpio_cfg_t fastspi_ss_pin = { .port = FASTSPI_SS_PORT,
                                               .mask = FASTSPI_SS_PIN,
                                               .func = MXC_GPIO_FUNC_ALT1,
                                               .pad = MXC_GPIO_PAD_WEAK_PULL_UP,
                                               .vssel = FASTSPI_VSSEL };

static const mxc_gpio_cfg_t fastspi_spi_pins = { .port = FASTSPI_PINS_PORT,
                                                 .mask = FASTSPI_PINS_MASK,
                                                 .func = MXC_GPIO_FUNC_ALT1,
                                                 .pad = MXC_GPIO_PAD_NONE,
                                                 .vssel = FASTSPI_VSSEL };

#endif // LIBRARIES_BOARDS_MAX78002_EVKIT_V1_INCLUDE_FASTSPI_CONFIG_H_
