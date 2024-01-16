/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

#ifndef EXAMPLES_MAX32665_GPIO_EXAMPLE_CONFIG_H_
#define EXAMPLES_MAX32665_GPIO_EXAMPLE_CONFIG_H_

#include "mxc_device.h"
#include "gpio.h"
#include "board.h"

/***** Definitions *****/
#if defined(BOARD_FTHR)
#define MXC_GPIO_PORT_IN MXC_GPIO1
#define MXC_GPIO_PIN_IN MXC_GPIO_PIN_10

#define MXC_GPIO_PORT_OUT MXC_GPIO0
#define MXC_GPIO_PIN_OUT MXC_GPIO_PIN_29

#define MXC_GPIO_PORT_INTERRUPT_IN MXC_GPIO0
#define MXC_GPIO_PIN_INTERRUPT_IN MXC_GPIO_PIN_16

#define MXC_GPIO_PORT_INTERRUPT_STATUS MXC_GPIO0
#define MXC_GPIO_PIN_INTERRUPT_STATUS MXC_GPIO_PIN_30
#elif defined(BOARD_FTHR2)
#define MXC_GPIO_PORT_IN MXC_GPIO0
#define MXC_GPIO_PIN_IN MXC_GPIO_PIN_24

#define MXC_GPIO_PORT_OUT MXC_GPIO0
#define MXC_GPIO_PIN_OUT MXC_GPIO_PIN_29

#define MXC_GPIO_PORT_INTERRUPT_IN MXC_GPIO0
#define MXC_GPIO_PIN_INTERRUPT_IN MXC_GPIO_PIN_28

#define MXC_GPIO_PORT_INTERRUPT_STATUS MXC_GPIO0
#define MXC_GPIO_PIN_INTERRUPT_STATUS MXC_GPIO_PIN_30
#else
#define MXC_GPIO_PORT_IN MXC_GPIO1
#define MXC_GPIO_PIN_IN MXC_GPIO_PIN_6

#define MXC_GPIO_PORT_OUT MXC_GPIO1
#define MXC_GPIO_PIN_OUT MXC_GPIO_PIN_14

#define MXC_GPIO_PORT_INTERRUPT_IN MXC_GPIO1
#define MXC_GPIO_PIN_INTERRUPT_IN MXC_GPIO_PIN_7

#define MXC_GPIO_PORT_INTERRUPT_STATUS MXC_GPIO1
#define MXC_GPIO_PIN_INTERRUPT_STATUS MXC_GPIO_PIN_15
#endif

#endif // EXAMPLES_MAX32665_GPIO_EXAMPLE_CONFIG_H_
