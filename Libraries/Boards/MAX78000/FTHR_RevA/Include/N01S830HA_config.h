/******************************************************************************
 *
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

#ifndef LIBRARIES_BOARDS_MAX78000_FTHR_REVA_INCLUDE_N01S830HA_CONFIG_H_
#define LIBRARIES_BOARDS_MAX78000_FTHR_REVA_INCLUDE_N01S830HA_CONFIG_H_

#include "N01S830HA.h"
#include "gpio.h"
#include "mxc_device.h"
#include "fastspi.h"

#define N01S830HA_HOLD_PIN_PORT MXC_GPIO0

#define N01S830HA_HOLD_PIN_MASK MXC_GPIO_PIN_9

#define N01S830HA_VSSEL MXC_GPIO_VSSEL_VDDIOH

// N01S830HA_HOLD_PIN can be defined by other files to completely re-define the
// hold pin struct if necessary.  Otherwise, the hold pin will be defined here using
// the port, pin, and vssel definitions above.

static const mxc_gpio_cfg_t N01S830HA_hold_pin = { .port = N01S830HA_HOLD_PIN_PORT,
                                                   .mask = N01S830HA_HOLD_PIN_MASK,
                                                   .func = MXC_GPIO_FUNC_OUT,
                                                   .pad = MXC_GPIO_PAD_WEAK_PULL_UP,
                                                   .vssel = N01S830HA_VSSEL };

#endif // LIBRARIES_BOARDS_MAX78000_FTHR_REVA_INCLUDE_N01S830HA_CONFIG_H_
