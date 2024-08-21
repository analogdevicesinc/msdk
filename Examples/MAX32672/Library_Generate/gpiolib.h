/**
 * @file    gpiolib.h
 * @brief   GPIO library header.
 * @details Function prototypes for GPIO library.
 */

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

#ifndef EXAMPLES_MAX32672_LIBRARY_GENERATE_GPIOLIB_H_
#define EXAMPLES_MAX32672_LIBRARY_GENERATE_GPIOLIB_H_

#include "gpio.h"

/**
 * @brief Sets the given GPIO to high state.
 * @param gpio GPIO to set.
 */
void gpio_set(const mxc_gpio_cfg_t *gpio);

/**
 * @brief Sets the given GPIO to low state.
 * @param gpio GPIO to set.
 */
void gpio_clear(const mxc_gpio_cfg_t *gpio);

/**
 * @brief Returns the input state of the given GPIO.
 * @param gpio GPIO to read.
 * @return Current GPIO state.
 */
int gpio_get(const mxc_gpio_cfg_t *gpio);

#endif // EXAMPLES_MAX32672_LIBRARY_GENERATE_GPIOLIB_H_
