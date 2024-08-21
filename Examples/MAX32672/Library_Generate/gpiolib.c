/**
 * @file    gpiolib
 * @brief   Example static library implementation.
 * @details GPIO library source.
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

#include "mxc_errors.h"
#include "gpio.h"
#include "gpiolib.h"

void gpio_set(const mxc_gpio_cfg_t *gpio)
{
    if (gpio->func == MXC_GPIO_FUNC_OUT)
        MXC_GPIO_OutSet(gpio->port, gpio->mask);
}

void gpio_clear(const mxc_gpio_cfg_t *gpio)
{
    if (gpio->func == MXC_GPIO_FUNC_OUT)
        MXC_GPIO_OutClr(gpio->port, gpio->mask);
}

int gpio_get(const mxc_gpio_cfg_t *gpio)
{
    if (gpio->func == MXC_GPIO_FUNC_IN)
        return MXC_GPIO_InGet(gpio->port, gpio->mask);

    return MXC_GPIO_OutGet(gpio->port, gpio->mask);
}
