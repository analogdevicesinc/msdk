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

/* **** Includes **** */
#include <stddef.h>
#include "mxc_sys.h"
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_pins.h"
#include "gpio.h"
#include "gpio_reva.h"
#include "gpio_common.h"
#include "uart.h"
#include "uart_revb.h"
#include "uart_common.h"
#include "mcr_regs.h"
#include "dma.h"

/* **** Functions **** */

int MXC_AFE_GPIO_Config(const mxc_gpio_cfg_t *cfg)
{
    int error;
    mxc_gpio_regs_t *gpio = cfg->port;

    // Configure alternate function
    error = MXC_GPIO_RevA_SetAF((mxc_gpio_reva_regs_t *)gpio, cfg->func, cfg->mask);

    if (error != E_NO_ERROR) {
        return error;
    }

    // Configure the pad
    switch (cfg->pad) {
    case MXC_GPIO_PAD_NONE:
        gpio->padctrl0 &= ~cfg->mask;
        gpio->padctrl1 &= ~cfg->mask;
        break;

    case MXC_GPIO_PAD_PULL_UP:
        gpio->padctrl0 |= cfg->mask;
        gpio->padctrl1 |= cfg->mask;
        gpio->ps |= cfg->mask;
        break;

    case MXC_GPIO_PAD_PULL_DOWN:
        gpio->padctrl0 |= cfg->mask;
        gpio->padctrl1 |= cfg->mask;
        gpio->ps &= ~cfg->mask;
        break;

    default:
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}
