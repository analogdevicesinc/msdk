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
#include "gpio_common.h"
#include "mxc_device.h"
#include "mxc_assert.h"
#include "gpio.h"

/* **** Globals **** */
static void (*callback[MXC_CFG_GPIO_INSTANCES][MXC_CFG_GPIO_PINS_PORT])(void *);
static void *cbparam[MXC_CFG_GPIO_INSTANCES][MXC_CFG_GPIO_PINS_PORT];
static uint8_t initialized = 0;
static mxc_gpio_config_lock_t cfg_lock = MXC_GPIO_CONFIG_UNLOCKED;

/* **** Functions **** */
int MXC_GPIO_Common_Init(uint32_t portmask)
{
    if (!initialized) {
        int i, j;

        for (i = 0; i < MXC_CFG_GPIO_INSTANCES; i++) {
            // Initialize call back arrays
            for (j = 0; j < MXC_CFG_GPIO_PINS_PORT; j++) {
                callback[i][j] = NULL;
            }
        }

        initialized = 1;
    }

    return E_NO_ERROR;
}

void MXC_GPIO_Common_RegisterCallback(const mxc_gpio_cfg_t *cfg, mxc_gpio_callback_fn func,
                                      void *cbdata)
{
    uint32_t mask;
    unsigned int pin;

    mask = cfg->mask;
    pin = 0;

    while (mask) {
        if (mask & 1) {
            callback[MXC_GPIO_GET_IDX(cfg->port)][pin] = func;
            cbparam[MXC_GPIO_GET_IDX(cfg->port)][pin] = cbdata;
        }

        pin++;
        mask >>= 1;
    }
}

void MXC_GPIO_Common_Handler(unsigned int port)
{
    uint32_t stat;
    unsigned int pin;

    MXC_ASSERT(port < MXC_CFG_GPIO_INSTANCES);

    mxc_gpio_regs_t *gpio = MXC_GPIO_GET_GPIO(port);

    stat = MXC_GPIO_GetFlags(gpio);
    MXC_GPIO_ClearFlags(gpio, stat);

    pin = 0;

    while (stat) {
        if (stat & 1) {
            if (callback[port][pin]) {
                callback[port][pin](cbparam[port][pin]);
            }
        }

        pin++;
        stat >>= 1;
    }
}

void MXC_GPIO_Common_SetConfigLock(mxc_gpio_config_lock_t locked)
{
    cfg_lock = locked;
}

mxc_gpio_config_lock_t MXC_GPIO_Common_GetConfigLock(void)
{
    return cfg_lock;
}
