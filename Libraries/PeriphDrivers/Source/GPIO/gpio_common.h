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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_GPIO_GPIO_COMMON_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_GPIO_GPIO_COMMON_H_

/* **** Includes **** */
#include "gpio_regs.h"
#include "gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* **** Function Prototypes **** */

int MXC_GPIO_Common_Init(uint32_t portmask);
void MXC_GPIO_Common_RegisterCallback(const mxc_gpio_cfg_t *cfg, mxc_gpio_callback_fn callback,
                                      void *cbdata);
void MXC_GPIO_Common_Handler(unsigned int port);
void MXC_GPIO_Common_SetConfigLock(mxc_gpio_config_lock_t locked);
mxc_gpio_config_lock_t MXC_GPIO_Common_GetConfigLock(void);

/**@} end of group gpio */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_GPIO_GPIO_COMMON_H_
