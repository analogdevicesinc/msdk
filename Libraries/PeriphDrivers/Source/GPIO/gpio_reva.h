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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_GPIO_GPIO_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_GPIO_GPIO_REVA_H_

/* **** Includes **** */
#include "gpio_reva_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Enumeration type for the pullup strength on a given pin.
 */
typedef enum {
    MXC_GPIO_PS_NONE, /**< No pull-up or pull-down strength required*/
    MXC_GPIO_PS_PULL_SELECT, /**< Selct pull-up or pull-down strength*/
} mxc_gpio_ps_t;

/* **** Function Prototypes **** */

uint32_t MXC_GPIO_RevA_InGet(mxc_gpio_reva_regs_t *port, uint32_t mask);
void MXC_GPIO_RevA_OutSet(mxc_gpio_reva_regs_t *port, uint32_t mask);
void MXC_GPIO_RevA_OutClr(mxc_gpio_reva_regs_t *port, uint32_t mask);
uint32_t MXC_GPIO_RevA_OutGet(mxc_gpio_reva_regs_t *port, uint32_t mask);
void MXC_GPIO_RevA_OutPut(mxc_gpio_reva_regs_t *port, uint32_t mask, uint32_t val);
void MXC_GPIO_RevA_OutToggle(mxc_gpio_reva_regs_t *port, uint32_t mask);
int MXC_GPIO_RevA_IntConfig(const mxc_gpio_cfg_t *cfg, mxc_gpio_int_pol_t pol);
void MXC_GPIO_RevA_EnableInt(mxc_gpio_reva_regs_t *port, uint32_t mask);
void MXC_GPIO_RevA_DisableInt(mxc_gpio_reva_regs_t *port, uint32_t mask);
void MXC_GPIO_RevA_ClearFlags(mxc_gpio_reva_regs_t *port, uint32_t flags);
uint32_t MXC_GPIO_RevA_GetFlags(mxc_gpio_reva_regs_t *port);
int MXC_GPIO_RevA_SetVSSEL(mxc_gpio_reva_regs_t *port, mxc_gpio_vssel_t vssel, uint32_t mask);
int MXC_GPIO_RevA_SetAF(mxc_gpio_reva_regs_t *port, mxc_gpio_func_t func, uint32_t mask);
void MXC_GPIO_RevA_SetWakeEn(mxc_gpio_reva_regs_t *port, uint32_t mask);
void MXC_GPIO_RevA_ClearWakeEn(mxc_gpio_reva_regs_t *port, uint32_t mask);
uint32_t MXC_GPIO_RevA_GetWakeEn(mxc_gpio_reva_regs_t *port);
int MXC_GPIO_RevA_SetDriveStrength(mxc_gpio_reva_regs_t *port, mxc_gpio_drvstr_t drvstr,
                                   uint32_t mask);

/**@} end of group gpio */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_GPIO_GPIO_REVA_H_
