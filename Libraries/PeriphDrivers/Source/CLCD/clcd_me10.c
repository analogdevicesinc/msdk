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
#include <string.h>
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "mxc_pins.h"
#include "clcd.h"
#include "clcd_reva.h"
#include "mxc_lock.h"

/* **** Definitions **** */

/* **** Globals **** */

/* ************************************************************************** */
int MXC_CLCD_Init(mxc_clcd_cfg_t *cfg)
{
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_TFT);

    MXC_GPIO_Config(&gpio_cfg_clcd_0);
    MXC_GPIO_Config(&gpio_cfg_clcd_1);
    MXC_GPIO_Config(&gpio_cfg_clcd_2);

    gpio_cfg_clcd_0.port->vssel |= gpio_cfg_clcd_0.mask;
    gpio_cfg_clcd_1.port->vssel |= gpio_cfg_clcd_1.mask;
    gpio_cfg_clcd_2.port->vssel |= gpio_cfg_clcd_2.mask;

    return MXC_CLCD_RevA_Init((mxc_clcd_reva_regs_t *)MXC_CLCD, cfg);
}

/* ************************************************************************* */
int MXC_CLCD_Shutdown(void)
{
    MXC_CLCD_RevA_Shutdown((mxc_clcd_reva_regs_t *)MXC_CLCD);

    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TFT);

    return E_NO_ERROR;
}

/* ************************************************************************* */
int MXC_CLCD_ConfigPanel(mxc_clcd_cfg_t *cfg)
{
    return MXC_CLCD_RevA_ConfigPanel((mxc_clcd_reva_regs_t *)MXC_CLCD, cfg);
}

/* ************************************************************************* */
int MXC_CLCD_Enable(void)
{
    return MXC_CLCD_RevA_Enable((mxc_clcd_reva_regs_t *)MXC_CLCD);
}

/* ************************************************************************* */
int MXC_CLCD_Disable(void)
{
    return MXC_CLCD_RevA_Disable((mxc_clcd_reva_regs_t *)MXC_CLCD);
}

/* ************************************************************************* */
int MXC_CLCD_SetFrameAddr(void *addr)
{
    return MXC_CLCD_RevA_SetFrameAddr((mxc_clcd_reva_regs_t *)MXC_CLCD, addr);
}
