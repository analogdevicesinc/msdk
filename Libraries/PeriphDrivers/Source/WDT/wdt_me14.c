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
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "wdt.h"
#include "wdt_reva.h"

/* **** Functions **** */

int MXC_WDT_Init(mxc_wdt_regs_t *wdt)
{
#ifndef MSDK_NO_GPIO_CLK_INIT
    if (wdt == MXC_WDT0) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_WDT0);
    } else if (wdt == MXC_WDT1) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_WDT1);
    } else if (wdt == MXC_WDT2) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_WDT2);
    } else {
        return E_BAD_PARAM;
    }
#else
    (void)wdt;
#endif

    return E_NO_ERROR;
}

int MXC_WDT_Shutdown(mxc_wdt_regs_t *wdt)
{
    if (wdt == MXC_WDT0) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_WDT0);
    } else if (wdt == MXC_WDT1) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_WDT1);
    } else if (wdt == MXC_WDT2) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_WDT2);
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}
void MXC_WDT_SetIntPeriod(mxc_wdt_regs_t *wdt, mxc_wdt_period_t period)
{
    MXC_WDT_RevA_SetIntPeriod((mxc_wdt_reva_regs_t *)wdt, period);
}

void MXC_WDT_SetResetPeriod(mxc_wdt_regs_t *wdt, mxc_wdt_period_t period)
{
    MXC_WDT_RevA_SetResetPeriod((mxc_wdt_reva_regs_t *)wdt, period);
}

void MXC_WDT_Enable(mxc_wdt_regs_t *wdt)
{
    MXC_WDT_RevA_Enable((mxc_wdt_reva_regs_t *)wdt);
}

void MXC_WDT_Disable(mxc_wdt_regs_t *wdt)
{
    MXC_WDT_RevA_Disable((mxc_wdt_reva_regs_t *)wdt);
}

void MXC_WDT_EnableInt(mxc_wdt_regs_t *wdt)
{
    MXC_WDT_RevA_EnableInt((mxc_wdt_reva_regs_t *)wdt, MXC_WDT_REVA_ENABLE);
}

void MXC_WDT_EnableReset(mxc_wdt_regs_t *wdt)
{
    MXC_WDT_RevA_EnableReset((mxc_wdt_reva_regs_t *)wdt, MXC_WDT_REVA_ENABLE);
}

void MXC_WDT_DisableInt(mxc_wdt_regs_t *wdt)
{
    MXC_WDT_RevA_EnableInt((mxc_wdt_reva_regs_t *)wdt, MXC_WDT_REVA_DISABLE);
}

void MXC_WDT_DisableReset(mxc_wdt_regs_t *wdt)
{
    MXC_WDT_RevA_EnableReset((mxc_wdt_reva_regs_t *)wdt, MXC_WDT_REVA_DISABLE);
}

void MXC_WDT_ResetTimer(mxc_wdt_regs_t *wdt)
{
    MXC_WDT_RevA_ResetTimer((mxc_wdt_reva_regs_t *)wdt);
}

int MXC_WDT_GetResetFlag(mxc_wdt_regs_t *wdt)
{
    return MXC_WDT_RevA_GetResetFlag((mxc_wdt_reva_regs_t *)wdt);
}

void MXC_WDT_ClearResetFlag(mxc_wdt_regs_t *wdt)
{
    MXC_WDT_RevA_ClearResetFlag((mxc_wdt_reva_regs_t *)wdt);
}

int MXC_WDT_GetIntFlag(mxc_wdt_regs_t *wdt)
{
    return MXC_WDT_RevA_GetIntFlag((mxc_wdt_reva_regs_t *)wdt);
}

void MXC_WDT_ClearIntFlag(mxc_wdt_regs_t *wdt)
{
    MXC_WDT_RevA_ClearIntFlag((mxc_wdt_reva_regs_t *)wdt);
}
