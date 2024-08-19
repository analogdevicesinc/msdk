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

void MXC_WDT_RevA_SetIntPeriod(mxc_wdt_reva_regs_t *wdt, mxc_wdt_period_t period)
{
    MXC_SETFIELD(wdt->ctrl, MXC_F_WDT_REVA_CTRL_INT_PERIOD, period);
}

void MXC_WDT_RevA_SetResetPeriod(mxc_wdt_reva_regs_t *wdt, mxc_wdt_period_t period)
{
    MXC_SETFIELD(wdt->ctrl, MXC_F_WDT_REVA_CTRL_RST_PERIOD,
                 (period
                  << (MXC_F_WDT_REVA_CTRL_RST_PERIOD_POS - MXC_F_WDT_REVA_CTRL_INT_PERIOD_POS)));
}

void MXC_WDT_RevA_Enable(mxc_wdt_reva_regs_t *wdt)
{
    wdt->ctrl |= MXC_F_WDT_REVA_CTRL_WDT_EN;
}

void MXC_WDT_RevA_Disable(mxc_wdt_reva_regs_t *wdt)
{
    wdt->ctrl &= ~(MXC_F_WDT_REVA_CTRL_WDT_EN);
}

void MXC_WDT_RevA_EnableInt(mxc_wdt_reva_regs_t *wdt, mxc_wdt_reva_en_t enable)
{
    if (enable) {
        wdt->ctrl |= MXC_F_WDT_REVA_CTRL_INT_EN;
    } else {
        wdt->ctrl &= ~(MXC_F_WDT_REVA_CTRL_INT_EN);
    }
}

void MXC_WDT_RevA_EnableReset(mxc_wdt_reva_regs_t *wdt, mxc_wdt_reva_en_t enable)
{
    if (enable) {
        wdt->ctrl |= MXC_F_WDT_REVA_CTRL_RST_EN;
    } else {
        wdt->ctrl &= ~(MXC_F_WDT_REVA_CTRL_RST_EN);
    }
}

void MXC_WDT_RevA_ResetTimer(mxc_wdt_reva_regs_t *wdt)
{
    wdt->rst = 0x00A5;
    wdt->rst = 0x005A;
}

int MXC_WDT_RevA_GetResetFlag(mxc_wdt_reva_regs_t *wdt)
{
    return !!(wdt->ctrl & MXC_F_WDT_REVA_CTRL_RST_FLAG);
}

void MXC_WDT_RevA_ClearResetFlag(mxc_wdt_reva_regs_t *wdt)
{
    wdt->ctrl &= ~(MXC_F_WDT_REVA_CTRL_RST_FLAG);
}

int MXC_WDT_RevA_GetIntFlag(mxc_wdt_reva_regs_t *wdt)
{
    return !!(wdt->ctrl & MXC_F_WDT_REVA_CTRL_INT_FLAG);
}

void MXC_WDT_RevA_ClearIntFlag(mxc_wdt_reva_regs_t *wdt)
{
    wdt->ctrl &= ~(MXC_F_WDT_REVA_CTRL_INT_FLAG);
}
