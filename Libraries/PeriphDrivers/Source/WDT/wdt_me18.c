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
#include "wdt_revb.h"

static bool g_is_clock_locked[MXC_CFG_WDT_INSTANCES] = { [0 ... MXC_CFG_WDT_INSTANCES - 1] =
                                                             false };

/* **** Functions **** */

int MXC_WDT_Init(mxc_wdt_regs_t *wdt, mxc_wdt_cfg_t *cfg)
{
#ifndef MSDK_NO_GPIO_CLK_INIT
    if (wdt == MXC_WDT0) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_WDT0);
    } else if (wdt == MXC_WDT1) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_WDT1);
    } else {
        return E_BAD_PARAM;
    }
#endif

    MXC_WDT_RevB_Init((mxc_wdt_revb_regs_t *)wdt, (mxc_wdt_revb_cfg_t *)cfg);

    return E_NO_ERROR;
}

int MXC_WDT_Shutdown(mxc_wdt_regs_t *wdt)
{
    if (wdt == MXC_WDT0) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_WDT0);
    } else if (wdt == MXC_WDT1) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_WDT1);
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

void MXC_WDT_SetIntPeriod(mxc_wdt_regs_t *wdt, mxc_wdt_cfg_t *cfg)
{
    MXC_WDT_RevB_SetIntPeriod((mxc_wdt_revb_regs_t *)wdt, (mxc_wdt_revb_cfg_t *)cfg);
}

void MXC_WDT_SetResetPeriod(mxc_wdt_regs_t *wdt, mxc_wdt_cfg_t *cfg)
{
    MXC_WDT_RevB_SetResetPeriod((mxc_wdt_revb_regs_t *)wdt, (mxc_wdt_revb_cfg_t *)cfg);
}

void MXC_WDT_Enable(mxc_wdt_regs_t *wdt)
{
    MXC_WDT_RevB_Enable((mxc_wdt_revb_regs_t *)wdt);
}

void MXC_WDT_Disable(mxc_wdt_regs_t *wdt)
{
    MXC_WDT_RevB_Disable((mxc_wdt_revb_regs_t *)wdt);
}

void MXC_WDT_EnableInt(mxc_wdt_regs_t *wdt)
{
    MXC_WDT_RevB_EnableInt((mxc_wdt_revb_regs_t *)wdt, MXC_WDT_REVB_ENABLE);
}

void MXC_WDT_DisableInt(mxc_wdt_regs_t *wdt)
{
    MXC_WDT_RevB_EnableInt((mxc_wdt_revb_regs_t *)wdt, MXC_WDT_REVB_DISABLE);
}

void MXC_WDT_EnableReset(mxc_wdt_regs_t *wdt)
{
    MXC_WDT_RevB_EnableReset((mxc_wdt_revb_regs_t *)wdt, MXC_WDT_REVB_ENABLE);
}

void MXC_WDT_DisableReset(mxc_wdt_regs_t *wdt)
{
    MXC_WDT_RevB_EnableReset((mxc_wdt_revb_regs_t *)wdt, MXC_WDT_REVB_DISABLE);
}

void MXC_WDT_ResetTimer(mxc_wdt_regs_t *wdt)
{
    MXC_WDT_RevB_ResetTimer((mxc_wdt_revb_regs_t *)wdt);
}

int MXC_WDT_GetResetFlag(mxc_wdt_regs_t *wdt)
{
    return MXC_WDT_RevB_GetResetFlag((mxc_wdt_revb_regs_t *)wdt);
}

void MXC_WDT_ClearResetFlag(mxc_wdt_regs_t *wdt)
{
    MXC_WDT_RevB_ClearResetFlag((mxc_wdt_revb_regs_t *)wdt);
}

int MXC_WDT_GetIntFlag(mxc_wdt_regs_t *wdt)
{
    return MXC_WDT_RevB_GetIntFlag((mxc_wdt_revb_regs_t *)wdt);
}

void MXC_WDT_ClearIntFlag(mxc_wdt_regs_t *wdt)
{
    MXC_WDT_RevB_ClearIntFlag((mxc_wdt_revb_regs_t *)wdt);
}

int MXC_WDT_SetClockSource(mxc_wdt_regs_t *wdt, mxc_wdt_clock_t clock_source)
{
    if (g_is_clock_locked[MXC_WDT_GET_IDX(wdt)]) {
        return E_NO_ERROR; // Return no error so that Init doesn't failed if called after this function
    }

    const uint8_t clock_source_num = 8;
    uint8_t idx = 0;
    uint8_t instance = 0;
    mxc_wdt_clock_t clock_sources[2][8] = {
        { MXC_WDT_PCLK, MXC_WDT_IBRO_CLK, MXC_WDT_NO_CLK, MXC_WDT_NO_CLK, MXC_WDT_NO_CLK,
          MXC_WDT_NO_CLK, MXC_WDT_NO_CLK, MXC_WDT_NO_CLK },
        { MXC_WDT_IBRO_CLK, MXC_WDT_INRO_CLK, MXC_WDT_ERTCO_CLK, MXC_WDT_NO_CLK, MXC_WDT_NO_CLK,
          MXC_WDT_NO_CLK, MXC_WDT_NO_CLK, MXC_WDT_NO_CLK }
    };

    if (wdt == MXC_WDT0) {
        instance = 0;
    } else if (wdt == MXC_WDT1) {
        instance = 1;
    } else {
        return E_BAD_PARAM;
    }

    for (idx = 0; idx < clock_source_num; idx++) {
        if (clock_sources[instance][idx] == clock_source) {
            break;
        }
    }

    MXC_WDT_RevB_SetClockSource((mxc_wdt_revb_regs_t *)wdt, idx);

    return E_NO_ERROR;
}

int MXC_WDT_LockClockSource(mxc_wdt_regs_t *wdt, bool lock)
{
    g_is_clock_locked[MXC_WDT_GET_IDX(wdt)] = lock;
    return E_NO_ERROR;
}
