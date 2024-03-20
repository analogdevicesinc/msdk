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
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "wdt_revb.h"

/* **** Functions **** */

int MXC_WDT_RevB_Init(mxc_wdt_revb_regs_t *wdt, mxc_wdt_revb_cfg_t *cfg)
{
    if (wdt == NULL || cfg == NULL) {
        return E_NULL_PTR;
    }

    if (cfg->mode & MXC_WDT_REVB_WINDOWED) {
        wdt->ctrl |= MXC_F_WDT_REVB_CTRL_WIN_EN;
    } else {
        wdt->ctrl &= ~(MXC_F_WDT_REVB_CTRL_WIN_EN);
    }

    return E_NO_ERROR;
}

void MXC_WDT_RevB_SetIntPeriod(mxc_wdt_revb_regs_t *wdt, mxc_wdt_revb_cfg_t *cfg)
{
    MXC_SETFIELD(wdt->ctrl, MXC_F_WDT_REVB_CTRL_INT_LATE_VAL, cfg->upperIntPeriod);

    if (cfg->mode & MXC_WDT_REVB_WINDOWED) {
        MXC_SETFIELD(wdt->ctrl, MXC_F_WDT_REVB_CTRL_INT_EARLY_VAL,
                     (cfg->lowerIntPeriod << MXC_F_WDT_REVB_CTRL_INT_EARLY_VAL_POS));
    }
}

void MXC_WDT_RevB_SetResetPeriod(mxc_wdt_revb_regs_t *wdt, mxc_wdt_revb_cfg_t *cfg)
{
    MXC_SETFIELD(wdt->ctrl, MXC_F_WDT_REVB_CTRL_RST_LATE_VAL,
                 (cfg->upperResetPeriod << MXC_F_WDT_REVB_CTRL_RST_LATE_VAL_POS));

    if (cfg->mode & MXC_WDT_REVB_WINDOWED) {
        MXC_SETFIELD(wdt->ctrl, MXC_F_WDT_REVB_CTRL_RST_EARLY_VAL,
                     (cfg->lowerResetPeriod << MXC_F_WDT_REVB_CTRL_RST_EARLY_VAL_POS));
    }
}

void MXC_WDT_RevB_Enable(mxc_wdt_revb_regs_t *wdt)
{
    wdt->rst = 0xFE; // Feed seuqence chips
    wdt->rst = 0xED;
    wdt->ctrl |= MXC_F_WDT_REVB_CTRL_EN; // Direct write chips
}

void MXC_WDT_RevB_Disable(mxc_wdt_revb_regs_t *wdt)
{
    wdt->rst = 0xDE; // Feed sequence chips
    wdt->rst = 0xAD;
    wdt->ctrl &= ~(MXC_F_WDT_REVB_CTRL_EN); // Direct write chips
}

void MXC_WDT_RevB_EnableInt(mxc_wdt_revb_regs_t *wdt, mxc_wdt_revb_en_t enable)
{
    if (enable) {
        wdt->ctrl |= MXC_F_WDT_REVB_CTRL_WDT_INT_EN;
    } else {
        wdt->ctrl &= ~(MXC_F_WDT_REVB_CTRL_WDT_INT_EN);
    }
}

void MXC_WDT_RevB_EnableReset(mxc_wdt_revb_regs_t *wdt, mxc_wdt_revb_en_t enable)
{
    if (enable) {
        wdt->ctrl |= MXC_F_WDT_REVB_CTRL_WDT_RST_EN;
    } else {
        wdt->ctrl &= ~(MXC_F_WDT_REVB_CTRL_WDT_RST_EN);
    }
}

void MXC_WDT_RevB_ResetTimer(mxc_wdt_revb_regs_t *wdt)
{
    wdt->rst = 0x00A5;
    wdt->rst = 0x005A;
}

int MXC_WDT_RevB_GetResetFlag(mxc_wdt_revb_regs_t *wdt)
{
    return (wdt->ctrl & (MXC_F_WDT_REVB_CTRL_RST_LATE | MXC_F_WDT_REVB_CTRL_RST_EARLY));
}

void MXC_WDT_RevB_ClearResetFlag(mxc_wdt_revb_regs_t *wdt)
{
    wdt->ctrl &= ~(MXC_F_WDT_REVB_CTRL_RST_LATE | MXC_F_WDT_REVB_CTRL_RST_EARLY);
}

int MXC_WDT_RevB_GetIntFlag(mxc_wdt_revb_regs_t *wdt)
{
    return !!(wdt->ctrl & (MXC_F_WDT_REVB_CTRL_INT_LATE | MXC_F_WDT_REVB_CTRL_INT_EARLY));
}

void MXC_WDT_RevB_ClearIntFlag(mxc_wdt_revb_regs_t *wdt)
{
    wdt->ctrl &= ~(MXC_F_WDT_REVB_CTRL_INT_LATE | MXC_F_WDT_REVB_CTRL_INT_EARLY);
}

void MXC_WDT_RevB_SetClockSource(mxc_wdt_revb_regs_t *wdt, int clock_source)
{
    const uint8_t clock_source_num = 8; // Max number of clock sources for Rev B WDT
    (void)clock_source_num;

    MXC_ASSERT((clock_source < clock_source_num) && (clock_source >= 0));
    MXC_SETFIELD(wdt->clksel, MXC_F_WDT_REVB_CLKSEL_SOURCE,
                 (clock_source << MXC_F_WDT_REVB_CLKSEL_SOURCE_POS));
}
