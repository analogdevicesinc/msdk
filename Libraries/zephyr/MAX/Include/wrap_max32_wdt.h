/******************************************************************************
 *
 * Copyright (C) 2023 Analog Devices, Inc.
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

#ifndef LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_WDT_H_
#define LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_WDT_H_

/***** Includes *****/
#include <wdt.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char mode; ///< WDT mode
    mxc_wdt_period_t upperResetPeriod; ///< Reset upper limit
    mxc_wdt_period_t lowerResetPeriod; ///< Reset lower limit
    mxc_wdt_period_t upperIntPeriod; ///< Interrupt upper limit
    mxc_wdt_period_t lowerIntPeriod; ///< Interrupt lower limit
} wrap_mxc_wdt_cfg_t;

/*
 *  MAX32665, MAX32666 related mapping
 */
#if defined(CONFIG_SOC_MAX32665) || (CONFIG_SOC_MAX32666)

#define WRAP_MXC_F_WDT_CTRL_EN MXC_F_WDT_CTRL_WDT_EN

typedef enum {
    MXC_WDT_COMPATIBILITY = 0,
    MXC_WDT_WINDOWED = 1, /* Not support on MAX32665/6, added here to get common zephyr driver */
} mxc_wdt_mode_t;

static inline int Wrap_MXC_WDT_Init(mxc_wdt_regs_t *wdt, wrap_mxc_wdt_cfg_t *cfg)
{
    int ret;

    if (cfg && ((mxc_wdt_mode_t)cfg->mode == MXC_WDT_COMPATIBILITY)) {
        /* only support compatibility mode */
        ret = MXC_WDT_Init(wdt);
    } else {
        ret = -1;
    }

    return ret;
}

static inline void Wrap_MXC_WDT_SetResetPeriod(mxc_wdt_regs_t *wdt, wrap_mxc_wdt_cfg_t *cfg)
{
    (void)cfg;

    MXC_WDT_SetResetPeriod(wdt, cfg->upperResetPeriod);
}

static inline void Wrap_MXC_WDT_SetIntPeriod(mxc_wdt_regs_t *wdt, wrap_mxc_wdt_cfg_t *cfg)
{
    return MXC_WDT_SetIntPeriod(wdt, cfg->upperIntPeriod);
}

static inline int Wrap_MXC_WDT_SelectClockSource(mxc_wdt_regs_t *wdt, uint32_t clock_src)
{
    (void)wdt;
    (void)clock_src;

    return 0;
}

/*
 *  MAX32690, MAX32655 related mapping
 */
#elif defined(CONFIG_SOC_MAX32690) || (CONFIG_SOC_MAX32655) || (CONFIG_SOC_MAX32670) || \
    (CONFIG_SOC_MAX32672) || (CONFIG_SOC_MAX32662) || (CONFIG_SOC_MAX32675) ||          \
    (CONFIG_SOC_MAX32680)

#define WRAP_MXC_F_WDT_CTRL_EN MXC_F_WDT_CTRL_EN

static inline int Wrap_MXC_WDT_Init(mxc_wdt_regs_t *wdt, wrap_mxc_wdt_cfg_t *cfg)
{
    mxc_wdt_cfg_t mxc_cfg;

    mxc_cfg.mode = (mxc_wdt_mode_t)cfg->mode;
    mxc_cfg.upperResetPeriod = cfg->upperResetPeriod;
    mxc_cfg.lowerResetPeriod = cfg->lowerResetPeriod;
    mxc_cfg.upperIntPeriod = cfg->upperIntPeriod;
    mxc_cfg.lowerIntPeriod = cfg->lowerIntPeriod;

    return MXC_WDT_Init(wdt, &mxc_cfg);
}

static inline void Wrap_MXC_WDT_SetResetPeriod(mxc_wdt_regs_t *wdt, wrap_mxc_wdt_cfg_t *cfg)
{
    MXC_WDT_SetResetPeriod(wdt, (mxc_wdt_cfg_t *)cfg);
}

static inline void Wrap_MXC_WDT_SetIntPeriod(mxc_wdt_regs_t *wdt, wrap_mxc_wdt_cfg_t *cfg)
{
    mxc_wdt_cfg_t mxc_cfg;

    mxc_cfg.mode = (mxc_wdt_mode_t)cfg->mode;
    mxc_cfg.upperResetPeriod = cfg->upperResetPeriod;
    mxc_cfg.lowerResetPeriod = cfg->lowerResetPeriod;
    mxc_cfg.upperIntPeriod = cfg->upperIntPeriod;
    mxc_cfg.lowerIntPeriod = cfg->lowerIntPeriod;

    return MXC_WDT_SetIntPeriod(wdt, &mxc_cfg);
}

static inline int Wrap_MXC_WDT_SelectClockSource(mxc_wdt_regs_t *wdt, uint32_t clock_src)
{
    mxc_wdt_clock_t clk_src;

    switch (clock_src) {
    case 0: // ADI_MAX32_PRPH_CLK_SRC_PCLK
        clk_src = MXC_WDT_PCLK;
        break;
    case 2: // ADI_MAX32_PRPH_CLK_SRC_IBRO
        clk_src = MXC_WDT_IBRO_CLK;
        break;
    case 5: // ADI_MAX32_PRPH_CLK_SRC_INRO
#if defined(CONFIG_SOC_MAX32662)
        clk_src = MXC_WDT_NANO_CLK;
#else
        clk_src = MXC_WDT_INRO_CLK;
#endif
        break;
#if !(defined(CONFIG_SOC_MAX32675) || (CONFIG_SOC_MAX32680))
    case 4: // ADI_MAX32_PRPH_CLK_SRC_ERTCO
        clk_src = MXC_WDT_ERTCO_CLK;
        break;
#endif
    default:
        return -1;
    }

    return MXC_WDT_SetClockSource(wdt, clk_src);
}

#endif // part number

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_WDT_H_
