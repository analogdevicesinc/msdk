/******************************************************************************
 *
 * Copyright (C) 2023-2026 Analog Devices, Inc.
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

#ifndef LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_TMR_H_
#define LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_TMR_H_

/***** Includes *****/
#include <tmr.h>
#include <lp.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    mxc_tmr_pres_t pres;
    mxc_tmr_mode_t mode;
    int bitMode; /* Some PN does not support it, check mxc_tmr_bit_mode_t in tmr.h file */
    int clock; /* Some PN does not support it, check mxc_tmr_clock_t in tmr.h file */
    uint32_t cmp_cnt; /**< Compare register value in timer ticks */
    unsigned pol; /**< Polarity (0 or 1) */
} wrap_mxc_tmr_cfg_t;

/*
 *  MAX32665, MAX32666 related mapping
 */
#if defined(CONFIG_SOC_MAX32665) || defined(CONFIG_SOC_MAX32666) || defined(CONFIG_SOC_MAX32650)

/* All timers are 32bits */
#define WRAP_MXC_IS_32B_TIMER(idx) (1)

/* Prescaler lookup table range for MAX32650/665/666
 * These devices require special register mapping for prescaler values > 128
 */
#define TMR_PRES_MIN_LOG2 8 /* LOG2(256) - minimum extended prescaler */
#define TMR_PRES_MAX_LOG2 12 /* LOG2(4096) - maximum extended prescaler */

static inline int Wrap_MXC_TMR_Init(mxc_tmr_regs_t *tmr, wrap_mxc_tmr_cfg_t *cfg)
{
    mxc_tmr_pres_t tmr_prescaler_lut[] = { TMR_PRES_256, TMR_PRES_512, TMR_PRES_1024, TMR_PRES_2048,
                                           TMR_PRES_4096 };
    mxc_tmr_cfg_t mxc_cfg;
    uint32_t pres_log2;

    mxc_cfg.mode = cfg->mode;
    mxc_cfg.cmp_cnt = cfg->cmp_cnt;
    mxc_cfg.pol = cfg->pol;

    if (cfg->pres <= TMR_PRES_128) {
        mxc_cfg.pres = cfg->pres;
    } else {
        /* cfg->pres is equal to (LOG2(prescaler) * TMR_PRES_2) */
        pres_log2 = cfg->pres / TMR_PRES_2;

        /* Limit prescaler to valid range (TMR_PRES_256 to TMR_PRES_4096) */
        if (pres_log2 < TMR_PRES_MIN_LOG2 || pres_log2 > TMR_PRES_MAX_LOG2) {
            return -1;
        }
        mxc_cfg.pres = tmr_prescaler_lut[pres_log2 - TMR_PRES_MIN_LOG2];
    }

    MXC_TMR_Init(tmr, &mxc_cfg);
    return 0;
}

static inline int Wrap_MXC_TMR_GetClockIndex(int z_clock)
{
    if (z_clock == 0) {
        /* Only peripheral clock is supported, just return 0 */
        return 0;
    } else {
        return -1; /* Not supported */
    }
}

static inline void Wrap_MXC_TMR_EnableWakeup(mxc_tmr_regs_t *tmr, wrap_mxc_tmr_cfg_t *cfg)
{
    (void)tmr;
    (void)cfg;
}

static inline void Wrap_MXC_TMR_ClearWakeupFlags(mxc_tmr_regs_t *tmr)
{
    (void)tmr;
}

static inline void Wrap_MXC_TMR_DisableInt(mxc_tmr_regs_t *tmr)
{
    (void)tmr;
}

static inline void Wrap_MXC_TMR_EnableInt(mxc_tmr_regs_t *tmr)
{
    (void)tmr;
}

static inline int Wrap_MXC_TMR_GetPendingInt(mxc_tmr_regs_t *tmr)
{
    uint32_t mask = MXC_F_TMR_INTR_IRQ;
    uint32_t flags;

    flags = MXC_TMR_GetFlags(tmr);

    return ((flags & mask) == mask);
}

/*
 *  MAX32690, MAX32655 related mapping
 */
#elif defined(CONFIG_SOC_MAX32690) || defined(CONFIG_SOC_MAX32655) || \
    defined(CONFIG_SOC_MAX32670) || defined(CONFIG_SOC_MAX32672) ||   \
    defined(CONFIG_SOC_MAX32662) || defined(CONFIG_SOC_MAX32675) ||   \
    defined(CONFIG_SOC_MAX32680) || defined(CONFIG_SOC_MAX32657) ||   \
    defined(CONFIG_SOC_MAX78002) || defined(CONFIG_SOC_MAX78000)

#if defined(CONFIG_SOC_MAX32672) || defined(CONFIG_SOC_MAX32675) || \
    defined(CONFIG_SOC_MAX32657) || defined(CONFIG_SOC_MAX32670)
/* All timers are 32bits */
#define WRAP_MXC_IS_32B_TIMER(idx) (1)
#elif defined(CONFIG_SOC_MAX32662)
#define WRAP_MXC_IS_32B_TIMER(idx) (MXC_TMR_GET_IDX(idx) == 3 ? 0 : 1)
#else
#define WRAP_MXC_IS_32B_TIMER(idx) \
    (MXC_TMR_GET_IDX(idx) == 4 ? 0 : MXC_TMR_GET_IDX(idx) == 5 ? 0 : 1)
#endif

static inline int Wrap_MXC_TMR_Init(mxc_tmr_regs_t *tmr, wrap_mxc_tmr_cfg_t *cfg)
{
    mxc_tmr_cfg_t mxc_cfg;

    mxc_cfg.pres = cfg->pres;
    mxc_cfg.mode = cfg->mode;
    mxc_cfg.cmp_cnt = cfg->cmp_cnt;
    mxc_cfg.pol = cfg->pol;
    mxc_cfg.bitMode = (mxc_tmr_bit_mode_t)cfg->bitMode;
    mxc_cfg.clock = (mxc_tmr_clock_t)cfg->clock;

#if defined(CONFIG_SOC_MAX32662)
    return MXC_TMR_Init(tmr, &mxc_cfg, 0, (sys_map_t)0);
#else
    return MXC_TMR_Init(tmr, &mxc_cfg, 0); // init_pins not used
#endif
}

static inline int Wrap_MXC_TMR_GetClockIndex(int z_clock)
{
    switch (z_clock) {
    case 0: // ADI_MAX32_PRPH_CLK_SRC_PCLK
        return MXC_TMR_APB_CLK;
    case 1: // ADI_MAX32_PRPH_CLK_SRC_EXTCLK
        return MXC_TMR_EXT_CLK;
    case 2: // ADI_MAX32_PRPH_CLK_SRC_IBRO
        return MXC_TMR_8M_CLK;
#if !defined(CONFIG_SOC_MAX78002) && !defined(CONFIG_SOC_MAX78000)
    case 3: //ADI_MAX32_PRPH_CLK_SRC_ERFO
        return MXC_TMR_32M_CLK;
#endif
    case 4: //ADI_MAX32_PRPH_CLK_SRC_ERTCO
        return MXC_TMR_32K_CLK;
    case 5: //ADI_MAX32_PRPH_CLK_SRC_INRO
        return MXC_TMR_INRO_CLK;
#if defined(CONFIG_SOC_MAX32655) || defined(CONFIG_SOC_MAX32680) || \
    defined(CONFIG_SOC_MAX32690) || defined(CONFIG_SOC_MAX78002) || defined(CONFIG_SOC_MAX78000)
    case 6: //ADI_MAX32_PRPH_CLK_SRC_ISO
        return MXC_TMR_ISO_CLK;
#endif
    }

    return -1; /* Not supported */
}

static inline void Wrap_MXC_TMR_EnableWakeup(mxc_tmr_regs_t *tmr, wrap_mxc_tmr_cfg_t *cfg)
{
#if defined(CONFIG_SOC_MAX32657)
    (void)tmr;
    (void)cfg;
#else
    mxc_tmr_cfg_t mxc_cfg;

    mxc_cfg.pres = cfg->pres;
    mxc_cfg.mode = cfg->mode;
    mxc_cfg.cmp_cnt = cfg->cmp_cnt;
    mxc_cfg.pol = cfg->pol;
    mxc_cfg.bitMode = (mxc_tmr_bit_mode_t)cfg->bitMode;
    mxc_cfg.clock = (mxc_tmr_clock_t)cfg->clock;

    // Enable wakeup source in power seq register
    MXC_LP_EnableTimerWakeup(tmr);
    // Enable Timer wake-up source
    MXC_TMR_EnableWakeup(tmr, &mxc_cfg);
#endif
}

static inline void Wrap_MXC_TMR_ClearWakeupFlags(mxc_tmr_regs_t *tmr)
{
    if (tmr->wkfl & MXC_F_TMR_WKFL_A) {
        // Write 1 to clear
        tmr->wkfl |= MXC_F_TMR_WKFL_A;
    }
}

static inline void Wrap_MXC_TMR_DisableInt(mxc_tmr_regs_t *tmr)
{
    MXC_TMR_DisableInt(tmr);
}

static inline void Wrap_MXC_TMR_EnableInt(mxc_tmr_regs_t *tmr)
{
    MXC_TMR_EnableInt(tmr);
}

static inline int Wrap_MXC_TMR_GetPendingInt(mxc_tmr_regs_t *tmr)
{
    uint32_t mask = MXC_F_TMR_INTFL_IRQ_A | MXC_F_TMR_INTFL_IRQ_B;
    uint32_t flags;

    flags = MXC_TMR_GetFlags(tmr);

    return ((flags & mask) == mask);
}

#endif // part number

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_TMR_H_
