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
#include <stdbool.h>
#include "mxc_assert.h"
#include "tmr.h"
#include "tmr_revb.h"
#include "gpio.h"
#include "mxc_pins.h"
#include "mxc_lock.h"

/* **** Definitions **** */
#define TIMER_16A_OFFSET 0
#define TIMER_16B_OFFSET 16

typedef struct {
    bool configured;
    uint32_t freq;
} mxc_tmr_revb_clksrc_freq_t;

static mxc_tmr_revb_clksrc_freq_t tmr_clksrc[MXC_CFG_TMR_INSTANCES];
static bool g_is_clock_locked[MXC_CFG_TMR_INSTANCES] = { [0 ... MXC_CFG_TMR_INSTANCES - 1] =
                                                             false };

/* **** Functions **** */
int MXC_TMR_RevB_Init(mxc_tmr_revb_regs_t *tmr, mxc_tmr_cfg_t *cfg, uint8_t clk_src)
{
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    if (cfg == NULL) {
        return E_NULL_PTR;
    }

    // Default 32 bit timer
    if (cfg->bitMode & (MXC_TMR_BIT_MODE_16A | MXC_TMR_BIT_MODE_16B)) {
        tmr->ctrl1 &= ~MXC_F_TMR_REVB_CTRL1_CASCADE;
    } else {
        tmr->ctrl1 |= MXC_F_TMR_REVB_CTRL1_CASCADE;
    }

    // Clear interrupt flag
    tmr->intfl |= (MXC_F_TMR_REVB_INTFL_IRQ_A | MXC_F_TMR_REVB_INTFL_IRQ_B);

    MXC_TMR_RevB_SetClockSource(tmr, cfg->bitMode, clk_src);
    MXC_TMR_RevB_SetPrescalar(tmr, cfg->bitMode, clk_src);

    //TIMER_16B only supports compare, oneshot and continuous modes.
    switch (cfg->mode) {
    case MXC_TMR_MODE_ONESHOT:
        MXC_TMR_RevB_ConfigGeneric((mxc_tmr_revb_regs_t *)tmr, cfg);
        break;

    case MXC_TMR_MODE_CONTINUOUS:
        MXC_TMR_RevB_ConfigGeneric((mxc_tmr_revb_regs_t *)tmr, cfg);
        break;

    case MXC_TMR_MODE_COUNTER:
        if (cfg->bitMode == MXC_TMR_BIT_MODE_16B) {
            return E_NOT_SUPPORTED;
        }

        MXC_TMR_RevB_ConfigGeneric(tmr, cfg);
        break;

    case MXC_TMR_MODE_CAPTURE:
        if (cfg->bitMode == MXC_TMR_BIT_MODE_16B) {
            return E_NOT_SUPPORTED;
        }

        MXC_TMR_RevB_ConfigGeneric(tmr, cfg);
        break;

    case MXC_TMR_MODE_COMPARE:
        MXC_TMR_RevB_ConfigGeneric((mxc_tmr_revb_regs_t *)tmr, cfg);
        break;

    case MXC_TMR_MODE_GATED:
        if (cfg->bitMode == MXC_TMR_BIT_MODE_16B) {
            return E_NOT_SUPPORTED;
        }

        MXC_TMR_RevB_ConfigGeneric(tmr, cfg);
        break;

    case MXC_TMR_MODE_CAPTURE_COMPARE:
        if (cfg->bitMode == MXC_TMR_BIT_MODE_16B) {
            return E_NOT_SUPPORTED;
        }

        MXC_TMR_RevB_ConfigGeneric(tmr, cfg);
        break;

    case MXC_TMR_MODE_DUAL_EDGE:
        if (cfg->bitMode == MXC_TMR_BIT_MODE_16B) {
            return E_NOT_SUPPORTED;
        }

        MXC_TMR_RevB_ConfigGeneric(tmr, cfg);
        break;

    case MXC_TMR_MODE_PWM:
        if (cfg->bitMode == MXC_TMR_BIT_MODE_16B) {
            return E_NOT_SUPPORTED;
        }

        MXC_TMR_RevB_ConfigGeneric((mxc_tmr_revb_regs_t *)tmr, cfg);
        break;
    }

    return E_NO_ERROR;
}

void MXC_TMR_RevB_LockClockSource(mxc_tmr_revb_regs_t *tmr, bool lock)
{
    g_is_clock_locked[MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr)] = lock;
}

void MXC_TMR_RevB_SetClockSource(mxc_tmr_revb_regs_t *tmr, mxc_tmr_bit_mode_t bit_mode,
                                 uint8_t clk_src)
{
    if (g_is_clock_locked[MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr)])
        return;

    // Select clock Source
    // Note:  For 32-bit cascade mode, TMR A and TMR B clock sources must be
    //        the same to ensure proper operation.  (See MAX32670 UG Rev 4 Section 13.4)
    if (bit_mode == TMR_BIT_MODE_16A || bit_mode == TMR_BIT_MODE_32) {
        MXC_SETFIELD(tmr->ctrl1, MXC_F_TMR_CTRL1_CLKSEL_A,
                     (clk_src << MXC_F_TMR_CTRL1_CLKSEL_A_POS));
    }
    if (bit_mode == TMR_BIT_MODE_16B || bit_mode == TMR_BIT_MODE_32) {
        MXC_SETFIELD(tmr->ctrl1, MXC_F_TMR_CTRL1_CLKSEL_B,
                     (clk_src << MXC_F_TMR_CTRL1_CLKSEL_B_POS));
    }
}

void MXC_TMR_RevB_SetPrescalar(mxc_tmr_revb_regs_t *tmr, mxc_tmr_bit_mode_t bit_mode,
                               mxc_tmr_pres_t prescalar)
{
    // Set prescaler
    // Note:  For 32-bit cascade mode, TMR A and TMR B clock sources must be
    //        the same to ensure proper operation.  (See MAX32670 UG Rev 4 Section 13.4)
    if (bit_mode == TMR_BIT_MODE_16A || bit_mode == TMR_BIT_MODE_32) {
        MXC_SETFIELD(tmr->ctrl0, MXC_F_TMR_CTRL0_CLKDIV_A, prescalar);
    }
    if (bit_mode == TMR_BIT_MODE_16B || bit_mode == TMR_BIT_MODE_32) {
        // mxc_tmr_pres_t is for for CLKDIV_A register settings [7:4]
        // Field positions for CLKDIV_B are Located at [23:20]. Shift 16 more bits.
        MXC_SETFIELD(tmr->ctrl0, MXC_F_TMR_CTRL0_CLKDIV_B, (prescalar) << 16);
    }
}

void MXC_TMR_RevB_SetClockSourceFreq(mxc_tmr_revb_regs_t *tmr, int clksrc_freq)
{
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    tmr_clksrc[tmr_id].configured = true;
    tmr_clksrc[tmr_id].freq = clksrc_freq;
}

int MXC_TMR_RevB_GetClockSourceFreq(mxc_tmr_revb_regs_t *tmr)
{
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    if (tmr_clksrc[tmr_id].configured == false) {
        return E_BAD_STATE;
    }

    return tmr_clksrc[tmr_id].freq;
}

void MXC_TMR_RevB_ConfigGeneric(mxc_tmr_revb_regs_t *tmr, mxc_tmr_cfg_t *cfg)
{
    uint32_t timerOffset;
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    if (cfg == NULL) {
        return;
    }

    if (cfg->bitMode == MXC_TMR_BIT_MODE_16B) {
        timerOffset = TIMER_16B_OFFSET;
    } else {
        timerOffset = TIMER_16A_OFFSET;
    }

    tmr->ctrl0 |= (MXC_F_TMR_REVB_CTRL0_CLKEN_A << timerOffset);
    while (!(tmr->ctrl1 & (MXC_F_TMR_REVB_CTRL1_CLKRDY_A << timerOffset))) {}

    tmr->ctrl0 |= (cfg->mode << timerOffset);
    tmr->ctrl0 |= ((cfg->pol << MXC_F_TMR_REVB_CTRL0_POL_A_POS) << timerOffset);
    //enable timer interrupt if needed
    tmr->cnt = (0x1 << timerOffset);
    while (!(tmr->intfl & (MXC_F_TMR_REVB_INTFL_WRDONE_A << timerOffset))) {}

    tmr->cmp = (cfg->cmp_cnt << timerOffset);
#if TARGET_NUM == 32655 || TARGET_NUM == 78000 || TARGET_NUM == 32690 || TARGET_NUM == 78002
    tmr->ctrl1 &= ~(MXC_F_TMR_REVB_CTRL1_OUTEN_A << timerOffset);
#else
    tmr->ctrl1 |= (MXC_F_TMR_REVB_CTRL1_OUTEN_A << timerOffset);
#endif
}

void MXC_TMR_RevB_Shutdown(mxc_tmr_revb_regs_t *tmr)
{
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    // Stop timer before disable it.
    MXC_TMR_RevB_Stop(tmr);
    // Disable timer and clear settings
    tmr->ctrl0 = 0;
    while (tmr->ctrl1 & MXC_F_TMR_REVB_CTRL1_CLKRDY_A) {}
    tmr_clksrc[tmr_id].configured = false;
}

void MXC_TMR_RevB_Start(mxc_tmr_revb_regs_t *tmr)
{
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    /* If a timer's clk is enabled, it's a reliable signal that the
    clock itself is configured and we should start it.  This check is
    relevant for dual-mode timer configs
    */

    if (tmr->ctrl0 & MXC_F_TMR_CTRL0_CLKEN_A) {
        tmr->ctrl0 |= MXC_F_TMR_REVB_CTRL0_EN_A;
        while (!(tmr->ctrl1 & MXC_F_TMR_REVB_CTRL1_CLKEN_A)) {}
    }

    if (tmr->ctrl0 & MXC_F_TMR_CTRL0_CLKEN_B) {
        tmr->ctrl0 |= MXC_F_TMR_REVB_CTRL0_EN_B;
        while (!(tmr->ctrl1 & MXC_F_TMR_REVB_CTRL1_CLKEN_B)) {}
    }
}

void MXC_TMR_RevB_Stop(mxc_tmr_revb_regs_t *tmr)
{
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    // Will stop both timers in a dual-mode config
    tmr->ctrl0 &= ~(MXC_F_TMR_REVB_CTRL0_EN_A | MXC_F_TMR_REVB_CTRL0_EN_B);
}

int MXC_TMR_RevB_SetPWM(mxc_tmr_revb_regs_t *tmr, uint32_t pwm)
{
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    if (pwm > (tmr->cmp)) {
        return E_BAD_PARAM;
    }

    bool timera_is_running = tmr->ctrl0 & MXC_F_TMR_CTRL0_EN_A;
    bool timerb_is_running = tmr->ctrl0 & MXC_F_TMR_CTRL0_EN_B;

    if (timera_is_running || timerb_is_running) {
        MXC_TMR_RevB_ClearFlags(tmr); // Clear flags so we can catch the next one
        while (!MXC_TMR_RevB_GetFlags(tmr)) {} // Wait for next PWM transition
        MXC_TMR_RevB_Stop(tmr); // Pause timer
        MXC_TMR_RevB_SetCount(tmr, 0); // Reset the count
        MXC_TMR_RevB_ClearFlags(
            tmr); // Clear flags since app code wants the new PWM transitions set by this function
    }

    tmr->pwm = pwm;
    while (!(tmr->intfl & MXC_F_TMR_REVB_INTFL_WRDONE_A)) {}

    if (timera_is_running) {
        tmr->ctrl0 |= MXC_F_TMR_REVB_CTRL0_EN_A; // Unpause A
    }

    if (timerb_is_running) {
        tmr->ctrl0 |= MXC_F_TMR_REVB_CTRL0_EN_B; // Unpause B
    }

    return E_NO_ERROR;
}

uint32_t MXC_TMR_RevB_GetCompare(mxc_tmr_revb_regs_t *tmr)
{
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    return tmr->cmp;
}

uint32_t MXC_TMR_RevB_GetCapture(mxc_tmr_revb_regs_t *tmr)
{
    uint32_t pwm;
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    // read pwm register twice
    pwm = tmr->pwm;
    pwm = tmr->pwm;
    return pwm;
}

uint32_t MXC_TMR_RevB_GetCount(mxc_tmr_revb_regs_t *tmr)
{
    uint32_t cnt;
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    // read cnt register twice
    cnt = tmr->cnt;
    cnt = tmr->cnt;
    return cnt;
}

uint32_t MXC_TMR_RevB_GetPeriod(mxc_tmr_revb_regs_t *tmr, uint32_t clk_frequency,
                                uint32_t prescalar, uint32_t frequency)
{
    uint32_t periodTicks;
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    periodTicks = clk_frequency / (frequency * prescalar);

    return periodTicks;
}

void MXC_TMR_RevB_ClearFlags(mxc_tmr_revb_regs_t *tmr)
{
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    tmr->intfl |= (MXC_F_TMR_REVB_INTFL_IRQ_A | MXC_F_TMR_REVB_INTFL_IRQ_B);
}

uint32_t MXC_TMR_RevB_GetFlags(mxc_tmr_revb_regs_t *tmr)
{
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    return (tmr->intfl & (MXC_F_TMR_REVB_INTFL_IRQ_A | MXC_F_TMR_REVB_INTFL_IRQ_B));
}

void MXC_TMR_RevB_EnableInt(mxc_tmr_revb_regs_t *tmr)
{
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    tmr->ctrl1 |= MXC_F_TMR_REVB_CTRL1_IE_A | MXC_F_TMR_REVB_CTRL1_IE_B;
}

void MXC_TMR_RevB_DisableInt(mxc_tmr_revb_regs_t *tmr)
{
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    tmr->ctrl1 &= ~(MXC_F_TMR_REVB_CTRL1_IE_A | MXC_F_TMR_REVB_CTRL1_IE_B);
}

void MXC_TMR_RevB_EnableWakeup(mxc_tmr_revb_regs_t *tmr, mxc_tmr_cfg_t *cfg)
{
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    // Enable Timer wake-up source
    if (cfg->bitMode == MXC_TMR_BIT_MODE_16B) {
        tmr->ctrl1 |= MXC_F_TMR_REVB_CTRL1_WE_B;
    } else {
        tmr->ctrl1 |= MXC_F_TMR_REVB_CTRL1_WE_A;
    }
}

void MXC_TMR_RevB_DisableWakeup(mxc_tmr_revb_regs_t *tmr, mxc_tmr_cfg_t *cfg)
{
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    // Disable Timer wake-up source
    if (cfg->bitMode == MXC_TMR_BIT_MODE_16B) {
        tmr->ctrl1 &= ~MXC_F_TMR_REVB_CTRL1_WE_B;
    } else {
        tmr->ctrl1 &= ~MXC_F_TMR_REVB_CTRL1_WE_A;
    }
}

void MXC_TMR_RevB_SetCompare(mxc_tmr_revb_regs_t *tmr, uint32_t cmp_cnt)
{
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    tmr->cmp = cmp_cnt;
}

void MXC_TMR_RevB_SetCount(mxc_tmr_revb_regs_t *tmr, uint32_t cnt)
{
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    tmr->cnt = cnt;
    while (!(tmr->intfl & MXC_F_TMR_REVB_INTFL_WRDONE_A)) {}
}

void MXC_TMR_RevB_TO_Start(mxc_tmr_revb_regs_t *tmr, uint32_t us)
{
    uint64_t ticks;
    int clk_shift = 0;

    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    if (us == 0) {
        return;
    }

    ticks = (uint64_t)us * (uint64_t)PeripheralClock / (uint64_t)1000000;

    while (ticks > 0xFFFFFFFFUL) {
        ticks >>= 1;
        ++clk_shift;
    }

    mxc_tmr_pres_t prescale = (mxc_tmr_pres_t)(clk_shift << MXC_F_TMR_REVB_CTRL0_CLKDIV_A_POS);
    mxc_tmr_cfg_t cfg;

    // Initialize the timer in one-shot mode
    cfg.pres = prescale;
    cfg.mode = MXC_TMR_MODE_ONESHOT;
    cfg.bitMode = MXC_TMR_BIT_MODE_32;
    cfg.clock = MXC_TMR_APB_CLK;
    cfg.cmp_cnt = ticks;
    cfg.pol = 0;

    MXC_TMR_Stop((mxc_tmr_regs_t *)tmr);
#if TARGET_NUM == 32662
    MXC_TMR_Init((mxc_tmr_regs_t *)tmr, &cfg, false, MAP_A);
#else
    MXC_TMR_Init((mxc_tmr_regs_t *)tmr, &cfg, false);
#endif
    tmr->ctrl1 |= MXC_F_TMR_REVB_CTRL1_CASCADE;
    MXC_TMR_ClearFlags((mxc_tmr_regs_t *)tmr);
    MXC_TMR_Start((mxc_tmr_regs_t *)tmr);
}

int MXC_TMR_RevB_GetTime(mxc_tmr_revb_regs_t *tmr, uint32_t ticks, uint32_t *time,
                         mxc_tmr_unit_t *units)
{
    int tmr_id = MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr);
    (void)tmr_id;
    MXC_ASSERT(tmr_id >= 0);

    uint64_t temp_time = 0;
    uint32_t prescale = (tmr->ctrl0 & MXC_F_TMR_REVB_CTRL0_CLKDIV_A) >>
                        MXC_F_TMR_REVB_CTRL0_CLKDIV_A_POS;

    int timerClock = MXC_TMR_RevB_GetClockSourceFreq(tmr);

    // Confirm clock is configured by checking for error return.
    if (timerClock < 0) {
        return timerClock;
    }

    temp_time = (uint64_t)ticks * 1000 * (1 << (prescale & 0xF)) / (timerClock / 1000000);

    if (!(temp_time & 0xffffffff00000000)) {
        *time = temp_time;
        *units = MXC_TMR_UNIT_NANOSEC;
        return E_NO_ERROR;
    }

    temp_time = (uint64_t)ticks * 1000 * (1 << (prescale & 0xF)) / (timerClock / 1000);

    if (!(temp_time & 0xffffffff00000000)) {
        *time = temp_time;
        *units = MXC_TMR_UNIT_MICROSEC;
        return E_NO_ERROR;
    }

    temp_time = (uint64_t)ticks * 1000 * (1 << (prescale & 0xF)) / timerClock;

    if (!(temp_time & 0xffffffff00000000)) {
        *time = temp_time;
        *units = MXC_TMR_UNIT_MILLISEC;
        return E_NO_ERROR;
    }

    temp_time = (uint64_t)ticks * (1 << (prescale & 0xF)) / timerClock;

    if (!(temp_time & 0xffffffff00000000)) {
        *time = temp_time;
        *units = MXC_TMR_UNIT_SEC;
        return E_NO_ERROR;
    }

    return E_INVALID;
}

int MXC_TMR_RevB_GetTicks(mxc_tmr_revb_regs_t *tmr, uint32_t time, mxc_tmr_unit_t units,
                          uint32_t *ticks)
{
    uint32_t unit_div0, unit_div1;
    uint32_t timerClock;
    uint32_t prescale;
    uint64_t temp_ticks;

    timerClock = PeripheralClock;

    prescale = ((tmr->ctrl0 & MXC_F_TMR_CTRL0_CLKDIV_A) >> MXC_F_TMR_CTRL0_CLKDIV_A_POS);

    switch (units) {
    case MXC_TMR_UNIT_NANOSEC:
        unit_div0 = 1000000;
        unit_div1 = 1000;
        break;
    case MXC_TMR_UNIT_MICROSEC:
        unit_div0 = 1000;
        unit_div1 = 1000;
        break;
    case MXC_TMR_UNIT_MILLISEC:
        unit_div0 = 1;
        unit_div1 = 1000;
        break;
    case MXC_TMR_UNIT_SEC:
        unit_div0 = 1;
        unit_div1 = 1;
        break;
    default:
        return E_BAD_PARAM;
    }

    temp_ticks = (uint64_t)time * (timerClock / unit_div0) / (unit_div1 * (1 << (prescale & 0xF)));

    //make sure ticks is within a 32 bit value
    if (!(temp_ticks & 0xffffffff00000000) && (temp_ticks & 0xffffffff)) {
        *ticks = temp_ticks;
        return E_NO_ERROR;
    }

    return E_INVALID;
}
