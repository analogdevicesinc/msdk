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

#include "tmr.h"
#include "tmr_revb.h"
#include "tmr_common.h"
#include "lpgcr_regs.h"
#include "stdbool.h"

int MXC_TMR_Init(mxc_tmr_regs_t *tmr, mxc_tmr_cfg_t *cfg, bool init_pins)
{
    uint8_t tmr_id;
    uint8_t clockSource = MXC_TMR_CLK0;

    if (cfg == NULL) {
        return E_NULL_PTR;
    }

    tmr_id = MXC_TMR_GET_IDX(tmr);
    MXC_ASSERT(tmr_id >= 0);

    switch (cfg->clock) {
    case MXC_TMR_60M_CLK:
        if (tmr_id > 3) { // Timers 4-5 do not support this clock source
            return E_NOT_SUPPORTED;
        }

        clockSource = MXC_TMR_CLK1;
        MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_ISO);
        MXC_TMR_RevB_SetClockSourceFreq((mxc_tmr_revb_regs_t *)tmr, ISO_FREQ);
        break;

    case MXC_TMR_8M_CLK:
        if (tmr_id > 3) { // Timers 4-5 do not support this clock source
            return E_NOT_SUPPORTED;
        }

        clockSource = MXC_TMR_CLK2;
        MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_IBRO);
        MXC_TMR_RevB_SetClockSourceFreq((mxc_tmr_revb_regs_t *)tmr, IBRO_FREQ);
        break;

    case MXC_TMR_32K_CLK:
        if (tmr_id == 4) {
            clockSource = MXC_TMR_CLK1;
        } else if (tmr_id < 4) {
            clockSource = MXC_TMR_CLK3;
        } else { // Timers 5 do not support this clock source
            return E_NOT_SUPPORTED;
        }

        MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_ERTCO);
        MXC_TMR_RevB_SetClockSourceFreq((mxc_tmr_revb_regs_t *)tmr, ERTCO_FREQ);
        break;

    case MXC_TMR_8K_CLK:
        if (tmr_id < 4) { // Timers 0-3 do not support this clock source
            return E_NOT_SUPPORTED;
        }

        clockSource = MXC_TMR_CLK2;
        MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_INRO);
        MXC_TMR_RevB_SetClockSourceFreq((mxc_tmr_revb_regs_t *)tmr, INRO_FREQ);
        break;

    default:
        MXC_TMR_RevB_SetClockSourceFreq((mxc_tmr_revb_regs_t *)tmr, PeripheralClock);
        break;
    }

    MXC_TMR_RevB_SetClockSource((mxc_tmr_revb_regs_t *)tmr, bit_mode, clk_src);
    return clockSource;
    /*  Note this function returns the actual value of the clockSource field to set because 
        our TMR API was written extremely jankily.  The Init function accepts this register value
        as an argument, so we need to pass that back up from this API... */
}

void MXC_TMR_SetPrescalar(mxc_tmr_regs_t *tmr, mxc_tmr_bit_mode_t bit_mode,
                          mxc_tmr_pres_t prescalar)
{
    MXC_TMR_RevB_SetPrescalar((mxc_tmr_revb_regs_t *)tmr, bit_mode, prescalar);
}

void MXC_TMR_Shutdown(mxc_tmr_regs_t *tmr)
{
    MXC_ASSERT(MXC_TMR_GET_IDX(tmr) >= 0);

    MXC_TMR_RevB_Shutdown((mxc_tmr_revb_regs_t *)tmr);

    // System settigns
    //diasble peripheral clock
    if (tmr == MXC_TMR0) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TMR0);
    }

    if (tmr == MXC_TMR1) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TMR1);
    }

    if (tmr == MXC_TMR2) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TMR2);
    }

    if (tmr == MXC_TMR3) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TMR3);
    }

    if (tmr == MXC_TMR4) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TMR4);
    }

    if (tmr == MXC_TMR5) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TMR5);
    }
}

void MXC_TMR_Start(mxc_tmr_regs_t *tmr)
{
    MXC_TMR_RevB_Start((mxc_tmr_revb_regs_t *)tmr);
}

void MXC_TMR_Stop(mxc_tmr_regs_t *tmr)
{
    MXC_TMR_RevB_Stop((mxc_tmr_revb_regs_t *)tmr);
}

int MXC_TMR_SetPWM(mxc_tmr_regs_t *tmr, uint32_t pwm)
{
    return MXC_TMR_RevB_SetPWM((mxc_tmr_revb_regs_t *)tmr, pwm);
}

uint32_t MXC_TMR_GetCompare(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_RevB_GetCompare((mxc_tmr_revb_regs_t *)tmr);
}

uint32_t MXC_TMR_GetCapture(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_RevB_GetCapture((mxc_tmr_revb_regs_t *)tmr);
}

uint32_t MXC_TMR_GetPeriod(mxc_tmr_regs_t *tmr, mxc_tmr_clock_t clock, uint32_t prescalar,
                           uint32_t frequency)
{
    uint32_t clockFrequency = PeripheralClock;
    uint8_t tmr_id = MXC_TMR_GET_IDX(tmr);

    MXC_ASSERT(tmr_id >= 0);

    if (tmr_id > 3) {
        switch (clock) {
        case MXC_TMR_APB_CLK:
            clockFrequency = IBRO_FREQ;
            break;

        case MXC_TMR_32K_CLK:
            clockFrequency = ERTCO_FREQ;
            break;

        case MXC_TMR_8K_CLK:
            clockFrequency = INRO_FREQ;
            break;

        default:
            break;
        }
    } else {
        switch (clock) {
        case MXC_TMR_APB_CLK:
            clockFrequency = PeripheralClock;
            break;

        case MXC_TMR_60M_CLK:
            clockFrequency = ISO_FREQ;
            break;

        case MXC_TMR_8M_CLK:
            clockFrequency = IBRO_FREQ;
            break;

        case MXC_TMR_32K_CLK:
            clockFrequency = ERTCO_FREQ;
            break;

        default:
            break;
        }
    }

    return MXC_TMR_RevB_GetPeriod((mxc_tmr_revb_regs_t *)tmr, clockFrequency, prescalar, frequency);
}

uint32_t MXC_TMR_GetCount(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_RevB_GetCount((mxc_tmr_revb_regs_t *)tmr);
}

void MXC_TMR_ClearFlags(mxc_tmr_regs_t *tmr)
{
    MXC_TMR_RevB_ClearFlags((mxc_tmr_revb_regs_t *)tmr);
}

uint32_t MXC_TMR_GetFlags(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_RevB_GetFlags((mxc_tmr_revb_regs_t *)tmr);
}

void MXC_TMR_EnableInt(mxc_tmr_regs_t *tmr)
{
    MXC_TMR_RevB_EnableInt((mxc_tmr_revb_regs_t *)tmr);
}

void MXC_TMR_DisableInt(mxc_tmr_regs_t *tmr)
{
    MXC_TMR_RevB_DisableInt((mxc_tmr_revb_regs_t *)tmr);
}

void MXC_TMR_EnableWakeup(mxc_tmr_regs_t *tmr, mxc_tmr_cfg_t *cfg)
{
    MXC_TMR_RevB_EnableWakeup((mxc_tmr_revb_regs_t *)tmr, cfg);
}

void MXC_TMR_DisableWakeup(mxc_tmr_regs_t *tmr, mxc_tmr_cfg_t *cfg)
{
    MXC_TMR_RevB_DisableWakeup((mxc_tmr_revb_regs_t *)tmr, cfg);
}

void MXC_TMR_SetCompare(mxc_tmr_regs_t *tmr, uint32_t cmp_cnt)
{
    MXC_TMR_RevB_SetCompare((mxc_tmr_revb_regs_t *)tmr, cmp_cnt);
}

void MXC_TMR_SetCount(mxc_tmr_regs_t *tmr, uint32_t cnt)
{
    MXC_TMR_RevB_SetCount((mxc_tmr_revb_regs_t *)tmr, cnt);
}

void MXC_TMR_Delay(mxc_tmr_regs_t *tmr, uint32_t us)
{
    MXC_TMR_Common_Delay(tmr, us);
}

void MXC_TMR_TO_Start(mxc_tmr_regs_t *tmr, uint32_t us)
{
    MXC_TMR_RevB_TO_Start((mxc_tmr_revb_regs_t *)tmr, us);
}

int MXC_TMR_TO_Check(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_Common_TO_Check(tmr);
}

void MXC_TMR_TO_Stop(mxc_tmr_regs_t *tmr)
{
    MXC_TMR_Common_TO_Stop(tmr);
}

void MXC_TMR_TO_Clear(mxc_tmr_regs_t *tmr)
{
    MXC_TMR_Common_TO_Clear(tmr);
}

unsigned int MXC_TMR_TO_Elapsed(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_Common_TO_Elapsed(tmr);
}

unsigned int MXC_TMR_TO_Remaining(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_Common_TO_Remaining(tmr);
}

void MXC_TMR_SW_Start(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_Common_SW_Start(tmr);
}

unsigned int MXC_TMR_SW_Stop(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_Common_SW_Stop(tmr);
}

int MXC_TMR_GetTime(mxc_tmr_regs_t *tmr, uint32_t ticks, uint32_t *time, mxc_tmr_unit_t *units)
{
    return MXC_TMR_RevB_GetTime((mxc_tmr_revb_regs_t *)tmr, ticks, time, units);
}

int MXC_TMR_GetTicks(mxc_tmr_regs_t *tmr, uint32_t time, mxc_tmr_unit_t units, uint32_t *ticks)
{
    return MXC_TMR_RevB_GetTicks((mxc_tmr_revb_regs_t *)tmr, time, units, ticks);
}
