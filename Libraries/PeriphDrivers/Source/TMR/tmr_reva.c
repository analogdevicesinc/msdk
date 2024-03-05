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
#include <string.h>
#include "mxc_assert.h"
#include "tmr.h"
#include "tmr_reva.h"
#include "gpio.h"
#include "mxc_pins.h"
#include "mxc_lock.h"

/* **** Functions **** */
void MXC_TMR_RevA_Init(mxc_tmr_reva_regs_t *tmr, mxc_tmr_cfg_t *cfg)
{
    // Clear interrupt flag
    tmr->intr = MXC_F_TMR_REVA_INTR_IRQ;

    // Set the prescaler
    switch (cfg->pres) {
    case MXC_TMR_PRES_1:
        tmr->cn |= (MXC_S_TMR_REVA_CN_PRES_DIV1);
        break;

    case MXC_TMR_PRES_2:
        tmr->cn |= (MXC_S_TMR_REVA_CN_PRES_DIV2);
        break;

    case MXC_TMR_PRES_4:
        tmr->cn |= (MXC_S_TMR_REVA_CN_PRES_DIV4);
        break;

    case MXC_TMR_PRES_8:
        tmr->cn |= (MXC_S_TMR_REVA_CN_PRES_DIV8);
        break;

    case MXC_TMR_PRES_16:
        tmr->cn |= (MXC_S_TMR_REVA_CN_PRES_DIV16);
        break;

    case MXC_TMR_PRES_32:
        tmr->cn |= (MXC_S_TMR_REVA_CN_PRES_DIV32);
        break;

    case MXC_TMR_PRES_64:
        tmr->cn |= (MXC_S_TMR_REVA_CN_PRES_DIV64);
        break;

    case MXC_TMR_PRES_128:
        tmr->cn |= (MXC_S_TMR_REVA_CN_PRES_DIV128);
        break;

    case MXC_TMR_PRES_256:
        tmr->cn |= (MXC_F_TMR_REVA_CN_PRES3);
        tmr->cn &= ~(MXC_S_TMR_REVA_CN_PRES_DIV1);
        break;

    case MXC_TMR_PRES_512:
        tmr->cn |= (MXC_F_TMR_REVA_CN_PRES3);
        tmr->cn |= (MXC_S_TMR_REVA_CN_PRES_DIV4);
        break;

    case MXC_TMR_PRES_1024:
        tmr->cn |= (MXC_F_TMR_REVA_CN_PRES3);
        tmr->cn |= (MXC_S_TMR_REVA_CN_PRES_DIV8);
        break;

    case MXC_TMR_PRES_2048:
        tmr->cn |= (MXC_F_TMR_REVA_CN_PRES3);
        tmr->cn |= (MXC_S_TMR_REVA_CN_PRES_DIV16);
        break;

    case MXC_TMR_PRES_4096:
        tmr->cn |= (MXC_F_TMR_REVA_CN_PRES3);
        tmr->cn |= (MXC_S_TMR_REVA_CN_PRES_DIV32);
        break;

    default:
        tmr->cn |= (MXC_S_TMR_REVA_CN_PRES_DIV1);
        break;
    }

    tmr->cn |= cfg->mode << MXC_F_TMR_REVA_CN_TMODE_POS;
    tmr->cn |= (cfg->pol) << MXC_F_TMR_REVA_CN_TPOL_POS;
    //enable timer interrupt if needed
    tmr->cnt = 0x1;
    tmr->cmp = cfg->cmp_cnt;
}

void MXC_TMR_RevA_Shutdown(mxc_tmr_reva_regs_t *tmr)
{
    MXC_ASSERT(MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr) >= 0);

    // Disable timer and clear settings
    tmr->cn = 0;
}

void MXC_TMR_RevA_Start(mxc_tmr_reva_regs_t *tmr)
{
    MXC_ASSERT(MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr) >= 0);

    tmr->cn |= MXC_F_TMR_REVA_CN_TEN;
}

void MXC_TMR_RevA_Stop(mxc_tmr_reva_regs_t *tmr)
{
    MXC_ASSERT(MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr) >= 0);

    tmr->cn &= ~MXC_F_TMR_REVA_CN_TEN;
}

int MXC_TMR_RevA_SetPWM(mxc_tmr_reva_regs_t *tmr, uint32_t pwm)
{
    MXC_ASSERT(MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr) >= 0);

    if (pwm > (tmr->cmp)) {
        return E_BAD_PARAM;
    }

    tmr->pwm = pwm;

    return E_NO_ERROR;
}

uint32_t MXC_TMR_RevA_GetCompare(mxc_tmr_reva_regs_t *tmr)
{
    MXC_ASSERT(MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr) >= 0);

    return tmr->cmp;
}

uint32_t MXC_TMR_RevA_GetCapture(mxc_tmr_reva_regs_t *tmr)
{
    MXC_ASSERT(MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr) >= 0);

    return tmr->pwm; //check this
}

uint32_t MXC_TMR_RevA_GetCount(mxc_tmr_reva_regs_t *tmr)
{
    MXC_ASSERT(MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr) >= 0);

    return tmr->cnt;
}

void MXC_TMR_RevA_ClearFlags(mxc_tmr_reva_regs_t *tmr)
{
    MXC_ASSERT(MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr) >= 0);

    tmr->intr = MXC_F_TMR_REVA_INTR_IRQ;
}

uint32_t MXC_TMR_RevA_GetFlags(mxc_tmr_reva_regs_t *tmr)
{
    MXC_ASSERT(MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr) >= 0);

    return tmr->intr;
}

void MXC_TMR_RevA_SetCompare(mxc_tmr_reva_regs_t *tmr, uint32_t cmp_cnt)
{
    MXC_ASSERT(MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr) >= 0);

    tmr->cmp = cmp_cnt;
}

void MXC_TMR_RevA_SetCount(mxc_tmr_reva_regs_t *tmr, uint32_t cnt)
{
    MXC_ASSERT(MXC_TMR_GET_IDX((mxc_tmr_regs_t *)tmr) >= 0);

    tmr->cnt = cnt;
}

void MXC_TMR_RevA_TO_Start(mxc_tmr_reva_regs_t *tmr, uint32_t us)
{
    uint64_t ticks;
    int clk_shift = 0;
    mxc_tmr_cfg_t cfg;

    ticks = (uint64_t)us * (uint64_t)PeripheralClock / (uint64_t)1000000;

    while (ticks > 0xFFFFFFFFUL) {
        ticks >>= 1;
        ++clk_shift;
    }

    mxc_tmr_pres_t prescale = (mxc_tmr_pres_t)clk_shift << MXC_F_TMR_REVA_CN_PRES_POS;

    memset(&cfg, 0, sizeof(mxc_tmr_cfg_t));

    // Initialize the timer in one-shot mode
    cfg.pres = prescale;
    cfg.mode = MXC_TMR_MODE_ONESHOT;
    cfg.cmp_cnt = ticks;
    cfg.pol = 0;

    MXC_TMR_Stop((mxc_tmr_regs_t *)tmr);
    MXC_TMR_Init((mxc_tmr_regs_t *)tmr, &cfg);
    MXC_TMR_ClearFlags((mxc_tmr_regs_t *)tmr);
    MXC_TMR_Start((mxc_tmr_regs_t *)tmr);
}

int MXC_TMR_RevA_GetTime(mxc_tmr_reva_regs_t *tmr, uint32_t ticks, uint32_t *time,
                         mxc_tmr_unit_t *units)
{
    uint64_t temp_time = 0;
    uint32_t timerClock = PeripheralClock;
    uint32_t prescale =
        ((tmr->cn & MXC_F_TMR_REVA_CN_PRES) >> MXC_F_TMR_REVA_CN_PRES_POS) |
        (((tmr->cn & MXC_F_TMR_REVA_CN_PRES3) >> (MXC_F_TMR_REVA_CN_PRES3_POS)) << 3);

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
