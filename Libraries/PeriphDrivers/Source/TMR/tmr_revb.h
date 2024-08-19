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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_TMR_TMR_REVB_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_TMR_TMR_REVB_H_

/* **** Includes **** */
#include <stddef.h>
#include <stdbool.h>
#include "mxc_assert.h"
#include "tmr.h"
#include "gpio.h"
#include "mxc_pins.h"
#include "mxc_lock.h"
#include "tmr_revb_regs.h"

typedef enum {
    MXC_TMR_CLK0,
    MXC_TMR_CLK1,
    MXC_TMR_CLK2,
    MXC_TMR_CLK3,
} mxc_tmr_clksel_t;

/* **** Functions **** */
int MXC_TMR_RevB_Init(mxc_tmr_revb_regs_t *tmr, mxc_tmr_cfg_t *cfg, uint8_t clk_src);
void MXC_TMR_RevB_LockClockSource(mxc_tmr_revb_regs_t *tmr, bool lock);
void MXC_TMR_RevB_SetClockSource(mxc_tmr_revb_regs_t *tmr, mxc_tmr_bit_mode_t bit_mode,
                                 uint8_t clk_src);
void MXC_TMR_RevB_SetPrescalar(mxc_tmr_revb_regs_t *tmr, mxc_tmr_bit_mode_t bit_mode,
                               mxc_tmr_pres_t prescalar);
void MXC_TMR_RevB_SetClockSourceFreq(mxc_tmr_revb_regs_t *tmr, int clksrc_freq);
int MXC_TMR_RevB_GetClockSourceFreq(mxc_tmr_revb_regs_t *tmr);
void MXC_TMR_RevB_ConfigGeneric(mxc_tmr_revb_regs_t *tmr, mxc_tmr_cfg_t *cfg);
void MXC_TMR_RevB_Shutdown(mxc_tmr_revb_regs_t *tmr);
void MXC_TMR_RevB_Start(mxc_tmr_revb_regs_t *tmr);
void MXC_TMR_RevB_Stop(mxc_tmr_revb_regs_t *tmr);
int MXC_TMR_RevB_SetPWM(mxc_tmr_revb_regs_t *tmr, uint32_t pwm);
uint32_t MXC_TMR_RevB_GetCompare(mxc_tmr_revb_regs_t *tmr);
uint32_t MXC_TMR_RevB_GetCapture(mxc_tmr_revb_regs_t *tmr);
uint32_t MXC_TMR_RevB_GetCount(mxc_tmr_revb_regs_t *tmr);
uint32_t MXC_TMR_RevB_GetPeriod(mxc_tmr_revb_regs_t *tmr, uint32_t clk_frequency,
                                uint32_t prescalar, uint32_t frequency);
void MXC_TMR_RevB_ClearFlags(mxc_tmr_revb_regs_t *tmr);
uint32_t MXC_TMR_RevB_GetFlags(mxc_tmr_revb_regs_t *tmr);
void MXC_TMR_RevB_EnableInt(mxc_tmr_revb_regs_t *tmr);
void MXC_TMR_RevB_DisableInt(mxc_tmr_revb_regs_t *tmr);
void MXC_TMR_RevB_EnableWakeup(mxc_tmr_revb_regs_t *tmr, mxc_tmr_cfg_t *cfg);
void MXC_TMR_RevB_DisableWakeup(mxc_tmr_revb_regs_t *tmr, mxc_tmr_cfg_t *cfg);
void MXC_TMR_RevB_SetCompare(mxc_tmr_revb_regs_t *tmr, uint32_t cmp_cnt);
void MXC_TMR_RevB_SetCount(mxc_tmr_revb_regs_t *tmr, uint32_t cnt);
void MXC_TMR_RevB_TO_Start(mxc_tmr_revb_regs_t *tmr, uint32_t us);
int MXC_TMR_RevB_GetTime(mxc_tmr_revb_regs_t *tmr, uint32_t ticks, uint32_t *time,
                         mxc_tmr_unit_t *units);
int MXC_TMR_RevB_GetTicks(mxc_tmr_revb_regs_t *tmr, uint32_t time, mxc_tmr_unit_t units,
                          uint32_t *ticks);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_TMR_TMR_REVB_H_
