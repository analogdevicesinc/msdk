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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_TMR_TMR_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_TMR_TMR_REVA_H_

/* **** Includes **** */
#include <stddef.h>
#include "mxc_assert.h"
#include "tmr.h"
#include "gpio.h"
#include "mxc_pins.h"
#include "mxc_lock.h"
#include "tmr_reva_regs.h"

/* **** Functions **** */
void MXC_TMR_RevA_Init(mxc_tmr_reva_regs_t *tmr, mxc_tmr_cfg_t *cfg);
void MXC_TMR_RevA_Shutdown(mxc_tmr_reva_regs_t *tmr);
void MXC_TMR_RevA_Start(mxc_tmr_reva_regs_t *tmr);
void MXC_TMR_RevA_Stop(mxc_tmr_reva_regs_t *tmr);
int MXC_TMR_RevA_SetPWM(mxc_tmr_reva_regs_t *tmr, uint32_t pwm);
uint32_t MXC_TMR_RevA_GetCompare(mxc_tmr_reva_regs_t *tmr);
uint32_t MXC_TMR_RevA_GetCapture(mxc_tmr_reva_regs_t *tmr);
uint32_t MXC_TMR_RevA_GetCount(mxc_tmr_reva_regs_t *tmr);
void MXC_TMR_RevA_ClearFlags(mxc_tmr_reva_regs_t *tmr);
uint32_t MXC_TMR_RevA_GetFlags(mxc_tmr_reva_regs_t *tmr);
void MXC_TMR_RevA_SetCompare(mxc_tmr_reva_regs_t *tmr, uint32_t cmp_cnt);
void MXC_TMR_RevA_SetCount(mxc_tmr_reva_regs_t *tmr, uint32_t cnt);
void MXC_TMR_RevA_TO_Start(mxc_tmr_reva_regs_t *tmr, uint32_t us);
int MXC_TMR_RevA_GetTime(mxc_tmr_reva_regs_t *tmr, uint32_t ticks, uint32_t *time,
                         mxc_tmr_unit_t *units);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_TMR_TMR_REVA_H_
