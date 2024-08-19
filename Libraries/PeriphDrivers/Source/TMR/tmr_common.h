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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_TMR_TMR_COMMON_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_TMR_TMR_COMMON_H_

/* **** Includes **** */
#include <stddef.h>
#include "mxc_assert.h"
#include "tmr.h"

/* **** Functions **** */
void MXC_TMR_Common_Delay(mxc_tmr_regs_t *tmr, uint32_t us);
void MXC_TMR_Common_TO_Start(mxc_tmr_regs_t *tmr, uint32_t us);
int MXC_TMR_Common_TO_Check(mxc_tmr_regs_t *tmr);
void MXC_TMR_Common_TO_Stop(mxc_tmr_regs_t *tmr);
void MXC_TMR_Common_TO_Clear(mxc_tmr_regs_t *tmr);
unsigned int MXC_TMR_Common_TO_Elapsed(mxc_tmr_regs_t *tmr);
unsigned int MXC_TMR_Common_TO_Remaining(mxc_tmr_regs_t *tmr);
void MXC_TMR_Common_SW_Start(mxc_tmr_regs_t *tmr);
unsigned int MXC_TMR_Common_SW_Stop(mxc_tmr_regs_t *tmr);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_TMR_TMR_COMMON_H_
