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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_PT_PT_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_PT_PT_REVA_H_

#include <stdio.h>
#include "pt.h"
#include "gcr_regs.h"
#include "pt_reva_regs.h"
#include "ptg_reva_regs.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"

void MXC_PT_RevA_Init(mxc_ptg_reva_regs_t *ptg, mxc_clk_scale_t clk_scale);
int MXC_PT_RevA_Shutdown(mxc_ptg_reva_regs_t *ptg, uint32_t pts);
int MXC_PT_RevA_Config(mxc_ptg_reva_regs_t *ptg, mxc_pt_cfg_t *cfg);
int MXC_PT_RevA_SqrWaveConfig(mxc_ptg_reva_regs_t *ptg, mxc_pt_cfg_t *sqwcfg, unsigned channel,
                              uint32_t freq);
void MXC_PT_RevA_Start(mxc_ptg_reva_regs_t *ptg, unsigned pts);
void MXC_PT_RevA_Stop(mxc_ptg_reva_regs_t *ptg, unsigned pts);
uint32_t MXC_PT_RevA_IsActive(mxc_ptg_reva_regs_t *ptg, uint32_t pts);
void MXC_PT_RevA_SetPattern(unsigned pts, uint32_t pattern);
void MXC_PT_RevA_EnableInt(mxc_ptg_reva_regs_t *ptg, uint32_t pts);
void MXC_PT_RevA_DisableInt(mxc_ptg_reva_regs_t *ptg, uint32_t pts);
uint32_t MXC_PT_RevA_GetFlags(mxc_ptg_reva_regs_t *ptg);
void MXC_PT_RevA_ClearFlags(mxc_ptg_reva_regs_t *ptg, uint32_t flags);
void MXC_PT_RevA_EnableStopInt(mxc_ptg_reva_regs_t *ptg, uint32_t pts);
void MXC_PT_RevA_DisableStopInt(mxc_ptg_reva_regs_t *ptg, uint32_t pts);
uint32_t MXC_PT_RevA_GetStopFlags(mxc_ptg_reva_regs_t *ptg);
void MXC_PT_RevA_ClearStopFlags(mxc_ptg_reva_regs_t *ptg, uint32_t flags);
void MXC_PT_RevA_EnableRestart(unsigned start, unsigned stop, uint8_t restartIndex);
void MXC_PT_RevA_DisableRestart(unsigned channel, uint8_t restartIndex);
void MXC_PT_RevA_Resync(mxc_ptg_reva_regs_t *ptg, uint32_t pts);
void MXC_PT_RevA_EnableReadyInt(mxc_ptg_reva_regs_t *ptg, uint32_t pts);
void MXC_PT_RevA_DisableReadyInt(mxc_ptg_reva_regs_t *ptg, uint32_t pts);
uint32_t MXC_PT_RevA_GetReadyFlags(mxc_ptg_reva_regs_t *ptg);
void MXC_PT_RevA_ClearReadyFlags(mxc_ptg_reva_regs_t *ptg, uint32_t flags);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_PT_PT_REVA_H_
