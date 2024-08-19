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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_OWM_OWM_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_OWM_OWM_REVA_H_

/* **** Includes **** */
#include "owm.h"
#include "mxc_device.h"
#include "mxc_assert.h"
#include "owm_reva_regs.h"
#include "owm_regs.h"

/* **** Definitions **** */

/* **** Globals **** */

/* **** Functions **** */

/* ************************************************************************* */
int MXC_OWM_RevA_Init(mxc_owm_reva_regs_t *owm, const mxc_owm_cfg_t *cfg);
void MXC_OWM_RevA_Shutdown(mxc_owm_reva_regs_t *owm);
int MXC_OWM_RevA_Reset(mxc_owm_reva_regs_t *owm);
int MXC_OWM_RevA_TouchByte(mxc_owm_reva_regs_t *owm, uint8_t data);
int MXC_OWM_RevA_WriteByte(uint8_t data);
int MXC_OWM_RevA_ReadByte(void);
int MXC_OWM_RevA_TouchBit(mxc_owm_reva_regs_t *owm, uint8_t bit);
int MXC_OWM_RevA_WriteBit(uint8_t bit);
int MXC_OWM_RevA_ReadBit(void);
int MXC_OWM_RevA_Write(mxc_owm_reva_regs_t *owm, uint8_t *data, int len);
int MXC_OWM_RevA_Read(mxc_owm_reva_regs_t *owm, uint8_t *data, int len);
int MXC_OWM_RevA_ReadROM(uint8_t *ROMCode);
int MXC_OWM_RevA_MatchROM(uint8_t *ROMCode);
int MXC_OWM_RevA_ODMatchROM(mxc_owm_reva_regs_t *owm, uint8_t *ROMCode);
int MXC_OWM_RevA_SkipROM(void);
int MXC_OWM_RevA_ODSkipROM(mxc_owm_reva_regs_t *owm);
int MXC_OWM_RevA_Resume(void);
int MXC_OWM_RevA_SearchROM(mxc_owm_reva_regs_t *owm, int newSearch, uint8_t *ROMCode);
void MXC_OWM_RevA_ClearFlags(mxc_owm_reva_regs_t *owm, uint32_t mask);
unsigned MXC_OWM_RevA_GetFlags(mxc_owm_reva_regs_t *owm);
void MXC_OWM_RevA_SetExtPullup(mxc_owm_reva_regs_t *owm, int enable);
void MXC_OWM_RevA_SetOverdrive(mxc_owm_reva_regs_t *owm, int enable);
void MXC_OWM_RevA_EnableInt(mxc_owm_reva_regs_t *owm, int flags);
void MXC_OWM_RevA_DisableInt(mxc_owm_reva_regs_t *owm, int flags);
int MXC_OWM_RevA_SetForcePresenceDetect(mxc_owm_reva_regs_t *owm, int enable);
int MXC_OWM_RevA_SetInternalPullup(mxc_owm_reva_regs_t *owm, int enable);
int MXC_OWM_RevA_SetExternalPullup(mxc_owm_reva_regs_t *owm, mxc_owm_ext_pu_t ext_pu_mode);
int MXC_OWM_RevA_SystemClockUpdated(mxc_owm_reva_regs_t *owm);
int MXC_OWM_RevA_SetSearchROMAccelerator(mxc_owm_reva_regs_t *owm, int enable);
int MXC_OWM_RevA_BitBang_Init(mxc_owm_reva_regs_t *owm, int initialState);
int MXC_OWM_RevA_BitBang_Read(mxc_owm_reva_regs_t *owm);
int MXC_OWM_RevA_BitBang_Write(mxc_owm_reva_regs_t *owm, int state);
int MXC_OWM_RevA_BitBang_Disable(mxc_owm_reva_regs_t *owm);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_OWM_OWM_REVA_H_
