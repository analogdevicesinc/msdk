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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_SIMO_SIMO_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_SIMO_SIMO_REVA_H_

#include "mxc_device.h"
#include "simo.h"
#include "simo_reva_regs.h"

/* **** Function Prototypes **** */
void MXC_SIMO_RevA_SetVregO_A(mxc_simo_reva_regs_t *simo, uint32_t voltage);
void MXC_SIMO_RevA_SetVregO_B(mxc_simo_reva_regs_t *simo, uint32_t voltage);
void MXC_SIMO_RevA_SetVregO_C(mxc_simo_reva_regs_t *simo, uint32_t voltage);
void MXC_SIMO_RevA_SetVregO_D(mxc_simo_reva_regs_t *simo, uint32_t voltage);

uint32_t MXC_SIMO_RevA_GetOutReadyA(mxc_simo_reva_regs_t *simo);
uint32_t MXC_SIMO_RevA_GetOutReadyB(mxc_simo_reva_regs_t *simo);
uint32_t MXC_SIMO_RevA_GetOutReadyC(mxc_simo_reva_regs_t *simo);
uint32_t MXC_SIMO_RevA_GetOutReadyD(mxc_simo_reva_regs_t *simo);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_SIMO_SIMO_REVA_H_
