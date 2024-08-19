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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_QDEC_QDEC_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_QDEC_QDEC_REVA_H_

#include <stdio.h>
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "mxc_lock.h"
#include "qdec.h"
#include "qdec_reva_regs.h"

int MXC_QDEC_RevA_Init(mxc_qdec_reva_regs_t *qdec, mxc_qdec_req_t *req);
int MXC_QDEC_RevA_Shutdown(mxc_qdec_reva_regs_t *qdec);
void MXC_QDEC_RevA_EnableInt(mxc_qdec_reva_regs_t *qdec, uint32_t flags);
void MXC_QDEC_RevA_DisableInt(mxc_qdec_reva_regs_t *qdec, uint32_t flags);
int MXC_QDEC_RevA_GetFlags(mxc_qdec_reva_regs_t *qdec);
void MXC_QDEC_RevA_ClearFlags(mxc_qdec_reva_regs_t *qdec, uint32_t flags);
void MXC_QDEC_RevA_SetMaxCount(mxc_qdec_reva_regs_t *qdec, uint32_t maxCount);
int MXC_QDEC_RevA_GetMaxCount(mxc_qdec_reva_regs_t *qdec);
void MXC_QDEC_RevA_SetInitial(mxc_qdec_reva_regs_t *qdec, uint32_t initial);
int MXC_QDEC_RevA_GetInitial(mxc_qdec_reva_regs_t *qdec);
void MXC_QDEC_RevA_SetCompare(mxc_qdec_reva_regs_t *qdec, uint32_t compare);
int MXC_QDEC_RevA_GetCompare(mxc_qdec_reva_regs_t *qdec);
void MXC_QDEC_RevA_SetIndex(mxc_qdec_reva_regs_t *qdec, uint32_t index);
int MXC_QDEC_RevA_GetIndex(mxc_qdec_reva_regs_t *qdec);
int MXC_QDEC_RevA_GetCapture(mxc_qdec_reva_regs_t *qdec);
int MXC_QDEC_RevA_Handler(mxc_qdec_reva_regs_t *qdec);
int MXC_QDEC_RevA_GetPosition(mxc_qdec_reva_regs_t *qdec);
int MXC_QDEC_RevA_GetDirection(mxc_qdec_reva_regs_t *qdec);

#endif //  LIBRARIES_PERIPHDRIVERS_SOURCE_QDEC_QDEC_REVA_H_
