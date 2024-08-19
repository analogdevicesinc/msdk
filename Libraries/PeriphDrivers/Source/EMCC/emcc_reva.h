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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_EMCC_EMCC_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_EMCC_EMCC_REVA_H_

/***** Includes *****/
#include "emcc_reva_regs.h"
#include "mxc_device.h"

/***** Definitions *****/

/**
 * @brief   Enumeration type for the EMCC Cache ID Register
 */
typedef enum {
    EMCC_REVA_CACHE_ID_RELNUM, ///< Release Number
    EMCC_REVA_CACHE_ID_PARTNUM, ///< Part Number
    EMCC_REVA_CACHE_ID_CCHID ///< Cache ID
} mxc_emcc_reva_info_t;

/***** Function Prototypes *****/
#if TARGET_NUM != 32650
#include "emcc.h"
uint32_t MXC_EMCC_RevA_ID(mxc_emcc_reva_regs_t *emcc, mxc_emcc_info_t id);
#endif

uint32_t MXC_EMCC_RevA_CacheSize(mxc_emcc_reva_regs_t *emcc);
uint32_t MXC_EMCC_RevA_MemSize(mxc_emcc_reva_regs_t *emcc);
void MXC_EMCC_RevA_Enable(mxc_emcc_reva_regs_t *emcc);
void MXC_EMCC_RevA_Disable(mxc_emcc_reva_regs_t *emcc);
void MXC_EMCC_RevA_WriteAllocateEnable(mxc_emcc_reva_regs_t *emcc);
void MXC_EMCC_RevA_WriteAllocateDisable(mxc_emcc_reva_regs_t *emcc);
void MXC_EMCC_RevA_CriticalWordFirstEnable(mxc_emcc_reva_regs_t *emcc);
void MXC_EMCC_RevA_CriticalWordFirstDisable(mxc_emcc_reva_regs_t *emcc);
uint32_t MXC_EMCC_RevA_Ready(mxc_emcc_reva_regs_t *emcc);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_EMCC_EMCC_REVA_H_
