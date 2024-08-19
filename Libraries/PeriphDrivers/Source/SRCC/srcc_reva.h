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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_SRCC_SRCC_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_SRCC_SRCC_REVA_H_

/***** Includes *****/
#include "srcc_reva_regs.h"
#include "mxc_device.h"

/***** Definitions *****/

/**
 * @brief   Enumeration type for the SRCC Cache ID Register
 */
typedef enum {
    SRCC_REVA_CACHE_ID_RELNUM, ///< Release Number
    SRCC_REVA_CACHE_ID_PARTNUM, ///< Part Number
    SRCC_REVA_CACHE_ID_CCHID ///< Cache ID
} mxc_srcc_reva_cache_id_t;

/***** Function Prototypes *****/
#if TARGET_NUM != 32650
#include "srcc.h"
uint32_t MXC_SRCC_RevA_ID(mxc_srcc_reva_regs_t *srcc, mxc_srcc_cache_id_t id);
#endif
uint32_t MXC_SRCC_RevA_CacheSize(mxc_srcc_reva_regs_t *srcc);
uint32_t MXC_SRCC_RevA_MemSize(mxc_srcc_reva_regs_t *srcc);
void MXC_SRCC_RevA_Enable(mxc_srcc_reva_regs_t *srcc);
void MXC_SRCC_RevA_Disable(mxc_srcc_reva_regs_t *srcc);
void MXC_SRCC_RevA_WriteAllocateEnable(mxc_srcc_reva_regs_t *srcc);
void MXC_SRCC_RevA_WriteAllocateDisable(mxc_srcc_reva_regs_t *srcc);
void MXC_SRCC_RevA_CriticalWordFirstEnable(mxc_srcc_reva_regs_t *srcc);
void MXC_SRCC_RevA_CriticalWordFirstDisable(mxc_srcc_reva_regs_t *srcc);
uint32_t MXC_SRCC_RevA_Ready(mxc_srcc_reva_regs_t *srcc);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_SRCC_SRCC_REVA_H_
