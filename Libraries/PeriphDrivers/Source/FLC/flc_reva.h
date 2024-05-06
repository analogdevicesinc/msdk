/**
 * @file flc_reva.h
 * @brief      Flash RevA Controller driver.
 * @details    This driver can be used to operate on the embedded flash memory.
 */
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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_FLC_FLC_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_FLC_FLC_REVA_H_

/* **** Includes **** */
#include <string.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "flc.h"
#include "flc_common.h"
#include "flc_reva_regs.h"

/**
 * @ingroup flc
 * @{
 */

/* **** Definitions **** */

/* **** Globals **** */

/* **** Functions **** */

int MXC_FLC_RevA_Busy(void);

int MXC_FLC_RevA_MassErase(mxc_flc_reva_regs_t *flc);

int MXC_FLC_RevA_PageErase(mxc_flc_reva_regs_t *flc, uint32_t addr);

int MXC_FLC_RevA_Write32(mxc_flc_reva_regs_t *flc, uint32_t locgialAddr, uint32_t data,
                         uint32_t physicalAddr);

int MXC_FLC_RevA_Write32Using128(mxc_flc_reva_regs_t *flc, uint32_t locgialAddr, uint32_t data,
                                 uint32_t physicalAddr);

int MXC_FLC_RevA_Write128(mxc_flc_reva_regs_t *flc, uint32_t addr, uint32_t *data);

void MXC_FLC_RevA_SetFLCInt(mxc_flc_reva_regs_t *flc);

mxc_flc_reva_regs_t *MXC_FLC_RevA_GetFLCInt(void);

int MXC_FLC_RevA_EnableInt(uint32_t mask);

int MXC_FLC_RevA_DisableInt(uint32_t mask);

int MXC_FLC_RevA_GetFlags(void);

int MXC_FLC_RevA_ClearFlags(uint32_t mask);

int MXC_FLC_RevA_UnlockInfoBlock(mxc_flc_reva_regs_t *flc, uint32_t address);

int MXC_FLC_RevA_LockInfoBlock(mxc_flc_reva_regs_t *flc, uint32_t address);

int MXC_FLC_RevA_BlockPageWrite(uint32_t address, uint32_t bank_base);

int MXC_FLC_RevA_BlockPageRead(uint32_t address, uint32_t bank_base);

/**@} end of group flc */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_FLC_FLC_REVA_H_
