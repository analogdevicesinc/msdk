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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_LPCMP_LPCMP_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_LPCMP_LPCMP_REVA_H_

#include <stdio.h>
#include "lpcmp_reva_regs.h"
#include "lpcmp.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "mcr_regs.h"
#include "mxc_lock.h"

int MXC_LPCMP_RevA_Init(mxc_lpcmp_ctrl_reg_t ctrl_reg);

int MXC_LPCMP_RevA_Shutdown(mxc_lpcmp_ctrl_reg_t ctrl_reg);

int MXC_LPCMP_RevA_EnableInt(mxc_lpcmp_ctrl_reg_t ctrl_reg);

int MXC_LPCMP_RevA_DisableInt(mxc_lpcmp_ctrl_reg_t ctrl_reg);

int MXC_LPCMP_RevA_GetFlags(mxc_lpcmp_ctrl_reg_t ctrl_reg);

int MXC_LPCMP_RevA_ClearFlags(mxc_lpcmp_ctrl_reg_t ctrl_reg);

int MXC_LPCMP_RevA_SelectPolarity(mxc_lpcmp_ctrl_reg_t ctrl_reg, mxc_lpcmp_polarity_t pol);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_LPCMP_LPCMP_REVA_H_
