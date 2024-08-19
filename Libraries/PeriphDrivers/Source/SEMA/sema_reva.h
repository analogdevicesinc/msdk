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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_SEMA_SEMA_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_SEMA_SEMA_REVA_H_

#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "sema.h"
#include "sema_reva_regs.h"

typedef void (*mxc_sema_complete_cb_t)(int result);

int MXC_SEMA_RevA_Init(mxc_sema_reva_regs_t *sema_regs);
int MXC_SEMA_RevA_InitBoxes(mxc_sema_reva_regs_t *sema_regs);
int MXC_SEMA_RevA_GetSema(mxc_sema_reva_regs_t *sema_regs, unsigned sema);
int MXC_SEMA_RevA_CheckSema(mxc_sema_reva_regs_t *sema_regs, unsigned sema);
uint32_t MXC_SEMA_RevA_Status(mxc_sema_reva_regs_t *sema_regs);
void MXC_SEMA_RevA_FreeSema(mxc_sema_reva_regs_t *sema_regs, unsigned sema);

int MXC_SEMA_RevA_ReadBox(mxc_sema_reva_regs_t *sema_regs, uint8_t *data, unsigned len);
int MXC_SEMA_RevA_WriteBox(mxc_sema_reva_regs_t *sema_regs, const uint8_t *data, unsigned len);

int MXC_SEMA_RevA_WriteBoxAsync(mxc_sema_reva_regs_t *sema_regs, mxc_sema_complete_cb_t cb,
                                const uint8_t *data, unsigned len);
int MXC_SEMA_RevA_ReadBoxAsync(mxc_sema_reva_regs_t *sema_regs, mxc_sema_complete_cb_t cb,
                               uint8_t *data, unsigned len);

int MXC_SEMA_RevA_Handler(mxc_sema_reva_regs_t *sema_regs);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_SEMA_SEMA_REVA_H_
