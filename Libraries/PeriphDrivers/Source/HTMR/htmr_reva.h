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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_HTMR_HTMR_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_HTMR_HTMR_REVA_H_

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "htmr_reva_regs.h"

/* **** Definitions **** */

int MXC_HTMR_RevA_Init(mxc_htmr_reva_regs_t *htmr, uint32_t sec, uint8_t ssec);
int MXC_HTMR_RevA_Start(mxc_htmr_reva_regs_t *htmr);
int MXC_HTMR_RevA_Stop(mxc_htmr_reva_regs_t *htmr);
int MXC_HTMR_RevA_GetShortCount(mxc_htmr_reva_regs_t *htmr);
int MXC_HTMR_RevA_GetLongCount(mxc_htmr_reva_regs_t *htmr);
int MXC_HTMR_RevA_SetLongAlarm(mxc_htmr_reva_regs_t *htmr, uint32_t ras);
int MXC_HTMR_RevA_SetShortAlarm(mxc_htmr_reva_regs_t *htmr, uint32_t rssa);
int MXC_HTMR_RevA_CheckBusy(mxc_htmr_reva_regs_t *htmr);
int MXC_HTMR_RevA_GetFlags(mxc_htmr_reva_regs_t *htmr);
int MXC_HTMR_RevA_ClearFlags(mxc_htmr_reva_regs_t *htmr, int flags);
int MXC_HTMR_RevA_EnableInt(mxc_htmr_reva_regs_t *htmr, uint32_t mask);
int MXC_HTMR_RevA_DisableInt(mxc_htmr_reva_regs_t *htmr, uint32_t mask);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_HTMR_HTMR_REVA_H_
