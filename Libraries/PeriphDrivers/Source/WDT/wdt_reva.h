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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_WDT_WDT_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_WDT_WDT_REVA_H_

/* **** Includes **** */
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "wdt.h"
#include "wdt_reva_regs.h"

/* **** Definitions **** */
typedef enum {
    MXC_WDT_REVA_DISABLE = 0,
    MXC_WDT_REVA_ENABLE = 1,
} mxc_wdt_reva_en_t;

/* **** Functions **** */
void MXC_WDT_RevA_SetIntPeriod(mxc_wdt_reva_regs_t *wdt, mxc_wdt_period_t period);
void MXC_WDT_RevA_SetResetPeriod(mxc_wdt_reva_regs_t *wdt, mxc_wdt_period_t period);
void MXC_WDT_RevA_Enable(mxc_wdt_reva_regs_t *wdt);
void MXC_WDT_RevA_Disable(mxc_wdt_reva_regs_t *wdt);
void MXC_WDT_RevA_EnableInt(mxc_wdt_reva_regs_t *wdt, mxc_wdt_reva_en_t enable);
void MXC_WDT_RevA_EnableReset(mxc_wdt_reva_regs_t *wdt, mxc_wdt_reva_en_t enable);
void MXC_WDT_RevA_ResetTimer(mxc_wdt_reva_regs_t *wdt);
int MXC_WDT_RevA_GetResetFlag(mxc_wdt_reva_regs_t *wdt);
void MXC_WDT_RevA_ClearResetFlag(mxc_wdt_reva_regs_t *wdt);
int MXC_WDT_RevA_GetIntFlag(mxc_wdt_reva_regs_t *wdt);
void MXC_WDT_RevA_ClearIntFlag(mxc_wdt_reva_regs_t *wdt);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_WDT_WDT_REVA_H_
