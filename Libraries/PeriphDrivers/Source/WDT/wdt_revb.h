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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_WDT_WDT_REVB_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_WDT_WDT_REVB_H_

/* **** Includes **** */
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "wdt_revb_regs.h"

/* **** Definitions **** */
typedef enum {
    MXC_WDT_REVB_PERIOD_2_31 = MXC_S_WDT_REVB_CTRL_INT_LATE_VAL_WDT2POW31, ///< Period 2^31
    MXC_WDT_REVB_PERIOD_2_30 = MXC_S_WDT_REVB_CTRL_INT_LATE_VAL_WDT2POW30, ///< Period 2^30
    MXC_WDT_REVB_PERIOD_2_29 = MXC_S_WDT_REVB_CTRL_INT_LATE_VAL_WDT2POW29, ///< Period 2^29
    MXC_WDT_REVB_PERIOD_2_28 = MXC_S_WDT_REVB_CTRL_INT_LATE_VAL_WDT2POW28, ///< Period 2^28
    MXC_WDT_REVB_PERIOD_2_27 = MXC_S_WDT_REVB_CTRL_INT_LATE_VAL_WDT2POW27, ///< Period 2^27
    MXC_WDT_REVB_PERIOD_2_26 = MXC_S_WDT_REVB_CTRL_INT_LATE_VAL_WDT2POW26, ///< Period 2^26
    MXC_WDT_REVB_PERIOD_2_25 = MXC_S_WDT_REVB_CTRL_INT_LATE_VAL_WDT2POW25, ///< Period 2^25
    MXC_WDT_REVB_PERIOD_2_24 = MXC_S_WDT_REVB_CTRL_INT_LATE_VAL_WDT2POW24, ///< Period 2^24
    MXC_WDT_REVB_PERIOD_2_23 = MXC_S_WDT_REVB_CTRL_INT_LATE_VAL_WDT2POW23, ///< Period 2^23
    MXC_WDT_REVB_PERIOD_2_22 = MXC_S_WDT_REVB_CTRL_INT_LATE_VAL_WDT2POW22, ///< Period 2^22
    MXC_WDT_REVB_PERIOD_2_21 = MXC_S_WDT_REVB_CTRL_INT_LATE_VAL_WDT2POW21, ///< Period 2^21
    MXC_WDT_REVB_PERIOD_2_20 = MXC_S_WDT_REVB_CTRL_INT_LATE_VAL_WDT2POW20, ///< Period 2^20
    MXC_WDT_REVB_PERIOD_2_19 = MXC_S_WDT_REVB_CTRL_INT_LATE_VAL_WDT2POW19, ///< Period 2^19
    MXC_WDT_REVB_PERIOD_2_18 = MXC_S_WDT_REVB_CTRL_INT_LATE_VAL_WDT2POW18, ///< Period 2^18
    MXC_WDT_REVB_PERIOD_2_17 = MXC_S_WDT_REVB_CTRL_INT_LATE_VAL_WDT2POW17, ///< Period 2^17
    MXC_WDT_REVB_PERIOD_2_16 = MXC_S_WDT_REVB_CTRL_INT_LATE_VAL_WDT2POW16, ///< Period 2^16
} mxc_wdt_revb_period_t;

typedef enum {
    MXC_WDT_REVB_COMPATIBILITY = 0,
    MXC_WDT_REVB_WINDOWED = 1,
} mxc_wdt_revb_mode_t;

typedef enum {
    MXC_WDT_REVB_DISABLE = 0,
    MXC_WDT_REVB_ENABLE = 1,
} mxc_wdt_revb_en_t;

typedef struct {
    mxc_wdt_revb_mode_t mode; ///< WDT mode
    mxc_wdt_revb_period_t upperResetPeriod; ///< Reset upper limit
    mxc_wdt_revb_period_t lowerResetPeriod; ///< Reset lower limit
    mxc_wdt_revb_period_t upperIntPeriod; ///< Interrupt upper limit
    mxc_wdt_revb_period_t lowerIntPeriod; ///< Interrupt lower limit
} mxc_wdt_revb_cfg_t;

/* **** Functions **** */
int MXC_WDT_RevB_Init(mxc_wdt_revb_regs_t *wdt, mxc_wdt_revb_cfg_t *cfg);
void MXC_WDT_RevB_SetIntPeriod(mxc_wdt_revb_regs_t *wdt, mxc_wdt_revb_cfg_t *cfg);
void MXC_WDT_RevB_SetResetPeriod(mxc_wdt_revb_regs_t *wdt, mxc_wdt_revb_cfg_t *cfg);
void MXC_WDT_RevB_Enable(mxc_wdt_revb_regs_t *wdt);
void MXC_WDT_RevB_Disable(mxc_wdt_revb_regs_t *wdt);
void MXC_WDT_RevB_EnableInt(mxc_wdt_revb_regs_t *wdt, mxc_wdt_revb_en_t enable);
void MXC_WDT_RevB_EnableReset(mxc_wdt_revb_regs_t *wdt, mxc_wdt_revb_en_t enable);
void MXC_WDT_RevB_ResetTimer(mxc_wdt_revb_regs_t *wdt);
int MXC_WDT_RevB_GetResetFlag(mxc_wdt_revb_regs_t *wdt);
void MXC_WDT_RevB_ClearResetFlag(mxc_wdt_revb_regs_t *wdt);
int MXC_WDT_RevB_GetIntFlag(mxc_wdt_revb_regs_t *wdt);
void MXC_WDT_RevB_ClearIntFlag(mxc_wdt_revb_regs_t *wdt);
void MXC_WDT_RevB_SetClockSource(mxc_wdt_revb_regs_t *wdt, int clock_source);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_WDT_WDT_REVB_H_
