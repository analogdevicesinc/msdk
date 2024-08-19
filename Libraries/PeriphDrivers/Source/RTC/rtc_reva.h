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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_RTC_RTC_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_RTC_RTC_REVA_H_

#include "gpio.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_sys.h"
#include "rtc_reva_regs.h"
#include "rtc.h"
#include "tmr.h"

typedef enum {
    MXC_RTC_REVA_SQUARE_WAVE_DISABLED, ///< Sq. wave output disabled
    MXC_RTC_REVA_SQUARE_WAVE_ENABLED, ///< Sq. wave output enabled
} mxc_rtc_reva_sqwave_en_t;

#define MXC_RTC_REVA_CTRL_RESET_DEFAULT (0x0000UL)
#define MXC_RTC_REVA_IS_BUSY (MXC_F_RTC_REVA_CTRL_BUSY & MXC_RTC->ctrl)
#define MXC_RTC_REVA_IS_ENABLED (MXC_F_RTC_REVA_CTRL_RTCE & MXC_RTC->ctrl)

#define MXC_RTC_REVA_TRIM_PERIODS 5
#define MXC_RTC_REVA_TICKS_PER_PERIOD (MXC_RTC_MAX_SSEC / MXC_RTC_REVA_TRIM_PERIODS)
#define PPM(ppm) ((ppm * 1000000) / 4096)

#define MXC_BUSY_TIMEOUT 1000 // Timeout counts for the Busy bit

int MXC_RTC_RevA_Init(mxc_rtc_reva_regs_t *rtc, uint32_t sec, uint32_t ssec);
int MXC_RTC_RevA_EnableInt(mxc_rtc_reva_regs_t *rtc, uint32_t mask);
int MXC_RTC_RevA_DisableInt(mxc_rtc_reva_regs_t *rtc, uint32_t mask);
int MXC_RTC_RevA_SetTimeofdayAlarm(mxc_rtc_reva_regs_t *rtc, uint32_t ras);
int MXC_RTC_RevA_SetSubsecondAlarm(mxc_rtc_reva_regs_t *rtc, uint32_t rssa);
int MXC_RTC_RevA_Start(mxc_rtc_reva_regs_t *rtc);
int MXC_RTC_RevA_Stop(mxc_rtc_reva_regs_t *rtc);
int MXC_RTC_RevA_SquareWave(mxc_rtc_reva_regs_t *rtc, mxc_rtc_reva_sqwave_en_t sqe,
                            mxc_rtc_freq_sel_t ft);
int MXC_RTC_RevA_Trim(mxc_rtc_reva_regs_t *rtc, int8_t trm);
int MXC_RTC_RevA_GetFlags(mxc_rtc_reva_regs_t *rtc);
int MXC_RTC_RevA_ClearFlags(mxc_rtc_reva_regs_t *rtc, int flags);
int MXC_RTC_RevA_GetSubSecond(mxc_rtc_reva_regs_t *rtc);
int MXC_RTC_RevA_GetSecond(mxc_rtc_reva_regs_t *rtc);
int MXC_RTC_RevA_GetSubSeconds(mxc_rtc_reva_regs_t *rtc, uint32_t *ssec);
int MXC_RTC_RevA_GetSeconds(mxc_rtc_reva_regs_t *rtc, uint32_t *sec);
int MXC_RTC_RevA_GetTime(mxc_rtc_reva_regs_t *rtc, uint32_t *sec, uint32_t *subsec);
int MXC_RTC_RevA_GetBusyFlag(mxc_rtc_reva_regs_t *rtc);
int MXC_RTC_RevA_TrimCrystal(mxc_rtc_reva_regs_t *rtc, mxc_tmr_regs_t *tmr);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_RTC_RTC_REVA_H_
