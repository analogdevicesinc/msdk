/******************************************************************************
 *
 * Copyright (C) 2023-2025 Analog Devices, Inc.
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

#ifndef LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_LP_H_
#define LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_LP_H_

/***** Includes *****/
#include <lp.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  MAX32665, MAX32666 related mapping
 */
#if defined(CONFIG_SOC_MAX32665) || defined(CONFIG_SOC_MAX32666) || \
    defined(CONFIG_SOC_MAX32670) || defined(CONFIG_SOC_MAX32672) || \
    defined(CONFIG_SOC_MAX32662) || defined(CONFIG_SOC_MAX32675) || defined(CONFIG_SOC_MAX32650)

static inline void Wrap_MXC_LP_EnterLowPowerMode(void)
{
    MXC_LP_EnterDeepSleepMode();
}

static inline void Wrap_MXC_LP_EnterMicroPowerMode(void)
{
    MXC_LP_EnterDeepSleepMode();
}

static inline void Wrap_MXC_LP_EnterStandbyMode(void)
{
    MXC_LP_EnterDeepSleepMode();
}

static inline void Wrap_MXC_LP_EnterPowerDownMode(void)
{
#if defined(CONFIG_SOC_MAX32650)
    MXC_LP_EnterBackupMode();
#else
    MXC_LP_EnterShutDownMode();
#endif
}

/*
 *  MAX32690, MAX32655 related mapping
 */
#elif defined(CONFIG_SOC_MAX32690) || defined(CONFIG_SOC_MAX32655) || \
    defined(CONFIG_SOC_MAX32680) || defined(CONFIG_SOC_MAX32657) ||   \
    defined(CONFIG_SOC_MAX78002) || defined(CONFIG_SOC_MAX78000)

static inline void Wrap_MXC_LP_EnterLowPowerMode(void)
{
#if defined(CONFIG_SOC_MAX32657)
    MXC_LP_EnterSleepMode();
#else
    MXC_LP_EnterLowPowerMode();
#endif
}

static inline void Wrap_MXC_LP_EnterMicroPowerMode(void)
{
#if defined(CONFIG_SOC_MAX32657)
    MXC_LP_EnterSleepMode();
#else
    MXC_LP_EnterMicroPowerMode();
#endif
}

static inline void Wrap_MXC_LP_EnterStandbyMode(void)
{
    MXC_LP_EnterStandbyMode();
}

static inline void Wrap_MXC_LP_EnterPowerDownMode(void)
{
    MXC_LP_EnterPowerDownMode();
}

#if defined(CONFIG_SOC_MAX32657)
static inline int Wrap_MXC_LP_EnterBackupMode(void)
{
    MXC_LP_EnterBackupMode();

    /* For compatibility with Zephyr PM */
    return -1;
}

static inline void Wrap_MXC_LP_EnableSramRetention(uint32_t mask)
{
    MXC_LP_EnableSramRetention(mask);
}

static inline void Wrap_MXC_LP_DisableSramRetention()
{
    MXC_LP_DisableSramRetention(0x1F);
}

static inline void Wrap_MXC_LP_EnableRetentionReg(void)
{
    MXC_LP_EnableRetentionReg();
}

static inline void Wrap_MXC_LP_DisableRetentionReg(void)
{
    MXC_LP_DisableRetentionReg();
}
#endif // CONFIG_SOC_MAX32657

#endif // part number

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_LP_H_
