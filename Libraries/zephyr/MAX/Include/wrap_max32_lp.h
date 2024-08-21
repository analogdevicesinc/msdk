/******************************************************************************
 *
 * Copyright (C) 2023 Analog Devices, Inc.
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
#if defined(CONFIG_SOC_MAX32665) || (CONFIG_SOC_MAX32666) || (CONFIG_SOC_MAX32670) || \
    (CONFIG_SOC_MAX32672) || (CONFIG_SOC_MAX32662) || (CONFIG_SOC_MAX32675)

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
    MXC_LP_EnterShutDownMode();
}

/*
 *  MAX32690, MAX32655 related mapping
 */
#elif defined(CONFIG_SOC_MAX32690) || (CONFIG_SOC_MAX32655) || (CONFIG_SOC_MAX32680)

static inline void Wrap_MXC_LP_EnterLowPowerMode(void)
{
    MXC_LP_EnterLowPowerMode();
}

static inline void Wrap_MXC_LP_EnterMicroPowerMode(void)
{
    MXC_LP_EnterMicroPowerMode();
}

static inline void Wrap_MXC_LP_EnterStandbyMode(void)
{
    MXC_LP_EnterStandbyMode();
}

static inline void Wrap_MXC_LP_EnterPowerDownMode(void)
{
    MXC_LP_EnterPowerDownMode();
}

#endif // part number

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_LP_H_
