/******************************************************************************
 *
 * Copyright (C) 2024-2025 Analog Devices, Inc.
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

#ifndef LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_RTC_H_
#define LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_RTC_H_

/***** Includes *****/
#include <rtc.h>
#include <mxc_sys.h>
#include "wrap_max32_sys.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_SOC_MAX32657)

static inline int Wrap_MXC_RTC_Init(uint32_t sec, uint16_t ssec, uint8_t clock_source)
{
    int ret;

    if (Wrap_MXC_SYS_Select32KClockSource(clock_source) == E_NO_ERROR) {
        ret = MXC_RTC_Init(sec, ssec);
    } else {
        ret = E_BAD_PARAM;
    }

    return ret;
}

#else

static inline int Wrap_MXC_RTC_Init(uint32_t sec, uint16_t ssec, uint8_t clock_source)
{
    int ret;

    // Return -1 if clock source not equal to ADI_MAX32_PRPH_CLK_SRC_ERTCO
    if (clock_source != 4) {
        return -1;
    }

    ret = MXC_RTC_Init(sec, ssec);

    return ret;
}
#endif

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_RTC_H_
