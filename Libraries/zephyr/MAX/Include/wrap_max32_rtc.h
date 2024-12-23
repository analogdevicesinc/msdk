/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_SOC_MAX32657)

static inline mxc_rtc_clock_t wrap_get_clock_source_instance(uint8_t clock_source)
{
    mxc_rtc_clock_t clk_src;

    switch (clock_source) {
    case 4: // ADI_MAX32_PRPH_CLK_SRC_ERTCO
        clk_src = MXC_RTC_ERTCO_CLK;
        break;
    case 5: // ADI_MAX32_PRPH_CLK_SRC_INRO
        clk_src = MXC_RTC_INRO_CLK;
        break;
    default:
        return -1;
    }

    return clk_src;
}

static inline int Wrap_MXC_RTC_Init(uint32_t sec, uint16_t ssec, uint8_t clock_source)
{
    int ret;
    mxc_rtc_clock_t clk_src;

    clk_src = wrap_get_clock_source_instance(clock_source);

    if (MXC_RTC_SetClockSource(clk_src) == E_NO_ERROR) {
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
