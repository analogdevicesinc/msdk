/******************************************************************************
 *
 * Copyright (C) 2025 Croxel, Inc.
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

#ifndef LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_I2S_H_
#define LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_I2S_H_

/***** Includes *****/
#include <i2s.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_SOC_MAX32655) || defined(CONFIG_SOC_MAX32662) || \
    defined(CONFIG_SOC_MAX32672) || defined(CONFIG_SOC_MAX32675) || \
    defined(CONFIG_SOC_MAX32680) || defined(CONFIG_SOC_MAX32690) || \
    defined(CONFIG_SOC_MAX78000) || defined(CONFIG_SOC_MAX78002)

static inline uint32_t Wrap_MXC_I2S_CalculateClockDiv(uint32_t sampleRate, uint16_t wordSize,
                                                      uint32_t i2s_clk)
{
#if defined(CONFIG_SOC_MAX32690) || defined(CONFIG_SOC_MAX78000) || defined(CONFIG_SOC_MAX78002)
    /* usage reference taken from Examples/MAX78000/CNN/Kws20_demo */
    return MXC_I2S_CalculateClockDiv(sampleRate, wordSize, i2s_clk);
#else
    /* usage reference taken from Examples/MAX32655\I2S */
    return MXC_I2S_CalculateClockDiv(sampleRate, wordSize) + 1;
#endif
}

#else
/* CONFIG_SOC_MAX32650 ||  CONFIG_SOC_MAX32660 */
#error "Unsupported SoC for wrap_max32_i2s.h"

#endif /* closing of line:29 */

#ifdef __cplusplus
}
#endif

#endif /* LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_I2S_H_ */
