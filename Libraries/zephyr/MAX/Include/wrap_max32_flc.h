/******************************************************************************
 *
 * Copyright (C) 2025 Analog Devices, Inc.
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

#ifndef LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_FLC_H_
#define LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_FLC_H_

/***** Includes *****/
#include <flc.h>
#include <icc.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_SOC_MAX32690)

static inline int Wrap_MXC_FLC_Write(uint32_t address, uint32_t length, uint32_t *buffer)
{
    int ret = 0;

    MXC_ICC_Disable(MXC_ICC);
    ret = MXC_FLC_Write(address, length, buffer);
    MXC_ICC_Enable(MXC_ICC);

    return ret;
}

#else

static inline int Wrap_MXC_FLC_Write(uint32_t address, uint32_t length, uint32_t *buffer)
{
    return MXC_FLC_Write(address, length, buffer);
}

#endif // part number

#if defined(CONFIG_SOC_MAX32657)

static inline int Wrap_MXC_FLC_Read(uint32_t address, void *buffer, int len)
{
    int ret = 0;

    ret = MXC_FLC_ReadECC(address, buffer, len);

    return ret;
}

#else

static inline int Wrap_MXC_FLC_Read(uint32_t address, void *buffer, int len)
{
    MXC_FLC_Read(address, buffer, len);

    return 0;
}

#endif /* CONFIG_SOC_MAX32657 */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_FLC_H_
