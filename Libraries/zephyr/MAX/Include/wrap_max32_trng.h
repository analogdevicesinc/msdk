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

#ifndef LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_TRNG_H_
#define LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_TRNG_H_

/***** Includes *****/
#include <trng.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  MAX32665, MAX32666 related mapping
 */
#if defined(CONFIG_SOC_MAX32665) || (CONFIG_SOC_MAX32666)

static inline int Wrap_MXC_TRNG_RandomInt_NonBlocking(uint32_t *data)
{
    if ((MXC_TRNG->st & MXC_F_TRNG_ST_RND_RDY) == 0) {
        return -1; /* Not ready */
    }
    *data = MXC_TRNG->data;

    return 0;
}

/*
 *  MAX32690, MAX32655 related mapping
 */
#elif defined(CONFIG_SOC_MAX32690) || (CONFIG_SOC_MAX32655) || (CONFIG_SOC_MAX32670) || \
    (CONFIG_SOC_MAX32672) || (CONFIG_SOC_MAX32662) || (CONFIG_SOC_MAX32675) ||          \
    (CONFIG_SOC_MAX32680)

static inline int Wrap_MXC_TRNG_RandomInt_NonBlocking(uint32_t *data)
{
    if ((MXC_TRNG->status & MXC_F_TRNG_STATUS_RDY) == 0) {
        return -1; /* Not ready */
    }
    *data = MXC_TRNG->data;

    return 0;
}
#endif // part number

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_TRNG_H_
