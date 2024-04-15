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

#ifndef LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_UTILS_H_
#define LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_UTILS_H_

/***** Includes *****/

#ifdef __cplusplus
extern "C" {
#endif

/* 
 * Find least significant bit set in a 32-bit word
 * Returns
 *    least significant bit set, 0 if val is 0
 */
static inline int wrap_utils_find_lsb_set(uint32_t val)
{
    int lsb = 0;

    for (int i = 0; i < 32; i++) {
        if (val & (1 << i)) {
            lsb = (i + 1);
            break;
        }
    }

    return lsb;
}

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_UTILS_H_
