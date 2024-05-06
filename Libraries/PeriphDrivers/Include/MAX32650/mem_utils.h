/**
 * @file 	mem_utils.h
 * @brief 	memory utility functions
 */
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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_MEM_UTILS_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_MEM_UTILS_H_

#include "mxc_device.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup    mem_utils Memory Utility Functions 
 * @ingroup     devicelibs 
 * @{
 */
/* **** Definitions **** */

/* **** Global Data **** */

/* **** Function Prototypes **** */

/**
 * @brief      32-bit wide memory copy.
 *
 * @param      dst   Pointer to the destination location
 * @param      src   Pointer to the source location
 * @param[in]  len   Number of bytes to copy which must be a multiple of 4.
 * @note       This function assumes the destination and source are 32-bit
 *             word aligned. A minimum of 1 word is copied (len = 4).
 */
void memcpy32(uint32_t *dst, uint32_t *src, unsigned int len);

/**
 *
 * @brief      Compares the first @c len bytes of src and @c dst and
 *             returns #E_NO_ERROR if they match.
 * @param      dst   Destination address
 * @param      src   Source address
 * @param[in]  len   Number of bytes to compare between the @c src and
 *                   @c dst.
 * @note       The compare is done 32-bits at a time and @c len should be
 *             a multiple of 4 and any bytes beyond a multiple of 4 are
 *             ignored. For example, the following call to memcmp32() only
 *             compares the first 8 bytes (2 32-bit words) of @c dst and
 *             @c src and would return 0 indicating the memory is the same.
 *             @code
 *                int retval;
 *                int len = 11;
 *                char src[len] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
 *                char dst[len] = {0, 1, 2, 3, 4, 5, 6, 7, 0, 0, 0};
 *
 *                if (memcmp32(dst, src, len) != 0)
 *                {
 *                   printf("Memory compare failed\n");
 *                }
 *                else
 *                {
 *                   // memcmp32 passes even though the src and dst are
 *                   // not identical starting at src[8] and dst[8]
 *                   printf("memcmp32 passed.\n");
 *                }
 *             @endcode
 *
 * @retval     #E_NO_ERROR Contents of @c src and @c dst are equal
 * @retval     #E_INVALID Memory is not equal
 */
int memcmp32(uint32_t *dst, uint32_t *src, unsigned int len);

/**@} end of group mem_utils */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_MEM_UTILS_H_
