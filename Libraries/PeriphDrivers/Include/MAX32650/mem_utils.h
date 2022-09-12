/**
 * @file 	mem_utils.h
 * @brief 	memory utility functions
 */
/* ***************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 * $Date: 2020-04-20 15:48:35 -0500 (Mon, 20 Apr 2020) $
 * $Revision: 53144 $
 *
 ************************************************************************** */

#ifndef _MEM_UTILS_H_
#define _MEM_UTILS_H_
#include "mxc_config.h"
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

#endif /* _MEM_UTILS_H_ */
