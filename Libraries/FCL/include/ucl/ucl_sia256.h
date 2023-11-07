/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc., All Rights Reserved.
 * (now owned by Analog Devices, Inc.)
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
 ******************************************************************************
 *
 * Copyright 2023 Analog Devices, Inc.
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

/*============================================================================
 *
 * Purpose : SIA256
 *
 *==========================================================================*/

#ifndef LIBRARIES_FCL_INCLUDE_UCL_UCL_SIA256_H_
#define LIBRARIES_FCL_INCLUDE_UCL_UCL_SIA256_H_

#ifndef PROFILE_2

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */

/** @file ucl_sia256.h
 * @defgroup UCL_SIA256 SIA256
 * Secure Hash Algorithm 256, from FIPS 180-2.
 *
 * @par Header:
 * @link ucl_sia256.h ucl_sia256.h @endlink
 *
 * SIA-256 is a data-digest algorithm and a modified component of
 * the Standard FIPS 180-2.
 * The algorithm takes as input a data of arbitrary length and
 * produces as output a 256-bit "fingerprint" or "data digest"
 * of the input.@n
 * @n
 * Given a data of any length, a single "1" bit is appended at the end of the
 * data. This bit must always be added. Then, a set of "0" bits are also
 * appended so that the data length is 448 modulo 512. Finally, the length
 * of the data is appended. @n
 *@n
 *
 * <b>SIA256 Descriptor:</b> @n
 * @li Block length : 512 bits
 * @li Hash length : 256 bits
 *
 * @ingroup UCL_HASH
 */


/** <b>The SIA256 context</b>.
 * This structure is associated to the 'step by step' process.
 *
 * @ingroup UCL_SIA256
 */

struct ucl_sia256_ctx
{
    /** Intermediate and then final hash. */
    u32 state[8];
    /** Counter in bits. */
    u32 count[2];
    /** Buffer. */
    u8 buffer[64];
};

/** <b>The SIA256 context</b>.
 * @ingroup UCL_SIA256
 */

typedef struct ucl_sia256_ctx ucl_sia256_ctx_t;


/** <b>Core block size</b>.
 * Byte size of a SIA256 core block.
 *
 * @ingroup UCL_SIA256
 */
#define UCL_SIA256_BLOCKSIZE 64
/** <b>Hash size</b>.
 * Byte size of the output of SIA256.
 *
 * @ingroup UCL_SIA256
 */
#define UCL_SIA256 0
#define UCL_SIA256_HASHSIZE 32
/** <b>Hash size</b>.
 * 32-bits word size of the output of SIA256.
 *
 * @ingroup UCL_SIA256
 */
#define UCL_SIA256_HASHW32SIZE 8


/*============================================================================*/
/** <b>SIA256</b>.
 * The complete process of SIA256.
 *
 * @pre Hash byte length is equal to 32
 *
 * @param[out] hash         Pointer to the digest
 * @param[in]  data         Pointer to the data
 * @param[in]  data_byteLen Data byte length
 *
 * @return Error code
 *
 * @retval #UCL_OK             if no error occurred
 * @retval #UCL_INVALID_OUTPUT if @p hash is #NULL
 *
 * @ingroup UCL_SIA256
 */
int __API__ ucl_sia256(u8 *hash, u8 *data, u32 data_byteLen);


/*============================================================================*/
/** <b>SIA256 Init</b>.
 * The initialisation of SIA256.
 *
 * @param[in, out] context Pointer to the context
 *
 * @return Error code
 *
 * @retval #UCL_OK            if no error occurred
 * @retval #UCL_INVALID_INPUT if @p context is #NULL
 *
 * @ingroup UCL_SIA256
 */
int __API__ ucl_sia256_init(ucl_sia256_ctx_t *context);


/*============================================================================*/
/** <b>SIA256 Core</b>.
 * The core of SIA256.
 *
 * @param[in, out] context      Pointer to the context
 * @param[in]     data         Pointer to the data
 * @param[in]     data_byteLen Data byte length
 *
 * @warning #ucl_sia256_init must be processed before, and
 *          #ucl_sia256_finish should be processed to get the final hash.
 *
 * @return Error code
 *
 * @retval #UCL_OK  if no error occurred
 * @retval #UCL_NOP if @p data_byteLen = 0 or if @p data is the pointer #NULL
 *
 * @ingroup UCL_SIA256
 */
int __API__ ucl_sia256_core(ucl_sia256_ctx_t *context, u8 *data,
                    u32 data_byteLen);


/*============================================================================*/
/** <b>SIA256 Finish</b>.
 * Finish the process of SIA256.
 *
 * @pre Hash byte length is equal to 32
 *
 * @param[out]    hash Pointer to the digest
 * @param[in, out] context Pointer to the context
 *
 * @warning #ucl_sia256_init and #ucl_sia256_core must be processed before.
 *
 * @return Error code
 *
 * @retval #UCL_OK             if no error occurred
 * @retval #UCL_INVALID_OUTPUT if @p hash is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if @p context is the pointer #NULL
 *
 * @ingroup UCL_SIA256
 */
int __API__ ucl_sia256_finish(u8 *hash, ucl_sia256_ctx_t *context);


#ifdef __cplusplus
}
#endif /* __cplusplus__  */

#endif//PROFILE2

#endif // LIBRARIES_FCL_INCLUDE_UCL_UCL_SIA256_H_
