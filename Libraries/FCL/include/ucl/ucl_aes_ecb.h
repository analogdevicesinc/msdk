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

#ifndef LIBRARIES_FCL_INCLUDE_UCL_UCL_AES_ECB_H_
#define LIBRARIES_FCL_INCLUDE_UCL_UCL_AES_ECB_H_

#include "ucl/ucl_aes.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus  */

/** @file ucl_aes_ecb.h
 * @defgroup UCL_ECB_AES AES ECB
 * Encrypt / Decrypt with AES in ECB (Electronic Codebook) mode.
 *
 * @par Header:
 * @link ucl_aes_ecb.h ucl_aes_ecb.h @endlink
 *
 * @ingroup UCL_ECB
 */

/** <b>AES-ECB</b>.
 * Complete process.
 *
 * @param[out] dst    Pointer to the output data
 * @param[in]  src    Pointer to the input data
 * @param[in]  len    Data byte length
 * @param[in]  key    Pointer to the AES Key
 * @param[in]  keylen Key byte length:
 *                        @li #UCL_AES_KEYLEN_128
 *                        @li #UCL_AES_KEYLEN_192
 *                        @li #UCL_AES_KEYLEN_256
 * @param[in]  mode   The mode (Encryption/Decryption):
 *                        @li #UCL_CIPHER_ENCRYPT
 *                        @li #UCL_CIPHER_DECRYPT
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_INPUT  One of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT One of the output is the pointer NULL
 * @retval #UCL_INVALID_ARG    @p len is not a multiple of
 *                             #UCL_AES_BLOCKSIZE or @p keylen is invalid
 *
 * @ingroup UCL_ECB_AES
 */
int ucl_aes_ecb(u8 *dst, u8 *src, u32 len, u8 *key, u32 keylen, int mode);

/** <b>AES-ECB Init</b>.
 * Initialise AES ECB Context.
 *
 * @warning on platforms using a AES hardware block, this function is required to change the key
 * the use of another context in the ucl_aes_core function will not mean using the other context key
 *
 * @param[out] ctx    Pointer to the context
 * @param[in]  key    Pointer to the AES Key
 * @param[in]  keylen Key byte length:
 *                        @li #UCL_AES_KEYLEN_128
 *                        @li #UCL_AES_KEYLEN_192
 *                        @li #UCL_AES_KEYLEN_256
 * @param[in]  mode   The mode (Encryption/Decryption):
 *                        @li #UCL_CIPHER_ENCRYPT
 *                        @li #UCL_CIPHER_DECRYPT
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_INPUT  The input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT The output is the pointer #NULL
 * @retval #UCL_INVALID_ARG    @p keylen is invalid
 * @retval #UCL_INVALID_MODE   The mode is not one of those uaescribed
 *
 * @ingroup UCL_ECB_AES
 */
int ucl_aes_ecb_init(ucl_aes_ctx_t *ctx, u8 *key, u32 keylen, int mode);

/** <b>AES-ECB Core</b>.
 * Process the Data.
 *
 * @param[out]    dst Pointer to the output data
 * @param[out, in] ctx AES context
 * @param[in]     src Pointer to the input data
 * @param[in]     len Data byte length
 *
 * @pre The byte length must be a multiple of #UCL_AES_BLOCKSIZE.
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_INPUT  One of the input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT The output is the pointer #NULL
 * @retval #UCL_INVALID_ARG    The byte length is not a multiple of
 *                             #UCL_AES_BLOCKSIZE
 *
 * @ingroup UCL_ECB_AES
 */
int ucl_aes_ecb_core(u8 *dst, ucl_aes_ctx_t *ctx, u8 *src, u32 len);

/** <b>AES-ECB Finish</b>.
 * Zeroize the context.
 *
 * @param[out, in] ctx Pointer to the context
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_OUTPUT The output is the pointer #NULL
 *
 * @ingroup UCL_ECB_AES
 */
int ucl_aes_ecb_finish(ucl_aes_ctx_t *ctx);

/** <b>AES-ECB Core</b>.
 * Process the Data.
 *
 * @param[out]  dst Pointer to the output data
 * @param[out, in] ctx AES context
 * @param[in]  src Pointer to the input data
 * @param[in]  len Data byte length
 *
 * @pre The byte length must be a multiple of #UCL_AES_BLOCKSIZE.
 *
 * @return Error code
 *
 * @retval #UCL_OK    No error occurred
 * @retval #UCL_INVALID_INPUT One of the input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT The output is the pointer #NULL
 * @retval #UCL_INVALID_ARG  The byte length is not a multiple of
 *          #UCL_AES_BLOCKSIZE
 *
 * @ingroup UCL_ECB_AES */
  int ucl_aes_ecb_core_context(u8 *dst, ucl_aes_ctx_t *ctx, u8 *src, u32 len);


#ifdef __cplusplus
}
#endif /* __cplusplus  */

#endif // LIBRARIES_FCL_INCLUDE_UCL_UCL_AES_ECB_H_
