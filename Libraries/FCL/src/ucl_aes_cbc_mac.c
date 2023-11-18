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

#include <ucl/ucl_types.h>
#include <ucl/ucl_defs.h>
#include <ucl/ucl_retdefs.h>

#include <ucl/ucl_aes.h>
#include <ucl/ucl_aes_ecb.h>
#include <ucl/ucl_aes_cbc_mac.h>
#include <ucl/ucl_aes_cbc.h>
#include <string.h>

extern int _ucl_increment_dst;

/*============================================================================ */
/** < b >AES-CBC-MAC < /b >.
 *  ISO/IEC 9797 Part I MAC Algorithm 3.
 *
 * @param[out] tmac   Pointer to the mac (may be truncated)
 * @param[in]  tmac_byteLen  Length of the mac
 * @param[in]  data  Pointer to the input data
 * @param[in]  key    Pointer to the 3DES 128-bits key
 * @param[in]  data_byteLen  Data byte length
 *
 * @warning The message must be padded before process. Use a padding method
 * described in ISO/IEC 9797.
 *
 * @pre @p data_byteLen is a multiple of 8.
 *
 * @return Error code
 *
 * @retval #UCL_OK       if no error occurred
 * @retval #UCL_INVALID_INPUT  if one of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT  if one of the output is the pointer NULL
 * @retval  #UCL_INVALID_ARG   if @p data_byteLen is not a multiple of 8
 *
 * @ingroup UCL_CBC_MAC
 */
int ucl_aes_cbc_mac(u8 *tmac, u8 tmac_byteLen, u8 *src, u32 len, u8 *key, u32 keylen)
{
    u8 mac[UCL_AES_BLOCKSIZE];
    ucl_aes_ctx_t ctx;

    if ((src == NULL) || (key == NULL)) {
        return UCL_INVALID_INPUT;
    }

    if ((tmac == NULL)) {
        return UCL_INVALID_OUTPUT;
    }

    if ((len % UCL_AES_BLOCKSIZE) != 0) {
        return UCL_INVALID_ARG;
    }

    if (tmac_byteLen > UCL_AES_BLOCKSIZE) {
        return UCL_INVALID_ARG;
    }

    ucl_aes_cbc_mac_init(&ctx, key, keylen);
    _ucl_increment_dst = UCL_NOP;
    ucl_aes_cbc_mac_core(&ctx, src, len);
    ucl_aes_cbc_mac_finish(tmac, tmac_byteLen, &ctx);

    return(UCL_OK);
}


/*============================================================================ */
/** < b >AES-CBC-MAC Init < /b >.
 * Initialise AES CBC MAC Context.
 *
 * @param[out] ctx  Pointer to the context
 * @param[in]   key  Pointer to the AES key
 *
 * @return Error code
 *
 * @retval #UCL_OK       if no error occurred
 * @retval #UCL_INVALID_INPUT  if the input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT  if the output is the pointer #NULL
 *
 * @ingroup UCL_CBC_MAC
 */

int ucl_aes_cbc_mac_init(ucl_aes_ctx_t *ctx, u8 *key, u32 keylen)
{
    u8 ivnull[UCL_AES_BLOCKSIZE];
    int i;
    if (ctx == NULL) {
        return UCL_INVALID_OUTPUT;
    }
    if (key == NULL) {
      return UCL_INVALID_INPUT;
    }

    memset(ivnull, 0, UCL_AES_BLOCKSIZE);
    ctx->mode = UCL_CIPHER_ENCRYPT;

    return(ucl_aes_cbc_init(ctx, key, keylen, ivnull, UCL_CIPHER_ENCRYPT));
}


/*============================================================================ */
/** < b >AES-CBC-MAC Core < /b >.
 * Process the Data.
 *
 * @param[in, out] ctx     Pointer to the context
 * @param[in]     data   Pointer to the data
 * @param[in]    data_byteLen   Byte length
 *
 * @pre The byte length must be a multiple of 8.
 *
 * @return Error code
 *
 * @retval #UCL_OK       if no error occurred
 * @retval #UCL_INVALID_INPUT  if one of the input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT  if the output is the pointer #NULL
 * @retval #UCL_INVALID_ARG   if the byte length is not a multiple of 8
 *
 * @ingroup UCL_CBC_MAC
 */
int ucl_aes_cbc_mac_core(ucl_aes_ctx_t *ctx, u8 *src, u32 len)
{
    u8 dst[1];

    return(ucl_aes_cbc_core(dst, ctx, src, len));
}

/** < b >AES-CBC-MAC Finish < /b >.
 * Zeroize the context and return result.
 *
 * @param[out]   tmac     Pointer to the mac
 * @param[in]    tmac_byteLen  MAC byte length
 * @param[in, out] ctx      Pointer to the context
 *
 * @return Error code
 *
 * @retval #UCL_OK       if no error occurred
 * @retval #UCL_INVALID_OUTPUT  if one of the outputs is the pointer #NULL
 *
 * @ingroup UCL_CBC_MAC
 */
int ucl_aes_cbc_mac_finish(u8 *tmac, u32 tmac_byteLen, ucl_aes_ctx_t *ctx)
{
    if (ctx == NULL) {
        return UCL_INVALID_OUTPUT;
    }

    memcpy(tmac, ctx->memory, tmac_byteLen);

    return(UCL_OK);
}
