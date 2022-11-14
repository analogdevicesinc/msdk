/*******************************************************************************
* Copyright (C) 2018 Maxim Integrated Products, Inc., All rights Reserved.
* 
* This software is protected by copyright laws of the United States and
* of foreign countries. This material may also be protected by patent laws
* and technology transfer regulations of the United States and of foreign
* countries. This software is furnished under a license agreement and/or a
* nondisclosure agreement and may only be used or reproduced in accordance
* with the terms of those agreements. Dissemination of this information to
* any party or parties not specified in the license agreement and/or
* nondisclosure agreement is expressly prohibited.
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
*******************************************************************************
*/
#ifndef _UCL_AES_CBC_MAC_H_
#define _UCL_AES_CBC_MAC_H_

#include "ucl/ucl_aes.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus  */

/** @file ucl_aes_cbc_mac.h
 * @defgroup UCL_CBC_MAC_AES AES CBC_MAC
 * AES in CBC_MAC (Cipher Block Chaining-MAC) mode.
 *
 * @par Header:
 * @link ucl_aes_cbc_mac.h ucl_aes_cbc_mac.h @endlink
 *
 * @ingroup UCL_CBC_MAC
 */

/** <b>AES-CBC_MAC</b>.
 * AES in CBC_MAC (Cipher Block Chaining MAC) mode.
 *
 *
 * @param[out] tmac    Pointer to the outputted MAC
 * @param[in] tmac_byteLen    outputted MAC byte length
 * @param[in]  src    Pointer to the input data
 * @param[in]  len    Data byte length
 * @param[in]  key    Pointer to the AES Key
 * @param[in]  keylen Key byte length:
 *                        @li #UCL_AES_KEYLEN_128
 *                        @li #UCL_AES_KEYLEN_192
 *                        @li #UCL_AES_KEYLEN_256
 *
 * @return Error code
 *
 * @retval #UCL_OK             no error occurred
 * @retval #UCL_INVALID_INPUT  one of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT one of the output is the pointer NULL
 * @retval  #UCL_INVALID_ARG   @p len is not a multiple of
 *                             #UCL_AES_BLOCKSIZE or @p keylen is invalid
 *
 * @ingroup UCL_CBC_MAC_AES
 */
  int ucl_aes_cbc_mac(u8 *tmac, u8 tmac_byteLen, u8 *src, u32 len, u8 *key, u32 keylen);

/** <b>AES-CBC_MAC Init</b>.
 * Initialise AES CBC_MAC Context.
 *
 * @param[out] ctx    Pointer to the context
 * @param[in]  key    Pointer to the AES Key
 * @param[in]  keylen Key byte length:
 *                        @li #UCL_AES_KEYLEN_128
 *                        @li #UCL_AES_KEYLEN_192
 *                        @li #UCL_AES_KEYLEN_256
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_INPUT  The input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT The output is the pointer #NULL
 * @retval #UCL_INVALID_ARG    @p keylen is invalid
 *
 * @ingroup UCL_CBC_MAC_AES 
 */
int ucl_aes_cbc_mac_init(ucl_aes_ctx_t *ctx, u8 *key, u32 keylen);

/** <b>AES-CBC_MAC Core</b>.
 * Process the Data.
 *
 * @param[out,in] ctx  Pointer to the context
 * @param[in]     src  Pointer to the data
 * @param[in]     len  Data byte length
 *
 * @pre The byte length must be a multiple of #UCL_AES_BLOCKSIZE.
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_INPUT  One of the input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT The output is the pointer #NULL
 * @retval #UCL_INVALID_ARG    The data byte length is not a multiple of
 *                             #UCL_AES_BLOCKSIZE
 *
 * @ingroup UCL_CBC_MAC_AES 
 */
int ucl_aes_cbc_mac_core(ucl_aes_ctx_t *ctx, u8 *src, u32 len);

/**<b>AES-CBC_MAC Finish</b>.
 * Output the MAC and zeroize the context.
 *
 * @param[out] tmac    Pointer to the outputted MAC
 * @param[in] tmac_byteLen    outputted MAC byte length
 * @param[out,in] ctx Pointer to the context
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_OUTPUT The output is the pointer #NULL
 *
 * @ingroup UCL_CBC_MAC_AES 
 */
  int ucl_aes_cbc_mac_finish(u8 *tmac, u32 tmac_byteLen, ucl_aes_ctx_t *ctx);

#ifdef __cplusplus
}
#endif /* __cplusplus  */

#endif /*_UCL_AES_CBC_MAC_H_*/
