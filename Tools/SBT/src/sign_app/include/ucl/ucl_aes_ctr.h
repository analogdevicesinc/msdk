/*============================================================================
 *
 * ucl_aes_ctr.h [30-mar-06]
 *
 *==========================================================================*/
/*============================================================================
 *
 * Copyright © 2009 Innova Card. All Rights Reserved. Do not disclose.
 *
 * This software is the confidential and proprietary information of
 * Innova Card ("Confidential Information"). You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered
 * into with Innova Card.
 *
 * Innova Card makes no representations or warranties about the suitability of
 * the software, either express or implied, including but not limited to
 * the implied warranties of merchantability, fitness for a particular purpose,
 * or non-infrigement. Innova Card shall not be liable for any damages suffered
 * by licensee as the result of using, modifying or distributing this software
 * or its derivatives.
 *
 *==========================================================================*/
/*============================================================================
 *
 * Purpose :
 *
 *==========================================================================*/
#ifndef _UCL_AES_CTR_H_
#define _UCL_AES_CTR_H_

#include "ucl/ucl_aes.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus  */

/** @file ucl_aes_ctr.h
 * @defgroup UCL_CTR_AES AES CTR
 * Encrypt / Decrypt with AES in CTR (Electronic Codebook) mode.
 *
 * @par Header:
 * @link ucl_aes_ctr.h ucl_aes_ctr.h @endlink
 *
 * @ingroup UCL_CTR
 */


/*============================================================================*/
/** <b>AES-CTR</b>.
 * Complete process.
 *
 * @param[out] dst    Pointer to the output data
 * @param[in]  src    Pointer to the input data
 * @param[in]  len    Data byte length
 * @param[in/out] counter: pointer to the 128-bit/16-byte counter array value
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
 * @ingroup UCL_CTR_AES
 */
  int ucl_aes_ctr(u8 *dst, u8 *src, u32 len, u8 *counter, u8 *key, u32 keylen, int mode);


/*============================================================================*/
/** <b>AES-CTR Init</b>.
 * Initialise AES CTR Context.
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
 * @ingroup UCL_CTR_AES
 */
int ucl_aes_ctr_init(ucl_aes_ctx_t *ctx, u8 *key, u32 keylen, int mode);


/*============================================================================*/
/** <b>AES-CTR Core</b>.
 * Process the Data.
 *
 * @param[out]    dst Pointer to the output data
 * @param[out,in] ctx AES context
 * @param[out,in] counter CTR mode counter
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
 * @ingroup UCL_CTR_AES
 */
  int ucl_aes_ctr_core(u8 *dst, ucl_aes_ctx_t *ctx, u8 *src, u32 len,u8 *counter);


/*============================================================================*/
/** <b>AES-CTR Finish</b>.
 * Zeroize the context.
 *
 * @param[out,in] ctx Pointer to the context
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_OUTPUT The output is the pointer #NULL
 *
 * @ingroup UCL_CTR_AES
 */
int ucl_aes_ctr_finish(ucl_aes_ctx_t *ctx);

/** <b>AES-CTR Core</b>.
 * Process the Data.
 *
 * @param[out]  dst Pointer to the output data
 * @param[out,in] ctx AES context
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
 * @ingroup UCL_CTR_AES */
  int ucl_aes_ctr_core_context(u8 *dst, ucl_aes_ctx_t *ctx, u8 *src, u32 len,u8 *counter);


#ifdef __cplusplus
}
#endif /* __cplusplus  */

#endif /*_UCL_AES_CTR_H_*/
