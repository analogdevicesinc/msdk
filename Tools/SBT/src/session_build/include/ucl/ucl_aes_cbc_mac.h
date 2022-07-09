/*============================================================================
 *
 * ucl_aes_cbc_mac.h [30-mar-06]
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
#ifndef _UCL_AES_CBC_MAC_H_
#define _UCL_AES_CBC_MAC_H_

#include "ucl/ucl_aes.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus  */

/** @file ucl_aes_cbc_mac.h
 * @defgroup UCL_CBC_MAC_AES AES CBC_MAC
 * Encrypt / Decrypt with AES in CBC_MAC (Cipher Block Chaining) mode.
 *
 * @par Header:
 * @link ucl_aes_cbc_mac.h ucl_aes_cbc_mac.h @endlink
 *
 * @ingroup UCL_CBC_MAC
 */

/*============================================================================*/
/** <b>AES-CBC_MAC</b>.
 * Encrypt / Decrypt with AES in CBC_MAC (Cipher Block Chaining) mode.
 *
 *
 * @param[out] dst    Pointer to the output data
 * @param[in]  src    Pointer to the input data
 * @param[in]  len    Data byte length
 * @param[in]  key    Pointer to the AES Key
 * @param[in]  keylen Key byte length:
 *                        @li #UCL_AES_KEYLEN_128
 *                        @li #UCL_AES_KEYLEN_192
 *                        @li #UCL_AES_KEYLEN_256
 * @param[in] mode    The mode (Encryption/Decryption):
 *                        @li #UCL_CIPHER_ENCRYPT
 *                        @li #UCL_CIPHER_DECRYPT
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
int ucl_aes_cbc_mac(u8 *dst, u8 *src, u32 len, u8 *key, u32 keylen, int mode);


/*============================================================================*/
/** <b>AES-CBC_MAC Init</b>.
 * Initialise AES CBC_MAC Context.
 *
 * @param[out] ctx    Pointer to the context
 * @param[in]  key    Pointer to the AES Key
 * @param[in]  keylen Key byte length:
 *                        @li #UCL_AES_KEYLEN_128
 *                        @li #UCL_AES_KEYLEN_192
 *                        @li #UCL_AES_KEYLEN_256
 * @param[in]  mode   The mode (Encryption/Decryption) :
 *                        @li #UCL_CIPHER_ENCRYPT
 *                        @li #UCL_CIPHER_DECRYPT
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_INPUT  The input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT The output is the pointer #NULL
 * @retval #UCL_INVALID_ARG    @p keylen is invalid
 * @retval #UCL_INVALID_MODE   The mode is not one of those described
 *
 * @ingroup UCL_CBC_MAC_AES 
 */
int ucl_aes_cbc_mac_init(ucl_aes_ctx_t *ctx, u8 *key, u32 keylen, int mode);


/*============================================================================*/
/** <b>AES-CBC_MAC Core</b>.
 * Process the Data.
 *
 * @param[out]    dst  Pointer to the processed data
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
int ucl_aes_cbc_mac_core(u8 *dst, ucl_aes_ctx_t *ctx, u8 *src, u32 len);


/*============================================================================*/
/**<b>AES-CBC_MAC Finish</b>.
 * Zeroize the context.
 *
 * @param[out,in] ctx Pointer to the context
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_OUTPUT The output is the pointer #NULL
 *
 * @ingroup UCL_CBC_MAC_AES 
 */
int ucl_aes_cbc_mac_finish(ucl_aes_ctx_t *ctx);


#ifdef __cplusplus
}
#endif /* __cplusplus  */

#endif /*_UCL_AES_CBC_MAC_H_*/
