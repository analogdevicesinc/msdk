/*============================================================================
 *
 * ucl_uaes_cbc.h
 *
 *==========================================================================*/
/*============================================================================
 *
 * Copyright © 2009 Innova Card.
 * All Rights Reserved. Do not disclose.
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
#ifndef _UCL_UAES_CBC_H_
#define _UCL_UAES_CBC_H_

#include "ucl_uaes.h"

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */


/** @file ucl_uaes_cbc.h
 * @defgroup UCL_CBC_UAES USIP AES CBC
 * Encrypt / Decrypt with USIP&reg; AES in CBC (Cipher Block Chaining) mode.
 *
 * @par Header:
 * @link ucl_uaes_cbc.h ucl_uaes_cbc.h @endlink
 *
 * @warning If you use those functions you erase the USIP&reg; AES context.
 *
 * @note A USIP&reg; AES interface must be defined.
 *
 * @ingroup UCL_CBC
 */


/*============================================================================*/
/** <b>USIP&reg; AES-CBC</b>.
 * Complete process.
 *
 * @pre @li The data byte length must be a multiple of 16.
 *   @li The key length is 16 bytes.
 *
 * @param[out] dst  Pointer to the output data
 * @param[in]  key  Pointer to the USIP&reg; AES Key
 * @param[in]  IV   Pointer to the initialization vector
 * @param[in]  src  Pointer to the input data
 * @param[in]  len  Data byte length
 * @param[in]  mode The mode (Encryption/Decryption):
 *                  @li #UCL_CIPHER_ENCRYPT
 *                  @li #UCL_CIPHER_DECRYPT
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_INPUT  One of the input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT One of the output is the pointer #NULL
 * @retval #UCL_INVALID_ARG    @p len is not a multiple of 16
 *
 * @ingroup UCL_CBC_UAES
 */
int ucl_uaes_cbc(u8 *dst, u8 *src, u8 *key, u8 *IV,
                 u32 len, int mode);


/*============================================================================*/
/** <b>USIP&reg; AES-CBC Init</b>.
 * Initialize USIP&reg; AES CBC Context.
 *
 * @pre The key length is 16 bytes.
 *
 * @param[out] ctx  Pointer to the context
 * @param[in]  key  Pointer to the USIP&reg; AES Key
 * @param[in]  IV   Pointer to the initialization vector
 * @param[in]  mode The mode (Encryption/Decryption) :
 *                      @li #UCL_CIPHER_ENCRYPT
 *                      @li #UCL_CIPHER_DECRYPT
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_INPUT  The input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT The output is the pointer #NULL
 * @retval #UCL_INVALID_MODE   The mode is not one of those described
 *
 * @ingroup UCL_CBC_UAES */
int ucl_uaes_cbc_init(ucl_uaes_ctx_t *ctx, u8 *key, u8 *IV,
                      int mode);


/*============================================================================*/
/** <b>USIP&reg; AES-CBC Core</b>.
 * Process the Data.
 *
 * @pre The data byte length must be a multiple of 16.
 *
 * @param[out]    dst Pointer to the output data
 * @param[in,out] ctx Pointer to the context
 * @param[in]     src Pointer to the data
 * @param[in]     len Data byte length
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_INPUT  One of the input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT The output is the pointer #NULL
 * @retval #UCL_INVALID_ARG    The byte length is not a multiple of 16
 *
 * @ingroup UCL_CBC_UAES */
int ucl_uaes_cbc_core(u8 *dst, ucl_uaes_ctx_t *ctx, u8 *src, u32 len);


/*============================================================================*/
/** <b>USIP&reg; AES-CBC Finish</b>.
 * Zeroize the context.
 *
 * @param[out,in] ctx Pointer to the context
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_OUTPUT The output is the pointer #NULL
 *
 * @ingroup UCL_CBC_UAES */
int ucl_uaes_cbc_finish(ucl_uaes_ctx_t *ctx);


#ifdef __cplusplus
}
#endif /* _ cplusplus  */

#endif /* _UCL_UAES_CBC_H_ */
