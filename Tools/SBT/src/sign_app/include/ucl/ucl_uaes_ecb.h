/*============================================================================
 *
 * ucl_uaes_ecb.h
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
#ifndef _UCL_UAES_ECB_H_
#define _UCL_UAES_ECB_H_

#include "ucl_uaes.h"

#ifdef _cplusplus
extern "C" {
#endif /* _ cplusplus  */

/** @file ucl_uaes_ecb.h
 * @defgroup UCL_UAES_ECB USIP AES ECB
 * Encrypt/Decrypt with USIP&reg; AES in ECB (Electronic Codebook) mode.
 *
 * @par Header:
 * @link ucl_uaes_ecb.h ucl_uaes_ecb.h @endlink
 *
 * @warning If you use those functions you erase the USIP&reg; AES context.
 *
 * @note A USIP&reg; AES interface must be defined.
 *
 * @ingroup UCL_ECB
 */

/*============================================================================*/
/** <b>USIP&reg; AES-ECB</b>.
 * Complete process.
 *
 * @pre @li The data byte length must be a multiple of 16.
 *   @li The key length is 16 bytes.
 *
 * @param[out] dst  Pointer to the output data
 * @param[in]  key  Pointer to the USIP&reg; AES Key
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
 * @ingroup UCL_UAES_ECB */
int ucl_uaes_ecb(u8* dst, u8* src, u8* key, u32 len, int mode);

/*============================================================================*/
/** <b>USIP&reg; AES-ECB Init</b>.
 * Initialize USIP&reg; AES ECB Context.
 *
 * @pre The key length is 16 bytes.
 *
 * @param[out] ctx  Pointer to the context
 * @param[in]  key  Pointer to the USIP&reg; AES Key
 * @param[in]  mode The mode (Encryption/Decryption):
 *                  @li #UCL_CIPHER_ENCRYPT
 *                  @li #UCL_CIPHER_DECRYPT
 *
 * @return Error code
 *
 * @retval #UCL_OK    No error occurred
 * @retval #UCL_INVALID_INPUT The input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT The output is the pointer #NULL
 * @retval #UCL_INVALID_MODE The mode is not one of those described
 *
 * @ingroup UCL_UAES_ECB */
int ucl_uaes_ecb_init(ucl_uaes_ctx_t* ctx, u8* key, int mode);

/*============================================================================*/
/** <b>USIP&reg; AES-ECB Core</b>.
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
 * @ingroup UCL_UAES_ECB */
int ucl_uaes_ecb_core(u8* dst, ucl_uaes_ctx_t* ctx, u8* src, u32 len);

/*============================================================================*/
/** <b>USIP&reg; AES-ECB Finish</b>.
 * Zeroize the context.
 *
 * @param[out,in] ctx Pointer to the context
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_OUTPUT The output is the pointer #NULL
 *
 * @ingroup UCL_UAES_ECB */
int ucl_uaes_ecb_finish(ucl_uaes_ctx_t* ctx);

#ifdef _cplusplus
}
#endif /* _ cplusplus  */

#endif /* _UCL_UAES_ECB_H_ */
