/*============================================================================
 *
 * ucl_3des_ecb.h
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
#ifndef _UCL_3DES_ECB_H_
#define _UCL_3DES_ECB_H_

#include "ucl_3des.h"

#ifdef __cplusplus
extern "C" {
#endif /* __ cplusplus  */

/** @file ucl_3des_ecb.h
 * @defgroup UCL_3DES_ECB 3DES ECB
 * Encrypt/Decrypt with 3DES in ECB (Electronic Codebook) mode.
 *
 * @par Header:
 * @link ucl_3des_ecb.h ucl_3des_ecb.h @endlink
 *
 * @ingroup UCL_ECB
 */

/*============================================================================*/
/** <b>3DES-ECB</b>.
 * Complete process.
 *
 * @pre @li The byte length must be a multiple of 8.
 *      @li Input and Output Data have the same length.
 *      @li The key length is 16 or 24 bytes (See @link UCL_3DES 3DES @endlink).
 *
 * @param[out] dataOut      Pointer to decrypted/encryted message
 * @param[in]  dataIn       Pointer to encrypted/decrypted message
 * @param[in]  key          Pointer to the 3DES Key
 * @param[in]  data_byteLen Data byte length
 * @param[in]  mode         The mode (Encryption/Decryption) :
 *                              @li #UCL_CIPHER_ENCRYPT
 *                              @li #UCL_CIPHER_DECRYPT
 *
 * @return Error code
 *
 * @retval #UCL_OK             if no error occurred
 * @retval #UCL_INVALID_INPUT  if one of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT if one of the output is the pointer NULL
 * @retval #UCL_INVALID_MODE   if the @p mode is invalid
 * @retval #UCL_INVALID_ARG    if the data byte length is not a multiple of 8
 *
 * @see UCL_3DES
 *
 * @ingroup UCL_3DES_ECB
 */
int ucl_3des_ecb(u8* dataOut, u8* dataIn, u8* key, u32 data_byteLen, int mode);

/*============================================================================*/
/** <b>3DES-ECB Init</b>.
 * Initialize 3DES ECB Context.
 *
 * @pre The key length is 16 or 24 bytes (See @link UCL_3DES 3DES @endlink).
 *
 * @param[out] ctx  Pointer to the context
 * @param[in]  key  Pointer to the 3DES Key
 * @param[in]  mode The mode (Encryption/Decryption) :
 *                      @li #UCL_CIPHER_ENCRYPT
 *                      @li #UCL_CIPHER_DECRYPT
 *
 * @return Error code
 *
 * @retval #UCL_OK    if no error occurred
 * @retval #UCL_INVALID_INPUT if the input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT  if the output is the pointer #NULL
 * @retval #UCL_INVALID_MODE if the mode is not one of those 3described
 *
 * @ingroup UCL_3DES_ECB
 */
int ucl_3des_ecb_init(ucl_3des_ctx_t* ctx, u8* key, int mode);

/*============================================================================*/
/** <b>3DES-ECB Core</b>.
 * Process the Data.
 *
 * @pre @li The byte length must be a multiple of 8.
 *      @li Input and Output Data have the same length.
 *
 * @param[out]    dataOut      Pointer to the output data
 * @param[in,out] ctx          Pointer to the context
 * @param[in]     dataIn       Pointer to the data
 * @param[in]     data_byteLen Data byte length
 *
 * @return Error code
 *
 * @retval #UCL_OK             if no error occurred
 * @retval #UCL_INVALID_INPUT  if one of the input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT if the output is the pointer #NULL
 * @retval #UCL_INVALID_ARG    if the data byte length is not a multiple of 8
 *
 * @ingroup UCL_3DES_ECB
 */
int ucl_3des_ecb_core(u8* dataOut, ucl_3des_ctx_t* ctx, u8* dataIn, u32 data_byteLen);

/*============================================================================*/
/** <b>3DES-ECB Finish</b>.
 * Zeroize the context.
 *
 * @param[out,in] ctx Pointer to the context
 *
 * @return Error code
 *
 * @retval #UCL_OK             if no error occurred
 * @retval #UCL_INVALID_OUTPUT if the output is the pointer #NULL
 *
 * @ingroup UCL_3DES_ECB
 */
int ucl_3des_ecb_finish(ucl_3des_ctx_t* ctx);

#ifdef __cplusplus
}
#endif /* __cplusplus  */

#endif /* _UCL_3DES_ECB_H_ */
