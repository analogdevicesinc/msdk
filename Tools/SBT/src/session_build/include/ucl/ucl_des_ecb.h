/*============================================================================
 *
 * ucl_des_ecb.h
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
#ifndef _UCL_DES_ECB_H_
#define _UCL_DES_ECB_H_

#include "ucl_des.h"

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */

/** @file ucl_des_ecb.h
 * @defgroup UCL_DES_ECB DES ECB
 * Encrypt/Decrypt with DES in ECB (Electronic Codebook) mode.
 *
 * @par Header:
 * @link ucl_des_ecb.h ucl_des_ecb.h @endlink
 *
 * @ingroup UCL_ECB
 */

/*============================================================================*/
/** <b>DES-ECB</b>.
 * Complete process.
 *
 * @pre @li The byte length must be a multiple of 8.
 * @li Input and Output Data have the same length.
 *    @li The key length is 8 bytes.
 *
 * @param[out] dataOut      Pointer to decrypted/encryted message
 * @param[in]  dataIn       Pointer to encrypted/decrypted message
 * @param[in]  key          Pointer to the DES Key
 * @param[in]  data_byteLen Data byte length
 * @param[in]  mode         The mode (Encryption/Decryption):
 *                              @li #UCL_CIPHER_ENCRYPT
 *                              @li #UCL_CIPHER_DECRYPT
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_INPUT  If one of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT If one of the output is the pointer NULL
 * @retval #UCL_INVALID_MODE   If the @p mode is invalid
 * @retval #UCL_INVALID_ARG    If the data byte length is not a multiple of 8
 *
 * @see UCL_DES
 *
 * @ingroup UCL_DES_ECB
 */
int ucl_des_ecb(u8* dataOut, u8* dataIn, u8* key, u32 data_byteLen, int mode);

/*============================================================================*/
/** <b>DES-ECB Init</b>.
 * Initialize DES ECB Context.
 *
 * @pre The key length is 8 bytes.
 *
 * @param[out] ctx  Pointer to the context
 * @param[in]  key  Pointer to the DES Key
 * @param[in]  mode The mode (Encryption/Decryption) :
 *                      @li #UCL_CIPHER_ENCRYPT
 *                      @li #UCL_CIPHER_DECRYPT
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_INPUT  If the input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT If the output is the pointer #NULL
 * @retval #UCL_INVALID_MODE   If the mode is not one of those described
 *
 * @ingroup UCL_DES_ECB
 */
int ucl_des_ecb_init(ucl_des_ctx_t* ctx, u8* key, int mode);

/*============================================================================*/
/** <b>DES-ECB Core</b>.
 * Process the Data.
 *
 * @pre @li The data byte length must be a multiple of 8.
 *  @li Input and output data have the same length.
 *
 * @param[out]    dataOut      Pointer to the output data
 * @param[in,out] ctx          Pointer to the context
 * @param[in]     dataIn       Pointer to the data
 * @param[in]     data_byteLen Data byte length
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_INPUT  If one of the input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT If the output is the pointer #NULL
 * @retval #UCL_INVALID_ARG    If the byte length is not a multiple of 8
 *
 * @ingroup UCL_DES_ECB
 */
int ucl_des_ecb_core(u8* dataOut, ucl_des_ctx_t* ctx, u8* dataIn, u32 data_byteLen);

/*============================================================================*/
/** <b>DES-ECB Finish</b>.
 * Zeroize the context.
 *
 * @param[out, in] ctx Pointer to the context
 *
 * @return Error code
 * 
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_OUTPUT If the output is the pointer #NULL
 *
 * @ingroup UCL_DES_ECB
 */
int ucl_des_ecb_finish(ucl_des_ctx_t* ctx);

#ifdef __cplusplus
}
#endif /* _ cplusplus  */

#endif /* _UCL_DES_ECB_H_ */
