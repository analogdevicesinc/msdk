/*============================================================================
 *
 * ucl_des_cbc.h
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
 * Purpose : Cipher Block Chaining
 *
 *==========================================================================*/
#ifndef _UCL_DES_CBC_H_
#define _UCL_DES_CBC_H_

#include "ucl_des.h"

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */

/** @file ucl_des_cbc.h
 * @defgroup UCL_CBC_DES DES CBC
 * Encrypt / Decrypt with DES in CBC (Cipher Block Chaining) mode.
 *
 * @par Header:
 * @link ucl_des_cbc.h ucl_des_cbc.h @endlink
 *
 * @ingroup UCL_CBC
 */


/*============================================================================*/
/** <b>DES-CBC</b>.
 * Complete process.
 *
 * @pre @li The byte length must be a multiple of 8.
 *      @li Input and Output Data have the same length.
 *      @li The key length is 8 bytes.
 *
 * @param[out] dataOut      Pointer to the output data
 * @param[in]  dataIn       Pointer to the input data
 * @param[in]  key          Pointer to the DES Key
 * @param[in]  IV           Pointer to the initialization vector
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
 * @retval #UCL_INVALID_ARG    If @p data_byteLen is not a multiple of 8
 *
 * @see UCL_DES
 *
 * @ingroup UCL_CBC_DES
 */
int ucl_des_cbc(u8 *dataOut, u8 *dataIn, u8 *key, u8 *IV,
                u32 data_byteLen, int mode);


/*============================================================================*/
/** <b>DES-CBC Init</b>.
 * Initialize DES CBC Context.
 *
 * @pre The key length is 8 bytes.
 *
 * @param[out] ctx  Pointer to the context
 * @param[in]  key  Pointer to the DES Key
 * @param[in]  IV   Pointer to the initialization vector
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
 * @ingroup UCL_CBC_DES
 */
int ucl_des_cbc_init(ucl_des_ctx_t *ctx, u8 *key, u8 *IV,
                     int mode);


/*============================================================================*/
/** <b>DES-CBC Core</b>.
 * Process the Data.
 *
 * @param[out]    dataOut      Pointer to the output data
 * @param[in,out] ctx          Pointer to the context
 * @param[in]     dataIn       Pointer to the data
 * @param[in]     data_byteLen Data byte length
 *
 * @pre @li The data byte length must be a multiple of 8.
 *      @li Input and output data have the same length.
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_INPUT  If one of the input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT If the output is the pointer #NULL
 * @retval #UCL_INVALID_ARG    If the byte length is not a multiple of 8
 *
 * @ingroup UCL_CBC_DES
 */
int ucl_des_cbc_core(u8 *dataOut, ucl_des_ctx_t *ctx,
                     u8 *dataIn, u32 data_byteLen);


/*============================================================================*/
/** <b>DES-CBC Finish</b>.
 * Zeroize the context.
 *
 * @param[out,in] ctx  Pointer to the context
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_OUTPUT If the output is the pointer #NULL
 *
 * @ingroup UCL_CBC_DES
 */
int ucl_des_cbc_finish(ucl_des_ctx_t *ctx);

#ifdef __cplusplus
}
#endif /* _ cplusplus  */

#endif /* _UCL_DES_CBC_ */
