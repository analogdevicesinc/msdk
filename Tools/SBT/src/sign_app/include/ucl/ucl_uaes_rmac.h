/*============================================================================
 *
 * ucl_uaes_rmac.h
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
#ifndef _UCL_UAES_RMAC_H_
#define _UCL_UAES_RMAC_H_

#include "ucl_uaes.h"

#ifdef _cplusplus
extern "C" {
#endif /* _ cplusplus  */


/** @internal @file ucl_uaes_rmac.h
 * @defgroup UCL_UAES_RMAC USIP AES RMAC (Mode 1)
 * RMAC is a randomized message authentication code based on the AES-CBC-MAC.
 *
 * @par Header:
 * @link ucl_uaes_rmac.h ucl_uaes_rmac.h @endlink
 *
 * The 'RMAC' of a message is the output is @a B||@a R, where @a B is the mac and
 * @a R a random value generated inside the function or given as parameter (the
 * parameter @p rand define this choice).
 *
 * @note The message must be padded before process. Use a padding method
 * described in ISO/IEC 9797.
 *
 * @ingroup UCL_CBC_MAC
 */


/*============================================================================*/
/** <b>USIP&reg; AES RMAC (mode 1)</b>.
 * USIP&reg; AES RMAC complete process.
 *
 * @param[out]    rmac         The mac of the data (B)
 * @param[in,out] rand         The random value of the rmac (R)
 * @param[in]     key1         The first AES 128-bit Key
 * @param[in]     key2         The second AES 128-bit Key
 * @param[in]     data         The padded data
 * @param[in]     data_bytelen Data byte length
 *
 * @warning The data be padded before process. Use a padding method described
 * in ISO/IEC 9797-1:1999.
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_INPUT  If one of the input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT If one of the output is the pointer #NULL
 * @retval #UCL_INVALID_ARG    If @p data_byteLen = 0
 *
 * @ingroup UCL_UAES_RMAC
 */
int __API__ ucl_uaes_rmac(u8 *rmac, u8 *rand, u8 *key1, u8 *key2, u8 *data,
                  u32 data_bytelen);


/*============================================================================*/
/** <b>USIP&reg; AES RMAC Init</b>.
 * Initialize USIP&reg; AES RMAC Context.
 *
 * @pre The key length is 16 bytes.
 *
 * @param[out] ctx  Pointer to the context
 * @param[in]  key  Pointer to the 128-bit AES Key
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_INPUT  If the input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT If the output is the pointer #NULL
 *
 * @ingroup UCL_UAES_RMAC
 */
int __API__ ucl_uaes_rmac_init(ucl_uaes_ctx_t *ctx, u8 *key);


/*============================================================================*/
/** <b>USIP&reg; AES RMAC Core</b>.
 * Process the Data.
 *
 * @pre The data byte length must be a multiple of 16.
 *
 * @param[in,out] ctx          Pointer to the context
 * @param[in]     data         Pointer to the data
 * @param[in]     data_byteLen Data byte length
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_INPUT  If one of the input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT If the output is the pointer #NULL
 * @retval #UCL_INVALID_ARG    If the data byte length is not a multiple of 16
 *
 * @ingroup UCL_UAES_RMAC
 */
int __API__ ucl_uaes_rmac_core(ucl_uaes_ctx_t *ctx, u8 *data,
                       u32 data_byteLen);


/*============================================================================*/
/** <b>USIP&reg; AES RMAC Finish</b>.
 * Zeroize the context and return result.
 *
 * @param[out]    rmac Pointer to the RMAC
 * @param[in,out] ctx  Pointer to the context
 * @param[in]     rand Pointer to the random value
 * @param[in]     key  Pointer to the second key
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_OUTPUT If one of the outputs is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  If @p key is the pointer #NULL
 *
 * @ingroup UCL_UAES_RMAC
 */
int __API__ ucl_uaes_rmac_finish(u8 *rmac, ucl_uaes_ctx_t *ctx,
                         u8 *rand, u8 *key);


#ifdef _cplusplus
}
#endif /* _ cplusplus  */

#endif /*_UCL_UAES_RMAC_H_*/
