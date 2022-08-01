/*============================================================================
 *
 * ucl_des_cbc_mac.h
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
#ifndef _UCL_DES_CBC_MAC_H_
#define _UCL_DES_CBC_MAC_H_

#include "ucl_des.h"

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */

/** @file ucl_des_cbc_mac.h
 * @defgroup UCL_DES_CBC_MAC DES CBC MAC
 * ISO/IEC 9797-1:1999 MAC Algorithm 1 with DES.
 *
 * @par Header:
 * @link ucl_des_cbc_mac.h ucl_des_cbc_mac.h @endlink
 *
 * @note The message must be padded before process. Use a padding method
 * described in ISO/IEC 9797.
 *
 * @ingroup UCL_CBC_MAC
 */

/*============================================================================*/
/** <b>DES-CBC-MAC</b>.
 * Complete process.
 *
 * @pre @li The data byte length must be a multiple of 8.
 *      @li The @p mac byte length must be less or equal than 8 bytes.
 *      @li The key length is 8 bytes.
 *
 * @param[out] mac          Pointer to the mac
 * @param[in]  mac_byteLen  MAC byte length
 * @param[in]  key          Pointer to the DES Key
 * @param[in]  data         Pointer to the input data
 * @param[in]  data_byteLen Data byte length
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_INPUT  If one of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT If one of the output is the pointer NULL
 * @retval #UCL_INVALID_ARG    If @p data_byteLen is not a multiple of 8 or
 *                             If @p mac_byteLen is greater than 8
 *
 * @ingroup UCL_DES_CBC_MAC
 */
int ucl_des_cbc_mac(u8* mac, u32 mac_byteLen, u8* key, u8* data, u32 data_byteLen);

/*============================================================================*/
/** <b>DES-CBC-MAC Init</b>.
 * Initialize DES CBC MAC Context.
 *
 * @pre The key length is 8 bytes.
 *
 * @param[out] ctx Pointer to the context
 * @param[in]  key Pointer to the DES Key
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_INPUT  If the input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT If the output is the pointer #NULL
 *
 * @ingroup UCL_DES_CBC_MAC
 */
int ucl_des_cbc_mac_init(ucl_des_ctx_t* ctx, u8* key);

/*============================================================================*/
/** <b>DES-CBC-MAC Core</b>.
 * Process the Data.
 *
 * @pre The data byte length must be a multiple of 8.
 *
 * @param[out,in] ctx          Pointer to the context
 * @param[in]     data         Pointer to the data
 * @param[in]     data_byteLen Data byte length
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_INPUT  If one of the input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT If the output is the pointer #NULL
 * @retval #UCL_INVALID_ARG    If the data byte length is not a multiple of 8
 *
 * @ingroup UCL_DES_CBC_MAC
 */
int ucl_des_cbc_mac_core(ucl_des_ctx_t* ctx, u8* data, u32 data_byteLen);

/*============================================================================*/
/** <b>DES-CBC-MAC Finish</b>.
 * Zeroize the context and return result.
 *
 * @pre The @p mac byte length must be less or equal than 8 bytes.
 *
 * @param[out]    mac         Pointer to the mac
 * @param[in]     mac_byteLen MAC byte length
 * @param[in,out] ctx         Pointer to the context
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_OUTPUT If one of the outputs is the pointer #NULL
 * @retval #UCL_INVALID_ARG    If MAC byte length is greater than 8 bytes
 *
 * @ingroup UCL_DES_CBC_MAC
 */
int ucl_des_cbc_mac_finish(u8* mac, u32 mac_byteLen, ucl_des_ctx_t* ctx);

#ifdef __cplusplus
}
#endif /* _ cplusplus  */

#endif /* _UCL_DES_CBC_MAC_H_ */
