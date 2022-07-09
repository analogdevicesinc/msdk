/*============================================================================
 *
 * ucl_tdes_cbc_mac.h
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
#ifndef _UCL_TDES_CBC_MAC_H_
#define _UCL_TDES_CBC_MAC_H_

#include "ucl_des.h"

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */

/** @file ucl_tdes_cbc_mac.h
 * @defgroup UCL_TDES_CBC_MAC TDES CBC MAC
 * ISO/IEC 9797 Part I MAC Algorithm 3 with DES.
 *
 * @par Header:
 * @link ucl_tdes_cbc_mac.h ucl_tdes_cbc_mac.h @endlink
 *
 * @note The message must be padded before process. Use a padding method
 * described in ISO/IEC 9797.
 *
 * @ingroup UCL_CBC_MAC
 */


/** <b>TDES-CBC-MAC Context</b>.
 *
 * @ingroup UCL_TDES_CBC_MAC
 */
struct ucl_tdes_ctx
{
    /** DES Context.*/
    ucl_des_ctx_t des_ctx;
    /** Second Key.*/
    u32 key2[2];
} ;

/** <b>TDES-CBC-MAC Context</b>.
 *
 * @ingroup UCL_TDES_CBC_MAC
 */
typedef struct ucl_tdes_ctx ucl_tdes_ctx_t;


/*============================================================================*/
/** <b>TDES-CBC-MAC</b>.
 * Complete process.
 *
 * @pre @li The data byte length must be a multiple of 8.
 *  @li The @p mac byte length is 8 bytes.
 *   @li The key length is 16 bytes.
 *
 * @param[out] mac          Pointer to the mac
 * @param[in]  mac_byteLen  MAC byte Length
 * @param[in]  key          Pointer to the 3DES 128-bit key
 * @param[in]  data         Pointer to the input data
 * @param[in]  data_byteLen Data byte length
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_INPUT  If one of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT If one of the output is the pointer NULL
 * @retval #UCL_INVALID_ARG    If @p mac_byteLen is greater than 8 or
 *                             If @p data_byteLen is not a multiple of 8
 *
 * @ingroup UCL_TDES_CBC_MAC
 */
int ucl_tdes_cbc_mac(u8 *mac, u32 mac_byteLen, u8 *key, u8 *data,
                     u32 data_byteLen);


/*============================================================================*/
/** <b>TDES-CBC-MAC Init</b>.
 * Initialize TDES CBC MAC Context.
 *
 * @pre The key length is 16 bytes.
 *
 * @param[out] ctx  Pointer to the context
 * @param[in]  key  Pointer to the 3DES 128-bit key
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_INPUT  If the input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT If the output is the pointer #NULL
 *
 * @ingroup UCL_TDES_CBC_MAC
 */
int ucl_tdes_cbc_mac_init(ucl_tdes_ctx_t *ctx, u8 *key);


/*============================================================================*/
/** <b>TDES-CBC-MAC Core</b>.
 * Process the Data.
 *
 * @pre The data byte length must be a multiple of 8.
 *
 * @param[in,out] ctx         Pointer to the context
 * @param[in]     data        Pointer to the data
 * @param[in]     data_length Data byte length
 *
 * @pre The byte length must be a multiple of 8.
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_INPUT  If one of the input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT If the output is the pointer #NULL
 * @retval #UCL_INVALID_ARG    If the byte length is not a multiple of 8
 *
 * @ingroup UCL_TDES_CBC_MAC
 */
int ucl_tdes_cbc_mac_core(ucl_tdes_ctx_t *ctx, u8 *data,
                          u32 data_length);


/*============================================================================*/
/** <b>TDES-CBC-MAC Finish</b>.
 * Zeroize the context and return result.
 *
 * @pre The @p mac byte length must be less or equal than 8 bytes.
 *
 * @param[out]    mac         Pointer to the mac
 * @param[in]     mac_byteLen MAC byte Length
 * @param[in,out] ctx         Pointer to the context
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_OUTPUT If one of the outputs is the pointer #NULL
 *
 * @ingroup UCL_TDES_CBC_MAC
 */
int ucl_tdes_cbc_mac_finish(u8 *mac, u32 mac_byteLen,
                            ucl_tdes_ctx_t *ctx);


#ifdef __cplusplus
}
#endif /* _ cplusplus  */

#endif /* _UCL_TDES_CBC_MAC_H_ */
