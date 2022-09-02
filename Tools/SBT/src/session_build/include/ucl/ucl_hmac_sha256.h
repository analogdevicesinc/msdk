/*============================================================================
 *
 * ucl_hmac_sha256.h
 *
 *==========================================================================*/
/*============================================================================
 *
 * Copyright © 2009 Innova Card. All rights reserved. Do not disclose.
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
 * Purpose : HMAC SHA256
 *
 *==========================================================================*/
#ifndef _UCL_HMAC_SHA256_H_
#define _UCL_HMAC_SHA256_H_

#include "ucl_sha256.h"

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */

/** @file ucl_hmac_sha256.h
 * @defgroup UCL_HMAC_SHA256 HMAC SHA256
 * @f$ HMAC^{SHA256}_k(x) = SHA256(k' \oplus opad,\ SHA256(k' \oplus ipad,\ x) ) @f$.
 *
 * @par Header:
 * @link ucl_hmac_sha256.h ucl_hmac_sha256.h @endlink
 *
 * @ingroup UCL_HMAC
 */

/*============================================================================*/
/** <b>The complete process of HMAC-SHA256</b>.
 *
 * @param[out] mac             The 128-bit hmac of the message
 * @param[out] mac_byteLen     MAC byte length
 * @param[in]  message         The message
 * @param[in]  message_byteLen The byte length of the message
 * @param[in]  key             The key
 * @param[in]  key_byteLen     The key byte length
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_INPUT  If one of the inputs is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT If the output is the pointer #NULL
 *
 * @ingroup UCL_HMAC_SHA256
 */
int ucl_hmac_sha256(u8* mac, u32 mac_byteLen, u8* message, u32 message_byteLen, u8* key,
                    u32 key_byteLen);

/** <b>The initialisation of HMAC-SHA256</b>.
 *
 * @param[in,out] context     The SHA256 context
 * @param[in]     key         The key
 * @param[in]     key_byteLen The key byte length
 *
 * @return Error code
 *
 * @retval #UCL_OK            If no error occurred
 * @retval #UCL_INVALID_INPUT If @p context or @p key is #NULL
 *
 * @ingroup UCL_HMAC_SHA256
 */
int ucl_hmac_sha256_init(ucl_sha256_ctx_t* context, u8* key, u32 key_byteLen);

/** <b>The core of SHA256 </b>.
 *
 * @param[in,out] context The SHA256 context
 * @param[in]     data    Data
 * @param[in]     byteLen Data byte length
 *
 * @warning #ucl_hmac_sha256_init must be processed before.
 *
 * @return Error code
 *
 * @retval #UCL_OK            If no error occurred
 * @retval #UCL_INVALID_INPUT If one of the inputs are the pointer #NULL
 * @retval #UCL_NOP           If @p dataLen = 0
 *
 * @ingroup UCL_HMAC_SHA256
 */
int ucl_hmac_sha256_core(ucl_sha256_ctx_t* context, u8* data, u32 byteLen);

/** <b>Finish the process of SHA256</b>.
 *
 * @param[out] mac         The hmac of the message
 * @param[out] mac_byteLen MAC byte length
 * @param[in]  context     The SHA256 context
 * @param[in]  key         The key
 * @param[in]  key_byteLen The key byte length
 *
 * @warning #ucl_hmac_sha256_init and
 *          #ucl_hmac_sha256_core must be processed before.
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_OUTPUT If @ hmac is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  If @p context or @p key is the pointer #NULL
 *
 * @ingroup UCL_HMAC_SHA256
 */
int ucl_hmac_sha256_finish(u8* mac, u32 mac_byteLen, ucl_sha256_ctx_t* context, u8* key,
                           u32 key_byteLen);

#ifdef __cplusplus
}
#endif /* _ cplusplus  */

#endif /* _UCL_HMAC_SHA256_H_ */
