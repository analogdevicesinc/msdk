/*============================================================================
 *
 * ucl_pkcs5_pbkdf2.h
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
#ifndef _UCL_PKCS5V20_PBKDF2_H_
#define _UCL_PKCS5V20_PBKDF2_H_

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */


/** @defgroup UCL_PKCS5V20_PBKDF2 PBKDF2
 * @ref PKCS5 "PKCS #5" v2.0 Section 5.2: PBKDF2.
 *
 * PBKDF2 applies a pseudorandom function to derive keys. The length of the
 * derived key is essentially unbounded. (However, the maximum effective search
 * space for the derived key may be limited by the structure of the underlying
 * pseudorandom function.) @n
 * @n
 * PBKDF2 is recommended for new applications.
 *
 * @ingroup UCL_PKCS5V20_PBKDF
 */


/** @file ucl_pkcs5_pbkdf2.h
 * @defgroup UCL_PKCS5V20_PBKDF2_HMAC_SHA1 PBKDF2 HMAC-SHA1
 * PBKDF2 using HMAC-SHA1.
 *
 * @par Header:
 * @link ucl_pkcs5_pbkdf2.h ucl_pkcs5_pbkdf2.h @endlink
 *
 * @ingroup UCL_PKCS5V20_PBKDF2
 */

/** @file ucl_pkcs5_pbkdf2.h
 * @defgroup UCL_PKCS5V20_PBKDF2_HMAC_SHA256 PBKDF2 HMAC-SHA256
 * PBKDF2 using HMAC-SHA256.
 *
 * @par Header:
 * @link ucl_pkcs5_pbkdf2.h ucl_pkcs5_pbkdf2.h @endlink
 *
 * @ingroup UCL_PKCS5V20_PBKDF2
 */

/** @file ucl_pkcs5_pbkdf2.h
 * @defgroup UCL_PKCS5V20_PBKDF2_HMAC_SHA512 PBKDF2 HMAC-SHA512
 * PBKDF2 using HMAC-SHA512.
 *
 * @par Header:
 * @link ucl_pkcs5_pbkdf2.h ucl_pkcs5_pbkdf2.h @endlink
 *
 * @ingroup UCL_PKCS5V20_PBKDF2
 */

/** @file ucl_pkcs5_pbkdf2.h
 * @defgroup UCL_PKCS5V20_PBKDF2_HMAC_SHA384 PBKDF2 HMAC-SHA384
 * PBKDF2 using HMAC-SHA384.
 *
 * @par Header:
 * @link ucl_pkcs5_pbkdf2.h ucl_pkcs5_pbkdf2.h @endlink
 *
 * @ingroup UCL_PKCS5V20_PBKDF2
 */


/** <b>PBKDF2-HMAC-SHA1</b>.
 * PBKDF2 using HMAC-SHA1
 *
 * @param[out] derived_key         Pointer to the derived key
 * @param[in]  derived_key_byteLen Key byte length
 * @param[in]  password            Pointer to the password
 * @param[in]  password_byteLen    Password byte length
 * @param[in]  salt                Pointer to the Salt
 * @param[in]  salt_byteLen        Salt byte Length
 * @param[in]  count               Count
 *
 * @return Error code
 *
 * @retval #UCL_OK             if no error occurred
 * @retval #UCL_INVALID_OUTPUT if @p derived_key is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if one of the inputs is the pointer #NULL
 * @retval #UCL_INVALID_ARG    if @p count < 1000
 *
 * @see UCL_HMAC_SHA1
 *
 * @ingroup UCL_PKCS5V20_PBKDF2_HMAC_SHA1
 */
int ucl_pkcs5_pbkdf2_hmac_sha1(u8 *derived_key,
                               u32 derived_key_byteLen, u8 *password,
                               u32 password_byteLen, u8 *salt,
                               u32 salt_byteLen, u32 count);

/** @file ucl_pkcs5_pbkdf2.h
 * @defgroup UCL_PKCS5V20_PBKDF2_HMAC_SHA1 PBKDF2 HMAC-SHA1
 * PBKDF2 using HMAC-SHA1.
 *
 * @par Header:
 * @link ucl_pkcs5_pbkdf2.h ucl_pkcs5_pbkdf2.h @endlink
 *
 * @ingroup UCL_PKCS5V20_PBKDF2
 */


/** <b>PBKDF2-HMAC-SHA256</b>.
 * PBKDF2 using HMAC-SHA256
 *
 * @param[out] derived_key         Pointer to the derived key
 * @param[in]  derived_key_byteLen Key byte length
 * @param[in]  password            Pointer to the password
 * @param[in]  password_byteLen    Password byte length
 * @param[in]  salt                Pointer to the Salt
 * @param[in]  salt_byteLen        Salt byte Length
 * @param[in]  count               Count
 *
 * @return Error code
 *
 * @retval #UCL_OK             if no error occurred
 * @retval #UCL_INVALID_OUTPUT if @p derived_key is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if one of the inputs is the pointer #NULL
 * @retval #UCL_INVALID_ARG    if @p count < 1000
 *
 * @see UCL_HMAC_SHA256
 *
 * @ingroup UCL_PKCS5V20_PBKDF2_HMAC_SHA256
 */
int ucl_pkcs5_pbkdf2_hmac_sha256(u8 *derived_key, u32 derived_key_byteLen, u8 *password, u32 password_byteLen, u8 *salt, u32 salt_byteLen, u32 count);


/** <b>PBKDF2-HMAC-SHA384</b>.
 * PBKDF2 using HMAC-SHA384
 *
 * @param[out] derived_key         Pointer to the derived key
 * @param[in]  derived_key_byteLen Key byte length
 * @param[in]  password            Pointer to the password
 * @param[in]  password_byteLen    Password byte length
 * @param[in]  salt                Pointer to the Salt
 * @param[in]  salt_byteLen        Salt byte Length
 * @param[in]  count               Count
 *
 * @return Error code
 *
 * @retval #UCL_OK             if no error occurred
 * @retval #UCL_INVALID_OUTPUT if @p derived_key is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if one of the inputs is the pointer #NULL
 * @retval #UCL_INVALID_ARG    if @p count < 1000
 *
 * @see UCL_HMAC_SHA384
 *
 * @ingroup UCL_PKCS5V20_PBKDF2_HMAC_SHA384
 */
int ucl_pkcs5_pbkdf2_hmac_sha384(u8 *derived_key, u32 derived_key_byteLen, u8 *password, u32 password_byteLen, u8 *salt, u32 salt_byteLen, u32 count);


/** <b>PBKDF2-HMAC-SHA512</b>.
 * PBKDF2 using HMAC-SHA512
 *
 * @param[out] derived_key         Pointer to the derived key
 * @param[in]  derived_key_byteLen Key byte length
 * @param[in]  password            Pointer to the password
 * @param[in]  password_byteLen    Password byte length
 * @param[in]  salt                Pointer to the Salt
 * @param[in]  salt_byteLen        Salt byte Length
 * @param[in]  count               Count
 *
 * @return Error code
 *
 * @retval #UCL_OK             if no error occurred
 * @retval #UCL_INVALID_OUTPUT if @p derived_key is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if one of the inputs is the pointer #NULL
 * @retval #UCL_INVALID_ARG    if @p count < 1000
 *
 * @see UCL_HMAC_SHA512
 *
 * @ingroup UCL_PKCS5V20_PBKDF2_HMAC_SHA512
 */
int ucl_pkcs5_pbkdf2_hmac_sha512(u8 *derived_key, u32 derived_key_byteLen, u8 *password, u32 password_byteLen, u8 *salt, u32 salt_byteLen, u32 count);


#ifdef __cplusplus
}
#endif /* _ cplusplus  */

#endif /* _UCL_PKCS5V20_PBKDF2_H_ */
