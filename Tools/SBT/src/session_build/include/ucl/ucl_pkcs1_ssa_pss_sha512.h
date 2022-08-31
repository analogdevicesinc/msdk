/*============================================================================
 *
 * ucl_pkcs1_ssa_pss_sha512.h
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
 * Purpose : PKCS#1 V2.1 RSASSA-PSS with SHA512
 *
 *==========================================================================*/
#ifndef _UCL_RSA_SSA_PSS_SHA512_H_
#define _UCL_RSA_SSA_PSS_SHA512_H_

#include "ucl_rsa.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @file ucl_pkcs1_ssa_pss_sha512.h
 * @defgroup UCL_PKCS1V21_RSASSAPSS_SHA512 RSASSA-PSS-SHA512
 * RSAPSS with the hash function SHA512.
 *
 * @par Header:
 * @link ucl_pkcs1_ssa_pss_sha512.h ucl_pkcs1_ssa_pss_sha512.h @endlink
 *
 * @ingroup UCL_PKCS1V21_RSASSAPSS
 */

/*============================================================================*/
/** <b>RSASSA-PSS-SHA512 digest signature verification</b>.
 * Signature verification using the hash function SHA512.
 *
 * @param[in]  signature   Pointer to the signature to be checked
 * @param[in]  hash        The original message digest to be verified
 * @param[in]  hash_length The byte length of the digest
 * @param[in]  keyPu       RSA public key
 * @param[in] salt_length  Optional salt byte length
 *
 * @return Error code
 *
 * @retval #UCL_OK    if the signature is valid
 * @retval #UCL_ERROR if the signature is invalid
 *
 * @see UCL_RSA
 *
 * @ingroup UCL_PKCS1V21_RSASSAPSS_SHA512
 */

int ucl_pkcs1_ssa_pss_sha512_verify_digest(
    u8* signature, u8* hash, u32 hash_length, ucl_rsa_public_key_t* keyPu, u32 salt_length);

/*============================================================================*/
/** <b>RSASSA-PSS-SHA512 signature verification</b>.
 * Signature verification using the hash function SHA512.
 *
 * @param[in] signature      The signature to be verify
 * @param[in] message        The message
 * @param[in] message_length The message byte length
 * @param[in] keyPu          RSA public key
 * @param[in] salt_length    Optional salt byte length
 *
 * @note The signature length is the modulus length
 *
 * @return Error code
 *
 * @retval #UCL_OK  if the signature is valid
 * @retval #UCL_ERROR if the signature is invalid
 *
 * @see UCL_RSA
 *
 * @ingroup UCL_PKCS1V21_RSASSAPSS_SHA512
 */
int ucl_pkcs1_ssa_pss_sha512_verify(
    u8* signature, u8* message, u32 message_length, ucl_rsa_public_key_t* keyPu, u32 salt_length);

/** <b>RSASSA-PSS-SHA512 signature verification</b>.
 * Signature verification using the hash function SHA512.
 *
 * @param[in]  signature     Pointer to the signature to be checked
 * @param[in]  message      The original message to be verified
 * @param[in]  message_length   The byte length of the message
 * @param[in]  keyPu      RSA public key
 * @param[in] salt_length    Optional salt byte length
 * @param[in] is_digest       Use a digest instead of the message (use UCL_FALSE if not)
 *
 * @return Error code
 *
 * @retval #UCL_OK     if the signature is valid
 * @retval #UCL_ERROR    if the signature is invalid
 *
 * @see UCL_RSA
 *
 * @ingroup UCL_PKCS1V21_RSASSAPSS_SHA512
 */
int __API__ ucl_pkcs1_ssa_pss_sha512_verify_(u8* signature, u8* message, u32 message_length,
    ucl_rsa_public_key_t* keyPu, u32 salt_length, int is_digest);

/*============================================================================*/
/** <b>RSA-SSA-PSS-SHA512 signature generation</b>.
 * Signature generation using the hash function SHA512.
 * the message is hashed using the SHA512 function
 *
 * @param[out] signature      The generated signature
 * @param[in]  message        The message
 * @param[in]  message_length The message byte length
 * @param[in]  keyPr          RSA private key
 * @param[in]  salt_length    Optional salt byte length
 *
 * @note The output length is the modulus length
 *
 * @return Error code
 *
 * @retval #UCL_OK if no error occurred
 *
 * @see UCL_RSA
 *
 * @ingroup UCL_PKCS1V21_RSASSAPSS_SHA512
 */
int ucl_pkcs1_ssa_pss_sha512_sign(
    u8* signature, u8* message, u32 message_length, ucl_rsa_private_key_t* keyPr, u32 salt_length);

/*============================================================================*/
/** <b>RSA-SSA-PSS-SHA512 signature generation</b>.
 * Signature generation using the hash function SHA512.
 * it uses pre-hashed data, using the SHA512 function
 *
 * @param[out] signature      The generated signature
 * @param[in]  message        The message
 * @param[in]  message_length The message byte length
 * @param[in]  keyPr          RSA private key
 * @param[in]  salt_length    Optional salt byte length
 *
 * @note The output length is the modulus length
 *
 * @return Error code
 *
 * @retval #UCL_OK if no error occurred
 *
 * @see UCL_RSA
 *
 * @ingroup UCL_PKCS1V21_RSASSAPSS_SHA512
 */
int ucl_pkcs1_ssa_pss_sha512_hashed_sign(
    u8* signature, u8* message, u32 message_length, ucl_rsa_private_key_t* keyPr, u32 salt_length);

/*============================================================================*/
/** <b>RSASSA-PSS-SHA512 signature generation with CRT</b>.
 * Signature generation with CRT using the hash function SHA512.
 * the message is hashed using the SHA512 function
 *
 * @param[out] signature      The generated signature
 * @param[in]  message        The message
 * @param[in]  message_length The message byte length
 * @param[in]  key            RSA CRT private key
 * @param[in]  salt_length    Optional salt byte length
 *
 * @note The output length is the modulus length
 *
 * @return Error code
 *
 * @retval #UCL_OK if no error occurred
 *
 * @see UCL_RSA
 *
 * @ingroup UCL_PKCS1V21_RSASSAPSS_SHA512
 */
int ucl_pkcs1_ssa_pss_sha512_crt_sign(u8* signature, u8* message, u32 message_length,
    ucl_rsa_crt_private_key_t* key, u32 salt_length);

/*============================================================================*/
/** <b>RSASSA-PSS-SHA512 signature generation with CRT</b>.
 * Signature generation with CRT using the hash function SHA512.
 * it uses pre-hashed data, using the SHA512 function
 *
 * @param[out] signature      The generated signature
 * @param[in]  message        The message
 * @param[in]  message_length The message byte length
 * @param[in]  key            RSA CRT private key
 * @param[in]  salt_length    Optional salt byte length
 *
 * @note The output length is the modulus length
 *
 * @return Error code
 *
 * @retval #UCL_OK if no error occurred
 *
 * @see UCL_RSA
 *
 * @ingroup UCL_PKCS1V21_RSASSAPSS_SHA512
 */
int ucl_pkcs1_ssa_pss_sha512_hashed_crt_sign(u8* signature, u8* message, u32 message_length,
    ucl_rsa_crt_private_key_t* key, u32 salt_length);

#ifdef __cplusplus
}
#endif

#endif /*_UCL_RSA_SSA_PSS_SHA512_H_*/
