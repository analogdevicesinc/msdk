/*============================================================================
 *
 * ucl_pkcs1_ssa_pss.h
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
 * Purpose : PKCS#1 V2.1 RSASSA-PSS with pre-hased data
 *
 *==========================================================================*/
#ifndef _UCL_RSA_SSA_PSS_H_
#define _UCL_RSA_SSA_PSS_H_

#include "ucl_rsa.h"

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */

	/** @file ucl_pkcs1_ssa_pss.h
	 * @defgroup UCL_PKCS1V21_RSASSAPSS RSASSA-PSS
	 * RSAPSS with pre-hashed data.
	 *
	 * @par Header:
	 * @link ucl_pkcs1_ssa_pss.h ucl_pkcs1_ssa_pss.h @endlink
	 *
	 * @ingroup UCL_PKCS1V21_RSASSAPSS
	 */

	/*============================================================================*/
	/** <b>RSASSA-PSS digest signature verification</b>.
	 * Signature verification using pre-hashed data (SHA1 and SHA256).
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
	 * @ingroup UCL_PKCS1V21_RSASSAPSS
	 */

	int ucl_pkcs1_ssa_pss_verify_digest(u8 *signature, u8 *hash, u32 hash_length, ucl_rsa_public_key_t *keyPu, u32 salt_length);

	/*============================================================================*/
	/** <b>RSASSA-PSS signature verification</b>.
	 * Signature verification using pre-hashed data (SHA1 and SHA256).
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
	 * @retval #UCL_OK    if the signature is valid
	 * @retval #UCL_ERROR if the signature is invalid
	 *
	 * @see UCL_RSA
	 *
	 * @ingroup UCL_PKCS1V21_RSASSAPSS
	 */
	int ucl_pkcs1_ssa_pss_verify(u8 *signature, u8 *message, u32 message_length, ucl_rsa_public_key_t *keyPu, u32 salt_length);

	/*============================================================================*/
	/** <b>RSASSA-PSS signature verification</b>.
	 * Signature verification using pre-hashed data (SHA1 and SHA256).
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
	 * @retval #UCL_OK    if the signature is valid
	 * @retval #UCL_ERROR if the signature is invalid
	 *
	 * @see UCL_RSA
	 *
	 * @ingroup UCL_PKCS1V21_RSASSAPSS
	 */
	int ucl_pkcs1_ssa_pss_verify_digest(u8 *signature, u8 *hash, u32 hash_length, ucl_rsa_public_key_t *keyPu, u32 salt_length);

	/*============================================================================*/
	/** <b>RSA-SSA-PSS signature generation</b>.
	 * Signature generation using pre-hashed data (SHA1 and SHA256).
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
	 * @ingroup UCL_PKCS1V21_RSASSAPSS
	 */
	int ucl_pkcs1_ssa_pss_sign(u8 *signature, u8 *message, u32 message_length, ucl_rsa_private_key_t *keyPr, u32 salt_length);

	/*============================================================================*/
	/** <b>RSASSA-PSS signature generation with CRT</b>.
	 * Signature generation with CRT pre-hashed data (SHA1 and SHA256).
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
	 * @ingroup UCL_PKCS1V21_RSASSAPSS
	 */
	int ucl_pkcs1_ssa_pss_crt_sign(u8 *signature, u8 *message, u32 message_length, ucl_rsa_crt_private_key_t *key, u32 salt_length);

#ifdef __cplusplus
}
#endif /* _ cplusplus  */

#endif /*_UCL_RSA_SSA_PSS_H_*/
