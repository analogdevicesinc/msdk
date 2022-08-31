/*============================================================================
 *
 * ucl_pkcs1_ssa_pkcs1v15_md5.h
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
 * Purpose : PKCS#1 V2.1 RSASSA-PKCS1V15 with MD5
 *
 *==========================================================================*/
#ifndef _UCL_RSA_SSA_PKCS1V15_MD5_H_
#define _UCL_RSA_SSA_PKCS1V15_MD5_H_

#include "ucl_rsa.h"

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */

/** @file ucl_pkcs1_ssa_pkcs1v15_md5.h
 * @defgroup UCL_PKCS1V21_SSA_PKCSV15_MD5 RSASSA-PKCS1V15 MD5
 * Signature scheme RSA PKCS#1 V1.5 using the hash function MD5.
 *
 * @par Header:
 * @link ucl_pkcs1_ssa_pkcs1v15_md5.h ucl_pkcs1_ssa_pkcs1v15_md5.h @endlink
 *
 * @ingroup UCL_PKCS1V21_SSA_PKCS1V15
 */

/*============================================================================*/
/** <b>RSASSA-PKCS1V15-MD5 Signature Generation</b>.
 * Signature generation using hash function MD5.
 *
 * @param[out] signature      The generated signature
 * @param[in]  message        The message to be signed
 * @param[in]  message_length The message byte length
 * @param[in]  keyPr          RSA private key
 *
 * @note The output buffer length is the modulus length
 *
 * @return Error code
 *
 * @retval #UCL_OK if no error occurred
 *
 * @see UCL_RSA
 *
 * @ingroup UCL_PKCS1V21_SSA_PKCSV15_MD5
 */
int ucl_pkcs1_ssa_pkcs1v15_md5_sign(
    u8* signature, u8* message, u32 message_length, ucl_rsa_private_key_t* keyPr);

/*============================================================================*/
/** <b>RSASSA-PKCS1V15-MD5 Signature Generation</b>.
 * Signature generation using hash function MD5.
 * it uses pre-hashed data, using the MD5 function
 * @param[out] signature      The generated signature
 * @param[in]  message        The message to be signed, pre-hashed data
 * @param[in]  message_length The message byte length
 * @param[in]  keyPr          RSA private key
 *
 * @note The output buffer length is the modulus length
 *
 * @return Error code
 *
 * @retval #UCL_OK if no error occurred
 *
 * @see UCL_RSA
 *
 * @ingroup UCL_PKCS1V21_SSA_PKCSV15_MD5
 */
int ucl_pkcs1_ssa_pkcs1v15_md5_hashed_sign(
    u8* signature, u8* message, u32 message_length, ucl_rsa_private_key_t* keyPr);

/*============================================================================*/
/** <b>RSASSA-PKCS1V15-MD5 CRT Signature Generation</b>.
 * CRT Signature generation using hash function MD5.
 * it uses pre-hashed data, using the MD5 function
 *
 * @param[out] signature      The generated signature
 * @param[in]  message        The messageto be signed
 * @param[in]  message_length The message byte length
 * @param[in]  keyPr          RSA CRT private key
 *
 * @note The output buffer length is the modulus length
 *
 * @return Error code
 *
 * @retval #UCL_OK if no error occurred
 *
 * @see UCL_RSA
 *
 * @ingroup UCL_PKCS1V21_SSA_PKCSV15_MD5
 */
int ucl_pkcs1_ssa_pkcs1v15_md5_hashed_crt_sign(
    u8* signature, u8* message, u32 message_length, ucl_rsa_crt_private_key_t* keyPr);

/*============================================================================*/
/** <b>RSASSA-PKCS1V15-MD5 CRT Signature Generation</b>.
 * CRT Signature generation using hash function MD5.
 *
 * @param[out] signature      The generated signature
 * @param[in]  message        The messageto be signed
 * @param[in]  message_length The message byte length
 * @param[in]  keyPr          RSA CRT private key
 *
 * @note The output buffer length is the modulus length
 *
 * @return Error code
 *
 * @retval #UCL_OK if no error occurred
 *
 * @see UCL_RSA
 *
 * @ingroup UCL_PKCS1V21_SSA_PKCSV15_MD5
 */
int ucl_pkcs1_ssa_pkcs1v15_md5_crt_sign(
    u8* signature, u8* message, u32 message_length, ucl_rsa_crt_private_key_t* keyPr);

/*============================================================================*/
/** <b>RSASSA-PKCS1V15-MD5 Signature Verification</b>.
 * Signature verification using hash function MD5.
 *
 * @param[in] signature      The signature to verify
 * @param[in] message        The message
 * @param[in] message_length The message byte length
 * @param[in] keyPu          The RSA public key
 *
 * @note The signature length is the modulus length
 *
 * @return Error code
 *
 * @retval #UCL_OK if the signature is valid
 *
 * @see UCL_RSA
 *
 * @ingroup UCL_PKCS1V21_SSA_PKCSV15_MD5
 */
int ucl_pkcs1_ssa_pkcs1v15_md5_verify(
    u8* signature, u8* message, u32 message_length, ucl_rsa_public_key_t* keyPu);

/*============================================================================*/
/** <b>RSASSA-PKCS1V15-MD5 Signature Verification</b>.
 * Signature verification using hash function MD5.
 *
 * @param[in] signature      The signature to verify
 * @param[in]  hash        The original message digest to be verified
 * @param[in]  hash_length The byte length of the digest
 * @param[in] keyPu          The RSA public key
 *
 * @note The signature length is the modulus length
 *
 * @return Error code
 *
 * @retval #UCL_OK if the signature is valid
 *
 * @see UCL_RSA
 *
 * @ingroup UCL_PKCS1V21_SSA_PKCSV15_MD5
 */
int ucl_pkcs1_ssa_pkcs1v15_md5_verify_digest(
    u8* signature, u8* hash, u32 hash_length, ucl_rsa_public_key_t* keyPu);

#ifdef __cplusplus
}
#endif /* _ cplusplus  */

#endif /*_UCL_RSA_SSA_PKCS1V15_MD5_H_ */
