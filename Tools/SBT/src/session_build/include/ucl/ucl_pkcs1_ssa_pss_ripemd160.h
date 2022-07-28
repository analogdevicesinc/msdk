/*============================================================================
 *
 * ucl_pkcs1_ssa_pss_ripemd160.h
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
 * Purpose : PKCS#1 V2.1 RSASSA-PSS with RIPEMD160
 *
 *==========================================================================*/
#ifndef _UCL_RSA_SSA_PSS_RIPEMD160_H_
#define _UCL_RSA_SSA_PSS_RIPEMD160_H_

#include "ucl_rsa.h"

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */


/** @file ucl_pkcs1_ssa_pss_ripemd160.h
 * @defgroup UCL_PKCS1V21_RSASSAPSS_RIPEMD160 RSASSA-PSS-RIPEMD160
 * RSAPSS with the hash function RIPEMD160.
 *
 * @par Header:
 * @link ucl_pkcs1_ssa_pss_ripemd160.h ucl_pkcs1_ssa_pss_ripemd160.h @endlink
 *
 * @ingroup UCL_PKCS1V21_RSASSAPSS
 */


/*============================================================================*/
/** <b>RSASSA-PSS-RIPEMD160 signature verification</b>.
 * Signature verification using the hash function RIPEMD160.
 *
 * @param[in] signature      The signature to be verify
 * @param[in] message        The message
 * @param[in] message_length The message byte length
 * @param[in] keyPu          RSA public key
 * @param[in] salt_length    Optional salt byte length
 * 
 * @note The output length is the modulus length
 *
 * @return Error code
 *
 * @retval #UCL_OK    if the signature is valid
 * @retval #UCL_ERROR if the signature is invalid
 *
 * @see UCL_RSA
 *
 * @ingroup UCL_PKCS1V21_RSASSAPSS_RIPEMD160
 */
int __API__ ucl_pkcs1_ssa_pss_ripemd160_verify(u8 *signature, u8 *message,
                                       u32 message_length, 
                                       ucl_rsa_public_key_t *keyPu, 
                                       u32 salt_length);


/*============================================================================*/
/** <b>RSA-SSA-PSS-RIPEMD160 signature generation</b>.
 * Signature generation using the hash function RIPEMD160.
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
 * @ingroup UCL_PKCS1V21_RSASSAPSS_RIPEMD160
 */
int __API__ ucl_pkcs1_ssa_pss_ripemd160_sign(u8 *signature, u8 *message,
                                     u32 message_length, 
                                     ucl_rsa_private_key_t *keyPr, 
                                     u32 salt_length);


/*============================================================================*/
/** <b>RSASSA-PSS-RIPEMD160 signature generation with CRT</b>.
 * Signature generation with CRT using the hash function RIPEMD160.
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
 * @ingroup UCL_PKCS1V21_RSASSAPSS_RIPEMD160
 */
int __API__ ucl_pkcs1_ssa_pss_ripemd160_crt_sign(u8 *signature, u8 *message,
        u32 message_length, ucl_rsa_crt_private_key_t *key, u32 salt_length);


#ifdef __cplusplus
}
#endif /* _ cplusplus  */

#endif /*_UCL_RSA_SSA_PSS_RIPEMD160_H_*/
