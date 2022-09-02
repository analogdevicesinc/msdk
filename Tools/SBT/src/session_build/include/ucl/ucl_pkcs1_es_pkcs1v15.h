/*============================================================================
 *
 * ucl_pkcs1_es_pkcs1v15.h
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
 * Purpose : RSA Encryption Scheme PKCS#1 v1.5
 *
 *==========================================================================*/
#ifndef _UCL_RSA_ES_PKCS1V15_H_
#define _UCL_RSA_ES_PKCS1V15_H_

#include "ucl_rsa.h"

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */

/** @file ucl_pkcs1_es_pkcs1v15.h
 * @defgroup UCL_PKCS1V21_RSAESPKCS1V15 RSAES-PKCS1-v1.5
 * @ref PKCS1 "PKCS #1 V2.1" Section 7.2: RSAES-PKCS1-v1_5.
 *
 * @par Header:
 * @link ucl_pkcs1_es_pkcs1v15.h ucl_pkcs1_es_pkcs1v15.h @endlink
 *
 * RSAES-PKCS1-v1_5 combines the RSAEP and RSADP primitives with the
 * EME-PKCS1-v1_5 encoding method. It is mathematically equivalent to the
 * encryption scheme in PKCS #1 v1.5. RSAES-PKCS1-v1_5 can operate on messages
 * of length up to @f$ k - 11 @f$ octets (@a k is the byte length of the RSA
 * modulus), although care should be taken to avoid certain attacks on
 * low-exponent RSA due to Coppersmith, Franklin, Patarin, and Reiter when long
 * messages are encrypted. As a general rule, the use of this scheme for
 * encrypting an arbitrary message, as opposed to a randomly generated key,
 * is not recommended.
 *
 * @ingroup UCL_PKCS1V21
 */

/*============================================================================*/
/** <b>RSAES-PKCS1V15 encryption</b>.
 * Encryption complete process.
 *
 * @param[out] output       Pointer to encrypted output
 * @param[in]  input        Pointer to the message which must be processed
 * @param[in]  input_length Input byte length
 * @param[in]  keyPu        RSA public key
 * 
 * @note The output buffer length is the modulus length
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_INPUT  If one of the input pointers is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT If the output pointer is the pointer #NULL
 * @retval #UCL_INVALID_ARG    If the message to encrypt is too long (PKCS1 Error)
 *
 * @see UCL_RSA
 *
 * @ingroup UCL_PKCS1V21_RSAESPKCS1V15
 */
int ucl_pkcs1_es_pkcs1v15_encrypt(u8* output, u8* input, u32 input_length,
                                  ucl_rsa_public_key_t* keyPu);

/*============================================================================*/
/** <b>RSAES-PKCS1V15 decryption</b>.
 * Decryption complete process.
 *
 *
 * @param[out] output        Pointer to decrypted message
 * @param[out] output_length Pointer to the final length of the message
 * @param[in]  input         Pointer to the encrypted input
 * @param[in]  keyPr         RSA private key
 * 
 * @note the output_length shall be initialized with the supposed output length
 * it will be used for checking the decrypted output length and then avoid buffer overflow
 * in case the lengths do not match
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_INPUT  If one of the input pointers is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT If one of the output pointer is the pointer #NULL
 * @retval #UCL_INVALID_LENGTH If the given, supposed output length is smaller than the result output length
 * @retval #UCL_ERROR          If a decryption error occurred (PKCS1 Error)
 *
 *
 * @see UCL_RSA
 *
 * @ingroup UCL_PKCS1V21_RSAESPKCS1V15
 */
int ucl_pkcs1_es_pkcs1v15_decrypt(u8* output, u32* output_length, u8* input,
                                  ucl_rsa_private_key_t* keyPr);

/*============================================================================*/
/** <b>RSAES-PKCS1V15 decryption with CRT</b>.
 * Decryption with CRT complete process.
 *
 * @param[out] output        Pointer to decrypted message
 * @param[out] output_length Pointer to the final length of the message
 * @param[in]  input         Pointer to the encrypted input
 * @param[in]  keyPr         RSA private key
 * 
 * @note the output_length shall be initialized with the supposed output length
 * it will be used for checking the decrypted output length and then avoid buffer overflow
 * in case the lengths do not match
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_INPUT  If one of the input pointers is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT If one of the output pointer is the pointer #NULL
 * @retval #UCL_INVALID_LENGTH If the given, supposed output length is smaller than the result output length
 * @retval #UCL_ERROR          If a decryption error occurred (PKCS1 Error)
 *
 * @see UCL_RSA
 *
 * @ingroup UCL_PKCS1V21_RSAESPKCS1V15
 */
int ucl_pkcs1_es_pkcs1v15_crt_decrypt(u8* output, u32* output_length, u8* input,
                                      ucl_rsa_crt_private_key_t* keyPr);

#ifdef __cplusplus
}
#endif /* _ cplusplus  */

#endif /* _UCL_RSA_ES_PKCS1V15_H_ */
