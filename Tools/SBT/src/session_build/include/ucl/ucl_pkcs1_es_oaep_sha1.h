/*============================================================================
 *
 * ucl_pkcs1_es_oaep_sha1.h
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
 * Purpose : Algorithm for RSA_OAEP with SHA1
 *
 *==========================================================================*/
#ifndef _UCL_RSA_ES_OAEP_SHA1_H_
#define _UCL_RSA_ES_OAEP_SHA1_H_

#include "ucl_rsa.h"

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */

/** @file ucl_pkcs1_es_oaep_sha1.h
 * @defgroup UCL_PKCS1V21_OAEP_SHA1 RSA OAEP SHA1
 * Encrypt/Decrypt with RSA OAEP using the hash function SHA1.
 *
 * @par Header:
 * @link ucl_pkcs1_es_oaep_sha1.h ucl_pkcs1_es_oaep_sha1.h @endlink
 *
 * @ingroup UCL_PKCS1V21_RSAESOAEP
 */


/*============================================================================*/
/** <b>RSA-OAEP-SHA1 encryption</b>.
 * Encryption complete process using the hash function SHA1.
 *
 * @pre The message byte length must be lesser than @f$ k - 42 @f$ where
 * @a k is the modulus byte length
 *
 * @param[out] output                Pointer to encrypted output
 * @param[in]  input                 Pointer to the message which must be processed
 * @param[in]  input_length          Input byte length
 * @param[in]  keyPu                 RSA public key
 * @param[in]  optional_label        Pointer to optional label
 * @param[in]  optional_label_length Optional label byte length
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
 * @see UCL_SHA1
 *
 * @ingroup UCL_PKCS1V21_OAEP_SHA1
 */
int ucl_pkcs1_es_oaep_sha1_encrypt(u8 *output, u8 *input,
                                   u32 input_length, 
                                   ucl_rsa_public_key_t *keyPu, 
                                   u8 *optional_label, 
                                   u32 optional_label_length);


/*============================================================================*/
/** <b>RSA-OAEP-SHA1 decryption</b>.
 * Decryption complete process using the hash function SHA1.
 *
 * @param[out] output               Pointer to decrypted message
 * @param[out] output_length        Pointer to the final length of the message
 * @param[in] input                 Pointer to the encrypted input
 * @param[in] keyPr                 RSA private key
 * @param[in] optional_label        Pointer to optional label
 * @param[in] optional_label_length Optional label byte length
 * 
 * @note The output buffer length is the modulus length
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_INPUT  If one of the input pointers is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT If one of the output pointer is the pointer #NULL
 * @retval #UCL_ERROR          If a decryption error occurred (PKCS1 Error)
 *
 * @see UCL_RSA
 * @see UCL_SHA1
 *
 * @ingroup UCL_PKCS1V21_OAEP_SHA1
 */
int ucl_pkcs1_es_oaep_sha1_decrypt(u8 *output, u32 *output_length,
                                   u8 *input, ucl_rsa_private_key_t *keyPr, 
                                   u8 *optional_label,
                                   u32 optional_label_length);


/*============================================================================*/
/** <b>RSA-OAEP-SHA1 decryption with CRT</b>.
 * Decryption with CRT complete process using the hash function SHA1.
 *
 * @param[out] output               Pointer to decrypted message
 * @param[out] output_length        Pointer to length of the decrypted message
 * @param[in] input                 Pointer to the input
 * @param[in] key                   Pointer to the RSA CRT key
 * @param[in] optional_label        Pointer to optional label
 * @param[in] optional_label_length Optional label byte length
 * 
 * @note The output buffer length is the modulus length
 * 
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_INPUT  If one of the input pointers is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT If one of the output pointer is the pointer #NULL
 * @retval #UCL_ERROR          If a decryption error occurred (PKCS1 Error)
 *
 * @see UCL_RSA
 *
 * @ingroup UCL_PKCS1V21_OAEP_SHA1
 */
int ucl_pkcs1_es_oaep_sha1_crt_decrypt(u8 *output, u32 *output_length,
                                       u8 *input, 
                                       ucl_rsa_crt_private_key_t *key, 
                                       u8 *optional_label,
                                       u32 optional_label_length);


#ifdef __cplusplus
}
#endif /* _ cplusplus  */

#endif /* _UCL_RSA_ES_OAEP_SHA1_H_ */
