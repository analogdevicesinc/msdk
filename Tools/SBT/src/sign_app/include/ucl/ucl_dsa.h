/*============================================================================
 *
 * ucl_dsa.h
 *
 *==========================================================================*/
/*============================================================================
 *
 * Copyright © 2013 Maxim Integrated.
 * All Rights Reserved. Do not disclose.
 *
 * This software is the confidential and proprietary information of
 * Innova Card ("Confidential Information"). You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered
 * into with Maxim Integrated.
 *
 * Maxim Integrated makes no representations or warranties about the suitability of
 * the software, either express or implied, including but not limited to
 * the implied warranties of merchantability, fitness for a particular purpose,
 * or non-infrigement. Innova Card shall not be liable for any damages suffered
 * by licensee as the result of using, modifying or distributing this software
 * or its derivatives.
 *
 *
 * Purpose : DSA
 *
 *==========================================================================*/
#ifndef _UCL_DSA_H_
#define _UCL_DSA_H_
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus  */
/** @defgroup UCL_DSA DSA
 * the DSA cryptosystem.
 * 
 * @ingroup UCL_RSA
 */
/** <b>DSA signature </b>.
 * DSA key generation
 *
 * @param[out]	u8 *y: output, the public key
 * @param[out]	u8 *x: output, the secret key
 * @param[in]	u8 *p: input, the prime modulus,
 * @param[in]	u32 *plength: input, the prime modulus length, 
 * @param[in]	u8 *q: input, the prime divisor of p-1
 * @param[in]	u8 *g: input, the generator 
 *
 * @return Error code
 *
 * @retval #UCL_OK       if no error occurred
 * @retval #UCL_INVALID_INPUT in case of NULL pointers for input parameters
 * @retval #UCL_INVALID_OUTPUT in case of NULL pointers for output parameters
 * @retval #UCL_ERROR for wrong parameters
 * @retval #UCL_STACK_ERROR if the UCL stack has a problem
 *
 * @see UCL_DSA
 *
 * @ingroup UCL_DSA
 */
int ucl_dsa_keygen(u8* y, u8* x, u8* p, u32 plength, u8* q, u8* g);

/** <b>DSA signature </b>.
 * DSA Signature computation using supplied hash function.
 *
 * @param[out]	u8 *r_and_others: input/output, the r value for the computed signature (input if pre-computed with R_PRECOMP configuration)
	if R_PRECOMP or PRECOMP_R are used, the array is a double curve array as containing the r and the k-1.
 * @param[out]	u8 *s: output, the s value for the computed signature
 * @param[in]	u8 *p: input, the prime modulus,
 * @param[in]	u32 *plength: input, the prime modulus length,
 * @param[in]	u8 *q: input, the prime divisor of p-1
 * @param[in]	u8 *g: input, the generator
 * @param[in]	u8 *x: input, the secret key
 * @param[in]	u8 *input: input, the message or the hash digest to be signed,
 * @param[in]	u32 inputlength: input, the input length, in bytes
 * @param[in]	u32 configuration (combination of any of these lines) 
 *	UCL_MSG_INPUT or UCL_HASH_INPUT
 *	UCL_MSG_INPUT: the message will be hashed first,
 *	UCL_SHA256 or UCL_SHA1 
 *	UCL_HASH_INPUT: the input is already the hash digest,
 *	UCL_SHA256 or UCL_SHA1: hash function, if UCL_MSG_INPUT is used.
 *
 * @return Error code
 *
 * @retval #UCL_OK       if no error occurred
 * @retval #UCL_INVALID_INPUT in case of wrong parameters configuration
 * @retval #UCL_STACK_ERROR if the UCL stack has a problem
 * @retval #UCL_ERROR     if a decryption error occurred
 *
 * @see UCL_DSA
 * @see UCL_SHA1
 * @see UCL_SHA256
 *
 * @ingroup UCL_DSA
 */

int ucl_dsa_sign(u8* r_and_others, u8* s, u8* p, u32 plength, u8* q, u8* g, u8* x, u8* input,
                 u32 inputlength, u32 configuration);

/** <b>DSA signature verification </b>.
 * DSA Signature verification using supplied hash function.
 *
 * @param[int]	u8 *r: input, the r value for the computed signature (input if pre-computed with R_PRECOMP configuration)
	if R_PRECOMP or PRECOMP_R are used, the array is a double curve array as containing the r and the k-1.
 * @param[int]	u8 *s: output, the s value for the computed signature
 * @param[in]	u8 *p: input, the prime modulus,
 * @param[in]	u32 *plength: input, the prime modulus length,
 * @param[in]	u8 *q: input, the prime divisor of p-1
 * @param[in]	u8 *g: input, the generator
 * @param[in]	u8 *y: input, the public key
 * @param[in]	u8 *input: input, the message or the hash digest to be signed,
 * @param[in]	u32 inputlength: input, the input length, in bytes
 * @param[in]	u32 configuration (combination of any of these lines) 
 *	UCL_MSG_INPUT or UCL_HASH_INPUT
 *	UCL_MSG_INPUT: the message will be hashed first,
 *	UCL_SHA256 or UCL_SHA1 
 *	UCL_HASH_INPUT: the input is already the hash digest,
 *	UCL_SHA256 or UCL_SHA1: hash function, if UCL_MSG_INPUT is used.

 * @return Error code
 *
 * @retval #UCL_OK       if no error occurred
 * @retval #UCL_INVALID_INPUT in case of wrong parameters configuration
 * @retval #UCL_STACK_ERROR if the UCL stack has a problem
 * @retval #UCL_ERROR     if a decryption error occurred
 *
 * @see UCL_DSA
 * @see UCL_SHA1
 * @see UCL_SHA256
 *
 * @ingroup UCL_DSA
 */

int ucl_dsa_verify(u8* r, u8* s, u8* p, u32 plength, u8* q, u8* g, u8* y, u8* input,
                   u32 inputlength, u32 configuration);
#ifdef __cplusplus
}
#endif /* __cplusplus  */
#endif //UCL_DSA_H
