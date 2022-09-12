/*============================================================================
 *
 * ucl_rsa2vs.h
 *
 *==========================================================================*/
/*============================================================================
 *
 * Copyright © 2016 Maxim Integrated
 * All Rights Reserved. Do not disclose.
 *
 * This software is the confidential and proprietary information of
 * Maxim Integrated ("Confidential Information"). You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered
 * into with Maxim Integrated.
 *
 * Maxim Integrated makes no representations or warranties about the suitability of
 * the software, either express or implied, including but not limited to
 * the implied warranties of merchantability, fitness for a particular purpose,
 * or non-infrigement. Maxim Integrated shall not be liable for any damages suffered
 * by licensee as the result of using, modifying or distributing this software
 * or its derivatives.
 *
 *==========================================================================*/
/*============================================================================
 *
 * Purpose : RSA2VS
 *
 *==========================================================================*/
#ifndef _UCL_RSA2VS_H_
#define _UCL_RSA2VS_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus  */

/** @file ucl_rsa.h
 * @defgroup UCL_RSA2VS RSA2VS
 * The RSA2VS validation program
 *
 *
 * @ingroup UCL_RSA
 */

#define UCL_ERROR_B43 1 // if p is not odd
#define UCL_ERROR_B44 2 // if step 4.4 is not satisfied
#define UCL_ERROR_B452 3 // if p is probably not prime
#define UCL_ERROR_B45 4 // if step 4.5 is not satisfied
#define UCL_ERROR_B53 5 // if (p is prime and) q is not odd
#define UCL_ERROR_B55 6 // if (p is prime and) step 5.5 is not satisfied
#define UCL_ERROR_B562 7 // if (p is prime and) q is probably not prime
#define UCL_ERROR_B56 8 // if (p is prime and) step 5.6 is not satisfied

/** <b>RSA Parameters Generation</b>.
 * generate RSA keys pairs with FIPS 186-4 compliance.
 * for keys length=1024, it applies section B.3.6.
 * this function returns parameters required by the RSA2VS testing, section 6.2.1, item iv
 * more especially, it returns the values xp, xq, xp1,xp2,xq1,xq2,p1,p2,q1,q2,bitlen1 and bitlen2
 * Generate @a p, @a q, @a n and @a d such as:@n
 * @f$ n = p \times q@f$ @n
 * @f$ e \times d = 1 \bmod (p-1)(q-1)@f$ @n
 *
 * @param[out]  n   The pointer to @a n
 * @param[out]  p   The pointer to @a p
 * @param[out]  q   The pointer to @a q
 * @param[out]  d   The pointer to @a d
 * @param[out]  xp   The pointer to @a xp
 * @param[out]  xq   The pointer to @a xq
 * @param[out]  xp1   The pointer to @a xp1
 * @param[out]  xp2   The pointer to @a xp2
 * @param[out]  xq1   The pointer to @a xq1
 * @param[out]  xq2   The pointer to @a xq2
 * @param[out]  p1   The pointer to @a p1
 * @param[out]  p2   The pointer to @a p2
 * @param[out]  q1   The pointer to @a q1
 * @param[out]  q2   The pointer to @a q2
 * @param[out]  bitlen1   The pointer to @a bitlen1
 * @param[out]  bitlen2   The pointer to @a bitlen2

 * @param[in]   e   The pointer to @a e
 * @param[in]   t   The length of @p e
 * @param[in]   s   The precision
 *
 * @return Error code
 *
 * @retval #UCL_OK
 *
 * @ingroup UCL_RSA2VS
 */
int ucl_rsa_param_gen_fips186_4_B36_RSA2VS(u32 *n, u32 *p, u32 *q, u32 *d, u32 *xp1, u32 *xp2,
                                           u32 *p1, u32 *p2, u32 *xp, u32 *xq1, u32 *xq2, u32 *q1,
                                           u32 *q2, u32 *xq, u32 *bitlen1, u32 *bitlen2, u32 *e,
                                           u32 t, u32 s);

/** <b>RSA Parameters Generation</b>.
 * generate RSA keys pairs with FIPS 186-4 compliance.
 * for keys length=1024, it applies section B.3.6.
 * this function returns parameters required by the RSA2VS testing, section 6.2.1, item iv
 * more especially, it returns the values xp, xq, xp1,xp2,xq1,xq2,p1,p2,q1,q2,bitlen1 and bitlen2
 * Generate @a p, @a q, @a n and @a d such as:@n
 * @f$ n = p \times q@f$ @n
 * @f$ e \times d = 1 \bmod (p-1)(q-1)@f$ @n
 *
 * the parameters are in bytes (u8)
 *
 *
 * @param[out]  n   The pointer to @a n
 * @param[out]  p   The pointer to @a p
 * @param[out]  q   The pointer to @a q
 * @param[out]  d   The pointer to @a d
 * @param[out]  xp   The pointer to @a xp
 * @param[out]  xq   The pointer to @a xq
 * @param[out]  xp1   The pointer to @a xp1
 * @param[out]  xp2   The pointer to @a xp2
 * @param[out]  xq1   The pointer to @a xq1
 * @param[out]  xq2   The pointer to @a xq2
 * @param[out]  p1   The pointer to @a p1
 * @param[out]  p2   The pointer to @a p2
 * @param[out]  q1   The pointer to @a q1
 * @param[out]  q2   The pointer to @a q2
 * @param[out]  bitlen1   The pointer to @a bitlen1
 * @param[out]  bitlen2   The pointer to @a bitlen2

 * @param[in]   e   The pointer to @a e
 * @param[in]   t   The length of @p e
 * @param[in]   s   The precision
 *
 * @return Error code
 *
 * @retval #UCL_OK
 *
 * @ingroup UCL_RSA2VS
 */
int ucl_rsa_param_generation_fips186_4_B36_RSA2VS(u8 *n, u8 *p, u8 *q, u8 *d, u8 *xp1, u8 *xp2,
                                                  u8 *p1, u8 *p2, u8 *xp, u8 *xq1, u8 *xq2, u8 *q1,
                                                  u8 *q2, u8 *xq, int *bitlen1, int *bitlen2, u8 *e,
                                                  int t, int modulus_bytes_size);

/** <b>FIPS 186-4 prime number generation</b>.
 * prime number generation compliant with FIPS 186-4 appendix B3.3
 *
 * Generate a prime number @p p for which @p p-1
 * is relatively prime to @p e
 *
 * @param[in] p the random number to test
 * @param[in] q the (optional) random number to test (q shall be #NULL to not be tested)
 * @param[in] e the public exponent
 * @param[in] s the precision, i.e. the number size in words
 *
 * @return Error code or Test value
 *
 * @retval #UCL_STACK_OVERFLOW if the buffer memory is too small
 * @retval #UCL_STACK_ERROR if there is an error with the buffer memory
 * @retval #UCL_INVALID_INPUT  if e is the pointer #NULL or p is the pointer #NULL
 * @retval #UCL_ERROR_B43 if p is not odd
 * @retval #UCL_ERROR_B44 if step 4.4 is not satisfied
 * @retval #UCL_ERROR_B452 if p is probably not prime
 * @retval #UCL_ERROR_B45 if step 4.5 is not satisfied
 * @retval #UCL_ERROR_B53 if (p is prime and) q is not odd
 * @retval #UCL_ERROR_B55 if (p is prime and) step 5.5 is not satisfied
 * @retval #UCL_ERROR_B562 if (p is prime and) q is probably not prime
 * @retval #UCL_ERROR_B56 if (p is prime and) step 5.6 is not satisfied
 * @retval #UCL_OK  if everything went ok (p and q probably primes numbers)
 *
 * @ingroup UCL_RSA2VS
 */
int ucl_fips186_4_kat_B33(u32 *p, u32 *q, u32 *e, u32 s);

#ifdef __cplusplus
}
#endif /* __cplusplus  */

#endif /* _UCL_RSA2VS_H_ */
