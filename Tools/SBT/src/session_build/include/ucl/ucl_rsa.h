/*============================================================================
 *
 * ucl_rsa.h
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
 * Purpose : RSA
 *
 *==========================================================================*/
#ifndef _UCL_RSA_H_
#define _UCL_RSA_H_

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */

/** @file ucl_rsa.h
 * @defgroup UCL_RSA RSA
 * The RSA cryptosystem.
 *
 * @par Header:
 * @link ucl_rsa.h ucl_rsa.h @endlink
 *
 * It named after its inventors R. Rivest, A. Shamir, and L. Adleman, is the
 * most widely used public-key cryptosystem. It may be used to provide both
 * secrecy and digital signatures and its security is based on the
 * dificulty of the integer factorization problem.@n
 * @n
 * <b>Principle:</b>@n
 * @n
 * Let @a e a small integer (in general @a 3 or @f$ 2^{16}+1 @f$), the public
 * exponent.@n
 * Let @f$ n = p \times q @f$, the modulus, such as @a p and @a q are two large primes, and
 * @f$ \gcd(\phi(n), e) = 1 @f$ where @f$ \phi(n) = (p-1) \times (q-1) @f$.@n
 * Let @f$ d = e^{-1} \bmod \phi(n) @f$, the private exponent.@n
 * @n
 * (@a n, @a e) is named the public key and (@a n, @a d) the private key.@n
 * @n
 * Encryption/Decryption primitives:@n
 * If @f$ 0 < m < n @f$ is a message, then the ciphetext is @f$ c = m^e \bmod n @f$
 * and @f$ m = c^d \bmod n @f$.@n
 * @n
 * Signature/Verification primitives:@n
 * If @f$ 0 < m < n @f$ is a message, then the signature is @f$ S = m^d \bmod n @f$ @n
 * To check a @a S' signature, calculate @f$ m' = S'^d \bmod n @f$ and compare with @a m.@n
 * @n
 * <b>CRT</b>:
 * In order to speed up the RSA calculation, the CRT can be applied.@n
 * @n
 * @note This section contains the RSA primitives compliants with
 *    @ref PKCS1 "PKCS #1 V2.1".
 *
 * @ingroup UCL_PKC
 */

/** <b>RSA Public Key Structure</b>.
 * Let @f$ n = p \times q @f$ and @a e the public exponent.
 *
 * @ingroup UCL_RSA
 */

struct ucl_rsa_public_key {
    /** The public exponent @p e. */
    u8 public_exponent[UCL_RSA_PUBLIC_EXPONENT_MAXSIZE];
    /** The modulus @p n. */
    u8 modulus[UCL_RSA_KEY_MAXSIZE];
    /** The modulus byte length. */
    u32 modulus_length;
    /** The public exponent byte length. */
    u32 public_exponent_length;
};

/** <b>RSA Public Key</b>.
 * @ingroup UCL_RSA
 */

typedef struct ucl_rsa_public_key ucl_rsa_public_key_t;

/** <b>RSA Private Key Structure</b>.
 * Let @f$ n = p \times q @f$ and @a d the private exponent.
 *
 * @ingroup UCL_RSA
 */

struct ucl_rsa_private_key {
    /** The private exponent @p d. */
    u8* private_exponent;
    /** The modulus @p n. */
    u8* modulus;
    /** The modulus byte length. */
    u32 modulus_length;
};

/** <b>RSA Private Key</b>.
 * @ingroup UCL_RSA
 */

typedef struct ucl_rsa_private_key ucl_rsa_private_key_t;

/** <b>RSA CRT Private Key Structure</b>.
 *
 * Let @f$ n = p \times q @f$ and @a d the private exponent.
 * @ingroup UCL_RSA
 */

struct ucl_rsa_crt_private_key {
    /** @f$ d_p = d \bmod (p-1) @f$. */
    u8 exponent1[UCL_RSA_KEY_MAXSIZE / 2];
    /** @f$ d_q = d \bmod (q-1) @f$. */
    u8 exponent2[UCL_RSA_KEY_MAXSIZE / 2];
    /** @a p. */
    u8 prime1[UCL_RSA_KEY_MAXSIZE / 2];
    /** @a q. */
    u8 prime2[UCL_RSA_KEY_MAXSIZE / 2];
    /** @f$ q^{-1} \bmod p @f$ */
    u8 coefficient[UCL_RSA_KEY_MAXSIZE / 2];
    /** The public exponent @p e. */
    u8 public_exponent[UCL_RSA_PUBLIC_EXPONENT_MAXSIZE];
    /** The public exponent byte length. */
    u32 public_exponent_length;
    /** The modulus byte length. */
    u32 modulus_length;
};

/** <b>RSA CRT Private Key</b>.
 *
 * @ingroup UCL_RSA
 */

typedef struct ucl_rsa_crt_private_key ucl_rsa_crt_private_key_t;

/** <b>RSA Parameters Generation</b>.
 * generate RSA keys pairs with FIPS 186-4 compliance.
 * for keys length=1024, it applies section B3.6,
 * for keys length=2048, 3072 or 4096 (as a FIPS 186-4 extension), it applies section B3.3
 * Generate @a p, @a q, @a n and @a d such as:@n
 * @f$ n = p \times q@f$ @n
 * @f$ e \times d = 1 \bmod (p-1)(q-1)@f$ @n
 *
 * @param[out]  n   The pointer to @a n
 * @param[out]  p   The pointer to @a p
 * @param[out]  q   The pointer to @a q
 * @param[out]  d   The pointer to @a d
 * @param[in]   e   The pointer to @a e
 * @param[in]   t   The length of @p e
 * @param[in]   s   The precision
 *
 * @return Error code
 *
 * @retval #UCL_OK
 * @retval #UCL_ERROR if the key length is not authorized or if the public exponent is not
 * authorized
 * @retval #UCL_STACK_ERROR if the stack is too small for allocating temp variables
 * @retval #UCL_INVALID_OUTPUT  if the output is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if one of the input is the pointer #NULL
 * @ingroup UCL_RSA
 */
int ucl_rsa_param_gen_fips186_4(u32* n, u32* p, u32* q, u32* d, u32* e, u32 t, u32 s);

/** <b>RSA Parameters Generation</b>.
 * it applies the method described in IEEE-1363.
 * Generate @a p, @a q, @a n and @a d such as:@n
 * @f$ n = p \times q@f$ @n
 * @f$ e \times d = 1 \bmod (p-1)(q-1)@f$ @n
 *
 * the parameters are in words (u32)
 *
 * @param[out]  n   The pointer to @a n
 * @param[out]  p   The pointer to @a p
 * @param[out]  q   The pointer to @a q
 * @param[out]  d   The pointer to @a d
 * @param[in]   e   The pointer to @a e
 * @param[in]   t   The length of @p e
 * @param[in]   s   The precision
 *
 * @return Error code
 *
 * @retval #UCL_OK
 * @retval #UCL_STACK_ERROR if the stack is too small for allocating temp variables
 * @retval #UCL_INVALID_OUTPUT  if the output is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if one of the input is the pointer #NULL
 *
 * @ingroup UCL_RSA
 */
int ucl_rsa_param_gen(u32* n, u32* p, u32* q, u32* d, u32* e, u32 t, u32 s);

/** <b> RSA CRT Parameters Generation</b>.
 * Generate @a p, @a q, @a n , @a dp, @a dq and @a qInv such as: @n
 * @f$ n = p \times q@f$ @n
 * @f$ e \times dp = 1 \bmod (p-1)@f$ @n
 * @f$ e \times dq = 1 \bmod (q-1)@f$ @n
 * @f$ q \times qInv = 1 \bmod p @f$ @n
 *
 * @param[out]  n       The pointer to @a n
 * @param[out]  p       The pointer to @a p
 * @param[out]  q       The pointer to @a q
 * @param[out]  dp      The pointer to @a dp
 * @param[out]  dq      The pointer to @a dp
 * @param[out]  qInv    The pointer to @a qInv
 * @param[in]   e       The pointer to @a e
 * @param[in]   t       The length of @p e
 * @param[in]   s       The precision
 *
 * @return Error code
 *
 * @retval #UCL_OK
 * @retval #UCL_STACK_ERROR if the stack is too small for allocating temp variables
 * @retval #UCL_INVALID_OUTPUT  if the output is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if one of the input is the pointer #NULL
 *
 * @ingroup UCL_RSA
 */
int ucl_rsa_crt_param_gen_legacy(
    u32* n, u32* p, u32* q, u32* dp, u32* dq, u32* qInv, u32* e, u32 t, u32 s);

/** <b> RSA CRT Parameters Generation</b>.
 * generate RSA keys pairs with FIPS 186-4 compliance.
 * for keys length=1024, it applies section B3.6, otherwise it applies section B.3.3.
 * Generate @a p, @a q, @a n , @a dp, @a dq and @a qInv such as: @n
 * @f$ n = p \times q@f$ @n
 * @f$ e \times dp = 1 \bmod (p-1)@f$ @n
 * @f$ e \times dq = 1 \bmod (q-1)@f$ @n
 * @f$ q \times qInv = 1 \bmod p @f$ @n
 *
 * @param[out]  n       The pointer to @a n
 * @param[out]  p       The pointer to @a p
 * @param[out]  q       The pointer to @a q
 * @param[out]  dp      The pointer to @a dp
 * @param[out]  dq      The pointer to @a dp
 * @param[out]  qInv    The pointer to @a qInv
 * @param[in]   e       The pointer to @a e
 * @param[in]   t       The length of @p e
 * @param[in]   s       The precision
 *
 * @return Error code
 *
 * @retval #UCL_OK
 * @retval #UCL_ERROR if the key length is not authorized or if the public exponent is not
 * authorized
 * @retval #UCL_STACK_ERROR if the stack is too small for allocating temp variables
 * @retval #UCL_INVALID_OUTPUT  if the output is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if one of the input is the pointer #NULL
 * @ingroup UCL_RSA
 */
int ucl_rsa_crt_param_gen_fips186_4(
    u32* n, u32* d, u32* p, u32* q, u32* dp, u32* dq, u32* qInv, u32* e, u32 t, u32 s);
/** <b> RSA CRT Parameters Generation</b>.
 * it applies the method described in IEEE-1363.
 * Generate @a p, @a q, @a n , @a dp, @a dq and @a qInv such as: @n
 * @f$ n = p \times q@f$ @n
 * @f$ e \times dp = 1 \bmod (p-1)@f$ @n
 * @f$ e \times dq = 1 \bmod (q-1)@f$ @n
 * @f$ q \times qInv = 1 \bmod p @f$ @n
 *
 * @param[out]  n       The pointer to @a n
 * @param[out]  p       The pointer to @a p
 * @param[out]  q       The pointer to @a q
 * @param[out]  dp      The pointer to @a dp
 * @param[out]  dq      The pointer to @a dp
 * @param[out]  qInv    The pointer to @a qInv
 * @param[in]   e       The pointer to @a e
 * @param[in]   t       The length of @p e
 * @param[in]   s       The precision
 *
 * @return Error code
 *
 * @retval #UCL_OK
 * @retval #UCL_STACK_ERROR if the stack is too small for allocating temp variables
 * @retval #UCL_INVALID_OUTPUT  if the output is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if one of the input is the pointer #NULL
 *
 * @ingroup UCL_RSA
 */
int ucl_rsa_crt_param_gen(
    u32* n, u32* d, u32* p, u32* q, u32* dp, u32* dq, u32* qInv, u32* e, u32 t, u32 s);

/** <b>RSA Parameters Generation</b>.
 * generate RSA keys pairs with FIPS 186-4 compliance.
 * for keys length=1024, it applies section B3.6, otherwise it applies section B.3.3.
 * Generate @a p, @a q, @a n and @a d such as:@n
 * @f$ n = p \times q@f$ @n
 * @f$ e \times d = 1 \bmod (p-1)(q-1)@f$ @n
 *
 * the parameters are in bytes (u8)
 *
 * @param[out]  n   The pointer to @a n
 * @param[out]  p   The pointer to @a p
 * @param[out]  q   The pointer to @a q
 * @param[out]  d   The pointer to @a d
 * @param[in]   e   The pointer to @a e
 * @param[in]   t   The length of @p e
 * @param[in]   s   The precision
 *
 * @return Error code
 *
 * @retval #UCL_OK
 * @retval #UCL_ERROR if the key length is not authorized or if the public exponent is not
 * authorized
 * @retval #UCL_STACK_ERROR if the stack is too small for allocating temp variables
 * @ingroup UCL_RSA
 */
int ucl_rsa_param_generation_fips186_4(
    u8* n, u8* p, u8* q, u8* d, u8* e, int t, int modulus_bytes_size);

/** <b>RSA Parameters Generation</b>.
 * it applies the method described in IEEE-1363.
 * Generate @a p, @a q, @a n and @a d such as:@n
 * @f$ n = p \times q@f$ @n
 * @f$ e \times d = 1 \bmod (p-1)(q-1)@f$ @n
 *
 * the parameters are in bytes (u8)
 *
 * @param[out]  n   The pointer to @a n
 * @param[out]  p   The pointer to @a p
 * @param[out]  q   The pointer to @a q
 * @param[out]  d   The pointer to @a d
 * @param[in]   e   The pointer to @a e
 * @param[in]   t   The length of @p e
 * @param[in]   modulus_bytes_size   The modulus size in bytes
 *
 * @return Error code
 *
 * @retval #UCL_OK
 * @retval #UCL_STACK_ERROR if the stack is too small for allocating temp variables
 * @retval #UCL_INVALID_OUTPUT  if the output is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if one of the input is the pointer #NULL
 *
 * @ingroup UCL_RSA
 */
int ucl_rsa_param_generation(u8* n, u8* p, u8* q, u8* d, u8* e, int t, int modulus_bytes_size);

/** <b> RSA CRT Parameters Generation</b>.
 * generate RSA keys pairs with FIPS 186-4 compliance.
 * for keys length=1024, it applies section B3.6, otherwise it applies section B.3.3.
 * Generate @a p, @a q, @a n , @a dp, @a dq and @a qInv such as: @n
 * @f$ n = p \times q@f$ @n
 * @f$ e \times dp = 1 \bmod (p-1)@f$ @n
 * @f$ e \times dq = 1 \bmod (q-1)@f$ @n
 * @f$ q \times qInv = 1 \bmod p @f$ @n
 *
 * the parameters are in bytes (u8)
 *
 * @param[out]  n       The pointer to @a n
 * @param[out]  p       The pointer to @a p
 * @param[out]  q       The pointer to @a q
 * @param[out]  dp      The pointer to @a dp
 * @param[out]  dq      The pointer to @a dp
 * @param[out]  qInv    The pointer to @a qInv
 * @param[in]   e       The pointer to @a e
 * @param[in]   t       The length of @p e
 * @param[in]   s       The precision
 *
 * @return Error code
 *
 * @retval #UCL_OK
 * @retval #UCL_ERROR if the key length is not authorized or if the public exponent is not
 * authorized
 * @retval #UCL_STACK_ERROR if the stack is too small for allocating temp variables
 * @retval #UCL_INVALID_OUTPUT  if the output is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if one of the input is the pointer #NULL
 * @ingroup UCL_RSA
 */
int ucl_rsa_crt_param_generation_fips186_4(
    u8* n, u8* d, u8* p, u8* q, u8* dp, u8* dq, u8* qInv, u8* e, u32 t, u32 modulus_bytes_size);

/** RSA CRT Parameters Generation
 * Generate two primes @a p & @q and calculate @f$ n = p \times q @f$ \n
 *
 * the parameters are in bytes (u8)
 *
 * @param[out] n       The pointer to @a n
 * @param[out] d       The pointer to @a d
 * @param[out] p       The pointer to @a p
 * @param[out] q       The pointer to @a q
 * @param[out] dp      The pointer to @a dp
 * @param[out] dp      The pointer to @a dp
 * @param[out]  qInv    The pointer to @a qInv
 * @param[in]   e       The pointer to @a e
 * @param[in]   t     The length of @p e
 * @param[in]   modulus_bytes_size  The precision
 *
 * @return Error code
 *
 * @retval #UCL_OK
 * @retval #UCL_STACK_ERROR if the stack is too small for allocating temp variables
 * @retval #UCL_INVALID_OUTPUT  if the output is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if one of the input is the pointer #NULL
 *
 * @ingroup UCL_RSA
 */
int ucl_rsa_crt_param_generation(
    u8* n, u8* d, u8* p, u8* q, u8* dp, u8* dq, u8* qInv, u8* e, u32 t, u32 modulusbytes_size);

/** <b>Miller-Rabin Test</b>.
 * If @p n is a large positive integer, the following probabilistic algorithm
 * will determine whether @p n is prime or composite, with arbitrarily small
 * probability of error. [Knu81]
 *
 * @param[in] n the number to be tested
 * @param[in] s the precision, i.e. the number size in words
 * @param[in] t the number of Miller-Rabin tests to perform
 *
 * @return Error code or Test value
 *
 * @retval #UCL_IS_PRIME     if the number n is prime
 * @retval #UCL_ISNOT_PRIME  if the number n is not prime
 * @retval #UCL_STACK_OVERFLOW if the buffer memory is too small
 * @retval #UCL_STACK_ERROR if there is an error with the buffer memory
 * @retval #UCL_INVALID_INPUT  if n is the pointer #NULL
 *
 * @ingroup UCL_RSA
 */
int fpa_is_prime(u32* n, u32 s, u32 t);

/** <b>prime number generation</b>.
 * prime number generation following IEEE1363 Annex A.16.11
 * Generate a prime number @p p between @p pMax and @p pMin for which @p p-1
 * is relatively prime to @p e
 *
 * @param[out] p the generated prime number
 * @param[in] e the public exponent
 * @param[in] s the precision, i.e. the number size in words
 *
 * @return Error code or Test value
 *
 * @retval #UCL_STACK_OVERFLOW if the buffer memory is too small
 * @retval #UCL_STACK_ERROR if there is an error with the buffer memory
 * @retval #UCL_INVALID_INPUT  if e is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT  if p is the pointer #NULL
 *
 * @retval #UCL_OK  if everything went ok
 *
 * @ingroup UCL_RSA
 */
int fpa_gen_prime(u32* p, u32* e, u32* pMin, u32* pMax, u32 s);

/** <b>FIPS 186-4 prime number generation</b>.
 * prime number generation compliant with FIPS 186-4 appendix B3.3
 *
 * Generate a prime number @p p for which @p p-1
 * is relatively prime to @p e
 *
 * @param[out] p the generated prime number
 * @param[in] e the public exponent
 * @param[in] s the precision, i.e. the number size in words
 *
 * @return Error code or Test value
 *
 * @retval #UCL_STACK_OVERFLOW if the buffer memory is too small
 * @retval #UCL_STACK_ERROR if there is an error with the buffer memory
 * @retval #UCL_INVALID_INPUT  if e is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT  if p is the pointer #NULL
 *
 * @retval #UCL_OK  if everything went ok
 *
 * @ingroup UCL_RSA
 */
int fpa_gen_prime_fips186_4(u32* p, u32* e, u32 t);

/** <b>RSA Primitive - Decryption</b>.
 * It's equivalent to generate a signature.
 *
 * @param[out] dst The result of decryption
 * @param[in] src The ciphertext
 * @param[in] d Private exponent
 * @param[in] n The modulus
 * @param[in] s The precision
 *
 * @return Error code
 *
 * @retval #UCL_OK     if no error occurred
 * @retval #UCL_INVALID_OUTPUT  if the output is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if one of the input is the pointer #NULL or if the src is greater
 * than the modulus
 * @retval #UCL_INVALID_PRECISION if @p s is out of range
 * @retval #UCL_RSA_INVALID_INPUT if @f$ src > n @f$
 *
 * @note This primitive is compliant with RSADP and RSAVP1 from @ref PKCS1
 *    "PKCS #1 V2.1"
 *
 * @ingroup UCL_RSA
 */
int ucl_rsa_decryptBlock(u32* dst, u32* src, u32* d, u32* n, u32 s);

/*============================================================================*/
/** <b>RSA Primitive - Decryption with CRT</b>.
 * It's equivalent to generate a signature.
 *
 * @param[out] dst  The result of decryption
 * @param[in] src  The ciphertext
 * @param[in] dp  Private exponent 1
 * @param[in] dq  Private exponent 2
 * @param[in] p  Private factor 1
 * @param[in] q  Private factor 2
 * @param[in] qInv Inverse of q mod p
 * @param[in] e  Public exponent
 * @param[in] sE  Precision of the public exponent
 * @param[in] n  The modulus
 * @param[in] s  The precision
 *
 * @return Error code
 *
 * @retval #UCL_OK     if no error occurred
 * @retval #UCL_INVALID_OUTPUT  if the output is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if one of the input is the pointer #NULL
 * @retval #UCL_INVALID_PRECISION if @p s is out of range
 * @retval #UCL_RSA_INVALID_INPUT if @f$ input > n @f$
 *
 * @note This primitive is compliant with RSADP and RSAVP1 from @ref PKCS1
 *    "PKCS #1 V2.1"
 *
 * @ingroup UCL_RSA
 */
int ucl_rsa_crt_decryptBlock(
    u32* dst, u32* src, u32* dp, u32* dq, u32* p, u32* q, u32* qInv, u32* e, u32 sE, u32* n, u32 s);

/*============================================================================*/
/** <b>RSA Primitive - Decryption with CRT</b>.
 * It's equivalent to generate a signature.
 * this function does not have any protection, so it SHALL NOT be used when protections are required
 * @param[out] dst  The result of decryption
 * @param[in] src  The ciphertext
 * @param[in] dp  Private exponent 1
 * @param[in] dq  Private exponent 2
 * @param[in] p  Private factor 1
 * @param[in] q  Private factor 2
 * @param[in] qInv Inverse of q mod p
 * @param[in] e  Public exponent
 * @param[in] sE  Precision of the public exponent
 * @param[in] n  The modulus
 * @param[in] s  The precision
 *
 * @return Error code
 *
 * @retval #UCL_OK     if no error occurred
 * @retval #UCL_INVALID_OUTPUT  if the output is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if one of the input is the pointer #NULL
 * @retval #UCL_INVALID_PRECISION if @p s is out of range
 * @retval #UCL_RSA_INVALID_INPUT if @f$ input > n @f$
 *
 * @note This primitive is compliant with RSADP and RSAVP1 from @ref PKCS1
 *    "PKCS #1 V2.1"
 *
 * @ingroup UCL_RSA
 */
int ucl_rsa_crt_decryptBlock_without_protection(
    u32* dst, u32* src, u32* dp, u32* dq, u32* p, u32* q, u32* qInv, u32* e, u32 sE, u32* n, u32 s);

/** <b>RSA Primitive - Encryption</b>.
 * It's equivalent to check a signature.
 *
 * @param[out] dst The result of encryption
 * @param[in] src The plaintext
 * @param[in] e Public exponent
 * @param[in] sE Precision of the public exponent
 * @param[in] n The modulus
 * @param[in] s The precision
 *
 * @return Error code
 *
 * @retval #UCL_OK     if no error occurred
 * @retval #UCL_INVALID_OUTPUT  if the output is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if one of the input is the pointer #NULL, or if the public exponent
 * size is too large or if the src is greater than the modulus
 * @retval #UCL_INVALID_PRECISION if @p s or @p sE is out of range
 * @retval #UCL_RSA_INVALID_INPUT if @f$ input > n @f$
 *
 * @note This primitive is compliant with RSAEP and RSASP1 from @ref PKCS1
 *    "PKCS #1 V2.1"
 *
 * @ingroup UCL_RSA
 */
int ucl_rsa_encryptBlock(u32* dst, u32* src, u32* e, u32 sE, u32* n, u32 s);

/** RSA CRT Parameters re-generation
 * Generate CRT parameters @a dp, @a dq, @qInv and @n from two primes @a p & @q
 *
 * @param[out] n       The pointer to @a n
 * @param[out] dp      The pointer to @a dp
 * @param[out] dp      The pointer to @a dp
 * @param[out]  qInv    The pointer to @a qInv
 * @param[in] p       The pointer to @a p
 * @param[in] q       The pointer to @a q
 * @param[in]   e       The pointer to @a e
 * @param[in]   elength     The length of @p e
 * @param[in]   nlength     The length of n
 *
 * @return Error code
 *
 * @retval #UCL_OK
 *
 * @ingroup UCL_FPA_PRIME
 */
int ucl_rsa_crt_parameters_legacy(
    u32* n, u32* dp, u32* dq, u32* qInv, u32* p, u32* q, u32* e, u32 elength, u32 nlength);

/** RSA CRT Parameters re-generation
 * Generate CRT parameters @a n,@a d,@a dp, @a dq, @qInv and @n from two primes @a p & @q
 *
 * @param[out] n       The pointer to @a n
 * @param[out] d      The pointer to @a d
 * @param[out] dp      The pointer to @a dp
 * @param[out] dp      The pointer to @a dp
 * @param[out]  qInv    The pointer to @a qInv
 * @param[in] p       The pointer to @a p
 * @param[in] q       The pointer to @a q
 * @param[in]   e       The pointer to @a e
 * @param[in]   elength     The length of @p e
 * @param[in]   nlength     The length of n
 *
 * @return Error code
 *
 * @retval #UCL_OK
 *
 * @ingroup UCL_FPA_PRIME
 */
int ucl_rsa_crt_parameters(
    u32* n, u32* d, u32* dp, u32* dq, u32* qInv, u32* p, u32* q, u32* e, u32 elength, u32 nlength);

/** RSA prime numbers computation
 * Generate the two primes @a p & @q from @a d and @n
 *
 * @param[out] p       The pointer to @a p
 * @param[out] q      The pointer to @a q
 * @param[in] n       The pointer to @a n, the modulus
 * @param[in] d       The pointer to @a d, the private exponent
 * @param[in]   modulus_bytes_size     The length of n, in bytes
 *
 * @return Error code
 *
 * @retval #UCL_OK
 * @retval #UCL_INVALID_INPUT if the input parametes are incorrect
 * @retval #UCL_INVALID_INPUT if the input parametes are incorrect
 * @retval #UCL_STACK_ERROR if the buffer size is too small (e.g. for the key size)
 * @retval #UCL_OK
 *
 * @warning the only supported public exponent values are 3 and 65537
 *
 * @ingroup UCL_RSA
 */

int ucl_rsa_recover_primes_from_secret_exponent(
    u8* p, u8* q, u8* n, u8* d, u8* e, int modulus_bytes_size);

#ifdef __cplusplus
}
#endif /* _ cplusplus  */

#endif /* _UCL_RSA_H_ */
