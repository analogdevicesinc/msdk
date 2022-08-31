/*===========================================================================
 *
 *  ucl_stest.h
 *
 *==========================================================================*/
/*===========================================================================
 *
 * Copyright © 2009 Innova Card.
 * All Rights Reserved.
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
/*===========================================================================
 *
 * Purpose:
 *
 *==========================================================================*/
#ifndef UCL_STEST_H_
#define UCL_STEST_H_

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */

/** @file ucl_stest.h
 * @defgroup UCL_STEST Self Tests
 * Integrity Tests.
 *
 * @par Header:
 * @link ucl_stest.h ucl_stest.h @endlink
 *
 * This module provides functions to test UCL primitives.
 * Those functions must be used in the case of self tests are necessary.
 *
 * @ingroup UCL_MISC
 */

/** <b>DES Primitive self test</b>.
 * Test DES.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_cipher_des_stest(void);

/** <b>3DES Primitive self test</b>.
 * Test 3DES.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_cipher_3des_stest(void);

int ucl_cipher_3des_eee_stest(void);

/** <b>AES Primitive self test</b>.
 * Test AES 128/192/256 bits key length.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_cipher_aes_stest(void);

/** <b>RSA Primitive self test</b>.
 * Test RSA encryption, decryption and CRT decryption.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_pkc_rsa_stest(void);

/** <b>RSA public key Primitive self test</b>.
 * Test RSA encryption.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_pkc_public_rsa_stest(void);

/** <b>DSA Primitive self test</b>.
 * Test DSA encryption, decryption.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */

int ucl_pkc_dsa_stest(void);

/** <b>ECDSA secp224r1 SHA-1 self test</b>.
 * Test ECDSA function (verify/sign) for the secp224r1 curve and the SHA-1 hash function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_ecdsa_p224r1_sha1_stest(void);

/** <b>ECDSA secp224r1 SHA-1 self test</b>.
 * Test ECDSA function (verify/sign) for the secp224r1 curve and the SHA-1 hash function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_ecdsa_p224r1_sha224_stest(void);
/** <b>ECDSA secp521r1 SHA-512 self test</b>.
 * Test ECDSA function (verify/sign) for the secp521r1 curve and the SHA-512 hash function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_ecdsa_p521r1_sha512_stest(void);
/** <b>ECDSA secp384r1 SHA-384 self test</b>.
 * Test ECDSA function (verify/sign) for the secp384r1 curve and the SHA-384 hash function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_ecdsa_p384r1_sha384_stest(void);
/** <b>ECDSA secp160r1 SHA-256 self test</b>.
 * Test ECDSA function (verify/sign) for the secp160r1 curve and the SHA-256 hash function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_ecdsa_p160r1_sha256_stest(void);
/** <b>ECDSA secp160r1 SHA-1 self test</b>.
 * Test ECDSA function (verify/sign) for the secp160r1 curve and the SHA-1 hash function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_ecdsa_p160r1_sha1_stest(void);
/** <b>ECDSA secp192r1 SHA-1 self test</b>.
 * Test ECDSA function (verify/sign) for the secp192r1 curve and the SHA-1 hash function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_ecdsa_p192r1_sha1_stest(void);
/** <b>ECDSA secp192r1 SHA-256 self test</b>.
 * Test ECDSA function (verify/sign) for the secp192r1 curve and the SHA-256 hash function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_ecdsa_p192r1_sha256_stest(void);
/** <b>ECDSA secp256r1 SHA-256 self test</b>.
 * Test ECDSA function (verify/sign) for the secp256r1 curve and the SHA-256 hash function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_ecdsa_p256r1_sha256_stest(void);

/** <b>ECDSA secp256r1 SHA-1 self test</b>.
 * Test ECDSA function with the new API (verify) for the secp256r1 curve and the SHA-1 hash
 * function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_ecdsa_p256r1_sha1_stest(void);

// NEW API

/** <b>ECDSA secp224r1 SHA-1 self test</b>.
 * Test ECDSA function with the new API (verify/sign) for the secp224r1 curve and the SHA-1 hash
 * function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_ecdsa_p224r1_sha1_selftest(void);
/** <b>ECDSA secp224r1 SHA-224 self test</b>.
 * Test ECDSA function with the new API (verify/sign) for the secp224r1 curve and the SHA-224 hash
 * function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_ecdsa_p224r1_sha224_selftest(void);
/** <b>ECDSA secp521r1 SHA-512 self test</b>.
 * Test ECDSA function with the new API (verify/sign) for the secp521r1 curve and the SHA-512 hash
 * function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_ecdsa_p521r1_sha512_selftest(void);
/** <b>ECDSA secp384r1 SHA-384 self test</b>.
 * Test ECDSA function with the new API (verify/sign) for the secp384r1 curve and the SHA-384 hash
 * function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_ecdsa_p384r1_sha384_selftest(void);
/** <b>ECDSA secp160r1 SHA-256 self test</b>.
 * Test ECDSA function with the new API (verify/sign) for the secp160r1 curve and the SHA-256 hash
 * function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_ecdsa_p160r1_sha256_selftest(void);
/** <b>ECDSA secp160r1 SHA-1 self test</b>.
 * Test ECDSA function with the new API (verify/sign) for the secp160r1 curve and the SHA-1 hash
 * function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_ecdsa_p160r1_sha1_selftest(void);
/** <b>ECDSA secp192r1 SHA-1 self test</b>.
 * Test ECDSA function with the new API (verify/sign) for the secp192r1 curve and the SHA-1 hash
 * function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_ecdsa_p192r1_sha1_selftest(void);
/** <b>ECDSA secp192r1 SHA-256 self test</b>.
 * Test ECDSA function with the new API (verify/sign) for the secp192r1 curve and the SHA-256 hash
 * function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_ecdsa_p192r1_sha256_selftest(void);
/** <b>ECDSA secp256r1 SHA-256 self test</b>.
 * Test ECDSA function with the new API (verify/sign) for the secp256r1 curve and the SHA-256 hash
 * function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_ecdsa_p256r1_sha256_selftest(void);

/** <b>ECDSA secp256r1 SHA-1 self test</b>.
 * Test ECDSA function with the new API (verify) for the secp256r1 curve and the SHA-1 hash
 * function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_ecdsa_p256r1_sha1_selftest(void);

/** <b>HDES self test</b>.
 * Test HDES hash function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_hash_hdes_stest(void);

/** <b>SHA256 self test</b>.
 * Test SHA256 hash function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_hash_sha256_stest(void);

/** <b>SHA384 self test</b>.
 * Test SHA384 hash function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_hash_sha384_stest(void);

/** <b>SHA224 self test</b>.
 * Test SHA224 hash function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_hash_sha224_stest(void);

/** <b>SHA512 self test</b>.
 * Test SHA512 hash function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_hash_sha512_stest(void);

/** <b>SHA1 self test</b>.
 * Test SHA1 hash function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_hash_sha1_stest(void);

/** <b>MD5 self test</b>.
 * Test MD5 hash function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 */
int ucl_hash_md5_stest(void);

/** <b>SM3 self test</b>.
 * Test SM3 hash function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 */
int ucl_hash_sm3_stest(void);

/** <b>RIPEMD160 self test</b>.
 * Test RIPEMD160 hash function.
 *
 * @return Error code
 *
 * @retval UCL_OK    Test passed
 * @retval UCL_ERROR Test failed
 *
 * @ingroup UCL_STEST
 */
int ucl_hash_ripemd160_stest(void);

int ucl_hmac_sha256_stest(void);
int ucl_hmac_sha224_stest(void);
int ucl_hmac_sha384_stest(void);
int ucl_hmac_sha512_stest(void);
int ucl_hmac_sha1_stest(void);

#ifdef __cplusplus
}
#endif /* _ cplusplus  */

#endif /*UCL_STEST_H_*/
