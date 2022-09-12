#ifndef UCL_ECIES_H_
#define UCL_ECIES_H_
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus  */
#include <ucl/ucl_retdefs.h>
#include <ucl/ucl_types.h>
#include <ucl/ucl_config.h>
#include <ucl/ucl_defs.h>
#include <ucl/ucl_sys.h>
/** @defgroup UCL_ECC_ECIES ECIES computation.
 * ECIES computation

 * Elliptic Curve Integrated Encryption Scheme, or ECIES, is a hybrid encryption system. ECIES has been standardized in ANSI X9.63, IEEE 1363a, ISO/IEC 18033-2, and SECG SEC-1.
 * ECIES combines a Key Encapsulation Mechanism (KEM) with a Data Encapsulation Mechanism (DEM). The system independently derives a bulk encryption key and a MAC key from a common secret. Data is first encrypted under a symmetric cipher, and then the cipher text is MAC'd under an authentication scheme. Finally, the common secret is encrypted under the public part of a public/private key pair. The output of the encryption function is the tuple {K,C,T}, where K is the encrypted common secret, C is the ciphertext, and T is the authentication tag. There is some hand waiving around the "common secret" since its actually the result of applying a Key Agreement function, and it uses the static public key and an ephemeral key pair (Wikipedia)
 * 
 * @ingroup UCL_ECC
 */
/** <b>ECIES encryption</b>.
 * this function performs a ECIES AES HMAC SHA256 encryption on the P-192 curve
 * 
 * 
 * @param[out]  u8 *rx,u8 *ry: the resulting point for encryption
 * @param[out]  u8 *crypto: the encrypted message
 * @param[out]  u8 *t: the resulting HMAC
 * @param[in]  u32 keylength: the curve length, in bytes, i.e. SECP192R1_BYTESIZE for P-192
 * @param[in]  u8 * xG,u8 *yG: the curve base point
 * @param[in]  u8 *xQ,u8 * yQ: the public key
 * @param[in]  u8 *a,u8 *n,u8 *p: the curve parameters,
 * @param[in]  u8 *m: the message to be encrypted
 * @param[in]  u32 MsgLng: the message length, in bytes
 *
 * @return Error code
 * 
 * @retval #UCL_OK              No error occurred
 * @retval #UCL_ERROR           otherwise
 *
 * @ingroup UCL_ECC_ECIES */

int ucl_ecies_encrypt_p192r1_aes_hmac_sha256(unsigned char *rx, unsigned char *ry,
                                             unsigned char *crypto, unsigned char *t,
                                             unsigned int keylength, unsigned char *xG,
                                             unsigned char *yG, unsigned char *xQ,
                                             unsigned char *yQ, unsigned char *a, unsigned char *n,
                                             unsigned char *p, unsigned char *m, unsigned MsgLng);
/** <b>ECIES decryption</b>.
 * this function performs a ECIES AES HMAC SHA256 decryption on the P-192 curve
 * 
 * 
 * @param[out]  u8 *rx,u8 *ry: the resulting point for encryption
 * @param[out]  u8 *crypto: the encrypted message
 * @param[out]  u8 *t: the resulting HMAC
 * @param[in]  u32 keylength: the curve length, in bytes, i.e. SECP192R1_BYTESIZE for P-192
 * @param[in]  u8 * xG,u8 *yG: the curve base point
 * @param[in]  u8 *xQ,u8 * yQ: the public key
 * @param[in]  u8 *a,u8 *n,u8 *p: the curve parameters,
 * @param[in]  u8 *m: the message to be encrypted
 * @param[in]  u32 MsgLng: the message length, in bytes
 *
 * @return Error code
 * 
 * @retval #UCL_OK              No error occurred
 * @retval #UCL_ERROR           otherwise
 *
 * @ingroup UCL_ECC_ECIES */
int ucl_ecies_decrypt_p192r1_aes_hmac_sha256(unsigned char *m, unsigned int keylength,
                                             unsigned char *xG, unsigned char *yG, unsigned char *a,
                                             unsigned char *n, unsigned char *p, unsigned char *d,
                                             unsigned char *rx, unsigned char *ry,
                                             unsigned char *crypto, int crypto_len,
                                             unsigned char *t);
/** <b>ECIES encryption</b>.
 * this function performs a ECIES AES HMAC SHA256 encryption on the P-256 curve
 * 
 * 
 * @param[out]  u8 *rx,u8 *ry: the resulting point for encryption
 * @param[out]  u8 *crypto: the encrypted message
 * @param[out]  u8 *t: the resulting HMAC
 * @param[in]  u32 keylength: the curve length, in bytes, i.e. SECP192R1_BYTESIZE for P-192
 * @param[in]  u8 * xG,u8 *yG: the curve base point
 * @param[in]  u8 *xQ,u8 * yQ: the public key
 * @param[in]  u8 *a,u8 *n,u8 *p: the curve parameters,
 * @param[in]  u8 *m: the message to be encrypted
 * @param[in]  u32 MsgLng: the message length, in bytes
 *
 * @return Error code
 * 
 * @retval #UCL_OK              No error occurred
 * @retval #UCL_ERROR           otherwise
 *
 * @ingroup UCL_ECC_ECIES */

int ucl_ecies_encrypt_p256r1_aes_hmac_sha256(u8 *rx, u8 *ry, u8 *crypto, u8 *t, u32 keylength,
                                             u8 *xG, u8 *yG, u8 *xQ, u8 *yQ, u8 *a, u8 *n, u8 *p,
                                             u8 *m, u32 MsgLng);

/** <b>ECIES decryption</b>.
 * this function performs a ECIES AES HMAC SHA256 decryption on the P-256 curve
 * 
 * 
 * @param[out]  u8 *rx,u8 *ry: the resulting point for encryption
 * @param[out]  u8 *crypto: the encrypted message
 * @param[out]  u8 *t: the resulting HMAC
 * @param[in]  u32 keylength: the curve length, in bytes, i.e. SECP192R1_BYTESIZE for P-192
 * @param[in]  u8 * xG,u8 *yG: the curve base point
 * @param[in]  u8 *xQ,u8 * yQ: the public key
 * @param[in]  u8 *a,u8 *n,u8 *p: the curve parameters,
 * @param[in]  u8 *m: the message to be encrypted
 * @param[in]  u32 MsgLng: the message length, in bytes
 *
 * @return Error code
 * 
 * @retval #UCL_OK              No error occurred
 * @retval #UCL_ERROR           otherwise
 *
 * @ingroup UCL_ECC_ECIES */

int ucl_ecies_decrypt_p256r1_aes_hmac_sha256(u8 *m, u32 keylength, u8 *xG, u8 *yG, u8 *a, u8 *n,
                                             u8 *p, u8 *d, u8 *rx, u8 *ry, u8 *crypto,
                                             int crypto_len, u8 *t);

/** <b>ECIES encryption</b>.
 * this generic function performs a ECIES AES HMAC SHA256 encryption on the proposed curve
 * using the ECC API introduced in the UCL 2.4.9
 * @param[out]  ucl_type_ecc_u8_affine_point Q: the resulting point for encryption
 * @param[out]  u8 *crypto: the encrypted message
 * @param[out]  u8 *t: the resulting HMAC
 * @param[in]  ucl_type_ecc_u8_affine_point pubkey: the public key
 * @param[in]  u8 *input: the message to be encrypted
 * @param[in]  u32 inputlength: the message length, in bytes
 * @param[in]  ucl_type_curve *curve_params: a pointer to the curve domain parameters structure
 * 
 * @return Error code
 * 
 * @retval #UCL_OK              No error occurred
 * @retval #UCL_STACK_ERROR     if not enough UCL buffer memory available
 * @retval #UCL_ERROR           otherwise
 *
 * @ingroup UCL_ECC_ECIES */
int ucl_ecies_encrypt_aes_hmac_sha256(ucl_type_ecc_u8_affine_point Q, u8 *crypto, u8 *t,
                                      ucl_type_ecc_u8_affine_point pubkey, u8 *input,
                                      u32 inputlength, ucl_type_curve *curve_params);

/** <b>ECIES decryption</b>.
 * this generic function performs a ECIES AES HMAC SHA256 decryption on the proposed curve
 * using the ECC API introduced in the UCL 2.4.9
 * @param[out]  u8 *message: the decrypted message
 * @param[int]  u8 *d: the receiving entity secret key
 * @param[in]  ucl_type_ecc_u8_affine_point pubkey: the emitting entity public key
 * @param[in]  u8 *crypto: the message to be decrypted
 * @param[in]  u32 cryptolength: the cryptogram length, in bytes
 * @param[in]  u8 *t: the HMAC to be checked
 * @param[in]  ucl_type_curve *curve_params: a pointer to the curve domain parameters structure
 *
 * @return Error code
 * 
 * @retval #UCL_OK              No error occurred
 * @retval #UCL_STACK_ERROR     if not enough UCL buffer memory available
 * @retval #UCL_ERROR           if the MAC verification fails
 *
 * @ingroup UCL_ECC_ECIES */
int ucl_ecies_decrypt_aes_hmac_sha256(u8 *message, u8 *d, ucl_type_ecc_u8_affine_point pubkey,
                                      u8 *crypto, int cryptolength, u8 *t,
                                      ucl_type_curve *curve_params);

#ifdef __cplusplus
}
#endif /* __cplusplus  */

#endif //UCL_ECIES_H
