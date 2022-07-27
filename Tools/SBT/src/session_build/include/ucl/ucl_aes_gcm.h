/*============================================================================
 *
 * ucl_aes_gcm.h [22-sep-17]
 *
 *==========================================================================*/
/*============================================================================
 *
 * Copyright (c) 2017 Maxim Integrated. All Rights Reserved. Do not disclose.
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
 * Purpose : AES-GCM and AES-GMAC API description
 *
 *==========================================================================*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus  */

/** @file ucl_aes_gcm.h
 * @defgroup UCL_GCM_AES AES GCM
 * Encrypt / Decrypt with AES in GCM (Galois Counter Mode)
 * provide authentication with GMAC (Galois MAC).
 *
 * @par Header:
 * @link ucl_aes_gcm.h ucl_aes_gcm.h @endlink
 *
 * @ingroup UCL_GCM
 */
/** <b>AES-GCM</b>.
 * GCM and GMAC internal table initialization.
 * from 2.5.4, this function call is useless
 * as the table is now hardcoded
 * @ingroup UCL_GCM_AES
 */

void ucl_gcm_init_r(void);

#ifdef LARGE_MEMORY
/** <b>AES-GCM</b>.
 * GCM and GMAC internal table precomputation
 * used for platforms with RAM memory large enough (requires 4KB free space)
 * this precomputation provides better performances
 *
 * @param[in]  h: u8 array; h is the encryption of the 16-byte null block  0...0
 *
 * @ingroup UCL_GCM_AES
 */
void ucl_gcm_init_prod(u8* h);
#endif

#ifdef LARGE_MEMORY
/** <b>AES-GCM</b>.
 * GCM and GMAC internal table precomputation
 * used for platforms with RAM memory large enough (requires 4KB free space)
 * this precomputation provides better performances
 *
 * @param[in]  key, the AES key
 * @param[in]  keylen, the AES key len in bytes
 * @param[in]  mode, either UCL_CIPHER_ENCRYPT or UCL_CIPHER_DECRYPT
 *
 * @retval #UCL_OK             no error occurred
 * @retval #UCL_INVALID_INPUT  one of the input is the pointer NULL or key length does not match or mode does not match
 *
 * @ingroup UCL_GCM_AES
 */
int ucl_gcm_init_m0(u8* key, u32 keylen, int mode);
#endif

/** <b>AES-GCM</b>.
 * computes the GCM data (crypto+tag) from plaintext and authenticated add. data
 * @param[out]  auth_tag: result, u8 array representing the tag
 * @param[in]   auth_tag_bit_len: number of bits of the result tag; values between 64 and 128
 * @param[out]  c: the result u8 array representing the output (crypto in encryption, plain in decryption); its size is the same than the plaintext one
 * @param[in] p: the input u8 array representing the input (plain in encryption, crypto in decryption);
 * @param[in] input_bit_length_high, input_bit_len_low: two words representing the bit size of the input;the  maximum value is 2^39-256
 *  @param[in] aad: the input u8 array representing the add.authenticated data
 *  @param[in] aad_bit_length_high, aad_bit_len_low: two words representing the bit size of the AAD; the maximum value is 2^64
 *  @param[in] key: the u8 array representing the AES key
 *  @param[in] iv: the input u8 array representing the IV
 *  @param[in] iv_bit_length_high, iv_bit_len_low: two words representing the bit size of the IV;
the maximum value is 2^64
 *  @param[in] mode: UCL_CIPHER_ENCRYPT or UCL_CIPHER_DECRYPT; determines if p is encrypted or decrypted
 *  @return Error code
 *
 * @retval #UCL_OK             no error occurred
 * @retval #UCL_INVALID_INPUT  one of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT one of the output is the pointer NULL
 *
 * @ingroup UCL_GCM_AES

 */
int ucl_aes_gcm_auth(u8* auth_tag, int auth_tag_bit_len, u8* c, u8* p, int input_bit_len_high,
                     int input_bit_len_low, u8* aad, int aad_bit_len_high, int aad_bit_len_low,
                     u8* key, u8* iv, int iv_bit_len_high, int iv_bit_len_low, int mode);
/** <b>AES-GMAC</b>.
 * computes the GMAC data (tag) from authenticated add. data
 * @param[out]  auth_tag: result, u8 array representing the tag
 * @param[in]   auth_tag_bit_len: number of bits of the result tag; values between 64 and 128
 * @param[in] aad: the input u8 array representing the add.authenticated data
 * @param[in] aad_bit_length_high, aad_bit_len_low: two words representing the bit size of the AAD; the maximum value is 2^64
 * @param[in] key: the u8 array representing the AES key
 * @param[in] iv: the input u8 array representing the IV
 * @param[in] iv_bit_length_high, iv_bit_len_low: two words representing the bit size of the IV;
the maximum value is 2^64
 * @param[in] mode: UCL_CIPHER_ENCRYPT or UCL_CIPHER_DECRYPT; determines if p is encrypted or decrypted
 * @return Error code
 *
 * @retval #UCL_OK             no error occurred
 * @retval #UCL_INVALID_INPUT  one of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT one of the output is the pointer NULL
 *
 * @ingroup UCL_GCM_AES

 */
int ucl_aes_gmac_auth(u8* auth_tag, int auth_tag_bit_len, u8* aad, int aad_bit_len_high,
                      int aad_bit_len_low, u8* key, u8* iv, int iv_bit_len_high,
                      int iv_bit_len_low);

/** <b>AES-GCM</b>.
 * computes the GCM data (crypto+tag) from plaintext and authenticated add. data
 * @param[out]  auth_tag: result, u8 array representing the tag
 * @param[in]   auth_tag_bit_len: number of bits of the result tag; values between 64 and 128
 * @param[out]  c: the result u8 array representing the output (crypto in encryption, plain in decryption); its size is the same than the plaintext one
 * @param[in] p: the input u8 array representing the input (plain in encryption, crypto in decryption);
 * @param[in] input_bit_length_high, input_bit_len_low: two words representing the bit size of the input;the  maximum value is 2^39-256
 *  @param[in] aad: the input u8 array representing the add.authenticated data
 *  @param[in] aad_bit_length_high, aad_bit_len_low: two words representing the bit size of the AAD; the maximum value is 2^64
 *  @param[in] key: the u8 array representing the AES key
 *  @param[in] keylen: a u32 value representing the AES key len in bytes
 *  @param[in] iv: the input u8 array representing the IV
 *  @param[in] iv_bit_length_high, iv_bit_len_low: two words representing the bit size of the IV;
the maximum value is 2^64
 *  @param[in] mode: UCL_CIPHER_ENCRYPT or UCL_CIPHER_DECRYPT; determines if p is encrypted or decrypted
 *  @return Error code
 *
 * @retval #UCL_OK             no error occurred
 * @retval #UCL_INVALID_INPUT  one of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT one of the output is the pointer NULL
 *
 * @ingroup UCL_GCM_AES

 */
int ucl_aes_gcm(u8* auth_tag, int auth_tag_bit_len, u8* c, u8* p, int input_bit_len_high,
                int input_bit_len_low, u8* aad, int aad_bit_len_high, int aad_bit_len_low, u8* key,
                u32 keylen, u8* iv, int iv_bit_len_high, int iv_bit_len_low, int mode);
/** <b>AES-GMAC</b>.
 * computes the GMAC data (tag) from authenticated add. data
 * @param[out]  auth_tag: result, u8 array representing the tag
 * @param[in]   auth_tag_bit_len: number of bits of the result tag; values between 64 and 128
 * @param[in] aad: the input u8 array representing the add.authenticated data
 * @param[in] aad_bit_length_high, aad_bit_len_low: two words representing the bit size of the AAD; the maximum value is 2^64
 * @param[in] key: the u8 array representing the AES key
 * @param[in] keylen: a u32 value representing the AES key len in bytes
 * @param[in] iv: the input u8 array representing the IV
 * @param[in] iv_bit_length_high, iv_bit_len_low: two words representing the bit size of the IV;
the maximum value is 2^64
 * @param[in] mode: UCL_CIPHER_ENCRYPT or UCL_CIPHER_DECRYPT; determines if p is encrypted or decrypted
 * @return Error code
 *
 * @retval #UCL_OK             no error occurred
 * @retval #UCL_INVALID_INPUT  one of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT one of the output is the pointer NULL
 *
 * @ingroup UCL_GCM_AES

 */
int ucl_aes_gmac(u8* auth_tag, int auth_tag_bit_len, u8* aad, int aad_bit_len_high,
                 int aad_bit_len_low, u8* key, u32 keylen, u8* iv, int iv_bit_len_high,
                 int iv_bit_len_low);

void ucl_gcm_ghash_core(u8* x, u8* h, u8* c, int n, int u, int m, int v);
int ucl_gcm_ghash_finish(u8* x, u8* h, u8* c, int n, int u, int m, int v, int input_bit_len_high,
                         int input_bit_len_low);

/** <b>AES-GCM multi-block initialization</b>.
 * the AES-GCM multi-block processing defines three steps: initialization, blocks processing, final processing
 * this function is the initialization step
 * the initialization step performs the H, Y_0 and the ghash initialization 
 * it then requires the additional authenticated data, the IV and the key
 * @param[out]  res: ghash starting value
 * @param[out]  h: encryption of the null vector
 * @param[out]  eky0: the encryption of the Y_0 value
 * @param[out]  y: the Y_0 value (usually the IV)
 *  @param[in] aad: the input u8 array representing the add.authenticated data
 *  @param[in] aad_bit_length_high, aad_bit_len_low: two words representing the bit size of the AAD; the maximum value is 2^64
 *  @param[in] key: the u8 array representing the AES key
 *  @param[in] keylen: a u32 value representing the AES key len in bytes
 *  @param[in] iv: the input u8 array representing the IV
 *  @param[in] iv_bit_length_high, iv_bit_len_low: two words representing the bit size of the IV;
the maximum value is 2^64
 *  @param[in] mode: UCL_CIPHER_ENCRYPT or UCL_CIPHER_DECRYPT; determines if p is encrypted or decrypted
 *  @return Error code
 *
 * @retval #UCL_OK             no error occurred
 * @retval #UCL_INVALID_INPUT  one of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT one of the output is the pointer NULL
 *
 * @ingroup UCL_GCM_AES

 */

int ucl_aes_gcm_init(u8* res, u8* h, u8* eky0, u8* y, u8* aad, int aad_bit_len_high,
                     int aad_bit_len_low, u8* key, u32 keylen, u8* iv, int iv_bit_len_high,
                     int iv_bit_len_low, int mode);

/** <b>AES-GCM multi-block core processing</b>.
 * the AES-GCM multi-block processing defines three steps: initialization, blocks core processing, final processing
 * this function is the blocks core processing
 * this function handles only full AES blocks, i.e. a multiple of 128 bits
 * if the input length is a multiple of 128 bits, the last block shall not be processed in this core processing but in the final processing
 * @param[out]  res: ghash intermediate value
 * @param[in] auth_tag_bit_len: the length of the tag, in bits (usually 128 bits)
 * @param[out]  c: output of the AES processing (either encryption or decryption, determined by mode)
 * @param[in]  p: input of the AES processing
 *  @param[in] input_bit_length_high, input_bit_len_low: two words representing the bit size of the input (p); the maximum value is 2^64; it has to correspond to a multiple of 128 bits (i.e. full AES blocks)
 *  @param[in] aad_bit_length_high, aad_bit_len_low: two words representing the bit size of the AAD; the maximum value is 2^64
 * @param[in]  h: encryption of the null vector
 * @param[out]  y: the Y value (incremented for each new block)
 *  @param[in] key: the u8 array representing the AES key
 *  @param[in] keylen: a u32 value representing the AES key len in bytes
 *  @param[in] mode: UCL_CIPHER_ENCRYPT or UCL_CIPHER_DECRYPT; determines if p is encrypted or decrypted
 *  @return Error code
 *
 * @retval #UCL_OK             no error occurred
 * @retval #UCL_INVALID_INPUT  one of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT one of the output is the pointer NULL
 *
 * @ingroup UCL_GCM_AES

 */
int ucl_aes_gcm_core(u8* res, int auth_tag_bit_len, u8* c, u8* p, int input_bit_len_high,
                     int input_bit_len_low, int aad_bit_len_high, int aad_bit_len_low, u8* h, u8* y,
                     u8* key, u32 keylen, int mode);

/** <b>AES-GCM multi-block final processing</b>.
 * the AES-GCM multi-block processing defines three steps: initialization, blocks processing, final processing
 * this function is the final processing
 * @param[out]  auth_tag: the final tag value
 * @param[in] auth_tag_bit_len: the length of the tag, in bits (usually 128 bits)
 * @param[int]  res: ghash intermediate value
 * @param[out]  c: output of the AES processing (either encryption or decryption, determined by mode)
 * @param[in]  p: input of the AES processing
 *  @param[in] input_bit_length_high, input_bit_len_low: two words representing the bit size of the input (p); the maximum value is 2^64; this can not be a zero length so even the input size is a multiple of 128 bits, the last block shall be processed within this final processing and not in the core processing
 *  @param[in] inputtot_bit_length_high, inputtot_bit_len_low: two words representing the bit size of all the input data, so including the blocks transferred in the core processing and the last bits transferred in the final processing; the maximum value is 2^64
 *  @param[in] aad_bit_length_high, aad_bit_len_low: two words representing the bit size of the AAD; the maximum value is 2^64
 * @param[in]  h: encryption of the null vector
 * @param[in]  eky0: the encryption of the Y_0 value
 * @param[in/out]  y: the Y value (incremented for each new block)
 *  @param[in] key: the u8 array representing the AES key
 *  @param[in] keylen: a u32 value representing the AES key len in bytes
 *  @param[in] mode: UCL_CIPHER_ENCRYPT or UCL_CIPHER_DECRYPT; determines if p is encrypted or decrypted
 *  @return Error code
 *
 * @retval #UCL_OK             no error occurred
 * @retval #UCL_INVALID_INPUT  one of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT one of the output is the pointer NULL
 *
 * @ingroup UCL_GCM_AES

 */

int ucl_aes_gcm_finish(u8* auth_tag, int auth_tag_bit_len, u8* res, u8* c, u8* p,
                       int input_bit_len_high, int input_bit_len_low, int inputtot_bit_len_high,
                       int inputtot_bit_len_low, int aad_bit_len_high, int aad_bit_len_low, u8* h,
                       u8* eky0, u8* y, u8* key, u32 keylen, int mode);

#ifdef __cplusplus
}
#endif /* __cplusplus  */
