/*============================================================================
 *
 * ucl_aes_ccm.h [16-sep-14]
 *
 *==========================================================================*/

/*******************************************************************************
* Copyright (C) 2015 Maxim Integrated Products, Inc., All rights Reserved.
* * This software is protected by copyright laws of the United States and
* of foreign countries. This material may also be protected by patent laws
* and technology transfer regulations of the United States and of foreign
* countries. This software is furnished under a license agreement and/or a
* nondisclosure agreement and may only be used or reproduced in accordance
* with the terms of those agreements. Dissemination of this information to
* any party or parties not specified in the license agreement and/or
* nondisclosure agreement is expressly prohibited.
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/
/*============================================================================
 *
 * Purpose : AES-CCM API description
 *
 *==========================================================================*/

#ifndef _UCL_AES_CCM_H_
#define _UCL_AES_CCM_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus  */

/** @file ucl_aes_ccm.h
 * @defgroup UCL_CCM_AES AES CCM
 * Encrypt / Decrypt with AES in CCM (Counter with CBC-MAC) mode.
 *
 * @par Header:
 * @link ucl_aes_ccm.h ucl_aes_ccm.h @endlink
 *
 * @ingroup UCL_CCM
 */

typedef struct aes_ccm {
    u8 key[UCL_AES_KEYLEN_256];
    int keylen;
    int M;
    int L;
    u8 Xi[UCL_AES_BLOCKSIZE];
    u8 T[UCL_AES_BLOCKSIZE];
    u8 nonce[UCL_AES_BLOCKSIZE];
    int current_block_len;
    int nbblocks_to_build;
    int noblock;
} aes_ccm_t;

/** <b>AES-CCM</b>.
 * Encrypt with AES in CCM (Counter with CBC-MAC) mode.
 *
 *
 * @param[out] c    Pointer to the cryptotext
 * @param[out] U    Pointer to the authentication field
 * @param[in]  M    authentication field bytes length
 * @param[in]  L    length field bytes length
 * @param[in]  msg    Pointer to the plaintext
 * @param[in]  msg_len    plaintext byte length
 * @param[in]  aad    additional authenticated data; can be NULL if the length is zero
 * @param[in]  aad_len    additional authenticated data byte length; if zero, no AAD
 * @param[in]  nonce    Pointer to the nonce (its length is 15-L bytes)
 * @param[in]  key    Pointer to the AES Key
 * @param[in]  keylen Key byte length:
 *                        @li #UCL_AES_KEYLEN_128
 *                        @li #UCL_AES_KEYLEN_192
 *                        @li #UCL_AES_KEYLEN_256
 *
 * @return Error code
 *
 * @retval #UCL_OK             no error occurred
 * @retval #UCL_INVALID_INPUT  one of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT one of the output is the pointer NULL
 *
 * @ingroup UCL_CCM_AES
 */
int ucl_aes_ccm_encrypt(u8* c, u8* U, int M, int L, u8* msg, int msg_len, u8* aad, int aad_len,
                        u8* nonce, u8* key, int keylen);

/** <b>AES-CCM</b>.
 * Initialize AES CCM encryption.
 * Requirements:
 * the AAD, Additional Authenticated Data, shall be known
 * the plain data length, shall be known
 * the nonce shall be known
 *
 * @param[out] ctxt    Pointer to a AES CCM context
 * @param[in]  M    authentication field bytes length
 * @param[in]  L    length field bytes length
 * @param[in]  msg_len    plaintext byte length
 * @param[in]  aad    additional authenticated data; can be NULL if the length is zero
 * @param[in]  aad_len    additional authenticated data byte length; if zero, no AAD
 * @param[in]  nonce    Pointer to the nonce (its length is 15-L bytes)
 * @param[in]  key    Pointer to the AES Key
 * @param[in]  keylen Key byte length:
 *                        @li #UCL_AES_KEYLEN_128
 *                        @li #UCL_AES_KEYLEN_192
 *                        @li #UCL_AES_KEYLEN_256
 *
 * @return Error code
 *
 * @retval #UCL_OK             no error occurred
 * @retval #UCL_INVALID_INPUT  one of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT one of the output is the pointer NULL
 *
 * @ingroup UCL_CCM_AES
 */

int ucl_aes_ccm_encrypt_init(aes_ccm_t* ctxt, int M, int L, int msg_len, u8* aad, int aad_len,
                             u8* nonce, u8* key, int keylen);

/** <b>AES-CCM</b>.
 * AES CCM encryption core computation.
 * Requirements:
 * the plain data length shall be known a multiple of 16 bytes, except for the last call
 *
 * @param[out] c    Pointer to the encrypted data
 * @param[out] ctxt    Pointer to a AES CCM context

 * @param[in]  msg    Pointer to the plaintext
 * @param[in]  msg_len    plaintext byte length
 *
 * @return Error code
 *
 * @retval #UCL_OK             no error occurred
 * @retval #UCL_INVALID_INPUT  one of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT one of the output is the pointer NULL
 *
 * @ingroup UCL_CCM_AES
 */
int ucl_aes_ccm_encrypt_core(u8* c, aes_ccm_t* ctxt, u8* msg, int msg_len);

/** <b>AES-CCM</b>.
 *  AES CCM encryption final step
 *
 * @param[out] U    Pointer to the authentication field
 * @param[in] ctxt    Pointer to a AES CCM context
 *
 * @return Error code
 *
 * @retval #UCL_OK             no error occurred
 * @retval #UCL_INVALID_INPUT  one of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT one of the output is the pointer NULL
 *
 * @ingroup UCL_CCM_AES
 */

int ucl_aes_ccm_encrypt_finish(u8* U, aes_ccm_t* ctxt);

/** <b>AES-CCM</b>.
 * Decrypt with AES in CCM (Counter with CBC-MAC) mode.
 *
 *
 * @param[out]  msg    Pointer to the plaintext
 * @param[in]  M    authentication field bytes length
 * @param[in]  L    length field bytes length
 * @param[in] c    Pointer to the cryptotext
 * @param[in]  c_len    cryptotext byte length
 * @param[in] U    Pointer to the authentication field
 * @param[in]  aad    additional authenticated data; can be NULL if the length is zero
 * @param[in]  aad_len    additional authenticated data byte length; if zero, no AAD
 * @param[in]  nonce    Pointer to the nonce (its length is 15-L bytes)
 * @param[in]  key    Pointer to the AES Key
 * @param[in]  keylen Key byte length:
 *                        @li #UCL_AES_KEYLEN_128
 *                        @li #UCL_AES_KEYLEN_192
 *                        @li #UCL_AES_KEYLEN_256
 *
 * @return Error code
 *
 * @retval #UCL_OK             no error occurred
 * @retval #UCL_INVALID_INPUT  one of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT one of the output is the pointer NULL
 * @retval  #UCL_ERROR   if the authentication field verification fails
 *
 * @ingroup UCL_CCM_AES
 */

int ucl_aes_ccm_decrypt(u8* msg, int M, int L, u8* c, int c_len, u8* U, u8* aad, int aad_len,
                        u8* nonce, u8* key, int keylen);

/** <b>AES-CCM</b>.
 * Initialize AES CCM decryption.
 * Requirements:
 * the AAD, Additional Authenticated Data, shall be known
 * the encrypted data length, shall be known
 * the nonce shall be known
 * the authentication field, U, shall be known
 * 
 * @param[out] ctxt    Pointer to a AES CCM context
 * @param[in]  M    authentication field bytes length
 * @param[in]  L    length field bytes length
 * @param[in]  c_len    plaintext byte length
 * @param[in]  U    authentication field
 * @param[in]  aad    additional authenticated data; can be NULL if the length is zero
 * @param[in]  aad_len    additional authenticated data byte length; if zero, no AAD
 * @param[in]  nonce    Pointer to the nonce (its length is 15-L bytes)
 * @param[in]  key    Pointer to the AES Key
 * @param[in]  keylen Key byte length:
 *                        @li #UCL_AES_KEYLEN_128
 *                        @li #UCL_AES_KEYLEN_192
 *                        @li #UCL_AES_KEYLEN_256
 *
 * @return Error code
 *
 * @retval #UCL_OK             no error occurred
 * @retval #UCL_INVALID_INPUT  one of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT one of the output is the pointer NULL
 *
 * @ingroup UCL_CCM_AES
 */

int ucl_aes_ccm_decrypt_init(aes_ccm_t* ctxt, int M, int L, int c_len, u8* U, u8* aad, int aad_len,
                             u8* nonce, u8* key, int keylen);
/** <b>AES-CCM</b>.
 * AES CCM decryption core computation.
 * Requirements:
 * the encrypted data length shall be known a multiple of 16 bytes, except for the last call
 *
 * @param[out] msg    Pointer to the plain data
 * @param[out] ctxt    Pointer to a AES CCM context

 * @param[in]  c    Pointer to the encrypted data
 * @param[in]  c_len    encrypted data byte length
 *
 * @return Error code
 *
 * @retval #UCL_OK             no error occurred
 * @retval #UCL_INVALID_INPUT  one of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT one of the output is the pointer NULL
 *
 * @ingroup UCL_CCM_AES
 */
int ucl_aes_ccm_decrypt_core(u8* msg, aes_ccm_t* ctxt, u8* c, int c_len);

/** <b>AES-CCM</b>.
 *  AES CCM decryption final step
 *
 * @param[in] ctxt    Pointer to a AES CCM context
 *
 * @return Error code
 *
 * @retval #UCL_OK             no error occurred and authentication field verification succeeds in 
 * @retval #UCL_INVALID_INPUT  one of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT one of the output is the pointer NULL
 * @retval #UCL_ERROR   if the authentication field verification fails
 *
 * @ingroup UCL_CCM_AES
 */
int ucl_aes_ccm_decrypt_finish(aes_ccm_t* ctxt);
#ifdef __cplusplus
}
#endif /* __cplusplus  */
#endif //UCL_AES_CCM_H
