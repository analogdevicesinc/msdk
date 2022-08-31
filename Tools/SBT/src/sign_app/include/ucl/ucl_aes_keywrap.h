/*============================================================================
 *
 * ucl_aes_keywrap.h
 *
 * Copyright (c) 2015 Maxim Integrated. All Rights Reserved. Do not disclose.
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
 */
/*============================================================================
 *
 * Purpose : AES keywrap API
 *
 *==========================================================================*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus  */

/** @defgroup UCL_AES_KEYWRAP AES Key Wrap
 * AES keys wrap compliant with RFC 3394.
 * it uses the Advanced
 * Encryption Standard (AES) as a primitive to securely encrypt
 * plaintext key(s) with any associated integrity information and data,
 * such that the combination could be longer than the width of the AES
 * block size (128-bits).
 * @ingroup UCL_KEYWRAP
 */
/** <b>AES Key wrap</b>.
 * this function wraps a plain key with AES
 *
 * @param[out]  c: output, the pointer to the key wrapped with the AES
 * @param[in]  p: input, the pointer to the plain key, to be wrapped with the AES
 * @param[in]   p_size: input, the size in bytes of the plain key to be wrapped
 * @param[in]   key: input, the pointer to the AES key used for wrapping
 * @return Error code
 *
 * @retval #UCL_OK              No error occurred
 * @retval #UCL_INVALID_INPUT  if one of the inputs is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT  if one of the outputs is the pointer NULL
 *
 * @ingroup UCL_AES_KEYWRAP
 */
int ucl_aes_RFC3394_wrap(u8* c, u8* p, int byte_p_size, u8* key);
/** <b>AES Key unwrap</b>.
 * this function unwraps an encrypted key with AES
 *
 * @param[out]  p: output, the pointer to the key unwrapped with the AES
 * @param[in]  c: input, the pointer to the wrapped key, to be unwrapped with the AES
 * @param[in]   c_size: input, the size in bytes of the plain key to be wrapped
 * @param[in]   key: input, the pointer to the AES key used for wrapping
 * @return Error code
 *
 * @retval #UCL_OK              No error occurred
 * @retval #UCL_INVALID_INPUT  if one of the inputs is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT  if one of the outputs is the pointer NULL
 *
 * @ingroup UCL_AES_KEYWRAP */
int ucl_aes_RFC3394_unwrap(u8* p, u8* c, int byte_c_size, u8* key);
#ifdef __cplusplus
}
#endif /* __cplusplus  */
