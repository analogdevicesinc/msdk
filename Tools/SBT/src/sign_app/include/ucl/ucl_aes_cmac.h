#ifndef _UCL_AES_CMAC_H_
#define _UCL_AES_CMAC_H_
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus  */

#include "ucl/ucl_aes.h"
/** @file ucl_aes_cmac.h
 * @defgroup UCL_CMAC_AES AES CMAC
 * process CMAC authentication with AES
 *
 * @par Header:
 * @link ucl_aes_cmac.h ucl_aes_cmac.h @endlink
 *
 * @ingroup UCL_CMAC
 */

/** <b>AES-CMAC</b>.
 * AES CMAC sub keys generation
 * K1 and K2 are the 128-bit sub-keys generated from the initial key
 * using algorithm defined in NIST SP800-38D, section 6.1
 * @param[out] k1  Pointer to the subkey k1
 * @param[in] k2   Pointer to the subkey k2
 * @param[in] key  Pointer to the initial AES Key
 * @param[in] keylen Key byte length:
 *                         @li #UCL_AES_KEYLEN_128
 *                         @li #UCL_AES_KEYLEN_192
 *                         @li #UCL_AES_KEYLEN_256
 * @return Error code
 *
 * @retval #UCL_OK    No error occurred
 * @retval #UCL_INVALID_INPUT One of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT One of the output is the pointer NULL
 *
 * @ingroup UCL_CMAC_AES
 */
int ucl_aes_subkey_generation(u8* k1, u8* k2, u8* key, u32 keylen);

/** <b>AES-CMAC</b>.
 * AES CMAC tag computation
 *
 * @param[out] t  Pointer to the computed CMAC
 * @param[in] tlen   computed CMAC byte length (caution: in the SP800-38D, this is expressed in bit)
 * @param[in] src   Pointer to the message data
 * @param[in] len  message byte length
 * @param[in] key  Pointer to the AES Key
 * @param[in] keylen Key byte length:
 *                         @li #UCL_AES_KEYLEN_128
 *                         @li #UCL_AES_KEYLEN_192
 *                         @li #UCL_AES_KEYLEN_256
 * @return Error code
 *
 * @retval #UCL_OK    No error occurred
 * @retval #UCL_INVALID_INPUT One of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT One of the output is the pointer NULL
 * @retval #UCL_ERROR an error has occured in subkeys generation
 *
 * restrictions
 * tlen is expressed in number of bytes
 * len is expressed in number of bytes
 * @ingroup UCL_CMAC_AES */
int ucl_aes_cmac_compute(u8* t, int tlen, u8* src, u32 len, u8* key, u32 keylen);

/** <b>AES-CMAC</b>.
 * AES CMAC tag verification
 *
 * @param[int] t  Pointer to the proposed CMAC
 * @param[in] tlen   CMAC byte length (caution: in the SP800-38D, this is expressed in bit)
 * @param[in] src   Pointer to the message data
 * @param[in] len  message byte length
 * @param[in] key  Pointer to the AES Key
 * @param[in] keylen Key byte length:
 *                         @li #UCL_AES_KEYLEN_128
 *                         @li #UCL_AES_KEYLEN_192
 *                         @li #UCL_AES_KEYLEN_256
 * @return Error code
 *
 * @retval #UCL_OK    No error occurred
 * @retval #UCL_INVALID_INPUT One of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT One of the output is the pointer NULL
 * @retval #UCL_ERROR the tag is not verified or an error occured in tag recomputation
 *
 * restrictions
 * tlen is expressed in number of bytes
 * len is expressed in number of bytes
 * @ingroup UCL_CMAC_AES
 */
int ucl_aes_cmac_verify(u8* t, int tlen, u8* src, u32 len, u8* key, u32 keylen);
#ifdef __cplusplus
}
#endif /* __cplusplus  */

#endif
