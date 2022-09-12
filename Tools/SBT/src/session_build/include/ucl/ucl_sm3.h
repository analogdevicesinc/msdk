/**
 * \file sm3.h
 * thanks to Xyssl
 * SM3 standards:http://www.oscca.gov.cn/News/201012/News_1199.htm
 * author:goldboar
 * email:goldboar@163.com
 * 2011-10-26
 */
#include "ucl/ucl_config.h"
#include "ucl/ucl_defs.h"
#include "ucl/ucl_retdefs.h"
#include "ucl/ucl_types.h"

#ifndef UCL_SM3_H
#define UCL_SM3_H
#include <ucl/ucl_hash.h>
//#ifdef HASH_SM3

#define UCL_SM3_BLOCKSIZE 64

/** @file ucl_sm3.h
 * @defgroup UCL_SM3 SM3
 * Secure Hash Algorithm 256, from FIPS 180-2.
 *
 * @par Header:
 * @link ucl_sm3.h ucl_sm3.h @endlink
 *
 * SM3 is a data-digest algorithm and a component of
 * the chinese cryptographic algorithms standard (see http://www.oscca.gov.cn/UpFile /20101222141857786.pdf. and draft-shen-sm3-hash-00
 * The algorithm takes as input a data of arbitrary length and
 * produces as output a 256-bit "fingerprint" or "data digest"
 * of the input.@n
 * @n
 *
 * <b>SM3 Descriptor:</b> @n
 * @li Block length : 512 bits
 * @li Hash length : 256 bits
 *
 * @ingroup UCL_HASH
 */

/** <b>Hash size</b>.
 * Byte size of the output of SM3.
 *
 * @ingroup UCL_SM3
 */
#define UCL_SM3 7
#define UCL_SM3_HASHSIZE 32

typedef struct {
    u32 total[2]; /*!< number of bytes processed  */
    u32 state[8]; /*!< intermediate digest state  */
    u8 buffer[64]; /*!< data block being processed */

    u8 ipad[64]; /*!< HMAC: inner padding        */
    u8 opad[64]; /*!< HMAC: outer padding        */
} ucl_sm3_ctx_t;

#ifdef __cplusplus
extern "C" {
#endif

/** <b>SM3 Init</b>.
 * The initialisation of SM3.
 *
 * @param[in,out] context Pointer to the context
 *
 * @return Error code
 *
 * @retval #UCL_OK            if no error occurred
 * @retval #UCL_INVALID_INPUT if @p context is #NULL
 *
 * @ingroup UCL_SM3
 */
void ucl_sm3_init(ucl_sm3_ctx_t *ctx);

/** <b>SM3 update</b>.
 * The core of SM3.
 *
 * @param[in,out] context      Pointer to the context
 * @param[in]     input        Pointer to the data
 * @param[in]     ilen          Data byte length
 *
 * @warning #ucl_sm3_init must be processed before, and
 *          #ucl_sm3_finish should be processed to get the final hash.
 *
 * @return Error code
 *
 * @retval #UCL_OK  if no error occurred
 * @retval #UCL_NOP if @p data_byteLen = 0 or if @p data is the pointer #NULL
 *
 * @ingroup UCL_SM3
 */
void ucl_sm3_update(ucl_sm3_ctx_t *ctx, u8 *input, int ilen);

/** <b>SM3 Finish</b>.
 * Finish the process of SM3.
 *
 * @pre Hash byte length is equal to 32
 *
 * @param[out]    hash Pointer to the digest
 * @param[in,out] context Pointer to the context
 *
 * @warning #ucl_sm3_init and #ucl_sm3_core must be processed before.
 *
 * @return Error code
 *
 * @retval #UCL_OK             if no error occurred
 * @retval #UCL_INVALID_OUTPUT if @p hash is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if @p context is the pointer #NULL
 *
 * @ingroup UCL_SM3
 */
void ucl_sm3_finish(u8 *output, ucl_sm3_ctx_t *ctx);

/** <b>SM3</b>.
 * The complete process of SM3.
 *
 * @pre Hash byte length is equal to 32
 *
 * @param[out] hash         Pointer to the digest
 * @param[in]  data         Pointer to the data
 * @param[in]  data_byteLen Data byte length
 *
 * @return Error code
 *
 * @retval #UCL_OK             if no error occurred
 * @retval #UCL_INVALID_OUTPUT if @p hash is #NULL
 *
 * @ingroup UCL_SM3
 */
void ucl_sm3(u8 *output, u8 *input, int ilen);

/* @defgroup UCL_HMAC_SM3 HMAC SM3
 *
 * @par Header:
 *
 * @ingroup UCL_HMAC
 */

/** <b>The initialisation of HMAC-SM3</b>.
 *
 * @param[in,out] ctx     The SM3 context
 * @param[in]     key         The key
 * @param[in]     keylen The key byte length
 *
 * @return Error code
 *
 * @retval #UCL_OK            If no error occurred
 * @retval #UCL_INVALID_INPUT If @p context or @p key is #NULL
 *
 * @ingroup UCL_HMAC_SM3
 */
void ucl_sm3_hmac_start(ucl_sm3_ctx_t *ctx, u8 *key, int keylen);

/** <b>The core of SM3 </b>.
 *
 * @param[in,out] ctx The SM3 context
 * @param[in]     input    Data
 * @param[in]     ilen Data byte length
 *
 * @warning #ucl_hmac_sm3_start must be processed before.
 *
 * @return Error code
 *
 * @retval #UCL_OK            If no error occurred
 * @retval #UCL_INVALID_INPUT If one of the inputs are the pointer #NULL
 * @retval #UCL_NOP           If @p dataLen = 0
 *
 * @ingroup UCL_HMAC_SM3
 */

void ucl_sm3_hmac_update(ucl_sm3_ctx_t *ctx, u8 *input, int ilen);

/** <b>Finish the process of SM3</b>.
 *
 * @param[in]  ctx     The SM3 context
 * @param[out] output         The hmac of the message
 *
 * @warning #ucl_hmac_sm3_hmac_start and
 *          #ucl_hmac_sm3_hmac_update must be processed before.
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_OUTPUT If @ hmac is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  If @p context or @p key is the pointer #NULL
 *
 * @ingroup UCL_HMAC_SM3
 */
void ucl_sm3_hmac_finish(ucl_sm3_ctx_t *ctx, u8 *output);

/** <b>The complete process of HMAC-SM3</b>.
 *
 * @param[in]  key             The key
 * @param[in]  keylen     The key byte length
 * @param[in]  input         The message
 * @param[in]  ilen The byte length of the message
 * @param[out] output             The 128-bit hmac of the message
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_INPUT  If one of the inputs is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT If the output is the pointer #NULL
 *
 * @ingroup UCL_HMAC_SM3
 */
void ucl_sm3_hmac(u8 *key, int keylen, u8 *input, int ilen, u8 *output);

/** <b>SM3 KDF</b>.
 * the KDF based on SM3 as defined in the draft-shen-ecdsa-01 sections 6 and 7.
 *
 *
 * @param[out] *secret_key         Pointer to the KDF result
 * @param[in]  *z         Pointer to the data
 * @param[in]  zlen Data byte length
 * @param[in]  klen KDF result len, in bits
 *
 * limitation: klen shall be a multiple of 8
 *
 * @return Error code
 *
 * @retval #UCL_OK             if no error occurred
 * @retval #UCL_INVALID_OUTPUT if @p hash is #NULL
 *
 * @ingroup UCL_SM3
 */
int kdf_sm2_sm3(u8 *secret_key, u8 *z, int zlen, int klen);

#ifdef __cplusplus
}
//#endif//HASH_SM3
#endif //UCL_SM3.h

#endif /* ucl_sm3.h */
