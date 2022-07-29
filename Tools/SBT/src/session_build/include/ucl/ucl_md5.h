/*============================================================================
 *
 * ucl_md5.h
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
 * Purpose : MD5
 *
 *==========================================================================*/
#ifndef PROFILE_1
#ifndef _UCL_MD5_H_
#define _UCL_MD5_H_
#include <ucl/ucl_hash.h>
#ifdef HASH_MD5

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */

/** @file ucl_md5.h
 * @defgroup UCL_MD5 MD5
 * RFC 1321: Designed by Ronald Rivest.
 *
 * @par Header:
 * @link ucl_md5.h ucl_md5.h @endlink
 *
 * MD5 is a data-digest algorithm designed by Ronald Rivest.@n
 * The algorithm takes as input a data of arbitrary length and
 * produces as output a 128-bit "fingerprint" or "data digest"
 * of the input.@n
 * The MD5 has been designed to be efficient on 32-bit machines.@n
 * @n
 * The data is padded by appending a single "1" bit and a set of "0" bits
 * so that the length of the padded data equals to 448 modulo 512.
 * Padding is always performed, even if the length of the data is
 * already congruent to 448, modulo 512.
 * Then, the length of the data is appended at the end.@n
 *
 * <b>MD5 Descriptor:</b> @n
 * @li Block length : 512 bits (64 bytes)
 * @li Hash length : 128 bits (4 bytes)
 *
 * @ingroup UCL_HASH
 */

/** <b>The MD5 context</b>.
 * This structure is associated to the 'step by step' process.
 *
 * @ingroup UCL_MD5
 */
struct ucl_md5_ctx {
    u32 state[4];  /**< Intermediate and then final hash */
    u32 count[2];  /**< Counter in bits                  */
    u8 buffer[64]; /**< Buffer                           */
};

/** <b>The MD5 context</b>.
 * @ingroup UCL_MD5
 */
typedef struct ucl_md5_ctx ucl_md5_ctx_t;

/** <b>MD5 core block size</b>.
 * Byte size of a MD5 core block.
 *
 * @ingroup UCL_MD5
 */
#define UCL_MD5_BLOCKSIZE 64
/** <b>MD5 hash size</b>.
 * Byte size of the output of MD5.
 *
 * @ingroup UCL_MD5
 */
#define UCL_MD5_HASHSIZE 16
/** <b>MD5 hash size</b>.
 * 32-bits word size of the output of MD5.
 *
 * @ingroup UCL_MD5
 */
#define UCL_MD5_HASHW32SIZE 5
#define UCL_MD5             13

/*============================================================================*/
/** <b>MD5</b>.
 * The complete process of MD5.
 *
 * @pre Hash byte length is equal to 16
 *
 * @param[out] hash         Pointer to the digest
 * @param[in]  data         Pointer to the data
 * @param[in]  data_byteLen Data byte length
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_OUTPUT If @p hash is #NULL
 *
 * @ingroup UCL_MD5
 */
int ucl_md5(u8* hash, u8* data, u32 data_byteLen);

/*============================================================================*/
/** <b>MD5 Init</b>.
 * The initialisation of MD5.
 *
 * @param[in,out] context Pointer to the context
 *
 * @return Error code
 *
 * @retval #UCL_OK            If no error occurred
 * @retval #UCL_INVALID_INPUT If @p context is #NULL
 *
 * @ingroup UCL_MD5
 */
int ucl_md5_init(ucl_md5_ctx_t* context);

/*============================================================================*/
/** <b>MD5 Core</b>.
 * The core of MD5.
 *
 * @param[in,out] context      Pointer to the context
 * @param[in]     data         Pointer to the data
 * @param[in]     data_byteLen Data byte length
 *
 * @warning #ucl_md5_init must be processed before, and #ucl_md5_finish
 *          must be processed to get the final hash.
 *
 * @return Error code
 *
 * @retval #UCL_OK  If no error occurred
 * @retval #UCL_NOP If @p data_byteLen = 0 or if @p data is the pointer #NULL
 *
 * @ingroup UCL_MD5
 */
int ucl_md5_core(ucl_md5_ctx_t* context, u8* data, u32 data_byteLen);

/*============================================================================*/
/** <b>MD5 Finish</b>.
 * Finish the process of MD5.
 *
 * @pre Hash byte length is equal to 16
 *
 * @param[out]    hash    Pointer to the digest
 * @param[in,out] context Pointer to the context
 *
 * @warning #ucl_md5_init and #ucl_md5_core must be processed before.
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_OUTPUT If @p hash is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  If @p context is the pointer #NULL
 *
 * @ingroup UCL_MD5
 */
int ucl_md5_finish(u8* hash, ucl_md5_ctx_t* context);

#ifdef __cplusplus
}
#endif /* _ cplusplus  */
#endif //HASH_MD5
#endif /* _UCL_MD5_H_ */
#endif //PROFILE_1
