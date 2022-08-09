/*============================================================================
 *
 * ucl_sha1.h
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
 * Purpose :
 *
 *==========================================================================*/
#ifndef PROFILE_2
#ifndef _UCL_SHA1_H_
#define _UCL_SHA1_H_

#include <ucl/ucl_hash.h>
#ifdef HASH_SHA1
#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */

/** @file ucl_sha1.h
 * @defgroup UCL_SHA1 SHA1
 * Secure Hash Algorithm One (SHA-1), see FIPS 180-1.
 *
 * @par Header:
 * @link ucl_sha1.h ucl_sha1.h @endlink
 *
 * The standard FIPS 180-1 specifies a Secure Hash Algorithm, SHA-1, for
 * computing a condensed representation of a data or a data file.
 * When a data of any length @f$ < 2^{64} @f$ bits is input, the SHA-1 produces
 * a 160-bit output called a data digest.@n
 * FIPS 180-1 has been superseded by FIPS 180-2 that includes SHA-1 as one of
 * its components.@n
 * @n
 * Given a data of any length, a single "1" bit is appended at the end of the
 * data. This bit must always be added. Then, a set of "0" bits are also
 * appended so that the data length is 448 modulo 512. Finally, the length
 * of the data is appended. @n
 *@n
 *
 * <b>SHA1 Descriptor:</b> @n
 * @li Block length : 512 bits
 * @li Hash length : 160 bits
 *
 * @ingroup UCL_HASH
 */

/** <b>The SHA1 context</b>.
 * This structure is associated to the 'step by step' process.
 *
 * @ingroup UCL_SHA1
 */

struct ucl_sha1_ctx {
    /** Intermediate and then final hash. */
    u32 state[5];
    /** Counter in bits. */
    u32 count[2];
    /** Buffer. */
    u8 buffer[64];
};

/** <b>The SHA1 context</b>.
 * @ingroup UCL_SHA1
 */

typedef struct ucl_sha1_ctx ucl_sha1_ctx_t;

/** <b>Core block size</b>.
 * Byte size of a SHA1 core block.
 *
 * @ingroup UCL_SHA1
 */
#define UCL_SHA1_BLOCKSIZE 64
/** <b>Hash size</b>.
 * Byte size of the output of SHA1.
 *
 * @ingroup UCL_SHA1
 */
#define UCL_SHA1          0
#define UCL_SHA1_HASHSIZE 20
/** <b>Hash size</b>.
 * 32-bits word size of the output of SHA1.
 *
 * @ingroup UCL_SHA1
 */
#define UCL_SHA1_HASHW32SIZE 5

/*============================================================================*/
/** <b>SHA1</b>.
 * The complete process of SHA1.
 *
 * @pre Hash byte length is equal to 20
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
 * @ingroup UCL_SHA1
 */
int ucl_sha1(u8* hash, u8* data, u32 data_byteLen);

/*============================================================================*/
/** <b>SHA1 Init</b>.
 * The initialisation of SHA1.
 *
 * @param[in,out] context Pointer to the context
 *
 * @return Error code
 *
 * @retval #UCL_OK            if no error occurred
 * @retval #UCL_INVALID_INPUT if @p context is #NULL
 *
 * @ingroup UCL_SHA1
 */
int ucl_sha1_init(ucl_sha1_ctx_t* context);

/*============================================================================*/
/** <b>SHA1 Core</b>.
 * The core of SHA1.
 *
 * @param[in,out] context      Pointer to the context
 * @param[in]     data         Pointer to the data
 * @param[in]     data_byteLen Data byte length
 *
 * @warning #ucl_sha1_init must be processed before, and #ucl_sha1_finish
 *          should be processed to get the final hash.
 *
 * @return Error code
 *
 * @retval #UCL_OK if no error occurred
 * @retval #UCL_NOP if @p data_byteLen = 0 or if @p data is the pointer #NULL
 *
 * @ingroup UCL_SHA1
 */
int ucl_sha1_core(ucl_sha1_ctx_t* context, u8* data, u32 data_byteLen);

/*============================================================================*/
/** <b>SHA1 Finish</b>.
 * Finish the process of SHA1.
 *
 * @pre Hash byte length is equal to 16
 *
 * @param[out]    hash Pointer to the digest
 * @param[in,out] context Pointer to the context
 *
 * @warning #ucl_sha1_init and #ucl_sha1_core must be processed before.
 *
 * @return Error code
 *
 * @retval #UCL_OK             if no error occurred
 * @retval #UCL_INVALID_OUTPUT if @p hash is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if @p context is the pointer #NULL
 *
 * @ingroup UCL_SHA1
 */
int ucl_sha1_finish(u8* hash, ucl_sha1_ctx_t* context);

#ifdef __cplusplus
}
#endif /* _ cplusplus  */
#endif //HASH_SHA1
#endif /* _UCL_SHA1_H_ */
#endif //PROFILE2
