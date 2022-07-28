/*============================================================================
 *
 * ucl_ripemd160.h
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
 * Purpose : RIPEMD160
 *
 *==========================================================================*/
#ifndef _UCL_RIMPEMD160_H_
#define _UCL_RIMPEMD160_H_
#include <ucl/ucl_hash.h>
#ifdef HASH_RIPEMD160

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */

/** @file ucl_ripemd160.h
 * @defgroup UCL_RIPEMD160 RIPEMD160
 * RIPEMD160 designed inside the RIPE european project.
 *
 * @par Header:
 * @link ucl_ripemd160.h ucl_ripemd160.h @endlink
 *
 * RIPEMD-160 is a 160-bit cryptographic hash function,
 * designed by Hans Dobbertin, Antoon Bosselaers, and Bart Preneel. @n
 * RIPEMD-160 is a strengthened version of RIPEMD with a 160-bit hash result.@n
 * RIPEMD-160 is also part of the ISO/IEC international standard
 * ISO/IEC 10118-3:2003 on dedicated hash functions @n
 * @n
 * <b>RIPEMD160 Descriptor:</b> @n
 * @li Block length : 512 bits
 * @li Hash length : 160 bits
 *
 * @ingroup UCL_HASH
 */


/** <b>The RIPEMD160 context</b>.
 * This structure is associated to the 'step by step' process.
 *
 * @ingroup UCL_RIPEMD160
 */

struct ucl_ripemd160_ctx
{
    /** Intermediate and then final hash. */
    u32 state[5];
    /** Counter in bits. */
    u32 count[2];
    /** Buffer. */
    u8 buffer[64];
};

/** <b>The RIPEMD160 context</b>.
 * @ingroup UCL_RIPEMD160
 */

typedef struct ucl_ripemd160_ctx ucl_ripemd160_ctx_t;

/** <b>Core block size</b>.
 * Byte size of a RIPEMD160 core block.
 *
 * @ingroup UCL_RIPEMD160
 */
#define UCL_RIPEMD160_BLOCKSIZE 64
/** <b>Hash size</b>.
 * Byte size of the output of RIPEMD160.
 *
 * @ingroup UCL_RIPEMD160
 */
#define UCL_RIPEMD160_HASHSIZE 20
/** <b>Hash size</b>.
 * 32-bits word size of the output of RIPEMD160.
 *
 * @ingroup UCL_RIPEMD160
 */
#define UCL_RIPEMD160_HASHW32SIZE 5


/*============================================================================*/
/** <b>RIPEMD160</b>.
 * The complete process of RIPEMD160.
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
 * @ingroup UCL_RIPEMD160
 */
int __API__ ucl_ripemd160(u8 *hash, u8 *data, u32 data_byteLen);


/** <b>RIPEMD160 Init</b>.
 * The initialisation of RIPEMD160.
 *
 * @param[in,out] context Pointer to the context
 *
 * @return Error code
 *
 * @retval #UCL_OK            if no error occurred
 * @retval #UCL_INVALID_INPUT if @p context is #NULL
 *
 * @ingroup UCL_RIPEMD160
 */
int __API__ ucl_ripemd160_init(ucl_ripemd160_ctx_t *context);


/** <b>RIPEMD160 Core</b>.
 * The core of RIPEMD160.
 *
 * @param[in,out] context      Pointer to the context
 * @param[in]     data         Pointer to the data
 * @param[in]     data_byteLen Data byte length
 *
 * @warning #ucl_ripemd160_init must be processed before, and
 *          #ucl_ripemd160_finish should be processed to get the 
 *          final hash.
 *
 * @return Error code
 *
 * @retval #UCL_OK  if no error occurred
 * @retval #UCL_NOP if @p data_byteLen = 0 or if @p data is the pointer #NULL
 *
 * @ingroup UCL_RIPEMD160
 */
int __API__ ucl_ripemd160_core(ucl_ripemd160_ctx_t *context, u8 *data,
                       u32 data_byteLen);


/** <b>RIPEMD160 Finish</b>.
 * Finish the process of RIPEMD160.
 *
 * @pre Hash byte length is equal to 20
 *
 * @param[out]    hash    Pointer to the digest
 * @param[in,out] context Pointer to the context
 *
 * @warning #ucl_ripemd160_init and #ucl_ripemd160_core must be processed 
 *          before.
 *
 * @return Error code
 *
 * @retval #UCL_OK             if no error occurred
 * @retval #UCL_INVALID_OUTPUT if @p hash is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if @p context is the pointer #NULL
 *
 * @ingroup UCL_RIPEMD160
 */
int __API__ ucl_ripemd160_finish(u8 *hash, ucl_ripemd160_ctx_t *context);


#ifdef __cplusplus
}
#endif /* _ cplusplus  */
#endif//HASH_RIPEMD160
#endif /* _UCL_RIMPEMD160_H_ */
