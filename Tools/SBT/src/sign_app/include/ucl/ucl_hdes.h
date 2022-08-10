/*============================================================================
 *
 * ucl_hdes.h
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
 * Purpose : HDES, ISO/IEC 10118-2
 *
 *==========================================================================*/
#ifndef _UCL_HDES_H_
#define _UCL_HDES_H_

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */

/** @file ucl_hdes.h
 * @defgroup UCL_HDES HDES
 * Hash function two padding method 3, based on ISO/IEC 10118-2.
 *
 * @par Header:
 * @link ucl_hdes.h ucl_hdes.h @endlink
 *
 * ISO/IEC 10118-2, "Information Technology - Security Techniques -
 * Hash functions, Part 2: Hash functions using an n-bit block cipher
 * algorithm", 1994.
 *
 * <b>HDES Descriptor:</b> @n
 * @li Block length : 64 bits (8 bytes)
 * @li Hash length : 128 bits (4 bytes)
 *
 * @ingroup UCL_HASH
 */

/** <b>The HDES context</b>.
 * This structure is associated to the 'step by step' process.
 *
 * @ingroup UCL_HDES
 */
struct ucl_hdes_ctx_t {
    u32 state[4]; /**< Intermediate and then final hash */
    u32 count[2]; /**< Counter in bits                  */
    u8 buffer[8]; /**< Buffer                           */
};

/** <b>The HDES context</b>.
 * @ingroup UCL_HDES
 */
typedef struct ucl_hdes_ctx_t ucl_hdes_ctx_t;

/** <b>HDES core block size</b>.
 * Byte size of a HDES core block.
 *
 * @ingroup UCL_HDES
 */
#define UCL_HDES_BLOCKSIZE 8
/** <b>HDES hash size</b>.
 * Byte size of the HDES output.
 *
 * @ingroup UCL_HDES
 */
#define UCL_HDES_HASHSIZE 16

/*============================================================================*/
/** <b>HDES</b>.
 * The complete process of HDES.
 *
 * @param[out] hash         Pointer to the digest
 * @param[in]  data         Pointer to the data
 * @param[in]  data_byteLen Data byte length
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_INPUT  If one of the inputs is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT If the output is the pointer #NULL
 *
 * @ingroup UCL_HDES
 */
int ucl_hdes(u8* hash, u8* data, u32 data_byteLen);

/*============================================================================*/
/** <b>HDES Init</b>.
 * The initialisation of HDES.
 *
 * @param[in,out] context
 *
 * @return Error code
 *
 * @retval #UCL_OK            If no error occurred
 * @retval #UCL_INVALID_INPUT If @p context is #NULL
 *
 * @ingroup UCL_HDES
 */
int ucl_hdes_init(ucl_hdes_ctx_t* context);

/*============================================================================*/
/** <b>HDES Core</b>.
 * The core of HDES.
 *
 * @param[in,out] context      Pointer to the context
 * @param[in]     data         Pointer to the data
 * @param[in]     data_byteLen Data byte length
 *
 * @warning #ucl_hdes_init must be processed before, and #ucl_hdes_finish
 *          must be processed to get the final hash.
 *
 * @return Error code
 *
 * @retval #UCL_OK            If no error occurred
 * @retval #UCL_INVALID_INPUT If one of the inputs are the pointer #NULL
 * @retval #UCL_NOP           If @p data_byteLen = 0
 *
 * @ingroup UCL_HDES
 */
int ucl_hdes_core(ucl_hdes_ctx_t* context, u8* data, u32 data_byteLen);

/*============================================================================*/
/** <b>HDES Finish</b>.
 * Finish the process of HDES.
 *
 * @param[out] hash    Pointer to the digest
 * @param[in]  context Pointer to the context
 *
 * @warning #ucl_hdes_init and #ucl_hdes_core must be processed before.
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_OUTPUT If @ hash is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  If @p context is the pointer #NULL
 *
 * @ingroup UCL_HDES
 */
int ucl_hdes_finish(u8* hash, ucl_hdes_ctx_t* context);

#ifdef __cplusplus
}
#endif /* _ cplusplus  */

#endif /* _UCL_HDES_H_ */
