/*============================================================================
 *
 * ucl_olddefs.h
 *
 *==========================================================================*/
/*============================================================================
 *
 * Copyright © 2009 Innova Card.
 * All Rights Reserved.
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
 * Description:
 *
 *==========================================================================*/
#ifndef _UCL_OLDDEFS_H_
#define _UCL_OLDDEFS_H_

/** <b>32-bit Word</b>.
 *
 * @ingroup UCL_DEFINES_D
 */
#define WORD32 unsigned int
/** <b>16-bit Word</b>.
 *
 * @ingroup UCL_DEFINES_D
 */
#define WORD16 unsigned short
/** <b>8-bit Word</b>.
 *
 * @ingroup UCL_DEFINES_D
 */
#define WORD8 unsigned char
/** <b>Altenative 8-bit Word definition</b>.
 *
 * @ingroup UCL_DEFINES_D
 */
#define BYTE unsigned char

#define ucl_3des_ctx_type ucl_3des_ctx_t
#define ucl_des_ctx_type ucl_des_ctx_t
#define ucl_tdes_ctx_type ucl_tdes_ctx_t
#define ucl_sha1_ctx_type ucl_sha1_ctx_t
#define ucl_sha256_ctx_type ucl_sha256_ctx_t
#define ucl_md5_ctx_type ucl_md5_ctx_t
#define ucl_hdes_ctx_type ucl_hdes_ctx_t
#define ucl_rsa_public_key_type ucl_rsa_public_key_t
#define ucl_rsa_private_key_type ucl_rsa_private_key_t
#define ucl_rsa_crt_private_key_type ucl_rsa_crt_private_key_t

#define ucl_3des_ctx_t ucl_3des_ctx_t
#define ucl_des_ctx_t ucl_des_ctx_t
#define ucl_tdes_ctx_t ucl_tdes_ctx_t
#define ucl_sha1_ctx_t ucl_sha1_ctx_t
#define ucl_sha256_ctx_t ucl_sha256_ctx_t
#define ucl_md5_ctx_t ucl_md5_ctx_t
#define ucl_hdes_ctx_t ucl_hdes_ctx_t
#define ucl_rsa_public_key_t ucl_rsa_public_key_t
#define ucl_rsa_private_key_t ucl_rsa_private_key_t
#define ucl_rsa_crt_private_key_t ucl_rsa_crt_private_key_t

#endif /*_UCL_OLDDEFS_H_*/
