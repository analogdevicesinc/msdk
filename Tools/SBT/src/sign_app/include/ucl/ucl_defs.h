/*============================================================================
 *
 * ucl_defs.h
 *
 *==========================================================================*/
/*============================================================================
 *
 * Copyright © 2009 Innova Card.
 * All Rights Reserved.
 * Copyright © 2009-2014 Maxim Integrated Products, Inc.
 *  All Rights Reserved. Do not disclose.
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
 * Description: Usual definitions
 *
 *==========================================================================*/
#ifndef _UCL_DEFS_H_
#define _UCL_DEFS_H_

/** @defgroup UCL_DEFINITIONS Definitions
 *
 */

/** @defgroup UCL_DEFINES Defines
 *
 * @ingroup UCL_DEFINITIONS
 */

#ifndef NULL
#define NULL 0x0
#endif

/*
 * Algorithm masks and types.
 */
#define UCL_ALG_TYPE_MASK 0x000000ff
#define UCL_ALG_TYPE_CIPHER 0x00000001
#define UCL_ALG_TYPE_HASH 0x00000002
#define UCL_ALG_TYPE_COMPRESS 0x00000004
#define UCL_ALG_TYPE_MAC 0x00000008

/*
 * Transform masks and values.
 */
#define UCL_FLAG_MODE_MASK 0x000000ff
#define UCL_FLAG_REQ_MASK 0x000fff00
#define UCL_FLAG_RES_MASK 0xfff00000

#define UCL_FLAG_MODE_ECB 0x00000001
#define UCL_FLAG_MODE_CBC 0x00000002
#define UCL_FLAG_MODE_CFB 0x00000004
#define UCL_FLAG_MODE_CTR 0x00000008
#define UCL_FLAG_MODE_OFB 0x00000010

#define UCL_FLAG_REQ_WEAK_KEY 0x00000100

#define UCL_FLAG_RES_WEAK_KEY 0x00100000
#define UCL_FLAG_RES_BAD_KEYLEN 0x00200000
#define UCL_FLAG_RES_BAD_KEYSCHED 0x00400000
#define UCL_FLAG_RES_BAD_BLOCKLEN 0x00800000
#define UCL_FLAG_RES_BAD_FLAGS 0x01000000

/*==============================================================================
 * CIPHER
 *============================================================================*/
/** <b>Encryption Mode</b>.
 * Cipher in Encryption Mode.
 *
 * @ingroup UCL_BLOCK_CIPHER
 */
#define UCL_CIPHER_ENCRYPT 0x0
/** <b>Decryption Mode</b>.
 * Cipher in Decryption Mode.
 *
 * @ingroup UCL_BLOCK_CIPHER
 */
#define UCL_CIPHER_DECRYPT 0x1

/** <b>EEE Encryption Mode</b>.
 * Cipher in Encryption Mode for EEE mode.
 *
 * @ingroup UCL_BLOCK_CIPHER
 */
#define UCL_CIPHER_ENCRYPT_EEE 0x2
/** <b>DDD Decryption Mode</b>.
 * Cipher in Decryption Mode for EEE mode
 *
 * @ingroup UCL_BLOCK_CIPHER
 */
#define UCL_CIPHER_DECRYPT_EEE 0x3

#define UCL_CIPHER_MODE_LAST UCL_CIPHER_DECRYPT_EEE

/*==============================================================================
 * ASN1
 *============================================================================*/
#define UCL_ASN1_ID_MD5_SIZE 18
#define UCL_ASN1_ID_SHA256_SIZE 19
#define UCL_ASN1_ID_SHA224_SIZE 19
#define UCL_ASN1_ID_SHA384_SIZE 19
#define UCL_ASN1_ID_SHA512_SIZE 19
#define UCL_ASN1_ID_SHA1_SIZE 15
#define UCL_ASN1_ID_RIPEMD160_SIZE 18

/*==============================================================================
 * Big Number
 *============================================================================*/
/* <b>Sign</b>.
 * The value of an negative integer last word.
 * For complement representation of big integer.
 *
 * @ingroup UCL_DEFINES
 */
#define SIGNED 0xFFFFFFFF

/** The precision.
 * @ingroup UCL_FPA
 */
#define UCL_FPA_PRECISION 65
/** The precision for unsigned large integer.
 * @ingroup UCL_FPA
 */
#define UCL_FPA_UPRECISION (UCL_FPA_PRECISION - 1)
/** Double precision.
 * @ingroup UCL_FPA
 */
#define UCL_FPA_DB_PRECISION (2 * UCL_FPA_PRECISION)
/** Half precision.
 * @ingroup UCL_FPA
 */
#define UCL_FPA_HF_PRECISION (UCL_FPA_PRECISION / 2)
/** Maximum precision.
 * @ingroup UCL_FPA
 */
#define UCL_FPA_MAX_PRECISION (UCL_FPA_DB_PRECISION + 2)

/*
1    1         0
5432109876543210
             HHH
*/
#define UCL_NO_INPUT 0
#define UCL_HASH_INPUT 1
#define UCL_MSG_INPUT 2
#define UCL_HASH_FIPS_INPUT 3
#define UCL_NO_PRECOMP 0
#define UCL_R_PRECOMP 1
#define UCL_PRECOMP_R 2
#define UCL_NO_PRECOMP_TRICK 0
#define UCL_PRECOMP_TRICK 1
#define UCL_PRECOMP_FULL_TRICK 2
#define UCL_PRECOMP_MASK 7 // so 3 bits: 9,10,11
#define UCL_PRECOMP_TRICK_MASK 7 // so 3 bits: 12,13,14
#define UCL_INPUT_MASK 3 // so 2 bits: 7,8
#define UCL_CURVE_MASK 15 // so 4 bits: 3,4,5,6
#define UCL_HASH_MASK 7 // so 3 bits: 0,1,2
#define UCL_HASH_SHIFT 0
#define UCL_CURVE_SHIFT 3
#define UCL_INPUT_SHIFT 7
#define UCL_PRECOMP_SHIFT 9
#define UCL_PRECOMP_TRICK_SHIFT 12

#endif /*_UCL_DEFS_H_*/
