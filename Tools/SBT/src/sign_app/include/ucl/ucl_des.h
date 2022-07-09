/*============================================================================
 *
 * ucl_des.h
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
 * Purpose : The Data Encryption Standard (DES)
 *
 *==========================================================================*/
#ifndef _UCL_DES_H_
#define _UCL_DES_H_

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */


/** @file ucl_des.h
 * @defgroup UCL_DES DES
 * The Data Encryption Standard (DES), see @ref FIPS46_3 "FIPS PUB 46.3 NIST".
 *
 * @par Header:
 * @link ucl_des.h ucl_des.h @endlink
 *
 * DES is a Feistel cipher which processes plaintext blocks of 64 bits,
 * producing 64-bit ciphertext blocks. The effective size of the secret key
 * is 56 bits; more precisely, the input key is specified as a 64-bit key,
 * 8 bits of which (bits 8; 16; ... ; 64) may be used as parity bits. @n
 * To encrypt/decrypt a block, process the key schedule and then, the main part
 * of the DES. @n
 * Many implementation of the DES can be provided, with different levels
 * of security. @n
 * @n
 * <b>DES Descriptor:</b> @n
 * @li Length of Input/Output Block: 64 bits
 * @li Length of key: 64 bits (8 parity bits)
 *
 * For messages longer than 64 bits use an Operation Modes.
 *
 * @see UCL_OPERATION_MODES
 * @ingroup UCL_BLOCK_CIPHER
 */


/*============================================================================*/
/** <b>DES Block Size</b>.
 * The byte length of the DES core data block.
 *
 * @ingroup UCL_DES
 */
#define UCL_DES_BLOCKSIZE 8
/** <b>DES Key Size</b>.
 * The byte length of the DES key.
 *
 * @ingroup UCL_DES
 */
#define UCL_DES_KEYSIZE 8

#ifdef UCL_DES_RANDOMIZED_KEY
/** <b>Number of subkeys invovled in DES</b>.
 * The number of subkeys depend on the implementation.
 *
 * @ingroup UCL_DES
 */
#define UCL_DES_NB_SUBKEYS 32
#else
/** <b>Number of subkeys invovled in DES</b>.
* The number of subkeys depend on the implementation.
*
* @ingroup UCL_DES
*/
#define UCL_DES_NB_SUBKEYS 16
#endif

/*============================================================================*/
/** <b>DES Context</b>.
 * This structure is involved in the operation modes.
 * @ingroup UCL_DES
 */
struct ucl_des_ctx
{

    int mode;     /**< Ciphering Mode.     */
    u8 memory[8]; /**< Intermediate state. */
    u32 index;    /**< Index.              */
    u32 subKeys[UCL_DES_NB_SUBKEYS*2];
    /**< DES Sub-keys.
     * @see UCL_DES_NB_SUBKEYS
     */
};

/** <b>DES Context typedef</b>.
 * @ingroup UCL_DES
 */
typedef struct ucl_des_ctx ucl_des_ctx_t;


/*============================================================================*/
/** <b>DES for Single Block</b>.
 * This function apply the DES to a single block. For message longer than 64 bits
 * use an @link UCL_OPERATION_MODES Operation Modes @endlink.
 *
 * @param[out] dst  The decrypted/encrypted block
 * @param[in]  src  A block to process
 * @param[in]  key  A 64-bit DES key
 * @param[in]  mode The mode (Encryption/Decryption) :
 *                      @li #UCL_CIPHER_ENCRYPT
 *                      @li #UCL_CIPHER_DECRYPT
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_INPUT  One of the input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT One of the output is the pointer #NULL
 * @retval #UCL_INVALID_MODE   The mode is not one of those described
 *
 * @note For more than one block use an @link UCL_OPERATION_MODES
 *       Operation Modes @endlink.
 *
 * @ingroup UCL_DES
 */
int ucl_des(u8 *dst, u8 *src, u8 *key, int mode);

int ucl_des_dpa2(u8 *dst, u8 *src, u8 *key, int mode);

/** <b>3DES-CBC Xor</b>.
 * Xor 2 buffers and store the result in a third.
 *
 * @param[out] buffOut Pointer to the output buffer (result)
 * @param[in] buffIn1 Pointer to the 1st buffer
 * @param[in] buffIn2 Pointer to the 2nd buffer
 * @param[in] length number of bytes to process
 *
 * @ingroup UCL_CBC_3DES
 */
void ucl_do_xor(u8* buffOut, u8* buffIn1, u8* buffIn2, int length);



#ifdef __cplusplus
}
#endif /* _ cplusplus  */

#endif /* _UCL_DES_H_ */
