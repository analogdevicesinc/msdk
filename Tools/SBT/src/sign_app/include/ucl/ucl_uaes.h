/*============================================================================
 *
 * ucl_uaes.h
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
 * Purpose : USIP&reg; AES
 *
 *==========================================================================*/
#ifndef _UCL_UAES_H_
#define _UCL_UAES_H_

#ifdef _cplusplus
extern "C" {
#endif /* _ cplusplus  */


/** @file ucl_uaes.h
 * @defgroup UCL_UAES USIP AES Interface
 * Interface for Hardware AES.
 *
 * @par Header:
 * @link ucl_uaes.h ucl_uaes.h @endlink
 *
 * @n
 * <b>USIP&reg; AES Descriptor:</b> @n
 * @li Length of Input/Output Block: 128 bits
 * @li Length of key: 128 bits
 *
 * For messages longer than 128 bits use an Operation Modes.
 *
 * @see UCL_OPERATION_MODES
 * @ingroup UCL_BLOCK_CIPHER
 */


/** <b>UAES Key Byte Size</b>.
 *  @ingroup UCL_UAES */
#define UCL_UAES_KEY_SIZE 16
/** <b>UAES Block Byte Size</b>.
 *  @ingroup UCL_UAES */
#define UCL_UAES_BLOCK_SIZE 16


/** <b>USIP&reg; AES Interface definition </b>.
 *  @ingroup UCL_UAES */
struct usip_aes
{
    int(*setkey)(const u8 *key);            /**< Set key */
    int(*dkeygen)(void);                    /**< Generate decryption key */
    int(*decrypt)(u8 *dataOut, u8 *dataIn); /**< Decryption */
    int(*encrypt)(u8 *dataOut, u8 *dataIn); /**< Encryption */
};


/** <b>USIP&reg; AES Interface typedef</b>.
 */
typedef struct usip_aes usip_aes_t;

/** <b>USIP&reg; AES Context</b>.
 * This structure is involved in the operation rmac.
 *
 * @ingroup UCL_UAES */
struct ucl_uaes_ctx
{
    /** Ciphering mode.*/
    int mode;
    /** Intermediate state.*/
    u8 memory[16];
    /** USIP&reg; AES Key.*/
    u8 key[16];
};

/** <b>USIP&reg; AES Context</b>.
 * @ingroup UCL_UAES
 */
typedef struct ucl_uaes_ctx ucl_uaes_ctx_t;


/** <b>USIP&reg; AES Set Key Option</b>.
 * The function #ucl_uaes_setkey set an encryption key.
 *
 * @ingroup UCL_UAES
 */
#define UCL_UAES_SET_KEY    0
/** <b>USIP&reg; AES Set Key Option</b>.
 * The function #ucl_uaes_setkey set a key, and precalculates the decryption
 * key.
 *
 * @ingroup UCL_UAES
 */
#define UCL_UAES_SET_KEY_AND_DKEY 1
/** <b>USIP&reg; AES Set Key Option</b>.
 * The function #ucl_uaes_setkey precalculates the decryption key with
 * the current key.
 *
 * @ingroup UCL_UAES
 */
#define UCL_UAES_SET_DKEY   2
/** <b>USIP&reg; AES Set Key Option</b>.
 * The function #ucl_uaes_setkey precalculates the decryption key with
 * the current key and set a new key.
 *
 * @ingroup UCL_UAES
 */
#define UCL_UAES_SET_DKEY_AND_KEY 3


/*============================================================================*/
/** <b>USIP&reg; AES Set Key</b>
 * The function configure the USIP&reg; AES.
 *
 * @param[in] key    Pointer to an 128-bit AES key.
 * @param[in] option A configuration option between:
 *                      @li #UCL_UAES_SET_KEY
 *                      @li #UCL_UAES_SET_KEY_AND_DKEY
 *                      @li #UCL_UAES_SET_DKEY
 *                      @li #UCL_UAES_SET_DKEY_AND_KEY
 *
 * @return Error code
 *
 * @retval #UCL_OK                If no error occurred
 * @retval #UCL_INVALID_INPUT     If @p key is #NULL and @p option is not
 * @retval #UCL_INVALID_ARG       If @p option is not one of the possbile values
 * @retval #UCL_NO_UAES_INTERFACE If there is not interface to the USIP&reg; AES
 *
 * @ingroup UCL_UAES
 */

int ucl_uaes_setkey(const u8 *key, int option);


/*============================================================================*/
/** <b>USIP&reg; AES Decryption</b>
 * The function decrypts an AES enciphered message.
 *
 * @param[out] plaintext  A pointer to the 128 bit plain text.
 * @param[in]  ciphertext A pointer to the 128 bit ciphered text.
 *
 * @return Error code
 *
 * @retval #UCL_OK                If no error occurred
 * @retval #UCL_INVALID_INPUT     If @p dataIn is #NULL
 * @retval #UCL_INVALID_OUTPUT    If @p dataOut is #NULL
 * @retval #UCL_NO_UAES_INTERFACE If there is not interface to the USIP&reg; AES
 *
 * @ingroup UCL_UAES
 */
int ucl_uaes_decrypt(u8 *plaintext, u8 *ciphertext);


/*============================================================================*/
/** <b>USIP&reg; AES Encryption</b>
 * The function encrypts a message with the AES algorithm.
 *
 * @param[out] ciphertext A pointer to the 128 bit ciphered text.
 * @param[in]  plaintext  A pointer to the 128 bit plain text.
 *
 * @return Error code
 *
 * @retval #UCL_OK                If no error occurred
 * @retval #UCL_INVALID_INPUT     If @p dataIn is #NULL
 * @retval #UCL_INVALID_OUTPUT    If @p dataOut is #NULL
 * @retval #UCL_NO_UAES_INTERFACE If there is not interface to the USIP&reg; AES
 *
 * @ingroup UCL_UAES
 */
int ucl_uaes_encrypt(u8 *ciphertext, u8 *plaintext);


#ifdef _cplusplus
}
#endif /* _ cplusplus  */

#endif /* _UCL_UAES_H_ */
