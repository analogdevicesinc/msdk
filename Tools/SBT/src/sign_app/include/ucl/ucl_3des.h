/*============================================================================
 *
 * ucl_3des.h
 *
* Copyright (C) 2017 Maxim Integrated Products, Inc., All rights Reserved.
* 
* This software is protected by copyright laws of the United States and
* of foreign countries. This material may also be protected by patent laws
* and technology transfer regulations of the United States and of foreign
* countries. This software is furnished under a license agreement and/or a
* nondisclosure agreement and may only be used or reproduced in accordance
* with the terms of those agreements. Dissemination of this information to
* any party or parties not specified in the license agreement and/or
* nondisclosure agreement is expressly prohibited.
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/
/*============================================================================
 *
 * Purpose : The Triple Data Encryption Standard (DES)
 *
 *==========================================================================*/
#ifndef _UCL_3DES_H_
#define _UCL_3DES_H_

#include "ucl/ucl_des.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus  */

/** @file ucl_3des.h
 * @defgroup UCL_3DES 3DES
 * The Triple DES, see FIPS 46-3 and NIST SP 800-67.
 *
 * @par Header:
 * @link ucl_3des.h ucl_3des.h @endlink
 *
 * <b>3DES Descriptor:</b>
 * @li Length of Input/Output Block: 64 bits
 * @li Length of Key: 192 (24 parity bits) (see note).
 *
 * @note A Triple des key @p K is the concatenation of three DES key: @p K0,
 * @p K1 and @p K2. K is a 128-bit Triple des key if @p K0 = @p K2.
 *
 * @n
 * For messages longer than 64 bits use an Operation mode.@n
 * @n
 *
 * @see UCL_OPERATION_MODES
 * @ingroup UCL_BLOCK_CIPHER
 */

/*============================================================================*/
/** <b> 3DES Block Size</b>.
 * The byte length of the DES core data block.
 */
#define UCL_3DES_BLOCKSIZE 8
/** <b> 3DES Key size</b>.
 * Number of DES key for a 3DES.
 *
 * @ingroup UCL_3DES
 */
#define UCL_3DES_KEYNUMBER 3
/** <b> 3DES Key length</b>.
 * The byte length of the DES key.
 */
#define UCL_3DES_KEYSIZE (8 * UCL_3DES_KEYNUMBER)

/** <b>3DES Context</b>.
 * This structure is involved in the operation modes.
 * @see UCL_OPERATION_MODES
 *
 * @ingroup UCL_3DES
 */
struct ucl_3des_ctx {
    int mode;                                                 /**< Ciphering Mode.*/
    u8 memory[8];                                             /**< Intermediate state.*/
    u32 index;                                                /**< index.*/
    u32 subKeys[UCL_3DES_KEYNUMBER * UCL_DES_NB_SUBKEYS * 2]; /**< 3DES Sub-keys.
                                                            * @see UCL_3DES_KEYNUMBER
                                                            * @see UCL_DES_NB_SUBKEYS
                                                            */
};

/** <b>3DES Context</b>.
 * @ingroup UCL_3DES
 */
typedef struct ucl_3des_ctx ucl_3des_ctx_t;

/*============================================================================*/
/** <b>3DES for Single Block</b>.
 * The complete 3DES for only one block.
 *
 * @param[out] dst  Output Block (encrypted/decrypted).
 * @param[in]  src  Input Block to encrypt/decrypt.
 * @param[in]  key  A 3DES key.
 * @param[in]  mode The 3DES mode (Encryption/Decryption).
 *
 * @return Error code
 *
 * @retval #UCL_OK             if no error occurred
 * @retval #UCL_INVALID_INPUT  if one of the input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT if one of the output is the pointer #NULL
 * @retval #UCL_INVALID_MODE   if the mode is not one of those described
 *
 * @note For more than one block use an @link UCL_OPERATION_MODES
 *    Operation Modes @endlink.
 *
 * @ingroup UCL_3DES
 */
int ucl_3des(u8* dst, u8* src, u8* key, int mode);

void tdes_eee_ks(u32* subKeys, u8* key, int mode);

#ifdef __cplusplus
}
#endif /* __cplusplus  */

#endif /* _UCL_3DES_H_ */
