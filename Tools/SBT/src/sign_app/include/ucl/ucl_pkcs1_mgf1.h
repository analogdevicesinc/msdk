/*============================================================================
 *
 * ucl_pkcs1_mgf1.h
 *
 *==========================================================================*/
/*******************************************************************************
 * Copyright (C) 2018 Maxim Integrated Products, Inc., All rights Reserved.
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
 *******************************************************************************/
/*============================================================================
 *
 * Purpose : PKCS#1 V2.1 MGF1 hash function management
 *
 *==========================================================================*/
#ifndef _UCL_PKCS1_MGF1_H_
#define _UCL_PKCS1_MGF1_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @file ucl_pkcs1_mgf1.h
 * @defgroup UCL_PKCS1V21_MGF1 PKCS1v21-MGF1
 * MGF1 hash functions management
 *
 * @par Header:
 * @link ucl_pkcs1_mgf1.h ucl_pkcs1_mgf1.h @endlink
 *
 * @ingroup UCL_PKCS1V21
 */

/*============================================================================*/
/** <b>PKCS#1 MGF1 hash function selection</b>.
 * this function allows to select the PKCS#1 MGF1 hash function, whatever the hash
 * function used in the PKCS#1 scheme
 * This is not a GOOD PRACTICE !! but Android test suite allows that
 * This so selected hash function will be then be used for RSA PKCS#1 ES-OAEP and SSA-PSS
 * until either another one is selected, or the selection is cleared
 * @param[in]  hsh_identifier: the hsh function identifier, as defined in hash function include
 * files
 * @return Error code
 *
 * @retval #UCL_OK    if the selection is ok
 * @retval #UCL_INVALID_INPUT if the selection is not valide
 *
 * @see UCL_RSA
 *
 * @ingroup UCL_PKCS1V21_MGF1
 */

int ucl_set_mgf1_hash_function(int hash_identifier);

/*============================================================================*/
/** <b>PKCS#1 MGF1 hash function selection clearance</b>.
 * this function allows to unselect the PKCS#1 MGF1 hash function, i.e. once done, the hash
 * function used in the MGF1 will be the same than in the PKCS#1 scheme
 * This is the GOOD PRACTICE !!
 * It impacts RSA PKCS#1 ES-OAEP and SSA-PSS
 *
 *
 * @see UCL_RSA
 *
 * @ingroup UCL_PKCS1V21_MGF1
 */

void ucl_clear_mgf1_hash_function(void);

/*============================================================================*/
/** <b>PKCS#1 MGF1 hash function retrieval </b>.
 * this function returns the PKCS#1 MGF1 hash function, whatever the hash
 * function used in the PKCS#1 scheme
 * if the MGF1 hash function has been set using the ucl_set_mgf1_hash_function
 * it returns the value of this hash function
 * otherwise
 * This so selected hash function will be then be used for RSA PKCS#1 ES-OAEP and SSA-PSS
 * until either another one is selected, or the selection is cleared
 * @param[in]  hsh_identifier: the hsh function identifier, as defined in hash function include
 * files
 * @return Error code
 *
 * @retval hash function identifier   if the selection with ucl_set_mgf1_hash_function has been
 * performed
 * @retval #UCL_UNDEFINED_HASH if not (either no selection or selection clearance performed)
 *
 * @see UCL_RSA
 *
 * @ingroup UCL_PKCS1V21_MGF1
 */

int ucl_get_mgf1_hash_function(void);

int ucl_pkcs1_mgf1_hash(
    u8* mask, u32 mask_length, u8* mgf_seed, u32 mgf_seed_length, int hash_identifier);
#endif // PKCS1_MGF1_H_
