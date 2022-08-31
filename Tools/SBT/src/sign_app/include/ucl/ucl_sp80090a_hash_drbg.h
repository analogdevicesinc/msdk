/*******************************************************************************
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
 * Purpose : SP800-90A HASH DRBG
 *
 *==========================================================================*/

#ifndef UCL_SP80090A_HASH_DRBG_H
#define UCL_SP80090A_HASH_DRBG_H

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */

#define UCL_SP800_90A_SHA256_SEEDLENGTH 55
#define UCL_SP800_90A_SHA512_SEEDLENGTH 111

/** @file ucl_sp80090a_hash_drbg.h
 * @defgroup UCL_SP80090A SP800-90A interface
 * SP800-90A Hash DRBG-compliant source for random numbers
 *
 * @par Header:
 * @link ucl_sp80090a_hash_drbg.h ucl_sp80090a_hash_drbg.h @endlink
 *
 *
 * @ingroup UCL_RAND
 */

/** <b>SP800-90A Hash DRBG Internal State structure</b>.
 * The implementation follow the specification.
 * Therefore, the functions names are similar to the specification routines names.
 * @ingroup UCL_SP80090A
 */

struct ucl_sp80090a_internal_state {
    /** the value V, updated during each call */
    u8 v[UCL_SP800_90A_SHA512_SEEDLENGTH];
    /** the constant C, depending on the seed */
    u8 c[UCL_SP800_90A_SHA512_SEEDLENGTH];
    /** the reseed counter, indicating how many requests can be made since the instantiation or a
     * reseeding */
    u64 reseed_counter;
    /** the reseed limit number, requiring a reseeding if the reseed counter reaches this value */
    u64 reseed_limit;
};

typedef struct ucl_sp80090a_internal_state ucl_sp80090a_internal_state_t;

/** <b>SP800690A hash DRBG hash_df function  based on SHA256</b>.
 * this function is a derivation function using the SHA256 hash function
 * the input parameters are given as three distinct parameters instead of one single input_string
 * as specified, to ease giving various kinds of parameters, with various lengths
 * @param[out]  requested_bits: the pointer to the requested bits
 * @param[in]  entropy_input: the pointer to the string of bits obtained from the source of entropy
 * input
 * @param[in]  entropy_input_len:   The length in bytes of the entropy_input string
 * @param[in] nonce: the pointer the string of the nonce string
 * @param[in]   nonce_len: the length in bytes of the nonce input string
 * @param[in] personalization_string: the pointer the string of personalization string
 * @param[in]   personalization_string_len: the length in bytes of the personalization string
 * @param[in] nb_of_bits_to_return: the requested bits number, expressed in bits, being a multiple
 * of 8
 * @return Error code
 *
 * @retval #UCL_OK
 * @retval #UCL_INVALID_OUTPUT  if the output is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if one of the input is the pointer #NULL
 * @ingroup UCL_SP80090A
 */
int ucl_sp80090a_hash_df_sha256(u8* requested_bits, u8* entropy_input, u32 entropy_input_len,
    u8* nonce, u32 nonce_len, u8* personalization_string, u32 personalization_string_len,
    u32 nb_of_bits_to_return);

/** <b>SP800690A hash DRBG internal state reseeding  based on SHA256</b>.
 * this function updates the internal state, using the provided entropy input and the additional
 * input
 *
 * @param[in/out]  internal_state:   The pointer to the internal state structure
 * @param[in]  entropy_input: the pointer to the string of bits obtained from the source of entropy
 * input
 * @param[in]  entropy_input_len:   The length in bytes of the entropy_input string
 * @param[in] additional_input: the pointer the string of additional input string received from the
 * consuming application
 * @param[in]   additional_input_len: the length in bytes of the additional input string
 *
 * @return Error code
 *
 * @retval #UCL_OK
 * @retval #UCL_ERROR if an error has occured in the hash df operation
 * @retval #UCL_INVALID_OUTPUT  if the output is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if one of the input is the pointer #NULL
 * @ingroup UCL_SP80090A
 */

int ucl_sp80090a_reseeding_sha256(ucl_sp80090a_internal_state_t* internal_state, u8* entropy_input,
    u32 entropy_input_len, u8* additional_input, u32 additional_input_len);

/** <b>SP800690A hash DRBG internal state instantiation  based on SHA256</b>.
 * this function instantiates the internal state, using the provided entropy input, the nonce and
 * the personalization string
 *
 * @param[out]  internal_state:   The pointer to the internal state structure
 * @param[in]  entropy_input: the pointer to the string of bits obtained from the source of entropy
 * input
 * @param[in]  entropy_input_len:   The length in bytes of the entropy_input string
 * @param[in] nonce: the pointer the string of the nonce string
 * @param[in]   nonce_len: the length in bytes of the nonce input string
 * @param[in] personalization_string: the pointer the string of personalization string
 * @param[in]   personalization_string_len: the length in bytes of the personalization string
 * @param[in] additional_input: the pointer the string of additional input string received from the
 * consuming applcation
 * @param[in]   additional_input_len: the length in bytes of the additional input string
 *
 * @return Error code
 *
 * @retval #UCL_OK
 * @retval #UCL_ERROR if an error has occured in the hash df operation
 * @retval #UCL_INVALID_OUTPUT  if the output is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if one of the input is the pointer #NULL
 * @ingroup UCL_SP80090A
 */
int ucl_sp80090a_instantiate_sha256(ucl_sp80090a_internal_state_t* internal_state,
    u8* entropy_input, u32 entropy_input_len, u8* nonce, u32 nonce_len, u8* personalization_string,
    u32 personalization_string_len);

/** <b>SP800690A hashgen function  based on SHA256</b>.
 * this function returns generated bits to the generate function
 * defined p43
 * @param[out]  requested_bits: the pointer to the requested bits
 * @param[in] v: the pointer to the V value in the internal state
 * @param[in] nb_of_bits_to_return: the requested bits number, expressed in bits, being a multiple
 * of 8
 *
 * @return Error code
 *
 * @retval #UCL_OK
 * @retval #UCL_ERROR if an error has occured in the hash df operation
 * @retval #UCL_INVALID_OUTPUT  if the output is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if one of the input is the pointer #NULL
 * @ingroup UCL_SP80090A
 */
int hashgen_sha256(u8* requested_bits, int nb_of_bits_to_return, u8* v);

/** <b>SP800690A bits generation function based on SHA256 </b>.
 * this function returns requested pseudorandom bits
 * defined p42, section 10.1.1.4
 * @param[out]  requested_bits: the pointer to the requested bits
 * @param[in] nb_of_bits_to_return: the requested bits number, expressed in bits, being a multiple
 * of 8
 * @param[in] internal_state: the pointer to the internal state
 * @param[in] additional_input: the pointer the string of additional input string received from the
 * consuming applcation
 * @param[in]   additional_input_len: the length in bytes of the additional input string
 *
 * @return Error code
 *
 * @retval #UCL_OK
 * @retval #UCL_RESEED_REQUIRED if a reseeding is required; no bits are outputted
 * @retval #UCL_INVALID_OUTPUT  if the output is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if one of the input is the pointer #NULL
 * @ingroup UCL_SP80090A
 */
int ucl_sp80090a_hash_drbg_sha256_generate(u8* requested_bits, int nb_of_bits_to_return,
    ucl_sp80090a_internal_state_t* internal_state, u8* additional_input, int additional_input_len);

int ucl_sp80090a_hash_df_sha512(u8* requested_bits, u8* entropy_input, u32 entropy_input_len,
    u8* nonce, u32 nonce_len, u8* personalization_string, u32 personalization_string_len,
    u32 nb_of_bits_to_return);
int ucl_sp80090a_reseeding_sha512(ucl_sp80090a_internal_state_t* internal_state, u8* entropy_input,
    u32 entropy_input_len, u8* additional_input, u32 additional_input_len);
int ucl_sp80090a_instantiate_sha512(ucl_sp80090a_internal_state_t* internal_state,
    u8* entropy_input, u32 entropy_input_len, u8* nonce, u32 nonce_len, u8* personalization_string,
    u32 personalization_string_len);

u32 __API__ ceiling_div(u32 a, u32 b);

#ifdef __cplusplus
}
#endif /* _ cplusplus  */
#endif // UCL_SP80090A_HASH_DRBG_H
