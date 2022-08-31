/*============================================================================
 *
 *  ucl_ecc_mult.h
 *
 *==========================================================================*/
/*============================================================================
 *
 * Copyright © 2009 Innova Card. All Rights Reserved. Do not disclose.
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
#ifndef UCL_ECC_MULT_H_
#define UCL_ECC_MULT_H_
#ifdef _usip
#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */

/** @defgroup UCL_ECC_MULT ECC Point Multiplication.
 * ECC Point Multiplication.
 *
 * @ingroup UCL_ECC
 */

/** <b>Multiplication without precomputation</b>.
 * @ingroup UCL_ECC_MULT
 */
#define UCL_ECC_OPTION_NOPRECOMPUT 0x0

/** <b>Multiplication with precomputation</b>.
 * Option only available for @f$ GF(2^m) @f$.
 * @ingroup UCL_ECC_MULT
 */
#define UCL_ECC_OPTION_PRECOMPUT 0x1

/** <b>Multiplication using montgomery</b>.
 * Option only available for @f$ GF(2^m) @f$.
 * @ingroup UCL_ECC_MULT
 */
#define UCL_ECC_OPTION_MONTGOMERY 0x2

/* ========================================================================== */
/** <b>Point multiplication</b>.
 * Compute @f$ R = k.P @f$.
 *
 * The parameter @p opt defines the used algorithm:
 *  @li #UCL_ECC_OPTION_NOPRECOMPUT: With NAF method
 *  @li #UCL_ECC_OPTION_PRECOMPUT:   With precomputation (Only for @f$ GF(2^m) @f$)
 *  @li #UCL_ECC_OPTION_MONTGOMERY:  With montgomery (Only for @f$ GF(2^m) @f$)
 *
 * @param[out]  R   The result R (affine)
 * @param[in]   P   The base point (affine)
 * @param[in]   k   The secret key
 * @param[in]   sk  The size of the secret key k
 * @param[in]   e   The elliptic curve
 * @param[in]   opt Computation option
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_OUTPUT @p R is not in affine coordinates
 * @retval #UCL_INVALID_INPUT  @p P is not in affine coordinates or @p opt is invalid
 * @retval #UCL_STACK_ERROR    UCL Stack error
 * @retval #UCL_INVALID_ARG    Invalid options @p opt
 *
 * @ingroup UCL_ECC_MULT
 */
int __API__ ucl_ecc_mult_unknown(
    ucl_ecc_point_st* R, ucl_ecc_point_st* P, ucl_ecc_curve_st* e, u32* k, u32 sk, int opt);

/* ========================================================================== */
/** <b>Fixed-Point Multiplication Precomputation</b>.
 * Allocate and organize memory, and do precomputation for fixed point
 * comb 2-base algorithm.
 *
 * @param[out]  mP  A pointer to the precomputation
 * @param[in]   w   The size of precomputations
 * @param[in]   P   The base point (affine)
 * @param[in]   e   The elliptic curve
 *
 * @return Error code
 *
 * @retval #UCL_OK          No error occurred
 * @retval #UCL_STACK_ERROR UCL Stack error
 *
 * @see #ucl_ecc_mult_fixed
 *
 * @ingroup UCL_ECC_MULT
 */
int __API__ ucl_ecc_mult_fixed_precomp(
    ucl_ecc_point_st*** mP, u32 w, ucl_ecc_point_st* P, ucl_ecc_curve_st* e);

/* ========================================================================== */
/** <b>Fixed-Point Multiplication</b>.
 * Compute @f$ R = k.P @f$ with fixed-point multiplication using
 * comb 2-bases method.
 *
 * @param[out]  R   The result R
 * @param[in]   mP  The precomputations (Computed by #ucl_ecc_mult_fixed_precomp)
 * @param[in]   w   The size of precomputations
 * @param[in]   k   The secret key
 * @param[in]   sk  The size of the secret key
 * @param[in]   e   The elliptic curve
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_OUTPUT @p R point is not in affine coordinates
 * @retval #UCL_INVALID_INPUT  @p mP is a #NULL pointer or w is lower than 2
 * @retval #UCL_STACK_ERROR    UCL Stack error
 *
 * @see #ucl_ecc_mult_fixed_precomp
 *
 * @ingroup UCL_ECC_MULT
 */
int __API__ ucl_ecc_mult_fixed(
    ucl_ecc_point_st* R, ucl_ecc_point_st** mP, u32 w, u32* k, u32 sk, ucl_ecc_curve_st* e);

#ifdef __cplusplus
}
#endif /* _ cplusplus  */
#endif // usip
#endif /* UCL_ECC_MULT_H_ */
