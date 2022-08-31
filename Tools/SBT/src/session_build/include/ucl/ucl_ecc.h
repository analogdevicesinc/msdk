/*============================================================================
 *
 *    ucl_ecc.h
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
#ifndef UCL_ECC_H_
#define UCL_ECC_H_

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */

#ifdef __usip

/* ========================================================================== */

/** @defgroup UCL_ECC_FLAGS ECC Curve Flags.
 * Elliptic Curve Flags.
 *
 * This module regroups the flag options defining a curve:@n
 *  @li UCL_ECC_CURVE_GF_* : Field of the curve.
 *  @li UCL_ECC_CURVE_T_* : Type of the curve.
 *  @li UCL_ECC_CURVE_A_* : Type of the parameter @p a.
 *
 * The flag of a curve is a combination of those three group:
 * flag = UCL_ECC_CURVE_GF_X | UCL_ECC_CURVE_T_Y | UCL_ECC_CURVE_A_Z
 *
 * @ingroup UCL_ECC
 */

/** <b>Prime Field Curve (@f$ GF(q) @f$)</b>.
 * @ingroup UCL_ECC_FLAGS
 */
#define UCL_ECC_CURVE_GF_Q (0x00)

/** <b>Binary Curve (@f$ GF(2^m) @f$)</b>.
 * @ingroup UCL_ECC_FLAGS
 */
#define UCL_ECC_CURVE_GF_2M (0x01)

/** <b>Random Curve</b>.
 * @ingroup UCL_ECC_FLAGS
 */
#define UCL_ECC_CURVE_T_RAND (0x00)

/** <b>NIST Curve</b>.
 * @ingroup UCL_ECC_FLAGS
 */
#define UCL_ECC_CURVE_T_NIST (0x04)

/** <b>Koblitz Curve</b>.
 * @ingroup UCL_ECC_FLAGS
 */
#define UCL_ECC_CURVE_T_KOB (0x08)

/** <b>NIST Koblitz Curve</b>.
 * @ingroup UCL_ECC_FLAGS
 */
#define UCL_ECC_CURVE_T_NKOB (0x0C)

/** <b>Random @p a coefficient</b>.
 * @ingroup UCL_ECC_FLAGS
 */
#define UCL_ECC_CURVE_A_RAND (0x00)

/** <b>Special @p a coefficient</b>.
 * @ingroup UCL_ECC_FLAGS
 */
#define UCL_ECC_CURVE_A_SPEC (0x10)

/* ========================================================================== */

/** <b>Elliptic point</b>.
 * Three coordinates structure for affine & projective points.
 *
 * @ingroup UCL_ECC
 */
typedef struct {
    u32* x; /**< x coordinate (affine)     */
    u32* y; /**< y coordinate (affine)     */
    u32* z; /**< z coordinate (projective) */
} ucl_ecc_point_st;

/** <b>Extended point</b>.
 * Point of a curve with extended coordinates.
 *
 * @ingroup UCL_ECC
 */
typedef struct {
    u32* x; /**< x coordinate (affine)               */
    u32* y; /**< y coordinate (affine)               */
    u32* z; /**< z coordinate (projective)           */
    u32* az; /**< az coordinate (extended projective) */
} ucl_ecc_point_ext_st;

struct ucl_ecc_mult_s {
    int type;
    int options;
    ucl_ecc_point_st** mP;
};

typedef struct ucl_ecc_mult_s ucl_ecc_mult_t;

/** <b>Elliptic Curve</b>.
 * Elliptic curve structure.
 *
 * @ingroup UCL_ECC
 */
struct ucl_ecc_curve_s {
    u32 flags; /**< Flags defining the curve.                                */
    u32 field;
    u32 options; /**< Options                                                  */
    u32* q; /**< @p q is the prime order or the irreducible polynom.      */
    u32 s; /**< Precision of @p q                                        */
    u32 param1; /**< Parameter 1                                              */
    u32* param2; /**< Parameter 2                                              */
    int (*mult)(u32*, u32*, u32*, u32, u32); /**< Field multilication. */
    int (*inv)(u32*, u32*, u32*, u32, u32, u32*); /**< Field inversion. */
    u32* seed; /**< If the curve has been randomly generated, the seed used. */
    u32* a; /**< Coefficient a. (could be NULL)                           */
    u32* b; /**< Coefficient b.                                           */
    ucl_ecc_point_st* P; /**< The base point in affine coordinate.                     */
    u32* n; /**< The base point order                                     */
    u32 h; /**< The cofactor (the order of the curve).                   */
    int (*add_loop)(void*, void*, void*, void*, u32*); /**< Add in loop */
    int (*sub_loop)(void*, void*, void*, void*, u32*); /**< Sub in loop */
    int (*neg)(void*, void*); /**< Negation in projective coordinate.                       */
    int (*add)(void*, void*, void*, void*, u32*); /**< Add in projective coordinate. */
    int (*kdbl)(void*, void*, void*, u32, u32*); /**< Double in projective coordinate. */
    int (*kconv)(void*, void*, u32); /**< Conversion from projective to affine coordinate. */
    int (*begin)(void*, void*); /**< Prepare a point to be used on this field.                */
    int (*finish)(void*, void*); /**< To recover a point from calculus on this field.          */
    ucl_ecc_mult_t ecc_mult;
};

/** <b>Elliptic Curve</b>.
 *
 * @ingroup UCL_ECC
 */

typedef struct ucl_ecc_curve_s ucl_ecc_curve_st;

/* ========================================================================== */

/* ========================================================================== */
/** <b>Elliptic Curve Initialization</b>.
 * Init an elliptic curve.
 *
 * @param[in,out] e The curve to initialize.
 *
 * @return Error code
 *
 * @retval #UCL_OK            No error occurred
 * @retval #UCL_INVALID_INPUT The input curve contains a #NULL pointer forbidden
 * @retval #UCL_STACK_ERROR   UCL Stack error
 * @retval #UCL_ERROR         Invalid curve flags
 *
 * @warning It is mandatory to initialize a curve before to use it.
 *
 * @ingroup UCL_ECC
 */
int __API__ ucl_ecc_init(ucl_ecc_curve_st* e);
#else
/** @defgroup UCL_ECC Elliptic Curves.
 * ECC Elliptic Curves
 *
 * @ingroup UCL_PKC
 */

#endif // usip

#ifdef __cplusplus
}
#endif /* _ cplusplus  */

#endif /*UCL_ECC_H_*/
