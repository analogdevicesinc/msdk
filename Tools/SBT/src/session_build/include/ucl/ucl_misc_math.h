/*============================================================================
 *
 * ucl_misc_math.h
 *
 *==========================================================================*/
/*============================================================================
 *
 * Copyright © 2009 Innova Card. All rights reserved. Do not disclose.
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
 * Purpose : Miscenaleous Function for Mathematic.
 *
 *==========================================================================*/
#ifndef _UCL_MISC_MATH_H_
#define _UCL_MISC_MATH_H_

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */

/** @file ucl_misc_math.h
 * @defgroup UCL_MISC_MATH Mathematical functions
 * Mathematical miscellaneous functions.
 *
 * @par Header:
 * @link ucl_misc_math.h ucl_misc_math.h @endlink
 *
 * Those particular functions use special MIPS� 4KSd instruction.
 *
 * @ingroup UCL_MISC
 */


/*============================================================================*/
/** <b>Euclidian Division of a 64bits word by a 32bits word </b>.
 *
 * Let @a a and @a q two 64bits unsigned words such that @n
 * @f$ a = a1 \times 2^{32} + a0\ and\ q = q1 \times 2^{32} + q0 @f$ @n
 * Let @p b and @a r two 32-bit unsigned words @n
 * @n
 * If preconditions are satisfied then @n
 * @f$ a = q \times b + r @f$
 *
 * @param[out] ptQ1 Pointer to the higher word of the quotient @a q
 * @param[out] ptQ0 Pointer to the lower word of the quotient @a q
 * @param[out] ptR  Pointer to the reminder @a r
 * @param[in]  a1   The higher word of @a a
 * @param[in]  a0   The lower word of @a a
 * @param[in]  b    The divisor
 *
 * @return Error code
 *
 * @retval #UCL_OK                If no error occurred
 * @retval #UCL_INVALID_OUTPUT    If an output is the pointer #NULL
 * @retval #UCL_INVALID_INPUT     If an intput is the pointer #NULL
 * @retval #UCL_INVALID_PRECISION If invalid precision
 *
 * @ingroup UCL_MISC_MATH
 */
int ucl_misc_div64by32(u32 *ptQ1, u32 *ptQ0, u32 *ptR, u32 a1,
                       u32 a0, u32 b);


/*============================================================================*/
/** <b>Increment a 64-bit word by a 32-bit word </b>.
 *
 * @param[in] b A 64-bit word
 * @param[in] a A 32-bit word
 *
 * @return The carry.
 *
 * @retval #UCL_INVALID_INPUT if an intput is the pointer #NULL
 *
 * @ingroup UCL_MISC_MATH
 */
int ucl_misc_incW64byW32(u32 *b, u32 a);


#ifdef __cplusplus
}
#endif /* _ cplusplus  */

#endif /* _UCL_MISC_MATH_H_ */
