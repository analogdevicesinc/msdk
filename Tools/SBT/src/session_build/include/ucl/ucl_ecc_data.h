/*===========================================================================
 *
 * ucl_ecc_data.h (21 mai 07)
 *
 *==========================================================================*/
/*===========================================================================
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
/*===========================================================================
 *
 * Purpose: Data conversion for ECC
 *
 *==========================================================================*/
#ifdef __mips
#ifndef UCL_ECC_DATA_H_
#define UCL_ECC_DATA_H_

#include <ucl/ucl_ecc.h>

#ifdef __cplusplus
extern "C"
{
#endif /* _ cplusplus  */

/** Octet String Representation - Compressed form.
 * @ingroup UCL_ECC
 */
#define UCL_ECC_POINT_OS_COMP     0
/** Octet String Representation - Uncompressed form.
 * @ingroup UCL_ECC
 */
#define UCL_ECC_POINT_OS_UNCOMP   1
/** Octet String Representation - Hybrid form.
 * @ingroup UCL_ECC
 */
#define UCL_ECC_POINT_OS_HYBRID   2

/* ========================================================================== */

/* <b>Representation of the point at infinity</b>.
 * os_len = 1 @n
 * os[0] = PC = 00 @n
 * @ingroup UCL_ECC
 */
static u8 _infinity[1] = { 0x00 };

/* ========================================================================== */

/** <b>Point to Octet String Conversion</b>.
 *
 * @param[out] os     Output
 * @param[in]  os_len Output length
 * @param[in]  P      The point
 * @param[in]  s      The precision of the point (word length)
 * @param[in]  field  The field:
 *                      @li #UCL_ECC_CURVE_GF_Q
 *                      @li #UCL_ECC_CURVE_GF_2M
 * @param[in]  opt    The conversion option, one of the folowing value:
 *                      @li #UCL_ECC_POINT_OS_COMP
 *                      @li #UCL_ECC_POINT_OS_UNCOMP
 *                      @li #UCL_ECC_POINT_OS_HYBRID
 *
 * @return Error code
 *
 * @retval #UCL_OK              No error occurred
 * @retval #UCL_INVALID_ARG     Presision, length or option invalid
 * @retval #UCL_INVALID_OUTPUT  Output is pointer NULL
 * @retval #UCL_INVALID_INPUT   Input is pointer NULL
 * @retval #UCL_NOT_IMPLEMENTED Compressed and Hybrid form not implemented
 *
 * @ingroup UCL_ECC
 */
int __API__ ucl_data_point2os(u8 *os, u32 os_len, ucl_ecc_point_st *P, u32 s, u32 field, int opt);

/** <b>Octet String to Point Conversion</b>.
 *
 * @param[out] P      The point
 * @param[in]  os     Output
 * @param[in]  os_len Output length
 * @param[in]  e      The curve
 *
 * @note the first byte contains a conversion type information (option).
 *
 * @retval #UCL_OK              No error occurred
 * @retval #UCL_INVALID_ARG     Presision, length or option invalid
 * @retval #UCL_INVALID_OUTPUT  Output is pointer NULL
 * @retval #UCL_INVALID_INPUT   Input is pointer NULL
 * @retval #UCL_NOT_IMPLEMENTED Compressed and Hybrid form not implemented
 *
 * @ingroup UCL_ECC
 */
int __API__ ucl_data_os2point(ucl_ecc_point_st *P, u8 *os, u32 os_len, ucl_ecc_curve_st *e);

#ifdef __cplusplus
}
#endif /* _ cplusplus  */

#endif /*UCL_ECC_DATA_H_*/
#endif//mips
