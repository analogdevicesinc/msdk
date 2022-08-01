/*============================================================================
 *
 * ucl_data_conv.h
 *
 *==========================================================================*/
/*============================================================================
 *
 * Copyright © 2009 Innova Card.
 * All Rights Reserved. Do not disclose.
 *
 * This software is the confidential and proprietary informatDATAn of
 * Innova Card ("Confidential InformatDATAn"). You shall not
 * disclose such Confidential InformatDATAn and shall use it only in
 * accordance with the terms of the license agreement you entered
 * into with Innova Card.
 *
 * Innova Card makes no representatDATAns or warranties about the suitability of
 * the software, either express or implied, including but not limited to
 * the implied warranties of merchantability, fitness for a particular purpose,
 * or non-infrigement. Innova Card shall not be liable for any damages suffered
 * by licensee as the result of using, modifying or distributing this software
 * or its derivatives.
 *
 *==========================================================================*/
/*============================================================================
 *
 * Purpose : Data Conversion/Format
 *
 *==========================================================================*/
#ifndef _UCL_DATA_CONV_H_
#define _UCL_DATA_CONV_H_

#ifdef __cplusplus
extern "C" {
#endif /* __ cplusplus  */

/** @file ucl_data_conv.h
 * @defgroup UCL_DATA_CONV Data Conversion
 * Data Conversion.
 *
 * @par Header:
 * @link ucl_data_conv.h ucl_data_conv.h @endlink
 *
 * @ingroup UCL_DATA
 */

/*============================================================================*/
/** <b>Words swap byte</b>.
 * Convet the endianess of 32-bit words array.
 *
 * @param[out] output 32-bit words data
 * @param[in]  input  32-bit words data
 * @param[in]  len    32-bit Word length of the data
 *
 * @return Error code
 *
 * @retval #UCL_INVALID_OUTPUT If output is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  If input is the pointer #NULL
 * @retval #UCL_NOP            If @p len = 0
 *
 * @ingroup UCL_DATA_CONV
 */
int ucl_data_wsb(u32* output, u32* input, u32 len);

/*============================================================================*/
/** <b>PKCS #1 OS2IP</b>.
 * Converts an octet string to a nonnegative integer.
 *
 * @param[out] it        Non negative integer
 * @param[in]  it_length Integer precision
 * @param[in]  os        Octet string
 * @param[in]  os_length Octet string length
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_OUTPUT If @p it is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  If @p os is the pointer #NULL
 * @retval #UCL_INVALID_ARG    If @p it_length < @p 4 x os_length
 *
 * @ingroup UCL_DATA_CONV
 */
int ucl_data_os2int(u32* it, u32 it_length, u8* os, u32 os_length);

/*============================================================================*/
/** <b>PKCS #1 I2OSP</b>.
 * Converts a nonnegative integer to an octet string of a specified length.
 *
 * @param[out] os        Octet string
 * @param[in]  os_length Octet string length
 * @param[in]  it        Non negative integer
 * @param[in]  it_length Integer precision
 *
 * @return Error code
 *
 * @retval #UCL_OK             If no error occurred
 * @retval #UCL_INVALID_OUTPUT If @p os is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  If @p it is the pointer #NULL
 * @retval #UCL_INVALID_ARG    If  4 x @p os_length < @p it_length
 *
 * @ingroup UCL_DATA_CONV
 */
int ucl_data_int2os(u8* os, u32 os_length, u32* it, u32 it_length);

#ifdef __cplusplus
}
#endif /* _ cplusplus  */

#endif /* _UCL_DATA_CONV_H_ */
