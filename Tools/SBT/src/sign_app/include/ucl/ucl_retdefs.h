/*============================================================================
 *
 * ucl_retdefs.h
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
 * Purpose : Return values definition.
 *
 *==========================================================================*/
#ifndef UCL_RETDEFS_H_
#define UCL_RETDEFS_H_

/** @file ucl_retdefs.h
 * @defgroup UCL_RETURN Definitions of returns
 *
 * @par Header:
 *  @link ucl_retdefs.h ucl_retdefs.h @endlink
 *
 * @ingroup UCL_DEFINITIONS
 */


/** <b>Carry</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_CARRY   1

/** <b>True</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_TRUE    1
/** <b>False</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_FALSE   0

/* ========================================================================== */


/** <b>No error occured</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_OK                      0
/** <b>Generic Error</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_ERROR                   -1
/** <b>Not a failure but no operation was performed</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_NOP                     -2
/** <b>Invalid cipher specified</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_INVALID_CIPHER          -3
/** <b>Invalid hash specified</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_INVALID_HASH            -4
/** <b>Generic invalid argument</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_INVALID_ARG             -5
/** <b>Invalid argument input</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_INVALID_INPUT           -6
/** <b>Invalid argument output</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_INVALID_OUTPUT          -7
/** <b>Invalid precision for Fixed-Precision Aritmetic</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_INVALID_PRECISION       -8
/** <b>Invalid RSA public key</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_INVALID_RSAPUBKEY       -9
/** <b>Invalid RSA private key</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_INVALID_RSAPRIVKEY      -10
/** Invalid RSA CRT key.
 * @ingroup UCL_RETURN
 */
#define UCL_INVALID_RSACRTKEY       -11
/** <b>Invalid RSA CRT alternative key</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_INVALID_RSACRTALTKEY    -12
/** <b>Error during CRT recomposition</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_RSACRT_ERROR            -13
/** <b>Error, division by zero</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_DIVISION_BY_ZERO        -14
/** <b>Invalid chosen mode</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_INVALID_MODE            -15
/** <b>Large number with invalid sign</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_INVALID_SIGN            -16
/** <b>Invalid input for RSA</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_RSA_INVALID_INPUT       -17
/** <b>TRNG timeout</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_RNGTIMEOUT              -18
/** <b>RSA PKCS1-v1.5 decryption error</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_RSAPKCS1_DECRYPTERR     -19
/** <b>Overflow</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_OVERFLOW                -20
/** <b>Error in the case of the function is disabled</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_FUNCTION_DISABLED       -21
/** <b>(Big) Integer not odd</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_INTEGER_NOT_ODD         -22
/** <b>Invalid exponant of RSA key</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_RSA_INVALID_EXPONENT    -23
/** <b>UCL Stack overflow</b>.
 * Not enough memory.
 * @ingroup UCL_RETURN
 */
#define UCL_STACK_OVERFLOW         -24
/** <b>UCL Stack not init</b>.
 * @see ucl_stack_init
 * @ingroup UCL_RETURN
 */
#define UCL_STACK_NOT_INIT          -25
/** <b>Invalid UCL Stack free</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_STACK_INVALID_FREE      -26
/** <b>The UCL Stack is disabled</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_STACK_DEFAULT           -27
/** <b>Use default UCL stack</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_STACK_ERROR             -28
/** <b>General Warning</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_WARNING                 -29
/** <b>PKCS1V25 Error - Invalid Signature </b>.
 * @ingroup UCL_RETURN
 */
#define UCL_PKCS1_INVALID_SIGNATURE -30
/** <b>No Interface for USIP&reg; AES</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_NO_UAES_INTERFACE      -31
/** <b>USIP&reg; AES Corrupted</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_UAES_CORRUPTED         -32
/** <b>USIP&reg; AES Error</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_UAES_ERROR              -33
/** <b>USIP&reg; TRNG Error</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_TRNG_ERROR              -34
/** <b>No Interface for USIP&reg; TRNG</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_NO_TRNG_INTERFACE       -35
/** <b>No Interface for USIP&reg; TRNG</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_TRNG_CORRUPTED          -36
/** <b>USIP&reg; UCL Not Init</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_NOT_INIT                -37
/** <b>ECC key is invalid</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_ECC_INVALID_KEY         -38

/** <b>RNG Interface Error</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_RNG_ERROR               -39

#define UCL_RESEED_REQUIRED         -40
/** <b>Functionality not implemented</b>.
 * @ingroup UCL_RETURN
 */
#define UCL_INVALID_LENGTH         -41
/** <b>the provided length does not match the computed length</b>.
 * @ingroup UCL_RETURN
 */

#define UCL_NOT_IMPLEMENTED         -99


#endif /* UCL_RETDEFS_H_ */
