/*============================================================================
 *
 * ucl_config.h
 *
 *==========================================================================*/
/*============================================================================
 *
 * Copyright © 2009 Innova Card. All Rights Reserved. Do not disclose.
 * Copyright © 2009-14 Maxim Integrated Products Inc.
 *
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
 * Purpose :
 *
 *==========================================================================*/
#ifndef _UCL_CONFIG_H_
#define _UCL_CONFIG_H_


#ifdef __MINGW32__

#  ifdef BUILD_SHARED_LIB
#    define __API__ __declspec(dllexport)
#  else
#    define __API__ __declspec(dllimport)
#  endif

#elif defined __GCC__

#  ifdef BUILD_SHARED_LIB
#    if __GNUC__ >= 4
#      define __API__ __attribute__ ((visibility ("default")))
#    else
#      define __API__
#    endif
#  else
#    define __API__
#  endif

#else
#  define __API__
#endif

/* JIBE_LINUX_CRYPTO_HW */
/* #define JIBE_LINUX_CRYPTO_HW */


/*#if defined (__jibe) && !defined (__linux)
#ifndef JIBE_COBRA
#define JIBE_COBRA
#endif
#endif*/

#if defined (__jibe) && defined (__linux) && defined(JIBE_LINUX_CRYPTO_HW)
#warning JIBE target will use the userland API to the kernel crypto drivers
#define JIBE_LINUX_HW
#endif

/** <b>UCL Stack default size</b>.
 * 8 Ko.
 * @ingroup UCL_CONFIG */
#define UCL_STACK_SIZE (8*1024)

/** <b>UCL RSA key max size</b>.
 * 512 bytes: 4096 bits.
 * @ingroup UCL_CONFIG
 */
//1024 is ok on mingw for rsa encrypt up to 3072
//but seems to be too large for jibe stack
#define UCL_RSA_KEY_MAXSIZE 512
#if defined(__lighthouse)||defined(__yumen)||defined(__max32550)||defined(__max32555)||defined(__max32552)||defined(__max32560)||defined(__max32510)||defined(__max32558)||defined(__max32590)||defined(__jibe)
#define UCL_RSA_KEY_MAA_MAXSIZE 256
#elif defined(__max32621)||defined(__max32631)||defined(__max32600)||defined(__wasp)
#define UCL_RSA_KEY_MAA_MAXSIZE 128
#elif defined(__max32652)
#define UCL_RSA_KEY_MAA_MAXSIZE 256
#else
//#if defined(__max32620)||defined(__max32630)
#define UCL_RSA_KEY_MAA_MAXSIZE 0
#endif


/** <b>UCL RSA public exponent max size</b>.
 * 4 bytes: 32 bits.
 * @ingroup UCL_CONFIG */
#define UCL_RSA_PUBLIC_EXPONENT_MAXSIZE 4

/** <b>UCL ECC Precision</b>.
 * @ingroup UCL_CONFIG */
#define UCL_ECC_PRECISION 17

#endif /*_UCL_CONFIG_H_*/
