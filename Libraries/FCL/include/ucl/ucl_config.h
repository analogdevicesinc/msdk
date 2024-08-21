/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

#ifndef LIBRARIES_FCL_INCLUDE_UCL_UCL_CONFIG_H_
#define LIBRARIES_FCL_INCLUDE_UCL_UCL_CONFIG_H_


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
#define JIBE_LINUX_CRYPTO_HW


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

/** <b>UCL RSA public exponent max size</b>.
 * 4 bytes: 32 bits.
 * @ingroup UCL_CONFIG */
#define UCL_RSA_PUBLIC_EXPONENT_MAXSIZE 4

/** <b>UCL ECC Precision</b>.
 * @ingroup UCL_CONFIG */
#define UCL_ECC_PRECISION 17

#endif // LIBRARIES_FCL_INCLUDE_UCL_UCL_CONFIG_H_
