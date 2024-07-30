/**
 * @file    mxc_assert.h
 * @brief   Assertion checks for debugging.
 */

/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
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

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32670_MXC_ASSERT_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32670_MXC_ASSERT_H_

/* **** Includes **** */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @ingroup    syscfg
 * @defgroup   mxc_assertions Assertion Checks for Debugging
 * @brief      Assertion checks for debugging.
 * @{
 */
/* **** Definitions **** */
/**
 * @note       To use debug assertions, the symbol @c MXC_ASSERT_ENABLE must be
 *             defined.
 */
///@cond
#ifdef MXC_ASSERT_ENABLE
/**
 * Macro that checks the expression for true and generates an assertion.
 * @note       To use debug assertions, the symbol @c MXC_ASSERT_ENABLE must be
 *             defined.
 */
#define MXC_ASSERT(expr)                       \
    if (!(expr)) {                             \
        mxc_assert(#expr, __FILE__, __LINE__); \
    }
/**
 * Macro that generates an assertion with the message "FAIL".
 * @note       To use debug assertions, the symbol @c MXC_ASSERT_ENABLE must be
 *             defined.
 */
#define MXC_ASSERT_FAIL() mxc_assert("FAIL", __FILE__, __LINE__);
#else
#define MXC_ASSERT(expr)
#define MXC_ASSERT_FAIL()
#endif
///@endcond
/* **** Globals **** */

/* **** Function Prototypes **** */

/**
 * @brief      Assert an error when the given expression fails during debugging.
 * @param      expr  String with the expression that failed the assertion.
 * @param      file  File containing the failed assertion.
 * @param      line  Line number for the failed assertion.
 * @note       This is defined as a weak function and can be overridden at the
 *             application layer to print the debugging information.
 *             @code
 *             printf("%s, file: %s, line %d\n", expr, file, line);
 *             @endcode
 * @note       To use debug assertions, the symbol @c MXC_ASSERT_ENABLE must be
 *             defined.
 */
void mxc_assert(const char *expr, const char *file, int line);

/**@} end of group MXC_Assertions*/

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32670_MXC_ASSERT_H_
