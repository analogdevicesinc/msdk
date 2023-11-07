/**
 * @file
 * @brief Provides logging facilaties for the NFC Contactless PCD L1 Stack
 */
/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc., All Rights Reserved.
 * (now owned by Analog Devices, Inc.)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
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
 *
 ******************************************************************************
 *
 * Copyright 2023 Analog Devices, Inc.
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

#ifndef EXAMPLES_MAX32572_MAX32572_DEMO_BAREMETAL_INCLUDE_NFC_EMV_L1_STACK_LOGGING_H_
#define EXAMPLES_MAX32572_MAX32572_DEMO_BAREMETAL_INCLUDE_NFC_EMV_L1_STACK_LOGGING_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @defgroup NFC_PCD_EMV_LVL1_STACK_LOGGING Logging
 *
 * @ingroup NFC_PCD_EMV_LVL1_STACK
 *
 * Provides routines to facilitate required DTE logging, and for Stack/PICC interaction
 * debugging.
 *
 * @{
 */

/** @defgroup NFC_PCD_EMV_LVL1_STACK_LOGGING_LVLS Logging Levels
 *
 * Available levels for EMV Contactless L1 Stack logging.
 *
 * @ref g_logging_level can be changed on the fly to any of these values for debugging
 *
 * @note Use of @ref DBG_LVL_FDB will delay some operations long enough to fail certain Level 1 Digital test cases
 *
 * @{
 */
#define DBG_LVL_NON 0 /**< No Stack logging or debugging, quite mode used for applications */
#define DBG_LVL_LOG 1 /**< Basic logging, this level is used to comply with DTE requirements */
#define DBG_LVL_ERR 2 /**< Same as @ref DBG_LVL_LOG plus various errors encountered */
#define DBG_LVL_WRN 3 /**< Same as @ref DBG_LVL_ERR plus various warning encountered */
#define DBG_LVL_INF \
    4 /**< Same as @ref DBG_LVL_WRN plus useful details about the flow in the stack */
/**< including uid level, uid, sfgi, and other interface details */
#define DBG_LVL_DBG \
    5 /**< Same as @ref DBG_LVL_INF plus TX buffer contents, and RF Driver response status */
#define DBG_LVL_FDB 6 /**< Same as @ref DBG_LVL_DBG plus RX buffer contents */

/** @} */ /* @defgroup NFC_PCD_EMV_LVL1_STACK_LOGGING_LVLS */

/** @defgroup NFC_PCD_EMV_LVL1_STACK_LOGGING_MACROS Logging Macros
 *
 * Convenience functions to assist with stack logging, these can be used similar to printf
 *
 * @note Use of @ref full_debug will delay some operations long enough to fail certain Level 1 Digital test cases
 *
 * @{
 */
#define logging(x...) \
    do_log(DBG_LVL_LOG, x) /**< Basic logging, this level is used to comply with DTE requirements */
#define error(x...) \
    do_log(DBG_LVL_ERR, x) /**< Same as @ref DBG_LVL_LOG plus various errors encountered */
#define warning(x...) \
    do_log(DBG_LVL_WRN, x) /**< Same as @ref DBG_LVL_ERR plus various warning encountered */
#define info(x...)      \
    do_log(DBG_LVL_INF, \
           x) /**< Same as @ref DBG_LVL_WRN plus useful details about the flow in the */
/**< stack including uid level, uid, sfgi, and other interface details */
#define debug(x...)  \
    do_log(          \
        DBG_LVL_DBG, \
        x) /**< Same as @ref DBG_LVL_INF plus TX buffer contents, and RF Driver response status */
#define full_debug(x...) \
    do_log(DBG_LVL_FDB, x) /**< Same as @ref DBG_LVL_DBG plus RX buffer contents */
/** @} */ /* @defgroup NFC_PCD_EMV_LVL1_STACK_LOGGING_MACROS */

/**
 * @brief Controls the logging level for the L1 Stack
 *
 * Stack Logging Level
 *
 * Defaults to output logging level messages only
 */
extern int32_t g_logging_level;

/**
 * @brief Conditionally print some logging information
 *
 * This function is called by @ref NFC_PCD_EMV_LVL1_STACK_LOGGING_MACROS to implement multilevel debugging messages
 *
 * @param[in]   req_level Level for this logging message, use one of @ref NFC_PCD_EMV_LVL1_STACK_LOGGING_LVLS
 * @param[in]   ... printf style formating string and variables to print etc.
 */
void do_log(int32_t req_level, ...);

/** @} */ /* @defgroup NFC_PCD_EMV_LVL1_STACK_LOGGING */

#ifdef __cplusplus
}
#endif

#endif // EXAMPLES_MAX32572_MAX32572_DEMO_BAREMETAL_INCLUDE_NFC_EMV_L1_STACK_LOGGING_H_
