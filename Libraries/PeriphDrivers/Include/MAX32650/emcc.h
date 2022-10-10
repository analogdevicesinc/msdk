/**
 * Copyright 2017 Maxim Integrated Products, Inc.,
 * All Rights Reserved.</center></h2>
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
 ********************************************************************************
*/

/*
 ******************************************************************************
 * @file    emcc.h
 * @brief   This file contains all functions prototypes and data types for the
 *          External Memory Cache Controller (EMCC) driver
 ******************************************************************************
*/

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_EMCC_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_EMCC_H_

/***** Includes *****/
#include "max32650.h"
#include "emcc_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup emcc External Memory Cache Controller (EMCC)
 * @ingroup periphlibs
 * @{
 */

/***** Definitions *****/

/**
 * Enumeration type for the EMCC Cache ID Register
 */
typedef enum {
    MXC_EMCC_CACHE_ID_RELNUM, // Release Number
    MXC_EMCC_CACHE_ID_PARTNUM, // Part Number
    MXC_EMCC_CACHE_ID_CCHID // Cache ID
} mxc_emcc_cache_id_t;

/***** Function Prototypes *****/

/**
 * @brief   Reads the data from the EMCC Cache ID Register
 * @param   id      Enumeration type for the EMCC Cache ID Register
 * @returns The contents of EMCC cache ID Register
 */
uint32_t MXC_EMCC_ID(mxc_emcc_cache_id_t id);

/**
 * @brief   Gets the cache size in Kbytes. The default value is 16KB.
 * @returns Cache size, in Kbytes
 */
uint32_t MXC_EMCC_CacheSize(void);

/**
 * @brief   Gets the main memory size in units of 128KB. The default value is 512MB.
 * @returns Main memory size, in units of 128KB
 */
uint32_t MXC_EMCC_MemSize(void);

/**
 * @brief   Enables the data cache controller
 */
void MXC_EMCC_Enable(void);

/**
 * @brief   Disables the data cache controller
 */
void MXC_EMCC_Disable(void);

/**
 * @brief   Flushes the data cache controller
 */
void MXC_EMCC_Flush(void);

/**
 * @brief   Enables write-allocate mode with data cache controller
 */
void MXC_EMCC_WriteAllocEnable(void);

/**
 * @brief   Disables write-allocate mode with data cache controller
 */
void MXC_EMCC_WriteAllocDisable(void);

/**
 * @brief   Enables critical-word-first mode with data cache controller
 */
void MXC_EMCC_CriticalWordFirstEnable(void);

/**
 * @brief   Disables critical-word-first mode with data cache controller
 */
void MXC_EMCC_CriticalWordFirstDisable(void);

/**
 * @brief   Reads the EMCC Cache ready flag, which is set and cleared by hardware
 * @returns EMCC Cache ready flag
 */
uint32_t MXC_EMCC_Ready(void);

/**
 * @brief   Invalidate the entire contents of the data cache.
 */
void MXC_EMCC_Invalidate_All(void);

/**@} end of group emcc */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_EMCC_H_
