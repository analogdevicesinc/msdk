/**
 * @file    srcc.h
 * @brief   This file contains all functions prototypes and data types for the
 *          External Memory Cache Controller (SRCC) driver
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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_SRCC_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_SRCC_H_

/***** Includes *****/
#include "srcc_regs.h"
#include "mxc_device.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup srcc SPIX Cache Controller (SRCC)
 * @ingroup periphlibs
 * @{
 */

/***** Definitions *****/

/**
 * @brief   Enumeration type for the SRCC Cache ID Register
 */
typedef enum {
    SRCC_CACHE_ID_RELNUM, ///< Release Number
    SRCC_CACHE_ID_PARTNUM, ///< Part Number
    SRCC_CACHE_ID_CCHID ///< Cache ID
} mxc_srcc_cache_id_t;

/***** Function Prototypes *****/

/**
 * @brief   Reads the data from the SRCC Cache ID Register
 * @param   id      Enumeration type for the SRCC Cache ID Register
 * @returns The contents of SRCC cache ID Register
 */
uint32_t MXC_SRCC_ID(mxc_srcc_cache_id_t id);

/**
 * @brief   Gets the cache size in Kbytes. The default value is 16KB.
 * @returns Cache size, in Kbytes
 */
uint32_t MXC_SRCC_CacheSize(void);

/**
 * @brief   Gets the main memory size in units of 128KB. The default value is 512MB.
 * @returns Main memory size, in units of 128KB
 */
uint32_t MXC_SRCC_MemSize(void);

/**
 * @brief   Enables the data cache controller
 */
void MXC_SRCC_Enable(void);

/**
 * @brief   Disables the data cache controller
 */
void MXC_SRCC_Disable(void);

/**
 * @brief   Flushes the data cache controller
 */
void MXC_SRCC_Flush(void);

/**
 * @brief   Enables write-allocate mode with data cache controller
 */
void MXC_SRCC_WriteAllocEnable(void);

/**
 * @brief   Disables write-allocate mode with data cache controller
 */
void MXC_SRCC_WriteAllocDisable(void);

/**
 * @brief   Enables critical-word-first mode with data cache controller
 */
void MXC_SRCC_CriticalWordFirstEnable(void);

/**
 * @brief   Disables critical-word-first mode with data cache controller
 */
void MXC_SRCC_CriticalWordFirstDisable(void);

/**
 * @brief   Reads the SRCC Cache ready flag, which is set and cleared by hardware
 * @returns SRCC Cache ready flag
 */
uint32_t MXC_SRCC_Ready(void);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_SRCC_H_
/**
 * @} srcc
 */
