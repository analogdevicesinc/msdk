/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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

/**
 * @file    sfcc.h
 * @brief   SPI Flash Controller Cache(SFCC) function prototypes and data types.
 */

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32572_SFCC_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32572_SFCC_H_

/* **** Includes **** */
#include <stdint.h>
#include "sfcc_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup sfcc SFCC
 * @ingroup periphlibs
 * @{
 */

/**
 * @brief Enumeration type for the Cache ID Register
 */
typedef enum {
    SFCC_INFO_RELNUM, ///< Identifies the Release Number
    SFCC_INFO_PARTNUM, ///< Specifies the value of Cache ID Part Number
    SFCC_INFO_ID ///< Specifies the value of Cache ID
} mxc_sfcc_info_t;

/**
 * @brief   Reads the data from the Cache Id Register.
 * @param   cid Enumeration type for Cache Id Register.
 * @retval  Returns the contents of Cache Id Register.
 */
int MXC_SFCC_ID(mxc_sfcc_info_t cid);

/**
 * @brief   Enable the instruction cache controller.
 */
void MXC_SFCC_Enable(void);

/**
 * @brief   Disable the instruction cache controller.
 */
void MXC_SFCC_Disable(void);

/**
 * @brief   Flush the instruction cache controller.
 */
void MXC_SFCC_Flush(void);

/**@} end of group sfcc */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32572_SFCC_H_
