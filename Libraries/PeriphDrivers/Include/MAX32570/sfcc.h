/**
 * @file    sfcc.h
 * @brief   SPI Flash Controller Cache(SFCC) function prototypes and data types.
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
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_SFCC_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_SFCC_H_

/* **** Includes **** */
#include <stdint.h>
#include "icc_regs.h"
#include "icc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup sfcc SFCC
 * @ingroup periphlibs
 * @{
 */

/**
 * @brief   Reads the data from the Cache Id Register.
 * @param   cid Enumeration type for Cache Id Register.
 * @retval  Returns the contents of Cache Id Register.
 */
int MXC_SFCC_ID(mxc_icc_info_t cid);

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

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_SFCC_H_
