/**
 * @file       flc_common.h
 * @brief      Common functions for the flash controller driver.
 * @details    This driver can be used to operate on the embedded flash memory.
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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_FLC_FLC_COMMON_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_FLC_FLC_COMMON_H_

/* **** Includes **** */
#include "mxc_sys.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup flc Flash Controller  (FLC)
 * @ingroup periphlibs
 * @{
 */

/***** Definitions *****/

/***** Function Prototypes *****/

int MXC_FLC_Com_VerifyData(uint32_t address, uint32_t length, uint32_t *data);

int MXC_FLC_Com_Write(uint32_t address, uint32_t length, uint32_t *buffer);

void MXC_FLC_Com_Read(int address, void *buffer, int len);

volatile uint32_t *MXC_FLC_GetWELR(uint32_t address, uint32_t page_num);

volatile uint32_t *MXC_FLC_GetRLR(uint32_t address, uint32_t page_num);

/**@} end of group flc */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_FLC_FLC_COMMON_H_
