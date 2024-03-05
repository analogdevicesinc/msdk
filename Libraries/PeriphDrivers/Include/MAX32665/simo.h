/**
 * @file    simo.h
 * @brief   SIMO function prototypes and data types.
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
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_SIMO_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_SIMO_H_

/* **** Includes **** */
#include "mxc_device.h"
#include "simo_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* **** Definitions **** */
#define VREGO_LOW_RANGE_BASE 500
#define VREGO_HIGH_RANGE_BASE 600

/* **** Function Prototypes **** */
void MXC_SIMO_SetVregO_A(uint32_t voltage);
void MXC_SIMO_SetVregO_B(uint32_t voltage);
void MXC_SIMO_SetVregO_C(uint32_t voltage);
void MXC_SIMO_SetVregO_D(uint32_t voltage);

uint32_t MXC_SIMO_GetOutReadyA(void);
uint32_t MXC_SIMO_GetOutReadyB(void);
uint32_t MXC_SIMO_GetOutReadyC(void);
uint32_t MXC_SIMO_GetOutReadyD(void);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_SIMO_H_
