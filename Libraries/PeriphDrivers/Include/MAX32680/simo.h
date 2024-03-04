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
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32680_SIMO_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32680_SIMO_H_

/* **** Includes **** */
#include "mxc_device.h"
#include "simo_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup simo SIMO
 * @ingroup periphlibs
 * @{
 */

/* **** Definitions **** */
#define VREGO_LOW_RANGE_BASE 500
#define VREGO_HIGH_RANGE_BASE 600

/* **** Function Prototypes **** */
void MXC_SIMO_setVregO_A(uint32_t voltage);
void MXC_SIMO_setVregO_B(uint32_t voltage);
void MXC_SIMO_setVregO_C(uint32_t voltage);
void MXC_SIMO_setVregO_D(uint32_t voltage);

void MXC_SIMO_setIpkA(uint32_t peak_current);
void MXC_SIMO_setIpkB(uint32_t peak_current);
void MXC_SIMO_setIpkC(uint32_t peak_current);
void MXC_SIMO_setIpkD(uint32_t peak_current);

void MXC_SIMO_setMaxTon(uint32_t ontime);

void MXC_SIMO_setAlertThresholdA(uint32_t threshold);
void MXC_SIMO_setAlertThresholdB(uint32_t threshold);
void MXC_SIMO_setAlertThresholdC(uint32_t threshold);
void MXC_SIMO_setAlertThresholdD(uint32_t threshold);

void MXC_SIMO_setZeroCrossCalA(uint32_t zerocross);
void MXC_SIMO_setZeroCrossCalB(uint32_t zerocross);
void MXC_SIMO_setZeroCrossCalC(uint32_t zerocross);
void MXC_SIMO_setZeroCrossCalD(uint32_t zerocross);

uint32_t MXC_SIMO_getOutReadyA(void);
uint32_t MXC_SIMO_getOutReadyB(void);
uint32_t MXC_SIMO_getOutReadyC(void);
uint32_t MXC_SIMO_getOutReadyD(void);

/**@} end of group simo */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32680_SIMO_H_
