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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_SYSTEM_CORE1_MAX32665_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_SYSTEM_CORE1_MAX32665_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief   Start Core 1 code.
 */
void Start_Core1(void);

/**
 * @brief   Stops Core 1 by disabling CPU1 clock.
 */
void Stop_Core1(void);

/**
 * @brief   Main function for Core 1 Code.
 *          The user should override this function in their application code.
 */
int main_core1(void);

/**
 * @brief   Equivalent to PreInit for Core 0,
 *          Can be used for preliminary initialization.
 */
void PreInit_Core1(void);

/**
 * @brief   Initialize the system for Core 1.
 */
void SystemInit_Core1(void);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_SYSTEM_CORE1_MAX32665_H_
