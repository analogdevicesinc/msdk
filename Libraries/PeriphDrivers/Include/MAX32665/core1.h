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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_CORE1_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_CORE1_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file    core1.h
 * @brief   Startup Code for MAX32665 Family CPU1
 * @details These functions are called at the startup of the second ARM core (CPU1/Core1)
 */

/**
 * @brief Starts the code on core 1
 *        Core1 code beings executing from Core1_Main()
 */
void Core1_Start(void);

/**
 * @brief Stops code executing in Core 1
 */
void Core1_Stop(void);

/**
 * @brief Main function for Core 1 Code
 *        The user should override this function
 *        in their application code
 */
int Core1_Main(void);

/**
 * @brief Equivalent to PreInit for Core 0
 *        Can be used for preliminary initialization
 */
void PreInit_Core1(void);

/**
 * @brief Equivalent to PreInit for Core 1
 *        Enables FPU, and ICache
 *        Sets interrupt vector
 */
void SystemInit_Core1(void);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_CORE1_H_
