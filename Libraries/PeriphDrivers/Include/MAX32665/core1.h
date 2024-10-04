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

#include "mxc_device.h"

#warning "core1.h is deprecated (05-24-2024)."
#warning "Use mxc_device.h instead, and set `ARM_DUALCORE=1` in project.mk"
#warning "Core 1 Startup/System code is located at Libraries/CMSIS/Device/Maxim/MAX32665/"

/**
 * @file    core1.h
 * @brief   Startup Code for MAX32665 Family CPU1
 * @details These functions are called at the startup of the second ARM core (CPU1/Core1)
 */

/**
 * @brief Starts the code on core 1
 *        Core1 code beings executing from Core1_Main()
 */
#if defined(__GNUC__)
inline __attribute__((deprecated("Use Start_Core1(); instead."))) void Core1_Start(void)
{
    Start_Core1();
}
#endif

/**
 * @brief Stops code executing in Core 1
 */
#if defined(__GNUC__)
inline __attribute__((deprecated("Use Stop_Core1(); instead."))) void Core1_Stop(void)
{
    Stop_Core1();
}
#endif

/**
 * @brief Main function for Core 1 Code
 *        The user should override this function
 *        in their application code
 */
#if defined(__GNUC__)
inline __attribute__((deprecated(
    "Use `int main_core1(void)` instead - main_core1 is Core 1's entry point where code starts, not Core1_Main."))) int
Core1_Main(void);
#endif

// void PreInit_Core1(void) is now located in system_core_max32665.h
// void SystemInit_Core1(void) is now located in system_core_max32665.h

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_CORE1_H_
