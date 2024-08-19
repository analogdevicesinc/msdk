/**
 * @file    mxc_device.h
 * @brief   Device specific header file.
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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32520_MXC_DEVICE_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32520_MXC_DEVICE_H_

#include "max32520.h"
#include "mxc_errors.h"
#include "mxc_pins.h"

#ifndef TARGET
#error TARGET NOT DEFINED
#endif

// Create a string definition for the TARGET
#define STRING_ARG(arg) #arg
#define STRING_NAME(name) STRING_ARG(name)
#define TARGET_NAME STRING_NAME(TARGET)

// Define which revisions of the IP we are using
#ifndef TARGET_REV
#error TARGET_REV NOT DEFINED
#endif

#if (TARGET_REV == 0x4131)
// A1
#define MXC_TMR_REV 0
#define MXC_UART_REV 0
#else

#error TARGET_REV NOT SUPPORTED

#endif /* if(TARGET_REV == ...) */

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32520_MXC_DEVICE_H_
