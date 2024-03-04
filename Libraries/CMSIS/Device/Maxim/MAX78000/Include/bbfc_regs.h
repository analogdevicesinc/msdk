/**
 * @file    bbfc_regs.h
 * @brief   The BBFC block has been renamed to GCFR.  
 *          This include file is for backwards compatibility only.
 * @note    This file is @deprecated.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78000_INCLUDE_BBFC_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78000_INCLUDE_BBFC_REGS_H_

// The BBFC block has been renamed to GCFR.  
// This include file is for backwards compatibility only.

// Warning only pops up when a project builds with this file included.
#warning "MXC_BBFC (bbfc_regs.h) name is deprecated. Please use MXC_GCFR (gcfr_regs.h)."

#include "gcfr_regs.h"
#define MXC_BBFC MXC_GCFR

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78000_INCLUDE_BBFC_REGS_H_
