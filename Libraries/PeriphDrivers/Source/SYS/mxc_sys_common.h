/**
 * @file    mxc_sys.h
 * @brief   System level header file.
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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_SYS_MXC_SYS_COMMON_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_SYS_MXC_SYS_COMMON_H_

#include "mxc_device.h"
#include "gcr_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup mxc_sys_common System Configuration (MXC_SYS)
 * @ingroup syscfg
 * @details API for system configuration common to all parts including getting package type.
 * @{
 */

/** @brief Enumeration to select Package Type*/
typedef enum {
    MXC_SYS_PKG_TQFN = 1,
    MXC_SYS_PKG_BGA = 2,
    MXC_SYS_PKG_WLP = 3,
    MXC_SYS_PKG_UNSET = 0xff
} mxc_sys_package_type_t;

typedef struct {
    uint8_t day, month, year;
} mxc_sys_date_t;

/**
 * @brief Reads the device package type.
 * @returns         Device Package Type (See mxc_sys_package_t for available options)
 */
mxc_sys_package_type_t MXC_SYS_GetPackageType(void);
/**
 * @brief Set the package type. (Acts as an override to what is in Info Block)
 * @param new_pkg_type       Device Package Type (See mxc_sys_package_t for available options)
 * @returns         E_NO_ERROR if package exists.
 */
int MXC_SYS_SetPackageType(mxc_sys_package_type_t new_pkg_type);
/**
 * @brief Get date of production test
 * @param date_info       Pointer to date information struct
 * @returns         E_NO_ERROR if date is valid. E_INVALID otherwise
 */
int MXC_SYS_GetTestDate(mxc_sys_date_t *date_info);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_SYS_MXC_SYS_COMMON_H_
