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

/**
 * @file mxc_sys.c
 * @brief      System layer driver.
 * @details    This driver is used to control the system layer of the device.
 */

/* **** Includes **** */
#include <stddef.h>
#include <stdbool.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys_common.h"
#include "mxc_sys.h"
#include "flc.h"

/**
 * @ingroup mxc_sys
 * @{
 */

/* **** Definitions **** */
#define DAY_CODE_OFFSET 0x14
#define MONTH_CODE_OFFSET 0x15
#define YEAR_CODE_OFFSET 0x16
#define PKG_CODE_OFFSET 0x17

#define REG8_VAL(addr) (*(volatile uint8_t *)(MXC_INFO_MEM_BASE + addr))

/* **** Globals **** */
static mxc_sys_package_type_t pkg_type = MXC_SYS_PKG_UNSET;
/* **** Functions **** */

/* ************************************************************************** */
mxc_sys_package_type_t MXC_SYS_GetPackageType(void)
{
    if (pkg_type != MXC_SYS_PKG_UNSET) {
        return pkg_type;
    }

    mxc_sys_date_t date;

    // Package codes were only introduced when test date was
    if (MXC_SYS_GetTestDate(&date) != E_NO_ERROR) {
        return MXC_SYS_PKG_UNSET;
    }

    MXC_FLC_UnlockInfoBlock(MXC_INFO_MEM_BASE);

    const uint8_t maybe_pkg_type = REG8_VAL(PKG_CODE_OFFSET);
    const int err = MXC_SYS_SetPackageType(maybe_pkg_type);

    MXC_FLC_LockInfoBlock(MXC_INFO_MEM_BASE);

    MXC_ASSERT(err == E_NO_ERROR);

    return pkg_type;
}
int MXC_SYS_SetPackageType(mxc_sys_package_type_t new_pkg_type)
{
    switch (new_pkg_type) {
    case MXC_SYS_PKG_TQFN:
    case MXC_SYS_PKG_BGA:
    case MXC_SYS_PKG_WLP:
    case MXC_SYS_PKG_UNSET:
        pkg_type = new_pkg_type;
        return E_NO_ERROR;
    default:
        return E_BAD_PARAM;
    }
}

int MXC_SYS_GetTestDate(mxc_sys_date_t *date_info)
{
    MXC_ASSERT(date_info);

    MXC_FLC_UnlockInfoBlock(MXC_INFO_MEM_BASE);

    date_info->day = REG8_VAL(DAY_CODE_OFFSET);
    date_info->month = REG8_VAL(MONTH_CODE_OFFSET);
    date_info->year = REG8_VAL(YEAR_CODE_OFFSET);

    MXC_FLC_LockInfoBlock(MXC_INFO_MEM_BASE);

    // Flash is cleared if not valid
    // Year 2255 is valid
    if (date_info->month > 12 || date_info->day > 31) {
        return E_INVALID;
    }

    return E_NO_ERROR;
}

/**@} end of mxc_sys */
