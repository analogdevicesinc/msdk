/**
 * @file       rpu.c
 * @brief      This file contains the function implementations for the
 *             RPU peripheral module.
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

/* **** Includes **** */
#include <string.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "mxc_errors.h"
#include "rpu.h"
#include "rpu_reva.h"
#include "rpu_regs.h"

/* **** Functions **** */
int MXC_RPU_RevA_Allow(mxc_rpu_device_t periph, uint32_t allow_mask)
{
    // MAX32665-family only uses the bottom 9 bits of the RPU registers
    if (allow_mask & (0xFFFFFFFF << MXC_RPU_NUM_BUS_MASTERS)) {
        return E_BAD_PARAM;
    }

    // Writes to the RPU registers are ignored in thread (unprivileged) operation
    if (MXC_RPU_RevA_IsAllowed() != E_NO_ERROR) {
        return E_BAD_STATE;
    }

    // Add the register offset (periph) to the RPU base address to get the register address
    uint32_t *access_control_reg = (uint32_t *)(MXC_BASE_RPU + (uint32_t)periph);

    // Read-Modify-Write the register to enable access to bus masters specified in the mask
    *access_control_reg = (~allow_mask) & *access_control_reg;

    return E_NO_ERROR;
}

int MXC_RPU_RevA_Disallow(mxc_rpu_device_t periph, uint32_t disallow_mask)
{
    // MAX32665-family only uses the bottom 9 bits of the RPU registers
    if (disallow_mask & (0xFFFFFFFF << MXC_RPU_NUM_BUS_MASTERS)) {
        return E_BAD_PARAM;
    }

    // Writes to the RPU registers are ignored in thread (unprivileged) operation
    if (MXC_RPU_RevA_IsAllowed() != E_NO_ERROR) {
        return E_BAD_STATE;
    }

    // Add the register offset (periph) to the RPU Base Address to get the register address
    uint32_t *access_control_reg = (uint32_t *)(MXC_BASE_RPU + (uint32_t)periph);

    // Read-Modify-Write the register to disable access to bus masters specified in the mask
    *access_control_reg = disallow_mask | *access_control_reg;

    return E_NO_ERROR;
}

int MXC_RPU_RevA_IsAllowed(void)
{
    // Get the value of the ARM Core Control Register
    CONTROL_Type ctrl = (CONTROL_Type)__get_CONTROL();

    if (!(ctrl.b.nPRIV)) {
        return E_NO_ERROR;
    }

    return E_BAD_STATE;
}
