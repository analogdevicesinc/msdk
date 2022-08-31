/**
 * @file       rpu.c
 * @brief      This file contains the function implementations for the
 *             RPU peripheral module.
 */

/* *****************************************************************************
 * Copyright (C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 * $Date: 2019-02-26 15:48:52 -0600 (Tue, 26 Feb 2019) $
 * $Revision: 41251 $
 *
 **************************************************************************** */

/* **** Includes **** */
#include "rpu_reva.h"
#include "mxc_assert.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_sys.h"
#include "rpu.h"
#include "rpu_regs.h"
#include <string.h>

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
    uint32_t* access_control_reg = (uint32_t*)(MXC_BASE_RPU + (uint32_t)periph);

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
    uint32_t* access_control_reg = (uint32_t*)(MXC_BASE_RPU + (uint32_t)periph);

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
