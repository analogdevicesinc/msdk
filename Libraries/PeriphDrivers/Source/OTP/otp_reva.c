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
#include <stddef.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_errors.h"
#include "mxc_sys.h"
#include "otp.h"
#include "otp_reva.h"

/* **** Functions **** */

int MXC_OTP_RevA_Init(mxc_otp_reva_regs_t *otp, mxc_otp_clkdiv_t pclkdiv)
{
    MXC_SETFIELD(otp->clkdiv, MXC_F_OTP_REVA_CLKDIV_PCLKDIV,
                 (pclkdiv << MXC_F_OTP_CLKDIV_PCLKDIV_POS));

    return E_NO_ERROR;
}

int MXC_OTP_RevA_IsLocked(mxc_otp_reva_regs_t *otp)
{
    return !(otp->status & MXC_F_OTP_REVA_STATUS_UNLOCK1);
}

void MXC_OTP_RevA_Unlock(mxc_otp_reva_regs_t *otp)
{
    otp->actrl0 = 0xBEEFBA55;
}

void MXC_OTP_RevA_Lock(mxc_otp_reva_regs_t *otp)
{
    otp->actrl0 = 0;
}

int MXC_OTP_RevA_Write(mxc_otp_reva_regs_t *otp, uint16_t addr, uint32_t *data, uint16_t size)
{
    int i;
    int error;

    // Don't write out of accessible user block space.
    if (addr > (MXC_OTP_MEM_BASE + MXC_OTP_MEM_SIZE)) {
        return E_BAD_PARAM;
    }

    if ((addr + size) > (MXC_OTP_MEM_BASE + MXC_OTP_MEM_SIZE)) {
        return E_BAD_PARAM;
    }

    if (MXC_OTP_IsLocked()) {
        MXC_OTP_Unlock();
    }

    for (i = 0; i < size; i++) {
        error = MXC_OTP_Write32(addr + i, data[i]);
        if (error != E_NO_ERROR) {
            MXC_OTP_Lock();
            return error;
        }
    }

    MXC_OTP_Lock();

    return E_NO_ERROR;
}

int MXC_OTP_RevA_Write32(mxc_otp_reva_regs_t *otp, uint16_t addr, uint32_t data)
{
    // Make sure OTP memory is unlocked.
    if (MXC_OTP_IsLocked()) {
        return E_BAD_STATE;
    }

    // Check address range.
    if (addr > (MXC_OTP_MEM_BASE + MXC_OTP_MEM_SIZE)) {
        return E_BAD_PARAM;
    }

    otp->wdata = data;

    MXC_SETFIELD(otp->ctrl, MXC_F_OTP_REVA_CTRL_ADDR, (addr << MXC_F_OTP_REVA_CTRL_ADDR_POS));

    // Start Write operation
    otp->ctrl |= MXC_F_OTP_REVA_CTRL_WRITE;

    // Wait until operation is complete then verify it hasn't failed
    while (otp->status & MXC_F_OTP_REVA_STATUS_BUSY) {}

    if (otp->status & MXC_F_OTP_REVA_STATUS_FAIL) {
        return E_BAD_STATE;
    }

    return E_NO_ERROR;
}

int MXC_OTP_RevA_Read(mxc_otp_reva_regs_t *otp, uint16_t addr, uint32_t *data, uint16_t size)
{
    int i;
    int error;

    // Don't read out of accessible block space.
    if (addr > MXC_OTP_MEM_SIZE) {
        return E_BAD_PARAM;
    }

    if ((addr + size) > MXC_OTP_MEM_SIZE) {
        return E_BAD_PARAM;
    }

    for (i = 0; i < size; i++) {
        error = MXC_OTP_Read32(addr + i, data + i);
        if (error != E_NO_ERROR) {
            return error;
        }
    }

    return E_NO_ERROR;
}

int MXC_OTP_RevA_Read32(mxc_otp_reva_regs_t *otp, uint16_t addr, uint32_t *data)
{
    // User block (1k-2k) is readable in normal mode (without unlocking).
    MXC_SETFIELD(otp->ctrl, MXC_F_OTP_REVA_CTRL_ADDR, (addr << MXC_F_OTP_REVA_CTRL_ADDR_POS));

    // Start Read Operation
    otp->ctrl |= MXC_F_OTP_REVA_CTRL_READ;

    // Wait until operation is complete then verify it hasn't failed
    while (otp->status & MXC_F_OTP_REVA_STATUS_BUSY) {}

    if (otp->status & MXC_F_OTP_REVA_STATUS_FAIL) {
        return E_BAD_STATE;
    }

    *data = otp->rdata;

    return E_NO_ERROR;
}
