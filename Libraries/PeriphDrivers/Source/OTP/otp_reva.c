/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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
 ******************************************************************************/

/* **** Includes **** */
#include <stddef.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_errors.h"
#include "otp.h"
#include "otp_reva.h"

/* **** Functions **** */
int MXC_OTP_RevA_CheckUnlockMode(mxc_otp_reva_regs_t *otp)
{
    return 0;
}

void MXC_OTP_RevA_Unlock(mxc_otp_reva_regs_t *otp)
{
    otp->actrl1 = 0x3A7F5CA3;
    otp->actrl1 = 0xA1E34F20;
    otp->actrl1 = 0x9608B2C1;
}

void MXC_OTP_RevA_Lock(mxc_otp_reva_regs_t *otp)
{
    otp->actrl1 = 0;
}

void MXC_OTP_RevA_UnlockUserBlock(mxc_otp_reva_regs_t *otp)
{
    otp->actrl0 = 0xBEEFBA55;
}

void MXC_OTP_RevA_LockUserBlock(mxc_otp_reva_regs_t *otp)
{
    otp->actrl0 = 0;
}

int MXC_OTP_RevA_Write(mxc_otp_reva_regs_t *otp, uint16_t addr, uint32_t data)
{
    // Check address range and mode access (unlocked).
    if (otp->status & MXC_F_OTP_REVA_STATUS_UNLOCK3) {
        // Sys and User Block (0-2k)
        if (addr > MXC_OTP_MEM_SIZE) {
            return E_BAD_PARAM;
        }
    } else if (otp->status & MXC_F_OTP_REVA_STATUS_UNLOCK1) {
        // User block only (1k-2k)
        if ((addr < (MXC_OTP_MEM_SIZE / 2)) || (addr > MXC_OTP_MEM_SIZE)) {
            return E_BAD_PARAM;
        }
    } else {
        // OTP not unlocked
        return E_BAD_STATE;
    }

    // TODO: Check if order of writing data and address matters.
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

int MXC_OTP_RevA_MultiWrite(mxc_otp_reva_regs_t *otp, uint16_t addr, uint32_t *data, uint16_t size)
{
    int i;
    int error;

    // Don't write out of accessible block space.
    if (addr > MXC_OTP_MEM_SIZE) {
        return E_BAD_PARAM;
    }

    if ((addr + size) > MXC_OTP_MEM_SIZE) {
        return E_BAD_PARAM;
    }

    MXC_OTP_Unlock();

    for (i = 0; i < size; i++) {
        error = MXC_OTP_Write(addr + i, data[i]);
        if (error != E_NO_ERROR) {
            MXC_OTP_Lock();
            return error;
        }
    }

    MXC_OTP_Lock();

    return E_NO_ERROR;
}

int MXC_OTP_RevA_Read(mxc_otp_reva_regs_t *otp, uint16_t addr, uint32_t *data)
{
    // User block (1k-2k) is readable in normal mode (without unlocking).
    // Check for address range and appropriate read access.
    if (addr < (MXC_OTP_MEM_SIZE / 2)) {
        if (!(otp->status & MXC_F_OTP_REVA_STATUS_UNLOCK3)) {
            return E_BAD_STATE;
        }
    } else if (addr > MXC_OTP_MEM_SIZE) {
        return E_BAD_PARAM;
    }

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

int MXC_OTP_RevA_MultiRead(mxc_otp_reva_regs_t *otp, uint16_t addr, uint32_t *data, uint16_t size)
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

    MXC_OTP_Unlock();

    for (i = 0; i < size; i++) {
        error = MXC_OTP_Read(addr + i, data + i);
        if (error != E_NO_ERROR) {
            MXC_OTP_Lock();
            return error;
        }
    }

    MXC_OTP_Lock();

    return E_NO_ERROR;
}
