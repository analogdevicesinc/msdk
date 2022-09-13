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
#include <stdint.h>
#include "mxc_sys.h"
#include "mxc_errors.h"
#include "mxc_device.h"
#include "mxc_assert.h"
#include "otp.h"
#include "otp_reva.h"

/* **** Functions **** */

int MXC_OTP_Init(mxc_otp_clkdiv_t pclkdiv)
{
    int lowest_pclkdiv_int;
    int lowest_pclkdiv_dec;

    // Divide by 16 is default divider value.
    if (pclkdiv < MXC_OTP_CLK_DIV2 || pclkdiv > MXC_OTP_CLK_DIV32) {
        pclkdiv = MXC_OTP_CLK_DIV16;
    }

    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_OTP);

    // Note: mxc_otp_clkdiv_t values based on predefined register definitions.

    // Verify minimum timing requirement for OTP Controller. (>=400ns) Check UG.
    // Note: mxc_otp_clkdiv_t values based on predefined register definitions.
    //  For example: pclkdiv = MXC_OTP_CLK_DIV2 = 0x01
    //  Divider value = pclkdiv + 1
    //  Divide by 2 = MXC_OTP_CLK_DIV2 + 1 = 0x01 + 1
    lowest_pclkdiv_int = (PeripheralClock / 2500000); // (1 / (400ns)) = 2,500,000

    // Do not truncate down if equation above doesn't perfectly divide out.
    lowest_pclkdiv_dec = (100 * PeripheralClock / 2500000) - lowest_pclkdiv_int;
    if (lowest_pclkdiv_dec > 0) {
        lowest_pclkdiv_int += 1;
    }

    // Get raw clkdiv value. Divider value = pclkdiv + 1
    lowest_pclkdiv_int -= 1;

    if (pclkdiv < lowest_pclkdiv_int) {
        return E_BAD_PARAM;
    }

    return MXC_OTP_RevA_Init((mxc_otp_reva_regs_t *)MXC_OTP, pclkdiv);
}

int MXC_OTP_IsLocked(void)
{
    return MXC_OTP_RevA_IsLocked((mxc_otp_reva_regs_t *)MXC_OTP);
}

void MXC_OTP_Unlock(void)
{
    MXC_OTP_RevA_Unlock((mxc_otp_reva_regs_t *)MXC_OTP);
}

void MXC_OTP_Lock(void)
{
    MXC_OTP_RevA_Lock((mxc_otp_reva_regs_t *)MXC_OTP);
}

int MXC_OTP_Write(uint16_t addr, uint32_t *data, uint16_t size)
{
    return MXC_OTP_RevA_Write((mxc_otp_reva_regs_t *)MXC_OTP, addr, data, size);
}

int MXC_OTP_Write32(uint16_t addr, uint32_t data)
{
    return MXC_OTP_RevA_Write32((mxc_otp_reva_regs_t *)MXC_OTP, addr, data);
}

int MXC_OTP_Read(uint16_t addr, uint32_t *data, uint16_t size)
{
    return MXC_OTP_RevA_Read((mxc_otp_reva_regs_t *)MXC_OTP, addr, data, size);
}

int MXC_OTP_Read32(uint16_t addr, uint32_t *data)
{
    return MXC_OTP_RevA_Read32((mxc_otp_reva_regs_t *)MXC_OTP, addr, data);
}
