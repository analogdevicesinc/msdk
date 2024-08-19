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
