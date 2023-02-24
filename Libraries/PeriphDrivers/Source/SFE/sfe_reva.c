/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include "mxc_sys.h"
#include "sfe_reva.h"

int MXC_SFE_RevA_Init(void)
{
    return E_NO_ERROR;
}

int MXC_SFE_RevA_Shutdown(void)
{
    return E_NO_ERROR;
}

int MXC_SFE_RevA_ReadEnable(mxc_sfe_reva_regs_t *sfe)
{
    sfe->cfg |= MXC_F_SFE_REVA_CFG_RD_EN;
    return E_NO_ERROR;
}

int MXC_SFE_RevA_WriteEnable(mxc_sfe_reva_regs_t *sfe)
{
    sfe->cfg |= MXC_F_SFE_REVA_CFG_WR_EN;
    return E_NO_ERROR;
}

int MXC_SFE_RevA_SetFlashAddress(mxc_sfe_reva_regs_t *sfe, uint32_t lowerAdd, uint32_t upperAdd)
{
    sfe->flash_sba = lowerAdd;
    sfe->flash_sta = upperAdd;
    sfe->sfdp_sba = 0x10008000; // FLash base address

    return E_NO_ERROR;
}

int MXC_SFE_RevA_SetRAMAddress(mxc_sfe_reva_regs_t *sfe, uint32_t lowerAdd, uint32_t upperAdd)
{
    sfe->ram_sba = lowerAdd;
    sfe->ram_sta = upperAdd;

    return E_NO_ERROR;
}

int MXC_SFE_RevA_SetHostAddress(mxc_sfe_reva_regs_t *sfe, uint32_t RAMAdd, uint32_t FLASHAdd)
{
    sfe->hfsa = FLASHAdd;
    sfe->hrsa = RAMAdd;

    return E_NO_ERROR;
}
