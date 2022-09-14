/* ****************************************************************************
 * Copyright (C) 2019 Maxim Integrated Products, Inc., All Rights Reserved.
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
 *************************************************************************** */

#include "emcc.h"
#include "emcc_reva.h"
#include "gcr_regs.h"
#include "mxc_device.h"

uint32_t MXC_EMCC_ID(mxc_emcc_info_t id)
{
    return MXC_EMCC_RevA_ID((mxc_emcc_reva_regs_t *)MXC_EMCC, id);
}

uint32_t MXC_EMCC_CacheSize(void)
{
    return MXC_EMCC_RevA_CacheSize((mxc_emcc_reva_regs_t *)MXC_EMCC);
}

uint32_t MXC_EMCC_MemSize(void)
{
    return MXC_EMCC_RevA_MemSize((mxc_emcc_reva_regs_t *)MXC_EMCC);
}

void MXC_EMCC_Enable(void)
{
    MXC_GCR->sysctrl &= ~MXC_F_GCR_SYSCTRL_SYSCACHE_DIS;
    MXC_EMCC_RevA_Enable((mxc_emcc_reva_regs_t *)MXC_EMCC);
}

void MXC_EMCC_Disable(void)
{
    MXC_EMCC_RevA_Disable((mxc_emcc_reva_regs_t *)MXC_EMCC);
    MXC_GCR->sysctrl |= MXC_F_GCR_SYSCTRL_SYSCACHE_DIS;
}

void MXC_EMCC_Flush(void)
{
    MXC_GCR->sysctrl |= MXC_F_GCR_SYSCTRL_ICC0_FLUSH;
}

void MXC_EMCC_WriteAllocateEnable(void)
{
    MXC_EMCC_RevA_WriteAllocateEnable((mxc_emcc_reva_regs_t *)MXC_EMCC);
}

void MXC_EMCC_WriteAllocateDisable(void)
{
    MXC_EMCC_RevA_WriteAllocateDisable((mxc_emcc_reva_regs_t *)MXC_EMCC);
}

void MXC_EMCC_CriticalWordFirstEnable(void) //cwfst_dis
{
    MXC_EMCC_RevA_CriticalWordFirstEnable((mxc_emcc_reva_regs_t *)MXC_EMCC);
}

void MXC_EMCC_CriticalWordFirstDisable(void) //cwfst_dis
{
    MXC_EMCC_RevA_CriticalWordFirstDisable((mxc_emcc_reva_regs_t *)MXC_EMCC);
}

uint32_t MXC_EMCC_Ready(void)
{
    return MXC_EMCC_RevA_Ready((mxc_emcc_reva_regs_t *)MXC_EMCC);
}
