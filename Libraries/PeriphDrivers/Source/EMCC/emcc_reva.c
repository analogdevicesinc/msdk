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

#include "emcc_reva.h"

#if TARGET_NUM != 32650
#include "emcc.h"
uint32_t MXC_EMCC_RevA_ID(mxc_emcc_reva_regs_t *emcc, mxc_emcc_info_t id)
{
    switch (id) {
    case EMCC_INFO_RELNUM:
        return (((emcc->info) & MXC_F_EMCC_REVA_INFO_RELNUM)) >> MXC_F_EMCC_REVA_INFO_RELNUM_POS;

    case EMCC_INFO_PARTNUM:
        return (((emcc->info) & MXC_F_EMCC_REVA_INFO_PARTNUM)) >> MXC_F_EMCC_REVA_INFO_PARTNUM_POS;

    case EMCC_INFO_ID:
    default:
        return (((emcc->info) & MXC_F_EMCC_REVA_INFO_ID)) >> MXC_F_EMCC_REVA_INFO_ID_POS;
    }
}
#endif

uint32_t MXC_EMCC_RevA_CacheSize(mxc_emcc_reva_regs_t *emcc)
{
    return (((emcc->sz) & MXC_F_EMCC_REVA_SZ_CCH)) >> MXC_F_EMCC_REVA_SZ_CCH_POS;
}

uint32_t MXC_EMCC_RevA_MemSize(mxc_emcc_reva_regs_t *emcc)
{
    return (emcc->sz & MXC_F_EMCC_REVA_SZ_MEM) >> MXC_F_EMCC_REVA_SZ_MEM_POS;
}

void MXC_EMCC_RevA_Enable(mxc_emcc_reva_regs_t *emcc)
{
    emcc->ctrl |= MXC_F_EMCC_REVA_CTRL_EN;
}

void MXC_EMCC_RevA_Disable(mxc_emcc_reva_regs_t *emcc)
{
    emcc->ctrl &= ~MXC_F_EMCC_REVA_CTRL_EN;
}

void MXC_EMCC_RevA_WriteAllocateEnable(mxc_emcc_reva_regs_t *emcc)
{
    emcc->ctrl |= MXC_F_EMCC_REVA_CTRL_WRITE_ALLOC;
}

void MXC_EMCC_RevA_WriteAllocateDisable(mxc_emcc_reva_regs_t *emcc)
{
    emcc->ctrl &= ~MXC_F_EMCC_REVA_CTRL_WRITE_ALLOC;
}

void MXC_EMCC_RevA_CriticalWordFirstEnable(mxc_emcc_reva_regs_t *emcc) //cwfst_dis
{
    emcc->ctrl |= MXC_F_EMCC_REVA_CTRL_CWFST_DIS;
}

void MXC_EMCC_RevA_CriticalWordFirstDisable(mxc_emcc_reva_regs_t *emcc) //cwfst_dis
{
    emcc->ctrl &= ~MXC_F_EMCC_REVA_CTRL_CWFST_DIS;
}

uint32_t MXC_EMCC_RevA_Ready(mxc_emcc_reva_regs_t *emcc)
{
    return (emcc->ctrl & MXC_F_EMCC_REVA_CTRL_RDY) >> MXC_F_EMCC_REVA_CTRL_RDY_POS;
}
