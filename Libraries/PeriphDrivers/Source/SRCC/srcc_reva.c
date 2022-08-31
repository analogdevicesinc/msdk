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

#include "srcc_reva.h"

#if TARGET_NUM != 32650
#include "srcc.h"
uint32_t MXC_SRCC_RevA_ID(mxc_srcc_reva_regs_t* srcc, mxc_srcc_cache_id_t id)
{
    switch (id) {
    case SRCC_CACHE_ID_RELNUM:
        return (((srcc->cache_id) & MXC_F_SRCC_REVA_CACHE_ID_RELNUM))
            >> MXC_F_SRCC_REVA_CACHE_ID_RELNUM_POS;

    case SRCC_CACHE_ID_PARTNUM:
        return (((srcc->cache_id) & MXC_F_SRCC_REVA_CACHE_ID_PARTNUM))
            >> MXC_F_SRCC_REVA_CACHE_ID_PARTNUM_POS;

    case SRCC_CACHE_ID_CCHID:
    default:
        return (((srcc->cache_id) & MXC_F_SRCC_REVA_CACHE_ID_CCHID))
            >> MXC_F_SRCC_REVA_CACHE_ID_CCHID_POS;
    }
}
#endif

uint32_t MXC_SRCC_RevA_CacheSize(mxc_srcc_reva_regs_t* srcc)
{
    return (((srcc->memcfg) & MXC_F_SRCC_REVA_MEMCFG_CCHSZ)) >> MXC_F_SRCC_REVA_MEMCFG_CCHSZ_POS;
}

uint32_t MXC_SRCC_RevA_MemSize(mxc_srcc_reva_regs_t* srcc)
{
    return (srcc->memcfg & MXC_F_SRCC_REVA_MEMCFG_MEMSZ) >> MXC_F_SRCC_REVA_MEMCFG_MEMSZ_POS;
}

void MXC_SRCC_RevA_Enable(mxc_srcc_reva_regs_t* srcc)
{
    srcc->cache_ctrl |= MXC_F_SRCC_REVA_CACHE_CTRL_CACHE_EN;
}

void MXC_SRCC_RevA_Disable(mxc_srcc_reva_regs_t* srcc)
{
    srcc->cache_ctrl &= ~MXC_F_SRCC_REVA_CACHE_CTRL_CACHE_EN;
}

void MXC_SRCC_RevA_WriteAllocateEnable(mxc_srcc_reva_regs_t* srcc)
{
    srcc->cache_ctrl |= MXC_F_SRCC_REVA_CACHE_CTRL_WRITE_ALLOC_EN;
}

void MXC_SRCC_RevA_WriteAllocateDisable(mxc_srcc_reva_regs_t* srcc)
{
    srcc->cache_ctrl &= ~MXC_F_SRCC_REVA_CACHE_CTRL_WRITE_ALLOC_EN;
}

void MXC_SRCC_RevA_CriticalWordFirstEnable(mxc_srcc_reva_regs_t* srcc) // cwfst_dis
{
    srcc->cache_ctrl &= ~MXC_F_SRCC_REVA_CACHE_CTRL_CWFST_DIS;
}

void MXC_SRCC_RevA_CriticalWordFirstDisable(mxc_srcc_reva_regs_t* srcc) // cwfst_dis
{
    srcc->cache_ctrl |= MXC_F_SRCC_REVA_CACHE_CTRL_CWFST_DIS;
}

uint32_t MXC_SRCC_RevA_Ready(mxc_srcc_reva_regs_t* srcc)
{
    return (srcc->cache_ctrl & MXC_F_SRCC_REVA_CACHE_CTRL_CACHE_RDY)
        >> MXC_F_SRCC_REVA_CACHE_CTRL_CACHE_RDY_POS;
}
