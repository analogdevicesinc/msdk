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

/* **** Includes **** */
#include "emcc.h"
#include "srcc_reva.h"

/* ************************************************************************** */
uint32_t MXC_EMCC_ID(mxc_emcc_cache_id_t id)
{
    switch (id) {
    case MXC_EMCC_CACHE_ID_RELNUM:
        return ((MXC_EMCC->cache_id) & MXC_F_EMCC_CACHE_ID_RELNUM) >>
               MXC_F_EMCC_CACHE_ID_RELNUM_POS;
    case MXC_EMCC_CACHE_ID_PARTNUM:
        return ((MXC_EMCC->cache_id) & MXC_F_EMCC_CACHE_ID_PARTNUM) >>
               MXC_F_EMCC_CACHE_ID_PARTNUM_POS;
    case MXC_EMCC_CACHE_ID_CCHID:
        return ((MXC_EMCC->cache_id) & MXC_F_EMCC_CACHE_ID_CCHID) >> MXC_F_EMCC_CACHE_ID_CCHID_POS;
    default:
        return E_BAD_PARAM;
    }
}

/* ************************************************************************** */
uint32_t MXC_EMCC_CacheSize(void)
{
    return MXC_SRCC_RevA_CacheSize((mxc_srcc_reva_regs_t *)MXC_EMCC);
}

/* ************************************************************************** */
uint32_t MXC_EMCC_MemSize(void)
{
    return MXC_SRCC_RevA_MemSize((mxc_srcc_reva_regs_t *)MXC_EMCC);
}

/* ************************************************************************** */
void MXC_EMCC_Enable(void)
{
    MXC_SRCC_RevA_Enable((mxc_srcc_reva_regs_t *)MXC_EMCC);
}

/* ************************************************************************** */
void MXC_EMCC_Disable(void)
{
    MXC_SRCC_RevA_Disable((mxc_srcc_reva_regs_t *)MXC_EMCC);
}

/* ************************************************************************** */
void MXC_EMCC_Flush(void)
{
    MXC_EMCC_Disable();
    MXC_EMCC_Enable();
}

/* ************************************************************************** */
void MXC_EMCC_WriteAllocEnable(void)
{
    /* When a cache line is allocated on write operations, this is called
    "write-allocate". However, there can be performance problems with
    "write-allocate" because software frequently operates memset() on large
    portions of memory. This can "pollute" the cache with unwanted cache lines.
    To avoid this issue, the write-allocate feature is disable by default. The
    write- allocate enable bit is in CACHE_CTRL[1]. */
    MXC_SRCC_RevA_WriteAllocateEnable((mxc_srcc_reva_regs_t *)MXC_EMCC);
}

/* ************************************************************************** */
void MXC_EMCC_WriteAllocDisable(void)
{
    MXC_SRCC_RevA_WriteAllocateDisable((mxc_srcc_reva_regs_t *)MXC_EMCC);
}

/* ************************************************************************** */
void MXC_EMCC_CriticalWordFirstEnable(void)
{
    if (!(MXC_EMCC->cache_ctrl &
          MXC_F_EMCC_CACHE_CTRL_ENABLE)) { //CWFST_DIS field only writable when cache disabled
        MXC_EMCC->cache_ctrl &= ~MXC_F_EMCC_CACHE_CTRL_CWFST_DIS;
    }
}

/* ************************************************************************** */
void MXC_EMCC_CriticalWordFirstDisable(void)
{
    if (!(MXC_EMCC->cache_ctrl &
          MXC_F_EMCC_CACHE_CTRL_ENABLE)) { //CWFST_DIS field only writable when cache disabled
        MXC_EMCC->cache_ctrl |= MXC_F_EMCC_CACHE_CTRL_CWFST_DIS;
    }
}

/* ************************************************************************** */
uint32_t MXC_EMCC_Ready(void)
{
    /* Cache Ready flag. Cleared by hardware when at any time the cache as a
    whole is invalidated ( including a system reset). When this bit is 0, the
    cache is effectively in bypass mode (data fetches will come from main memory
    or from the line fill buffer). Set by hardware when the invalidate operation
    is complete and the cache is ready. */
    return MXC_SRCC_RevA_Ready((mxc_srcc_reva_regs_t *)MXC_EMCC);
}

/* ************************************************************************** */
void MXC_EMCC_Invalidate_All(void)
{
    /* Invalidate All Cache Contents. Any time this register location is written
    (regardless of the data value), the cache controller immediately begins
    invalidating the entire contents of the cache memory. The cache will be in
    bypass mode until the invalidate operation is complete. System software can
    examine the Cache Ready bit (CACHE_CTRL.CACHE_RDY) to determine when the
    invalidate operation is complete. Note that it is not necessary to disable
    the cache controller prior to beginning this operation. Reads from this
    register always return 0. */
    MXC_EMCC->invalidate |= MXC_F_EMCC_INVALIDATE_IA;
}
