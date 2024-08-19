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
