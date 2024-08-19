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

#include "srcc_reva.h"

#if TARGET_NUM != 32650
#include "srcc.h"
uint32_t MXC_SRCC_RevA_ID(mxc_srcc_reva_regs_t *srcc, mxc_srcc_cache_id_t id)
{
    switch (id) {
    case SRCC_CACHE_ID_RELNUM:
        return (((srcc->cache_id) & MXC_F_SRCC_REVA_CACHE_ID_RELNUM)) >>
               MXC_F_SRCC_REVA_CACHE_ID_RELNUM_POS;

    case SRCC_CACHE_ID_PARTNUM:
        return (((srcc->cache_id) & MXC_F_SRCC_REVA_CACHE_ID_PARTNUM)) >>
               MXC_F_SRCC_REVA_CACHE_ID_PARTNUM_POS;

    case SRCC_CACHE_ID_CCHID:
    default:
        return (((srcc->cache_id) & MXC_F_SRCC_REVA_CACHE_ID_CCHID)) >>
               MXC_F_SRCC_REVA_CACHE_ID_CCHID_POS;
    }
}
#endif

uint32_t MXC_SRCC_RevA_CacheSize(mxc_srcc_reva_regs_t *srcc)
{
    return (((srcc->memcfg) & MXC_F_SRCC_REVA_MEMCFG_CCHSZ)) >> MXC_F_SRCC_REVA_MEMCFG_CCHSZ_POS;
}

uint32_t MXC_SRCC_RevA_MemSize(mxc_srcc_reva_regs_t *srcc)
{
    return (srcc->memcfg & MXC_F_SRCC_REVA_MEMCFG_MEMSZ) >> MXC_F_SRCC_REVA_MEMCFG_MEMSZ_POS;
}

void MXC_SRCC_RevA_Enable(mxc_srcc_reva_regs_t *srcc)
{
    srcc->cache_ctrl |= MXC_F_SRCC_REVA_CACHE_CTRL_CACHE_EN;
}

void MXC_SRCC_RevA_Disable(mxc_srcc_reva_regs_t *srcc)
{
    srcc->cache_ctrl &= ~MXC_F_SRCC_REVA_CACHE_CTRL_CACHE_EN;
}

void MXC_SRCC_RevA_WriteAllocateEnable(mxc_srcc_reva_regs_t *srcc)
{
    srcc->cache_ctrl |= MXC_F_SRCC_REVA_CACHE_CTRL_WRITE_ALLOC_EN;
}

void MXC_SRCC_RevA_WriteAllocateDisable(mxc_srcc_reva_regs_t *srcc)
{
    srcc->cache_ctrl &= ~MXC_F_SRCC_REVA_CACHE_CTRL_WRITE_ALLOC_EN;
}

void MXC_SRCC_RevA_CriticalWordFirstEnable(mxc_srcc_reva_regs_t *srcc) //cwfst_dis
{
    srcc->cache_ctrl &= ~MXC_F_SRCC_REVA_CACHE_CTRL_CWFST_DIS;
}

void MXC_SRCC_RevA_CriticalWordFirstDisable(mxc_srcc_reva_regs_t *srcc) //cwfst_dis
{
    srcc->cache_ctrl |= MXC_F_SRCC_REVA_CACHE_CTRL_CWFST_DIS;
}

uint32_t MXC_SRCC_RevA_Ready(mxc_srcc_reva_regs_t *srcc)
{
    return (srcc->cache_ctrl & MXC_F_SRCC_REVA_CACHE_CTRL_CACHE_RDY) >>
           MXC_F_SRCC_REVA_CACHE_CTRL_CACHE_RDY_POS;
}
