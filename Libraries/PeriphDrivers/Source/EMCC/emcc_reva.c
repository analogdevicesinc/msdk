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
