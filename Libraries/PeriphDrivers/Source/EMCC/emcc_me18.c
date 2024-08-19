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
