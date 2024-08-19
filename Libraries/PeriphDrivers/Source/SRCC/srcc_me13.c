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

#include "srcc.h"
#include "srcc_reva.h"
#include "gcr_regs.h"
#include "mxc_device.h"

uint32_t MXC_SRCC_ID(mxc_srcc_cache_id_t id)
{
    return MXC_SRCC_RevA_ID((mxc_srcc_reva_regs_t *)MXC_SRCC, id);
}

uint32_t MXC_SRCC_CacheSize(void)
{
    return MXC_SRCC_RevA_CacheSize((mxc_srcc_reva_regs_t *)MXC_SRCC);
}

uint32_t MXC_SRCC_MemSize(void)
{
    return MXC_SRCC_RevA_MemSize((mxc_srcc_reva_regs_t *)MXC_SRCC);
}

void MXC_SRCC_Enable(void)
{
    MXC_GCR->sysctrl &= ~MXC_F_GCR_SYSCTRL_SRCC_DIS;
    MXC_SRCC_RevA_Enable((mxc_srcc_reva_regs_t *)MXC_SRCC);
}

void MXC_SRCC_Disable(void)
{
    MXC_SRCC_RevA_Disable((mxc_srcc_reva_regs_t *)MXC_SRCC);
    MXC_GCR->sysctrl |= MXC_F_GCR_SYSCTRL_SRCC_DIS;
}

void MXC_SRCC_Flush(void)
{
    MXC_GCR->sysctrl |= MXC_F_GCR_SYSCTRL_ICC0_FLUSH;
}
void MXC_SRCC_WriteAllocateEnable(void)
{
    MXC_SRCC_RevA_WriteAllocateEnable((mxc_srcc_reva_regs_t *)MXC_SRCC);
}

void MXC_SRCC_WriteAllocateDisable(void)
{
    MXC_SRCC_RevA_WriteAllocateDisable((mxc_srcc_reva_regs_t *)MXC_SRCC);
}

void MXC_SRCC_CriticalWordFirstEnable(void) //cwfst_dis
{
    MXC_SRCC_RevA_CriticalWordFirstEnable((mxc_srcc_reva_regs_t *)MXC_SRCC);
}

void MXC_SRCC_CriticalWordFirstDisable(void) //cwfst_dis
{
    MXC_SRCC_RevA_CriticalWordFirstDisable((mxc_srcc_reva_regs_t *)MXC_SRCC);
}

uint32_t MXC_SRCC_Ready(void)
{
    return MXC_SRCC_RevA_Ready((mxc_srcc_reva_regs_t *)MXC_SRCC);
}
