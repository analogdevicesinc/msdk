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
