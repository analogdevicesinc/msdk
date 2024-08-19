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
#include "lpcmp_reva.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_delay.h"
#include "mxc_sys.h"
#include "mcr_regs.h"
#include "mxc_lock.h"

int MXC_LPCMP_RevA_Init(mxc_lpcmp_ctrl_reg_t ctrl_reg)
{
    *ctrl_reg |= MXC_F_LPCMP_REVA_CTRL_EN;
    return E_NO_ERROR;
}

int MXC_LPCMP_RevA_Shutdown(mxc_lpcmp_ctrl_reg_t ctrl_reg)
{
    *ctrl_reg &= ~MXC_F_LPCMP_REVA_CTRL_EN;
    return E_NO_ERROR;
}

int MXC_LPCMP_RevA_EnableInt(mxc_lpcmp_ctrl_reg_t ctrl_reg)
{
    *ctrl_reg |= MXC_F_LPCMP_REVA_CTRL_INT_EN;
    return E_NO_ERROR;
}

int MXC_LPCMP_RevA_DisableInt(mxc_lpcmp_ctrl_reg_t ctrl_reg)
{
    *ctrl_reg &= ~MXC_F_LPCMP_REVA_CTRL_INT_EN;
    return E_NO_ERROR;
}

int MXC_LPCMP_RevA_GetFlags(mxc_lpcmp_ctrl_reg_t ctrl_reg)
{
    return !!(*ctrl_reg & MXC_F_LPCMP_REVA_CTRL_INT_FL);
}

int MXC_LPCMP_RevA_ClearFlags(mxc_lpcmp_ctrl_reg_t ctrl_reg)
{
    *ctrl_reg |= MXC_F_LPCMP_REVA_CTRL_INT_FL;
    return E_NO_ERROR;
}

int MXC_LPCMP_RevA_SelectPolarity(mxc_lpcmp_ctrl_reg_t ctrl_reg, mxc_lpcmp_polarity_t pol)
{
    *ctrl_reg &= ~MXC_F_LPCMP_REVA_CTRL_POL;
    *ctrl_reg |= (pol << MXC_F_LPCMP_REVA_CTRL_POL_POS);
    return E_NO_ERROR;
}
