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
#include <string.h>
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "clcd_reva.h"
#include "clcd.h"
#include "mxc_lock.h"

/* **** Definitions **** */

/* **** Globals **** */

/* ************************************************************************** */
int MXC_CLCD_RevA_Init(mxc_clcd_reva_regs_t *clcd, mxc_clcd_cfg_t *cfg)
{
    int error;

    // Clear registers
    clcd->ctrl = 0;
    clcd->vtim_0 = 0;
    clcd->vtim_1 = 0;
    clcd->clk = 0;
    clcd->htim = 0;
    if ((error = MXC_CLCD_ConfigPanel(cfg)) != E_NO_ERROR) {
        return error;
    }
    // Disable and clear interrupts
    clcd->int_en = 0;
    clcd->stat = clcd->stat;

    return E_NO_ERROR;
}

/* ************************************************************************* */
int MXC_CLCD_RevA_Shutdown(mxc_clcd_reva_regs_t *clcd)
{
    // Disable and clear interrupts
    clcd->int_en = 0;
    clcd->stat = clcd->stat;

    // Clear registers
    clcd->ctrl = 0;
    clcd->vtim_0 = 0;
    clcd->vtim_1 = 0;
    clcd->clk = 0;
    clcd->htim = 0;

    return E_NO_ERROR;
}

/* ************************************************************************* */
int MXC_CLCD_RevA_ConfigPanel(mxc_clcd_reva_regs_t *clcd, mxc_clcd_cfg_t *cfg)
{
    int i;

    clcd->clk = ((MXC_S_CLCD_REVA_CLK_EDGE_RISEEDGE) | (MXC_S_CLCD_REVA_CLK_HPOL_ACTIVEHI) |
                 (MXC_F_CLCD_REVA_CLK_VPOL) | (MXC_S_CLCD_REVA_CLK_DPOL_ACTIVEHI) |
                 ((PeripheralClock / cfg->frequency - 1) << MXC_F_CLCD_REVA_CLK_CLKDIV_POS));

    clcd->vtim_0 = (cfg->vbackporch << MXC_F_CLCD_REVA_VTIM_0_VBACKPORCH_POS) | (cfg->height - 1);
    clcd->vtim_1 = (cfg->vfrontporch << MXC_F_CLCD_REVA_VTIM_1_VFRONTPORCH_POS) |
                   ((cfg->vsyncwidth - 1) << MXC_F_CLCD_REVA_VTIM_1_VSYNCWIDTH_POS);

    clcd->htim = ((cfg->hfrontporch - 1) << MXC_F_CLCD_REVA_HTIM_HFRONTPORCH_POS) |
                 ((cfg->hbackporch - 1) << MXC_F_CLCD_REVA_HTIM_HBACKPORCH_POS) |
                 ((cfg->hsyncwidth - 1) << MXC_F_CLCD_REVA_HTIM_HSYNCWIDTH_POS) |
                 (((cfg->width >> 4) - 1) << MXC_F_CLCD_REVA_HTIM_HSIZE_POS);

    clcd->ctrl |= cfg->bpp;

    for (i = 0; i < cfg->paletteSize; i++) {
        clcd->palette[i] = cfg->palette[i];
    }
    return E_NO_ERROR;
}

/* ************************************************************************* */
int MXC_CLCD_RevA_Enable(mxc_clcd_reva_regs_t *clcd)
{
    clcd->ctrl |= MXC_S_CLCD_REVA_CTRL_LCDEN_ENABLE | (8 << MXC_F_CLCD_REVA_CTRL_DISPTYPE_POS) |
                  MXC_F_CLCD_REVA_CTRL_PEN;

    return E_NO_ERROR;
}

/* ************************************************************************* */
int MXC_CLCD_RevA_Disable(mxc_clcd_reva_regs_t *clcd)
{
    clcd->ctrl &= (~MXC_S_CLCD_REVA_CTRL_LCDEN_ENABLE);

    return E_NO_ERROR;
}

/* ************************************************************************* */
int MXC_CLCD_RevA_SetFrameAddr(mxc_clcd_reva_regs_t *clcd, void *addr)
{
    clcd->ctrl &= (~MXC_S_CLCD_REVA_CTRL_LCDEN_ENABLE);
    clcd->fr = (uint32_t)addr;
    clcd->ctrl |= (MXC_S_CLCD_REVA_CTRL_LCDEN_ENABLE);

    return E_NO_ERROR;
}
