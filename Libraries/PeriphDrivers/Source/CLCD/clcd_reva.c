/* *****************************************************************************
 * Copyright (C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
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
 * $Date: 2017-06-08 11:25:01 -0500 (Thu, 08 Jun 2017) $
 * $Revision: 28447 $
 *
 **************************************************************************** */

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
int MXC_CLCD_RevA_Init(mxc_clcd_reva_regs_t* clcd, mxc_clcd_cfg_t* cfg)
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
int MXC_CLCD_RevA_Shutdown(mxc_clcd_reva_regs_t* clcd)
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
int MXC_CLCD_RevA_ConfigPanel(mxc_clcd_reva_regs_t* clcd, mxc_clcd_cfg_t* cfg)
{

    int i;
    
    clcd->clk = ((MXC_S_CLCD_REVA_CLK_EDGE_RISEEDGE) | (MXC_S_CLCD_REVA_CLK_HPOL_ACTIVEHI)
                     | (MXC_F_CLCD_REVA_CLK_VPOL) | (MXC_S_CLCD_REVA_CLK_DPOL_ACTIVEHI)
                     | ((PeripheralClock / cfg->frequency - 1) << MXC_F_CLCD_REVA_CLK_CLKDIV_POS));
                     
    clcd->vtim_0 = (cfg->vbackporch << MXC_F_CLCD_REVA_VTIM_0_VBACKPORCH_POS) |
                       (cfg->height - 1);
    clcd->vtim_1 = (cfg->vfrontporch << MXC_F_CLCD_REVA_VTIM_1_VFRONTPORCH_POS) | ((
                           cfg->vsyncwidth - 1) << MXC_F_CLCD_REVA_VTIM_1_VSYNCWIDTH_POS);
                           
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
int MXC_CLCD_RevA_Enable(mxc_clcd_reva_regs_t* clcd)
{

    clcd->ctrl |= MXC_S_CLCD_REVA_CTRL_LCDEN_ENABLE | (8 << MXC_F_CLCD_REVA_CTRL_DISPTYPE_POS) |
                      MXC_F_CLCD_REVA_CTRL_PEN;
                      
    return E_NO_ERROR;
}

/* ************************************************************************* */
int MXC_CLCD_RevA_Disable(mxc_clcd_reva_regs_t* clcd)
{

    clcd->ctrl &= (~MXC_S_CLCD_REVA_CTRL_LCDEN_ENABLE) ;
    
    return E_NO_ERROR;
}

/* ************************************************************************* */
int MXC_CLCD_RevA_SetFrameAddr(mxc_clcd_reva_regs_t* clcd, void* addr)
{

    clcd->ctrl &= (~MXC_S_CLCD_REVA_CTRL_LCDEN_ENABLE) ;
    clcd->fr = (uint32_t)addr;
    clcd->ctrl |= (MXC_S_CLCD_REVA_CTRL_LCDEN_ENABLE) ;
    
    return E_NO_ERROR;
}
