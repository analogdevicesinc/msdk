/* ****************************************************************************
 * Copyright(C) 2019 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files(the "Software"),
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
 *************************************************************************** */

#include "pt.h"
#include "adc.h"
#include "gcr_regs.h"
#include "pt_regs.h"
#include "ptg_regs.h"
#include "pt_reva.h"

void MXC_PT_Init(mxc_clk_scale_t clk_scale)
{
    MXC_ASSERT(clk_scale <= 128);
    
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_PT);
    MXC_SYS_Reset_Periph(MXC_SYS_RESET1_PT);
    
    //set clock scale
    MXC_GCR->clkctrl &= ~MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV128;
    
    switch(clk_scale) {
    case MXC_PT_CLK_DIV1:
        MXC_GCR->clkctrl |= MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV1;
        break;
        
    case MXC_PT_CLK_DIV2:
        MXC_GCR->clkctrl |= MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV2;
        break;
        
    case MXC_PT_CLK_DIV4:
        MXC_GCR->clkctrl |= MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV4;
        break;
        
    case MXC_PT_CLK_DIV8:
        MXC_GCR->clkctrl |= MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV8;
        break;
        
    case MXC_PT_CLK_DIV16:
        MXC_GCR->clkctrl |= MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV16;
        break;
        
    case MXC_PT_CLK_DIV32:
        MXC_GCR->clkctrl |= MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV32;
        break;
        
    case MXC_PT_CLK_DIV64:
        MXC_GCR->clkctrl |= MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV64;
        break;
        
    case MXC_PT_CLK_DIV128:
        MXC_GCR->clkctrl |= MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV128;
        break;
    }
    
    MXC_PT_RevA_Init((mxc_ptg_reva_regs_t*) MXC_PTG, clk_scale);
}

void MXC_PT_Shutdown(uint32_t pts)
{
    if (MXC_PT_RevA_Shutdown((mxc_ptg_reva_regs_t*) MXC_PTG, pts)) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_PT);
    }
}

int MXC_PT_Config(mxc_pt_cfg_t* cfg)
{
    MXC_PT_RevA_Config((mxc_ptg_reva_regs_t*) MXC_PTG, cfg);
    
    switch(cfg->channel) {
    case 0:
        MXC_GPIO_Config(&gpio_cfg_pt0);
        break;
        
    case 1:
        MXC_GPIO_Config(&gpio_cfg_pt1);
        break;
        
    case 2:
        MXC_GPIO_Config(&gpio_cfg_pt2);
        break;
        
    case 3:
        MXC_GPIO_Config(&gpio_cfg_pt3);
        break;
        
    case 4:
        MXC_GPIO_Config(&gpio_cfg_pt4);
        break;
        
    case 5:
        MXC_GPIO_Config(&gpio_cfg_pt5);
        break;
        
    case 6:
        MXC_GPIO_Config(&gpio_cfg_pt6);
        break;
        
    case 7:
        MXC_GPIO_Config(&gpio_cfg_pt7);
        break;
        
    default:
        return E_BAD_PARAM;
        break;
    }
    
    return E_NO_ERROR;
}

int MXC_PT_SqrWaveConfig(unsigned channel, uint32_t freq)
{
    mxc_pt_cfg_t sqwcfg;
    
    MXC_PT_RevA_SqrWaveConfig((mxc_ptg_reva_regs_t*) MXC_PTG, &sqwcfg, channel, freq);
    return MXC_PT_Config(&sqwcfg);
}

void MXC_PT_Start(unsigned pts)
{
    MXC_PT_RevA_Start((mxc_ptg_reva_regs_t*) MXC_PTG, pts);
}

void MXC_PT_Stop(unsigned pts)
{
    MXC_PT_RevA_Stop((mxc_ptg_reva_regs_t*) MXC_PTG, pts);
}

uint32_t MXC_PT_IsActive(uint32_t pts)
{
    return MXC_PT_RevA_IsActive((mxc_ptg_reva_regs_t*) MXC_PTG, pts);
}

void MXC_PT_SetPattern(unsigned pts, uint32_t pattern)
{
    MXC_PT_RevA_SetPattern(pts, pattern);
}

void MXC_PT_EnableInt(uint32_t pts)
{
    MXC_PT_RevA_EnableInt((mxc_ptg_reva_regs_t*) MXC_PTG, pts);
}

void MXC_PT_DisableInt(uint32_t pts)
{
    MXC_PT_RevA_DisableInt((mxc_ptg_reva_regs_t*) MXC_PTG, pts);
}

uint32_t MXC_PT_GetFlags(void)
{
    return MXC_PT_RevA_GetFlags((mxc_ptg_reva_regs_t*) MXC_PTG);
}

void MXC_PT_ClearFlags(uint32_t flags)
{
    MXC_PT_RevA_ClearFlags((mxc_ptg_reva_regs_t*) MXC_PTG, flags);
}

void MXC_PT_EnableRestart(unsigned start, unsigned stop, uint8_t restartIndex)
{
    MXC_PT_RevA_EnableRestart(start, stop, restartIndex);
}

void MXC_PT_DisableRestart(unsigned channel, uint8_t restartIndex)
{
    MXC_PT_RevA_DisableRestart(channel, restartIndex);
}

void MXC_PT_Resync(uint32_t pts)
{
    MXC_PT_RevA_Resync((mxc_ptg_reva_regs_t*) MXC_PTG, pts);
}
