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

#include "mxc_sys.h"
#include "mxc_pins.h"
#include "gpio.h"
#include "pt.h"
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

    switch (clk_scale) {
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

    MXC_PT_RevA_Init((mxc_ptg_reva_regs_t *)MXC_PTG, clk_scale);
}

void MXC_PT_Shutdown(uint32_t pts)
{
    if (MXC_PT_RevA_Shutdown((mxc_ptg_reva_regs_t *)MXC_PTG, pts)) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_PT);
    }
}

int MXC_PT_Config(mxc_pt_cfg_t *cfg)
{
    MXC_PT_RevA_Config((mxc_ptg_reva_regs_t *)MXC_PTG, cfg);

    switch (cfg->channel) {
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

    default:
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

int MXC_PT_SqrWaveConfig(unsigned channel, uint32_t freq)
{
    mxc_pt_cfg_t sqwcfg;

    MXC_PT_RevA_SqrWaveConfig((mxc_ptg_reva_regs_t *)MXC_PTG, &sqwcfg, channel, freq);
    return MXC_PT_Config(&sqwcfg);
}

void MXC_PT_Start(unsigned pts)
{
    MXC_PT_RevA_Start((mxc_ptg_reva_regs_t *)MXC_PTG, pts);
}

void MXC_PT_Stop(unsigned pts)
{
    MXC_PT_RevA_Stop((mxc_ptg_reva_regs_t *)MXC_PTG, pts);
}

uint32_t MXC_PT_IsActive(uint32_t pts)
{
    return MXC_PT_RevA_IsActive((mxc_ptg_reva_regs_t *)MXC_PTG, pts);
}

void MXC_PT_SetPattern(unsigned pts, uint32_t pattern)
{
    MXC_PT_RevA_SetPattern(pts, pattern);
}

void MXC_PT_EnableStopInt(uint32_t pts)
{
    MXC_PT_RevA_EnableStopInt((mxc_ptg_reva_regs_t *)MXC_PTG, pts);
}

void MXC_PT_DisableStopInt(uint32_t pts)
{
    MXC_PT_RevA_DisableStopInt((mxc_ptg_reva_regs_t *)MXC_PTG, pts);
}

uint32_t MXC_PT_GetStopFlags(void)
{
    return MXC_PT_RevA_GetStopFlags((mxc_ptg_reva_regs_t *)MXC_PTG);
}

void MXC_PT_ClearStopFlags(uint32_t flags)
{
    MXC_PT_RevA_ClearStopFlags((mxc_ptg_reva_regs_t *)MXC_PTG, flags);
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
    MXC_PT_RevA_Resync((mxc_ptg_reva_regs_t *)MXC_PTG, pts);
}

void MXC_PT_EnableReadyInt(uint32_t pts)
{
    MXC_PT_RevA_EnableReadyInt((mxc_ptg_reva_regs_t *)MXC_PTG, pts);
}

void MXC_PT_DisableReadyInt(uint32_t pts)
{
    MXC_PT_RevA_DisableReadyInt((mxc_ptg_reva_regs_t *)MXC_PTG, pts);
}

uint32_t MXC_PT_GetReadyFlags(void)
{
    return MXC_PT_RevA_GetReadyFlags((mxc_ptg_reva_regs_t *)MXC_PTG);
}

void MXC_PT_ClearReadyFlags(uint32_t flags)
{
    MXC_PT_RevA_ClearReadyFlags((mxc_ptg_reva_regs_t *)MXC_PTG, flags);
}
