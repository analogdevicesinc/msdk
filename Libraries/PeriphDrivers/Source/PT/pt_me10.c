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
#include <stddef.h>
#include "mxc_errors.h"
#include "mxc_sys.h"
#include "gpio.h"
#include "pt.h"
#include "pt_reva.h"

/* ************************************************************************* */
void MXC_PT_Init(mxc_ptg_regs_t *ptg, mxc_clk_scale_t clk_scale)
{
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_PT);
    MXC_SYS_Reset_Periph(MXC_SYS_RESET_PT);

    MXC_PT_RevA_Init((mxc_ptg_reva_regs_t *)ptg, clk_scale);
}

/* ************************************************************************* */
void MXC_PT_Shutdown(mxc_ptg_regs_t *ptg, uint32_t pts)
{
    MXC_PT_RevA_Shutdown((mxc_ptg_reva_regs_t *)ptg, pts);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_PT);
}

/* ************************************************************************* */
int MXC_PT_Config(mxc_ptg_regs_t *ptg, mxc_pt_cfg_t *cfg)
{
    if (cfg->outputSelect) {
        switch (cfg->channel) {
        case 0:
            MXC_GPIO_Config(&gpio_cfg_pt0_1);
            break;
        case 1:
            MXC_GPIO_Config(&gpio_cfg_pt1_1);
            break;
        case 2:
            MXC_GPIO_Config(&gpio_cfg_pt2_1);
            break;
        case 3:
            MXC_GPIO_Config(&gpio_cfg_pt3_1);
            break;
        case 4:
            MXC_GPIO_Config(&gpio_cfg_pt4_1);
            break;
        case 5:
            MXC_GPIO_Config(&gpio_cfg_pt5_1);
            break;
        case 6:
            MXC_GPIO_Config(&gpio_cfg_pt6_1);
            break;
        case 7:
            MXC_GPIO_Config(&gpio_cfg_pt7_1);
            break;
        case 8:
            MXC_GPIO_Config(&gpio_cfg_pt8_1);
            break;
        case 9:
            MXC_GPIO_Config(&gpio_cfg_pt9_1);
            break;
        case 10:
            MXC_GPIO_Config(&gpio_cfg_pt10_1);
            break;
        case 11:
            MXC_GPIO_Config(&gpio_cfg_pt11_1);
            break;
        case 12:
            MXC_GPIO_Config(&gpio_cfg_pt12_1);
            break;
        case 13:
            MXC_GPIO_Config(&gpio_cfg_pt13_1);
            break;
        case 14:
            MXC_GPIO_Config(&gpio_cfg_pt14_1);
            break;
        case 15:
            MXC_GPIO_Config(&gpio_cfg_pt15_1);
            break;
        default:
            return E_BAD_PARAM;
        }
    } else {
        switch (cfg->channel) {
        case 0:
            MXC_GPIO_Config(&gpio_cfg_pt0_0);
            break;
        case 1:
            MXC_GPIO_Config(&gpio_cfg_pt1_0);
            break;
        case 2:
            MXC_GPIO_Config(&gpio_cfg_pt2_0);
            break;
        case 3:
            MXC_GPIO_Config(&gpio_cfg_pt3_0);
            break;
        case 4:
            MXC_GPIO_Config(&gpio_cfg_pt4_0);
            break;
        case 5:
            MXC_GPIO_Config(&gpio_cfg_pt5_0);
            break;
        case 6:
            MXC_GPIO_Config(&gpio_cfg_pt6_0);
            break;
        case 7:
            MXC_GPIO_Config(&gpio_cfg_pt7_0);
            break;
        case 8:
            MXC_GPIO_Config(&gpio_cfg_pt8_0);
            break;
        case 9:
            MXC_GPIO_Config(&gpio_cfg_pt9_0);
            break;
        case 10:
            MXC_GPIO_Config(&gpio_cfg_pt10_0);
            break;
        case 11:
            MXC_GPIO_Config(&gpio_cfg_pt11_0);
            break;
        case 12:
            MXC_GPIO_Config(&gpio_cfg_pt12_0);
            break;
        case 13:
            MXC_GPIO_Config(&gpio_cfg_pt13_0);
            break;
        case 14:
            MXC_GPIO_Config(&gpio_cfg_pt14_0);
            break;
        case 15:
            MXC_GPIO_Config(&gpio_cfg_pt15_0);
            break;
        default:
            return E_BAD_PARAM;
        }
    }

    return MXC_PT_RevA_Config((mxc_ptg_reva_regs_t *)ptg, cfg);
}

/* ************************************************************************* */
int MXC_PT_SqrWaveConfig(mxc_ptg_regs_t *ptg, unsigned channel, uint32_t freq, uint8_t outputSelect)
{
    mxc_pt_cfg_t sqwcfg;
    sqwcfg.outputSelect = (!!outputSelect);

    MXC_PT_RevA_SqrWaveConfig((mxc_ptg_reva_regs_t *)ptg, &sqwcfg, channel, freq);
    return MXC_PT_Config(ptg, &sqwcfg);
}

/* ************************************************************************* */
void MXC_PT_Start(mxc_ptg_regs_t *ptg, unsigned pts)
{
    MXC_PT_RevA_Start((mxc_ptg_reva_regs_t *)ptg, pts);
}

/* ************************************************************************* */
void MXC_PT_Stop(mxc_ptg_regs_t *ptg, unsigned pts)
{
    MXC_PT_RevA_Stop((mxc_ptg_reva_regs_t *)ptg, pts);
}

/* ************************************************************************* */
uint32_t MXC_PT_IsActive(mxc_ptg_regs_t *ptg, uint32_t pts)
{
    return MXC_PT_RevA_IsActive((mxc_ptg_reva_regs_t *)ptg, pts);
}

/* ************************************************************************* */
void MXC_PT_SetPattern(unsigned pts, uint32_t pattern)
{
    MXC_PT_RevA_SetPattern(pts, pattern);
}

/* ************************************************************************* */
void MXC_PT_EnableInt(mxc_ptg_regs_t *ptg, uint32_t pts)
{
    MXC_PT_RevA_EnableInt((mxc_ptg_reva_regs_t *)ptg, pts);
}

/* ************************************************************************* */
void MXC_PT_DisableInt(mxc_ptg_regs_t *ptg, uint32_t pts)
{
    MXC_PT_RevA_DisableInt((mxc_ptg_reva_regs_t *)ptg, pts);
}

/* ************************************************************************* */
uint32_t MXC_PT_GetFlags(mxc_ptg_regs_t *ptg)
{
    return MXC_PT_RevA_GetFlags((mxc_ptg_reva_regs_t *)ptg);
}

/* ************************************************************************* */
void MXC_PT_ClearFlags(mxc_ptg_regs_t *ptg, uint32_t flags)
{
    MXC_PT_RevA_ClearFlags((mxc_ptg_reva_regs_t *)ptg, flags);
}

/* ************************************************************************* */
void MXC_PT_EnableRestart(unsigned start, unsigned stop, uint8_t restartIndex)
{
    MXC_PT_RevA_EnableRestart(start, stop, restartIndex);
}

/* ************************************************************************* */
void MXC_PT_DisableRestart(unsigned channel, uint8_t restartIndex)
{
    MXC_PT_RevA_DisableRestart(channel, restartIndex);
}

/* ************************************************************************* */
void MXC_PT_Resync(mxc_ptg_regs_t *ptg, uint32_t pts)
{
    MXC_PT_RevA_Resync((mxc_ptg_reva_regs_t *)ptg, pts);
}
