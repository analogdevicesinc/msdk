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
#include "mxc_pins.h"
#include "hpb.h"
#include "hpb_reva.h"
#include "bbfc_regs.h"

/* **** Definitions **** */

/* **** Globals **** */

/* **** Functions **** */

/* ************************************************************************** */
void MXC_HPB_RegRead8(mxc_hpb_cfg_reg_val_t *cfg_reg_val, uint32_t base_addr, unsigned int index)
{
    MXC_HPB_RevA_RegRead8((mxc_hpb_reva_regs_t *)MXC_HPB, cfg_reg_val, base_addr, index);
}

/* ************************************************************************** */
void MXC_HPB_RegWrite8(const mxc_hpb_cfg_reg_val_t *cfg_reg_val, uint32_t base_addr,
                       unsigned int index)
{
    MXC_HPB_RevA_RegWrite8((mxc_hpb_reva_regs_t *)MXC_HPB, cfg_reg_val, base_addr, index);
}

/* ************************************************************************** */
void MXC_HPB_RegRead16(mxc_hpb_cfg_reg_val_t *cfg_reg_val, uint32_t base_addr, unsigned int index)
{
    MXC_HPB_RevA_RegRead16((mxc_hpb_reva_regs_t *)MXC_HPB, cfg_reg_val, base_addr, index);
}

/* ************************************************************************** */
void MXC_HPB_RegWrite16(const mxc_hpb_cfg_reg_val_t *cfg_reg_val, uint32_t base_addr,
                        unsigned int index)
{
    MXC_HPB_RevA_RegWrite16((mxc_hpb_reva_regs_t *)MXC_HPB, cfg_reg_val, base_addr, index);
}

/* ************************************************************************** */
int MXC_HPB_Init(const mxc_hpb_mem_config_t *mem0, const mxc_hpb_mem_config_t *mem1)
{
#ifndef MSDK_NO_GPIO_CLK_INIT

    /* Enable HyperBus Clocks */
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_HBC);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SCACHE);

#endif // MSDK_NO_GPIO_CLK_INIT

    /* Select drive strength on CK pin */
    MXC_BBFC->bbfcr0 = (2 << MXC_F_BBFC_BBFCR0_CKPDRV_POS) | (MXC_S_BBFC_BBFCR0_RDSDLLEN_EN);

    /* If Hyperbus, also select drive strength on NCK pin and enable RDS DLL */
    if ((mem0 && (mem0->device_type != MXC_HPB_DEV_XCCELA_PSRAM)) ||
        (mem1 && (mem1->device_type != MXC_HPB_DEV_XCCELA_PSRAM))) {
        MXC_BBFC->bbfcr0 |= (2 << MXC_F_BBFC_BBFCR0_CKNPDRV_POS);
    }

#ifndef MSDK_NO_GPIO_CLK_INIT

    /* Configure HyperBus GPIO Pins */
    if (mem0) {
        MXC_GPIO_Config(&gpio_cfg_hyp_cs0);
    }
    if (mem1) {
        MXC_GPIO_Config(&gpio_cfg_hyp_cs1);
    }
    MXC_GPIO_Config(&gpio_cfg_hyp);

#endif // MSDK_NO_GPIO_CLK_INIT

    /* Reset the controller */
    MXC_SYS_Reset_Periph(MXC_SYS_RESET_HBC);

    return MXC_HPB_RevA_Init((mxc_hpb_reva_regs_t *)MXC_HPB, mem0, mem1);
}

/* ************************************************************************** */
uint32_t MXC_HPB_GetStatus(void)
{
    return MXC_HPB_RevA_GetStatus((mxc_hpb_reva_regs_t *)MXC_HPB);
}

/* ************************************************************************** */
void MXC_HPB_EnableInt(unsigned polarity)
{
    MXC_HPB_RevA_EnableInt((mxc_hpb_reva_regs_t *)MXC_HPB, polarity);
}

/* ************************************************************************** */
unsigned MXC_HPB_GetFlag(void)
{
    return MXC_HPB_RevA_GetFlag((mxc_hpb_reva_regs_t *)MXC_HPB);
}
