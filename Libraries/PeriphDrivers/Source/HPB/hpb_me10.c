/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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
void MXC_HPB_RegWrite8(mxc_hpb_cfg_reg_val_t *cfg_reg_val, uint32_t base_addr, unsigned int index)
{
    MXC_HPB_RevA_RegWrite8((mxc_hpb_reva_regs_t *)MXC_HPB, cfg_reg_val, base_addr, index);
}

/* ************************************************************************** */
void MXC_HPB_RegRead16(mxc_hpb_cfg_reg_val_t *cfg_reg_val, uint32_t base_addr, unsigned int index)
{
    MXC_HPB_RevA_RegRead16((mxc_hpb_reva_regs_t *)MXC_HPB, cfg_reg_val, base_addr, index);
}

/* ************************************************************************** */
void MXC_HPB_RegWrite16(mxc_hpb_cfg_reg_val_t *cfg_reg_val, uint32_t base_addr, unsigned int index)
{
    MXC_HPB_RevA_RegWrite16((mxc_hpb_reva_regs_t *)MXC_HPB, cfg_reg_val, base_addr, index);
}

/* ************************************************************************** */
int MXC_HPB_Init(mxc_hpb_mem_config_t *mem0, mxc_hpb_mem_config_t *mem1)
{
    /* Enable HyperBus Clocks */
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_HBC);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SCACHE);

    /* Select drive strength on CK pin */
    MXC_BBFC->bbfcr0 = (2 << MXC_F_BBFC_BBFCR0_CKPDRV_POS) | (MXC_S_BBFC_BBFCR0_RDSDLLEN_EN);

    /* If Hyperbus, also select drive strength on NCK pin and enable RDS DLL */
    if ((mem0 && (mem0->device_type != MXC_HPB_DEV_XCCELA_PSRAM)) ||
        (mem1 && (mem1->device_type != MXC_HPB_DEV_XCCELA_PSRAM))) {
        MXC_BBFC->bbfcr0 |= (2 << MXC_F_BBFC_BBFCR0_CKNPDRV_POS);
    }

    /* Configure HyperBus GPIO Pins */
    if (mem0) {
        MXC_GPIO_Config(&gpio_cfg_hyp_cs0);
    }
    if (mem1) {
        MXC_GPIO_Config(&gpio_cfg_hyp_cs1);
    }
    MXC_GPIO_Config(&gpio_cfg_hyp);

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
