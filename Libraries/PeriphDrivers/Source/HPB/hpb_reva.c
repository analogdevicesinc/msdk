/* *****************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
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
 * $Date: 2017-05-15 09:31:32 -0500 (Mon, 15 May 2017) $
 * $Revision: 27976 $
 *
 **************************************************************************** */

/* **** Includes **** */
#include <string.h>
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "hpb_reva.h"
#include "emcc.h"
#include "mxc_sys.h"

/* **** Definitions **** */

/* **** Globals **** */

/* **** Functions **** */

/* ************************************************************************** */
static void MXC_HPB_RevA_ConfigMem(mxc_hpb_reva_regs_t *hpb, mxc_hpb_mem_config_t *mem,
                                   unsigned index)
{
    int i;

    // Set the base address
    hpb->membaddr[index] = mem->base_addr;

    // Set the device type and fixed latency mode (Xccela only)
    hpb->memctrl[index] = 0x3 | (mem->device_type << MXC_F_HPB_REVA_MEMCTRL_DEVTYPE_POS) |
                          ((!!(mem->fixed_latency)) << MXC_F_HPB_REVA_MEMCTRL_RDLAT_EN_POS);

    // Setup memory timings
    hpb->memtim[index] = (mem->read_cs_high << MXC_F_HPB_REVA_MEMTIM_RDCSHI_POS) |
                         (mem->write_cs_high << MXC_F_HPB_REVA_MEMTIM_WRCSHI_POS) |
                         (mem->read_cs_setup << MXC_F_HPB_REVA_MEMTIM_RDCSST_POS) |
                         (mem->write_cs_setup << MXC_F_HPB_REVA_MEMTIM_WRCSST_POS) |
                         (mem->read_cs_hold << MXC_F_HPB_REVA_MEMTIM_RDCSHD_POS) |
                         (mem->write_cs_hold << MXC_F_HPB_REVA_MEMTIM_WRCSHD_POS) |
                         (mem->latency_cycle << MXC_F_HPB_REVA_MEMTIM_LAT_POS);

    // Send configuration commands
    for (i = 0; i < mem->cfg_reg_val_len; i++) {
        if (mem->device_type == (mxc_hpb_device_t)MXC_HPB_REVA_DEV_XCCELA_PSRAM) {
            MXC_HPB_RegWrite8(&(mem->cfg_reg_val[i]), mem->base_addr, index);
        } else {
            MXC_HPB_RegWrite16(&(mem->cfg_reg_val[i]), mem->base_addr, index);
        }
    }
}

/* ************************************************************************** */
void MXC_HPB_RevA_RegRead8(mxc_hpb_reva_regs_t *hpb, mxc_hpb_cfg_reg_val_t *cfg_reg_val,
                           uint32_t base_addr, unsigned int index)
{
    if (!hpb || !cfg_reg_val || index > 1) {
        return;
    }

    MXC_EMCC_Disable();
    hpb->memctrl[index] |= MXC_F_HPB_REVA_MEMCTRL_CRT;
    cfg_reg_val->val = *((volatile unsigned char *)(base_addr + cfg_reg_val->addr));
    hpb->memctrl[index] &= ~(MXC_F_HPB_REVA_MEMCTRL_CRT);
    MXC_EMCC_Enable();
}

/* ************************************************************************** */
void MXC_HPB_RevA_RegWrite8(mxc_hpb_reva_regs_t *hpb, mxc_hpb_cfg_reg_val_t *cfg_reg_val,
                            uint32_t base_addr, unsigned int index)
{
    if (!hpb || !cfg_reg_val || index > 1) {
        return;
    }

    MXC_EMCC_Disable();
    hpb->memctrl[index] |= MXC_F_HPB_REVA_MEMCTRL_CRT;
    *((volatile unsigned char *)(base_addr + cfg_reg_val->addr)) = cfg_reg_val->val;
    hpb->memctrl[index] &= ~(MXC_F_HPB_REVA_MEMCTRL_CRT);
    MXC_EMCC_Enable();
}

/* ************************************************************************** */
void MXC_HPB_RevA_RegRead16(mxc_hpb_reva_regs_t *hpb, mxc_hpb_cfg_reg_val_t *cfg_reg_val,
                            uint32_t base_addr, unsigned int index)
{
    if (!hpb || !cfg_reg_val || index > 1) {
        return;
    }

    MXC_EMCC_Disable();
    hpb->memctrl[index] |= MXC_F_HPB_REVA_MEMCTRL_CRT;
    cfg_reg_val->val = *((volatile unsigned short *)(base_addr + cfg_reg_val->addr));
    hpb->memctrl[index] &= ~(MXC_F_HPB_REVA_MEMCTRL_CRT);
    MXC_EMCC_Enable();
}

/* ************************************************************************** */
void MXC_HPB_RevA_RegWrite16(mxc_hpb_reva_regs_t *hpb, mxc_hpb_cfg_reg_val_t *cfg_reg_val,
                             uint32_t base_addr, unsigned int index)
{
    if (!hpb || !cfg_reg_val || index > 1) {
        return;
    }

    MXC_EMCC_Disable();
    hpb->memctrl[index] |= MXC_F_HPB_REVA_MEMCTRL_CRT;
    *((volatile unsigned short *)(base_addr + cfg_reg_val->addr)) = cfg_reg_val->val;
    hpb->memctrl[index] &= ~(MXC_F_HPB_REVA_MEMCTRL_CRT);
    MXC_EMCC_Enable();
}

/* ************************************************************************** */
int MXC_HPB_RevA_Init(mxc_hpb_reva_regs_t *hpb, mxc_hpb_mem_config_t *mem0,
                      mxc_hpb_mem_config_t *mem1)
{
    if (mem0) {
        MXC_HPB_RevA_ConfigMem(hpb, mem0, 0);
    }
    if (mem1) {
        MXC_HPB_RevA_ConfigMem(hpb, mem1, 1);
    }

    return E_NO_ERROR;
}

/* ************************************************************************** */
uint32_t MXC_HPB_RevA_GetStatus(mxc_hpb_reva_regs_t *hpb)
{
    return hpb->stat;
}

/* ************************************************************************** */
void MXC_HPB_RevA_EnableInt(mxc_hpb_reva_regs_t *hpb, unsigned polarity)
{
    if (polarity) {
        hpb->inten |= MXC_F_HPB_REVA_INTEN_ERR;
    } else {
        hpb->inten &= ~(MXC_F_HPB_REVA_INTEN_ERR);
    }
}

/* ************************************************************************** */
unsigned MXC_HPB_RevA_GetFlag(mxc_hpb_reva_regs_t *hpb)
{
    return hpb->intfl;
}
