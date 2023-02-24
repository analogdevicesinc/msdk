/**
 * @file    hpb.h
 * @brief   HyperBus (HPB) function prototypes and data types.
 */

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

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_HPB_HPB_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_HPB_HPB_REVA_H_

/* **** Includes **** */
#include "hpb_reva_regs.h"
#include "hpb.h"
#include "mxc_sys.h"

#ifdef __cplusplus
extern "C" {
#endif

/* **** Definitions **** */
typedef enum {
    MXC_HPB_REVA_DEV_HYPER_FLASH = MXC_V_HPB_REVA_MEMCTRL_DEVTYPE_HYPERFLASH,
    MXC_HPB_REVA_DEV_XCCELA_PSRAM = MXC_V_HPB_REVA_MEMCTRL_DEVTYPE_XCCELA_PSRAM,
    MXC_HPB_REVA_DEV_HYPER_RAM = MXC_V_HPB_REVA_MEMCTRL_DEVTYPE_HYPERRAM,
} mxc_hpb_reva_device_t;

/* **** Function Prototypes **** */
void MXC_HPB_RevA_RegRead8(mxc_hpb_reva_regs_t *hpb, mxc_hpb_cfg_reg_val_t *cfg_reg_val,
                           uint32_t base_addr, unsigned int index);
void MXC_HPB_RevA_RegWrite8(mxc_hpb_reva_regs_t *hpb, mxc_hpb_cfg_reg_val_t *cfg_reg_val,
                            uint32_t base_addr, unsigned int index);
void MXC_HPB_RevA_RegRead16(mxc_hpb_reva_regs_t *hpb, mxc_hpb_cfg_reg_val_t *cfg_reg_val,
                            uint32_t base_addr, unsigned int index);
void MXC_HPB_RevA_RegWrite16(mxc_hpb_reva_regs_t *hpb, mxc_hpb_cfg_reg_val_t *cfg_reg_val,
                             uint32_t base_addr, unsigned int index);
int MXC_HPB_RevA_Init(mxc_hpb_reva_regs_t *hpb, mxc_hpb_mem_config_t *mem0,
                      mxc_hpb_mem_config_t *mem1);
uint32_t MXC_HPB_RevA_GetStatus(mxc_hpb_reva_regs_t *hpb);
void MXC_HPB_RevA_EnableInt(mxc_hpb_reva_regs_t *hpb, unsigned polarity);
unsigned MXC_HPB_RevA_GetFlag(mxc_hpb_reva_regs_t *hpb);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_HPB_HPB_REVA_H_
