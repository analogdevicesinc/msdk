/**
 * @file    hpb.h
 * @brief   HyperBus (HPB) function prototypes and data types.
 */

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
void MXC_HPB_RevA_RegWrite8(mxc_hpb_reva_regs_t *hpb, const mxc_hpb_cfg_reg_val_t *cfg_reg_val,
                            uint32_t base_addr, unsigned int index);
void MXC_HPB_RevA_RegRead16(mxc_hpb_reva_regs_t *hpb, mxc_hpb_cfg_reg_val_t *cfg_reg_val,
                            uint32_t base_addr, unsigned int index);
void MXC_HPB_RevA_RegWrite16(mxc_hpb_reva_regs_t *hpb, const mxc_hpb_cfg_reg_val_t *cfg_reg_val,
                             uint32_t base_addr, unsigned int index);
int MXC_HPB_RevA_Init(mxc_hpb_reva_regs_t *hpb, const mxc_hpb_mem_config_t *mem0,
                      const mxc_hpb_mem_config_t *mem1);
uint32_t MXC_HPB_RevA_GetStatus(mxc_hpb_reva_regs_t *hpb);
void MXC_HPB_RevA_EnableInt(mxc_hpb_reva_regs_t *hpb, unsigned polarity);
unsigned MXC_HPB_RevA_GetFlag(mxc_hpb_reva_regs_t *hpb);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_HPB_HPB_REVA_H_
