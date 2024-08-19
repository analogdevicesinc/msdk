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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_SDHC_SDHC_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_SDHC_SDHC_REVA_H_

/* **** Includes **** */
#include <string.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "sdhc_reva_regs.h"

/* **** Definitions **** */

/* **** Globals **** */

/* **** Functions **** */
void MXC_SDHC_RevA_Set_Clock_Config(mxc_sdhc_reva_regs_t *sdhc, unsigned int clk_div);
unsigned int MXC_SDHC_RevA_Get_Clock_Config(mxc_sdhc_reva_regs_t *sdhc);
int MXC_SDHC_RevA_Init(mxc_sdhc_reva_regs_t *sdhc, const mxc_sdhc_cfg_t *cfg);
void MXC_SDHC_RevA_PowerUp(mxc_sdhc_reva_regs_t *sdhc);
void MXC_SDHC_RevA_PowerDown(mxc_sdhc_reva_regs_t *sdhc);
int MXC_SDHC_RevA_Shutdown(mxc_sdhc_reva_regs_t *sdhc);
int MXC_SDHC_RevA_SendCommand(mxc_sdhc_reva_regs_t *sdhc, mxc_sdhc_cmd_cfg_t *sd_cmd_cfg);
int MXC_SDHC_RevA_SendCommandAsync(mxc_sdhc_reva_regs_t *sdhc, mxc_sdhc_cmd_cfg_t *sd_cmd_cfg);
void MXC_SDHC_RevA_Handler(mxc_sdhc_reva_regs_t *sdhc);
void MXC_SDHC_RevA_ClearFlags(mxc_sdhc_reva_regs_t *sdhc, uint32_t mask);
unsigned MXC_SDHC_RevA_GetFlags(mxc_sdhc_reva_regs_t *sdhc);
int MXC_SDHC_RevA_Card_Inserted(mxc_sdhc_reva_regs_t *sdhc);
void MXC_SDHC_RevA_Reset(mxc_sdhc_reva_regs_t *sdhc);
void MXC_SDHC_RevA_Reset_CMD_DAT(mxc_sdhc_reva_regs_t *sdhc);
int MXC_SDHC_RevA_Card_Busy(mxc_sdhc_reva_regs_t *sdhc);
unsigned int MXC_SDHC_RevA_Get_Host_Cn_1(mxc_sdhc_reva_regs_t *sdhc);
uint32_t MXC_SDHC_RevA_Get_Response32(mxc_sdhc_reva_regs_t *sdhc);
uint32_t MXC_SDHC_RevA_Get_Response32_Auto(mxc_sdhc_reva_regs_t *sdhc);
void MXC_SDHC_RevA_Get_Response128(mxc_sdhc_reva_regs_t *sdhc, unsigned char *response);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_SDHC_SDHC_REVA_H_
