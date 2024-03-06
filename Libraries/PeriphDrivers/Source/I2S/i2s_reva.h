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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_I2S_I2S_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_I2S_I2S_REVA_H_

/* **** Includes **** */
#include <stdint.h>
#include "i2s.h"
#include "i2s_reva_regs.h"

/****** Definitions *****/

/* **** Functions **** */
int MXC_I2S_RevA_Init(mxc_i2s_reva_regs_t *i2s, mxc_i2s_req_t *req);

int MXC_I2S_RevA_Shutdown(mxc_i2s_reva_regs_t *i2s);

int MXC_I2S_RevA_ConfigData(mxc_i2s_reva_regs_t *i2s, mxc_i2s_req_t *req);

void MXC_I2S_RevA_TXEnable(mxc_i2s_reva_regs_t *i2s);

void MXC_I2S_RevA_TXDisable(mxc_i2s_reva_regs_t *i2s);

void MXC_I2S_RevA_RXEnable(mxc_i2s_reva_regs_t *i2s);

void MXC_I2S_RevA_RXDisable(mxc_i2s_reva_regs_t *i2s);

int MXC_I2S_RevA_SetRXThreshold(mxc_i2s_reva_regs_t *i2s, uint8_t threshold);

int MXC_I2S_RevA_SetFrequency(mxc_i2s_reva_regs_t *i2s, mxc_i2s_ch_mode_t mode, uint16_t clkdiv);

int MXC_I2S_RevA_SetSampleRate(mxc_i2s_reva_regs_t *i2s, uint32_t smpl_rate,
                               mxc_i2s_wsize_t smpl_sz, uint32_t src_clk);

int MXC_I2S_RevA_GetSampleRate(mxc_i2s_reva_regs_t *i2s, uint32_t src_clk);

int MXC_I2S_RevA_CalculateClockDiv(mxc_i2s_reva_regs_t *i2s, uint32_t smpl_rate,
                                   mxc_i2s_wsize_t word_sz, uint32_t src_clk);

void MXC_I2S_RevA_Flush(mxc_i2s_reva_regs_t *i2s);

int MXC_I2S_RevA_FillTXFIFO(mxc_i2s_reva_regs_t *i2s, void *txData, mxc_i2s_wsize_t wordSize,
                            int len, int smpl_cnt);

int MXC_I2S_RevA_ReadRXFIFO(mxc_i2s_reva_regs_t *i2s, void *rxData, mxc_i2s_wsize_t wordSize,
                            int len, int smpl_cnt);

void MXC_I2S_RevA_EnableInt(mxc_i2s_reva_regs_t *i2s, uint32_t flags);

void MXC_I2S_RevA_DisableInt(mxc_i2s_reva_regs_t *i2s, uint32_t flags);

int MXC_I2S_RevA_GetFlags(mxc_i2s_reva_regs_t *i2s);

void MXC_I2S_RevA_ClearFlags(mxc_i2s_reva_regs_t *i2s, uint32_t flags);

int MXC_I2S_RevA_Transaction(mxc_i2s_reva_regs_t *i2s, mxc_i2s_req_t *i2s_req);

int MXC_I2S_RevA_TransactionAsync(mxc_i2s_reva_regs_t *i2s, mxc_i2s_req_t *i2s_req);

int MXC_I2S_RevA_TXDMAConfig(mxc_i2s_reva_regs_t *i2s, void *src_addr, int len);

int MXC_I2S_RevA_RXDMAConfig(mxc_i2s_reva_regs_t *i2s, void *dest_addr, int len);

void MXC_I2S_RevA_Handler(mxc_i2s_reva_regs_t *i2s);

void MXC_I2S_RevA_RegisterDMACallback(void (*callback)(int, int));

void MXC_I2S_RevA_RegisterAsyncCallback(void (*callback)(int));

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_I2S_I2S_REVA_H_
