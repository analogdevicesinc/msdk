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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_SPIMSS_I2S_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_SPIMSS_I2S_REVA_H_

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include "i2s.h"
#include "spimss_reva_regs.h"

int MXC_I2S_RevA_Init(mxc_spimss_reva_regs_t *spimss, const mxc_i2s_config_t *req,
                      void (*dma_ctz_cb)(int, int));
int MXC_I2S_RevA_Shutdown(mxc_spimss_reva_regs_t *spimss);
int MXC_I2S_RevA_Mute(mxc_spimss_reva_regs_t *spimss);
int MXC_I2S_RevA_Unmute(mxc_spimss_reva_regs_t *spimss);
int MXC_I2S_RevA_Pause(mxc_spimss_reva_regs_t *spimss);
int MXC_I2S_RevA_Unpause(mxc_spimss_reva_regs_t *spimss);
int MXC_I2S_RevA_Stop(mxc_spimss_reva_regs_t *spimss);
int MXC_I2S_RevA_Start(mxc_spimss_reva_regs_t *spimss);
int MXC_I2S_RevA_DMA_ClearFlags(void);
int MXC_I2S_RevA_DMA_SetAddrCnt(void *src_addr, void *dst_addr, unsigned int count);
int MXC_I2S_RevA_DMA_SetReload(void *src_addr, void *dst_addr, unsigned int count);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_SPIMSS_I2S_REVA_H_
