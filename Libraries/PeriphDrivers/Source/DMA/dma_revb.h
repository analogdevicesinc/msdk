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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_DMA_DMA_REVB_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_DMA_DMA_REVB_H_

/****** Includes *******/

/***** Definitions *****/

/******* Globals *******/

/****** Functions ******/
int MXC_DMA_RevB_Init(void);
int MXC_DMA_RevB_AcquireChannel(void);
int MXC_DMA_RevB_ReleaseChannel(int ch);
int MXC_DMA_RevB_ConfigChannel(mxc_dma_config_t config, mxc_dma_srcdst_t srcdst);
int MXC_DMA_RevB_AdvConfigChannel(mxc_dma_adv_config_t advConfig);
int MXC_DMA_RevB_SetSrcDst(mxc_dma_srcdst_t srcdst);
int MXC_DMA_RevB_GetSrcDst(mxc_dma_srcdst_t *srcdst);
int MXC_DMA_RevB_SetSrcReload(mxc_dma_srcdst_t srcdst);
int MXC_DMA_RevB_GetSrcReload(mxc_dma_srcdst_t *srcdst);
int MXC_DMA_RevB_SetCallback(int ch, void (*callback)(int, int));
int MXC_DMA_RevB_ChannelEnableInt(int ch, int flags);
int MXC_DMA_RevB_ChannelDisableInt(int ch, int flags);
int MXC_DMA_RevB_ChannelGetFlags(int ch);
int MXC_DMA_RevB_ChannelClearFlags(int ch, int flags);
int MXC_DMA_RevB_EnableInt(int ch);
int MXC_DMA_RevB_DisableInt(int ch);
int MXC_DMA_RevB_Start(int ch);
int MXC_DMA_RevB_Stop(int ch);
mxc_dma_ch_regs_t *MXC_DMA_RevB_GetCHRegs(int ch);
void MXC_DMA_RevB_Handler();
int MXC_DMA_RevB_MemCpy(void *dest, void *src, int len, mxc_dma_complete_cb_t callback);
int MXC_DMA_RevB_DoTransfer(mxc_dma_config_t config, mxc_dma_srcdst_t firstSrcDst,
                            mxc_dma_trans_chain_t callback);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_DMA_DMA_REVB_H_
