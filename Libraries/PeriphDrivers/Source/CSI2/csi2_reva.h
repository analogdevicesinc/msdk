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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_CSI2_CSI2_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_CSI2_CSI2_REVA_H_

/****** Includes *******/
#include <stdint.h>
#include "csi2.h"
#include "dma.h"
#include "dma_reva.h"
#include "csi2_reva_regs.h"

/***** Definitions *****/

/**
 * @brief   Enumeration type for the CSI-2 FIFO Triggers.
 */
typedef enum {
    MXC_CSI2_REVA_FF_TRIG_NO_TRIGGER = 0, ///< No FIFO Trigger
    MXC_CSI2_REVA_FF_TRIG_NOT_EMPTY = MXC_F_CSI2_REVA_RX_EINT_VFF_IF_FNEMPTY, ///< FIFO Not Empty
    MXC_CSI2_REVA_FF_TRIG_ABV_THD = MXC_F_CSI2_REVA_RX_EINT_VFF_IF_FTHD, ///< FIFO Above Threshold
    MXC_CSI2_REVA_FF_TRIG_FULL = MXC_F_CSI2_REVA_RX_EINT_VFF_IF_FFULL, ///< FIFO Full
} mxc_csi2_reva_fifo_trig_t;

typedef struct {
    bool success;
    uint32_t ctrl_err;
    uint32_t ppi_err;
    uint32_t vfifo_err;
    size_t frame_size;
    size_t bytes_captured;
} mxc_csi2_reva_capture_stats_t;

/******* Globals *******/

/****** Functions ******/

/******************************************/
/* Global Control/Configuration Functions */
/******************************************/

int MXC_CSI2_RevA_Init(mxc_csi2_reva_regs_t *csi2, mxc_csi2_req_t *req,
                       mxc_csi2_ctrl_cfg_t *ctrl_cfg, mxc_csi2_vfifo_cfg_t *vfifo_cfg);

int MXC_CSI2_RevA_Shutdown(mxc_csi2_reva_regs_t *csi2);

int MXC_CSI2_RevA_Start(mxc_csi2_reva_regs_t *csi2, int num_data_lanes);

int MXC_CSI2_RevA_Stop(mxc_csi2_reva_regs_t *csi2);

int MXC_CSI2_RevA_CaptureFrameDMA();

int MXC_CSI2_RevA_SetLaneCtrlSource(mxc_csi2_reva_regs_t *csi2, mxc_csi2_lane_src_t *src);

int MXC_CSI2_RevA_GetLaneCtrlSource(mxc_csi2_reva_regs_t *csi2, mxc_csi2_lane_src_t *src);

void MXC_CSI2_RevA_GetImageDetails(uint32_t *imgLen, uint32_t *w, uint32_t *h);

int MXC_CSI2_RevA_Callback(mxc_csi2_req_t *req, int retVal);

void MXC_CSI2_RevA_Handler();

/********************************/
/* CSI2 RX Controller Functions */
/********************************/

int MXC_CSI2_RevA_CTRL_Config(mxc_csi2_reva_regs_t *csi2, mxc_csi2_ctrl_cfg_t *cfg);

void MXC_CSI2_RevA_CTRL_EnableInt(mxc_csi2_reva_regs_t *csi2, uint32_t mask);

void MXC_CSI2_RevA_CTRL_DisableInt(mxc_csi2_reva_regs_t *csi2, uint32_t mask);

int MXC_CSI2_RevA_CTRL_GetFlags(mxc_csi2_reva_regs_t *csi2);

void MXC_CSI2_RevA_CTRL_ClearFlags(mxc_csi2_reva_regs_t *csi2, uint32_t flags);

/************************/
/* CSI2 VFIFO Functions */
/************************/

int MXC_CSI2_RevA_VFIFO_Config(mxc_csi2_reva_regs_t *csi2, mxc_csi2_vfifo_cfg_t *cfg);

int MXC_CSI2_RevA_VFIFO_ProcessRAWtoRGB(mxc_csi2_reva_regs_t *csi2, mxc_csi2_req_t *req);

int MXC_CSI2_RevA_VFIFO_NextFIFOTrigMode(mxc_csi2_reva_regs_t *csi2, uint8_t ff_not_empty,
                                         uint8_t ff_abv_thd, uint8_t ff_full);

void MXC_CSI2_RevA_VFIFO_EnableInt(mxc_csi2_reva_regs_t *csi2, uint32_t mask, uint32_t edge);

void MXC_CSI2_RevA_VFIFO_ChangeIntMode(mxc_csi2_reva_regs_t *csi2, uint32_t mask, uint32_t edge);

void MXC_CSI2_RevA_VFIFO_DisableInt(mxc_csi2_reva_regs_t *csi2, uint32_t mask);

int MXC_CSI2_RevA_VFIFO_GetFlags(mxc_csi2_reva_regs_t *csi2);

void MXC_CSI2_RevA_VFIFO_ClearFlags(mxc_csi2_reva_regs_t *csi2, uint32_t flags);

int MXC_CSI2_RevA_VFIFO_Enable(mxc_csi2_reva_regs_t *csi2);

int MXC_CSI2_RevA_VFIFO_Disable(mxc_csi2_reva_regs_t *csi2);

int MXC_CSI2_RevA_VFIFO_SetPayloadType(mxc_csi2_reva_regs_t *csi2, mxc_csi2_payload0_t payload0,
                                       mxc_csi2_payload1_t payload1);

int MXC_CSI2_RevA_VFIFO_GetPayloadType(mxc_csi2_reva_regs_t *csi2, uint32_t *payload0,
                                       uint32_t *payload1);

int MXC_CSI2_RevA_VFIFO_SetDMAMode(mxc_csi2_reva_regs_t *csi2, mxc_csi2_dma_mode_t dma_mode);

mxc_csi2_dma_mode_t MXC_CSI2_RevA_VFIFO_GetDMAMode(mxc_csi2_reva_regs_t *csi2);

int MXC_CSI2_RevA_VFIFO_SetRGBType(mxc_csi2_reva_regs_t *csi2, mxc_csi2_rgb_type_t rgb_type);

mxc_csi2_rgb_type_t MXC_CSI2_RevA_VFIFO_GetRGBType(mxc_csi2_reva_regs_t *csi2);

int MXC_CSI2_RevA_VFIFO_SetRAWFormat(mxc_csi2_reva_regs_t *csi2, mxc_csi2_raw_format_t raw_format);

mxc_csi2_raw_format_t MXC_CSI2_RevA_VFIFO_GetRAWFormat(mxc_csi2_reva_regs_t *csi2);

int MXC_CSI2_RevA_VFIFO_GetFIFOEntityCount(mxc_csi2_reva_regs_t *csi2);

void MXC_CSI2_RevA_VFIFO_SetAHBWait(mxc_csi2_reva_regs_t *csi2, mxc_csi2_ahbwait_t wait_en);

mxc_csi2_ahbwait_t MXC_CSI2_RevA_VFIFO_GetAHBWait(mxc_csi2_reva_regs_t *csi2);

/***********************************************/
/* CSI2 PHY Protocol Interface (PPI) Functions */
/***********************************************/

void MXC_CSI2_RevA_PPI_EnableInt(mxc_csi2_reva_regs_t *csi2, uint32_t mask);

void MXC_CSI2_RevA_PPI_DisableInt(mxc_csi2_reva_regs_t *csi2, uint32_t mask);

int MXC_CSI2_RevA_PPI_GetFlags(mxc_csi2_reva_regs_t *csi2);

void MXC_CSI2_RevA_PPI_ClearFlags(mxc_csi2_reva_regs_t *csi2, uint32_t flags);

int MXC_CSI2_RevA_PPI_Stop(void);

/************************************/
/* CSI2 DMA - Used for all features */
/************************************/

mxc_csi2_reva_capture_stats_t MXC_CSI2_RevA_DMA_GetCaptureStats();

bool MXC_CSI2_RevA_DMA_Frame_Complete(void);

int MXC_CSI2_RevA_DMA_Config(uint8_t *dst_addr, uint32_t byte_cnt, uint32_t burst_size);

int MXC_CSI2_RevA_DMA_GetChannel(void);

int MXC_CSI2_RevA_DMA_GetCurrentLineCnt(void);

int MXC_CSI2_RevA_DMA_GetCurrentFrameEndCnt(void);

void MXC_CSI2_RevA_DMA_Callback();

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_CSI2_CSI2_REVA_H_
