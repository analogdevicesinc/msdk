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
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_pins.h"
#include "mxc_sys.h"
#include "csi2.h"
#include "csi2_reva.h"
#include "dma.h"
#include "dma_reva.h"
#include "mcr_regs.h"
#include "nvic_table.h"

/* **** Definitions **** */

/* **** Globals **** */

/* **** Functions **** */

/******************************************/
/* Global Control/Configuration Functions */
/******************************************/

int MXC_CSI2_Init(mxc_csi2_req_t *req, mxc_csi2_ctrl_cfg_t *ctrl_cfg,
                  mxc_csi2_vfifo_cfg_t *vfifo_cfg)
{
    int error = E_NO_ERROR;
#ifdef __riscv
#warning "RISC-V Core does not have access to CSI2 IRQ.  Drivers are not supported on RISC-V"
    return E_NOT_SUPPORTED;
#else
    error = MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_IPLL);
    if (error)
        return error;

    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_CSI2);
    if (error)
        return error;

    // Turn on LDO2P5
    MXC_MCR->ldoctrl |= MXC_F_MCR_LDOCTRL_2P5EN;

    error = MXC_CSI2_RevA_Init((mxc_csi2_reva_regs_t *)MXC_CSI2, req, ctrl_cfg, vfifo_cfg);
    if (error)
        return error;

    MXC_NVIC_SetVector(CSI2_IRQn, MXC_CSI2_RevA_Handler);
    NVIC_EnableIRQ(CSI2_IRQn);
#endif
    return error;
}

int MXC_CSI2_Shutdown(void)
{
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_CSI2);

    return MXC_CSI2_RevA_Shutdown((mxc_csi2_reva_regs_t *)MXC_CSI2);
}

int MXC_CSI2_Start(int num_data_lanes)
{
    return MXC_CSI2_RevA_Start((mxc_csi2_reva_regs_t *)MXC_CSI2, num_data_lanes);
}

int MXC_CSI2_Stop(void)
{
    return MXC_CSI2_RevA_Stop((mxc_csi2_reva_regs_t *)MXC_CSI2);
}

int MXC_CSI2_CaptureFrame(int num_data_lanes)
{
    return MXC_CSI2_RevA_CaptureFrameDMA(num_data_lanes);
}

int MXC_CSI2_CaptureFrameDMA()
{
    return MXC_CSI2_RevA_CaptureFrameDMA();
}

int MXC_CSI2_SetLaneCtrlSource(mxc_csi2_lane_src_t *src)
{
    return MXC_CSI2_RevA_SetLaneCtrlSource((mxc_csi2_reva_regs_t *)MXC_CSI2, src);
}

int MXC_CSI2_GetLaneCtrlSource(mxc_csi2_lane_src_t *src)
{
    return MXC_CSI2_RevA_GetLaneCtrlSource((mxc_csi2_reva_regs_t *)MXC_CSI2, src);
}

void MXC_CSI2_GetImageDetails(uint32_t *imgLen, uint32_t *w, uint32_t *h)
{
    MXC_CSI2_RevA_GetImageDetails(imgLen, w, h);
}

int MXC_CSI2_Callback(mxc_csi2_req_t *req, int retVal)
{
    return MXC_CSI2_RevA_Callback(req, retVal);
}

// int MXC_CSI2_Handler(void)
// {
//     return MXC_CSI2_RevA_Handler((mxc_csi2_reva_regs_t *)MXC_CSI2);
// }

/********************************/
/* CSI2 RX Controller Functions */
/********************************/

int MXC_CSI2_CTRL_Config(mxc_csi2_ctrl_cfg_t *cfg)
{
    return MXC_CSI2_RevA_CTRL_Config((mxc_csi2_reva_regs_t *)MXC_CSI2, cfg);
}

void MXC_CSI2_CTRL_EnableInt(uint32_t mask)
{
    MXC_CSI2_RevA_CTRL_EnableInt((mxc_csi2_reva_regs_t *)MXC_CSI2, mask);
}

void MXC_CSI2_CTRL_DisableInt(uint32_t mask)
{
    MXC_CSI2_RevA_CTRL_DisableInt((mxc_csi2_reva_regs_t *)MXC_CSI2, mask);
}

int MXC_CSI2_CTRL_GetFlags(void)
{
    return MXC_CSI2_RevA_CTRL_GetFlags((mxc_csi2_reva_regs_t *)MXC_CSI2);
}

void MXC_CSI2_CTRL_ClearFlags(uint32_t flags)
{
    MXC_CSI2_RevA_CTRL_ClearFlags((mxc_csi2_reva_regs_t *)MXC_CSI2, flags);
}

/*****************************************/
/* CSI2 VFIFO - Control/Config functions */
/*****************************************/

int MXC_CSI2_VFIFO_Config(mxc_csi2_vfifo_cfg_t *cfg)
{
    return MXC_CSI2_RevA_VFIFO_Config((mxc_csi2_reva_regs_t *)MXC_CSI2, cfg);
}

int MXC_CSI2_VFIFO_ProcessRAWtoRGB(mxc_csi2_req_t *req)
{
    return MXC_CSI2_RevA_VFIFO_ProcessRAWtoRGB((mxc_csi2_reva_regs_t *)MXC_CSI2, req);
}

int MXC_CSI2_VFIFO_NextFIFOTrigMode(uint8_t ff_not_empty, uint8_t ff_abv_thd, uint8_t ff_full)
{
    return MXC_CSI2_RevA_VFIFO_NextFIFOTrigMode((mxc_csi2_reva_regs_t *)MXC_CSI2, ff_not_empty,
                                                ff_abv_thd, ff_full);
}

void MXC_CSI2_VFIFO_EnableInt(uint32_t mask, uint32_t edge)
{
    MXC_CSI2_RevA_VFIFO_EnableInt((mxc_csi2_reva_regs_t *)MXC_CSI2, mask, edge);
}

void MXC_CSI2_VFIFO_ChangeIntMode(uint32_t mask, uint32_t edge)
{
    MXC_CSI2_RevA_VFIFO_ChangeIntMode((mxc_csi2_reva_regs_t *)MXC_CSI2, mask, edge);
}

void MXC_CSI2_VFIFO_DisableInt(uint32_t mask)
{
    MXC_CSI2_RevA_VFIFO_DisableInt((mxc_csi2_reva_regs_t *)MXC_CSI2, mask);
}

int MXC_CSI2_VFIFO_GetFlags(void)
{
    return MXC_CSI2_RevA_VFIFO_GetFlags((mxc_csi2_reva_regs_t *)MXC_CSI2);
}

void MXC_CSI2_VFIFO_ClearFlags(uint32_t flags)
{
    MXC_CSI2_RevA_VFIFO_ClearFlags((mxc_csi2_reva_regs_t *)MXC_CSI2, flags);
}

int MXC_CSI2_VFIFO_Enable(void)
{
    return MXC_CSI2_RevA_VFIFO_Enable((mxc_csi2_reva_regs_t *)MXC_CSI2);
}

int MXC_CSI2_VFIFO_Disable(void)
{
    return MXC_CSI2_RevA_VFIFO_Disable((mxc_csi2_reva_regs_t *)MXC_CSI2);
}

int MXC_CSI2_VFIFO_SetPayloadType(mxc_csi2_payload0_t payload0, mxc_csi2_payload1_t payload1)
{
    return MXC_CSI2_RevA_VFIFO_SetPayloadType((mxc_csi2_reva_regs_t *)MXC_CSI2, payload0, payload1);
}

int MXC_CSI2_VFIFO_GetPayloadType(uint32_t *payload0, uint32_t *payload1)
{
    return MXC_CSI2_RevA_VFIFO_GetPayloadType((mxc_csi2_reva_regs_t *)MXC_CSI2, payload0, payload1);
}

int MXC_CSI2_VFIFO_SetDMAMode(mxc_csi2_dma_mode_t dma_mode)
{
    return MXC_CSI2_RevA_VFIFO_SetDMAMode((mxc_csi2_reva_regs_t *)MXC_CSI2, dma_mode);
}

mxc_csi2_dma_mode_t MXC_CSI2_VFIFO_GetDMAMode(void)
{
    return MXC_CSI2_RevA_VFIFO_GetDMAMode((mxc_csi2_reva_regs_t *)MXC_CSI2);
}

int MXC_CSI2_VFIFO_SetRGBType(mxc_csi2_rgb_type_t rgb_type)
{
    return MXC_CSI2_RevA_VFIFO_SetRGBType((mxc_csi2_reva_regs_t *)MXC_CSI2, rgb_type);
}

mxc_csi2_rgb_type_t MXC_CSI2_VFIFO_GetRGBType(void)
{
    return MXC_CSI2_RevA_VFIFO_GetRGBType((mxc_csi2_reva_regs_t *)MXC_CSI2);
}

int MXC_CSI2_VFIFO_SetRAWFormat(mxc_csi2_raw_format_t raw_format)
{
    return MXC_CSI2_RevA_VFIFO_SetRAWFormat((mxc_csi2_reva_regs_t *)MXC_CSI2, raw_format);
}

mxc_csi2_raw_format_t MXC_CSI2_VFIFO_GetRAWFormat(void)
{
    return MXC_CSI2_RevA_VFIFO_GetRAWFormat((mxc_csi2_reva_regs_t *)MXC_CSI2);
}

int MXC_CSI2_VFIFO_GetFIFOEntityCount(void)
{
    return MXC_CSI2_RevA_VFIFO_GetFIFOEntityCount((mxc_csi2_reva_regs_t *)MXC_CSI2);
}

void MXC_CSI2_VFIFO_SetAHBWait(mxc_csi2_ahbwait_t wait_en)
{
    MXC_CSI2_RevA_VFIFO_SetAHBWait((mxc_csi2_reva_regs_t *)MXC_CSI2, wait_en);
}

mxc_csi2_ahbwait_t MXC_CSI2_VFIFO_GetAHBWait(void)
{
    return MXC_CSI2_RevA_VFIFO_GetAHBWait((mxc_csi2_reva_regs_t *)MXC_CSI2);
}

/***********************************************/
/* CSI2 PHY Protocol Interface (PPI) Functions */
/***********************************************/

void MXC_CSI2_PPI_EnableInt(uint32_t mask)
{
    MXC_CSI2_RevA_PPI_EnableInt((mxc_csi2_reva_regs_t *)MXC_CSI2, mask);
}

void MXC_CSI2_PPI_DisableInt(uint32_t mask)
{
    MXC_CSI2_RevA_PPI_DisableInt((mxc_csi2_reva_regs_t *)MXC_CSI2, mask);
}

int MXC_CSI2_PPI_GetFlags(void)
{
    return MXC_CSI2_RevA_PPI_GetFlags((mxc_csi2_reva_regs_t *)MXC_CSI2);
}

void MXC_CSI2_PPI_ClearFlags(uint32_t flags)
{
    MXC_CSI2_RevA_PPI_ClearFlags((mxc_csi2_reva_regs_t *)MXC_CSI2, flags);
}

int MXC_CSI2_PPI_Stop(void)
{
    return MXC_CSI2_RevA_PPI_Stop();
}

/************************************/
/* CSI2 DMA - Used for all features */
/************************************/

bool MXC_CSI2_DMA_Frame_Complete(void)
{
    return MXC_CSI2_RevA_DMA_Frame_Complete();
}

mxc_csi2_capture_stats_t MXC_CSI2_GetCaptureStats()
{
    mxc_csi2_reva_capture_stats_t stats = MXC_CSI2_RevA_DMA_GetCaptureStats();
    mxc_csi2_capture_stats_t ret = { .success = stats.success,
                                     .ctrl_err = stats.ctrl_err,
                                     .ppi_err = stats.ppi_err,
                                     .vfifo_err = stats.vfifo_err,
                                     .frame_size = stats.frame_size,
                                     .bytes_captured = stats.bytes_captured };
    return ret;
}

int MXC_CSI2_DMA_Config(uint8_t *dst_addr, uint32_t byte_cnt, uint32_t burst_size)
{
    return MXC_CSI2_RevA_DMA_Config(dst_addr, byte_cnt, burst_size);
}

int MXC_CSI2_DMA_GetChannel(void)
{
    return MXC_CSI2_RevA_DMA_GetChannel();
}

int MXC_CSI2_DMA_GetCurrentLineCnt(void)
{
    return MXC_CSI2_RevA_DMA_GetCurrentLineCnt();
}

int MXC_CSI2_DMA_GetCurrentFrameEndCnt(void)
{
    return MXC_CSI2_RevA_DMA_GetCurrentFrameEndCnt();
}

void MXC_CSI2_DMA_Callback(int a, int b)
{
    MXC_CSI2_RevA_DMA_Callback((mxc_dma_reva_regs_t *)MXC_DMA, a, b);
}

/**@} end of group csi2 */
