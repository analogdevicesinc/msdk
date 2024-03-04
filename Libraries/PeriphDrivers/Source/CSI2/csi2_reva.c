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

#ifdef __riscv
#warning "CSI2 drivers not supported on RISC-V"
#else

/* **** Includes **** */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_pins.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "csi2.h"
#include "csi2_reva.h"
#include "nvic_table.h"
#include "dma.h"
#include "dma_reva.h"
#include "mcr_regs.h"

/* **** Definitions **** */

#define MXC_CSI2_REVA_CTRL_ERRINT_FL                                                 \
    (MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_EECC2 | MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_EECC1 | \
     MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_ECRC | MXC_F_CSI2_REVA_RX_EINT_CTRL_IE_EID)

#define MXC_CSI2_REVA_VFIFO_ERRINT_FL                                                 \
    (MXC_F_CSI2_REVA_RX_EINT_VFF_IF_OUTSYNC | MXC_F_CSI2_REVA_RX_EINT_VFF_IF_FMTERR | \
     MXC_F_CSI2_REVA_RX_EINT_VFF_IF_RAW_AHBERR)

#define MXC_CSI2_REVA_VFIFO_FIFOINT_FL                                              \
    (MXC_F_CSI2_REVA_RX_EINT_VFF_IF_FNEMPTY | MXC_F_CSI2_REVA_RX_EINT_VFF_IF_FTHD | \
     MXC_F_CSI2_REVA_RX_EINT_VFF_IF_FFULL)

#define MXC_CSI2_REVA_VFIFO_FIFOINT_EN                                              \
    (MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FNEMPTY | MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FTHD | \
     MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FFULL)

#define MXC_CSI2_REVA_VFIFO_DETECT_MD                                                   \
    (MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FNEMP_MD | MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FTHD_MD | \
     MXC_F_CSI2_REVA_RX_EINT_VFF_IE_FFUL_MD)

// A macro to convert a DMA channel number to an IRQn number
#define GetIRQnForDMAChannel(x)           \
    ((IRQn_Type)(((x) == 0) ? DMA0_IRQn : \
                 ((x) == 1) ? DMA1_IRQn : \
                 ((x) == 2) ? DMA2_IRQn : \
                              DMA3_IRQn))

/* **** Globals **** */

static volatile uint32_t dphy_rdy;
static volatile uint32_t frame_end_cnt;
static volatile uint32_t line_cnt;
static volatile uint32_t odd_line_byte_num;
static volatile uint32_t even_line_byte_num;
static volatile uint32_t line_byte_num;
static volatile uint32_t frame_byte_num;
static volatile bool g_frame_complete = false;

// Used for Non-DMA CSI2 Cases
static volatile uint32_t bits_per_pixel;
static volatile uint32_t fifo_burst_size;
static volatile mxc_csi2_reva_fifo_trig_t fifo_int_trig;
static volatile mxc_csi2_ahbwait_t ahbwait_en;

struct line_buffer {
    uint8_t *a; // Line buffer A
    uint8_t *b; // Line buffer B
    uint8_t *active; // Pointer to the active line buffer (volatile!)
    uint8_t *inactive; // Pointer to the inactive line buffer (safe to read)
} lb;

void _free_line_buffer(void)
{
    if (lb.a != NULL) {
        free((uint8_t *)lb.a);
        lb.a = NULL;
    }
    if (lb.b != NULL) {
        free((uint8_t *)lb.b);
        lb.b = NULL;
    }
}

int _init_line_buffer()
{
    _free_line_buffer();

    lb.a = (uint8_t *)malloc(line_byte_num);
    lb.b = (uint8_t *)malloc(line_byte_num);
    if (lb.a == NULL || lb.b == NULL)
        return E_NULL_PTR;

    lb.active = lb.a;
    lb.inactive = lb.b;
    return E_NO_ERROR;
}

void _swap_line_buffer()
{
    uint8_t *temp = lb.active;
    lb.active = lb.inactive;
    lb.inactive = temp;
}

typedef struct {
    mxc_csi2_req_t *req;
    mxc_csi2_ctrl_cfg_t *ctrl_cfg;
    mxc_csi2_vfifo_cfg_t *vfifo_cfg;
    int dma_channel;
    bool synced;
    mxc_csi2_reva_capture_stats_t capture_stats;
} csi2_reva_req_state_t;

volatile csi2_reva_req_state_t csi2_state;

/* **** Functions **** */

/******************************************/
/* Global Control/Configuration Functions */
/******************************************/

int MXC_CSI2_RevA_Init(mxc_csi2_reva_regs_t *csi2, mxc_csi2_req_t *req,
                       mxc_csi2_ctrl_cfg_t *ctrl_cfg, mxc_csi2_vfifo_cfg_t *vfifo_cfg)
{
    int error;

    dphy_rdy = 0;
    line_cnt = 0;
    frame_end_cnt = 0;

    csi2_state.req = req;
    csi2_state.ctrl_cfg = ctrl_cfg;
    csi2_state.vfifo_cfg = vfifo_cfg;
    csi2_state.synced = false;

    // Convert respective pixel bit number to bytes
    frame_byte_num = ((req->bits_per_pixel_odd + req->bits_per_pixel_even) * req->pixels_per_line *
                      req->lines_per_frame) >>
                     4;
    odd_line_byte_num = (req->bits_per_pixel_odd * req->pixels_per_line) >> 3;
    even_line_byte_num = (req->bits_per_pixel_even * req->pixels_per_line) >> 3;

    // Presetting with even line byte number as default
    line_byte_num = even_line_byte_num;

    // D-PHY reset
    csi2->dphy_rst_n = 0x00;

    error = MXC_CSI2_CTRL_Config(ctrl_cfg);
    if (error != E_NO_ERROR) {
        return error;
    }

    error = MXC_DMA_Init();
    if (error != E_NO_ERROR) {
        return error;
    }

    int channel = MXC_DMA_AcquireChannel();
    csi2_state.dma_channel = channel;

    // Configure VFIFO
    csi2->vfifo_pixel_num = req->pixels_per_line;
    csi2->vfifo_line_num = req->lines_per_frame;

    error = MXC_CSI2_VFIFO_Config(vfifo_cfg);
    if (error != E_NO_ERROR) {
        return error;
    }

    error = MXC_CSI2_VFIFO_ProcessRAWtoRGB(req);
    if (error != E_NO_ERROR) {
        return error;
    }

    return E_NO_ERROR;
}

int MXC_CSI2_RevA_Shutdown(mxc_csi2_reva_regs_t *csi2)
{
    int error;

    error = MXC_CSI2_Stop();
    if (error != E_NO_ERROR) {
        return error;
    }

    MXC_CSI2_CTRL_DisableInt(0xFFFFFFFF);
    MXC_CSI2_VFIFO_DisableInt(0xFFFFFFFF);
    MXC_CSI2_PPI_DisableInt(0xFFFFFFFF);

    error = MXC_DMA_ReleaseChannel(csi2_state.dma_channel);
    if (error != E_NO_ERROR) {
        return error;
    }

    return E_NO_ERROR;
}

int MXC_CSI2_RevA_Start(mxc_csi2_reva_regs_t *csi2, int num_data_lanes)
{
    int i;
    int enable_dlanes;

    dphy_rdy = 0;
    line_cnt = 0;
    frame_end_cnt = 0;
    csi2_state.synced = false;

    MXC_CSI2_CTRL_ClearFlags(0xFFFFFFFF);
    MXC_CSI2_VFIFO_ClearFlags(0xFFFFFFFF);
    MXC_CSI2_PPI_ClearFlags(0xFFFFFFFF);

    // Enable VFIFO
    csi2->vfifo_ctrl |= MXC_F_CSI2_REVA_VFIFO_CTRL_FIFOEN;

    // Release DPHY reset
    csi2->dphy_rst_n |= 0x01;

    // Enable CSI2 lanes
    csi2->cfg_clk_lane_en = 0x01;

    // This function assumes configured data lanes are used sequentially (e.g. x2 data lanes = data lane0 and lane1).
    enable_dlanes = 0;
    for (i = 0; i < num_data_lanes; i++) {
        enable_dlanes |= (1 << i);
    }

    csi2->cfg_data_lane_en = enable_dlanes;

    // Must include setting this bit.
    csi2->xcfgi_dw0b = 0x10000000;

    return E_NO_ERROR;
}

int MXC_CSI2_RevA_Stop(mxc_csi2_reva_regs_t *csi2)
{
    g_frame_complete = true;

    MXC_CSI2_VFIFO_Disable();

    // Reset DPHY
    csi2->dphy_rst_n = 0x00;

    // Enable CSI2 lanes
    csi2->cfg_clk_lane_en = 0;
    csi2->cfg_data_lane_en = 0;

    _free_line_buffer();

    return E_NO_ERROR;
}

int MXC_CSI2_RevA_CaptureFrameDMA()
{
    int i;
    int error;
    int dma_byte_cnt;
    int dlane_stop_inten;

    csi2_state.capture_stats.success = false;
    csi2_state.capture_stats.ctrl_err = 0;
    csi2_state.capture_stats.ppi_err = 0;
    csi2_state.capture_stats.vfifo_err = 0;
    csi2_state.capture_stats.bytes_captured = 0;
    g_frame_complete = false;

    mxc_csi2_req_t *req = csi2_state.req;
    mxc_csi2_vfifo_cfg_t *vfifo = csi2_state.vfifo_cfg;

    // Convert respective pixel bit number to bytes
    frame_byte_num = ((req->bits_per_pixel_odd + req->bits_per_pixel_even) * req->pixels_per_line *
                      req->lines_per_frame) >>
                     4;
    csi2_state.capture_stats.frame_size = frame_byte_num;
    odd_line_byte_num = (req->bits_per_pixel_odd * req->pixels_per_line) >> 3;
    even_line_byte_num = (req->bits_per_pixel_even * req->pixels_per_line) >> 3;

    // Select lower line byte number (Odd line)
    line_byte_num = odd_line_byte_num;

    if (vfifo->dma_whole_frame == MXC_CSI2_DMA_WHOLE_FRAME) {
        // Whole frame
        // dma_byte_cnt = frame_byte_num;
        // error = MXC_CSI2_DMA_Config(req->img_addr, dma_byte_cnt, vfifo->rx_thd);
        // if (error != E_NO_ERROR) {
        //     return error;
        // }

        /*
        Whole frame captures have been defeatured in favor of application-defined line handlers.
        We won't entirely burn this bridge now, so we'll leave the framework intact and return not supported.
        */
        return E_NOT_SUPPORTED;
    } else {
        // Line by line
        dma_byte_cnt = line_byte_num;

        error = _init_line_buffer();
        if (error)
            return error;

        error = MXC_CSI2_DMA_Config(lb.a, dma_byte_cnt, vfifo->rx_thd);
        if (error != E_NO_ERROR) {
            return error;
        }
    }

    // Enable Stop State interrupts for all used data lanes
    dlane_stop_inten = 0;
    for (i = 0; i < csi2_state.ctrl_cfg->num_lanes; i++) {
        dlane_stop_inten |= (1 << i);
    }

    dlane_stop_inten <<= MXC_F_CSI2_RX_EINT_PPI_IE_DL0STOP_POS;
    MXC_CSI2_PPI_EnableInt(dlane_stop_inten);

    // Clear all flags, enable all interrupts.
    MXC_CSI2->rx_eint_ctrl_if = 0xFFFFFFFF;
    MXC_CSI2->rx_eint_ppi_if = 0xFFFFFFFF;
    MXC_CSI2->rx_eint_vff_if = 0xFFFFFFFF;
    MXC_CSI2->rx_eint_ctrl_ie = 0xFFFFFFFF;
    MXC_CSI2->rx_eint_ppi_ie = 0xFFFFFFFF;
    MXC_CSI2->rx_eint_vff_ie = 0xFFFFFFFF;

    error = MXC_CSI2_Start(csi2_state.ctrl_cfg->num_lanes);
    if (error != E_NO_ERROR) {
        return error;
    }

    /*
    After starting, the drivers should wait for the PPI interrupt flags
    to indicate the CSI2 is synced up with the sensor.  Then, they should
    wait for the Frame Start VFIFO interrupt flag before enabling the DMA
    pipeline.

    Register polling is too slow to do that here, so it's implemented in the 
    interrupt handler. (MXC_CSI2_RevA_Handler)
    */

    while (!g_frame_complete) {}

    if (!csi2_state.capture_stats.success)
        return E_FAIL;
    else
        return E_NO_ERROR;
}

int MXC_CSI2_RevA_SetLaneCtrlSource(mxc_csi2_reva_regs_t *csi2, mxc_csi2_lane_src_t *src)
{
    if (src == NULL) {
        return E_BAD_PARAM;
    }

    csi2->cfg_d0_swap_sel = src->d0_swap_sel;
    csi2->cfg_d1_swap_sel = src->d1_swap_sel;
    csi2->cfg_d2_swap_sel = src->d2_swap_sel;
    csi2->cfg_d3_swap_sel = src->d3_swap_sel;
    csi2->cfg_c0_swap_sel = src->c0_swap_sel;

    return E_NO_ERROR;
}

int MXC_CSI2_RevA_GetLaneCtrlSource(mxc_csi2_reva_regs_t *csi2, mxc_csi2_lane_src_t *src)
{
    if (src == NULL) {
        return E_BAD_PARAM;
    }

    src->d0_swap_sel = csi2->cfg_d0_swap_sel;
    src->d1_swap_sel = csi2->cfg_d1_swap_sel;
    src->d2_swap_sel = csi2->cfg_d2_swap_sel;
    src->d3_swap_sel = csi2->cfg_d3_swap_sel;
    src->c0_swap_sel = csi2->cfg_c0_swap_sel;

    return E_NO_ERROR;
}

void MXC_CSI2_RevA_GetImageDetails(uint32_t *imgLen, uint32_t *w, uint32_t *h)
{
    *imgLen = frame_byte_num;
    *w = csi2_state.req->pixels_per_line;
    *h = csi2_state.req->lines_per_frame;
}

void MXC_CSI2_RevA_Handler()
{
    uint32_t ctrl_flags, vfifo_flags, ppi_flags;

    ctrl_flags = MXC_CSI2_CTRL_GetFlags();
    ppi_flags = MXC_CSI2_PPI_GetFlags();
    vfifo_flags = MXC_CSI2_VFIFO_GetFlags();

    // Clear Flags.
    MXC_CSI2_CTRL_ClearFlags(ctrl_flags);
    MXC_CSI2_PPI_ClearFlags(ppi_flags);
    MXC_CSI2_VFIFO_ClearFlags(vfifo_flags);

    // Mask out non-critical CTRL status flags
    ctrl_flags &= (0b11111);
    if (ctrl_flags) {
        csi2_state.capture_stats.ctrl_err |= ctrl_flags;
    }

    if (!csi2_state.synced && ppi_flags != 0) {
        /*
        When the PPI flags below have been signaled, the CSI2 interface
        has synced up with the sensor.  It's now safe to monitor the VFIFO.
        */
        csi2_state.synced = (bool)(ppi_flags & (MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0STOP |
                                                MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL1STOP |
                                                MXC_F_CSI2_REVA_RX_EINT_PPI_IF_CL0STOP));
    }

    // Mask out non-critical PPI status flags
    ppi_flags &= ~(MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL0STOP | MXC_F_CSI2_REVA_RX_EINT_PPI_IF_DL1STOP |
                   MXC_F_CSI2_REVA_RX_EINT_PPI_IF_CL0STOP);

    if (ppi_flags) {
        csi2_state.capture_stats.ppi_err |= ppi_flags;
    }

    if (vfifo_flags != 0) {
        if (vfifo_flags & MXC_F_CSI2_RX_EINT_VFF_IF_FS && csi2_state.synced) {
            /*
            "Frame Start" has been received.  Enable the DMA channel.
            MXC_CSI2_RevA_DMA_Handler does the heavy lifting from here.
            */
            MXC_DMA_Start(csi2_state.dma_channel);
        }

        // Mask out non-critical VFIFO status flags
        vfifo_flags &= (MXC_F_CSI2_RX_EINT_VFF_IF_UNDERRUN | MXC_F_CSI2_RX_EINT_VFF_IF_OVERRUN |
                        MXC_F_CSI2_RX_EINT_VFF_IF_OUTSYNC | MXC_F_CSI2_RX_EINT_VFF_IF_FMTERR |
                        MXC_F_CSI2_RX_EINT_VFF_IF_AHBWTO | MXC_F_CSI2_RX_EINT_VFF_IF_RAW_OVR |
                        MXC_F_CSI2_RX_EINT_VFF_IF_RAW_AHBERR);

        if (vfifo_flags) {
            csi2_state.capture_stats.vfifo_err |= vfifo_flags;
        }
    }

    if (csi2_state.capture_stats.ctrl_err | csi2_state.capture_stats.ppi_err |
        csi2_state.capture_stats.vfifo_err) {
        csi2_state.capture_stats.success = false;
        MXC_CSI2_RevA_Stop((mxc_csi2_reva_regs_t *)MXC_CSI2);
    }
}

/********************************/
/* CSI2 RX Controller Functions */
/********************************/

int MXC_CSI2_RevA_CTRL_Config(mxc_csi2_reva_regs_t *csi2, mxc_csi2_ctrl_cfg_t *cfg)
{
    // Set Power Ready
    csi2->aon_power_ready_n &= ~0x01;

    // Invert RX Clock
    if (cfg->invert_ppi_clk) {
        csi2->rxbyteclkhs_inv = 0x01;
    } else {
        csi2->rxbyteclkhs_inv = 0x00;
    }

    // Setting number of lanes used
    csi2->cfg_num_lanes = cfg->num_lanes;

    // Must select payload data type
    if ((cfg->payload0 == MXC_CSI2_PL0_DISABLE_ALL) &&
        (cfg->payload1 == MXC_CSI2_PL1_DISABLE_ALL)) {
        return E_NOT_SUPPORTED;
    }

    csi2->cfg_disable_payload_0 = cfg->payload0;
    csi2->cfg_disable_payload_1 = cfg->payload1;

    // Set flush count
    if (cfg->flush_cnt < 0 || cfg->flush_cnt > 15) {
        return E_BAD_PARAM;
    }

    csi2->cfg_flush_count = cfg->flush_cnt;

    // Configure Data and Clock Lane Control Source
    MXC_CSI2_SetLaneCtrlSource(&(cfg->lane_src));

    return E_NO_ERROR;
}

void MXC_CSI2_RevA_CTRL_EnableInt(mxc_csi2_reva_regs_t *csi2, uint32_t mask)
{
    // Clear flags before enabling
    csi2->rx_eint_ctrl_if |= mask;

    csi2->rx_eint_ctrl_ie |= mask;
}

void MXC_CSI2_RevA_CTRL_DisableInt(mxc_csi2_reva_regs_t *csi2, uint32_t mask)
{
    csi2->rx_eint_ctrl_ie &= ~mask;
}

int MXC_CSI2_RevA_CTRL_GetFlags(mxc_csi2_reva_regs_t *csi2)
{
    return (csi2->rx_eint_ctrl_if);
}

void MXC_CSI2_RevA_CTRL_ClearFlags(mxc_csi2_reva_regs_t *csi2, uint32_t flags)
{
    csi2->rx_eint_ctrl_if |= flags;
}

/************************/
/* CSI2 VFIFO Functions */
/************************/

int MXC_CSI2_RevA_VFIFO_Config(mxc_csi2_reva_regs_t *csi2, mxc_csi2_vfifo_cfg_t *cfg)
{
    int error;

    csi2->vfifo_cfg1 = (cfg->flow_ctrl) | ((cfg->wait_cyc << MXC_F_CSI2_VFIFO_CFG1_AHBWCYC_POS) &
                                           MXC_F_CSI2_VFIFO_CFG1_AHBWCYC);

    // Set virtual channel
    if (cfg->virtual_channel > 3 || cfg->virtual_channel < 0) {
        return E_BAD_PARAM;
    }

    MXC_SETFIELD(csi2->vfifo_cfg0, MXC_F_CSI2_REVA_VFIFO_CFG0_VC,
                 cfg->virtual_channel << MXC_F_CSI2_REVA_VFIFO_CFG0_VC_POS);

    error = MXC_CSI2_VFIFO_SetDMAMode(cfg->dma_mode);
    if (error != E_NO_ERROR) {
        return error;
    }

    // Enable AHB WAIT
    if (cfg->wait_en) {
        MXC_SETFIELD(csi2->vfifo_cfg0, MXC_F_CSI2_REVA_VFIFO_CFG0_AHBWAIT,
                     0x1 << MXC_F_CSI2_REVA_VFIFO_CFG0_AHBWAIT_POS);
    } else {
        MXC_SETFIELD(csi2->vfifo_cfg0, MXC_F_CSI2_REVA_VFIFO_CFG0_AHBWAIT,
                     0x0 << MXC_F_CSI2_REVA_VFIFO_CFG0_AHBWAIT_POS);
    }

    // Select FIFO Read Mode (One-by-One vs Direct Addressing for each entity)
    if (cfg->fifo_rd_mode) {
        MXC_SETFIELD(csi2->vfifo_cfg0, MXC_F_CSI2_REVA_VFIFO_CFG0_FIFORM,
                     0x1 << MXC_F_CSI2_REVA_VFIFO_CFG0_FIFORM_POS);
    } else {
        MXC_SETFIELD(csi2->vfifo_cfg0, MXC_F_CSI2_REVA_VFIFO_CFG0_FIFORM,
                     0x0 << MXC_F_CSI2_REVA_VFIFO_CFG0_FIFORM_POS);
    }

    // Enable Error Detection
    if (cfg->err_det_en == MXC_CSI2_ERR_DETECT_ENABLE) {
        MXC_SETFIELD(csi2->vfifo_cfg0, MXC_F_CSI2_REVA_VFIFO_CFG0_ERRDE,
                     0x1 << MXC_F_CSI2_REVA_VFIFO_CFG0_ERRDE_POS);
    } else {
        MXC_SETFIELD(csi2->vfifo_cfg0, MXC_F_CSI2_REVA_VFIFO_CFG0_ERRDE,
                     0x0 << MXC_F_CSI2_REVA_VFIFO_CFG0_ERRDE_POS);
    }

    // Select Normal mode or Full Bandwidth mode
    if (cfg->bandwidth_mode) {
        MXC_SETFIELD(csi2->vfifo_cfg0, MXC_F_CSI2_REVA_VFIFO_CFG0_FBWM,
                     0x1 << MXC_F_CSI2_VFIFO_CFG0_FBWM_POS);
    } else {
        MXC_SETFIELD(csi2->vfifo_cfg0, MXC_F_CSI2_REVA_VFIFO_CFG0_FBWM,
                     0x0 << MXC_F_CSI2_VFIFO_CFG0_FBWM_POS);
    }

    // Set RX Threshold
    if (cfg->rx_thd >= MXC_CSI2_FIFO_DEPTH) {
        return E_BAD_PARAM;
    }

    MXC_SETFIELD(csi2->vfifo_ctrl, MXC_F_CSI2_REVA_VFIFO_CTRL_THD,
                 cfg->rx_thd << MXC_F_CSI2_REVA_VFIFO_CTRL_THD_POS);

    return E_NO_ERROR;
}

/* Note for maintainers: The CSI2 RevA hardware has significant issues with its RAW->RGB
debayering engine.  It is strongly recommended to avoid using this until the 
issues have been resolved.  See the AI87 Design Jira for more details.
*/
int MXC_CSI2_RevA_VFIFO_ProcessRAWtoRGB(mxc_csi2_reva_regs_t *csi2, mxc_csi2_req_t *req)
{
    int error;

    csi2->vfifo_raw_buf0_addr = req->raw_buf0_addr;
    csi2->vfifo_raw_buf1_addr = req->raw_buf1_addr;

    // Process RAW to Selected RGB Type if applicable
    if (req->process_raw_to_rgb) {
        error = MXC_CSI2_VFIFO_SetRGBType(req->rgb_type);
        if (error != E_NO_ERROR) {
            return error;
        }

        error = MXC_CSI2_VFIFO_SetRAWFormat(req->raw_format);
        if (error != E_NO_ERROR) {
            return error;
        }

        if (req->autoflush) {
            MXC_SETFIELD(csi2->vfifo_raw_ctrl, MXC_F_CSI2_VFIFO_RAW_CTRL_RAW_FF_AFO,
                         0x1 << MXC_F_CSI2_VFIFO_RAW_CTRL_RAW_FF_AFO_POS);
        } else {
            MXC_SETFIELD(csi2->vfifo_raw_ctrl, MXC_F_CSI2_VFIFO_RAW_CTRL_RAW_FF_AFO,
                         0x0 << MXC_F_CSI2_VFIFO_RAW_CTRL_RAW_FF_AFO_POS);
        }

        csi2->vfifo_raw_ctrl |= MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RAW_CEN;
    }

    return E_NO_ERROR;
}

int MXC_CSI2_RevA_VFIFO_NextFIFOTrigMode(mxc_csi2_reva_regs_t *csi2, uint8_t ff_not_empty,
                                         uint8_t ff_abv_thd, uint8_t ff_full)
{
    // Disable FIFO-related interrupts and clear FIFO trigger detection mode before switching
    MXC_CSI2_VFIFO_DisableInt(MXC_CSI2_REVA_VFIFO_FIFOINT_EN);
    MXC_CSI2_VFIFO_ChangeIntMode(MXC_CSI2_REVA_VFIFO_FIFOINT_EN, 0);

    // Set to next FIFO trigger if applicable
    switch (fifo_int_trig) {
    case MXC_CSI2_REVA_FF_TRIG_NOT_EMPTY:
        if (ff_abv_thd) {
            fifo_int_trig = MXC_CSI2_REVA_FF_TRIG_ABV_THD;
        } else if (ff_full) {
            fifo_int_trig = MXC_CSI2_REVA_FF_TRIG_FULL;
        } else {
            fifo_int_trig = MXC_CSI2_REVA_FF_TRIG_NOT_EMPTY;
        }
        break;

    case MXC_CSI2_REVA_FF_TRIG_ABV_THD:
        if (ff_full) {
            fifo_int_trig = MXC_CSI2_REVA_FF_TRIG_FULL;
        } else if (ff_not_empty) {
            fifo_int_trig = MXC_CSI2_REVA_FF_TRIG_NOT_EMPTY;
        } else {
            fifo_int_trig = MXC_CSI2_REVA_FF_TRIG_ABV_THD;
        }
        break;

    case MXC_CSI2_REVA_FF_TRIG_FULL:
        if (ff_not_empty) {
            fifo_int_trig = MXC_CSI2_REVA_FF_TRIG_NOT_EMPTY;
        } else if (ff_abv_thd) {
            fifo_int_trig = MXC_CSI2_REVA_FF_TRIG_ABV_THD;
        } else {
            fifo_int_trig = MXC_CSI2_REVA_FF_TRIG_FULL;
        }
        break;

    default:
        fifo_int_trig = MXC_CSI2_REVA_FF_TRIG_NO_TRIGGER;
        break;
    }

    // Set new burst length and ahbwait parameters if applicable
    switch (fifo_int_trig) {
    case MXC_CSI2_REVA_FF_TRIG_NOT_EMPTY:
        fifo_burst_size = bits_per_pixel >> 1;
        ahbwait_en = MXC_CSI2_AHBWAIT_ENABLE;
        break;

    case MXC_CSI2_REVA_FF_TRIG_ABV_THD:
        fifo_burst_size = csi2_state.vfifo_cfg->rx_thd;
        ahbwait_en = MXC_CSI2_AHBWAIT_DISABLE;
        break;

    case MXC_CSI2_REVA_FF_TRIG_FULL:
        fifo_burst_size = 64; // Max burst size
        ahbwait_en = MXC_CSI2_AHBWAIT_DISABLE;
        break;

    default:
        return E_BAD_PARAM;
    }

    // Update new configurations for next FIFO read
    MXC_CSI2_VFIFO_SetAHBWait(ahbwait_en);
    MXC_CSI2_VFIFO_EnableInt(fifo_int_trig, 1);

    return E_NO_ERROR;
}

void MXC_CSI2_RevA_VFIFO_EnableInt(mxc_csi2_reva_regs_t *csi2, uint32_t mask, uint32_t edge)
{
    // Clear flags before enabling
    csi2->rx_eint_vff_if |= mask;

    csi2->rx_eint_vff_ie |= mask;

    // Set edge triggered or level triggered FIFO modes if applicable
    MXC_CSI2_VFIFO_ChangeIntMode(mask, edge);
}

void MXC_CSI2_RevA_VFIFO_ChangeIntMode(mxc_csi2_reva_regs_t *csi2, uint32_t mask, uint32_t edge)
{
    // Edge Triggered Mode
    if (edge && (mask & MXC_CSI2_REVA_VFIFO_FIFOINT_EN)) {
        // Set corresponding detection mode for FIFO not empty, above thd, and full interrupts
        csi2->rx_eint_vff_ie |=
            ((mask & MXC_CSI2_REVA_VFIFO_FIFOINT_EN) << MXC_F_CSI2_RX_EINT_VFF_IE_FNEMP_MD_POS);

        // Level Triggered Mode
    } else if (!edge && (mask & MXC_CSI2_REVA_VFIFO_FIFOINT_EN)) {
        // Clear corresponding detection mode for FIFO not empty, above thd, and full interrupts
        csi2->rx_eint_vff_ie &=
            ~((mask & MXC_CSI2_REVA_VFIFO_FIFOINT_EN) << MXC_F_CSI2_RX_EINT_VFF_IE_FNEMP_MD_POS);
    }
}

void MXC_CSI2_RevA_VFIFO_DisableInt(mxc_csi2_reva_regs_t *csi2, uint32_t mask)
{
    csi2->rx_eint_vff_ie &= ~mask;
}

int MXC_CSI2_RevA_VFIFO_GetFlags(mxc_csi2_reva_regs_t *csi2)
{
    return (csi2->rx_eint_vff_if);
}

void MXC_CSI2_RevA_VFIFO_ClearFlags(mxc_csi2_reva_regs_t *csi2, uint32_t flags)
{
    csi2->rx_eint_vff_if |= flags;
}

int MXC_CSI2_RevA_VFIFO_Enable(mxc_csi2_reva_regs_t *csi2)
{
    csi2->vfifo_ctrl |= MXC_F_CSI2_REVA_VFIFO_CTRL_FIFOEN;

    return E_NO_ERROR;
}

int MXC_CSI2_RevA_VFIFO_Disable(mxc_csi2_reva_regs_t *csi2)
{
    csi2->vfifo_ctrl &= ~MXC_F_CSI2_REVA_VFIFO_CTRL_FIFOEN;

    return E_NO_ERROR;
}

int MXC_CSI2_RevA_VFIFO_SetPayloadType(mxc_csi2_reva_regs_t *csi2, mxc_csi2_payload0_t payload0,
                                       mxc_csi2_payload1_t payload1)
{
    // Need to set one Payload data type
    if ((payload0 == MXC_CSI2_PL0_DISABLE_ALL) && (payload1 == MXC_CSI2_PL1_DISABLE_ALL)) {
        return E_BAD_PARAM;
    }

    csi2->cfg_disable_payload_0 = payload0;
    csi2->cfg_disable_payload_1 = payload1;

    return E_NO_ERROR;
}

int MXC_CSI2_RevA_VFIFO_GetPayloadType(mxc_csi2_reva_regs_t *csi2, uint32_t *payload0,
                                       uint32_t *payload1)
{
    if (payload0 == NULL || payload1 == NULL) {
        return E_NULL_PTR;
    }

    *payload0 = csi2->cfg_disable_payload_0;
    *payload1 = csi2->cfg_disable_payload_1;

    return E_NO_ERROR;
}

int MXC_CSI2_RevA_VFIFO_SetDMAMode(mxc_csi2_reva_regs_t *csi2, mxc_csi2_dma_mode_t dma_mode)
{
    // Check for valid DMA Mode
    if (dma_mode < MXC_CSI2_DMA_NO_DMA || dma_mode > MXC_CSI2_DMA_FIFO_FULL) {
        return E_BAD_PARAM;
    }

    MXC_SETFIELD(csi2->vfifo_cfg0, MXC_F_CSI2_REVA_VFIFO_CFG0_DMAMODE, dma_mode);

    return E_NO_ERROR;
}

mxc_csi2_dma_mode_t MXC_CSI2_RevA_VFIFO_GetDMAMode(mxc_csi2_reva_regs_t *csi2)
{
    int dma_mode;
    mxc_csi2_dma_mode_t result;

    dma_mode = csi2->vfifo_cfg0 & MXC_F_CSI2_REVA_VFIFO_CFG0_DMAMODE;
    switch (dma_mode) {
    // No DMA
    case MXC_S_CSI2_REVA_VFIFO_CFG0_DMAMODE_NO_DMA:
        result = MXC_CSI2_DMA_NO_DMA;
        break;

    // DMA Request
    case MXC_S_CSI2_REVA_VFIFO_CFG0_DMAMODE_DMA_REQ:
        result = MXC_CSI2_DMA_SEND_REQUEST;
        break;

    // FIFO Above Threshold
    case MXC_S_CSI2_REVA_VFIFO_CFG0_DMAMODE_FIFO_THD:
        result = MXC_CSI2_DMA_FIFO_ABV_THD;
        break;

    // FIFO Full
    case MXC_S_CSI2_REVA_VFIFO_CFG0_DMAMODE_FIFO_FULL:
        result = MXC_CSI2_DMA_FIFO_FULL;
        break;

    default:
        return E_BAD_PARAM;
    }

    return result;
}

int MXC_CSI2_RevA_VFIFO_SetRGBType(mxc_csi2_reva_regs_t *csi2, mxc_csi2_rgb_type_t rgb_type)
{
    // Check for valid RGB Type
    if (rgb_type < MXC_CSI2_TYPE_RGB444 || rgb_type > MXC_CSI2_TYPE_RGB888) {
        return E_BAD_PARAM;
    }

    MXC_SETFIELD(csi2->vfifo_raw_ctrl, MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP, rgb_type);

    return E_NO_ERROR;
}

mxc_csi2_rgb_type_t MXC_CSI2_RevA_VFIFO_GetRGBType(mxc_csi2_reva_regs_t *csi2)
{
    int rgb_type;
    mxc_csi2_rgb_type_t result;

    rgb_type = csi2->vfifo_raw_ctrl & MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP;

    switch (rgb_type) {
    // RGB444
    case MXC_S_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_RGB444:
        result = MXC_CSI2_TYPE_RGB444;
        break;

    // RGB555
    case MXC_S_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_RGB555:
        result = MXC_CSI2_TYPE_RGB555;
        break;

    // RGB565
    case MXC_S_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_RGB565:
        result = MXC_CSI2_TYPE_RGB565;
        break;

    // RGB666
    case MXC_S_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_RGB666:
        result = MXC_CSI2_TYPE_RGB666;
        break;

    // RGB888
    case MXC_S_CSI2_REVA_VFIFO_RAW_CTRL_RGB_TYP_RGG888:
        result = MXC_CSI2_TYPE_RGB888;
        break;

    default:
        return E_BAD_PARAM;
    }

    return result;
}

int MXC_CSI2_RevA_VFIFO_SetRAWFormat(mxc_csi2_reva_regs_t *csi2, mxc_csi2_raw_format_t raw_format)
{
    // Check for valid format
    if (raw_format < MXC_CSI2_FORMAT_RGRG_GBGB || raw_format > MXC_CSI2_FORMAT_BGBG_GRGR) {
        return E_BAD_PARAM;
    }

    MXC_SETFIELD(csi2->vfifo_raw_ctrl, MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT, raw_format);

    return E_NO_ERROR;
}

mxc_csi2_raw_format_t MXC_CSI2_RevA_VFIFO_GetRAWFormat(mxc_csi2_reva_regs_t *csi2)
{
    int raw_format;
    mxc_csi2_raw_format_t result;

    raw_format = (csi2->vfifo_raw_ctrl & MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT) >>
                 MXC_F_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT_POS;

    switch (raw_format) {
    // RGRG_GBGB
    case MXC_V_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT_RGRG_GBGB:
        result = MXC_CSI2_FORMAT_RGRG_GBGB;
        break;

    // GRGR_BGBG
    case MXC_V_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT_GRGR_BGBG:
        result = MXC_CSI2_FORMAT_GRGR_BGBG;
        break;

    // GBGB_RGRG
    case MXC_V_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT_GBGB_RGRG:
        result = MXC_CSI2_FORMAT_GBGB_RGRG;
        break;

    // BGBG_GRGR
    case MXC_V_CSI2_REVA_VFIFO_RAW_CTRL_RAW_FMT_BGBG_GRGR:
        result = MXC_CSI2_FORMAT_BGBG_GRGR;
        break;

    default:
        return E_BAD_STATE;
    }

    return result;
}

int MXC_CSI2_RevA_VFIFO_GetFIFOEntityCount(mxc_csi2_reva_regs_t *csi2)
{
    return ((csi2->vfifo_sts & MXC_F_CSI2_VFIFO_STS_FELT) >> MXC_F_CSI2_VFIFO_STS_FELT_POS);
}

void MXC_CSI2_RevA_VFIFO_SetAHBWait(mxc_csi2_reva_regs_t *csi2, mxc_csi2_ahbwait_t wait_en)
{
    // Enable AHB Wait
    if (wait_en) {
        if (csi2_state.vfifo_cfg->wait_en == MXC_CSI2_AHBWAIT_DISABLE) {
            csi2_state.vfifo_cfg->wait_en = MXC_CSI2_AHBWAIT_ENABLE;
        }

        MXC_SETFIELD(csi2->vfifo_cfg0, MXC_F_CSI2_REVA_VFIFO_CFG0_AHBWAIT,
                     (1 << MXC_F_CSI2_REVA_VFIFO_CFG0_AHBWAIT_POS));

        // Disable AHB Wait
    } else {
        if (csi2_state.vfifo_cfg->wait_en == MXC_CSI2_AHBWAIT_ENABLE) {
            csi2_state.vfifo_cfg->wait_en = MXC_CSI2_AHBWAIT_DISABLE;
        }

        MXC_SETFIELD(csi2->vfifo_cfg0, MXC_F_CSI2_REVA_VFIFO_CFG0_AHBWAIT,
                     (0 << MXC_F_CSI2_REVA_VFIFO_CFG0_AHBWAIT_POS));
    }
}

mxc_csi2_ahbwait_t MXC_CSI2_RevA_VFIFO_GetAHBWait(mxc_csi2_reva_regs_t *csi2)
{
    int ahbwait;

    ahbwait = (csi2->vfifo_cfg0 & MXC_F_CSI2_REVA_VFIFO_CFG0_AHBWAIT) >>
              MXC_F_CSI2_REVA_VFIFO_CFG0_AHBWAIT_POS;

    if (ahbwait) {
        return MXC_CSI2_AHBWAIT_ENABLE;
    } else {
        return MXC_CSI2_AHBWAIT_DISABLE;
    }
}

/***********************************************/
/* CSI2 PHY Protocol Interface (PPI) Functions */
/***********************************************/

void MXC_CSI2_RevA_PPI_EnableInt(mxc_csi2_reva_regs_t *csi2, uint32_t mask)
{
    // Clear flags before enabling
    csi2->rx_eint_ppi_if |= mask;

    csi2->rx_eint_ppi_ie |= mask;
}

void MXC_CSI2_RevA_PPI_DisableInt(mxc_csi2_reva_regs_t *csi2, uint32_t mask)
{
    csi2->rx_eint_ppi_ie &= ~mask;
}

int MXC_CSI2_RevA_PPI_GetFlags(mxc_csi2_reva_regs_t *csi2)
{
    return (csi2->rx_eint_ppi_if);
}

void MXC_CSI2_RevA_PPI_ClearFlags(mxc_csi2_reva_regs_t *csi2, uint32_t flags)
{
    csi2->rx_eint_ppi_if |= flags;
}

int MXC_CSI2_RevA_PPI_Stop(void)
{
    MXC_CSI2_PPI_DisableInt(0xFFFFFFFF);

    return E_NO_ERROR;
}

/************************************/
/* CSI2 DMA - Used for all features */
/************************************/

// #define GPIO_INDICATOR
/* ^ This can be uncommented to enable a GPIO toggle as each row is received
from the camera.  Each edge corresponds to a received row.  Useful for
debugging timing issues.
*/
#ifdef GPIO_INDICATOR
#define GPIO_INDICATOR_PORT MXC_GPIO1
#define GPIO_INDICATOR_PIN MXC_GPIO_PIN_11
mxc_gpio_cfg_t indicator = { .func = MXC_GPIO_FUNC_OUT,
                             .port = GPIO_INDICATOR_PORT,
                             .mask = GPIO_INDICATOR_PIN,
                             .vssel = MXC_GPIO_VSSEL_VDDIOH,
                             .drvstr = MXC_GPIO_DRVSTR_0,
                             .pad = MXC_GPIO_PAD_NONE };
#endif

bool MXC_CSI2_RevA_DMA_Frame_Complete(void)
{
    return g_frame_complete;
}

mxc_csi2_reva_capture_stats_t MXC_CSI2_RevA_DMA_GetCaptureStats()
{
    return csi2_state.capture_stats;
}

void MXC_CSI2_RevA_DMA_Handler()
{
    // Clear CTZ Status Flag
    if (MXC_DMA->ch[csi2_state.dma_channel].status & MXC_F_DMA_STATUS_CTZ_IF) {
        MXC_DMA->ch[csi2_state.dma_channel].status |= MXC_F_DMA_STATUS_CTZ_IF;

        if (csi2_state.vfifo_cfg->dma_whole_frame != MXC_CSI2_DMA_WHOLE_FRAME) {
            // line by line
            line_cnt++;
#ifdef GPIO_INDICATOR
            MXC_GPIO_OutToggle(indicator.port, indicator.mask);
#endif
            if (line_cnt > csi2_state.req->lines_per_frame) {
                // Frame complete
                line_cnt = 0;
                MXC_CSI2_RevA_Stop((mxc_csi2_reva_regs_t *)MXC_CSI2);
                csi2_state.capture_stats.success = true;
                // TODO(Jake): Call frame complete handler here when multi-frame exposures are implemented.
            } else {
                // There is a line to process
                // Swap line buffers and reload DMA
                csi2_state.capture_stats.bytes_captured += line_byte_num;
                _swap_line_buffer();
                MXC_DMA->ch[csi2_state.dma_channel].cnt = line_byte_num;
                MXC_DMA->ch[csi2_state.dma_channel].dst = (uint32_t)lb.active;
                MXC_DMA->ch[csi2_state.dma_channel].ctrl |= MXC_F_DMA_REVA_CTRL_EN;

                if (csi2_state.req->line_handler != NULL) {
                    // Call line handler with a pointer to the inactive line buffer
                    int error = csi2_state.req->line_handler(lb.inactive, line_byte_num);
                    if (error)
                        MXC_CSI2_RevA_Stop((mxc_csi2_reva_regs_t *)MXC_CSI2);
                }
            }
        } else {
            // whole frame
            MXC_CSI2_RevA_Stop((mxc_csi2_reva_regs_t *)MXC_CSI2);
        }
    }
}

int MXC_CSI2_RevA_DMA_Config(uint8_t *dst_addr, uint32_t byte_cnt, uint32_t burst_size)
{
    int error;
    mxc_dma_config_t config;
    mxc_dma_srcdst_t srcdst;
    mxc_dma_adv_config_t advConfig = { csi2_state.dma_channel, 0, 0, 0, 0, 0 };

#ifdef GPIO_INDICATOR
    MXC_GPIO_Config(&indicator);
    MXC_GPIO_OutSet(indicator.port, indicator.mask);
#endif

    config.reqsel = MXC_DMA_REQUEST_CSI2RX;
    config.ch = csi2_state.dma_channel;
    config.srcwd = MXC_DMA_WIDTH_WORD;
    config.dstwd = MXC_DMA_WIDTH_WORD;
    config.srcinc_en = 0;
    config.dstinc_en = 1;

    advConfig.ch = csi2_state.dma_channel;
    advConfig.burst_size = burst_size;

    srcdst.ch = csi2_state.dma_channel;
    srcdst.source = MXC_CSI2_FIFO;
    srcdst.dest = dst_addr;
    srcdst.len = byte_cnt;

    error = MXC_DMA_ConfigChannel(config, srcdst);
    if (error != E_NO_ERROR) {
        return error;
    }

    error = MXC_DMA_SetChannelInterruptEn(csi2_state.dma_channel, false, true);
    if (error != E_NO_ERROR) {
        return error;
    }

    error = MXC_DMA_AdvConfigChannel(advConfig);
    if (error != E_NO_ERROR) {
        return error;
    }

    error = MXC_DMA_EnableInt(csi2_state.dma_channel);
    if (error != E_NO_ERROR) {
        return error;
    }

    MXC_NVIC_SetVector(GetIRQnForDMAChannel(csi2_state.dma_channel), MXC_CSI2_RevA_DMA_Handler);
    NVIC_EnableIRQ(GetIRQnForDMAChannel(csi2_state.dma_channel));

    return E_NO_ERROR;
}

int MXC_CSI2_RevA_DMA_GetChannel(void)
{
    return csi2_state.dma_channel;
}

int MXC_CSI2_RevA_DMA_GetCurrentLineCnt(void)
{
    return line_cnt;
}

int MXC_CSI2_RevA_DMA_GetCurrentFrameEndCnt(void)
{
    return frame_end_cnt;
}

#endif

/**@} end of group csi2 */
