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

/****** Includes *******/
#include <stddef.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "spixr.h"
#include "spixr_reva.h"

/****** Functions ******/

int MXC_SPIXR_RevA_ReadRXFIFO(mxc_spixr_reva_regs_t *spixr, uint8_t *buf, int len)
{
    int i;

    if (buf == NULL) {
        return E_NULL_PTR;
    }

    for (i = 0; i < len; i++) {
        buf[i] = spixr->data8[0];
    }

    return E_NO_ERROR;
}

int MXC_SPIXR_RevA_WriteTXFIFO(mxc_spixr_reva_regs_t *spixr, uint8_t *buf, int len)
{
    int i;

    if (buf == NULL) {
        return E_NULL_PTR;
    }

    for (i = 0; i < len; i++) {
        spixr->data8[0] = buf[i];
    }

    return E_NO_ERROR;
}

void MXC_SPIXR_RevA_SetSS(mxc_spixr_reva_regs_t *spixr, int ssIdx)
{
    MXC_SETFIELD(spixr->ctrl1, MXC_F_SPIXR_REVA_CTRL1_SS,
                 ((1 << ssIdx) << MXC_F_SPIXR_REVA_CTRL1_SS_POS));
}

int MXC_SPIXR_RevA_GetSS(mxc_spixr_reva_regs_t *spixr)
{
    return (spixr->ctrl1 & MXC_F_SPIXR_REVA_CTRL1_SS) >> MXC_F_SPIXR_REVA_CTRL1_SS_POS;
}

void MXC_SPIXR_RevA_SetSSCtrl(mxc_spixr_reva_regs_t *spixr, int stayActive)
{
    MXC_SETFIELD(spixr->ctrl1, MXC_F_SPIXR_REVA_CTRL1_SS_CTRL,
                 ((!!stayActive) << MXC_F_SPIXR_REVA_CTRL1_SS_CTRL_POS));
}

int MXC_SPIXR_RevA_GetSSCtrl(mxc_spixr_reva_regs_t *spixr)
{
    return (spixr->ctrl1 & MXC_F_SPIXR_REVA_CTRL1_SS_CTRL) >> MXC_F_SPIXR_REVA_CTRL1_SS_CTRL_POS;
}

void MXC_SPIXR_RevA_Enable(mxc_spixr_reva_regs_t *spixr)
{
    spixr->ctrl1 |= MXC_F_SPIXR_REVA_CTRL1_SPIEN;
    spixr->ctrl1 |= MXC_F_SPIXR_REVA_CTRL1_MMEN;
}

void MXC_SPIXR_RevA_Disable(mxc_spixr_reva_regs_t *spixr)
{
    while (MXC_SPIXR_Busy()) {}

    spixr->ctrl1 &= ~MXC_F_SPIXR_REVA_CTRL1_SPIEN;
    spixr->ctrl1 &= ~(MXC_F_SPIXR_REVA_CTRL1_MMEN);
}

int MXC_SPIXR_RevA_IsEnabled(mxc_spixr_reva_regs_t *spixr)
{
    return !!(spixr->ctrl1 & MXC_F_SPIXR_REVA_CTRL1_SPIEN);
}

void MXC_SPIXR_RevA_ThreeWireModeEnable(mxc_spixr_reva_regs_t *spixr)
{
    spixr->ctrl3 |= MXC_F_SPIXR_REVA_CTRL3_THREE_WIRE;
}

void MXC_SPIXR_RevA_ThreeWireModeDisable(mxc_spixr_reva_regs_t *spixr)
{
    while (MXC_SPIXR_Busy()) {}

    spixr->ctrl3 &= ~MXC_F_SPIXR_REVA_CTRL3_THREE_WIRE;
}

int MXC_SPIXR_RevA_ThreeWireModeIsEnabled(mxc_spixr_reva_regs_t *spixr)
{
    return !!(spixr->ctrl3 & MXC_F_SPIXR_REVA_CTRL3_THREE_WIRE);
}

int MXC_SPIXR_RevA_GetTXFIFOCount(mxc_spixr_reva_regs_t *spixr)
{
    return (spixr->dma & MXC_F_SPIXR_REVA_DMA_TX_FIFO_CNT) >> MXC_F_SPIXR_REVA_DMA_TX_FIFO_CNT_POS;
}

int MXC_SPIXR_RevA_GetRXFIFOCount(mxc_spixr_reva_regs_t *spixr)
{
    return (spixr->dma & MXC_F_SPIXR_REVA_DMA_RX_FIFO_CNT) >> MXC_F_SPIXR_REVA_DMA_RX_FIFO_CNT_POS;
}

int MXC_SPIXR_RevA_SetWidth(mxc_spixr_reva_regs_t *spixr, mxc_spixr_reva_width_t width)
{
    if (width == MXC_SPIXR_REVA_INVALID) {
        return E_BAD_PARAM;
    }

    MXC_SETFIELD(spixr->ctrl3, MXC_F_SPIXR_REVA_CTRL3_DATA_WIDTH,
                 (width << MXC_F_SPIXR_REVA_CTRL3_DATA_WIDTH_POS));

    return E_NO_ERROR;
}

int MXC_SPIXR_RevA_SetSPIMode(mxc_spixr_reva_regs_t *spixr, mxc_spixr_reva_mode_t mode)
{
    if (mode > MXC_SPIXR_REVA_MODE_3) {
        return E_BAD_PARAM;
    }

    if ((mode == MXC_SPIXR_REVA_MODE_0) || (mode == MXC_SPIXR_REVA_MODE_1)) {
        MXC_SETFIELD(spixr->ctrl3, MXC_F_SPIXR_REVA_CTRL3_CPOL,
                     (0 << MXC_F_SPIXR_REVA_CTRL3_CPOL_POS));

    } else {
        MXC_SETFIELD(spixr->ctrl3, MXC_F_SPIXR_REVA_CTRL3_CPOL,
                     (1 << MXC_F_SPIXR_REVA_CTRL3_CPOL_POS));
    }

    if ((mode == MXC_SPIXR_REVA_MODE_0) || (mode == MXC_SPIXR_REVA_MODE_2)) {
        MXC_SETFIELD(spixr->ctrl3, MXC_F_SPIXR_REVA_CTRL3_CPHA,
                     (0 << MXC_F_SPIXR_REVA_CTRL3_CPHA_POS));

    } else {
        MXC_SETFIELD(spixr->ctrl3, MXC_F_SPIXR_REVA_CTRL3_CPHA,
                     (1 << MXC_F_SPIXR_REVA_CTRL3_CPHA_POS));
    }

    return E_NO_ERROR;
}

int MXC_SPIXR_RevA_SetSSPolarity(mxc_spixr_reva_regs_t *spixr, int active)
{
    MXC_SETFIELD(spixr->ctrl3, MXC_F_SPIXR_REVA_CTRL3_SSPOL,
                 (active << MXC_F_SPIXR_REVA_CTRL3_SSPOL_POS));
    return E_NO_ERROR;
}

void MXC_SPIXR_RevA_SetSSTiming(mxc_spixr_reva_regs_t *spixr, unsigned int ssIActDelay,
                                unsigned int postActive, unsigned int preActive)
{
    MXC_ASSERT(ssIActDelay < 0x100 && postActive < 0x100 && preActive < 0x100);
    MXC_SETFIELD(spixr->ss_time, MXC_F_SPIXR_REVA_SS_TIME_SSACT1,
                 (preActive << MXC_F_SPIXR_REVA_SS_TIME_SSACT1_POS));
    MXC_SETFIELD(spixr->ss_time, MXC_F_SPIXR_REVA_SS_TIME_SSACT2,
                 (postActive << MXC_F_SPIXR_REVA_SS_TIME_SSACT2_POS));
    MXC_SETFIELD(spixr->ss_time, MXC_F_SPIXR_REVA_SS_TIME_SSINACT,
                 (ssIActDelay << MXC_F_SPIXR_REVA_SS_TIME_SSINACT_POS));
}

int MXC_SPIXR_RevA_SetFrequency(mxc_spixr_reva_regs_t *spixr, int hz)
{
    int freq_div, hi_clk, lo_clk, scale;

    // Check if frequency is too high
    if (hz > (int)PeripheralClock) {
        return E_BAD_PARAM;
    }

    // Set the clock high and low
    freq_div = PeripheralClock / (hz);
    hi_clk = freq_div / 2;
    lo_clk = freq_div / 2;
    scale = 0;

    if (freq_div % 2) {
        hi_clk += 1;
    }

    while (hi_clk >= 16 && scale < 8) {
        hi_clk /= 2;
        lo_clk /= 2;
        scale++;
    }

    spixr->brg_ctrl = (lo_clk << MXC_F_SPIXR_REVA_BRG_CTRL_LOW_POS) |
                      (hi_clk << MXC_F_SPIXR_REVA_BRG_CTRL_HI_POS) |
                      (scale << MXC_F_SPIXR_REVA_BRG_CTRL_SCALE_POS);

    return MXC_SPIXR_GetFrequency();
}

int MXC_SPIXR_RevA_GetFrequency(mxc_spixr_reva_regs_t *spixr)
{
    int spixr_periph_clock, scale, hi, lo;

    scale = ((spixr->brg_ctrl & MXC_F_SPIXR_REVA_BRG_CTRL_SCALE) >>
             MXC_F_SPIXR_REVA_BRG_CTRL_SCALE_POS);
    hi = ((spixr->brg_ctrl & MXC_F_SPIXR_REVA_BRG_CTRL_HI) >> MXC_F_SPIXR_REVA_BRG_CTRL_HI_POS);
    lo = ((spixr->brg_ctrl & MXC_F_SPIXR_REVA_BRG_CTRL_LOW) >> MXC_F_SPIXR_REVA_BRG_CTRL_LOW_POS);

    spixr_periph_clock = PeripheralClock / (1 << scale);

    if (hi == 0) {
        hi = 1;
    }

    if (lo == 0) {
        lo = 1;
    }

    return spixr_periph_clock / (hi + lo);
}

int MXC_SPIXR_RevA_GetIntFlags(mxc_spixr_reva_regs_t *spixr)
{
    return spixr->int_fl;
}

void MXC_SPIXR_RevA_EnableInt(mxc_spixr_reva_regs_t *spixr, int flags)
{
    spixr->int_en |= flags;
}

void MXC_SPIXR_RevA_DisableInt(mxc_spixr_reva_regs_t *spixr, int flags)
{
    spixr->int_en &= ~flags;
}

int MXC_SPIXR_RevA_GetWakeUpFlags(mxc_spixr_reva_regs_t *spixr)
{
    return spixr->wake_fl;
}

void MXC_SPIXR_RevA_EnableWakeUp(mxc_spixr_reva_regs_t *spixr, int flags)
{
    spixr->wake_en |= flags;
}

void MXC_SPIXR_RevA_DisableWakeUp(mxc_spixr_reva_regs_t *spixr, int flags)
{
    spixr->wake_en &= ~flags;
}

void MXC_SPIXR_RevA_ExMemEnable(mxc_spixr_reva_regs_t *spixr)
{
    MXC_SETFIELD(spixr->xmem_ctrl, MXC_F_SPIXR_REVA_XMEM_CTRL_XMEM_EN,
                 ((unsigned)1 << MXC_F_SPIXR_REVA_XMEM_CTRL_XMEM_EN_POS));
}

void MXC_SPIXR_RevA_ExMemDisable(mxc_spixr_reva_regs_t *spixr)
{
    MXC_SETFIELD(spixr->xmem_ctrl, MXC_F_SPIXR_REVA_XMEM_CTRL_XMEM_EN,
                 (0 << MXC_F_SPIXR_REVA_XMEM_CTRL_XMEM_EN_POS));
}

void MXC_SPIXR_RevA_ExMemUseDummy(mxc_spixr_reva_regs_t *spixr, int delay255)
{
    MXC_SETFIELD(spixr->xmem_ctrl, MXC_F_SPIXR_REVA_XMEM_CTRL_DUMMY_CLK,
                 (delay255 << MXC_F_SPIXR_REVA_XMEM_CTRL_DUMMY_CLK_POS));
}

void MXC_SPIXR_RevA_ExMemSetWriteCommand(mxc_spixr_reva_regs_t *spixr, uint8_t command)
{
    MXC_SETFIELD(spixr->xmem_ctrl, MXC_F_SPIXR_REVA_XMEM_CTRL_WR_CMD,
                 (command << MXC_F_SPIXR_REVA_XMEM_CTRL_WR_CMD_POS));
}

uint8_t MXC_SPIXR_RevA_ExMemGetWriteCommand(mxc_spixr_reva_regs_t *spixr)
{
    return ((spixr->xmem_ctrl & MXC_F_SPIXR_REVA_XMEM_CTRL_WR_CMD) >>
            MXC_F_SPIXR_REVA_XMEM_CTRL_WR_CMD_POS);
}

void MXC_SPIXR_RevA_ExMemSetReadCommand(mxc_spixr_reva_regs_t *spixr, uint8_t command)
{
    MXC_SETFIELD(spixr->xmem_ctrl, MXC_F_SPIXR_REVA_XMEM_CTRL_RD_CMD,
                 (command << MXC_F_SPIXR_REVA_XMEM_CTRL_RD_CMD_POS));
}

uint8_t MXC_SPIXR_RevA_ExMemGetReadCommand(mxc_spixr_reva_regs_t *spixr)
{
    return ((spixr->xmem_ctrl & MXC_F_SPIXR_REVA_XMEM_CTRL_RD_CMD) >>
            MXC_F_SPIXR_REVA_XMEM_CTRL_RD_CMD_POS);
}

int MXC_SPIXR_RevA_Busy(mxc_spixr_reva_regs_t *spixr)
{
    return (spixr->stat == MXC_F_SPIXR_REVA_STAT_BUSY);
}

int MXC_SPIXR_RevA_Init(mxc_spixr_reva_regs_t *spixr, mxc_spixr_reva_cfg_t *cfg)
{
    MXC_ASSERT(cfg);
    MXC_SPIXR_Enable();
    spixr->ctrl1 |= (1 << MXC_F_SPIXR_REVA_CTRL1_SS_POS);

    spixr->ctrl3 |= ((cfg->numbits) << MXC_F_SPIXR_REVA_CTRL3_NUMBITS_POS) |
                    ((cfg->data_width) << MXC_F_SPIXR_REVA_CTRL3_DATA_WIDTH_POS);

    spixr->ss_time = ((cfg->ssel_act_1) << MXC_F_SPIXR_REVA_SS_TIME_SSACT1_POS) |
                     ((cfg->ssel_act_2) << MXC_F_SPIXR_REVA_SS_TIME_SSACT2_POS) |
                     ((cfg->ssel_inact) << MXC_F_SPIXR_REVA_SS_TIME_SSINACT_POS);

    MXC_SPIXR_SetFrequency(cfg->baud_freq);

    return E_NO_ERROR;
}

int MXC_SPIXR_RevA_Shutdown(mxc_spixr_reva_regs_t *spixr)
{
    return E_NO_ERROR;
}

void MXC_SPIXR_RevA_SendCommand(mxc_spixr_reva_regs_t *spixr, uint8_t *cmd, uint32_t length,
                                uint32_t tx_num_char)
{
    uint32_t i;

    spixr->ctrl2 = ((tx_num_char) << MXC_F_SPIXR_REVA_CTRL2_TX_NUM_CHAR_POS) |
                   (spixr->ctrl2 & MXC_F_SPIXR_REVA_CTRL2_RX_NUM_CHAR);

    while (MXC_SPIXR_Busy()) {}

    for (i = 0; i < length; i++) {
        spixr->data8[0] = cmd[i];
    }

    spixr->ctrl1 |= MXC_F_SPIXR_REVA_CTRL1_TX_START; /* Send command to RAM */

    while (!(spixr->dma & MXC_F_SPIXR_REVA_DMA_TX_FIFO_CNT)) {}
    /* Wait for TXFIFO cnt to reach 0*/
}

void MXC_SPIXR_RevA_TXFIFOEnable(mxc_spixr_reva_regs_t *spixr)
{
    spixr->dma |= MXC_F_SPIXR_REVA_DMA_TX_FIFO_EN;
}

void MXC_SPIXR_RevA_TXFIFODisable(mxc_spixr_reva_regs_t *spixr)
{
    spixr->dma &= ~(MXC_F_SPIXR_REVA_DMA_TX_FIFO_EN);
}

int MXC_SPIXR_RevA_TXFIFOIsEnabled(mxc_spixr_reva_regs_t *spixr)
{
    return !!(spixr->dma & MXC_F_SPIXR_REVA_DMA_TX_FIFO_EN);
}

void MXC_SPIXR_RevA_DmaTXFIFOEnable(mxc_spixr_reva_regs_t *spixr)
{
    spixr->dma |= MXC_F_SPIXR_REVA_DMA_TX_DMA_EN;
}

void MXC_SPIXR_RevA_DmaTXFIFODisable(mxc_spixr_reva_regs_t *spixr)
{
    spixr->dma &= ~(MXC_F_SPIXR_REVA_DMA_TX_DMA_EN);
}

int MXC_SPIXR_RevA_DmaTXFIFOIsEnabled(mxc_spixr_reva_regs_t *spixr)
{
    return !!(spixr->dma & MXC_F_SPIXR_REVA_DMA_TX_DMA_EN);
}

void MXC_SPIXR_RevA_RXFIFOEnable(mxc_spixr_reva_regs_t *spixr)
{
    spixr->dma |= MXC_F_SPIXR_REVA_DMA_RX_FIFO_EN;
}

void MXC_SPIXR_RevA_RXFIFODisable(mxc_spixr_reva_regs_t *spixr)
{
    spixr->dma &= ~(MXC_F_SPIXR_REVA_DMA_RX_FIFO_EN);
}

int MXC_SPIXR_RevA_RXFIFOIsEnabled(mxc_spixr_reva_regs_t *spixr)
{
    return !!(spixr->dma & MXC_F_SPIXR_REVA_DMA_RX_FIFO_EN);
}

void MXC_SPIXR_RevA_DmaRXFIFOEnable(mxc_spixr_reva_regs_t *spixr)
{
    spixr->dma |= MXC_F_SPIXR_REVA_DMA_RX_DMA_EN;
}

void MXC_SPIXR_RevA_DmaRXFIFODisable(mxc_spixr_reva_regs_t *spixr)
{
    spixr->dma &= ~(MXC_F_SPIXR_REVA_DMA_RX_DMA_EN);
}

int MXC_SPIXR_RevA_DmaRXFIFOIsEnabled(mxc_spixr_reva_regs_t *spixr)
{
    return !!(spixr->dma & MXC_F_SPIXR_REVA_DMA_RX_DMA_EN);
}

void MXC_SPIXR_RevA_TXFIFOClear(mxc_spixr_reva_regs_t *spixr)
{
    spixr->dma |= MXC_F_SPIXR_REVA_DMA_TX_FIFO_CLEAR;
}

void MXC_SPIXR_RevA_RXFIFOClear(mxc_spixr_reva_regs_t *spixr)
{
    spixr->dma |= MXC_F_SPIXR_REVA_DMA_RX_FIFO_CLEAR;
}
