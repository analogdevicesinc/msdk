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

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "spi.h"
#include "spi_reva1.h"
#include "dma_reva.h"

/* **** Definitions **** */
typedef struct {
    mxc_spi_reva_req_t *req;
    int started;
    unsigned last_size;
    unsigned ssDeassert;
    unsigned defaultTXData;
    int channelTx;
    int channelRx;
    int mtMode;
    int mtFirstTrans;
    bool txrx_req;
    uint8_t req_done;
    uint8_t async;
    bool hw_ss_control;
} spi_req_reva_state_t;

static spi_req_reva_state_t states[MXC_SPI_INSTANCES];

static uint32_t MXC_SPI_RevA1_MasterTransHandler(mxc_spi_reva_regs_t *spi, mxc_spi_reva_req_t *req);
static uint32_t MXC_SPI_RevA1_TransHandler(mxc_spi_reva_regs_t *spi, mxc_spi_reva_req_t *req);
static uint32_t MXC_SPI_RevA1_SlaveTransHandler(mxc_spi_reva_req_t *req);
static void MXC_SPI_RevA1_SwapByte(uint8_t *arr, size_t length);
static int MXC_SPI_RevA1_TransSetup(mxc_spi_reva_req_t *req);

int MXC_SPI_RevA1_Init(mxc_spi_reva_regs_t *spi, int masterMode, int quadModeUsed, int numSlaves,
                       unsigned ssPolarity, unsigned int hz)
{
    int spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    /*  Validate that the specified SPI instance exists.
        Note:  The Init function is the only one that performs this check.
        It catches the rare edge case where a user has casted
        a custom SPI regs struct with an incorrect base address.
     */
    if (spi_num < 0)
        return E_BAD_PARAM;

    states[spi_num].req = NULL;
    states[spi_num].last_size = 0;
    states[spi_num].ssDeassert = 1;
    states[spi_num].defaultTXData = 0;
    states[spi_num].mtMode = 0;
    states[spi_num].mtFirstTrans = 0;
    states[spi_num].channelTx = E_NO_DEVICE;
    states[spi_num].channelRx = E_NO_DEVICE;
    states[spi_num].hw_ss_control = true;

    spi->ctrl0 = (MXC_F_SPI_REVA_CTRL0_EN);
    spi->sstime =
        ((0x1 << MXC_F_SPI_REVA_SSTIME_PRE_POS) | (0x1 << MXC_F_SPI_REVA_SSTIME_POST_POS) |
         (0x1 << MXC_F_SPI_REVA_SSTIME_INACT_POS));

    //set master
    if (masterMode) {
        spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_MST_MODE;
    } else {
        spi->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL0_MST_MODE);
    }

    MXC_SPI_SetFrequency((mxc_spi_regs_t *)spi, hz);

    if (ssPolarity > (MXC_F_SPI_REVA_CTRL2_SS_POL >> MXC_F_SPI_REVA_CTRL2_SS_POL_POS)) {
        return E_BAD_PARAM;
    }

    //set slave select polarity
    spi->ctrl2 |= (ssPolarity << MXC_F_SPI_REVA_CTRL2_SS_POL_POS);

    // Clear the interrupts
    spi->intfl = spi->intfl;

    if (states[spi_num].hw_ss_control) {
        if (numSlaves == 1) {
            spi->ctrl0 |= MXC_S_SPI_REVA_CTRL0_SS_ACTIVE_SS0;
        }

        if (numSlaves == 2) {
            spi->ctrl0 |= (MXC_S_SPI_REVA_CTRL0_SS_ACTIVE_SS0 | MXC_S_SPI_REVA_CTRL0_SS_ACTIVE_SS1);
        }

        if (numSlaves == 3) {
            spi->ctrl0 |= (MXC_S_SPI_REVA_CTRL0_SS_ACTIVE_SS0 | MXC_S_SPI_REVA_CTRL0_SS_ACTIVE_SS1 |
                           MXC_S_SPI_REVA_CTRL0_SS_ACTIVE_SS2);
        }

        if (numSlaves == 4) {
            spi->ctrl0 |= (MXC_S_SPI_REVA_CTRL0_SS_ACTIVE_SS0 | MXC_S_SPI_REVA_CTRL0_SS_ACTIVE_SS1 |
                           MXC_S_SPI_REVA_CTRL0_SS_ACTIVE_SS2 | MXC_S_SPI_REVA_CTRL0_SS_ACTIVE_SS3);
        }
    }

    //set quad mode
    if (quadModeUsed) {
        spi->ctrl2 |= MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_QUAD;
    }

    return E_NO_ERROR;
}

int MXC_SPI_RevA1_Shutdown(mxc_spi_reva_regs_t *spi)
{
    mxc_spi_reva_req_t *temp_req;
    int spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    //disable and clear interrupts
    spi->inten = 0;
    spi->intfl = spi->intfl;

    // Disable SPI and FIFOS
    spi->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL0_EN);
    spi->dma &= ~(MXC_F_SPI_REVA_DMA_TX_FIFO_EN | MXC_F_SPI_REVA_DMA_RX_FIFO_EN);

    if (states[spi_num].req != NULL) {
        //save the request
        temp_req = states[spi_num].req;
        MXC_FreeLock((uint32_t *)(uint32_t *)&states[spi_num].req);

        // Callback if not NULL
        if (states[spi_num].req->completeCB != NULL) {
            states[spi_num].req->completeCB(temp_req, E_SHUTDOWN);
        }
    }

    // Clear registers
    spi->ctrl0 = 0;
    spi->ctrl1 = 0;
    spi->ctrl2 = 0;
    spi->sstime = 0;

    // release any acquired DMA channels
    if (states[spi_num].channelTx >= 0) {
        MXC_DMA_RevA_ReleaseChannel(states[spi_num].channelTx);
        states[spi_num].channelTx = E_NO_DEVICE;
    }
    if (states[spi_num].channelRx >= 0) {
        MXC_DMA_RevA_ReleaseChannel(states[spi_num].channelRx);
        states[spi_num].channelRx = E_NO_DEVICE;
    }

    return E_NO_ERROR;
}

int MXC_SPI_RevA1_ReadyForSleep(mxc_spi_reva_regs_t *spi)
{
    if (spi->stat & MXC_F_SPI_REVA_STAT_BUSY || (spi->dma & MXC_F_SPI_REVA_DMA_TX_LVL) ||
        (spi->dma & MXC_F_SPI_REVA_DMA_RX_LVL)) {
        return E_BUSY;
    } else {
        return E_NO_ERROR;
    }
}

int MXC_SPI_RevA1_SetFrequency(mxc_spi_reva_regs_t *spi, unsigned int hz)
{
    int hi_clk, lo_clk, scale;
    uint32_t freq_div;

    // Check if frequency is too high
    if (hz > PeripheralClock) {
        return E_BAD_PARAM;
    }

    // Set the clock high and low
    freq_div = MXC_SPI_GetPeripheralClock((mxc_spi_regs_t *)spi);
    freq_div = (freq_div / hz);

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

    if (scale == 8) {
        lo_clk = 15;
        hi_clk = 15;
    }

    MXC_SETFIELD(spi->clkctrl, MXC_F_SPI_REVA_CLKCTRL_LO,
                 (lo_clk << MXC_F_SPI_REVA_CLKCTRL_LO_POS));
    MXC_SETFIELD(spi->clkctrl, MXC_F_SPI_REVA_CLKCTRL_HI,
                 (hi_clk << MXC_F_SPI_REVA_CLKCTRL_HI_POS));
    MXC_SETFIELD(spi->clkctrl, MXC_F_SPI_REVA_CLKCTRL_CLKDIV,
                 (scale << MXC_F_SPI_REVA_CLKCTRL_CLKDIV_POS));

    return E_NO_ERROR;
}

unsigned int MXC_SPI_RevA1_GetFrequency(mxc_spi_reva_regs_t *spi)
{
    if (MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi) < 0) {
        // Can't return error code (negative values) due to return type.
        //  Return 0Hz instead.
        return 0;
    }

    unsigned scale, lo_clk, hi_clk;

    scale = (spi->clkctrl & MXC_F_SPI_REVA_CLKCTRL_CLKDIV) >> MXC_F_SPI_REVA_CLKCTRL_CLKDIV_POS;
    hi_clk = (spi->clkctrl & MXC_F_SPI_REVA_CLKCTRL_HI) >> MXC_F_SPI_REVA_CLKCTRL_HI_POS;
    lo_clk = (spi->clkctrl & MXC_F_SPI_REVA_CLKCTRL_LO) >> MXC_F_SPI_REVA_CLKCTRL_LO_POS;

    return (PeripheralClock / (1 << scale)) / (lo_clk + hi_clk);
}

int MXC_SPI_RevA1_SetDataSize(mxc_spi_reva_regs_t *spi, int dataSize)
{
    // HW has problem with these two character sizes
    if (dataSize == 1 || dataSize > 16) {
        return E_BAD_PARAM;
    }

    int spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    // Setup the character size
    if (!(spi->stat & MXC_F_SPI_REVA_STAT_BUSY) && states[spi_num].ssDeassert == 1) {
        //disable spi to change transfer size
        spi->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL0_EN);
        // set bit size
        states[spi_num].last_size = dataSize;

        if (dataSize < 16) {
            MXC_SETFIELD(spi->ctrl2, MXC_F_SPI_REVA_CTRL2_NUMBITS,
                         dataSize << MXC_F_SPI_REVA_CTRL2_NUMBITS_POS);
        } else {
            MXC_SETFIELD(spi->ctrl2, MXC_F_SPI_REVA_CTRL2_NUMBITS,
                         0 << MXC_F_SPI_REVA_CTRL2_NUMBITS_POS); //may not be neccessary
        }

        //enable spi to change transfer size
        spi->ctrl0 |= (MXC_F_SPI_REVA_CTRL0_EN);
    } else {
        return E_BAD_STATE;
    }

    return E_NO_ERROR;
}

int MXC_SPI_RevA1_GetDataSize(mxc_spi_reva_regs_t *spi)
{
    if (!(spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_NUMBITS)) {
        return 16;
    }

    return (spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_NUMBITS) >> MXC_F_SPI_REVA_CTRL2_NUMBITS_POS;
}

int MXC_SPI_RevA1_SetMTMode(mxc_spi_reva_regs_t *spi, int mtMode)
{
    int spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    if ((mtMode != 0) && (mtMode != 1)) {
        return E_BAD_PARAM;
    }

    if (states[spi_num].mtMode == 1) {
        if (mtMode == 0) {
            // exiting MT Mode: release any acquired DMA channels
            if (states[spi_num].channelTx >= 0) {
                MXC_DMA_RevA_ReleaseChannel(states[spi_num].channelTx);
                states[spi_num].channelTx = E_NO_DEVICE;
            }
            if (states[spi_num].channelRx >= 0) {
                MXC_DMA_RevA_ReleaseChannel(states[spi_num].channelRx);
                states[spi_num].channelRx = E_NO_DEVICE;
            }
        }
    } else if (mtMode == 1) {
        // entering MT Mode: set first transaction
        states[spi_num].mtFirstTrans = 1;
    }

    states[spi_num].mtMode = mtMode;

    return E_NO_ERROR;
}

int MXC_SPI_RevA1_GetMTMode(mxc_spi_reva_regs_t *spi)
{
    int spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    return states[spi_num].mtMode;
}

int MXC_SPI_RevA1_SetSlave(mxc_spi_reva_regs_t *spi, int ssIdx)
{
    // HW has problem with these two character sizes
    if (ssIdx >= MXC_SPI_SS_INSTANCES) {
        return E_BAD_PARAM;
    }

    //check if in master mode
    if (!(spi->ctrl0 & MXC_F_SPI_REVA_CTRL0_MST_MODE)) {
        return E_BAD_STATE;
    }

    int spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    if (states[spi_num].hw_ss_control) {
        // Setup the slave select
        // Activate chosen SS pin
        spi->ctrl0 |= (1 << ssIdx) << MXC_F_SPI_REVA_CTRL0_SS_ACTIVE_POS;
        // Deactivate all unchosen pins
        spi->ctrl0 &= ~MXC_F_SPI_REVA_CTRL0_SS_ACTIVE |
                      ((1 << ssIdx) << MXC_F_SPI_REVA_CTRL0_SS_ACTIVE_POS);
    }
    return E_NO_ERROR;
}

int MXC_SPI_RevA1_GetSlave(mxc_spi_reva_regs_t *spi)
{
    return ((spi->ctrl0 & MXC_F_SPI_REVA_CTRL0_SS_ACTIVE) >> MXC_F_SPI_REVA_CTRL0_SS_ACTIVE_POS) >>
           1;
}

int MXC_SPI_RevA1_SetWidth(mxc_spi_reva_regs_t *spi, mxc_spi_reva_width_t spiWidth)
{
    spi->ctrl2 &= ~(MXC_F_SPI_REVA_CTRL2_THREE_WIRE | MXC_F_SPI_REVA_CTRL2_DATA_WIDTH);

    switch (spiWidth) {
    case SPI_REVA_WIDTH_3WIRE:
        spi->ctrl2 |= MXC_F_SPI_REVA_CTRL2_THREE_WIRE;
        break;

    case SPI_REVA_WIDTH_STANDARD:
        spi->ctrl2 |= MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_MONO;
        break;

    case SPI_REVA_WIDTH_DUAL:
        spi->ctrl2 |= MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_DUAL;
        break;

    case SPI_REVA_WIDTH_QUAD:
        spi->ctrl2 |= MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_QUAD;
        break;
    }

    return E_NO_ERROR;
}

mxc_spi_reva_width_t MXC_SPI_RevA1_GetWidth(mxc_spi_reva_regs_t *spi)
{
    if (spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_THREE_WIRE) {
        return SPI_REVA_WIDTH_3WIRE;
    }

    if (spi->ctrl2 & MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_DUAL) {
        return SPI_REVA_WIDTH_DUAL;
    }

    if (spi->ctrl2 & MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_QUAD) {
        return SPI_REVA_WIDTH_QUAD;
    }

    return SPI_REVA_WIDTH_STANDARD;
}

int MXC_SPI_RevA1_SetMode(mxc_spi_reva_regs_t *spi, mxc_spi_reva_mode_t spiMode)
{
    switch (spiMode) {
    case SPI_REVA_MODE_0:
        spi->ctrl2 &= ~MXC_F_SPI_REVA_CTRL2_CLKPHA;
        spi->ctrl2 &= ~MXC_F_SPI_REVA_CTRL2_CLKPOL;
        break;

    case SPI_REVA_MODE_1:
        spi->ctrl2 |= MXC_F_SPI_REVA_CTRL2_CLKPHA;
        spi->ctrl2 &= ~MXC_F_SPI_REVA_CTRL2_CLKPOL;
        break;

    case SPI_REVA_MODE_2:
        spi->ctrl2 &= ~MXC_F_SPI_REVA_CTRL2_CLKPHA;
        spi->ctrl2 |= MXC_F_SPI_REVA_CTRL2_CLKPOL;
        break;

    case SPI_REVA_MODE_3:
        spi->ctrl2 |= MXC_F_SPI_REVA_CTRL2_CLKPHA;
        spi->ctrl2 |= MXC_F_SPI_REVA_CTRL2_CLKPOL;
        break;

    default:
        break;
    }

    return E_NO_ERROR;
}

mxc_spi_reva_mode_t MXC_SPI_RevA1_GetMode(mxc_spi_reva_regs_t *spi)
{
    if (spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_CLKPHA) {
        if (spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_CLKPOL) {
            return SPI_REVA_MODE_3;
        } else {
            return SPI_REVA_MODE_2;
        }
    } else {
        if (spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_CLKPOL) {
            return SPI_REVA_MODE_1;
        }
    }

    return SPI_REVA_MODE_0;
}

int MXC_SPI_RevA1_StartTransmission(mxc_spi_reva_regs_t *spi)
{
    if (MXC_SPI_GetActive((mxc_spi_regs_t *)spi) == E_BUSY) {
        return E_BUSY;
    }

    spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_START;

    return E_NO_ERROR;
}

int MXC_SPI_RevA1_GetActive(mxc_spi_reva_regs_t *spi)
{
    if (spi->stat & MXC_F_SPI_REVA_STAT_BUSY) {
        return E_BUSY;
    }

    return E_NO_ERROR;
}

int MXC_SPI_RevA1_AbortTransmission(mxc_spi_reva_regs_t *spi)
{
    int spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    // Disable interrupts, clear the flags
    spi->inten = 0;
    spi->intfl = spi->intfl;

    // Reset the SPI17Y to cancel the on ongoing transaction
    spi->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL0_EN);
    spi->ctrl0 |= (MXC_F_SPI_REVA_CTRL0_EN);

    // Unlock this SPI
    mxc_spi_reva_req_t *temp = states[spi_num].req;
    MXC_FreeLock((uint32_t *)&states[spi_num].req);

    // Callback if not NULL
    if (temp->completeCB != NULL) {
        temp->completeCB(states[spi_num].req, E_ABORT);
    }

    // release any acquired DMA channels
    if (states[spi_num].channelTx >= 0) {
        MXC_DMA_RevA_ReleaseChannel(states[spi_num].channelTx);
        states[spi_num].channelTx = E_NO_DEVICE;
    }
    if (states[spi_num].channelRx >= 0) {
        MXC_DMA_RevA_ReleaseChannel(states[spi_num].channelRx);
        states[spi_num].channelRx = E_NO_DEVICE;
    }
    if (states[spi_num].mtMode == 1) {
        states[spi_num].mtFirstTrans = 1;
    }

    return E_NO_ERROR;
}

unsigned int MXC_SPI_RevA1_ReadRXFIFO(mxc_spi_reva_regs_t *spi, unsigned char *bytes,
                                      unsigned int len)
{
    unsigned rx_avail, bits;

    if (!bytes || !len) {
        return 0;
    }

    rx_avail = MXC_SPI_GetRXFIFOAvailable((mxc_spi_regs_t *)spi);
    bits = MXC_SPI_GetDataSize((mxc_spi_regs_t *)spi);

    if (len > rx_avail) {
        len = rx_avail;
    }

    if (bits > 8) {
        len &= ~(unsigned)0x1;
    }

    unsigned count = 0;

    // Read from the FIFO
    while (len) {
        // Reading 2-8 bit wide messages from the FIFO16 or FIFO32 register
        //  swaps the ordering of the message.
        //  Example:
        //      When the SPI FIFO receives the message '00, 01, 02, 03', reading the FIFO16
        //      or FIFO32 register could result in '01, 00, 03, 02' - depending on the part.
        if (bits > 8) {
            if (len > 3) {
                memcpy((uint8_t *)(&bytes[count]), (void *)(&spi->fifo32), 4);
                len -= 4;
                count += 4;
            } else if (len > 1) {
                memcpy((uint8_t *)(&bytes[count]), (void *)(&spi->fifo16[0]), 2);
                len -= 2;
                count += 2;
            }

            // Don't read less than 2 bytes if we are using greater than 8 bit wide messages.
            //  Due to the nature of how this function is called in the drivers, it should never
            //  reach to this point.
            if (len == 1) {
                break;
            }

            // 9-16 bit wide messages should not be read from the FIFO8 register (cuts
            //  off the upper byte).
        } else {
            ((uint8_t *)bytes)[count++] = spi->fifo8[0];
            len -= 1;
        }
    }

    return count;
}

unsigned int MXC_SPI_RevA1_GetRXFIFOAvailable(mxc_spi_reva_regs_t *spi)
{
    return (spi->dma & MXC_F_SPI_REVA_DMA_RX_LVL) >> MXC_F_SPI_REVA_DMA_RX_LVL_POS;
}

unsigned int MXC_SPI_RevA1_WriteTXFIFO(mxc_spi_reva_regs_t *spi, unsigned char *bytes,
                                       unsigned int len)
{
    unsigned tx_avail, bits;

    if (!bytes || !len) {
        return 0;
    }

    tx_avail = MXC_SPI_GetTXFIFOAvailable((mxc_spi_regs_t *)spi);
    bits = MXC_SPI_GetDataSize((mxc_spi_regs_t *)spi);

    if (len > tx_avail) {
        len = tx_avail;
    }

    if (bits > 8) {
        len &= ~(unsigned)0x1;
    }

    unsigned count = 0;

    while (len) {
        // Writing 2-8 bit wide messages to the FIFO16 or FIFO32 register
        //  swaps the ordering of the message.
        //  Example:
        //      SPI FIFO is expected to transmit the message '00, 01, 02, 03'.
        //      Writing the four byte-wide characters to the FIFO16 or FIFO32 register could
        //      result in this message shifted out: '01, 00, 03, 02' - depending on the part.
        if (bits > 8) {
            if (len > 3) {
                memcpy((void *)(&spi->fifo32), (uint8_t *)(&bytes[count]), 4);
                len -= 4;
                count += 4;
            } else if (len > 1) {
                memcpy((void *)(&spi->fifo16[0]), (uint8_t *)(&bytes[count]), 2);
                len -= 2;
                count += 2;
            }

            // 9-16 bit wide messages should not be written to the FIFO8 register (cuts
            //  off the upper byte).
        } else {
            spi->fifo8[0] = ((uint8_t *)bytes)[count++];
            len -= 1;
        }
    }

    return count;
}

unsigned int MXC_SPI_RevA1_GetTXFIFOAvailable(mxc_spi_reva_regs_t *spi)
{
    return MXC_SPI_FIFO_DEPTH -
           ((spi->dma & MXC_F_SPI_REVA_DMA_TX_LVL) >> MXC_F_SPI_REVA_DMA_TX_LVL_POS);
}

void MXC_SPI_RevA1_ClearRXFIFO(mxc_spi_reva_regs_t *spi)
{
    spi->dma |= MXC_F_SPI_REVA_DMA_RX_FLUSH;
}

void MXC_SPI_RevA1_ClearTXFIFO(mxc_spi_reva_regs_t *spi)
{
    spi->dma |= MXC_F_SPI_REVA_DMA_TX_FLUSH;
}

int MXC_SPI_RevA1_SetRXThreshold(mxc_spi_reva_regs_t *spi, unsigned int numBytes)
{
    if (numBytes > 30) {
        return E_BAD_PARAM;
    }

    MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_RX_THD_VAL,
                 numBytes << MXC_F_SPI_REVA_DMA_RX_THD_VAL_POS);

    return E_NO_ERROR;
}

unsigned int MXC_SPI_RevA1_GetRXThreshold(mxc_spi_reva_regs_t *spi)
{
    return (spi->dma & MXC_F_SPI_REVA_DMA_RX_THD_VAL) >> MXC_F_SPI_REVA_DMA_RX_THD_VAL_POS;
}

int MXC_SPI_RevA1_SetTXThreshold(mxc_spi_reva_regs_t *spi, unsigned int numBytes)
{
    // Valid values for the threshold are 0x1 to 0x1F
    if (numBytes > 31 || numBytes == 0) {
        return E_BAD_PARAM;
    }

    MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_TX_THD_VAL,
                 numBytes << MXC_F_SPI_REVA_DMA_TX_THD_VAL_POS);

    return E_NO_ERROR;
}

unsigned int MXC_SPI_RevA1_GetTXThreshold(mxc_spi_reva_regs_t *spi)
{
    return (spi->dma & MXC_F_SPI_REVA_DMA_TX_THD_VAL) >> MXC_F_SPI_REVA_DMA_TX_THD_VAL_POS;
}

unsigned int MXC_SPI_RevA1_GetFlags(mxc_spi_reva_regs_t *spi)
{
    return spi->intfl;
}

void MXC_SPI_RevA1_ClearFlags(mxc_spi_reva_regs_t *spi)
{
    spi->intfl = spi->intfl;
}

void MXC_SPI_RevA1_EnableInt(mxc_spi_reva_regs_t *spi, unsigned int intEn)
{
    spi->inten |= intEn;
}

void MXC_SPI_RevA1_DisableInt(mxc_spi_reva_regs_t *spi, unsigned int intDis)
{
    spi->inten &= ~(intDis);
}

int MXC_SPI_RevA1_TransSetup(mxc_spi_reva_req_t *req)
{
    int spi_num;
    uint8_t bits;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)(req->spi));

    MXC_ASSERT(req->ssIdx < MXC_SPI_SS_INSTANCES);

    if ((!req) || ((req->txData == NULL) && (req->rxData == NULL))) {
        return E_BAD_PARAM;
    }

    // Setup the number of characters to transact
    if (req->txLen > (MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR >> MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS)) {
        return E_BAD_PARAM;
    }

    bits = MXC_SPI_GetDataSize((mxc_spi_regs_t *)req->spi);
    req->txCnt = 0;
    req->rxCnt = 0;

    states[spi_num].req = req;
    states[spi_num].started = 0;
    states[spi_num].req_done = 0;
    // HW requires disabling/renabling SPI block at end of each transaction (when SS is inactive).
    if (states[spi_num].ssDeassert == 1) {
        (req->spi)->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL0_EN);
    }

    //if  master
    if ((req->spi)->ctrl0 & MXC_F_SPI_REVA_CTRL0_MST_MODE) {
        // Setup the slave select
        MXC_SPI_SetSlave((mxc_spi_regs_t *)req->spi, req->ssIdx);
    }

    if (req->rxData != NULL && req->rxLen > 0) {
        MXC_SETFIELD((req->spi)->ctrl1, MXC_F_SPI_REVA_CTRL1_RX_NUM_CHAR,
                     req->rxLen << MXC_F_SPI_REVA_CTRL1_RX_NUM_CHAR_POS);
        (req->spi)->dma |= MXC_F_SPI_REVA_DMA_RX_FIFO_EN;
    } else {
        (req->spi)->ctrl1 &= ~(MXC_F_SPI_REVA_CTRL1_RX_NUM_CHAR);
        (req->spi)->dma &= ~(MXC_F_SPI_REVA_DMA_RX_FIFO_EN);
    }

    // Must use TXFIFO and NUM in full duplex//start  editing here
    if ((mxc_spi_reva_width_t)MXC_SPI_GetWidth((mxc_spi_regs_t *)req->spi) ==
            SPI_REVA_WIDTH_STANDARD &&
        !(((req->spi)->ctrl2 & MXC_F_SPI_REVA_CTRL2_THREE_WIRE) >>
          MXC_F_SPI_REVA_CTRL2_THREE_WIRE_POS)) {
        if (req->txData == NULL) {
            // Must have something to send, so we'll use the rx_data buffer initialized to 0.
            //SPI_SetDefaultTXData(spi, 0);
            memset(req->rxData, states[spi_num].defaultTXData,
                   (bits > 8 ? req->rxLen << 1 : req->rxLen));
            req->txData = req->rxData;
            req->txLen = req->rxLen;
        }
    }

    if (req->txData != NULL && req->txLen > 0) {
        MXC_SETFIELD((req->spi)->ctrl1, MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR,
                     req->txLen << MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS);
        (req->spi)->dma |= MXC_F_SPI_REVA_DMA_TX_FIFO_EN;
    } else {
        (req->spi)->ctrl1 &= ~(MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR);
        (req->spi)->dma &= ~(MXC_F_SPI_REVA_DMA_TX_FIFO_EN);
    }

    if ((req->txData != NULL && req->txLen) && (req->rxData != NULL && req->rxLen)) {
        states[spi_num].txrx_req = true;
    } else {
        states[spi_num].txrx_req = false;
    }

    (req->spi)->dma |= (MXC_F_SPI_REVA_DMA_TX_FLUSH | MXC_F_SPI_REVA_DMA_RX_FLUSH);
    (req->spi)->ctrl0 |= (MXC_F_SPI_REVA_CTRL0_EN);

    states[spi_num].ssDeassert = req->ssDeassert;
    // Clear master done flag
    (req->spi)->intfl = MXC_F_SPI_REVA_INTFL_MST_DONE;

    return E_NO_ERROR;
}

uint32_t MXC_SPI_RevA1_MasterTransHandler(mxc_spi_reva_regs_t *spi, mxc_spi_reva_req_t *req)
{
    uint32_t retval;
    int spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    // Leave slave select asserted at the end of the transaction
    if (states[spi_num].hw_ss_control && !req->ssDeassert) {
        spi->ctrl0 = (spi->ctrl0 & ~MXC_F_SPI_REVA_CTRL0_START) | MXC_F_SPI_REVA_CTRL0_SS_CTRL;
        // Note: Setting 0 to START bit to avoid race condition and duplicated starts.
        // See https://github.com/analogdevicesinc/msdk/issues/713
    }

    retval = MXC_SPI_RevA1_TransHandler(spi, req);

    if (!states[spi_num].started) {
        MXC_SPI_StartTransmission((mxc_spi_regs_t *)spi);
        states[spi_num].started = 1;
    }

    // Deassert slave select at the end of the transaction
    if (states[spi_num].hw_ss_control && req->ssDeassert) {
        spi->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL0_START | MXC_F_SPI_REVA_CTRL0_SS_CTRL);
    }

    return retval;
}

uint32_t MXC_SPI_RevA1_SlaveTransHandler(mxc_spi_reva_req_t *req)
{
    return MXC_SPI_RevA1_TransHandler(req->spi, req);
}

uint32_t MXC_SPI_RevA1_TransHandler(mxc_spi_reva_regs_t *spi, mxc_spi_reva_req_t *req)
{
    int remain, spi_num;
    uint32_t int_en = 0;
    uint32_t tx_length = 0, rx_length = 0;
    uint8_t bits;
    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    bits = MXC_SPI_GetDataSize((mxc_spi_regs_t *)req->spi);

    //MXC_F_SPI_REVA_CTRL2_NUMBITS data bits
    // Read/write 2x number of bytes if larger character size
    if (bits > 8) {
        tx_length = req->txLen * 2;
        rx_length = req->rxLen * 2;
    } else {
        tx_length = req->txLen;
        rx_length = req->rxLen;
    }

    if (req->txData != NULL) {
        req->txCnt += MXC_SPI_WriteTXFIFO((mxc_spi_regs_t *)spi, &(req->txData[req->txCnt]),
                                          tx_length - req->txCnt);
    }

    remain = tx_length - req->txCnt;

    // Set the TX interrupts
    // Write the FIFO //starting here
    if (remain) {
        if (remain >= MXC_SPI_FIFO_DEPTH) {
            MXC_SPI_SetTXThreshold((mxc_spi_regs_t *)spi, MXC_SPI_FIFO_DEPTH - 1);
        } else {
            MXC_SPI_SetTXThreshold((mxc_spi_regs_t *)spi, remain);
        }

        int_en |= MXC_F_SPI_REVA_INTEN_TX_THD;
    }
    // Break out if we've transmitted all the bytes and not receiving
    if ((req->rxData == NULL) && (req->txCnt == tx_length)) {
        spi->inten = 0;
        int_en = 0;
        MXC_FreeLock((uint32_t *)&states[spi_num].req);

        // Callback if not NULL
        if (states[spi_num].async && req->completeCB != NULL) {
            req->completeCB(req, E_NO_ERROR);
        }
    }

    // Read the RX FIFO
    if (req->rxData != NULL) {
        req->rxCnt += MXC_SPI_ReadRXFIFO((mxc_spi_regs_t *)spi, &(req->rxData[req->rxCnt]),
                                         rx_length - req->rxCnt);

        remain = rx_length - req->rxCnt;

        if (remain) {
            if (remain >= MXC_SPI_FIFO_DEPTH) {
                MXC_SPI_SetRXThreshold((mxc_spi_regs_t *)spi, 2);
            } else {
                MXC_SPI_SetRXThreshold((mxc_spi_regs_t *)spi, remain - 1);
            }

            int_en |= MXC_F_SPI_REVA_INTEN_RX_THD;
        }

        // Break out if we've received all the bytes and we're not transmitting
        if ((req->txData == NULL) && (req->rxCnt == rx_length)) {
            spi->inten = 0;
            int_en = 0;
            MXC_FreeLock((uint32_t *)&states[spi_num].req);

            // Callback if not NULL
            if (states[spi_num].async && req->completeCB != NULL) {
                req->completeCB(req, E_NO_ERROR);
            }
        }
    }
    // Break out once we've transmitted and received all of the data
    if ((req->rxCnt == rx_length) && (req->txCnt == tx_length)) {
        spi->inten = 0;
        int_en = 0;
        MXC_FreeLock((uint32_t *)&states[spi_num].req);

        // Callback if not NULL
        if (states[spi_num].async && req->completeCB != NULL) {
            req->completeCB(req, E_NO_ERROR);
        }
    }

    return int_en;
}

int MXC_SPI_RevA1_MasterTransaction(mxc_spi_reva_req_t *req)
{
    int error;

    if ((error = MXC_SPI_RevA1_TransSetup(req)) != E_NO_ERROR) {
        return error;
    }

    states[MXC_SPI_GET_IDX((mxc_spi_regs_t *)req->spi)].async = 0;

    //call master transHandler
    while (MXC_SPI_RevA1_MasterTransHandler(req->spi, req) != 0) {}

    while (!((req->spi)->intfl & MXC_F_SPI_REVA_INTFL_MST_DONE)) {}

    return E_NO_ERROR;
}

int MXC_SPI_RevA1_MasterTransactionAsync(mxc_spi_reva_req_t *req)
{
    int error;

    if ((error = MXC_SPI_RevA1_TransSetup(req)) != E_NO_ERROR) {
        return error;
    }

    states[MXC_SPI_GET_IDX((mxc_spi_regs_t *)req->spi)].async = 1;

    MXC_SPI_EnableInt((mxc_spi_regs_t *)req->spi, MXC_SPI_RevA1_MasterTransHandler(req->spi, req));

    return E_NO_ERROR;
}

int MXC_SPI_RevA1_MasterTransactionDMA(mxc_spi_reva_req_t *req, int reqselTx, int reqselRx,
                                       mxc_dma_regs_t *dma)
{
    int spi_num;
    uint8_t error, bits;
    mxc_dma_config_t config;
    mxc_dma_srcdst_t srcdst;
    mxc_dma_adv_config_t advConfig = {
        0, MXC_DMA_PRIO_HIGH, 0, MXC_DMA_TIMEOUT_4_CLK, MXC_DMA_PRESCALE_DISABLE, 0
    };

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)(req->spi));

    if (req->txData == NULL && req->rxData == NULL) {
        return E_BAD_PARAM;
    }

    if ((error = MXC_SPI_RevA1_TransSetup(req)) != E_NO_ERROR) {
        return error;
    }

    // for non-MT mode do this setup every time, for MT mode only first time
    if ((states[spi_num].mtMode == 0) ||
        ((states[spi_num].mtMode == 1) && (states[spi_num].mtFirstTrans == 1))) {
#if (TARGET_NUM == 32665 || TARGET_NUM == 32657)
        MXC_DMA_Init(dma);
        states[spi_num].channelTx = MXC_DMA_AcquireChannel(dma);
        states[spi_num].channelRx = MXC_DMA_AcquireChannel(dma);
#else
        MXC_DMA_Init();
        states[spi_num].channelTx = MXC_DMA_AcquireChannel();
        states[spi_num].channelRx = MXC_DMA_AcquireChannel();
#endif

        if ((states[spi_num].channelTx < 0) || (states[spi_num].channelRx < 0)) {
            states[spi_num].channelTx = E_NO_DEVICE;
            states[spi_num].channelRx = E_NO_DEVICE;
            return E_NO_DEVICE;
        }

        states[spi_num].mtFirstTrans = 0;

        // Configure SS for per-transaction or always on
        if (req->ssDeassert) {
            req->spi->ctrl0 &= ~MXC_F_SPI_REVA_CTRL0_SS_CTRL;
        } else {
            req->spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_SS_CTRL;
        }
    }

    bits = MXC_SPI_GetDataSize((mxc_spi_regs_t *)req->spi);

    /*
    There is a known issue with the SPI hardware and DMA.  The SPI FIFO must be pre-loaded before DMA is initiated,
    otherwise the it will not work properly.  To do that, we leverage the TransHandler function, which will load the
    FIFOs as much as possible.

    If the TX or RX length is less than the FIFO size, there will be nothing for the DMA to transfer.  We need extra logic
    to ensure that the callbacks are still run in this case.  The TransHandler function returns a mask indicating the enabled
    interrupts.  Interrupts are only enabled if DMA is still needed.  We check this mask to see if DMA is still needed for RX/TX.
    Otherwise, we start the transmission (FIFOs are loaded, but a start is still needed to empty them) and then manually run the callbacks.
    */
    uint32_t enabled_interrupts = MXC_SPI_RevA1_TransHandler(req->spi, req);
    // TX FIFO is loaded completely.  DMA is not needed.
    bool tx_is_complete = !(enabled_interrupts & MXC_F_SPI_REVA_INTEN_TX_THD) &&
                          (req->txCnt == req->txLen);
    // RX FIFO is loaded completely.  DMA is not needed.
    bool rx_is_complete = !(enabled_interrupts & MXC_F_SPI_REVA_INTEN_RX_THD) &&
                          (req->rxCnt == req->rxLen);

    if (bits <= 8) {
        MXC_SPI_SetTXThreshold((mxc_spi_regs_t *)req->spi, 1); //set threshold to 1 byte
        MXC_SPI_SetRXThreshold((mxc_spi_regs_t *)req->spi, 0); //set threshold to 0 bytes
    } else {
        MXC_SPI_SetTXThreshold((mxc_spi_regs_t *)req->spi, 2);
        MXC_SPI_SetRXThreshold((mxc_spi_regs_t *)req->spi, 0);
    }

    //tx
    if (req->txData != NULL && !tx_is_complete) {
        MXC_DMA_SetCallback(states[spi_num].channelTx, MXC_SPI_RevA1_DMACallback);

#if (TARGET_NUM == 32657)
        MXC_DMA_EnableInt(dma, states[spi_num].channelTx);
#else
        MXC_DMA_EnableInt(states[spi_num].channelTx);
#endif

        config.reqsel = (mxc_dma_reqsel_t)reqselTx;
        config.ch = states[spi_num].channelTx;
        advConfig.ch = states[spi_num].channelTx;
        advConfig.burst_size = 2;

        if (bits <= 8) {
            config.srcwd = MXC_DMA_WIDTH_BYTE;
            config.dstwd = MXC_DMA_WIDTH_BYTE;
        } else {
            config.srcwd = MXC_DMA_WIDTH_HALFWORD;
            config.dstwd = MXC_DMA_WIDTH_HALFWORD;
        }

        config.srcinc_en = 1;
        config.dstinc_en = 0;

        srcdst.ch = states[spi_num].channelTx;
        srcdst.source = &(req->txData[req->txCnt]);

        if (bits > 8) {
            srcdst.len = (req->txLen * 2) - req->txCnt;
        } else {
            srcdst.len = (req->txLen) - req->txCnt;
        }

        MXC_DMA_ConfigChannel(config, srcdst);
        MXC_DMA_Start(states[spi_num].channelTx);
        MXC_DMA_SetChannelInterruptEn(states[spi_num].channelTx, false, true);
        //MXC_DMA->ch[channel].ctrl |= MXC_F_DMA_CTRL_CTZ_IE;

        if (bits > 8) {
            MXC_DMA_AdvConfigChannel(advConfig);
            //MXC_SETFIELD (MXC_DMA->ch[channel].ctrl, MXC_F_DMA_CTRL_BURST_SIZE, 1 << MXC_F_DMA_CTRL_BURST_SIZE_POS);
        }
    }

    // rx
    if (req->rxData != NULL && !rx_is_complete) {
        MXC_DMA_SetCallback(states[spi_num].channelRx, MXC_SPI_RevA1_DMACallback);

#if (TARGET_NUM == 32657)
        MXC_DMA_EnableInt(dma, states[spi_num].channelRx);
#else
        MXC_DMA_EnableInt(states[spi_num].channelRx);
#endif

        config.reqsel = (mxc_dma_reqsel_t)reqselRx;
        config.ch = states[spi_num].channelRx;
        config.srcinc_en = 0;
        config.dstinc_en = 1;
        advConfig.ch = states[spi_num].channelRx;
        advConfig.burst_size = 1;

        if (bits <= 8) {
            config.srcwd = MXC_DMA_WIDTH_BYTE;
            config.dstwd = MXC_DMA_WIDTH_BYTE;
        } else {
            config.srcwd = MXC_DMA_WIDTH_HALFWORD;
            config.dstwd = MXC_DMA_WIDTH_HALFWORD;
        }

        srcdst.ch = states[spi_num].channelRx;
        srcdst.dest = req->rxData;

        if (bits <= 8) {
            srcdst.len = req->rxLen;
        } else {
            srcdst.len = req->rxLen * 2;
        }

        MXC_DMA_ConfigChannel(config, srcdst);
        MXC_DMA_Start(states[spi_num].channelRx);
        MXC_DMA_SetChannelInterruptEn(states[spi_num].channelRx, false, true);
        //MXC_DMA->ch[channel].ctrl |= MXC_F_DMA_CTRL_CTZ_IE;

        if (bits > 8) {
            MXC_DMA_AdvConfigChannel(advConfig);
            //MXC_SETFIELD (MXC_DMA->ch[channel].ctrl, MXC_F_DMA_CTRL_BURST_SIZE, 0 << MXC_F_DMA_CTRL_BURST_SIZE_POS);
        }
    }

    // Enable TX/RX DMA, but only if it's still needed.
    (req->spi)->dma |= ((!(tx_is_complete) << MXC_F_SPI_REVA_DMA_DMA_TX_EN_POS) |
                        (!(rx_is_complete) << MXC_F_SPI_REVA_DMA_DMA_RX_EN_POS));

    if (!states[spi_num].started) {
        MXC_SPI_StartTransmission((mxc_spi_regs_t *)req->spi);
        states[spi_num].started = 1;
    }

    // Manually run TX/RX callbacks if the FIFO pre-load already completed that portion of the transaction
    if (tx_is_complete) {
        MXC_SPI_RevA1_DMACallback(states[spi_num].channelTx, E_NO_ERROR);
    }

    if (rx_is_complete) {
        MXC_SPI_RevA1_DMACallback(states[spi_num].channelRx, E_NO_ERROR);
    }

    return E_NO_ERROR;
}

int MXC_SPI_RevA1_SlaveTransaction(mxc_spi_reva_req_t *req)
{
    int error;

    if ((error = MXC_SPI_RevA1_TransSetup(req)) != E_NO_ERROR) {
        return error;
    }

    states[MXC_SPI_GET_IDX((mxc_spi_regs_t *)req->spi)].async = 0;

    while (MXC_SPI_RevA1_SlaveTransHandler(req) != 0) {}

    return E_NO_ERROR;
}

int MXC_SPI_RevA1_SlaveTransactionAsync(mxc_spi_reva_req_t *req)
{
    int error;

    if ((error = MXC_SPI_RevA1_TransSetup(req)) != E_NO_ERROR) {
        return error;
    }

    states[MXC_SPI_GET_IDX((mxc_spi_regs_t *)req->spi)].async = 1;

    MXC_SPI_EnableInt((mxc_spi_regs_t *)req->spi, MXC_SPI_RevA1_SlaveTransHandler(req));

    return E_NO_ERROR;
}

int MXC_SPI_RevA1_SlaveTransactionDMA(mxc_spi_reva_req_t *req, int reqselTx, int reqselRx,
                                      mxc_dma_regs_t *dma)
{
    int spi_num;
    uint8_t error, bits;
    mxc_dma_config_t config;
    mxc_dma_srcdst_t srcdst;
    mxc_dma_adv_config_t advConfig = {
        0, MXC_DMA_PRIO_HIGH, 0, MXC_DMA_TIMEOUT_4_CLK, MXC_DMA_PRESCALE_DISABLE, 0
    };

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)(req->spi));

    if (req->txData == NULL && req->rxData == NULL) {
        return E_BAD_PARAM;
    }

    if ((error = MXC_SPI_RevA1_TransSetup(req)) != E_NO_ERROR) {
        return error;
    }

    // for non-MT mode do this setup every time, for MT mode only first time
    if ((states[spi_num].mtMode == 0) ||
        ((states[spi_num].mtMode == 1) && (states[spi_num].mtFirstTrans == 1))) {
#if (TARGET_NUM == 32665 || TARGET_NUM == 32657)
        MXC_DMA_Init(dma);
        states[spi_num].channelTx = MXC_DMA_AcquireChannel(dma);
        states[spi_num].channelRx = MXC_DMA_AcquireChannel(dma);
#else
        MXC_DMA_Init();
        states[spi_num].channelTx = MXC_DMA_AcquireChannel();
        states[spi_num].channelRx = MXC_DMA_AcquireChannel();
#endif

        if ((states[spi_num].channelTx < 0) || (states[spi_num].channelRx < 0)) {
            states[spi_num].channelTx = E_NO_DEVICE;
            states[spi_num].channelRx = E_NO_DEVICE;
            return E_NO_DEVICE;
        }

        states[spi_num].mtFirstTrans = 0;

        MXC_DMA_SetCallback(states[spi_num].channelTx, MXC_SPI_RevA1_DMACallback);
        MXC_DMA_SetCallback(states[spi_num].channelRx, MXC_SPI_RevA1_DMACallback);

#if (TARGET_NUM == 32657)
        MXC_DMA_EnableInt(dma, states[spi_num].channelTx);
        MXC_DMA_EnableInt(dma, states[spi_num].channelRx);
#else
        MXC_DMA_EnableInt(states[spi_num].channelTx);
        MXC_DMA_EnableInt(states[spi_num].channelRx);
#endif
    }

    bits = MXC_SPI_GetDataSize((mxc_spi_regs_t *)req->spi);

    MXC_SPI_RevA1_TransHandler(req->spi, req);

    if (bits <= 8) {
        MXC_SPI_SetTXThreshold((mxc_spi_regs_t *)req->spi, 1);
        MXC_SPI_SetRXThreshold((mxc_spi_regs_t *)req->spi, 0);
    } else {
        MXC_SPI_SetTXThreshold((mxc_spi_regs_t *)req->spi, 2);
        MXC_SPI_SetRXThreshold((mxc_spi_regs_t *)req->spi, 0);
    }

    //tx
    if (req->txData != NULL) {
        config.reqsel = (mxc_dma_reqsel_t)reqselTx;
        config.ch = states[spi_num].channelTx;
        advConfig.ch = states[spi_num].channelTx;
        advConfig.burst_size = 2;

        if (bits <= 8) {
            config.srcwd = MXC_DMA_WIDTH_BYTE;
            config.dstwd = MXC_DMA_WIDTH_BYTE;
        } else {
            config.srcwd = MXC_DMA_WIDTH_HALFWORD;
            config.dstwd = MXC_DMA_WIDTH_HALFWORD;
        }

        config.srcinc_en = 1;
        config.dstinc_en = 0;

        srcdst.ch = states[spi_num].channelTx;
        srcdst.source = &(req->txData[req->txCnt]);

        if (bits > 8) {
            srcdst.len = (req->txLen * 2) - req->txCnt;
        } else {
            srcdst.len = (req->txLen) - req->txCnt;
        }

        MXC_DMA_ConfigChannel(config, srcdst);
        MXC_DMA_Start(states[spi_num].channelTx);
        MXC_DMA_SetChannelInterruptEn(states[spi_num].channelTx, false, true);
        //MXC_DMA->ch[channel].ctrl |= MXC_F_DMA_CTRL_CTZ_IE;

        if (bits > 8) {
            MXC_DMA_AdvConfigChannel(advConfig);
            //MXC_SETFIELD (MXC_DMA->ch[channel].ctrl, MXC_F_DMA_CTRL_BURST_SIZE, 1 << MXC_F_DMA_CTRL_BURST_SIZE_POS);
        }
    }

    if (req->rxData != NULL) {
        config.reqsel = (mxc_dma_reqsel_t)reqselRx;
        config.ch = states[spi_num].channelRx;
        config.srcinc_en = 0;
        config.dstinc_en = 1;
        advConfig.ch = states[spi_num].channelRx;
        advConfig.burst_size = 1;

        if (bits <= 8) {
            config.srcwd = MXC_DMA_WIDTH_BYTE;
            config.dstwd = MXC_DMA_WIDTH_BYTE;
        } else {
            config.srcwd = MXC_DMA_WIDTH_HALFWORD;
            config.dstwd = MXC_DMA_WIDTH_HALFWORD;
        }

        srcdst.ch = states[spi_num].channelRx;
        srcdst.dest = req->rxData;

        if (bits <= 8) {
            srcdst.len = req->rxLen;
        } else {
            srcdst.len = req->rxLen * 2;
        }

        MXC_DMA_ConfigChannel(config, srcdst);
        MXC_DMA_Start(states[spi_num].channelRx);
        MXC_DMA_SetChannelInterruptEn(states[spi_num].channelRx, false, true);
        //MXC_DMA->ch[channel].ctrl |= MXC_F_DMA_CTRL_CTZ_IE;

        if (bits > 8) {
            MXC_DMA_AdvConfigChannel(advConfig);
            //MXC_SETFIELD (MXC_DMA->ch[channel].ctrl, MXC_F_DMA_CTRL_BURST_SIZE, 0 << MXC_F_DMA_CTRL_BURST_SIZE_POS);
        }
    }

    (req->spi)->dma |= (MXC_F_SPI_REVA_DMA_DMA_TX_EN | MXC_F_SPI_REVA_DMA_DMA_RX_EN);

    return E_NO_ERROR;
}

void MXC_SPI_RevA1_DMACallback(int ch, int error)
{
    mxc_spi_reva_req_t *temp_req;

    for (int i = 0; i < MXC_SPI_INSTANCES; i++) {
        if (states[i].req != NULL) {
            if (states[i].channelTx == ch) {
                states[i].req_done++;
            } else if (states[i].channelRx == ch) {
                states[i].req_done++;
                //save the request
                temp_req = states[i].req;

                if (MXC_SPI_GetDataSize((mxc_spi_regs_t *)temp_req->spi) > 8) {
                    MXC_SPI_RevA1_SwapByte(temp_req->rxData, temp_req->rxLen);
                }
            }

            if (!states[i].txrx_req || (states[i].txrx_req && states[i].req_done == 2)) {
                //save the request
                temp_req = states[i].req;
                MXC_FreeLock((uint32_t *)&states[i].req);
                // Callback if not NULL
                if (temp_req->completeCB != NULL) {
                    temp_req->completeCB(temp_req, E_NO_ERROR);
                }
                if (states[i].mtMode == 0) {
                    // release any acquired DMA channels
                    if (states[i].channelTx >= 0) {
                        MXC_DMA_RevA_ReleaseChannel(states[i].channelTx);
                        states[i].channelTx = E_NO_DEVICE;
                    }
                    if (states[i].channelRx >= 0) {
                        MXC_DMA_RevA_ReleaseChannel(states[i].channelRx);
                        states[i].channelRx = E_NO_DEVICE;
                    }
                }
                break;
            }
        }
    }
}

int MXC_SPI_RevA1_SetDefaultTXData(mxc_spi_reva_regs_t *spi, unsigned int defaultTXData)
{
    int spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    states[spi_num].defaultTXData = defaultTXData;
    return E_NO_ERROR;
}

void MXC_SPI_RevA1_AbortAsync(mxc_spi_reva_regs_t *spi)
{
    MXC_SPI_AbortTransmission((mxc_spi_regs_t *)spi);
}

void MXC_SPI_RevA1_AsyncHandler(mxc_spi_reva_regs_t *spi)
{
    int spi_num;
    unsigned rx_avail;
    uint32_t flags;

    // Clear the interrupt flags
    spi->inten = 0;
    flags = spi->intfl;
    spi->intfl = flags;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    // Figure out if this SPI has an active request
    if ((states[spi_num].req != NULL) && (flags)) {
        if ((spi->ctrl0 & MXC_F_SPI_REVA_CTRL0_MST_MODE) >> MXC_F_SPI_REVA_CTRL0_MST_MODE_POS) {
            do {
                spi->inten = MXC_SPI_RevA1_MasterTransHandler(spi, states[spi_num].req);
                rx_avail = MXC_SPI_RevA1_GetRXFIFOAvailable(spi);
            } while (rx_avail > MXC_SPI_RevA1_GetRXThreshold(spi));

        } else {
            do {
                spi->inten = MXC_SPI_RevA1_SlaveTransHandler(states[spi_num].req);
                rx_avail = MXC_SPI_RevA1_GetRXFIFOAvailable(spi);
            } while (rx_avail > MXC_SPI_RevA1_GetRXThreshold(spi));
        }
    }
}

//call in DMA IRQHANDLER with rxData for transmissions with bits > 8
void MXC_SPI_RevA1_SwapByte(uint8_t *arr, size_t length)
{
    MXC_ASSERT(arr != NULL);

    for (size_t i = 0; i < (length * 2); i += 2) {
        uint8_t tmp = arr[i];
        arr[i] = arr[i + 1];
        arr[i + 1] = tmp;
    }
}

void MXC_SPI_RevA1_HWSSControl(mxc_spi_reva_regs_t *spi, int state)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    states[spi_num].hw_ss_control = state ? true : false;

    return;
}
