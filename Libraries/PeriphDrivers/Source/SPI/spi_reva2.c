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
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "nvic_table.h"
#include "spi.h"
#include "spi_reva2.h"
#include "dma_reva.h"

/* **** Definitions **** */

// clang-format off
typedef struct {
    // Info from initialization.
    bool                dma_initialized;
    mxc_spi_type_t      controller_target;      // Controller or Target Mode.
    uint8_t             frame_size;             //
    mxc_spi_interface_t if_mode;


    // Transaction Data.
    uint8_t            *tx_buffer;
    uint32_t            tx_length_bytes;        // Terms of bytes
    uint32_t            tx_count_bytes;         // Terms of bytes
    uint8_t            *rx_buffer;
    uint32_t            rx_length_bytes;        // Terms of bytes
    uint32_t            rx_count_bytes;
    uint16_t            tx_dummy_value;

    mxc_spi_callback_t  callback;
    void                *callback_data;

    // Chip Select Info.
    bool                deassert;               // Target Select (TS) Deasserted at the end of a transmission.
    mxc_spi_tscontrol_t ts_control;

    // DMA Settings.
    mxc_dma_reva_regs_t *dma;
    int                 tx_dma_ch;
    int                 rx_dma_ch;

    // Status Fields.
    bool                transaction_done;
    bool                tx_done;
    bool                rx_done;
} mxc_spi_reva2_handle_data_t;
// clang-format on

static volatile mxc_spi_reva2_handle_data_t STATES[MXC_SPI_INSTANCES];

/* **** Private Functions **** */

// The unique title for Private functions will not be capitalized.

/** Private Function: writeTXFIFO16
 * Writes 2 bytes to the TX FIFO for 9-16 bit frame lengths.
 * This function helps package the frame when the STATES[n] fields
 * are all in terms of bytes.
 * 
 * @param   spi             Pointer to SPI instance.
 * @param   buffer          Pointer to buffer of messages to transmit.
 * @param   length_bytes    Number of messages (in terms of bytes) in buffer to transmit.
 * 
 * @return  count           The number of frames written to the TX FIFO.
 */
static uint32_t MXC_SPI_RevA2_writeTXFIFO16(mxc_spi_reva_regs_t *spi, uint8_t *buffer,
                                            uint32_t length_bytes)
{
    uint32_t tx_avail;
    uint32_t count = 0;

    if (buffer == NULL || length_bytes == 0) {
        return 0;
    }

    tx_avail = MXC_SPI_FIFO_DEPTH -
               ((spi->dma & MXC_F_SPI_REVA_DMA_TX_LVL) >> MXC_F_SPI_REVA_DMA_TX_LVL_POS);

    // Do not write more than the available FIFO size
    if (length_bytes > tx_avail) {
        length_bytes = tx_avail;
    }

    // Ensure even lengths for halfword frame lengths.
    // Note: Len is in terms of bytes, so sending 9-16bit transactions means sending
    //          2 bytes per frame.
    length_bytes &= ~0x01;

    while (length_bytes) {
        if (length_bytes > 3) {
            memcpy((void *)(&spi->fifo32), (uint8_t *)(&buffer[count]), 4);

            length_bytes -= 4;
            count += 4;

        } else if (length_bytes > 1) {
            memcpy((void *)(&spi->fifo16[0]), (uint8_t *)(&buffer[count]), 2);

            length_bytes -= 2;
            count += 2;
        }
    }

    return count;
}

/** Private Function: readRXFIFO16
 * Reads 2 bytes from the RX FIFO for 9-16 bit frame lengths.
 * This function helps package the frame when the STATES[n] fields
 * are all in terms of bytes.
 * 
 * @param   spi             Pointer to SPI instance.
 * @param   buffer          Pointer to buffer to store read messages.
 * @param   length_bytes    Number of messages (in terms of bytes) to store in receive buffer.
 * 
 * @return  count           The number of frames read from the RX FIFO.
 */
static uint32_t MXC_SPI_RevA2_readRXFIFO16(mxc_spi_reva_regs_t *spi, uint8_t *buffer,
                                           uint32_t length_bytes)
{
    uint32_t rx_avail;
    uint32_t count = 0;

    if (buffer == NULL || length_bytes == 0) {
        return 0;
    }

    rx_avail = (spi->dma & MXC_F_SPI_REVA_DMA_RX_LVL) >> MXC_F_SPI_REVA_DMA_RX_LVL_POS;

    // Do not read more than available frames in RX FIFO.
    if (length_bytes > rx_avail) {
        length_bytes = rx_avail;
    }

    // Ensure even lengths for halfword frame lengths.
    // Note: Len is in terms of bytes, so reading 9-16bit wide messages means reading
    //          2 bytes per frame.
    length_bytes &= ~0x01;

    if (length_bytes >= 2) {
        // Read from the FIFO
        while (length_bytes) {
            if (length_bytes > 3) {
                memcpy((uint8_t *)(&buffer[count]), (void *)(&spi->fifo32), 4);
                length_bytes -= 4;
                count += 4;

            } else if (length_bytes > 1) {
                memcpy((uint8_t *)(&buffer[count]), (void *)(&spi->fifo16[0]), 2);
                length_bytes -= 2;
                count += 2;
            }

            // Ensures read of less than 2 bytes aren't read.
            // Code should never get to this point.
            if (length_bytes == 1) {
                break;
            }
        }
    }

    return count;
}

/** Private Function: process
 * This function handles the reads and writes to the SPI RX/TX FIFO.
 * 
 * @param   spi     Pointer to SPI instance.
 */
static void MXC_SPI_RevA2_process(mxc_spi_reva_regs_t *spi)
{
    int8_t spi_num;
    int remain;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    // Write any pending bytes out.
    //  Dependent on 1) Valid TX Buffer, 2) TX Length not 0, and 3) TX FIFO Not Empty.
    if (STATES[spi_num].tx_buffer && STATES[spi_num].tx_length_bytes > 0) {
        // Write to the FIFO for byte size transactions (message sizes for 8 bits or less)
        if (STATES[spi_num].frame_size <= 8) {
            while (((spi->dma & MXC_F_SPI_REVA_DMA_TX_LVL) >> MXC_F_SPI_REVA_DMA_TX_LVL_POS) <
                   (MXC_SPI_FIFO_DEPTH)) {
                // Check for overflow.
                if (STATES[spi_num].tx_count_bytes == STATES[spi_num].tx_length_bytes) {
                    break;
                }

                spi->fifo8[0] = STATES[spi_num].tx_buffer[STATES[spi_num].tx_count_bytes];
                STATES[spi_num].tx_count_bytes += 1;
            }

            // Write to the FIFO for halfword size transactions (message sizes for 9 bits or greater)
        } else {
            STATES[spi_num].tx_count_bytes += MXC_SPI_RevA2_writeTXFIFO16(
                spi, &(STATES[spi_num].tx_buffer[STATES[spi_num].tx_count_bytes]),
                STATES[spi_num].tx_length_bytes - STATES[spi_num].tx_count_bytes);

            remain = STATES[spi_num].tx_length_bytes - STATES[spi_num].tx_count_bytes;

            if (remain) {
                if (remain >= MXC_SPI_FIFO_DEPTH) {
                    MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_TX_THD_VAL,
                                 ((MXC_SPI_FIFO_DEPTH - 1) << MXC_F_SPI_REVA_DMA_TX_THD_VAL_POS));
                } else {
                    MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_TX_THD_VAL,
                                 (remain << MXC_F_SPI_REVA_DMA_TX_THD_VAL_POS));
                }
            }
        }
    }

    if (STATES[spi_num].tx_count_bytes == STATES[spi_num].tx_length_bytes) {
        STATES[spi_num].tx_done = true;
    }

    // Unload any SPI data that has come in
    //  Dependent on 1) Valid RX Buffer, 2) RX Length not 0, and 3) RX FIFO Not Empty.
    if (STATES[spi_num].rx_buffer && STATES[spi_num].rx_length_bytes > 0) {
        // Read the FIFO for byte size transactions (message sizes for 8 bits or less)
        if (STATES[spi_num].frame_size <= 8) {
            while ((spi->dma & MXC_F_SPI_REVA_DMA_RX_LVL)) {
                // Check for overflow.
                if (STATES[spi_num].rx_count_bytes == STATES[spi_num].rx_length_bytes) {
                    break;
                }

                STATES[spi_num].rx_buffer[STATES[spi_num].rx_count_bytes] = spi->fifo8[0];
                STATES[spi_num].rx_count_bytes += 1;
            }

            // Read the FIFO for halfword size transactions (message sizes for 9 bits or greater)
        } else {
            STATES[spi_num].rx_count_bytes += MXC_SPI_RevA2_readRXFIFO16(
                spi, &(STATES[spi_num].rx_buffer[STATES[spi_num].rx_count_bytes]),
                STATES[spi_num].rx_length_bytes - STATES[spi_num].rx_count_bytes);

            remain = STATES[spi_num].rx_length_bytes - STATES[spi_num].rx_count_bytes;

            if (remain) {
                if (remain >= MXC_SPI_FIFO_DEPTH) {
                    MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_RX_THD_VAL,
                                 (2 << MXC_F_SPI_REVA_DMA_RX_THD_VAL_POS));
                } else {
                    MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_RX_THD_VAL,
                                 ((remain - 1) << MXC_F_SPI_REVA_DMA_RX_THD_VAL_POS));
                }
            }
        }
    }

    if (STATES[spi_num].rx_count_bytes == STATES[spi_num].rx_length_bytes) {
        STATES[spi_num].rx_done = true;
    }

    // Handle Target Transaction Completion.
    //  Unlike the Controller, there is no Target Done interrupt to handle the callback and set the
    //  transaction_done flag.
    if (STATES[spi_num].controller_target == MXC_SPI_TYPE_TARGET) {
        // Check if the transaction is complete.
        if (STATES[spi_num].tx_done == true && STATES[spi_num].rx_done == true) {
            // Callback if valid.
            // Note: If Target Select (TS) Control Scheme is set in SW_App mode, then the caller needs to ensure the
            //   Target Select (TS) pin is asserted or deasserted in their application.
            if (STATES[spi_num].callback) {
                STATES[spi_num].callback(STATES[spi_num].callback_data, E_NO_ERROR);
            }

            // Target is done after callback (if valid) is handled.
            STATES[spi_num].transaction_done = true;

            // Reset the SPI to complete the on-going transaction.
            //  SPIn may remain busy (SPI_STAT) even after the target select input
            //  is deasserted. This ensures the SPI block is not busy after a
            //  target transaction is completed.
            spi->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL0_EN);
            spi->ctrl0 |= (MXC_F_SPI_REVA_CTRL0_EN);
        }
    }
}

/** Private Function: resetStateStruct
 * This functions resets the STATE of an SPI instance.
 * 
 * @param   spi_num     Index number of SPI instance.
 */
static void MXC_SPI_RevA2_resetStateStruct(int8_t spi_num)
{
    // Init Data
    STATES[spi_num].dma_initialized = false;
    STATES[spi_num].controller_target = MXC_SPI_TYPE_CONTROLLER;
    STATES[spi_num].frame_size = 8; // 1 byte frame sizes.
    STATES[spi_num].if_mode = MXC_SPI_INTERFACE_STANDARD;

    // Transaction Members
    STATES[spi_num].tx_buffer = NULL;
    STATES[spi_num].tx_length_bytes = 0;
    STATES[spi_num].tx_count_bytes = 0;
    STATES[spi_num].rx_buffer = NULL;
    STATES[spi_num].rx_length_bytes = 0;
    STATES[spi_num].rx_count_bytes = 0;
    STATES[spi_num].deassert =
        true; // Default state is TS will be deasserted at the end of a transmission.
    STATES[spi_num].ts_control = MXC_SPI_TSCONTROL_HW_AUTO; // Default (0) state.

    // DMA
    STATES[spi_num].dma = NULL;
    STATES[spi_num].tx_dma_ch = -1;
    STATES[spi_num].rx_dma_ch = -1;

    // Status Members
    STATES[spi_num].transaction_done = false;
    STATES[spi_num].tx_done = false;
    STATES[spi_num].rx_done = false;
}

/* **** Public Functions **** */

int MXC_SPI_RevA2_Init(mxc_spi_reva_regs_t *spi, mxc_spi_type_t controller_target,
                       mxc_spi_interface_t if_mode, uint32_t freq, uint8_t ts_active_pol_mask)
{
    int error;
    int8_t spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // Reset STATE of current SPI instance.
    MXC_SPI_RevA2_resetStateStruct(spi_num);

    // Save init data states.
    STATES[spi_num].controller_target = controller_target;
    STATES[spi_num].frame_size = 8;
    STATES[spi_num].if_mode = if_mode;

    // Enable SPI port.
    spi->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL0_EN);
    spi->ctrl0 |= (MXC_F_SPI_REVA_CTRL0_EN);

    // Set Controller (L. Master) or Target (L. Slave) mode.
    switch (controller_target) {
    case MXC_SPI_TYPE_CONTROLLER:
        spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_MST_MODE;
        break;

    case MXC_SPI_TYPE_TARGET:
        spi->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL0_MST_MODE);
        break;

    default:
        return E_BAD_PARAM;
    }

    // Set default frame size to 8 bits wide.
    MXC_SETFIELD(spi->ctrl2, MXC_F_SPI_REVA_CTRL2_NUMBITS, 8 << MXC_F_SPI_REVA_CTRL2_NUMBITS_POS);

    // Remove any delay between TS (L. SS) and SCLK edges.
    spi->sstime = (1 << MXC_F_SPI_REVA_SSTIME_PRE_POS) | (1 << MXC_F_SPI_REVA_SSTIME_POST_POS) |
                  (1 << MXC_F_SPI_REVA_SSTIME_INACT_POS);

    // Enable TX/RX FIFOs
    spi->dma |= MXC_F_SPI_REVA_DMA_TX_FIFO_EN | MXC_F_SPI_REVA_DMA_RX_FIFO_EN;

    // Set TX and RX Threshold to (FIFO_DEPTH - 1) and (0), respectively.
    MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_TX_THD_VAL,
                 ((MXC_SPI_FIFO_DEPTH - 1) << MXC_F_SPI_REVA_DMA_TX_THD_VAL_POS));
    MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_RX_THD_VAL, (0 << MXC_F_SPI_REVA_DMA_RX_THD_VAL_POS));

    // Set Default Clock Mode (CPOL: 0, and CPHA: 0).
    error = MXC_SPI_SetClkMode((mxc_spi_regs_t *)spi, MXC_SPI_CLKMODE_0);
    if (error != E_NO_ERROR) {
        return error;
    }

    // Interface mode: 3-wire, standard (4-wire), dual, quad.
    error = MXC_SPI_SetInterface((mxc_spi_regs_t *)spi, if_mode);
    if (error != E_NO_ERROR) {
        return error;
    }

    error = MXC_SPI_SetFrequency((mxc_spi_regs_t *)spi, freq);
    if (error != E_NO_ERROR) {
        return error;
    }

    // Clear any interrupt flags that may already be set.
    spi->intfl = spi->intfl;
    spi->inten = 0;

    // Clear the HW TS settings (These are set in the transaction functions).
    MXC_SETFIELD(spi->ctrl0, MXC_F_SPI_REVA_CTRL0_SS_ACTIVE, 0);

    // Set the TS Active Polarity settings.
    MXC_SETFIELD(spi->ctrl2, MXC_F_SPI_REVA_CTRL2_SS_POL,
                 ts_active_pol_mask << MXC_F_SPI_REVA_CTRL2_SS_POL_POS);

    return E_NO_ERROR;
}

int MXC_SPI_RevA2_Config(mxc_spi_cfg_t *cfg)
{
    int error;
    int8_t spi_num;

    if (cfg == NULL) {
        return E_NULL_PTR;
    }

    // Ensure valid SPI instance.
    spi_num = MXC_SPI_GET_IDX(cfg->spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // Set Single Frame Size.
    error = MXC_SPI_SetFrameSize((cfg->spi), (cfg->frame_size));
    if (error != E_NO_ERROR) {
        return error;
    }

    // Set Clock Mode (CPOL and CPHA).
    error = MXC_SPI_SetClkMode((cfg->spi), (cfg->clk_mode));
    if (error != E_NO_ERROR) {
        return error;
    }

    // Setup DMA features if used.
    if (cfg->use_dma_tx || cfg->use_dma_rx) {
        error = MXC_SPI_RevA2_DMA_Init((mxc_spi_reva_regs_t *)(cfg->spi),
                                       (mxc_dma_reva_regs_t *)(cfg->dma), (cfg->use_dma_tx),
                                       (cfg->use_dma_rx));
        if (error != E_NO_ERROR) {
            return error;
        }
    }

    return E_NO_ERROR;
}

int MXC_SPI_RevA2_Shutdown(mxc_spi_reva_regs_t *spi)
{
    int8_t spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // Disable and clear interrupts.
    spi->inten = 0;
    spi->intfl = spi->intfl;

    // Disable SPI and FIFOS
    spi->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL0_EN);
    spi->dma &= ~(MXC_F_SPI_REVA_DMA_TX_FIFO_EN | MXC_F_SPI_REVA_DMA_RX_FIFO_EN);

    // Clear registers.
    spi->ctrl0 = 0;
    spi->ctrl1 = 0;
    spi->ctrl2 = 0;
    spi->sstime = 0;

    // Release any acquired DMA channels.
    if (STATES[spi_num].tx_dma_ch >= 0) {
        MXC_DMA_ReleaseChannel(STATES[spi_num].tx_dma_ch);
        STATES[spi_num].tx_dma_ch = E_NO_DEVICE;
    }
    if (STATES[spi_num].rx_dma_ch >= 0) {
        MXC_DMA_ReleaseChannel(STATES[spi_num].rx_dma_ch);
        STATES[spi_num].rx_dma_ch = E_NO_DEVICE;
    }

    if (STATES[spi_num].dma_initialized) {
        MXC_DMA_DeInit();
    }

    // Reset the SPI instance's STATE when shutting down.
    MXC_SPI_RevA2_resetStateStruct(spi_num);

    return E_NO_ERROR;
}

uint32_t MXC_SPI_RevA2_GetFlags(mxc_spi_reva_regs_t *spi)
{
    return spi->intfl;
}

void MXC_SPI_RevA2_ClearFlags(mxc_spi_reva_regs_t *spi)
{
    spi->intfl = spi->intfl;
}

void MXC_SPI_RevA2_EnableInt(mxc_spi_reva_regs_t *spi, uint32_t en)
{
    spi->inten |= en;
}

void MXC_SPI_RevA2_DisableInt(mxc_spi_reva_regs_t *spi, uint32_t dis)
{
    spi->inten &= ~(dis);
}

int MXC_SPI_RevA2_SetTSControl(mxc_spi_reva_regs_t *spi, mxc_spi_tscontrol_t ts_control)
{
    int8_t spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    switch (ts_control) {
    case MXC_SPI_TSCONTROL_HW_AUTO:
        break;

    case MXC_SPI_TSCONTROL_SW_APP:
        spi->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL0_SS_ACTIVE);
        spi->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL2_SS_POL);
        break;

    default:
        return E_BAD_PARAM;
    }

    STATES[spi_num].ts_control = ts_control;

    return E_NO_ERROR;
}

mxc_spi_tscontrol_t MXC_SPI_RevA2_GetTSControl(mxc_spi_reva_regs_t *spi)
{
    int8_t spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    return (STATES[spi_num].ts_control);
}

int MXC_SPI_RevA2_SetFrequency(mxc_spi_reva_regs_t *spi, uint32_t freq)
{
    int hi_clk, lo_clk, scale;
    uint32_t freq_div;

    // Check if frequency is too high
    if (freq > PeripheralClock) {
        return E_BAD_PARAM;
    }

    // Set the clock high and low
    freq_div = MXC_SPI_GetPeripheralClock((mxc_spi_regs_t *)spi);
    freq_div = (freq_div / freq);

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

int MXC_SPI_RevA2_GetFrequency(mxc_spi_reva_regs_t *spi)
{
    unsigned scale, lo_clk, hi_clk;

    scale = (spi->clkctrl & MXC_F_SPI_REVA_CLKCTRL_CLKDIV) >> MXC_F_SPI_REVA_CLKCTRL_CLKDIV_POS;
    hi_clk = (spi->clkctrl & MXC_F_SPI_REVA_CLKCTRL_HI) >> MXC_F_SPI_REVA_CLKCTRL_HI_POS;
    lo_clk = (spi->clkctrl & MXC_F_SPI_REVA_CLKCTRL_LO) >> MXC_F_SPI_REVA_CLKCTRL_LO_POS;

    return (PeripheralClock / (1 << scale)) / (lo_clk + hi_clk);
}

int MXC_SPI_RevA2_SetFrameSize(mxc_spi_reva_regs_t *spi, int frame_size)
{
    int8_t spi_num;
    int saved_enable_state;

    // HW has problem with these two character sizes
    if (frame_size <= 1 || frame_size > 16) {
        return E_BAD_PARAM;
    }

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    if ((spi->stat & MXC_F_SPI_REVA_STAT_BUSY) &&
        (STATES[spi_num].controller_target == MXC_SPI_TYPE_CONTROLLER)) {
        return E_BAD_STATE;
    }

    // Set up the character size.
    saved_enable_state = spi->ctrl0 | MXC_F_SPI_REVA_CTRL0_EN;

    // If enabled, disable SPI before changing character size.
    if (saved_enable_state) {
        spi->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL0_EN);
    }

    // Update data size from save Init function.
    STATES[spi_num].frame_size = frame_size;

    if (frame_size < 16) {
        MXC_SETFIELD(spi->ctrl2, MXC_F_SPI_REVA_CTRL2_NUMBITS,
                     frame_size << MXC_F_SPI_REVA_CTRL2_NUMBITS_POS);
    } else {
        // Set to 16 bits per character as default.
        MXC_SETFIELD(spi->ctrl2, MXC_F_SPI_REVA_CTRL2_NUMBITS,
                     0 << MXC_F_SPI_REVA_CTRL2_NUMBITS_POS);
    }

    // Return back to original SPI enable state.
    MXC_SETFIELD(spi->ctrl0, MXC_F_SPI_REVA_CTRL0_EN, saved_enable_state);

    return E_NO_ERROR;
}

int MXC_SPI_RevA2_GetFrameSize(mxc_spi_reva_regs_t *spi)
{
    // NUMBITS = 0 means 16-bits per character
    if (!(spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_NUMBITS)) {
        return 16;
    } else {
        return ((spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_NUMBITS) >> MXC_F_SPI_REVA_CTRL2_NUMBITS_POS);
    }
}

int MXC_SPI_RevA2_SetInterface(mxc_spi_reva_regs_t *spi, mxc_spi_interface_t if_mode)
{
    // Clear before setting
    spi->ctrl2 &= ~(MXC_F_SPI_REVA_CTRL2_THREE_WIRE | MXC_F_SPI_REVA_CTRL2_DATA_WIDTH);

    switch (if_mode) {
    case MXC_SPI_INTERFACE_3WIRE:
        spi->ctrl2 |= MXC_F_SPI_REVA_CTRL2_THREE_WIRE;
        break;

    case MXC_SPI_INTERFACE_STANDARD:
        spi->ctrl2 |= MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_MONO;
        break;

    case MXC_SPI_INTERFACE_DUAL:
        spi->ctrl2 |= MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_DUAL;
        break;

    case MXC_SPI_INTERFACE_QUAD:
        spi->ctrl2 |= MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_QUAD;
        break;

    // Default set to to 3-Wire
    default:
        spi->ctrl2 |= MXC_F_SPI_REVA_CTRL2_THREE_WIRE;
        break;
    }

    // Save state of new mode
    STATES[MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi)].if_mode = if_mode;

    return E_NO_ERROR;
}

mxc_spi_interface_t MXC_SPI_RevA2_GetInterface(mxc_spi_reva_regs_t *spi)
{
    if (spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_THREE_WIRE) {
        return MXC_SPI_INTERFACE_3WIRE;
    }

    if (spi->ctrl2 & MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_DUAL) {
        return MXC_SPI_INTERFACE_DUAL;
    }

    if (spi->ctrl2 & MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_QUAD) {
        return MXC_SPI_INTERFACE_QUAD;
    }

    return MXC_SPI_INTERFACE_STANDARD;
}

int MXC_SPI_RevA2_SetClkMode(mxc_spi_reva_regs_t *spi, mxc_spi_clkmode_t clk_mode)
{
    switch (clk_mode) {
    // CPOL: 0    CPHA: 0
    case MXC_SPI_CLKMODE_0:
        spi->ctrl2 &= ~MXC_F_SPI_REVA_CTRL2_CLKPHA;
        spi->ctrl2 &= ~MXC_F_SPI_REVA_CTRL2_CLKPOL;
        break;

    // CPOL: 0    CPHA: 1
    case MXC_SPI_CLKMODE_1:
        spi->ctrl2 &= ~MXC_F_SPI_REVA_CTRL2_CLKPHA;
        spi->ctrl2 |= MXC_F_SPI_REVA_CTRL2_CLKPOL;
        break;

    // CPOL: 1    CPHA: 0
    case MXC_SPI_CLKMODE_2:
        spi->ctrl2 |= MXC_F_SPI_REVA_CTRL2_CLKPHA;
        spi->ctrl2 &= ~MXC_F_SPI_REVA_CTRL2_CLKPOL;
        break;

    // CPOL: 1    CPHA: 1
    case MXC_SPI_CLKMODE_3:
        spi->ctrl2 |= MXC_F_SPI_REVA_CTRL2_CLKPHA;
        spi->ctrl2 |= MXC_F_SPI_REVA_CTRL2_CLKPOL;
        break;

    // Mode 0 by default.
    default:
        spi->ctrl2 &= ~MXC_F_SPI_REVA_CTRL2_CLKPHA;
        spi->ctrl2 &= ~MXC_F_SPI_REVA_CTRL2_CLKPOL;
        break;
    }

    return E_NO_ERROR;
}

mxc_spi_clkmode_t MXC_SPI_RevA2_GetClkMode(mxc_spi_reva_regs_t *spi)
{
    if (spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_CLKPHA) {
        if (spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_CLKPOL) {
            return MXC_SPI_CLKMODE_3;
        } else {
            return MXC_SPI_CLKMODE_2;
        }
    } else {
        if (spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_CLKPOL) {
            return MXC_SPI_CLKMODE_1;
        }
    }

    return MXC_SPI_CLKMODE_0;
}

int MXC_SPI_RevA2_SetCallback(mxc_spi_reva_regs_t *spi, mxc_spi_callback_t callback, void *data)
{
    int8_t spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    STATES[spi_num].callback = callback;
    STATES[spi_num].callback_data = data;

    return E_NO_ERROR;
}

int MXC_SPI_RevA2_GetActive(mxc_spi_reva_regs_t *spi)
{
    if (spi->stat & MXC_F_SPI_REVA_STAT_BUSY) {
        return E_BUSY;
    }

    return E_NO_ERROR;
}

int MXC_SPI_RevA2_ReadyForSleep(mxc_spi_reva_regs_t *spi)
{
    if (spi->stat & MXC_F_SPI_REVA_STAT_BUSY || (spi->dma & MXC_F_SPI_REVA_DMA_TX_LVL) ||
        (spi->dma & MXC_F_SPI_REVA_DMA_RX_LVL)) {
        return E_BUSY;
    } else {
        return E_NO_ERROR;
    }
}

int MXC_SPI_RevA2_SetDummyTX(mxc_spi_reva_regs_t *spi, uint16_t tx_value)
{
    STATES[MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi)].tx_dummy_value = tx_value;

    return E_NO_ERROR;
}

int MXC_SPI_RevA2_StartTransmission(mxc_spi_reva_regs_t *spi)
{
    if (MXC_SPI_GetActive((mxc_spi_regs_t *)spi) == E_BUSY) {
        return E_BUSY;
    }

    spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_START;

    return E_NO_ERROR;
}

int MXC_SPI_RevA2_AbortTransmission(mxc_spi_reva_regs_t *spi)
{
    int8_t spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    // Disable interrupts, clear the flags.
    spi->inten = 0;
    spi->intfl = spi->intfl;

    // Cancel on-going transaction before enabling.
    spi->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL0_EN);
    spi->ctrl0 |= (MXC_F_SPI_REVA_CTRL0_EN);

    // Callback if not NULL
    if (STATES[spi_num].callback != NULL) {
        STATES[spi_num].callback(STATES[spi_num].callback_data, E_ABORT);
    }

    return E_NO_ERROR;
}

uint8_t MXC_SPI_RevA2_GetTXFIFOAvailable(mxc_spi_reva_regs_t *spi)
{
    return MXC_SPI_FIFO_DEPTH -
           ((spi->dma & MXC_F_SPI_REVA_DMA_TX_LVL) >> MXC_F_SPI_REVA_DMA_TX_LVL_POS);
}

uint8_t MXC_SPI_RevA2_GetRXFIFOAvailable(mxc_spi_reva_regs_t *spi)
{
    return (spi->dma & MXC_F_SPI_REVA_DMA_RX_LVL) >> MXC_F_SPI_REVA_DMA_RX_LVL_POS;
}

int MXC_SPI_RevA2_ClearTXFIFO(mxc_spi_reva_regs_t *spi)
{
    uint32_t save_state;

    save_state = (spi->dma & (MXC_F_SPI_REVA_DMA_TX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_TX_EN));

    // Disable FIFOs before clearing as recommended by UG.
    spi->dma &= ~(MXC_F_SPI_REVA_DMA_TX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_TX_EN);
    spi->dma |= (MXC_F_SPI_REVA_DMA_TX_FLUSH);

    // Revert to previous state.
    MXC_SETFIELD(spi->dma, (MXC_F_SPI_REVA_DMA_TX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_TX_EN),
                 save_state);

    return E_NO_ERROR;
}

int MXC_SPI_RevA2_ClearRXFIFO(mxc_spi_reva_regs_t *spi)
{
    uint32_t save_state;

    save_state = (spi->dma & (MXC_F_SPI_REVA_DMA_RX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_RX_EN));

    // Disable FIFOs before clearing as recommended by UG.
    spi->dma &= ~(MXC_F_SPI_REVA_DMA_RX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_RX_EN);
    spi->dma |= (MXC_F_SPI_REVA_DMA_RX_FLUSH);

    // Revert to previous state.
    MXC_SETFIELD(spi->dma, (MXC_F_SPI_REVA_DMA_RX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_RX_EN),
                 save_state);

    return E_NO_ERROR;
}

int MXC_SPI_RevA2_SetTXThreshold(mxc_spi_reva_regs_t *spi, uint8_t thd_val)
{
    // Valid values for the threshold are 0x1 to 0x1F
    if (thd_val > (MXC_SPI_FIFO_DEPTH - 1) || thd_val == 0) {
        return E_BAD_PARAM;
    }

    MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_TX_THD_VAL,
                 thd_val << MXC_F_SPI_REVA_DMA_TX_THD_VAL_POS);

    return E_NO_ERROR;
}

int MXC_SPI_RevA2_SetRXThreshold(mxc_spi_reva_regs_t *spi, uint8_t thd_val)
{
    if (thd_val >= (MXC_SPI_FIFO_DEPTH - 1)) {
        return E_BAD_PARAM;
    }

    MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_RX_THD_VAL,
                 thd_val << MXC_F_SPI_REVA_DMA_RX_THD_VAL_POS);

    return E_NO_ERROR;
}

uint8_t MXC_SPI_RevA2_GetTXThreshold(mxc_spi_reva_regs_t *spi)
{
    return (spi->dma & MXC_F_SPI_REVA_DMA_TX_THD_VAL) >> MXC_F_SPI_REVA_DMA_TX_THD_VAL_POS;
}

uint8_t MXC_SPI_RevA2_GetRXThreshold(mxc_spi_reva_regs_t *spi)
{
    return (spi->dma & MXC_F_SPI_REVA_DMA_RX_THD_VAL) >> MXC_F_SPI_REVA_DMA_RX_THD_VAL_POS;
}

/* ** DMA-Specific Functions ** */

// Available for switching between DMA and non-DMA transactions
int MXC_SPI_RevA2_DMA_Init(mxc_spi_reva_regs_t *spi, mxc_dma_reva_regs_t *dma, bool use_dma_tx,
                           bool use_dma_rx)
{
    int error;
    int tx_ch, rx_ch; // For readability.
    int8_t spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    if (dma == NULL) {
        return E_NULL_PTR;
    }

    if (STATES[spi_num].dma_initialized) {
        // Exit function if DMA is already initialized.
        return E_NO_ERROR;
    }

    STATES[spi_num].dma = dma;

#if (MXC_DMA_INSTANCES == 1)
    error = MXC_DMA_Init();
#else
    error = MXC_DMA_Init(dma);
#endif
    if (error != E_NO_ERROR) {
        return error;
    }

    // Set up SPI DMA TX.
    if (use_dma_tx) {
        STATES[spi_num].tx_dma_ch = MXC_DMA_AcquireChannel();
        tx_ch = STATES[spi_num].tx_dma_ch;

        if (STATES[spi_num].tx_dma_ch < 0) {
            return E_NONE_AVAIL;
        }

        // TX Channel
        STATES[spi_num].dma->ch[tx_ch].ctrl |= (MXC_F_DMA_REVA_CTRL_CTZ_IE);
        STATES[spi_num].dma->inten |= (1 << tx_ch);
    }

    // Set up SPI DMA RX.
    if (use_dma_rx) {
        STATES[spi_num].rx_dma_ch = MXC_DMA_AcquireChannel();
        rx_ch = STATES[spi_num].rx_dma_ch;

        if (STATES[spi_num].rx_dma_ch < 0) {
            return E_NONE_AVAIL;
        }

        // RX Channel
        STATES[spi_num].dma->ch[rx_ch].ctrl |= (MXC_F_DMA_REVA_CTRL_CTZ_IE);
        STATES[spi_num].dma->inten |= (1 << rx_ch);
    }

    error = MXC_SPI_DMA_SetRequestSelect((mxc_spi_regs_t *)spi, use_dma_tx, use_dma_rx);
    if (error != E_NO_ERROR) {
        return error;
    }

    STATES[spi_num].dma_initialized = true;

    return E_NO_ERROR;
}

// Available to chech whether DMA is already initialized for SPI instance.
//      Useful for switching from non-DMA to DMA transactions.
bool MXC_SPI_RevA2_DMA_GetInitialized(mxc_spi_reva_regs_t *spi)
{
    return (STATES[MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi)].dma_initialized);
}

int MXC_SPI_RevA2_DMA_GetTXChannel(mxc_spi_reva_regs_t *spi)
{
    return (STATES[MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi)].tx_dma_ch);
}

int MXC_SPI_RevA2_DMA_GetRXChannel(mxc_spi_reva_regs_t *spi)
{
    return (STATES[MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi)].rx_dma_ch);
}

int MXC_SPI_RevA2_DMA_SetRequestSelect(mxc_spi_reva_regs_t *spi, uint32_t tx_reqsel,
                                       uint32_t rx_reqsel)
{
    int8_t spi_num;
    uint32_t tx_ch;
    uint32_t rx_ch;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    // Ensure DMA was configured before setting DMA Request Selects.
    if (STATES[spi_num].dma == NULL) {
        return E_BAD_STATE;
    }

    tx_ch = STATES[spi_num].tx_dma_ch;
    rx_ch = STATES[spi_num].rx_dma_ch;

    // This function will overwrite the current DMA TX/RX Request Selects.
    if (tx_reqsel != -1) {
        STATES[spi_num].dma->ch[tx_ch].ctrl |= tx_reqsel;
    }

    if (rx_reqsel != -1) {
        STATES[spi_num].dma->ch[rx_ch].ctrl |= rx_reqsel;
    }

    return E_NO_ERROR;
}

// Swaps the upper and lower byte of 2-byte wide frame transactions.
// HW Bug: For 2-byte wide frame transactions, RX DMA swaps the
//      LSB and MSB.
// Example: TX: 0x1234 => RX: 0x3412
// Note: Use __REV assembly instruction for quicker Swap implementation.
void MXC_SPI_RevA2_DMA_SwapByte(uint8_t *buffer, uint32_t length_bytes)
{
    int i;

    MXC_ASSERT(buffer != NULL);

    for (i = 0; i < length_bytes; i += 2) {
        uint8_t temp = buffer[i];
        buffer[i] = buffer[i + 1];
        buffer[i + 1] = temp;
    }
}

/* ** Transaction Helper Functions ** */

// SPI DMA/non-DMA Transaction Setup Helper Function.
static void MXC_SPI_RevA2_transactionSetup(mxc_spi_reva_regs_t *spi, uint8_t *tx_buffer,
                                           uint32_t tx_length_frames, uint8_t *rx_buffer,
                                           uint32_t rx_length_frames, bool use_dma)
{
    int tx_dummy_length_frames;
    int8_t spi_num;
    // For readability purposes.
    int rx_ch, tx_ch;

    // Ensure valid SPI Instance.
    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    // Initialize SPIn state to handle data.
    STATES[spi_num].transaction_done = false;

    STATES[spi_num].tx_buffer = tx_buffer;
    STATES[spi_num].tx_count_bytes = 0;
    STATES[spi_num].tx_done = false;

    STATES[spi_num].rx_buffer = rx_buffer;
    STATES[spi_num].rx_count_bytes = 0;
    STATES[spi_num].rx_done = false;

    // Max number of frames to transmit/receive.
    MXC_ASSERT(tx_length_frames <
               (MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR >> MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS));
    MXC_ASSERT(rx_length_frames <
               (MXC_F_SPI_REVA_CTRL1_RX_NUM_CHAR >> MXC_F_SPI_REVA_CTRL1_RX_NUM_CHAR_POS));

    // STATES[n] TX/RX Length Fields are in terms of number of bytes to send/receive.
    if (STATES[spi_num].frame_size <= 8) {
        STATES[spi_num].tx_length_bytes = tx_length_frames;
        STATES[spi_num].rx_length_bytes = rx_length_frames;
    } else {
        STATES[spi_num].tx_length_bytes = tx_length_frames * 2;
        STATES[spi_num].rx_length_bytes = rx_length_frames * 2;
    }

    // Set the number of messages to transmit/receive for the SPI transaction.
    if (STATES[spi_num].if_mode == MXC_SPI_INTERFACE_STANDARD) {
        if (rx_length_frames > tx_length_frames) {
            // In standard 4-wire mode, the RX_NUM_CHAR field of ctrl1 is ignored.
            // The number of bytes to transmit AND receive is set by TX_NUM_CHAR,
            // because the hardware always assume full duplex. Therefore extra
            // dummy bytes must be transmitted to support half duplex.
            tx_dummy_length_frames = rx_length_frames - tx_length_frames;

            // Check whether new frame length exceeds the possible number of frames to transmit.
            MXC_ASSERT((tx_length_frames + tx_dummy_length_frames) <
                       (MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR >> MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS));

            spi->ctrl1 = ((tx_length_frames + tx_dummy_length_frames)
                          << MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS);
        } else {
            spi->ctrl1 = (tx_length_frames << MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS);
        }
    } else { // mode != MXC_SPI_INTERFACE_STANDARD
        spi->ctrl1 = (tx_length_frames << MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS) |
                     (rx_length_frames << MXC_F_SPI_REVA_CTRL1_RX_NUM_CHAR_POS);
    }

    // Disable FIFOs before clearing as recommended by UG.
    spi->dma &= ~(MXC_F_SPI_REVA_DMA_TX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_TX_EN |
                  MXC_F_SPI_REVA_DMA_RX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_RX_EN);
    spi->dma |= (MXC_F_SPI_REVA_DMA_TX_FLUSH | MXC_F_SPI_REVA_DMA_RX_FLUSH);

    //>>> Start of SPI DMA transaction setup.
    if (use_dma) {
        // Enable TX FIFO before configuring.
        spi->dma |= (MXC_F_SPI_REVA_DMA_TX_FIFO_EN);

        // Set TX and RX Thresholds before loading FIFO.
        MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_TX_THD_VAL,
                     ((MXC_SPI_FIFO_DEPTH - 1) << MXC_F_SPI_REVA_DMA_TX_THD_VAL_POS));
        MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_RX_THD_VAL,
                     (0 << MXC_F_SPI_REVA_DMA_RX_THD_VAL_POS));

        // Set up DMA TX Transactions.
        // Note: Number of transmitting frames greatly depends on the SPI DMA register settings for
        //      the DMA burst size and TX Threshold values.
        // 1) For TX transmissions.
        if (tx_length_frames > 1) {
            // For readability purposes.
            tx_ch = STATES[spi_num].tx_dma_ch;

            // Configure DMA TX depending on frame width.
            // 2-8 bit wide frames.
            if (STATES[spi_num].frame_size <= 8) {
                // Hardware requires writing the first byte into the FIFO manually.
                spi->fifo8[0] = tx_buffer[0];

                // Threshold set to 2 frames (2 bytes) after pre-loading first byte for DMA.
                //  This is the minimum threshold to handle any number of transmitting frames.
                //  Note: This case is handling TX transactions of greater than 1 frame.
                //        Threshold of 1 frame does not work.
                MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_TX_THD_VAL,
                             (2 << MXC_F_SPI_REVA_DMA_TX_THD_VAL_POS));

                STATES[spi_num].dma->ch[tx_ch].src = (uint32_t)(tx_buffer + 1); // 1 Byte offset
                STATES[spi_num].dma->ch[tx_ch].cnt = (tx_length_frames - 1);

                // Set to 3 bytes (3 frames) burst size.
                //  Due to design: burst_size = threshold + 1
                //  Note: Assigning value of 2 to register-field equals 3 bytes transferred in/out of DMA.
                //        Add 1 to the register-field setting to get the number of bytes for burst.
                MXC_SETFIELD(STATES[spi_num].dma->ch[tx_ch].ctrl, MXC_F_DMA_REVA_CTRL_BURST_SIZE,
                             (2 << MXC_F_DMA_REVA_CTRL_BURST_SIZE_POS));

                // Set source and destination width to one byte.
                MXC_SETFIELD(STATES[spi_num].dma->ch[tx_ch].ctrl, MXC_F_DMA_REVA_CTRL_SRCWD,
                             MXC_S_DMA_REVA_CTRL_SRCWD_BYTE);
                MXC_SETFIELD(STATES[spi_num].dma->ch[tx_ch].ctrl, MXC_F_DMA_REVA_CTRL_DSTWD,
                             MXC_S_DMA_REVA_CTRL_DSTWD_BYTE);

                // 9-16 bit wide frames.
            } else {
                // Hardware requires writing the first bytes into the FIFO manually.
                STATES[spi_num].tx_count_bytes +=
                    MXC_SPI_RevA2_writeTXFIFO16(spi, (uint8_t *)(STATES[spi_num].tx_buffer), 2);

                // Threshold set to 3 frames (6 bytes) after pre-loading FIFO for DMA.
                //  This is the minimum threshold to handle any number of transmitting frames.
                //  Note: This case is handling TX transactions of greater than 1 frame.
                //        Threshold of 1 or 2 frames does not work.
                MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_TX_THD_VAL,
                             (3 << MXC_F_SPI_REVA_DMA_TX_THD_VAL_POS));

                STATES[spi_num].dma->ch[tx_ch].src =
                    (uint32_t)(tx_buffer + STATES[spi_num].tx_count_bytes);
                STATES[spi_num].dma->ch[tx_ch].cnt =
                    (STATES[spi_num].tx_length_bytes - STATES[spi_num].tx_count_bytes);

                // Set to 4 bytes (2 frames) burst size.
                //  Due to design: burst_size = threshold + 1
                //  Note: Assigning value of 3 to register-field equals 4 bytes.
                //        Add 1 to the register-field setting to get the number of bytes for burst.
                MXC_SETFIELD(STATES[spi_num].dma->ch[tx_ch].ctrl, MXC_F_DMA_REVA_CTRL_BURST_SIZE,
                             (3 << MXC_F_DMA_REVA_CTRL_BURST_SIZE_POS));

                // Set source and destination width to two bytes.
                MXC_SETFIELD(STATES[spi_num].dma->ch[tx_ch].ctrl, MXC_F_DMA_REVA_CTRL_SRCWD,
                             MXC_S_DMA_REVA_CTRL_SRCWD_HALFWORD);
                MXC_SETFIELD(STATES[spi_num].dma->ch[tx_ch].ctrl, MXC_F_DMA_REVA_CTRL_DSTWD,
                             MXC_S_DMA_REVA_CTRL_DSTWD_HALFWORD);
            }

            STATES[spi_num].dma->ch[tx_ch].ctrl |= MXC_F_DMA_REVA_CTRL_SRCINC;
            STATES[spi_num].dma->ch[tx_ch].ctrl |= MXC_F_DMA_REVA_CTRL_EN; // Start the DMA

            // 2) For single character transmissions.
            //    NOTE: Single-length transmissions does not trigger CTZ.
        } else if (tx_length_frames == 1) {
            // Write first frame into FIFO.
            if (STATES[spi_num].frame_size <= 8) {
                spi->fifo8[0] = tx_buffer[0];
            } else {
                MXC_SPI_RevA2_writeTXFIFO16(spi, (uint8_t *)(STATES[spi_num].tx_buffer), 2);
            }

            // If there is no RX DMA and only one frame is transmitted, then
            //  the transaction is done. Single-length transmissions
            //  does not trigger a CTZ interrupt.
            if (rx_length_frames > 0 && rx_buffer != NULL) {
                STATES[spi_num].transaction_done = true;
            }

            STATES[spi_num].tx_done = true;

            // 3) Set up DMA TX for RX only transactions.
            //    Note: Even if you are not transmitting anything in standard 4-wire mode,
            //      the hardware always assume full duplex. Therefore dummy bytes
            //      must be transmitted to support half duplex. The number of bytes to transmit
            //      AND receive is set by TX_NUM_CHAR, and the RX_NUM_CHAR field of ctrl1 is ignored.
        } else if (tx_length_frames == 0 && STATES[spi_num].if_mode == MXC_SPI_INTERFACE_STANDARD) {
            // For readability purposes.
            tx_ch = STATES[spi_num].tx_dma_ch;

            // Configure TX DMA channel to retransmit the dummy byte.
            STATES[spi_num].dma->ch[tx_ch].src = (uint32_t)(&(STATES[spi_num].tx_dummy_value));
            STATES[spi_num].dma->ch[tx_ch].cnt = STATES[spi_num].rx_length_bytes; // Only receiving
            STATES[spi_num].dma->ch[tx_ch].ctrl &= ~MXC_F_DMA_REVA_CTRL_SRCINC;
            STATES[spi_num].dma->ch[tx_ch].ctrl |= MXC_F_DMA_REVA_CTRL_EN; // Start the DMA
        }

        // Enable SPI TX DMA after configuring.
        spi->dma |= (MXC_F_SPI_REVA_DMA_DMA_TX_EN);

        // Set up DMA RX Transactions.
        if (rx_length_frames > 0 && rx_buffer != NULL) {
            // For readability purposes.
            rx_ch = STATES[spi_num].rx_dma_ch;

            // Enable RX DMA channel before configuring.
            spi->dma |= (MXC_F_SPI_REVA_DMA_RX_FIFO_EN);

            // Set RX threshold to minimum value to handle any number of received frames.
            MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_RX_THD_VAL,
                         (0 << MXC_F_SPI_REVA_DMA_RX_THD_VAL_POS));

            STATES[spi_num].dma->ch[rx_ch].dst = (uint32_t)rx_buffer;
            STATES[spi_num].dma->ch[rx_ch].cnt = STATES[spi_num].rx_length_bytes;

            // Set to one byte burst size - minimum value to handle any number of recevied frames.
            MXC_SETFIELD(STATES[spi_num].dma->ch[rx_ch].ctrl, MXC_F_DMA_REVA_CTRL_BURST_SIZE,
                         (0 << MXC_F_DMA_REVA_CTRL_BURST_SIZE_POS));

            // Match frame size (in terms of bytes) in DMA ctrl settings.
            if (STATES[spi_num].frame_size <= 8) {
                // Set source and destination width to one byte
                MXC_SETFIELD(STATES[spi_num].dma->ch[rx_ch].ctrl, MXC_F_DMA_REVA_CTRL_SRCWD,
                             MXC_S_DMA_REVA_CTRL_SRCWD_BYTE);
                MXC_SETFIELD(STATES[spi_num].dma->ch[rx_ch].ctrl, MXC_F_DMA_REVA_CTRL_DSTWD,
                             MXC_S_DMA_REVA_CTRL_DSTWD_BYTE);
            } else {
                // Set source destination width to 2 bytes
                MXC_SETFIELD(STATES[spi_num].dma->ch[rx_ch].ctrl, MXC_F_DMA_REVA_CTRL_SRCWD,
                             MXC_S_DMA_REVA_CTRL_SRCWD_HALFWORD);
                MXC_SETFIELD(STATES[spi_num].dma->ch[rx_ch].ctrl, MXC_F_DMA_REVA_CTRL_DSTWD,
                             MXC_S_DMA_REVA_CTRL_DSTWD_HALFWORD);
            }

            STATES[spi_num].dma->ch[rx_ch].ctrl |= MXC_F_DMA_REVA_CTRL_DSTINC;
            STATES[spi_num].dma->ch[rx_ch].ctrl |= MXC_F_DMA_REVA_CTRL_EN; // Start the DMA

            // Enable SPI RX DMA after configuring.
            spi->dma |= (MXC_F_SPI_REVA_DMA_DMA_RX_EN);
        }
        //<<< End of SPI DMA transaction setup.
        //>>> Start of SPI non-DMA transaction setup.
    } else {
        // Finish setting up SPI for TX and RX.
        if (tx_length_frames > 0) {
            // Enable TX FIFO & TX Threshold crossed interrupt.
            spi->dma |= (MXC_F_SPI_REVA_DMA_TX_FIFO_EN);
            spi->inten |= MXC_F_SPI_REVA_INTEN_TX_THD;

            // Set TX Threshold to minimum value after re-enabling TX FIFO.
            MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_TX_THD_VAL,
                         (1 << MXC_F_SPI_REVA_DMA_TX_THD_VAL_POS));
        }

        if (rx_length_frames > 0) {
            // Enable RX FIFO & RX Threshold crossed interrupt.
            spi->dma |= (MXC_F_SPI_REVA_DMA_RX_FIFO_EN);
            spi->inten |= MXC_F_SPI_REVA_INTEN_RX_THD;

            // Set RX Threshold to minimum value after re-enabling RX FIFO.
            MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_RX_THD_VAL,
                         (0 << MXC_F_SPI_REVA_DMA_RX_THD_VAL_POS));
        }

        // This private function, MXC_SPI_RevA2_process, call fills the TX FIFO as much as possible
        //   before launching the transaction. Subsequent FIFO management will be handled after
        //   transaction has started.
        MXC_SPI_RevA2_process(spi);
    } //<<< End of SPI non-DMA transaction setup.
}

// Helper function that handles the Target Select assertion/deassertion at start of transaction.
// hw_ts_active_pol is either 1 or 0.
static void MXC_SPI_RevA2_handleTSControl(mxc_spi_reva_regs_t *spi, uint8_t deassert,
                                          uint8_t hw_ts_index)
{
    int8_t spi_num;

    // Ensure valid SPI Instance.
    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    // Handle target-select (L. SS) deassertion if HW is selected as Target Select (TS) Control Scheme. This must be done
    //   AFTER launching the transaction to avoid a glitch on the TS line if:
    //     - The TS line is asserted
    //     - We want to deassert the line as part of this transaction
    //
    // As soon as the SPI hardware receives CTRL0->START it seems to reinitialize the Target Select (TS) pin based
    //   on the value of CTRL->SS_CTRL, which causes the glitch.
    if (STATES[spi_num].ts_control == MXC_SPI_TSCONTROL_HW_AUTO) {
        // In HW Auto Scheme, only use the target index member.
        // Limitation: This implemention only support transactions with one target at a time.
        MXC_SETFIELD(spi->ctrl0, MXC_F_SPI_REVA_CTRL0_SS_ACTIVE,
                     ((1 << hw_ts_index) << MXC_F_SPI_REVA_CTRL0_SS_ACTIVE_POS));

        if (deassert) {
            spi->ctrl0 &= ~MXC_F_SPI_REVA_CTRL0_SS_CTRL;
        } else {
            spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_SS_CTRL;
        }
    }

    // Add support for SW_DRV TS Control here in the future.
}

/* ** Transaction Functions ** */

int MXC_SPI_RevA2_ControllerTransaction(mxc_spi_reva_regs_t *spi, uint8_t *tx_buffer,
                                        uint32_t tx_length_frames, uint8_t *rx_buffer,
                                        uint32_t rx_length_frames, uint8_t deassert,
                                        uint8_t hw_ts_index)
{
    int8_t spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    // Make sure DMA is not initialized.
    if (STATES[spi_num].dma_initialized == true) {
        return E_BAD_STATE;
    }

    // Make sure SPI Instance is in Controller mode (L. Master).
    if (STATES[spi_num].controller_target != MXC_SPI_TYPE_CONTROLLER) {
        return E_BAD_STATE;
    }

    // Save target settings.
    STATES[spi_num].deassert = deassert;

    // Setup SPI registers for non-DMA transaction.
    MXC_SPI_RevA2_transactionSetup(spi, tx_buffer, tx_length_frames, rx_buffer, rx_length_frames,
                                   false);

    // Start the SPI transaction.
    spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_START;

    // Handle Target Select Pin (Only applicable in HW_AUTO TS control scheme).
    MXC_SPI_RevA2_handleTSControl(spi, deassert, hw_ts_index);

    // Complete transaction once it started.
    while (STATES[spi_num].transaction_done == false) {
        if (STATES[spi_num].tx_done == true && STATES[spi_num].rx_done == true) {
            if (!(spi->stat & MXC_F_SPI_REVA_STAT_BUSY)) {
                STATES[spi_num].transaction_done = true;
            }
        }

        MXC_SPI_RevA2_process(spi);
    }

    return E_SUCCESS;
}

int MXC_SPI_RevA2_ControllerTransactionAsync(mxc_spi_reva_regs_t *spi, uint8_t *tx_buffer,
                                             uint32_t tx_length_frames, uint8_t *rx_buffer,
                                             uint32_t rx_length_frames, uint8_t deassert,
                                             uint8_t hw_ts_index)
{
    int8_t spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    // Make sure DMA is not initialized.
    if (STATES[spi_num].dma_initialized == true) {
        return E_BAD_STATE;
    }

    // Make sure SPI Instance is in Controller mode (L. Master).
    if (STATES[spi_num].controller_target != MXC_SPI_TYPE_CONTROLLER) {
        return E_BAD_STATE;
    }

    // Save target settings.
    STATES[spi_num].deassert = deassert;

    // Setup SPI registers for non-DMA transaction.
    MXC_SPI_RevA2_transactionSetup(spi, tx_buffer, tx_length_frames, rx_buffer, rx_length_frames,
                                   false);

    // Enable Controller Done Interrupt.
    spi->inten |= MXC_F_SPI_REVA_INTEN_MST_DONE;

    // Start the SPI transaction.
    spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_START;

    // Handle Target Select Pin (Only applicable in HW_AUTO TS control scheme).
    MXC_SPI_RevA2_handleTSControl(spi, deassert, hw_ts_index);

    return E_SUCCESS;
}

int MXC_SPI_RevA2_ControllerTransactionDMA(mxc_spi_reva_regs_t *spi, uint8_t *tx_buffer,
                                           uint32_t tx_length_frames, uint8_t *rx_buffer,
                                           uint32_t rx_length_frames, uint8_t deassert,
                                           uint8_t hw_ts_index, mxc_dma_reva_regs_t *dma)
{
    int8_t spi_num;
    int error;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    // More overhead, but this function will initialize DMA if it wasn't done earlier.
    if (STATES[spi_num].dma_initialized == false) {
        error = MXC_SPI_RevA2_DMA_Init(spi, dma, true, true);
        if (error != E_NO_ERROR) {
            return error;
        }
    }

    // Make sure SPI Instance is in Controller mode (L. Master).
    if (STATES[spi_num].controller_target != MXC_SPI_TYPE_CONTROLLER) {
        return E_BAD_STATE;
    }

    // Save target settings.
    STATES[spi_num].deassert = deassert;

    // Setup SPI registers for non-DMA transaction.
    MXC_SPI_RevA2_transactionSetup(spi, tx_buffer, tx_length_frames, rx_buffer, rx_length_frames,
                                   true);

    // Start the SPI transaction.
    spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_START;

    // Handle Target Select Pin (Only applicable in HW_AUTO TS control scheme).
    MXC_SPI_RevA2_handleTSControl(spi, deassert, hw_ts_index);

    return E_SUCCESS;
}

int MXC_SPI_RevA2_TargetTransaction(mxc_spi_reva_regs_t *spi, uint8_t *tx_buffer,
                                    uint32_t tx_length_frames, uint8_t *rx_buffer,
                                    uint32_t rx_length_frames)
{
    int8_t spi_num;

    // Ensure valid SPI Instance.
    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    // Make sure DMA is not initialized.
    if (STATES[spi_num].dma_initialized == true) {
        return E_BAD_STATE;
    }

    // Make sure SPI Instance is in Target mode (L. Slave).
    if (STATES[spi_num].controller_target != MXC_SPI_TYPE_TARGET) {
        return E_BAD_STATE;
    }

    // Setup SPI registers for non-DMA transaction.
    MXC_SPI_RevA2_transactionSetup(spi, tx_buffer, tx_length_frames, rx_buffer, rx_length_frames,
                                   false);

    // Wait for Target Select pin to be asserted before starting transaction.
    while ((spi->stat & MXC_F_SPI_REVA_STAT_BUSY) == 0) {}

    // Complete transaction once started.
    while (STATES[spi_num].transaction_done == false) {
        if (STATES[spi_num].tx_count_bytes == STATES[spi_num].tx_length_bytes &&
            STATES[spi_num].rx_count_bytes == STATES[spi_num].rx_length_bytes) {
            STATES[spi_num].transaction_done = true;
        }

        MXC_SPI_RevA2_process(spi);
    }

    // Wait until transaction is complete.
    while (spi->stat & MXC_F_SPI_REVA_STAT_BUSY) {}

    return E_SUCCESS;
}

int MXC_SPI_RevA2_TargetTransactionAsync(mxc_spi_reva_regs_t *spi, uint8_t *tx_buffer,
                                         uint32_t tx_length_frames, uint8_t *rx_buffer,
                                         uint32_t rx_length_frames)
{
    int8_t spi_num;

    // Ensure valid SPI Instance.
    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    // Make sure DMA is not initialized.
    if (STATES[spi_num].dma_initialized == true) {
        return E_BAD_STATE;
    }

    // Make sure SPI Instance is in Target mode (L. Slave).
    if (STATES[spi_num].controller_target != MXC_SPI_TYPE_TARGET) {
        return E_BAD_STATE;
    }

    // Setup SPI registers for non-DMA transaction.
    MXC_SPI_RevA2_transactionSetup(spi, tx_buffer, tx_length_frames, rx_buffer, rx_length_frames,
                                   false);

    return E_SUCCESS;
}

int MXC_SPI_RevA2_TargetTransactionDMA(mxc_spi_reva_regs_t *spi, uint8_t *tx_buffer,
                                       uint32_t tx_length_frames, uint8_t *rx_buffer,
                                       uint32_t rx_length_frames, mxc_dma_reva_regs_t *dma)
{
    int8_t spi_num;
    int error;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    // More overhead, but this function will initialize DMA if it wasn't done earlier.
    if (STATES[spi_num].dma_initialized == false) {
        error = MXC_SPI_RevA2_DMA_Init(spi, dma, true, true);
        if (error != E_NO_ERROR) {
            return error;
        }
    }

    // Make sure SPI Instance is in Target mode (L. Slave).
    if (STATES[spi_num].controller_target != MXC_SPI_TYPE_TARGET) {
        return E_BAD_STATE;
    }

    // Setup SPI registers for DMA transaction.
    MXC_SPI_RevA2_transactionSetup(spi, tx_buffer, tx_length_frames, rx_buffer, rx_length_frames,
                                   true);

    // Target transaction is ready.
    return E_SUCCESS;
}

/* ** Handler Functions ** */

void MXC_SPI_RevA2_Handler(mxc_spi_reva_regs_t *spi)
{
    int8_t spi_num;
    uint32_t status = spi->intfl;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    // Master done (TX complete)
    if (status & MXC_F_SPI_REVA_INTFL_MST_DONE) {
        spi->intfl |= MXC_F_SPI_REVA_INTFL_MST_DONE; // Clear flag

        // Callback if valid.
        // Note: If Target Select (TS) Control Scheme is set in SW_App mode, then the caller needs to ensure the
        //   Target Select (TS) pin is asserted or deasserted in their application.
        if (STATES[spi_num].callback) {
            STATES[spi_num].callback(STATES[spi_num].callback_data, E_NO_ERROR);
        }

        // Controller is done after callback (if valid) is handled.
        STATES[spi_num].transaction_done = true;
    }

    // Handle RX Threshold
    if (status & MXC_F_SPI_REVA_INTFL_RX_THD) {
        spi->intfl |= MXC_F_SPI_REVA_INTFL_RX_THD;

        // RX threshold has been crossed, there's data to unload from the FIFO
        MXC_SPI_RevA2_process(spi);
    }

    // Handle TX Threshold
    if (status & MXC_F_SPI_REVA_INTFL_TX_THD) {
        spi->intfl |= MXC_F_SPI_REVA_INTFL_TX_THD;

        // TX threshold has been crossed, we need to refill the FIFO
        MXC_SPI_RevA2_process(spi);
    }
}

void MXC_SPI_RevA2_DMA_TX_Handler(mxc_spi_reva_regs_t *spi)
{
    int8_t spi_num;
    uint32_t tx_ch;
    uint32_t status;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    tx_ch = STATES[spi_num].tx_dma_ch;
    status = STATES[spi_num].dma->ch[tx_ch].status;

    // Count-to-Zero (DMA TX complete)
    if (status & MXC_F_DMA_REVA_STATUS_CTZ_IF) {
        STATES[spi_num].tx_done = true;
        STATES[spi_num].dma->ch[tx_ch].status |= MXC_F_DMA_REVA_STATUS_CTZ_IF;

        // For completeness-sake.
        STATES[spi_num].tx_count_bytes = STATES[spi_num].tx_length_bytes;

        // Callback if valid and if you're only transmitting.
        // Note: If Target Select (TS) Control Scheme is set in SW_App mode, then the caller needs to ensure the
        //   Target Select (TS) pin is asserted or deasserted in their application.
        if (STATES[spi_num].rx_buffer == NULL) {
            if (STATES[spi_num].callback) {
                STATES[spi_num].callback(STATES[spi_num].callback_data, E_NO_ERROR);
            }
        }

        // TX Transaction is done if there's no RX transaction.
        if (STATES[spi_num].rx_length_bytes == 0 || STATES[spi_num].tx_buffer == NULL) {
            STATES[spi_num].transaction_done = true;
        }
    }

    // Bus Error
    if (status & MXC_F_DMA_REVA_STATUS_BUS_ERR) {
        STATES[spi_num].dma->ch[tx_ch].status |= MXC_F_DMA_REVA_STATUS_BUS_ERR;
    }
}

void MXC_SPI_RevA2_DMA_RX_Handler(mxc_spi_reva_regs_t *spi)
{
    int8_t spi_num;
    uint32_t rx_ch;
    uint32_t status;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    rx_ch = STATES[spi_num].rx_dma_ch;
    status = STATES[spi_num].dma->ch[rx_ch].status;

    // Count-to-Zero (DMA RX complete).
    if (status & MXC_F_DMA_REVA_STATUS_CTZ_IF) {
        // HW Bug: For 2-byte wide frame transactions, RX DMA swaps the
        //      LSB and MSB.
        // Example: TX: 0x1234 => RX: 0x3412
        if (STATES[spi_num].frame_size > 8) {
            MXC_SPI_RevA2_DMA_SwapByte(STATES[spi_num].rx_buffer, STATES[spi_num].rx_length_bytes);
        }

        STATES[spi_num].rx_done = 1;
        STATES[spi_num].dma->ch[rx_ch].status |= MXC_F_DMA_STATUS_CTZ_IF;

        // For completeness-sake.
        STATES[spi_num].rx_count_bytes = STATES[spi_num].rx_length_bytes;

        // Callback if valid.
        // Note: If Target Select (TS) Control Scheme is set in SW_App mode, then the caller needs to ensure the
        //   Target Select (TS) pin is asserted or deasserted in their application.
        if (STATES[spi_num].callback) {
            STATES[spi_num].callback(STATES[spi_num].callback_data, E_NO_ERROR);
        }

        // RX transaction determines the controller is done if TX transaction is also present.
        if (STATES[spi_num].tx_length_bytes > 0 && STATES[spi_num].tx_buffer != NULL) {
            STATES[spi_num].transaction_done = true;
        }
    }

    // Bus Error
    if (status & MXC_F_DMA_REVA_STATUS_BUS_ERR) {
        STATES[spi_num].dma->ch[rx_ch].status |= MXC_F_DMA_STATUS_BUS_ERR;
    }
}
