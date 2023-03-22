/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
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
#include "spi_fast.h"
#include "spi_revb.h"
#include "dma_reva.h"

/* **** Definitions **** */

// clang-format off
typedef struct {
    // Info from initialization.
    bool                initialized;
    mxc_spi_init_t      init;
// TODO: Copy what you need

    // Transaction Data.
    uint16_t            *tx_buffer;
    uint32_t            tx_len;
    uint16_t            *rx_buffer;
    uint32_t            rx_len;
    
    // Chip Select Info.
    bool                deassert; // CS Deasserted at the end of a transmission.
    mxc_spi_target_t    current_cs_cfg;

    // DMA Settings.
    mxc_dma_reva_regs_t *dma;
    int                 tx_dma_ch;
    int                 rx_dma_ch;
// TODO: Should the request select data be saved or should there be a function that
//  sets the proper Request Selects bits in DMA registers and call that function in
//  chip-specific level Init function
// Make a function for these two
    int                 tx_dma_reqsel;
    int                 rx_dma_reqsel;

    // Status Fields.
    bool                controller_done; // Master
    bool                target_done;     // Slave
    bool                tx_done;
    bool                rx_done;
} mxc_spi_handle_data_t;
// clang-format on

static volatile mxc_spi_handle_data_t STATES[MXC_SPI_INSTANCES];

/* **** Functions **** */

// Private function in drivers will not have starting capital in name
static void MXC_SPI_RevB_process(mxc_spi_reva_regs_t *spi)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);

    // Unload any SPI data that has come in
    //  Dependent on 1) Valid RX Buffer, 2) RX Length not 0, and 3) RX FIFO Not Empty.
    while(STATES[spi_num].rx_buffer && STATES[spi_num].rx_len > 0 && (spi->dma & MXC_F_SPI_REVA_DMA_RX_LVL)) {
        *(STATES[spi_num].rx_buffer)++ = spi->fifo32;
        STATES[spi_num].rx_len--;
    }

    // Write any pending bytes out.
    //  Dependent on 1) Valid TX Buffer, 2) TX Length not 0, and 3) TX FIFO Not Empty.
    while(STATES[spi_num].tx_buffer && STATES[spi_num].tx_len > 0 && (((spi->dma & MXC_F_SPI_REVA_DMA_TX_LVL) >> MXC_F_SPI_REVA_DMA_TX_LVL_POS) < MXC_SPI_FIFO_DEPTH)) {
        spi->fifo32 = *(STATES[spi_num].tx_buffer)++;
        STATES[spi_num].tx_len--;
    }
}

static int MXC_SPI_RevB_resetStateStruct(int spi_num)
{
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // Init Data
    STATES[spi_num].init = NULL;
    STATES[spi_num].cs_pins = NULL;

    // Transaction Members
    STATES[spi_num].tx_buffer = NULL;
    STATES[spi_num].tx_len = 0;
    STATES[spi_num].rx_buffer = NULL;
    STATES[spi_num].rx_len = 0;
    STATES[spi_num].deassert = true;    // Default state is CS will be deasserted at the end of a transmission.

    // DMA
    STATES[spi_num].dma = NULL;
    STATES[spi_num].tx_dma_ch = -1;
    STATES[spi_num].rx_dma_ch = -1;
    STATES[spi_num].tx_dma_reqsel = -1;
    STATES[spi_num].rx_dma_reqsel = -1;

    // Status Members
    STATES[spi_num].controller_done = false;
    STATES[spi_num].tx_done = false;
    STATES[spi_num].rx_done = false;
    STATES[spi_num].target_done = false;

    return E_NO_ERROR;
}

static int MXC_SPI_RevB_resetAllStateStructs(int spi_num)
{
    int i;
    int error;

    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    for (i = 0; i < MXC_SPI_INSTANCES; i++) {
        error = MXC_SPI_RevB_resetStateStruct(i);
        if (error != E_NO_ERROR) {
            return error;
        }
    }

    return E_NO_ERROR;
}


int MXC_SPI_RevB_Init(mxc_spi_init_t *init)
{
    int error, spi_num;
    mxc_spi_target_t *cs_cfg;

    if (init == NULL) {
        return E_NULL_PTR;
    }

    // Ensure valid SPI instance.
    spi_num = MXC_SPI_GET_IDX(init->spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // For code readability.
    cs_cfg = &(init->cs_config);
    if (cs_cfg == NULL) {
        return E_NULL_PTR;
    }

    // Reset STATE of current SPI instance.
    error = MXC_SPI_RevB_resetStateStruct(spi_num);
    if (error != E_NO_ERROR) {
        return error;
    }

    // Save init data for transactions and handlers.
    STATES[spi_num].init = *init;
    STATES[spi_num].dma = NULL;
    STATES[spi_num].cs_pins = init->cs_pins;

    // Set up CS Control Scheme.
    if (init->cs_control == MXC_SPI_CSCONTROL_HW_AUTO) {
        // If hardware is handling CS pin, make sure the correct alternate function is chosen.
        if ((cs_cfg->pins != NULL) && (cs_cfg->pins.func != MXC_GPIO_FUNC_OUT)) {
            error = MXC_GPIO_Config(&(init->cs_pins));
            if (error != E_NO_ERROR) {
                return error;
            }

            (init->spi)->ctrl0 |= (cs_cfg->index << MXC_F_SPI_REVA_CTRL0_SS_ACTIVE_POS);

            // Set CS Polarity (Default, active low (0))
            if (cs_cfg->active_polarity == 0) {
                (init->spi)->ctrl2 &= ~(cs_cfg->index << MXC_F_SPI_REVA_CTRL2_SS_POL_POS);
            } else {
                (init->spi)->ctrl2 |= (cs_cfg->index << MXC_F_SPI_REVA_CTRL2_SS_POL_POS);
            }

        } else {
            return E_BAD_PARAM;
        }

    } else if (init->cs_control == MXC_SPI_CSCONTROL_SW_DRV) {
        // If SPI driver is handling CS pin, make sure the pin function is in OUT mode.
        if ((cs_cfg->pins != NULL) && (cs_cfg->pins.func == MXC_GPIO_FUNC_OUT)) {
            error = MXC_GPIO_Config(&(cs_cfg->pins));
            if (error != E_NO_ERROR) {
                return error;
            }

            // Set CS Polarity 
            if (cs_cfg->active_polarity == 0) {
                // Active LOW (0), Set CS Idle State to HIGH (1)
                cs_cfg->pins.port->out_set |= cs_cfg->pins.mask;
            } else {
                // Active HIGH (1), Set CS Idle State to LOW (0)
                cs_cfg->pins.port->out_clr |= cs_cfg->pins.mask;
            }

        } else {
            return E_BAD_PARAM;
        }

    // Don't do anything if SW Application is handling CS pin.
    } else if (init->cs_control != MXC_SPI_CSCONTROL_SW_APP) {
        // Not a valid CS Control option.
        return E_BAD_PARAM;
    }

    // Enable SPI port.
    (init->spi)->ctrl0 |= MXC_F_SPI_REVA_CTRL0_EN;

    // Select Controller (L. Master) or Target (L. Slave) Mode.
    if (init->type == MXC_SPI_TYPE_CONTROLLER) {
        (init->spi)->ctrl0 |= MXC_F_SPI_REVA_CTRL0_MST_MODE;
    } else {
        (init->spi)->ctrl0 &= ~MXC_F_SPI_REVA_CTRL0_MST_MODE;
    }

//TODO: Can probably replace this with the SetDataSize Function.
    // Set character size
    if (init->data_size <= 1 || init->data_size > 16) {
        return E_BAD_PARAM;
    } else {
        (init->spi)->ctrl2 |= (init->data_size) << MXC_F_SPI_REVA_CTRL2_NUMBITS_POS;
    }

    // Remove any delay between SS and SCLK edges.
    (init->spi)->sstime = (1 << MXC_F_SPI_REVA_SSTIME_PRE_POS) | (1 << MXC_F_SPI_REVA_SSTIME_POST_POS) | (1 << MXC_F_SPI_REVA_SSTIME_INACT_POS);

    // Enable TX/RX FIFOs
    (init->spi)->dma |= MXC_F_SPI_REVA_DMA_TX_FIFO_EN | MXC_F_SPI_REVA_DMA_RX_FIFO_EN;
    
    // Set TX and RX Threshold to 31 and 0, respectively.
    MXC_SETFIELD((init->spi)->dma, MXC_F_SPI_REVA_DMA_TX_THD_VAL, (31 << MXC_F_SPI_REVA_DMA_TX_THD_VAL_POS));
    MXC_SETFIELD((init->spi)->dma, MXC_F_SPI_REVA_DMA_RX_THD_VAL, (0 << MXC_F_SPI_REVA_DMA_RX_THD_VAL_POS));

    error = MXC_SPI_SetFrequency((init->spi), (init->freq));
    if (error != E_NO_ERROR) {
        return error;
    }

    // Clear any interrupt flags that may already be set.
    (init->spi)->intfl = (init->spi)->intfl;

    // Setup DMA features if used.
    if (init->use_dma) {
        // Even though the Init Struct has a pointer to the DMA instance,
        //   this will make the code a bit more readable since the DMA
        //   instance is now type casted with the DMA RevA Registers.
        STATES[spi_num].dma = (mxc_dma_reva_regs_t *)(init->dma);

#if (MXC_DMA_INSTANCES == 1)
        error = MXC_DMA_Init();
#else
        error = MXC_DMA_Init(init->dma);
#endif
        if (error != E_NO_ERROR) {
            return error;
        }

        // Acquire DMA Channels for SPI TX/RX
        STATES[spi_num].tx_dma_ch = MXC_DMA_AcquireChannel();
        STATES[spi_num].rx_dma_ch = MXC_DMA_AcquireChannel();

        // Check if failed to acquire channel.
        if (STATES[spi_num].tx_dma_ch < 0 || STATES[spi_num].rx_dma_ch < 0) {
            return E_NONE_AVAIL;
        }

// TODO: Caller will deal with enabling SetVector
        // MXC_NVIC_SetVector(MXC_DMA_GET_IRQ(STATES[spi_num].tx_dma_ch), MXC_SPI_RevB_DMA_TX_Handler);
        // NVIC_EnableIRQ(MXC_DMA_GET_IRQ(STATES[spi_num].tx_dma_ch));

        // MXC_NVIC_SetVector(MXC_DMA_GET_IRQ(STATES[spi_num].rx_dma_ch), MXC_SPI_RevB_DMA_RX_Handler);
        // NVIC_EnableIRQ(MXC_DMA_GET_IRQ(STATES[spi_num].rx_dma_ch));
    }

    // If successful, mark STATE of this SPI instance as initialized.
    STATES[spi_num].initialized = true;

    return E_NO_ERROR;
}

int MXC_SPI_RevB_Shutdown(mxc_spi_reva_regs_t *spi)
{
    int spi_num, error;

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
        MXC_DMA_ReleaseChannel(STATES[spi_num].tx_dma_ch);
        STATES[spi_num].rx_dma_ch = E_NO_DEVICE;
    }

    // Reset the SPI instance's STATE when shutting down.
    error = MXC_SPI_RevB_resetStateStruct(spi_num);
    if (error != E_NO_ERROR) {
        return error;
    }

    return E_NO_ERROR;
}

int MXC_SPI_RevB_DMA_GetTXChannel(mxc_spi_reva_regs_t *spi)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    return (STATES[spi_num].tx_dma_ch); 
}

int MXC_SPI_RevB_DMA_GetRXChannel(mxc_spi_reva_regs_t *spi)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    return (STATES[spi_num].rx_dma_ch); 
}

int MXC_SPI_RevB_SetFrequency(mxc_spi_reva_regs_t *spi, uint32_t freq)
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

int MXC_SPI_RevB_GetFrequency(mxc_spi_reva_regs_t *spi)
{
    if (MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi) < 0) {
        return E_BAD_PARAM;
    }

    unsigned scale, lo_clk, hi_clk;

    scale = (spi->clkctrl & MXC_F_SPI_REVA_CLKCTRL_CLKDIV) >> MXC_F_SPI_REVA_CLKCTRL_CLKDIV_POS;
    hi_clk = (spi->clkctrl & MXC_F_SPI_REVA_CLKCTRL_HI) >> MXC_F_SPI_REVA_CLKCTRL_HI_POS;
    lo_clk = (spi->clkctrl & MXC_F_SPI_REVA_CLKCTRL_LO) >> MXC_F_SPI_REVA_CLKCTRL_LO_POS;

    return (PeripheralClock / (1 << scale)) / (lo_clk + hi_clk);
}

int MXC_SPI_RevB_SetDataSize(mxc_spi_reva_regs_t *spi, int data_size)
{
    int spi_num;
    int saved_enable_state;

    // HW has problem with these two character sizes
    if (data_size == 1 || data_size > 16) {
        return E_BAD_PARAM;
    }

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // Set up the character size.
    if (!(spi->stat & MXC_F_SPI_REVA_STAT_BUSY)) {
        saved_enable_state = spi->ctrl0 | MXC_F_SPI_REVA_CTRL0_EN;

        // If enabled, disable SPI before changing character size.
        if (saved_enable_state) {
            spi->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL0_EN);
        }

        // Update data size from save Init function.
        STATES[spi_num].init.data_size = data_size;

        if (data_size < 16) {
            MXC_SETFIELD(spi->ctrl2, MXC_F_SPI_REVA_CTRL2_NUMBITS,
                         data_size << MXC_F_SPI_REVA_CTRL2_NUMBITS_POS);
        } else {
            // Set to 16 bits per character as default.
            MXC_SETFIELD(spi->ctrl2, MXC_F_SPI_REVA_CTRL2_NUMBITS,
                         0 << MXC_F_SPI_REVA_CTRL2_NUMBITS_POS);
        }

        // Return back to original SPI enable state.
        MXC_SETFIELD(spi->ctrl0, MXC_F_SPI_REVA_CTRL0_EN, saved_enable_state);
    } else {
        return E_BAD_STATE;
    }

    return E_NO_ERROR;
}

int MXC_SPI_RevB_GetDataSize(mxc_spi_reva_regs_t *spi)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // NUMBITS = 0 means 16-bits per character
    if (!(spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_NUMBITS)) {
        return 16;
    } else {
        return ((spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_NUMBITS) >> MXC_F_SPI_REVA_CTRL2_NUMBITS_POS);
    }
}

int MXC_SPI_RevB_SetWidth(mxc_spi_reva_regs_t *spi, mxc_spi_datawidth_t width)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // Clear before setting
    spi->ctrl2 &= ~(MXC_F_SPI_REVA_CTRL2_THREE_WIRE | MXC_F_SPI_REVA_CTRL2_DATA_WIDTH);

    switch (width) {
        case MXC_SPI_WIDTH_3WIRE:
            spi->ctrl2 |= MXC_F_SPI_REVA_CTRL2_THREE_WIRE;
            break;

        case MXC_SPI_WIDTH_STANDARD:
            spi->ctrl2 |= MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_MONO;
            break;

        case MXC_SPI_WIDTH_DUAL:
            spi->ctrl2 |= MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_DUAL;
            break;

        case MXC_SPI_WIDTH_QUAD:
            spi->ctrl2 |= MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_QUAD;
            break;

        // Default set to to 3-Wire
        default:
            spi->ctrl2 |= MXC_F_SPI_REVA_CTRL2_THREE_WIRE;
            break;
    }

    // Save state of new width.
    STATES[spi_num].init.width = width;

    return E_NO_ERROR;
}

mxc_spi_datawidth_t MXC_SPI_RevB_GetWidth(mxc_spi_reva_regs_t *spi)
{
    if (spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_THREE_WIRE) {
        return MXC_SPI_WIDTH_3WIRE;
    }

    if (spi->ctrl2 & MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_DUAL) {
        return MXC_SPI_WIDTH_DUAL;
    }

    if (spi->ctrl2 & MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_QUAD) {
        return MXC_SPI_WIDTH_QUAD;
    }

    return MXC_SPI_WIDTH_STANDARD;
}

int MXC_SPI_RevB_SetClkMode(mxc_spi_reva_regs_t *spi, mxc_spi_clkmode_t clk_mode)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

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

    // Save state of new clock mode.
    STATES[spi_num].init.clk_mode = clk_mode;

    return E_NO_ERROR;
}

mxc_spi_clkmode_t MXC_SPI_RevB_GetClkMode(mxc_spi_reva_regs_t *spi)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

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

int MXC_SPI_RevB_DMA_SetRequestSelect(mxc_spi_reva_regs_t *spi, uint32_t spi_tx_reqsel, uint32_t spi_rx_reqsel)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // This function will overwrite the current DMA TX/RX Request Selects.
    STATES[spi_num].tx_dma_reqsel = -1;
    STATES[spi_num].rx_dma_reqsel = -1;

    if (spi_tx_reqsel != -1) {
        STATES[spi_num].tx_dma_reqsel = spi_tx_reqsel;
    }

    if (spi_rx_reqsel != -1) {
        STATES[spi_num].rx_dma_reqsel = spi_rx_reqsel;
    }

    return E_NO_ERROR;
}

int MXC_SPI_RevB_SetRegisterCallback(mxc_spi_reva_regs_t *spi, mxc_spi_callback_t callback, void *data)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    if (STATES[spi_num].initialized == false) {
        return E_BAD_STATE;
    }

    STATES[spi_num].init.callback = callback;
    STATES[spi_num].init.callback_data = data;

    return E_NO_ERROR;
}

int MXC_SPI_RevB_GetActive(mxc_spi_reva_regs_t *spi)
{
    if (spi->stat & MXC_F_SPI_REVA_STAT_BUSY) {
        return E_BUSY;
    }

    return E_NO_ERROR;
}

/* ** Transaction Functions ** */
// TODO: Request Object
int MXC_SPI_RevB_MasterTransaction(mxc_spi_reva_regs_t *spi, uint16_t *tx_buffer, uint32_t tx_len, uint16_t *rx_buffer, uint32_t rx_len, uint32_t deassert, mxc_spi_target_t *cs_cfg)
{
    int spi_num, tx_dummy_len;

    // Ensure valid SPI Instance.
    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // Ensure valid chip select option.  
    if (cs_cfg == NULL) {
        return E_NULL_PTR;
    }

    // Make sure SPI Instance was initialized.
    if (STATES[spi_num].initialized == false) {
        return E_BAD_STATE;
    }

// TODO: Check if initializing DMA affects non-DMA transactions
    // Make sure DMA is not initialized.
    if (STATES[spi_num].init.use_dma == true) {
        return E_BAD_STATE;
    }

    // Initialize SPIn state to handle data.
    STATES[spi_num].controller_done = false;

    STATES[spi_num].tx_buffer = tx_buffer;
    STATES[spi_num].tx_len = tx_len;
    STATES[spi_num].tx_done = false;

    STATES[spi_num].rx_buffer = rx_buffer;
    STATES[spi_num].tx_len = tx_len;
    STATES[spi_num].rx_done = false;

    STATES[spi_num].deassert = deassert;
    STATES[spi_num].current_cs_cfg = *cs_cfg;

    // Set the number of bytes to transmit/receive for the SPI transaction.
    if (STATES[spi_num].init.width == MXC_SPI_WIDTH_STANDARD) {
        if (rx_len > tx_len) {
            // In standard 4-wire mode, the RX_NUM_CHAR field of ctrl1 is ignored.
            // The number of bytes to transmit AND receive is set by TX_NUM_CHAR,
            // because the hardware always assume full duplex.  Therefore extra
            // dummy bytes must be transmitted to support half duplex. 
            tx_dummy_len = rx_len - tx_len;
            spi->ctrl1 = ((tx_len + tx_dummy_len) << MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS);
        } else {
            spi->ctrl1 = tx_len << MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS;
        }
    } else { // width != MXC_SPI_WIDTH_STANDARD
        spi->ctrl1 = (tx_len << MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS) |
                     (rx_len << MXC_F_SPI_REVA_CTRL1_RX_NUM_CHAR_POS);
    }

    // Disable FIFOs before clearing as recommended by UG.
    spi->dma &= ~(MXC_F_SPI_REVA_DMA_TX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_TX_EN | MXC_F_SPI_REVA_DMA_RX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_RX_EN);
    spi->dma |= (MXC_F_SPI_REVA_DMA_TX_FLUSH | MXC_F_SPI_REVA_DMA_RX_FLUSH);

    // Enable Interrupts
    spi->inten |= MXC_F_SPI_REVA_INTEN_MST_DONE;

    if (tx_len > 0) {
        // Enable TX FIFO & TX Threshold crossed interrupt 
        spi->dma |= (MXC_F_SPI_REVA_DMA_TX_FIFO_EN);
        spi->inten |= MXC_F_SPI_REVA_INTEN_TX_THD;
    }

    if (rx_len > 0) {
        // Enable RX FIFO & RX Threshold crossed interrupt
        spi->dma |= (MXC_F_SPI_REVA_DMA_RX_FIFO_EN);
        spi->inten |= MXC_F_SPI_REVA_INTEN_RX_THD;
    }

    // This private function, MXC_SPI_RevB_process, call fills the TX FIFO as much as possible
    //   before launching the transaction. Subsequent FIFO management will
    //   be handled from the MXC_SPI_Handler which should be called in SPI_IRQHandler. 
    MXC_SPI_RevB_process(spi);

    // Toggle Chip Select Pin if handled by the driver
    if (STATES[spi_num].init.cs_control == MXC_SPI_CSCONTROL_SW_DRV) {
        // Make sure the selected Target Select (L. SS) pin is enabled as an output.
        if (cs_cfg->pins.func != MXC_GPIO_FUNC_OUT) {
            return E_BAD_STATE;
        }

        // Active HIGH (1).
        if (cs_cfg->active_polarity == 1) {
            cs_cfg->pins.port->out_set |= cs_cfg->pins.mask;
        // Active LOW (0).
        } else {
            cs_cfg->pins.port->out_clr |= cs_cfg->pins.mask;
        }
    }

    // Start the SPI transaction.
    spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_START;

    // Handle target-select (L. SS) deassertion if HW is selected as CS Control Scheme. This must be done 
    //   AFTER launching the transaction to avoid a glitch on the SS line if:
    //     - The SS line is asserted
    //     - We want to deassert the line as part of this transaction
    //
    // As soon as the SPI hardware receives CTRL0->START it seems to reinitialize the CS pin based
    //   on the value of CTRL->SS_CTRL, which causes the glitch.
    if (STATES[spi_num].init.cs_control == MXC_SPI_CSCONTROL_HW_AUTO) {
        // In HW Auto Scheme, only use the cs_cfg index member.
        MXC_SETFIELD(spi->ctrl0, MXC_F_SPI_REVA_CTRL0_SS_ACTIVE, cs_cfg->index << MXC_F_SPI_REVA_CTRL0_SS_ACTIVE_POS);

        if (deassert) { 
            spi->ctrl0 &= ~MXC_F_SPI_REVA_CTRL0_SS_CTRL;
        } else {
            spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_SS_CTRL;
        }
    }

    return E_SUCCESS;
}

// TODO: Match Polling, Async, and DMA function names of chip-specific level
int MXC_SPI_RevB_MasterTransactionB(mxc_spi_reva_regs_t *spi, uint16_t *tx_buffer, uint32_t tx_len, uint16_t *rx_buffer, uint32_t rx_len, uint32_t deassert, mxc_spi_target_t *cs_cfg)
{
    int error;
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // This function fills in the STATES value for the flags that checks for blocking status.
    error = MXC_SPI_RevB_MasterTransaction(spi, tx_buffer, tx_len, rx_buffer, rx_len, deassert, cs_cfg);
    if (error != E_NO_ERROR) {
        return error;
    }

    // Blocking
    while(!((STATES[spi_num].rx_done && STATES[spi_num].controller_done) && (STATES[spi_num].tx_buffer != NULL && STATES[spi_num].tx_len > 0)) && !(STATES[spi_num].rx_done && (STATES[spi_num].rx_buffer != NULL && STATES[spi_num].rx_len > 0))) {}

    return E_SUCCESS;
}


int MXC_SPI_RevB_MasterTransactionDMA(mxc_spi_reva_regs_t *spi, uint16_t *tx_buffer, uint32_t tx_len, uint16_t *rx_buffer, uint32_t rx_len, bool deassert, mxc_spi_target_t *cs_cfg)
{
    int spi_num, tx_dummy_len;
    // For readability purposes.
    int rx_ch, tx_ch;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // Make sure DMA is initialized.
    if (STATES[spi_num].init.use_dma == false) {
        return E_BAD_STATE;
    }

    // Make sure SPI Instance was initialized.
    if (STATES[spi_num].initialized == false) {
        return E_BAD_STATE;
    }

    // Initialize SPIn state to handle data.  
    STATES[spi_num].controller_done = false;

    STATES[spi_num].tx_buffer = tx_buffer;
    STATES[spi_num].tx_len = tx_len;
    STATES[spi_num].tx_done = false;

    STATES[spi_num].rx_buffer = rx_buffer;
    STATES[spi_num].tx_len = tx_len;
    STATES[spi_num].rx_done = false;

    STATES[spi_num].deassert = deassert;
    STATES[spi_num].current_cs_cfg = *cs_cfg;

    // Set the number of bytes to transmit/receive for the SPI transaction
    if (STATES[spi_num].init.width == MXC_SPI_WIDTH_STANDARD) {
        if (rx_len > tx_len) {
            // In standard 4-wire mode, the RX_NUM_CHAR field of ctrl1 is ignored.
            //  The number of bytes to transmit AND receive is set by TX_NUM_CHAR,
            //  because the hardware always assume full duplex. Therefore extra
            //  dummy bytes must be transmitted to support half duplex. 
            tx_dummy_len = rx_len - tx_len;
            spi->ctrl1 = ((tx_len + tx_dummy_len) << MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS);
        } else {
            spi->ctrl1 = tx_len << MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS;
        }
    } else { // width != MXC_SPI_WIDTH_STANDARD
        spi->ctrl1 = (tx_len << MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS) |
                     (rx_len << MXC_F_SPI_REVA_CTRL1_RX_NUM_CHAR_POS);
    }

    // Disable FIFOs before clearing as recommended by UG
    spi->dma &= ~(MXC_F_SPI_REVA_DMA_TX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_TX_EN | MXC_F_SPI_REVA_DMA_RX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_RX_EN);
    spi->dma |= (MXC_F_SPI_REVA_DMA_TX_FLUSH | MXC_F_SPI_REVA_DMA_RX_FLUSH);

    // Set up DMA TX Transactions.
    // 1) For TX transmissions.
    if (tx_len > 1) {
        // For readability purposes.
        tx_ch = STATES[spi_num].tx_dma_ch;

        // Configure TX DMA channel to fill the SPI TX FIFO
        spi->dma |= (MXC_F_SPI_REVA_DMA_TX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_TX_EN | (31 << MXC_F_SPI_REVA_DMA_TX_THD_VAL_POS));
        
        // Hardware requires writing the first byte into the FIFO manually.
        spi->fifo32 = tx_buffer[0];
        STATES[spi_num].dma->ch[tx_ch].src = (uint32_t)(tx_buffer + 1);
        STATES[spi_num].dma->ch[tx_ch].cnt = tx_len - 1;
        STATES[spi_num].dma->ch[tx_ch].ctrl |= MXC_F_DMA_REVA_CTRL_SRCINC;
        STATES[spi_num].dma->ch[tx_ch].ctrl |= MXC_F_DMA_REVA_CTRL_EN;  // Start the DMA

    // 2) For single character transmissions.
    } else if (tx_len == 1) {
        // Single-length transactions does not trigger CTZ
        spi->dma |= (MXC_F_SPI_REVA_DMA_TX_FIFO_EN | MXC_F_SPI_DMA_DMA_TX_EN);
        spi->fifo32 = *tx_buffer; // Write first byte into FIFO
        STATES[spi_num].tx_done = true;

    // 3) Set up DMA TX for RX only transactions.
    // Note: Even if you are not transmitting anything in standard 4-wire mode, 
    //  the hardware always assume full duplex. Therefore dummy bytes 
    //  must be transmitted to support half duplex. The number of bytes to transmit 
    //  AND receive is set by TX_NUM_CHAR, and the RX_NUM_CHAR field of ctrl1 is ignored.
    } else if (tx_len == 0 && STATES[spi_num].init.width == MXC_SPI_WIDTH_STANDARD) {
        // For readability purposes.
        tx_ch = STATES[spi_num].tx_dma_ch;

        // Configure TX DMA channel to retransmit the dummy byte
        spi->dma |= (MXC_F_SPI_REVA_DMA_TX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_TX_EN);
        STATES[spi_num].dma->ch[tx_ch].src = (uint32_t)&(STATES[spi_num].init.tx_dummy_value);
        STATES[spi_num].dma->ch[tx_ch].cnt = rx_len;
        STATES[spi_num].dma->ch[tx_ch].ctrl &= ~MXC_F_DMA_REVA_CTRL_SRCINC;
        STATES[spi_num].dma->ch[tx_ch].ctrl |= MXC_F_DMA_REVA_CTRL_EN;  // Start the DMA
    }

    // Set up DMA RX Transactions.
    if (rx_len > 0) {
        // For readability purposes.
        rx_ch = STATES[spi_num].rx_dma_ch;

        // Configure RX DMA channel to unload the SPI RX FIFO
        spi->dma |= (MXC_F_SPI_REVA_DMA_RX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_RX_EN);
        STATES[spi_num].dma->ch[rx_ch].dst = (uint32_t)rx_buffer;
        STATES[spi_num].dma->ch[rx_ch].cnt = rx_len;
        STATES[spi_num].dma->ch[rx_ch].ctrl |= MXC_F_DMA_REVA_CTRL_EN;  // Start the DMA
    }

    // Toggle Chip Select Pin if handled by the driver
    if (STATES[spi_num].init.cs_control == MXC_SPI_CSCONTROL_SW_DRV) {
        // Make sure the selected Target Select (L. SS) pin is enabled as an output.
        if (cs_cfg->pins.func != MXC_GPIO_FUNC_OUT) {
            return E_BAD_STATE;
        }

        // Active HIGH (1).
        if (cs_cfg->active_polarity == 1) {
            cs_cfg->pins.port->out_set |= cs_cfg->pins.mask;
        // Active LOW (0).
        } else {
            cs_cfg->pins.port->out_clr |= cs_cfg->pins.mask;
        }
    }

    // Start the SPI transaction
    spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_START;

    // Handle slave-select (SS) deassertion.  This must be done AFTER launching the transaction
    // to avoid a glitch on the SS line if:
    // - The SS line is asserted
    // - We want to deassert the line as part of this transaction
    //
    // As soon as the SPI hardware receives CTRL0->START it seems to reinitialize the SS pin based
    // on the value of CTRL->SS_CTRL, which causes the glitch.
    if (STATES[spi_num].init.cs_control == MXC_SPI_CSCONTROL_HW_AUTO) {
        // In HW Auto Scheme, only use the cs_cfg index member.
        MXC_SETFIELD(spi->ctrl0, MXC_F_SPI_REVA_CTRL0_SS_ACTIVE, cs_cfg->index << MXC_F_SPI_REVA_CTRL0_SS_ACTIVE_POS);

        if (deassert) { 
            spi->ctrl0 &= ~MXC_F_SPI_REVA_CTRL0_SS_CTRL;
        } else {
            spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_SS_CTRL;
        }
    }

    return E_SUCCESS;
}

int MXC_SPI_RevB_MasterTransactionDMAB(mxc_spi_reva_regs_t *spi, uint16_t *tx_buffer, uint32_t tx_len, uint16_t *rx_buffer, uint32_t rx_len, bool deassert, mxc_spi_target_t *cs_cfg)
{
    int error;
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // This function fills in the STATES value for the flags that checks for blocking status.
    error = MXC_SPI_RevB_MasterTransactionDMA(spi, tx_buffer, tx_len, rx_buffer, rx_len, deassert, cs_cfg);
    if (error != E_NO_ERROR) {
        return error;
    }

    // Blocking
    while(!((STATES[spi_num].rx_done && STATES[spi_num].controller_done) && (STATES[spi_num].tx_buffer != NULL && STATES[spi_num].tx_len > 0)) && !(STATES[spi_num].rx_done && (STATES[spi_num].rx_buffer != NULL && STATES[spi_num].rx_len > 0))) {}

    return E_SUCCESS;
}

/* ** Handler Functions ** */

void MXC_SPI_RevB_Handler(mxc_spi_reva_regs_t *spi)
{
    int spi_num;
    uint32_t status = spi->intfl;

    // Used later for readability purposes on handling Chip Select.
    mxc_gpio_regs_t *cs_port;
    uint32_t cs_mask;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);

    // Master done (TX complete)
    if (status & MXC_F_SPI_REVA_INTFL_MST_DONE) { 
        STATES[spi_num].controller_done = true;
        spi->intfl |= MXC_F_SPI_REVA_INTFL_MST_DONE; // Clear flag

        // Toggle CS Pin if Driver is handling it.
        if (STATES[spi_num].init.cs_control == MXC_SPI_CSCONTROL_SW_DRV) {
            if (STATES[spi_num].deassert == true) {
                // Readability for handling Chip Select.
                cs_port = STATES[spi_num].current_cs_cfg.pins.port;
                cs_mask = STATES[spi_num].current_cs_cfg.pins.mask;
            
                cs_port->out ^= cs_mask;
            } // Don't deassert the CS pin if false for multiple repeated transactions.
        }

        // Callback if valid.
        // Note: If CS Control Scheme is set in SW_App mode, then the caller needs to ensure the
        //   CS pin is asserted or deasserted in their application.
        if (STATES[spi_num].init.callback) {
            STATES[spi_num].init.callback(STATES[spi_num].init.callback_data);
        }
    }

    // Handle RX Threshold
    if (status & MXC_F_SPI_REVA_INTFL_RX_THD) {
        spi->intfl |= MXC_F_SPI_REVA_INTFL_RX_THD;
        if (!(STATES[spi_num].init.use_dma)) {
            // RX threshold has been crossed, there's data to unload from the FIFO
            MXC_SPI_RevB_process(spi);
        }
    }

    // Handle TX Threshold
    if (status & MXC_F_SPI_REVA_INTFL_TX_THD) {
        spi->intfl |= MXC_F_SPI_REVA_INTFL_TX_THD;
        if (!(STATES[spi_num].init.use_dma)) {
            // TX threshold has been crossed, we need to refill the FIFO
            MXC_SPI_RevB_process(spi);
        }
    }
}

void MXC_SPI_RevB_DMA_TX_Handler(mxc_spi_reva_regs_t *spi)
{
    int spi_num;
    uint32_t tx_ch;
    uint32_t status;

    // Used later for readability purposes on handling Chip Select.
    mxc_gpio_regs_t *cs_port;
    uint32_t cs_mask;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);

    tx_ch = STATES[spi_num].tx_dma_ch;
    status = STATES[spi_num].dma->ch[tx_ch].status;

    // Count-to-Zero (DMA TX complete)
    if (status & MXC_F_DMA_REVA_STATUS_CTZ_IF) { 
        STATES[spi_num].dma->ch[tx_ch].status |= MXC_F_DMA_REVA_STATUS_CTZ_IF;

        // Toggle CS Pin if Driver is handling it.
        if (STATES[spi_num].init.cs_control == MXC_SPI_CSCONTROL_SW_DRV) {
            if (STATES[spi_num].deassert == true) {
                // Readability for handling Chip Select.
                cs_port = STATES[spi_num].current_cs_cfg.pins.port;
                cs_mask = STATES[spi_num].current_cs_cfg.pins.mask;
            
                cs_port->out ^= cs_mask;
            } // Don't deassert the CS pin if false for multiple repeated transactions.
        }

        // Callback if valid.
        // Note: If CS Control Scheme is set in SW_App mode, then the caller needs to ensure the
        //   CS pin is asserted or deasserted in their application.
        if (STATES[spi_num].init.callback) {
            STATES[spi_num].init.callback(STATES[spi_num].init.callback_data);
        }
    }

    // Bus Error
    if (status & MXC_F_DMA_REVA_STATUS_BUS_ERR) {
        STATES[spi_num].dma->ch[tx_ch].status |= MXC_F_DMA_REVA_STATUS_BUS_ERR;
    }
}

void MXC_SPI_RevB_DMA_RX_Handler(mxc_spi_reva_regs_t *spi)
{
    int spi_num;
    uint32_t rx_ch;
    uint32_t status;
    // Used later for readability purposes on handling Chip Select.
    mxc_gpio_regs_t *cs_port;
    uint32_t cs_mask;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);

    rx_ch = STATES[spi_num].rx_dma_ch;
    status = STATES[spi_num].dma->ch[rx_ch].status;

    // Count-to-Zero (DMA RX complete)
    if (status & MXC_F_DMA_REVA_STATUS_CTZ_IF) { 
        STATES[spi_num].rx_done = 1;
        STATES[spi_num].dma->ch[rx_ch].status |= MXC_F_DMA_STATUS_CTZ_IF;

        // Toggle CS Pin if Driver is handling it.
        if (STATES[spi_num].init.cs_control == MXC_SPI_CSCONTROL_SW_DRV) {
            if (STATES[spi_num].deassert == true) {
                // Readability for handling Chip Select.
                cs_port = STATES[spi_num].current_cs_cfg.pins.port;
                cs_mask = STATES[spi_num].current_cs_cfg.pins.mask;
            
                cs_port->out ^= cs_mask;
            } // Don't deassert the CS pin if false for multiple repeated transactions.
        }

        // Callback if valid.
        // Note: If CS Control Scheme is set in SW_App mode, then the caller needs to ensure the
        //   CS pin is asserted or deasserted in their application.
        if (STATES[spi_num].init.callback) {
            STATES[spi_num].init.callback(STATES[spi_num].init.callback_data);
        }
    }

    // Bus Error
    if (status & MXC_F_DMA_REVA_STATUS_BUS_ERR) {
        STATES[spi_num].dma->ch[rx_ch].status |= MXC_F_DMA_STATUS_BUS_ERR;
    }
}
