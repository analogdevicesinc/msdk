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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_SPI_SPI_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_SPI_SPI_REVA_H_

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "spi_regs.h"
#include "spi_reva_regs.h"
#include "spi.h"
#include "dma.h"
#include "dma_reva_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    mxc_spi_regs_t       *spi;            // Selected SPI Instance
    mxc_gpio_cfg_t       *spi_pins;       // Main SPI pins (i.e. MOSI, MISO, CLK)
    mxc_gpio_cfg_t       cs_pins;        // Pins for chip select.
    mxc_spi_state_t      type;            // Controller (L. Master) vs Target (L. Slave)
    mxc_spi_clkmode_t    clk_mode;
    uint32_t             freq;            // Clock Frequency
    uint8_t              data_size;       // Number of bits per character sent
    mxc_spi_width_t      width;           // 3-wire, standard, dual, and quad modes
    uint16_t             tx_dummy_value;  // Value of dummy bytes to be sent

    // Select Chip Select Control
    mxc_spi_cscontrol_t  cs_control;      // CS Control Scheme (auto HW, driver, or app controlled)
    uint32_t             cs_index;        // Index of Slave Select for Auto HW mode.

    // DMA
    bool                 use_dma;
    mxc_dma_regs_t       *dma;

    // Callback
    mxc_spi_callback_t   callback;
    void                 *callback_data;
} mxc_spi_init_t;

/**
 * @brief Structure used to initialize SPI pins.
 *
 * @note All values must be initialized.
 *
 * @note True equals pin is set for the spi function false the pin is left to its latest state.
 */
typedef struct {
    bool clock; ///<Clock pin
    bool ss0; ///< Slave select pin 0
    bool ss1; ///< Slave select pin 1
    bool ss2; ///< Slave select pin 2
    bool miso; ///< miso pin
    bool mosi; ///< mosi pin
    bool sdio2; ///< SDIO2 pin
    bool sdio3; ///< SDIO3 pin
    bool vddioh; ///< VDDIOH Select
} mxc_spi_pins_t;

// Type or MSMode
typedef enum {
    MXC_SPI_TYPE_MASTER = 0,
    MXC_SPI_TYPE_CONTROLLER = 0,
    MXC_SPI_TYPE_SLAVE = 1,
    MXC_SPI_TYPE_TARGET = 1
} mxc_spi_type_t;

typedef enum {
    MXC_SPI_CLKMODE_0 = 0,  // CPOL: 0    CPHA: 0
    MXC_SPI_CLKMODE_1 = 1,  // CPOL: 0    CPHA: 1
    MXC_SPI_CLKMODE_2 = 2,  // CPOL: 1    CPHA: 0
    MXC_SPI_CLKMODE_3 = 3   // CPOL: 1    CPHA: 1
} mxc_spi_clkmode_t;

// TODO: Check if DATAWIDTH is the best name for this
typedef enum {
    MXC_SPI_WIDTH_3WIRE = 0,
    MXC_SPI_WIDTH_STANDARD = 0,
    MXC_SPI_WIDTH_DUAL = 1,
    MXC_SPI_WIDTH_QUAD = 2
} mxc_spi_width_t;

// SS Control Scheme
typedef enum {
    MXC_SPI_CSCONTROL_HW_AUTO = 0,  // Automatically by hardware
    MXC_SPI_CSCONTROL_SW_DRV = 1,   // Through software by the driver
    MXC_SPI_CSCONTROL_SW_APP = 2    // Through software in the application
} mxc_spi_cscontrol_t;

// CS Control Scheme
typedef enum {
    MXC_SPI_STATE_READY = 0, // Ready for transaction
    MXC_SPI_STATE_BUSY = 1   // Busy transferring
} mxc_spi_state_t;

/**
 * @brief   The callback routine used to indicate the transaction has terminated.
 *
 * @param   req         The details of the transaction.
 * @param   result      See \ref MXC_Error_Codes for the list of error codes.
 */
typedef void (*mxc_spi_callback_t)(void*);


typedef struct {
    // From Init Data
    mxc_spi_init_t      init;
    mxc_dma_reva_regs_t *dma;
    mxc_gpio_cfg_t      cs_pins

    // Transaction States.
    uint16_t            *tx_buffer;
    uint32_t            tx_len;
    bool                tx_done;
    uint16_t            *rx_buffer;
    uint32_t            rx_len;
    bool                rx_done;

    // DMA
    int                 tx_dma_ch;
    int                 rx_dma_ch;
    uint32_t            dma_rx_reqsel;
    uint32_t            dma_tx_reqsel;

    // Target Control Scheme
    bool                controller_done; // Master
    bool                target_done;     // Slave
} mxc_spi_handle_data_t;

static volatile mxc_spi_handle_data_t STATES[MXC_SPI_INSTANCES];


void MXC_SPI_RevA_DMA_TX_Handler(mxc_spi_reva_regs_t *spi)
{
    int spi_num;
    uint32_t tx_channel;
    uint32_t status;

    spi_num = MXC_SPI_GET_IDX(spi);
    MXC_ASSERT(spi_num >= 0);

    tx_channel = STATES[spi_num].tx_dma_ch;
    status = STATES[spi_num].dma->ch[tx_channel]->status;

    // Count-to-Zero (DMA TX complete)
    if (status & MXC_F_DMA_REVA_STATUS_CTZ_IF) { 
        STATES[spi_num].dma->ch[tx_channel]->status |= MXC_F_DMA_REVA_STATUS_CTZ_IF;
    }

    // Bus Error
    if (status & MXC_F_DMA_REVA_STATUS_BUS_ERR) {
        STATES[spi_num].dma->ch[tx_channel]->status |= MXC_F_DMA_REVA_STATUS_BUS_ERR;
    }
}

void MXC_SPI_Reva_DMA_RX_Handler(mxc_spi_reva_regs_t *spi)
{
    int spi_num;
    uint32_t rx_channel;
    uint32_t status;

    spi_num = MXC_SPI_GET_IDX(spi);
    MXC_ASSERT(spi_num >= 0);

    rx_channel = STATES[spi_num].rx_dma_ch;
    status = STATES[spi_num].dma->ch[rx_channel]->status;

    // Count-to-Zero (DMA RX complete)
    if (status & MXC_F_DMA_REVA_STATUS_CTZ_IF) { 
        STATES[spi_num].rx_done = 1;
        STATES[spi_num].dma->ch[rx_channel]->status |= MXC_F_DMA_STATUS_CTZ_IF;
    }

    // Bus Error
    if (status & MXC_F_DMA_REVA_STATUS_BUS_ERR) {
        STATES[spi_num].dma->ch[rx_channel]->status |= MXC_F_DMA_STATUS_BUS_ERR;
    }
}

void MXC_SPI_RevA_Handler(mxc_spi_reva_regs_t *spi)
{
    int spi_num;
    uint32_t status = spi->intfl;

    spi_num = MXC_SPI_GET_IDX(spi);
    MXC_ASSERT(spi_num >= 0);

    // Master done (TX complete)
    if (status & MXC_F_SPI_REVA_INTFL_MST_DONE) { 
        STATES[spi_num].controller_done = true;
        SPI->intfl |= MXC_F_SPI_REVA_INTFL_MST_DONE; // Clear flag

        // Toggle CS Pin if HW or Driver is handling it.
        if (STATES[spi_num].init.cscontrol == MXC_SPI_CSCONTROL_SW_DRV) {
            STATES[spi_num].init.cs_pins.port->out ^= STATES[spi_num].init.cs_pins.mask;
        }
    }

    // Handle RX Threshold
    if (status & MXC_F_SPI_REVA_INTFL_RX_THD) {
        spi->intfl |= MXC_F_SPI_REVA_INTFL_RX_THD;
        if (!(STATES[spi_num].use_dma)) {
            // RX threshold has been crossed, there's data to unload from the FIFO
            MXC_SPI_RevB_Process(spi);
        }
    }

    // Handle TX Threshold
    if (status & MXC_F_SPI_REVA_INTFL_TX_THD) {
        spi->intfl |= MXC_F_SPI_REVA_INTFL_TX_THD;
        if (!(STATES[spi_num].use_dma)) {
            // TX threshold has been crossed, we need to refill the FIFO
            MXC_SPI_RevB_process(spi);
        }
    }

    // Callback if valid
    if (STATES[spi_num].callback) {
        STATES[spi_num].callback(STATES[spi_num].callback_data);
    }
}

// Private function in drivers will not have starting capital in name
static void MXC_SPI_RevB_process(mxc_spi_reva_regs_t *spi)
{
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


int MXC_SPI_RevA_Init(mxc_spi_init_t *init)
{
    int error, spi_num;

    spi_num = MXC_SPI_GET_IDX(init->spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    if (init == NULL) {
        return E_NULL_PTR;
    }

    // Save init data for transactions and handlers.
    STATES[spi_num].init = *init;
    STATES[spi_num].dma = NULL;
    STATES[spi_num].cs_pins = init->cs_pins;

    // Set up CS Control Scheme.
    if (init->cs_control == MXC_SPI_CSCONTROL_HW_AUTO) {
        // If hardware is handling CS pin, make sure the correct alternate function is chosen.
        if (&(init->cs_pins) != NULL && init->cs_pins.func != MXC_GPIO_FUNC_OUT) {
            error = MXC_GPIO_Config(&(init->cs_pins));
            if (error != E_NO_ERROR) {
                return error;
            }

            (init->spi)->ctrl0 |= (init->cs_index << MXC_F_SPI_REVA_CTRL0_SS_ACTIVE_POS);
        } else {
            E_BAD_PARAM;
        }
    } else if (init->cs_control == MXC_SPI_CSCONTROL_SW_DRV) {
        // If SPI driver is handling CS pin, make sure the pin function is in OUT mode.
        if (&(init->cs_pins) != NULL && init->cs_pins.func == MXC_GPIO_FUNC_OUT) {
            error = MXC_GPIO_Config(&(init->cs_pins));
            if (error != E_NO_ERROR) {
                return error;
            }
        } else {
            E_BAD_PARAM;
        }
    // Don't do anything if SW Application is handling CS pin.
    } else if (init->cscontrol != MXC_SPI_CSCONTROL_SW_APP) {
        // Not a valid CS Control option.
        return E_BAD_PARAM;
    }

    // Enable SPI port.
    (init->spi)->ctrl0 |= MXC_F_SPI_REVA_REVA_CTRL0_EN;

    // Select Controller (L. Master) or Target (L. Slave) Mode.
    if (init->type == MXC_SPI_TYPE_CONTROLLER) {
        (init->spi)->ctrl0 |= MXC_F_SPI_REVA_REVA_CTRL0_MST_MODE;
    } else {
        (init->spi)->ctrl0 &= ~MXC_F_SPI_REVA_REVA_CTRL0_MST_MODE;
    }

//TODO: Can probably replace this with the SetDataSize Function.
    // Set character size
    if (init->dataSize <= 1 || init->dataSize > 16) {
        return E_BAD_PARAM;
    } else {
        (init->spi)->ctrl2 |= (init->dataSize) << MXC_F_SPI_REVA_REVA_CTRL2_NUMBITS_POS;
    }

    // Remove any delay between SS and SCLK edges.
    (init->spi)->sstime = (1 << MXC_F_SPI_REVA_REVA_SSTIME_PRE_POS) | (1 << MXC_F_SPI_REVA_REVA_SSTIME_POST_POS) | (1 << MXC_F_SPI_REVA_REVA_SSTIME_INACT_POS);

    // Enable TX/RX FIFOs
    (init->spi)->dma |= MXC_F_SPI_REVA_REVA_DMA_TX_FIFO_EN | MXC_F_SPI_REVA_REVA_DMA_RX_FIFO_EN;
    
    // Set TX and RX Threshold to 31 and 0, respectively.
    MXC_SETFIELD((init->spi)->dma, MXC_F_SPI_REVA_REVA_DMA_TX_THD_VAL, (31 << MXC_F_SPI_REVA_REVA_DMA_TX_THD_VAL_POS));
    MXC_SETFIELD((init->spi)->dma, MXC_F_SPI_REVA_REVA_DMA_RX_THD_VAL, (0 << MXC_F_SPI_REVA_REVA_DMA_RX_THD_VAL_POS));

    error = MXC_SPI_SetFrequency((init->spi), (init->freq));
    if (error != E_NO_ERROR) {
        return error;
    }

    // Clear any interrupt flags that may already be set.
    (init->spi)->intfl = (init->spi)->intfl;

    // Setup DMA features if used.
    if (init->useDMA) {
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

// TODO: Should the caller deal with enabling SetVector or should it be handled inside the driver?
        MXC_NVIC_SetVector(MXC_DMA_GET_IRQ(STATES[spi_num].tx_dma_ch), SPI_DMA_TX_IRQHandler);
        NVIC_EnableIRQ(MXC_DMA_GET_IRQ(STATES[spi_num].tx_dma_ch));

        MXC_NVIC_SetVector(MXC_DMA_GET_IRQ(STATES[spi_num].rx_dma_ch), SPI_DMA_RX_IRQHandler);
        NVIC_EnableIRQ(MXC_DMA_GET_IRQ(STATES[spi_num].rx_dma_ch));
    }
}


int MXC_SPI_RevB_MasterTransaction(mxc_spi_reva_regs_t *spi, uint16_t *tx_buffer, uint32_t tx_len, uint16_t *rx_buffer, uint32_t rx_len, uint32_t deassert, uint32_t idx_mask)
{
    int spi_num, tx_dummy_len;

    spi_num = MXC_SPI_GET_IDX(spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
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

    // Set the number of bytes to transmit/receive for the SPI transaction.
    if (STATES[spi_num].width == SPI_WIDTH_STANDARD) {
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
    } else { // width != SPI_WIDTH_STANDARD
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

    // Toggle Target Select Pin if handled by the driver
    if (STATES[spi_num].init.cscontrol == MXC_SPI_CSCONTROL_SW_DRV) {
        // Make sure the selected Target Select (L. SS) pin is enabled as an output.
        if (STATES[spi_num].init.cs_pins.port->outen |= idx_maske) {
            return E_BAD_STATE;
        }

        STATES[spi_num].init.cs_pins.mask = idx_mask;
        STATES[spi_num].init.cs_pins.port->out_set ^= idx_mask;
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
    if (STATES[spi_num].init.cscontrol == MXC_SPI_CSCONTROL_HW_AUTO) {
        // Select Target Index
        MXC_SETFIELD(spi->ctrl0, MXC_F_SPI_REVA_CTRL0_SS_ACTIVE, idx_mask << MXC_F_SPI_REVA_CTRL0_SS_ACTIVE_POS);

        if (deassert) { 
            spi->ctrl0 &= ~MXC_F_SPI_REVA_CTRL0_SS_CTRL;
        } else {
            spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_SS_CTRL;
        }
    }

    return E_SUCCESS;
}

int MXC_SPI_RevB_MasterTransactionB(mxc_spi_reva_regs_t *spi, uint16_t *tx_buffer, uint32_t tx_len, uint16_t *rx_buffer, uint32_t rx_len)
{
    int error;

    // This function fills in the STATES value for the flags that checks for blocking status.
    error = MXC_SPI_RevB_MasterTransaction(spi, tx_buffer, tx_len, rx_buffer, rx_len);
    if (error != E_NO_ERROR) {
        return error;
    }

    // Blocking
    while(!((STATES[spi_num].rx_done && STATE[spi_num].controller_done) && (STATES[spi_num].tx_buffer != NULL && STATES[spi_num].tx_len > 0)) && !(STATES[spi_num].rx_done && (STATES[spi_num].rx_buffer != NULL && STATES[spi_num].rx_len > 0))) {}

    return E_SUCCESS;
}


int MXC_SPI_RevB_MasterTransactionDMA(mxc_spi_reva_regs_t *spi, uint16_t *tx_buffer, uint32_t tx_len, uint16_t *rx_buffer, uint32_t rx_len, bool deassert)
{
    int spi_num, tx_dummy_len;

    spi_num = MXC_SPI_GET_IDX(spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // Make sure DMA is initialized.
    if (STATES[spi_num].use_dma == false) {
        return E_BAD_STATE;
    }

    // Initialize SPIn state to handle data.
    STATES[spi_num].use_dma = true;    
    STATES[spi_num].controller_done = false;

    STATES[spi_num].tx_buffer = tx_buffer;
    STATES[spi_num].tx_len = tx_len;
    STATES[spi_num].tx_done = false;

    STATES[spi_num].rx_buffer = rx_buffer;
    STATES[spi_num].tx_len = tx_len;
    STATES[spi_num].rx_done = false;

    // Set the number of bytes to transmit/receive for the SPI transaction
    if (STATES[spi_num].width == SPI_WIDTH_STANDARD) {
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
    } else { // width != SPI_WIDTH_STANDARD
        spi->ctrl1 = (tx_len << MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS) |
                     (rx_len << MXC_F_SPI_REVA_CTRL1_RX_NUM_CHAR_POS);
    }

    // Disable FIFOs before clearing as recommended by UG
    spi->dma &= ~(MXC_F_SPI_REVA_DMA_TX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_TX_EN | MXC_F_SPI_REVA_DMA_RX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_RX_EN);
    spi->dma |= (MXC_F_SPI_REVA_DMA_TX_FLUSH | MXC_F_SPI_REVA_DMA_RX_FLUSH);

    // Handle TX
    if (tx_len > 1) {
        // Configure TX DMA channel to fill the SPI TX FIFO
        spi->dma |= (MXC_F_SPI_DMA_TX_FIFO_EN | MXC_F_SPI_DMA_DMA_TX_EN | (31 << MXC_F_SPI_DMA_TX_THD_VAL_POS));
        // Hardware requires writing the first byte into the FIFO manually.
        spi->fifo32 = tx_buffer[0];
        STATES[spi_num].dma->ch[STATES[spi_num].tx_dma_ch].src = (uint32_t)(tx_buffer + 1);
        STATES[spi_num].dma->ch[STATES[spi_num].tx_dma_ch].cnt = tx_len - 1;
        STATES[spi_num].dma->ch[STATES[spi_num].tx_dma_ch].ctrl |= MXC_F_DMA_REVA_CTRL_SRCINC;
        STATES[spi_num].dma->ch[STATES[spi_num].tx_dma_ch].ctrl |= MXC_F_DMA_REVA_CTRL_EN;  // Start the DMA
    } else if (txlen == 1) {
        // Single-length transactions does not trigger CTZ
        SPI->dma |= (MXC_F_SPI_DMA_TX_FIFO_EN | MXC_F_SPI_DMA_DMA_TX_EN);
        SPI->fifo32 = *tx_buffer; // Write first byte into FIFO
        STATES[spi_num].tx_done = true;
    } else if (txlen == 0 && STATES[spi_num].width == MXC_SPI_WIDTH_STANDARD) {
        // Configure TX DMA channel to retransmit a dummy byte
        spi->dma |= (MXC_F_SPI_REVA_DMA_TX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_TX_EN);
        STATES[spi_num].dma->ch[STATES[spi_num].tx_dma_ch].src = (uint32_t)&(STATES[spi_num].tx_dummy_value);
        STATES[spi_num].dma->ch[STATES[spi_num].tx_dma_ch].cnt = rx_len;
        STATES[spi_num].dma->ch[STATES[spi_num].tx_dma_ch].ctrl &= ~MXC_F_DMA_REVA_CTRL_SRCINC;
        STATES[spi_num].dma->ch[STATES[spi_num].tx_dma_ch].ctrl |= MXC_F_DMA_REVA_CTRL_EN;  // Start the DMA
    }

    // Handle RX
    if (rxlen > 0) {
        // Configure RX DMA channel to unload the SPI RX FIFO
        SPI->dma |= (MXC_F_SPI_DMA_RX_FIFO_EN | MXC_F_SPI_DMA_DMA_RX_EN);
        STATES[spi_num].dma->ch[STATES[spi_num].rx_dma_ch].dst = (uint32_t)rx_buffer;
        STATES[spi_num].dma->ch[STATES[spi_num].rx_dma_ch].cnt = rx_len;
        STATES[spi_num].dma->ch[STATES[spi_num].rx_dma_ch].ctrl |= MXC_F_DMA_REVA_CTRL_EN;  // Start the DMA
    }

    // Start the SPI transaction
    SPI->ctrl0 |= MXC_F_SPI_CTRL0_START;

    // Handle slave-select (SS) deassertion.  This must be done AFTER launching the transaction
    // to avoid a glitch on the SS line if:
    // - The SS line is asserted
    // - We want to deassert the line as part of this transaction

    // As soon as the SPI hardware receives CTRL0->START it seems to reinitialize the SS pin based
    // on the value of CTRL->SS_CTRL, which causes the glitch.
    if (deassert) {
        SPI->ctrl0 &= ~MXC_F_SPI_CTRL0_SS_CTRL;
    } else {
        SPI->ctrl0 |= MXC_F_SPI_CTRL0_SS_CTRL;
    }

    return E_SUCCESS;
}

int MXC_SPI_RevB_MasterTransactionDMAB(mxc_spi_reva_regs_t *spi, uint16_t *tx_buffer, uint32_t tx_len, uint16_t *rx_buffer, uint32_t rx_len, bool deassert)
{
    int error;

    // This function fills in the STATES value for the flags that checks for blocking status.
    error = MXC_SPI_RevB_MasterTransactionDMA(spi, tx_buffer, tx_len, rx_buffer, rx_len);
    if (error != E_NO_ERROR) {
        return error;
    }

    // Blocking
    while(!((STATES[spi_num].rx_done && STATE[spi_num].controller_done) && (STATES[spi_num].tx_buffer != NULL && STATES[spi_num].tx_len > 0)) && !(STATES[spi_num].rx_done && (STATES[spi_num].rx_buffer != NULL && STATES[spi_num].rx_len > 0))) {}

    return E_SUCCESS;
}

void MXC_


//************************** Old
typedef enum {
    SPI_REVA_WIDTH_3WIRE,
    SPI_REVA_WIDTH_STANDARD,
    SPI_REVA_WIDTH_DUAL,
    SPI_REVA_WIDTH_QUAD,
} mxc_spi_reva_width_t;

typedef enum {
    SPI_REVA_MODE_0,
    SPI_REVA_MODE_1,
    SPI_REVA_MODE_2,
    SPI_REVA_MODE_3,
} mxc_spi_reva_mode_t;

typedef struct _mxc_spi_reva_req_t mxc_spi_reva_req_t;

struct _mxc_spi_reva_req_t {
    mxc_spi_reva_regs_t *spi;
    int ssIdx;
    int ssDeassert;
    uint8_t *txData;
    uint8_t *rxData;
    uint32_t txLen;
    uint32_t rxLen;
    uint32_t txCnt;
    uint32_t rxCnt;
    spi_complete_cb_t completeCB;
};

int MXC_SPI_RevA_Init(mxc_spi_reva_regs_t *spi, int masterMode, int quadModeUsed, int numSlaves,
                      unsigned ssPolarity, unsigned int hz);
int MXC_SPI_RevA_Shutdown(mxc_spi_reva_regs_t *spi);
int MXC_SPI_RevA_ReadyForSleep(mxc_spi_reva_regs_t *spi);
int MXC_SPI_RevA_SetFrequency(mxc_spi_reva_regs_t *spi, unsigned int hz);
unsigned int MXC_SPI_RevA_GetFrequency(mxc_spi_reva_regs_t *spi);
int MXC_SPI_RevA_SetDataSize(mxc_spi_reva_regs_t *spi, int dataSize);
int MXC_SPI_RevA_GetDataSize(mxc_spi_reva_regs_t *spi);
int MXC_SPI_RevA_SetMTMode(mxc_spi_reva_regs_t *spi, int mtMode);
int MXC_SPI_RevA_GetMTMode(mxc_spi_reva_regs_t *spi);
int MXC_SPI_RevA_SetSlave(mxc_spi_reva_regs_t *spi, int ssIdx);
int MXC_SPI_RevA_GetSlave(mxc_spi_reva_regs_t *spi);
int MXC_SPI_RevA_SetWidth(mxc_spi_reva_regs_t *spi, mxc_spi_reva_width_t spiWidth);
mxc_spi_reva_width_t MXC_SPI_RevA_GetWidth(mxc_spi_reva_regs_t *spi);
int MXC_SPI_RevA_SetMode(mxc_spi_reva_regs_t *spi, mxc_spi_reva_mode_t spiMode);
mxc_spi_reva_mode_t MXC_SPI_RevA_GetMode(mxc_spi_reva_regs_t *spi);
int MXC_SPI_RevA_StartTransmission(mxc_spi_reva_regs_t *spi);
int MXC_SPI_RevA_GetActive(mxc_spi_reva_regs_t *spi);
int MXC_SPI_RevA_AbortTransmission(mxc_spi_reva_regs_t *spi);
unsigned int MXC_SPI_RevA_ReadRXFIFO(mxc_spi_reva_regs_t *spi, unsigned char *bytes,
                                     unsigned int len);
unsigned int MXC_SPI_RevA_WriteTXFIFO(mxc_spi_reva_regs_t *spi, unsigned char *bytes,
                                      unsigned int len);
unsigned int MXC_SPI_RevA_GetTXFIFOAvailable(mxc_spi_reva_regs_t *spi);
unsigned int MXC_SPI_RevA_GetRXFIFOAvailable(mxc_spi_reva_regs_t *spi);
void MXC_SPI_RevA_ClearRXFIFO(mxc_spi_reva_regs_t *spi);
void MXC_SPI_RevA_ClearTXFIFO(mxc_spi_reva_regs_t *spi);
int MXC_SPI_RevA_SetRXThreshold(mxc_spi_reva_regs_t *spi, unsigned int numBytes);
unsigned int MXC_SPI_RevA_GetRXThreshold(mxc_spi_reva_regs_t *spi);
int MXC_SPI_RevA_SetTXThreshold(mxc_spi_reva_regs_t *spi, unsigned int numBytes);
unsigned int MXC_SPI_RevA_GetTXThreshold(mxc_spi_reva_regs_t *spi);
unsigned int MXC_SPI_RevA_GetFlags(mxc_spi_reva_regs_t *spi);
void MXC_SPI_RevA_ClearFlags(mxc_spi_reva_regs_t *spi);
void MXC_SPI_RevA_EnableInt(mxc_spi_reva_regs_t *spi, unsigned int mask);
void MXC_SPI_RevA_DisableInt(mxc_spi_reva_regs_t *spi, unsigned int mask);
int MXC_SPI_RevA_MasterTransaction(mxc_spi_reva_req_t *req);
int MXC_SPI_RevA_MasterTransactionAsync(mxc_spi_reva_req_t *req);
int MXC_SPI_RevA_MasterTransactionDMA(mxc_spi_reva_req_t *req, int reqselTx, int reqselRx,
                                      mxc_dma_regs_t *dma);
int MXC_SPI_RevA_SlaveTransaction(mxc_spi_reva_req_t *req);
int MXC_SPI_RevA_SlaveTransactionAsync(mxc_spi_reva_req_t *req);
int MXC_SPI_RevA_SlaveTransactionDMA(mxc_spi_reva_req_t *req, int reqselTx, int reqselRx,
                                     mxc_dma_regs_t *dma);
void MXC_SPI_RevA_DMACallback(int ch, int error);
int MXC_SPI_RevA_SetDefaultTXData(mxc_spi_reva_regs_t *spi, unsigned int defaultTXData);
void MXC_SPI_RevA_AbortAsync(mxc_spi_reva_regs_t *spi);
void MXC_SPI_RevA_AsyncHandler(mxc_spi_reva_regs_t *spi);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_SPI_SPI_REVA_H_
