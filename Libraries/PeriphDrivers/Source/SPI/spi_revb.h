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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_SPI_SPI_REVB_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_SPI_SPI_REVB_H_

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

/* **** Functions **** */

void MXC_SPI_RevA_DMA_TX_Handler(mxc_spi_reva_regs_t *spi);

void MXC_SPI_Reva_DMA_RX_Handler(mxc_spi_reva_regs_t *spi);

void MXC_SPI_RevA_Handler(mxc_spi_reva_regs_t *spi);

int MXC_SPI_RevA_Init(mxc_spi_init_t *init);

void MXC_SPI_RevB_SetRegisterCallback(mxc_spi_callback_t callback, void *data);

int MXC_SPI_RevB_MasterTransaction(mxc_spi_reva_regs_t *spi, uint16_t *tx_buffer, uint32_t tx_len, uint16_t *rx_buffer, uint32_t rx_len, uint32_t deassert, uint32_t idx_mask);

int MXC_SPI_RevB_MasterTransactionB(mxc_spi_reva_regs_t *spi, uint16_t *tx_buffer, uint32_t tx_len, uint16_t *rx_buffer, uint32_t rx_len, uint32_t deassert, uint32_t idx_mask);

int MXC_SPI_RevB_MasterTransactionDMA(mxc_spi_reva_regs_t *spi, uint16_t *tx_buffer, uint32_t tx_len, uint16_t *rx_buffer, uint32_t rx_len, uint32_t deassert, uint32_t idx_mask);

int MXC_SPI_RevB_MasterTransactionDMAB(mxc_spi_reva_regs_t *spi, uint16_t *tx_buffer, uint32_t tx_len, uint16_t *rx_buffer, uint32_t rx_len, uint32_t deassert, uint32_t idx_mask);


#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_SPI_SPI_REVB_H_
