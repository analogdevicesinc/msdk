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
#include "spi_fast.h"
#include "dma.h"
#include "dma_reva_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* **** Functions **** */

int MXC_SPI_RevB_Init(mxc_spi_init_t *init);

int MXC_SPI_RevB_Shutdown(mxc_spi_reva_regs_t *spi);

int MXC_SPI_RevB_DMA_GetTXChannel(mxc_spi_reva_regs_t *spi);

int MXC_SPI_RevB_DMA_GetRXChannel(mxc_spi_reva_regs_t *spi);

int MXC_SPI_RevB_SetFrequency(mxc_spi_reva_regs_t *spi, uint32_t freq);

int MXC_SPI_RevB_GetFrequency(mxc_spi_reva_regs_t *spi);

int MXC_SPI_RevB_SetDataSize(mxc_spi_reva_regs_t *spi, int data_size);

int MXC_SPI_RevB_GetDataSize(mxc_spi_reva_regs_t *spi);

int MXC_SPI_RevB_SetWidth(mxc_spi_reva_regs_t *spi, mxc_spi_datawidth_t width);

mxc_spi_datawidth_t MXC_SPI_RevB_GetWidth(mxc_spi_reva_regs_t *spi);

int MXC_SPI_RevB_SetClkMode(mxc_spi_reva_regs_t *spi, mxc_spi_clkmode_t clk_mode);

mxc_spi_clkmode_t MXC_SPI_RevB_GetClkMode(mxc_spi_reva_regs_t *spi);

int MXC_SPI_RevB_DMA_SetRequestSelect(mxc_spi_reva_regs_t *spi, uint32_t spi_tx_reqsel, uint32_t spi_rx_reqsel);

int MXC_SPI_RevB_SetRegisterCallback(mxc_spi_reva_regs_t *spi, mxc_spi_callback_t callback, void *data);

int MXC_SPI_RevB_GetActive(mxc_spi_reva_regs_t *spi);

/* ** Transaction Functions ** */

int MXC_SPI_RevB_MasterTransaction(mxc_spi_reva_regs_t *spi, uint16_t *tx_buffer, uint32_t tx_len, uint16_t *rx_buffer, uint32_t rx_len, uint32_t deassert, mxc_spi_target_t *cs_cfg);

int MXC_SPI_RevB_MasterTransactionB(mxc_spi_reva_regs_t *spi, uint16_t *tx_buffer, uint32_t tx_len, uint16_t *rx_buffer, uint32_t rx_len, uint32_t deassert, mxc_spi_target_t *cs_cfg);

int MXC_SPI_RevB_MasterTransactionDMA(mxc_spi_reva_regs_t *spi, uint16_t *tx_buffer, uint32_t tx_len, uint16_t *rx_buffer, uint32_t rx_len, bool deassert, mxc_spi_target_t *cs_cfg);

int MXC_SPI_RevB_MasterTransactionDMAB(mxc_spi_reva_regs_t *spi, uint16_t *tx_buffer, uint32_t tx_len, uint16_t *rx_buffer, uint32_t rx_len, bool deassert, mxc_spi_target_t *cs_cfg);

/* ** Handler Functions ** */

void MXC_SPI_RevB_Handler(mxc_spi_reva_regs_t *spi);

void MXC_SPI_RevB_DMA_TX_Handler(mxc_spi_reva_regs_t *spi);

void MXC_SPI_RevB_DMA_RX_Handler(mxc_spi_reva_regs_t *spi);


#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_SPI_SPI_REVB_H_
