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
#include <stdbool.h>

#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_errors.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "spi_revb.h"
#include "spi_fast.h"
#include "dma.h"

/* **** Definitions **** */

/* ************************************************************************** */
// TODO: MXC_SPI_Init will be used once, the original RevA prototype is deprecated and removed.
// int MXC_SPI_Init_New(mxc_spi_init_t *init)

int MXC_SPI_Init(mxc_spi_regs_t *spi, int masterMode, int quadModeUsed, int numSlaves,
                 unsigned ssPolarity, unsigned int hz, mxc_spi_pins_t pins)
{
    int error, spi_num;

    // TODO(5-15-2023): Remove this section when the Init function is updated to
    //      int MXC_SPI_Init(mxc_spi_init_t *init)
    // This section is for backwards compatibility, before fully updating to new
    //   implementation.
//>>> 
    mxc_spi_init_t spi_init;
    mxc_spi_init_t *init = &spi_init;

    // Set up init struct.
    init->spi = spi;

    if (masterMode) {
        init->type = MXC_SPI_TYPE_CONTROLLER;  // L. Master
    } else {
        init->type = MXC_SPI_TYPE_TARGET; // L. Slave
    }

    init->freq = hz;

    if (quadModeUsed) {
        init->width = MXC_SPI_WIDTH_QUAD;
    } else {
        init->width = MXC_SPI_WIDTH_STANDARD;
    }

    // New SPI drivers will not use "mxc_spi_pins_t pins" anymore.
    init->spi_pins = NULL;

    // Due to different implementation, any Slave configuration is not done in the Init function.
    // ssPolarity and numSlaves not used.
//<<<

    spi_num = MXC_SPI_GET_IDX(init->spi);
    MXC_ASSERT(spi_num >= 0);

    // Check if frequency is too high
    if ((spi_num == 0) && (init->freq > PeripheralClock)) {
        return E_BAD_PARAM;
    }

    if ((spi_num == 1) && (init->freq > SystemCoreClock)) {
        return E_BAD_PARAM;
    }

    // Note: Target Select (L. SS) Pins will be configured in MXC_SPI_RevB_Init.
    // Configure GPIO for spi
    if (init->spi == MXC_SPI1) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_SPI1);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI1);

        // Configure SPI to default pins if not provided.
        if (init->spi_pins == NULL) {
            if (init->width == MXC_SPI_WIDTH_3WIRE) {
                error = MXC_GPIO_Config(&gpio_cfg_spi1_3wire);

            } else if (init->width == MXC_SPI_WIDTH_STANDARD) {
                error = MXC_GPIO_Config(&gpio_cfg_spi1_standard);

            } else if (init->width == MXC_SPI_WIDTH_DUAL) {
                error = MXC_GPIO_Config(&gpio_cfg_spi1_dual);

            } else if (init->width == MXC_SPI_WIDTH_QUAD) {
                error = MXC_GPIO_Config(&gpio_cfg_spi1_quad);

            } else {
                return E_BAD_PARAM;
            }

        } else {
            error = MXC_GPIO_Config(init->spi_pins);
        }

        // Ensure SPI GPIO pins were properly configured.
        if (error != E_NO_ERROR) {
            return error;
        }

// Handles RISCV SPI Numbering.
#ifdef MXC_SPI0
    } else if (init->spi == MXC_SPI0) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET1_SPI0);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI0);

        // Configure SPI to default pins if not provided.
        if (init->spi_pins == NULL) {
            if (init->width == MXC_SPI_WIDTH_3WIRE) {
                error = MXC_GPIO_Config(&gpio_cfg_spi0_3wire);

            } else if (init->width == MXC_SPI_WIDTH_STANDARD) {
// TODO: Standard vs Mono?
                error = MXC_GPIO_Config(&gpio_cfg_spi0_standard);

            } else if (init->width == MXC_SPI_WIDTH_DUAL) {
                error = MXC_GPIO_Config(&gpio_cfg_spi0_dual);

            } else if (init->width == MXC_SPI_WIDTH_QUAD) {
                error = MXC_GPIO_Config(&gpio_cfg_spi0_quad);

            } else {
                return E_BAD_PARAM;
            }

        } else {
            error = MXC_GPIO_Config(init->spi_pins);
        }

        // Ensure SPI GPIO pins were properly configured.
        if (error != E_NO_ERROR) {
            return error;
        }
#endif

    } else {
        return E_NO_DEVICE;
    }

    return MXC_SPI_RevB_Init(init);
}

int MXC_SPI_Shutdown(mxc_spi_regs_t *spi)
{
    int spi_num;
    spi_num = MXC_SPI_GET_IDX(spi);
    MXC_ASSERT(spi_num >= 0);
    (void)spi_num;

    MXC_SPI_RevB_Shutdown((mxc_spi_reva_regs_t *)spi);

    if (spi == MXC_SPI1) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPI1);
#ifdef MXC_SPI0
    } else if (spi == MXC_SPI0) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPI0);
#endif
    } else {
        return E_NO_DEVICE;
    }

    return E_NO_ERROR;
}

int MXC_SPI_DMA_GetTXChannel(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevB_DMA_GetTXChannel((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_DMA_GetRXChannel(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevB_DMA_GetRXChannel((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_SetFrequency(mxc_spi_regs_t *spi, uint32_t freq)
{
    return MXC_SPI_RevB_SetFrequency((mxc_spi_reva_regs_t *)spi, freq);
}

int MXC_SPI_GetFrequency(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevB_GetFrequency((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_SetDataSize(mxc_spi_regs_t *spi, int data_size)
{
    return MXC_SPI_RevB_SetDataSize((mxc_spi_reva_regs_t *)spi, data_size);
}

int MXC_SPI_GetDataSize(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevB_GetDataSize((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_SetWidth(mxc_spi_regs_t *spi, mxc_spi_datawidth_t width)
{
    return MXC_SPI_RevB_SetWidth((mxc_spi_reva_regs_t *)spi, width);
}

mxc_spi_datawidth_t MXC_SPI_GetWidth(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevB_GetWidth((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_SetClkMode(mxc_spi_regs_t *spi, mxc_spi_clkmode_t clk_mode)
{
    return MXC_SPI_RevB_SetClkMode((mxc_spi_reva_regs_t *)spi, clk_mode);
}

mxc_spi_clkmode_t MXC_SPI_GetClkMode(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevB_GetClkMode((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_DMA_SetRequestSelect(mxc_spi_req_t *req)
{
    int spi_num;
    int tx_reqsel = -1;
    int rx_reqsel = -1;

    spi_num = MXC_SPI_GET_IDX(req->spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_INVALID;
    }

    if (req->tx_buffer != NULL) {
        switch (spi_num) {
            case 0:
                tx_reqsel = MXC_DMA_REQUEST_SPI1TX;
                break;

            case 1:
                tx_reqsel = MXC_DMA_REQUEST_SPI0TX;
                break;

            default:
                return E_BAD_PARAM;
        }
    }

    if (req->rx_buffer != NULL) {
        switch (spi_num) {
            case 0:
                rx_reqsel = MXC_DMA_REQUEST_SPI1RX;
                break;

            case 1:
                rx_reqsel = MXC_DMA_REQUEST_SPI0RX;
                break;

            default:
                return E_BAD_PARAM;
        }
    }

    return MXC_SPI_RevB_DMA_SetRequestSelect((mxc_spi_reva_regs_t *)(req->spi), tx_reqsel, rx_reqsel);
}

int MXC_SPI_SetRegisterCallback(mxc_spi_regs_t *spi, mxc_spi_callback_t callback, void *data)
{
    return MXC_SPI_RevB_SetRegisterCallback((mxc_spi_reva_regs_t *)spi, callback, data);
}

int MXC_SPI_GetActive(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevB_GetActive((mxc_spi_reva_regs_t *)spi);
}

/* ** Transaction Functions ** */

int MXC_SPI_MasterTransaction(mxc_spi_req_t *req)
{
    return MXC_SPI_RevB_MasterTransactionB((mxc_spi_reva_regs_t *)(req->spi), req->tx_buffer, req->tx_len, req->rx_buffer, req->rx_len, req->deassert, NULL);
}

int MXC_SPI_MasterTransactionAsync(mxc_spi_req_t *req)
{
    return MXC_SPI_RevB_MasterTransaction((mxc_spi_reva_regs_t*)(req->spi), req->tx_buffer, req->tx_len, req->rx_buffer, req->rx_len, req->deassert, NULL);
}

int MXC_SPI_MasterTransactionDMA(mxc_spi_req_t *req)
{
    return MXC_SPI_RevB_MasterTransactionDMA((mxc_spi_reva_regs_t*)(req->spi), req->tx_buffer, req->tx_len, req->rx_buffer, req->rx_len, req->deassert, NULL);
}

int MXC_SPI_MTransaction(mxc_spi_regs_t *spi, uint16_t *tx_buffer, uint32_t tx_len, uint16_t *rx_buffer, uint32_t rx_len, uint8_t deassert, mxc_spi_target_t *cs_cfg)
{
    return MXC_SPI_RevB_MasterTransaction((mxc_spi_reva_regs_t *)spi, tx_buffer, tx_len, rx_buffer, rx_len, deassert, cs_cfg);;
}

int MXC_SPI_MTransactionB(mxc_spi_regs_t *spi, uint16_t *tx_buffer, uint32_t tx_len, uint16_t *rx_buffer, uint32_t rx_len, uint8_t deassert, mxc_spi_target_t *cs_cfg)
{
    return MXC_SPI_RevB_MasterTransactionB((mxc_spi_reva_regs_t *)spi, tx_buffer, tx_len, rx_buffer, rx_len, deassert, cs_cfg);
}

int MXC_SPI_MTransactionDMA(mxc_spi_regs_t *spi, uint16_t *tx_buffer, uint32_t tx_len, uint16_t *rx_buffer, uint32_t rx_len, uint8_t deassert, mxc_spi_target_t *cs_cfg)
{
    return MXC_SPI_RevB_MasterTransactionDMA((mxc_spi_reva_regs_t *)spi, tx_buffer, tx_len, rx_buffer, rx_len, deassert, cs_cfg);;
}

int MXC_SPI_MTransactionDMAB(mxc_spi_regs_t *spi, uint16_t *tx_buffer, uint32_t tx_len, uint16_t *rx_buffer, uint32_t rx_len, uint8_t deassert, mxc_spi_target_t *cs_cfg)
{
    return MXC_SPI_RevB_MasterTransactionDMAB((mxc_spi_reva_regs_t *)spi, tx_buffer, tx_len, rx_buffer, rx_len, deassert, cs_cfg);;
}

/* ** Handler Functions ** */

void MXC_SPI_Handler(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevB_Handler((mxc_spi_reva_regs_t *)spi);
}

void MXC_SPI_DMA_TX_Handler(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevB_DMA_TX_Handler((mxc_spi_reva_regs_t *)spi);
}

void MXC_SPI_DMA_RX_Handler(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevB_DMA_RX_Handler((mxc_spi_reva_regs_t *)spi);
}
