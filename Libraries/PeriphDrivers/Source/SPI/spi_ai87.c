/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc., All Rights Reserved.
 * (now owned by Analog Devices, Inc.)
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
 ******************************************************************************
 *
 * Copyright 2023 Analog Devices, Inc.
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
#include <stdbool.h>

#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "spi_reva1.h"
#include "dma.h"

/* **** Definitions **** */

/* ************************************************************************** */
int MXC_SPI_Init(mxc_spi_regs_t *spi, mxc_spi_type_t controller_target, mxc_spi_interface_t if_mode,
                 int numTargets, uint8_t ts_active_pol_mask, uint32_t freq, mxc_spi_pins_t pins)
{
    int spi_num;

    // Remap input parameters for v1 implementation.
    int masterMode = controller_target;
    int quadModeUsed = if_mode;
    int numSlaves = numTargets;
    int ssPolarity = ts_active_pol_mask;
    int hz = freq;

    spi_num = MXC_SPI_GET_IDX(spi);
    MXC_ASSERT(spi_num >= 0);

    if (numSlaves > MXC_SPI_SS_INSTANCES) {
        return E_BAD_PARAM;
    }

    // Check if frequency is too high
    if ((spi_num == 0) && (hz > PeripheralClock)) {
        return E_BAD_PARAM;
    }

    if ((spi_num == 1) && (hz > SystemCoreClock)) {
        return E_BAD_PARAM;
    }

    mxc_gpio_cfg_t gpio_cfg_spi;
    gpio_cfg_spi.pad = MXC_GPIO_PAD_NONE;
    gpio_cfg_spi.port = MXC_GPIO0;

    // Set VDDIO level
    if (pins.vddioh) {
        gpio_cfg_spi.vssel = MXC_GPIO_VSSEL_VDDIOH;
    } else {
        gpio_cfg_spi.vssel = MXC_GPIO_VSSEL_VDDIO;
    }

    // Configure GPIO for spi
    if (spi == MXC_SPI1) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_SPI1);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI1);

        //Define pins
        if (pins.ss1) {
            gpio_cfg_spi.mask = MXC_GPIO_PIN_26;
            gpio_cfg_spi.func = MXC_GPIO_FUNC_ALT2;
            MXC_GPIO_Config(&gpio_cfg_spi);
        }

        if (pins.ss2) {
            gpio_cfg_spi.func = MXC_GPIO_FUNC_ALT2;
            gpio_cfg_spi.mask = MXC_GPIO_PIN_27;
            MXC_GPIO_Config(&gpio_cfg_spi);
        }

        //clear mask
        gpio_cfg_spi.mask = 0;

        // check rest of the pins
        if (pins.clock) {
            gpio_cfg_spi.mask |= MXC_GPIO_PIN_23;
        }

        if (pins.miso) {
            gpio_cfg_spi.mask |= MXC_GPIO_PIN_22;
        }

        if (pins.mosi) {
            gpio_cfg_spi.mask |= MXC_GPIO_PIN_21;
        }

        if (pins.sdio2) {
            gpio_cfg_spi.mask |= MXC_GPIO_PIN_24;
        }

        if (pins.sdio3) {
            gpio_cfg_spi.mask |= MXC_GPIO_PIN_25;
        }

        if (pins.ss0) {
            gpio_cfg_spi.mask |= MXC_GPIO_PIN_20;
        }

        gpio_cfg_spi.func = MXC_GPIO_FUNC_ALT1;

#ifdef MXC_SPI0
    } else if (spi == MXC_SPI0) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET1_SPI0);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI0);

        //Define pins
        if (pins.ss1) {
            gpio_cfg_spi.mask = MXC_GPIO_PIN_11;
            gpio_cfg_spi.func = MXC_GPIO_FUNC_ALT2;
            MXC_GPIO_Config(&gpio_cfg_spi);
        }

        if (pins.ss2) {
            gpio_cfg_spi.func = MXC_GPIO_FUNC_ALT2;
            gpio_cfg_spi.mask = MXC_GPIO_PIN_10;
            MXC_GPIO_Config(&gpio_cfg_spi);
        }

        //clear mask
        gpio_cfg_spi.mask = 0;

        // check rest of the pins
        if (pins.clock) {
            gpio_cfg_spi.mask |= MXC_GPIO_PIN_7;
        }

        if (pins.miso) {
            gpio_cfg_spi.mask |= MXC_GPIO_PIN_6;
        }

        if (pins.mosi) {
            gpio_cfg_spi.mask |= MXC_GPIO_PIN_5;
        }

        if (pins.sdio2) {
            gpio_cfg_spi.mask |= MXC_GPIO_PIN_8;
        }

        if (pins.sdio3) {
            gpio_cfg_spi.mask |= MXC_GPIO_PIN_9;
        }

        if (pins.ss0) {
            gpio_cfg_spi.mask |= MXC_GPIO_PIN_4;
        }

        gpio_cfg_spi.func = MXC_GPIO_FUNC_ALT1;
#endif
    } else {
        return E_NO_DEVICE;
    }

    MXC_GPIO_Config(&gpio_cfg_spi);

    return MXC_SPI_RevA1_Init((mxc_spi_reva_regs_t *)spi, masterMode, quadModeUsed, numSlaves,
                              ssPolarity, hz);
}

int MXC_SPI_Shutdown(mxc_spi_regs_t *spi)
{
    int spi_num;
    spi_num = MXC_SPI_GET_IDX(spi);
    MXC_ASSERT(spi_num >= 0);
    (void)spi_num;

    MXC_SPI_RevA1_Shutdown((mxc_spi_reva_regs_t *)spi);

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

int MXC_SPI_ReadyForSleep(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_ReadyForSleep((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_GetPeripheralClock(mxc_spi_regs_t *spi)
{
    int retval;

    if (spi == MXC_SPI1) {
        retval = PeripheralClock;
#ifdef MXC_SPI0 // SPI0 is not accessible from the RISC core.
    } else if (spi == MXC_SPI0) {
        int sys_clk = (MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_SYSCLK_SEL) >>
                      MXC_F_GCR_CLKCTRL_SYSCLK_SEL_POS;
        switch (sys_clk) {
        case MXC_SYS_CLOCK_IPO:
            retval = IPO_FREQ;
            break;
        case MXC_SYS_CLOCK_IBRO:
            retval = IBRO_FREQ;
            break;
        case MXC_SYS_CLOCK_ISO:
            retval = ISO_FREQ;
            break;
        case MXC_SYS_CLOCK_INRO:
            retval = INRO_FREQ;
            break;
        case MXC_SYS_CLOCK_ERTCO:
            retval = ERTCO_FREQ;
            break;
        case MXC_SYS_CLOCK_EXTCLK:
            retval = EXTCLK_FREQ;
            break;
        default:
            return E_BAD_STATE;
        }
#endif // MXC_SPI0
    } else {
        return E_BAD_PARAM;
    }

    retval /= 2;

    return retval;
}

int MXC_SPI_SetFrequency(mxc_spi_regs_t *spi, unsigned int hz)
{
    return MXC_SPI_RevA1_SetFrequency((mxc_spi_reva_regs_t *)spi, hz);
}

unsigned int MXC_SPI_GetFrequency(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_GetFrequency((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_SetDataSize(mxc_spi_regs_t *spi, int dataSize)
{
    return MXC_SPI_RevA1_SetDataSize((mxc_spi_reva_regs_t *)spi, dataSize);
}

int MXC_SPI_GetDataSize(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_GetDataSize((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_SetSlave(mxc_spi_regs_t *spi, int ssIdx)
{
    return MXC_SPI_RevA1_SetSlave((mxc_spi_reva_regs_t *)spi, ssIdx);
}

int MXC_SPI_GetSlave(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_GetSlave((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_SetWidth(mxc_spi_regs_t *spi, mxc_spi_width_t spiWidth)
{
    return MXC_SPI_RevA1_SetWidth((mxc_spi_reva_regs_t *)spi, spiWidth);
}

mxc_spi_width_t MXC_SPI_GetWidth(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_GetWidth((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_SetMode(mxc_spi_regs_t *spi, mxc_spi_mode_t spiMode)
{
    return MXC_SPI_RevA1_SetMode((mxc_spi_reva_regs_t *)spi, spiMode);
}

mxc_spi_mode_t MXC_SPI_GetMode(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_GetMode((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_StartTransmission(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_StartTransmission((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_GetActive(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_GetActive((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_AbortTransmission(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_AbortTransmission((mxc_spi_reva_regs_t *)spi);
}

unsigned int MXC_SPI_ReadRXFIFO(mxc_spi_regs_t *spi, unsigned char *bytes, unsigned int len)
{
    return MXC_SPI_RevA1_ReadRXFIFO((mxc_spi_reva_regs_t *)spi, bytes, len);
}

unsigned int MXC_SPI_GetRXFIFOAvailable(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_GetRXFIFOAvailable((mxc_spi_reva_regs_t *)spi);
}

unsigned int MXC_SPI_WriteTXFIFO(mxc_spi_regs_t *spi, unsigned char *bytes, unsigned int len)
{
    return MXC_SPI_RevA1_WriteTXFIFO((mxc_spi_reva_regs_t *)spi, bytes, len);
}

unsigned int MXC_SPI_GetTXFIFOAvailable(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_GetTXFIFOAvailable((mxc_spi_reva_regs_t *)spi);
}

void MXC_SPI_ClearRXFIFO(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevA1_ClearRXFIFO((mxc_spi_reva_regs_t *)spi);
}

void MXC_SPI_ClearTXFIFO(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevA1_ClearTXFIFO((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_SetRXThreshold(mxc_spi_regs_t *spi, unsigned int numBytes)
{
    return MXC_SPI_RevA1_SetRXThreshold((mxc_spi_reva_regs_t *)spi, numBytes);
}

unsigned int MXC_SPI_GetRXThreshold(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_GetRXThreshold((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_SetTXThreshold(mxc_spi_regs_t *spi, unsigned int numBytes)
{
    return MXC_SPI_RevA1_SetTXThreshold((mxc_spi_reva_regs_t *)spi, numBytes);
}

unsigned int MXC_SPI_GetTXThreshold(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_GetTXThreshold((mxc_spi_reva_regs_t *)spi);
}

unsigned int MXC_SPI_GetFlags(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_GetFlags((mxc_spi_reva_regs_t *)spi);
}

void MXC_SPI_ClearFlags(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevA1_ClearFlags((mxc_spi_reva_regs_t *)spi);
}

void MXC_SPI_EnableInt(mxc_spi_regs_t *spi, unsigned int intEn)
{
    MXC_SPI_RevA1_EnableInt((mxc_spi_reva_regs_t *)spi, intEn);
}

void MXC_SPI_DisableInt(mxc_spi_regs_t *spi, unsigned int intDis)
{
    MXC_SPI_RevA1_DisableInt((mxc_spi_reva_regs_t *)spi, intDis);
}

int MXC_SPI_MasterTransaction(mxc_spi_req_t *req)
{
    return MXC_SPI_RevA1_MasterTransaction((mxc_spi_reva_req_t *)req);
}

int MXC_SPI_MasterTransactionAsync(mxc_spi_req_t *req)
{
    return MXC_SPI_RevA1_MasterTransactionAsync((mxc_spi_reva_req_t *)req);
}

int MXC_SPI_MasterTransactionDMA(mxc_spi_req_t *req)
{
    int reqselTx = -1;
    int reqselRx = -1;

    int spi_num;

    spi_num = MXC_SPI_GET_IDX(req->spi);
    MXC_ASSERT(spi_num >= 0);

    if (req->txData != NULL) {
        switch (spi_num) {
        case 0:
            reqselTx = MXC_DMA_REQUEST_SPI1TX;
            break;

        case 1:
            reqselTx = MXC_DMA_REQUEST_SPI0TX;
            break;

        default:
            return E_BAD_PARAM;
        }
    }

    if (req->rxData != NULL) {
        switch (spi_num) {
        case 0:
            reqselRx = MXC_DMA_REQUEST_SPI1RX;
            break;

        case 1:
            reqselRx = MXC_DMA_REQUEST_SPI0RX;
            break;

        default:
            return E_BAD_PARAM;
        }
    }

    return MXC_SPI_RevA1_MasterTransactionDMA((mxc_spi_reva_req_t *)req, reqselTx, reqselRx,
                                              MXC_DMA);
}

int MXC_SPI_SlaveTransaction(mxc_spi_req_t *req)
{
    return MXC_SPI_RevA1_SlaveTransaction((mxc_spi_reva_req_t *)req);
}

int MXC_SPI_SlaveTransactionAsync(mxc_spi_req_t *req)
{
    return MXC_SPI_RevA1_SlaveTransactionAsync((mxc_spi_reva_req_t *)req);
}

int MXC_SPI_SlaveTransactionDMA(mxc_spi_req_t *req)
{
    int reqselTx = -1;
    int reqselRx = -1;

    int spi_num;

    spi_num = MXC_SPI_GET_IDX(req->spi);
    MXC_ASSERT(spi_num >= 0);

    if (req->txData != NULL) {
        switch (spi_num) {
        case 0:
            reqselTx = MXC_DMA_REQUEST_SPI1TX;
            break;

        case 1:
            reqselTx = MXC_DMA_REQUEST_SPI0TX;
            break;

        default:
            return E_BAD_PARAM;
            break;
        }
    }

    if (req->rxData != NULL) {
        switch (spi_num) {
        case 0:
            reqselRx = MXC_DMA_REQUEST_SPI1RX;
            break;

        case 1:
            reqselRx = MXC_DMA_REQUEST_SPI0RX;
            break;

        default:
            return E_BAD_PARAM;
            break;
        }
    }

    return MXC_SPI_RevA1_SlaveTransactionDMA((mxc_spi_reva_req_t *)req, reqselTx, reqselRx,
                                             MXC_DMA);
}

int MXC_SPI_SetDefaultTXData(mxc_spi_regs_t *spi, unsigned int defaultTXData)
{
    return MXC_SPI_RevA1_SetDefaultTXData((mxc_spi_reva_regs_t *)spi, defaultTXData);
}

void MXC_SPI_AbortAsync(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevA1_AbortAsync((mxc_spi_reva_regs_t *)spi);
}

void MXC_SPI_AsyncHandler(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevA1_AsyncHandler((mxc_spi_reva_regs_t *)spi);
}

/* ** SPI v2 functions to prevent build errors ** */

int MXC_SPI_Config(mxc_spi_cfg_t *cfg)
{
    return E_NOT_SUPPORTED;
}

int MXC_SPI_ConfigStruct(mxc_spi_cfg_t *cfg, bool use_dma_tx, bool use_dma_rx)
{
    return E_NOT_SUPPORTED;
}

int MXC_SPI_SetTSControl(mxc_spi_regs_t *spi, mxc_spi_tscontrol_t ts_control)
{
    return E_NOT_SUPPORTED;
}

mxc_spi_tscontrol_t MXC_SPI_GetTSControl(mxc_spi_regs_t *spi)
{
    return E_NOT_SUPPORTED;
}

int MXC_SPI_SetFrameSize(mxc_spi_regs_t *spi, int frame_size)
{
    return MXC_SPI_SetDataSize(spi, frame_size);
}

int MXC_SPI_GetFrameSize(mxc_spi_regs_t *spi)
{
    return MXC_SPI_GetDataSize(spi);
}

int MXC_SPI_SetInterface(mxc_spi_regs_t *spi, mxc_spi_interface_t mode)
{
    return E_NOT_SUPPORTED;
}

mxc_spi_interface_t MXC_SPI_GetInterface(mxc_spi_regs_t *spi)
{
    return E_NOT_SUPPORTED;
}

int MXC_SPI_SetClkMode(mxc_spi_regs_t *spi, mxc_spi_clkmode_t clk_mode)
{
    return E_NOT_SUPPORTED;
}

mxc_spi_clkmode_t MXC_SPI_GetClkMode(mxc_spi_regs_t *spi)
{
    return E_NOT_SUPPORTED;
}

int MXC_SPI_SetCallback(mxc_spi_regs_t *spi, mxc_spi_callback_t callback, void *data)
{
    return E_NOT_SUPPORTED;
}

int MXC_SPI_SetDummyTX(mxc_spi_regs_t *spi, uint16_t tx_value)
{
    return MXC_SPI_SetDefaultTXData(spi, tx_value);
}

/* ** DMA-Specific Functions ** */

int MXC_SPI_DMA_Init(mxc_spi_regs_t *spi, mxc_dma_regs_t *dma, bool use_dma_tx, bool use_dma_rx)
{
    return E_NOT_SUPPORTED;
}

bool MXC_SPI_DMA_GetInitialized(mxc_spi_regs_t *spi)
{
    return E_NOT_SUPPORTED;
}

int MXC_SPI_DMA_GetTXChannel(mxc_spi_regs_t *spi)
{
    return E_NOT_SUPPORTED;
}

int MXC_SPI_DMA_GetRXChannel(mxc_spi_regs_t *spi)
{
    return E_NOT_SUPPORTED;
}

int MXC_SPI_DMA_SetRequestSelect(mxc_spi_regs_t *spi, bool use_dma_tx, bool use_dma_rx)
{
    return E_NOT_SUPPORTED;
}

/* ** Transaction Functions ** */

int MXC_SPI_ControllerTransaction(mxc_spi_req_t *req)
{
    return MXC_SPI_MasterTransaction(req);
}

int MXC_SPI_ControllerTransactionAsync(mxc_spi_req_t *req)
{
    return MXC_SPI_MasterTransactionAsync(req);
}

int MXC_SPI_ControllerTransactionDMA(mxc_spi_req_t *req)
{
    return MXC_SPI_MasterTransactionDMA(req);
}

int MXC_SPI_TargetTransaction(mxc_spi_req_t *req)
{
    return MXC_SPI_SlaveTransaction(req);
}

int MXC_SPI_TargetTransactionAsync(mxc_spi_req_t *req)
{
    return MXC_SPI_SlaveTransactionAsync(req);
}

int MXC_SPI_TargetTransactionDMA(mxc_spi_req_t *req)
{
    return MXC_SPI_SlaveTransactionDMA(req);
}

/* ** Handler Functions ** */

void MXC_SPI_Handler(mxc_spi_regs_t *spi)
{
    MXC_SPI_AsyncHandler(spi);
}

void MXC_SPI_DMA_TX_Handler(mxc_spi_regs_t *spi)
{
    return;
}

void MXC_SPI_DMA_RX_Handler(mxc_spi_regs_t *spi)
{
    return;
}
