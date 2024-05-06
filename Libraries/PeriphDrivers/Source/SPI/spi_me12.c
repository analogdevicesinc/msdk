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
int MXC_SPI_Init(mxc_spi_regs_t *spi, int masterMode, int quadModeUsed, int numSlaves,
                 unsigned ssPolarity, unsigned int hz, mxc_spi_pins_t pins)
{
    int spi_num;

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

#ifndef MSDK_NO_GPIO_CLK_INIT
    mxc_gpio_cfg_t gpio_cfg_spi;
    gpio_cfg_spi.pad = MXC_GPIO_PAD_NONE;
    gpio_cfg_spi.port = MXC_GPIO0;

    // Set VDDIO level
    if (pins.vssel) {
        gpio_cfg_spi.vssel = MXC_GPIO_VSSEL_VDDIOH;
    } else {
        gpio_cfg_spi.vssel = MXC_GPIO_VSSEL_VDDIO;
    }

    // Configure GPIO for spi
    if (spi == MXC_SPI1) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_SPI1);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI1);

        //clear mask
        gpio_cfg_spi.mask = 0;

        if (pins.map_a) {
            // check rest of the pins
            if (pins.clock) {
                gpio_cfg_spi.mask |= MXC_GPIO_PIN_17;
            }
            if (pins.miso) {
                gpio_cfg_spi.mask |= MXC_GPIO_PIN_7;
            }
            if (pins.mosi) {
                gpio_cfg_spi.mask |= MXC_GPIO_PIN_8;
            }
            if (pins.ss0) {
                gpio_cfg_spi.mask |= MXC_GPIO_PIN_18;
            }

            gpio_cfg_spi.func = MXC_GPIO_FUNC_ALT1;
        } else {
            // check rest of the pins
            if (pins.clock) {
                gpio_cfg_spi.mask |= MXC_GPIO_PIN_11;
            }
            if (pins.miso) {
                gpio_cfg_spi.mask |= MXC_GPIO_PIN_13;
            }
            if (pins.mosi) {
                gpio_cfg_spi.mask |= MXC_GPIO_PIN_12;
            }
            if (pins.ss0) {
                gpio_cfg_spi.mask |= MXC_GPIO_PIN_10;
            }

            gpio_cfg_spi.func = MXC_GPIO_FUNC_ALT2;
        }
#ifdef MXC_SPI0
    } else if (spi == MXC_SPI0) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_SPI0);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI0);

        //clear mask
        gpio_cfg_spi.mask = 0;

        // check rest of the pins
        if (pins.clock) {
            gpio_cfg_spi.mask |= MXC_GPIO_PIN_4;
        }

        if (pins.miso) {
            gpio_cfg_spi.mask |= MXC_GPIO_PIN_2;
        }

        if (pins.mosi) {
            gpio_cfg_spi.mask |= MXC_GPIO_PIN_3;
        }

        if (pins.ss0) {
            gpio_cfg_spi.mask |= MXC_GPIO_PIN_5;
        }

        gpio_cfg_spi.func = MXC_GPIO_FUNC_ALT1;
#endif
    } else {
        return E_NO_DEVICE;
    }

    MXC_GPIO_Config(&gpio_cfg_spi);
#else
    (void)pins;
#endif // MSDK_NO_GPIO_CLK_INIT

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
    if (MXC_SPI_GET_IDX(spi) > -1 && MXC_SPI_GET_IDX(spi) < 3) {
        return PeripheralClock;
    } else if (MXC_SPI_GET_IDX(spi) > 2) {
        uint32_t clk_src = (MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_SYSCLK_SEL) >>
                           MXC_F_GCR_CLKCTRL_SYSCLK_SEL_POS;
        switch (clk_src) {
        case MXC_SYS_CLOCK_IPO:
            return IPO_FREQ;
        case MXC_SYS_CLOCK_ERFO:
            return ERFO_FREQ;
        case MXC_SYS_CLOCK_IBRO:
            return IBRO_FREQ;
        case MXC_SYS_CLOCK_INRO:
            return INRO_FREQ;
        case MXC_SYS_CLOCK_ERTCO:
            return ERTCO_FREQ;
        case MXC_SYS_CLOCK_EXTCLK:
            return HF_EXTCLK_FREQ;
        default:
            return E_BAD_PARAM;
        }
    } else {
        return E_BAD_PARAM;
    }
    return E_NO_ERROR;
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
            reqselTx = MXC_DMA_REQUEST_SPI0TX;
            break;

        case 1:
            reqselTx = MXC_DMA_REQUEST_SPI1TX;
            break;

        default:
            return E_BAD_PARAM;
        }
    }

    if (req->rxData != NULL) {
        switch (spi_num) {
        case 0:
            reqselRx = MXC_DMA_REQUEST_SPI0RX;
            break;

        case 1:
            reqselRx = MXC_DMA_REQUEST_SPI1RX;
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

void MXC_SPI_HWSSControl(mxc_spi_regs_t *spi, int state)
{
    MXC_SPI_RevA1_HWSSControl((mxc_spi_reva_regs_t *)spi, state);
}
