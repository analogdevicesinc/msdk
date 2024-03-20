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
    if ((spi_num < 3) && (hz > PeripheralClock)) {
        return E_BAD_PARAM;
    }

    if ((spi_num > 2) && (hz > SystemCoreClock)) {
        return E_BAD_PARAM;
    }

#ifndef MSDK_NO_GPIO_CLK_INIT
    // Configure GPIO for spi
    if (spi == MXC_SPI0) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_SPI0);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI0);
        MXC_GPIO_Config(&gpio_cfg_spi0);

        // Enable slave select pins
        if (pins.ss0) {
            MXC_GPIO_Config(&gpio_cfg_spi0_ss0);
        }

        if (pins.ss1) {
            MXC_GPIO_Config(&gpio_cfg_spi0_ss1);
        }

        if (pins.ss2) {
            return E_BAD_PARAM;
        }
    } else if (spi == MXC_SPI1) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_SPI1);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI1);
        MXC_GPIO_Config(&gpio_cfg_spi1);

        // Enable slave select pins
        if (pins.ss0) {
            MXC_GPIO_Config(&gpio_cfg_spi1_ss0);
        }

        if (pins.ss1) {
            MXC_GPIO_Config(&gpio_cfg_spi1_ss1);
        }

        if (pins.ss2) {
            MXC_GPIO_Config(&gpio_cfg_spi1_ss2);
        }
    } else if (spi == MXC_SPI2) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_SPI2);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI2);
        MXC_GPIO_Config(&gpio_cfg_spi2);

        // Enable slave select pins
        if (pins.ss0) {
            MXC_GPIO_Config(&gpio_cfg_spi2_ss0);
        }

        if (pins.ss1) {
            MXC_GPIO_Config(&gpio_cfg_spi2_ss1);
        }

        if (pins.ss2) {
            MXC_GPIO_Config(&gpio_cfg_spi2_ss2);
        }
#ifndef __riscv
    } else if (spi == MXC_SPI3) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET1_SPI3);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI3);
        MXC_GPIO_Config(&gpio_cfg_spi3);

        // Enable slave select pins
        if (pins.ss0) {
            MXC_GPIO_Config(&gpio_cfg_spi3_ss0);
        }

        if (pins.ss1) {
            MXC_GPIO_Config(&gpio_cfg_spi3_ss1);
        }

        if (pins.ss2) {
            MXC_GPIO_Config(&gpio_cfg_spi3_ss2);
        }
    } else if (spi == MXC_SPI4) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET1_SPI4);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI4);
        MXC_GPIO_Config(&gpio_cfg_spi4);

        // Enable slave select pins
        if (pins.ss0) {
            MXC_GPIO_Config(&gpio_cfg_spi4_ss0);
        }

        if (pins.ss1) {
            MXC_GPIO_Config(&gpio_cfg_spi4_ss1);
        }

        if (pins.ss2) {
            MXC_GPIO_Config(&gpio_cfg_spi4_ss2);
        }
#endif // __riscv
    } else {
        return E_NO_DEVICE;
    }
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

    if (spi == MXC_SPI0) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPI0);
    } else if (spi == MXC_SPI1) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPI1);
    } else if (spi == MXC_SPI2) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPI2);
#ifndef __riscv
    } else if (spi == MXC_SPI3) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPI3);
    } else if (spi == MXC_SPI4) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPI4);
#endif //__riscv
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
    int clk_freq = 0;

    if (MXC_SPI_GET_IDX(spi) > -1 && MXC_SPI_GET_IDX(spi) < 3) {
        clk_freq = PeripheralClock;
    } else if (MXC_SPI_GET_IDX(spi) > 2) {
        uint32_t clk_src = (MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_SYSCLK_SEL) >>
                           MXC_F_GCR_CLKCTRL_SYSCLK_SEL_POS;
        switch (clk_src) {
        case MXC_SYS_CLOCK_IPO:
            clk_freq = IPO_FREQ;
            break;
        case MXC_SYS_CLOCK_ERFO:
            clk_freq = ERFO_FREQ;
            break;
        case MXC_SYS_CLOCK_IBRO:
            clk_freq = IBRO_FREQ;
            break;
        case MXC_SYS_CLOCK_ISO:
            clk_freq = ISO_FREQ;
            break;
        case MXC_SYS_CLOCK_INRO:
            clk_freq = INRO_FREQ;
            break;
        case MXC_SYS_CLOCK_ERTCO:
            clk_freq = ERTCO_FREQ;
            break;
        case MXC_SYS_CLOCK_EXTCLK:
            clk_freq = EXTCLK_FREQ;
            break;
        default:
            return E_BAD_PARAM;
        }
    } else {
        return E_BAD_PARAM;
    }
    return (clk_freq / 2);
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
    return MXC_SPI_RevA1_SetWidth((mxc_spi_reva_regs_t *)spi, (mxc_spi_reva_width_t)spiWidth);
}

mxc_spi_width_t MXC_SPI_GetWidth(mxc_spi_regs_t *spi)
{
    return (mxc_spi_width_t)MXC_SPI_RevA1_GetWidth((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_SetMode(mxc_spi_regs_t *spi, mxc_spi_mode_t spiMode)
{
    return MXC_SPI_RevA1_SetMode((mxc_spi_reva_regs_t *)spi, (mxc_spi_reva_mode_t)spiMode);
}

mxc_spi_mode_t MXC_SPI_GetMode(mxc_spi_regs_t *spi)
{
    return (mxc_spi_mode_t)MXC_SPI_RevA1_GetMode((mxc_spi_reva_regs_t *)spi);
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
        case 2:
            reqselTx = MXC_DMA_REQUEST_SPI2TX;
            break;
        case 3:
            reqselTx = MXC_DMA_REQUEST_SPI3TX;
            break;
        case 4:
            reqselTx = MXC_DMA_REQUEST_SPI4TX;
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
        case 2:
            reqselRx = MXC_DMA_REQUEST_SPI2RX;
            break;
        case 3:
            reqselRx = MXC_DMA_REQUEST_SPI3RX;
            break;
        case 4:
            reqselRx = MXC_DMA_REQUEST_SPI4RX;
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
            reqselRx = MXC_DMA_REQUEST_SPI0TX;
            break;
        case 1:
            reqselRx = MXC_DMA_REQUEST_SPI1TX;
            break;
        case 2:
            reqselTx = MXC_DMA_REQUEST_SPI2TX;
            break;
        case 3:
            reqselTx = MXC_DMA_REQUEST_SPI3TX;
            break;
        case 4:
            reqselTx = MXC_DMA_REQUEST_SPI4TX;
            break;
        default:
            return E_BAD_PARAM;
            break;
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
        case 2:
            reqselTx = MXC_DMA_REQUEST_SPI2RX;
            break;
        case 3:
            reqselTx = MXC_DMA_REQUEST_SPI3RX;
            break;
        case 4:
            reqselTx = MXC_DMA_REQUEST_SPI4RX;
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
