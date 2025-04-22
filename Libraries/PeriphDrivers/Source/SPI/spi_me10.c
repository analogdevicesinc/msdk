/**
 * @file       spi.c
 * @brief      This file contains the function implementations for the
 *             Serial Peripheral Interface peripheral module.
 */

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

/* **** Includes **** */
#include <string.h>
#include "spi.h"
#include "spi_reva1.h"
#include "mxc_sys.h"
#include "mxc_errors.h"

/**
 * @ingroup spi
 * @{
 */

/* **** Definitions **** */

/* **** Functions **** */

int MXC_SPI_Init(mxc_spi_regs_t *spi, int masterMode, int quadModeUsed, int numSlaves,
                 unsigned ssPolarity, unsigned int hz)
{
    if (numSlaves > MXC_SPI_SS_INSTANCES) {
        return E_BAD_PARAM;
    }

    // Check if frequency is too high
    if (hz > PeripheralClock) {
        return E_BAD_PARAM;
    }

#ifndef MSDK_NO_GPIO_CLK_INIT
    switch (MXC_SPI_GET_IDX(spi)) {
    case 0:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_SPI0);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI0);
        MXC_GPIO_Config(&gpio_cfg_spi0_0);
        MXC_GPIO_Config(&gpio_cfg_spi0_1);
        break;
    case 1:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_SPI1);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI1);
        MXC_GPIO_Config(&gpio_cfg_spi1);
        break;
    case 2:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_SPI2);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI2);
        MXC_GPIO_Config(&gpio_cfg_spi2);
        break;
    case 3:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_SPI3);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI3);
        MXC_GPIO_Config(&gpio_cfg_spi3);
        break;
    default:
        return E_BAD_PARAM;
    }
#endif // MSDK_NO_GPIO_CLK_INIT

    return MXC_SPI_RevA1_Init((mxc_spi_reva_regs_t *)spi, masterMode, quadModeUsed, numSlaves,
                              ssPolarity, hz);
}

/* ************************************************************************ */
int MXC_SPI_Shutdown(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevA1_Shutdown((mxc_spi_reva_regs_t *)spi);

    switch (MXC_SPI_GET_IDX(spi)) {
    case 0:
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI0);
        break;
    case 1:
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI1);
        break;
    case 2:
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI2);
        break;
    case 3:
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI3);
        break;
    default:
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

/* ************************************************************************ */
int MXC_SPI_ReadyForSleep(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_ReadyForSleep((mxc_spi_reva_regs_t *)spi);
}

/* ************************************************************************ */
int MXC_SPI_GetPeripheralClock(mxc_spi_regs_t *spi)
{
    if (MXC_SPI_GET_IDX(spi) == 3) {
        return PeripheralClock * 2;
    } else if (MXC_SPI_GET_IDX(spi) != -1) {
        return PeripheralClock;
    } else {
        return E_BAD_PARAM;
    }
    return E_NO_ERROR;
}

/* ************************************************************************ */
int MXC_SPI_SetFrequency(mxc_spi_regs_t *spi, unsigned int hz)
{
    return MXC_SPI_RevA1_SetFrequency((mxc_spi_reva_regs_t *)spi, hz);
}

/* ************************************************************************ */
unsigned int MXC_SPI_GetFrequency(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_GetFrequency((mxc_spi_reva_regs_t *)spi);
}

/* ************************************************************************ */
int MXC_SPI_SetDataSize(mxc_spi_regs_t *spi, int dataSize)
{
    return MXC_SPI_RevA1_SetDataSize((mxc_spi_reva_regs_t *)spi, dataSize);
}

/* ************************************************************************ */
int MXC_SPI_GetDataSize(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_GetDataSize((mxc_spi_reva_regs_t *)spi);
}

/* ************************************************************************* */
/* Low-level functions                                                       */
/* ************************************************************************* */

int MXC_SPI_SetSlave(mxc_spi_regs_t *spi, int ssIdx)
{
    return MXC_SPI_RevA1_SetSlave((mxc_spi_reva_regs_t *)spi, ssIdx);
}

/* ************************************************************************ */
int MXC_SPI_GetSlave(mxc_spi_regs_t *spi)
{
    int spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    int slvSel = (spi->ctrl0 & MXC_F_SPI_CTRL0_SS_SEL) >> MXC_F_SPI_CTRL0_SS_SEL_POS;

    if (slvSel &
        (MXC_V_SPI_CTRL0_SS_SEL_SS0 | MXC_V_SPI_CTRL0_SS_SEL_SS1 | MXC_V_SPI_CTRL0_SS_SEL_SS2)) {
        return slvSel >> 1;
    } else {
        return 3;
    }
}

/* ************************************************************************ */
int MXC_SPI_SetWidth(mxc_spi_regs_t *spi, mxc_spi_width_t spiWidth)
{
    return MXC_SPI_RevA1_SetWidth((mxc_spi_reva_regs_t *)spi, spiWidth);
}

/* ************************************************************************ */
mxc_spi_width_t MXC_SPI_GetWidth(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_GetWidth((mxc_spi_reva_regs_t *)spi);
}

/* ************************************************************************ */
int MXC_SPI_SetMode(mxc_spi_regs_t *spi, mxc_spi_mode_t spiMode)
{
    return MXC_SPI_RevA1_SetMode((mxc_spi_reva_regs_t *)spi, (mxc_spi_reva_mode_t)spiMode);
}

/* ************************************************************************ */
mxc_spi_mode_t MXC_SPI_GetMode(mxc_spi_regs_t *spi)
{
    return (mxc_spi_mode_t)MXC_SPI_RevA1_GetMode((mxc_spi_reva_regs_t *)spi);
}

/* ************************************************************************ */
int MXC_SPI_StartTransmission(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_StartTransmission((mxc_spi_reva_regs_t *)spi);
}

/* ************************************************************************ */
int MXC_SPI_GetActive(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_GetActive((mxc_spi_reva_regs_t *)spi);
}

/* ************************************************************************ */
int MXC_SPI_AbortTransmission(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_AbortTransmission((mxc_spi_reva_regs_t *)spi);
}

/* ************************************************************************ */
unsigned int MXC_SPI_ReadRXFIFO(mxc_spi_regs_t *spi, unsigned char *bytes, unsigned int len)
{
    return MXC_SPI_RevA1_ReadRXFIFO((mxc_spi_reva_regs_t *)spi, bytes, len);
}

/* ************************************************************************ */
unsigned int MXC_SPI_GetRXFIFOAvailable(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_GetRXFIFOAvailable((mxc_spi_reva_regs_t *)spi);
}

/* ************************************************************************ */
unsigned int MXC_SPI_WriteTXFIFO(mxc_spi_regs_t *spi, unsigned char *bytes, unsigned int len)
{
    return MXC_SPI_RevA1_WriteTXFIFO((mxc_spi_reva_regs_t *)spi, bytes, len);
}

/* ************************************************************************ */
unsigned int MXC_SPI_GetTXFIFOAvailable(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_GetTXFIFOAvailable((mxc_spi_reva_regs_t *)spi);
}

/* ************************************************************************ */
void MXC_SPI_ClearRXFIFO(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevA1_ClearRXFIFO((mxc_spi_reva_regs_t *)spi);
}

/* ************************************************************************ */
void MXC_SPI_ClearTXFIFO(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevA1_ClearTXFIFO((mxc_spi_reva_regs_t *)spi);
}

/* ************************************************************************ */
int MXC_SPI_SetRXThreshold(mxc_spi_regs_t *spi, unsigned int numBytes)
{
    return MXC_SPI_RevA1_SetRXThreshold((mxc_spi_reva_regs_t *)spi, numBytes);
}

/* ************************************************************************ */
unsigned int MXC_SPI_GetRXThreshold(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_GetRXThreshold((mxc_spi_reva_regs_t *)spi);
}

/* ************************************************************************ */
int MXC_SPI_SetTXThreshold(mxc_spi_regs_t *spi, unsigned int numBytes)
{
    return MXC_SPI_RevA1_SetTXThreshold((mxc_spi_reva_regs_t *)spi, numBytes);
}

/* ************************************************************************ */
unsigned int MXC_SPI_GetTXThreshold(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_GetTXThreshold((mxc_spi_reva_regs_t *)spi);
}

/* ************************************************************************ */
unsigned int MXC_SPI_GetFlags(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA1_GetFlags((mxc_spi_reva_regs_t *)spi);
}

/* ************************************************************************ */
void MXC_SPI_ClearFlags(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevA1_ClearFlags((mxc_spi_reva_regs_t *)spi);
}

/* ************************************************************************ */
void MXC_SPI_EnableInt(mxc_spi_regs_t *spi, unsigned int mask)
{
    MXC_SPI_RevA1_EnableInt((mxc_spi_reva_regs_t *)spi, mask);
}

/* ************************************************************************ */
void MXC_SPI_DisableInt(mxc_spi_regs_t *spi, unsigned int mask)
{
    MXC_SPI_RevA1_DisableInt((mxc_spi_reva_regs_t *)spi, mask);
}

/* ************************************************************************* */
/* Transaction level functions                                               */
/* ************************************************************************* */

int MXC_SPI_MasterTransaction(mxc_spi_req_t *req)
{
    return MXC_SPI_RevA1_MasterTransaction((mxc_spi_reva_req_t *)req);
}

/* ************************************************************************ */
int MXC_SPI_MasterTransactionAsync(mxc_spi_req_t *req)
{
    return MXC_SPI_RevA1_MasterTransactionAsync((mxc_spi_reva_req_t *)req);
}

/* ************************************************************************ */
int MXC_SPI_MasterTransactionDMA(mxc_spi_req_t *req)
{
    int retVal = E_NO_ERROR;

    switch (MXC_SPI_GET_IDX(req->spi)) {
    case 0:
        retVal = MXC_SPI_RevA1_MasterTransactionDMA(
            (mxc_spi_reva_req_t *)req, MXC_DMA_REQUEST_SPI0TX, MXC_DMA_REQUEST_SPI0RX, MXC_DMA);
        break;
    case 1:
        retVal = MXC_SPI_RevA1_MasterTransactionDMA(
            (mxc_spi_reva_req_t *)req, MXC_DMA_REQUEST_SPI1TX, MXC_DMA_REQUEST_SPI1RX, MXC_DMA);
        break;
    case 2:
        retVal = MXC_SPI_RevA1_MasterTransactionDMA(
            (mxc_spi_reva_req_t *)req, MXC_DMA_REQUEST_SPI2TX, MXC_DMA_REQUEST_SPI2RX, MXC_DMA);
        break;
    case 3:
        retVal = MXC_SPI_RevA1_MasterTransactionDMA(
            (mxc_spi_reva_req_t *)req, MXC_DMA_REQUEST_SPI3TX, MXC_DMA_REQUEST_SPI3RX, MXC_DMA);
        break;
    default:
        return E_BAD_PARAM;
    }

    return retVal;
}

/* ************************************************************************ */
int MXC_SPI_SlaveTransaction(mxc_spi_req_t *req)
{
    return MXC_SPI_RevA1_SlaveTransaction((mxc_spi_reva_req_t *)req);
}

/* ************************************************************************ */
int MXC_SPI_SlaveTransactionAsync(mxc_spi_req_t *req)
{
    return MXC_SPI_RevA1_SlaveTransactionAsync((mxc_spi_reva_req_t *)req);
}

/* ************************************************************************ */
int MXC_SPI_SlaveTransactionDMA(mxc_spi_req_t *req)
{
    int retVal = E_NO_ERROR;

    switch (MXC_SPI_GET_IDX(req->spi)) {
    case 0:
        retVal = MXC_SPI_RevA1_SlaveTransactionDMA(
            (mxc_spi_reva_req_t *)req, MXC_DMA_REQUEST_SPI0TX, MXC_DMA_REQUEST_SPI0RX, MXC_DMA);
        break;
    case 1:
        retVal = MXC_SPI_RevA1_SlaveTransactionDMA(
            (mxc_spi_reva_req_t *)req, MXC_DMA_REQUEST_SPI1TX, MXC_DMA_REQUEST_SPI1RX, MXC_DMA);
        break;
    case 2:
        retVal = MXC_SPI_RevA1_SlaveTransactionDMA(
            (mxc_spi_reva_req_t *)req, MXC_DMA_REQUEST_SPI2TX, MXC_DMA_REQUEST_SPI2RX, MXC_DMA);
        break;
    case 3:
        retVal = MXC_SPI_RevA1_SlaveTransactionDMA(
            (mxc_spi_reva_req_t *)req, MXC_DMA_REQUEST_SPI3TX, MXC_DMA_REQUEST_SPI3RX, MXC_DMA);
        break;
    default:
        return E_BAD_PARAM;
    }

    return retVal;
}

/* ************************************************************************ */
int MXC_SPI_SetDefaultTXData(mxc_spi_regs_t *spi, unsigned int defaultTXData)
{
    return MXC_SPI_RevA1_SetDefaultTXData((mxc_spi_reva_regs_t *)spi, defaultTXData);
}

/* ************************************************************************ */
void MXC_SPI_AbortAsync(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevA1_AbortAsync((mxc_spi_reva_regs_t *)spi);
}

/* ************************************************************************ */
void MXC_SPI_AsyncHandler(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevA1_AsyncHandler((mxc_spi_reva_regs_t *)spi);
}

void MXC_SPI_HWSSControl(mxc_spi_regs_t *spi, int state)
{
    MXC_SPI_RevA1_HWSSControl((mxc_spi_reva_regs_t *)spi, state);
}

/**@} end of group spi */
