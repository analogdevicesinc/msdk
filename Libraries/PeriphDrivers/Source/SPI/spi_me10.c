/**
 * @file       spi.c
 * @brief      This file contains the function implementations for the
 *             Serial Peripheral Interface peripheral module.
 */

/* *****************************************************************************
 * Copyright (C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
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
 * $Date: 2017-06-08 11:25:01 -0500 (Thu, 08 Jun 2017) $
 * $Revision: 28447 $
 *
 **************************************************************************** */

/* **** Includes **** */
#include "spi.h"
#include "spi_reva.h"
#include "mxc_sys.h"
#include "mxc_errors.h"
#include <string.h>

/**
 * @ingroup spi
 * @{
 */

/* **** Definitions **** */

/* **** Functions **** */

int MXC_SPI_Init(mxc_spi_regs_t* spi, int masterMode, int quadModeUsed, int numSlaves,
                 unsigned ssPolarity, unsigned int hz)
{
    if (numSlaves > MXC_SPI_SS_INSTANCES) {
        return E_BAD_PARAM;
    }

    // Check if frequency is too high
    if (hz > PeripheralClock) {
        return E_BAD_PARAM;
    }

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

    return MXC_SPI_RevA_Init((mxc_spi_reva_regs_t*)spi, masterMode, quadModeUsed, numSlaves,
                             ssPolarity, hz);
}

/* ************************************************************************ */
int MXC_SPI_Shutdown(mxc_spi_regs_t* spi)
{
    MXC_SPI_RevA_Shutdown((mxc_spi_reva_regs_t*)spi);

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
int MXC_SPI_ReadyForSleep(mxc_spi_regs_t* spi)
{
    return MXC_SPI_RevA_ReadyForSleep((mxc_spi_reva_regs_t*)spi);
}

/* ************************************************************************ */
int MXC_SPI_GetPeripheralClock(mxc_spi_regs_t* spi)
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
int MXC_SPI_SetFrequency(mxc_spi_regs_t* spi, unsigned int hz)
{
    return MXC_SPI_RevA_SetFrequency((mxc_spi_reva_regs_t*)spi, hz);
}

/* ************************************************************************ */
unsigned int MXC_SPI_GetFrequency(mxc_spi_regs_t* spi)
{
    return MXC_SPI_RevA_GetFrequency((mxc_spi_reva_regs_t*)spi);
}

/* ************************************************************************ */
int MXC_SPI_SetDataSize(mxc_spi_regs_t* spi, int dataSize)
{
    return MXC_SPI_RevA_SetDataSize((mxc_spi_reva_regs_t*)spi, dataSize);
}

/* ************************************************************************ */
int MXC_SPI_GetDataSize(mxc_spi_regs_t* spi)
{
    return MXC_SPI_RevA_GetDataSize((mxc_spi_reva_regs_t*)spi);
}

/* ************************************************************************* */
/* Low-level functions                                                       */
/* ************************************************************************* */

int MXC_SPI_SetSlave(mxc_spi_regs_t* spi, int ssIdx)
{
    return MXC_SPI_RevA_SetSlave((mxc_spi_reva_regs_t*)spi, ssIdx);
}

/* ************************************************************************ */
int MXC_SPI_GetSlave(mxc_spi_regs_t* spi)
{
    int spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t*)spi);
    MXC_ASSERT(spi_num >= 0);

    int slvSel = (spi->ctrl0 & MXC_F_SPI_CTRL0_SS_SEL) >> MXC_F_SPI_CTRL0_SS_SEL_POS;

    if (slvSel &
        (MXC_V_SPI_CTRL0_SS_SEL_SS0 | MXC_V_SPI_CTRL0_SS_SEL_SS1 | MXC_V_SPI_CTRL0_SS_SEL_SS2)) {
        return slvSel >> 1;
    } else {
        return 3;
    }
}

/* ************************************************************************ */
int MXC_SPI_SetWidth(mxc_spi_regs_t* spi, mxc_spi_width_t spiWidth)
{
    return MXC_SPI_RevA_SetWidth((mxc_spi_reva_regs_t*)spi, spiWidth);
}

/* ************************************************************************ */
mxc_spi_width_t MXC_SPI_GetWidth(mxc_spi_regs_t* spi)
{
    return MXC_SPI_RevA_GetWidth((mxc_spi_reva_regs_t*)spi);
}

/* ************************************************************************ */
int MXC_SPI_SetMode(mxc_spi_regs_t* spi, mxc_spi_mode_t spiMode)
{
    return MXC_SPI_RevA_SetMode((mxc_spi_reva_regs_t*)spi, (mxc_spi_reva_mode_t)spiMode);
}

/* ************************************************************************ */
mxc_spi_mode_t MXC_SPI_GetMode(mxc_spi_regs_t* spi)
{
    return (mxc_spi_mode_t)MXC_SPI_RevA_GetMode((mxc_spi_reva_regs_t*)spi);
}

/* ************************************************************************ */
int MXC_SPI_StartTransmission(mxc_spi_regs_t* spi)
{
    return MXC_SPI_RevA_StartTransmission((mxc_spi_reva_regs_t*)spi);
}

/* ************************************************************************ */
int MXC_SPI_GetActive(mxc_spi_regs_t* spi)
{
    return MXC_SPI_RevA_GetActive((mxc_spi_reva_regs_t*)spi);
}

/* ************************************************************************ */
int MXC_SPI_AbortTransmission(mxc_spi_regs_t* spi)
{
    return MXC_SPI_RevA_AbortTransmission((mxc_spi_reva_regs_t*)spi);
}

/* ************************************************************************ */
unsigned int MXC_SPI_ReadRXFIFO(mxc_spi_regs_t* spi, unsigned char* bytes, unsigned int len)
{
    return MXC_SPI_RevA_ReadRXFIFO((mxc_spi_reva_regs_t*)spi, bytes, len);
}

/* ************************************************************************ */
unsigned int MXC_SPI_GetRXFIFOAvailable(mxc_spi_regs_t* spi)
{
    return MXC_SPI_RevA_GetRXFIFOAvailable((mxc_spi_reva_regs_t*)spi);
}

/* ************************************************************************ */
unsigned int MXC_SPI_WriteTXFIFO(mxc_spi_regs_t* spi, unsigned char* bytes, unsigned int len)
{
    return MXC_SPI_RevA_WriteTXFIFO((mxc_spi_reva_regs_t*)spi, bytes, len);
}

/* ************************************************************************ */
unsigned int MXC_SPI_GetTXFIFOAvailable(mxc_spi_regs_t* spi)
{
    return MXC_SPI_RevA_GetTXFIFOAvailable((mxc_spi_reva_regs_t*)spi);
}

/* ************************************************************************ */
void MXC_SPI_ClearRXFIFO(mxc_spi_regs_t* spi)
{
    MXC_SPI_RevA_ClearRXFIFO((mxc_spi_reva_regs_t*)spi);
}

/* ************************************************************************ */
void MXC_SPI_ClearTXFIFO(mxc_spi_regs_t* spi)
{
    MXC_SPI_RevA_ClearTXFIFO((mxc_spi_reva_regs_t*)spi);
}

/* ************************************************************************ */
int MXC_SPI_SetRXThreshold(mxc_spi_regs_t* spi, unsigned int numBytes)
{
    return MXC_SPI_RevA_SetRXThreshold((mxc_spi_reva_regs_t*)spi, numBytes);
}

/* ************************************************************************ */
unsigned int MXC_SPI_GetRXThreshold(mxc_spi_regs_t* spi)
{
    return MXC_SPI_RevA_GetRXThreshold((mxc_spi_reva_regs_t*)spi);
}

/* ************************************************************************ */
int MXC_SPI_SetTXThreshold(mxc_spi_regs_t* spi, unsigned int numBytes)
{
    return MXC_SPI_RevA_SetTXThreshold((mxc_spi_reva_regs_t*)spi, numBytes);
}

/* ************************************************************************ */
unsigned int MXC_SPI_GetTXThreshold(mxc_spi_regs_t* spi)
{
    return MXC_SPI_RevA_GetTXThreshold((mxc_spi_reva_regs_t*)spi);
}

/* ************************************************************************ */
unsigned int MXC_SPI_GetFlags(mxc_spi_regs_t* spi)
{
    return MXC_SPI_RevA_GetFlags((mxc_spi_reva_regs_t*)spi);
}

/* ************************************************************************ */
void MXC_SPI_ClearFlags(mxc_spi_regs_t* spi)
{
    MXC_SPI_RevA_ClearFlags((mxc_spi_reva_regs_t*)spi);
}

/* ************************************************************************ */
void MXC_SPI_EnableInt(mxc_spi_regs_t* spi, unsigned int mask)
{
    MXC_SPI_RevA_EnableInt((mxc_spi_reva_regs_t*)spi, mask);
}

/* ************************************************************************ */
void MXC_SPI_DisableInt(mxc_spi_regs_t* spi, unsigned int mask)
{
    MXC_SPI_RevA_DisableInt((mxc_spi_reva_regs_t*)spi, mask);
}

/* ************************************************************************* */
/* Transaction level functions                                               */
/* ************************************************************************* */

int MXC_SPI_MasterTransaction(mxc_spi_req_t* req)
{
    return MXC_SPI_RevA_MasterTransaction((mxc_spi_reva_req_t*)req);
}

/* ************************************************************************ */
int MXC_SPI_MasterTransactionAsync(mxc_spi_req_t* req)
{
    return MXC_SPI_RevA_MasterTransactionAsync((mxc_spi_reva_req_t*)req);
}

/* ************************************************************************ */
int MXC_SPI_MasterTransactionDMA(mxc_spi_req_t* req)
{
    int retVal = E_NO_ERROR;

    switch (MXC_SPI_GET_IDX(req->spi)) {
        case 0:
            retVal = MXC_SPI_RevA_MasterTransactionDMA(
                (mxc_spi_reva_req_t*)req, MXC_DMA_REQUEST_SPI0TX, MXC_DMA_REQUEST_SPI0RX, MXC_DMA);
            break;
        case 1:
            retVal = MXC_SPI_RevA_MasterTransactionDMA(
                (mxc_spi_reva_req_t*)req, MXC_DMA_REQUEST_SPI1TX, MXC_DMA_REQUEST_SPI1RX, MXC_DMA);
            break;
        case 2:
            retVal = MXC_SPI_RevA_MasterTransactionDMA(
                (mxc_spi_reva_req_t*)req, MXC_DMA_REQUEST_SPI2TX, MXC_DMA_REQUEST_SPI2RX, MXC_DMA);
            break;
        case 3:
            retVal = MXC_SPI_RevA_MasterTransactionDMA(
                (mxc_spi_reva_req_t*)req, MXC_DMA_REQUEST_SPI3TX, MXC_DMA_REQUEST_SPI3RX, MXC_DMA);
            break;
        default:
            return E_BAD_PARAM;
    }

    return retVal;
}

/* ************************************************************************ */
int MXC_SPI_SlaveTransaction(mxc_spi_req_t* req)
{
    return MXC_SPI_RevA_SlaveTransaction((mxc_spi_reva_req_t*)req);
}

/* ************************************************************************ */
int MXC_SPI_SlaveTransactionAsync(mxc_spi_req_t* req)
{
    return MXC_SPI_RevA_SlaveTransactionAsync((mxc_spi_reva_req_t*)req);
}

/* ************************************************************************ */
int MXC_SPI_SlaveTransactionDMA(mxc_spi_req_t* req)
{
    int retVal = E_NO_ERROR;

    switch (MXC_SPI_GET_IDX(req->spi)) {
        case 0:
            retVal = MXC_SPI_RevA_SlaveTransactionDMA(
                (mxc_spi_reva_req_t*)req, MXC_DMA_REQUEST_SPI0TX, MXC_DMA_REQUEST_SPI0RX, MXC_DMA);
            break;
        case 1:
            retVal = MXC_SPI_RevA_SlaveTransactionDMA(
                (mxc_spi_reva_req_t*)req, MXC_DMA_REQUEST_SPI1TX, MXC_DMA_REQUEST_SPI1RX, MXC_DMA);
            break;
        case 2:
            retVal = MXC_SPI_RevA_SlaveTransactionDMA(
                (mxc_spi_reva_req_t*)req, MXC_DMA_REQUEST_SPI2TX, MXC_DMA_REQUEST_SPI2RX, MXC_DMA);
            break;
        case 3:
            retVal = MXC_SPI_RevA_SlaveTransactionDMA(
                (mxc_spi_reva_req_t*)req, MXC_DMA_REQUEST_SPI3TX, MXC_DMA_REQUEST_SPI3RX, MXC_DMA);
            break;
        default:
            return E_BAD_PARAM;
    }

    return retVal;
}

/* ************************************************************************ */
int MXC_SPI_SetDefaultTXData(mxc_spi_regs_t* spi, unsigned int defaultTXData)
{
    return MXC_SPI_RevA_SetDefaultTXData((mxc_spi_reva_regs_t*)spi, defaultTXData);
}

/* ************************************************************************ */
void MXC_SPI_AbortAsync(mxc_spi_regs_t* spi)
{
    MXC_SPI_RevA_AbortAsync((mxc_spi_reva_regs_t*)spi);
}

/* ************************************************************************ */
void MXC_SPI_AsyncHandler(mxc_spi_regs_t* spi)
{
    MXC_SPI_RevA_AsyncHandler((mxc_spi_reva_regs_t*)spi);
}

/**@} end of group spi */