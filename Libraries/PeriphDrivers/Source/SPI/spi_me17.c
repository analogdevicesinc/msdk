/* ****************************************************************************
 * Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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
 *************************************************************************** */

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "spi_reva.h"
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

#if (TARGET != MAX78000 || TARGET_NUM == 32655)

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

#endif
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
    }
#ifdef MXC_SPI0
    else if (spi == MXC_SPI0) {
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
    }
#endif
    else {
        return E_NO_DEVICE;
    }

    MXC_GPIO_Config(&gpio_cfg_spi);

    return MXC_SPI_RevA_Init((mxc_spi_reva_regs_t *)spi, masterMode, quadModeUsed, numSlaves,
                             ssPolarity, hz);
}

int MXC_SPI_Shutdown(mxc_spi_regs_t *spi)
{
    int spi_num;
    spi_num = MXC_SPI_GET_IDX(spi);
    MXC_ASSERT(spi_num >= 0);
    (void)spi_num;

    MXC_SPI_RevA_Shutdown((mxc_spi_reva_regs_t *)spi);

    if (spi == MXC_SPI1) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPI1);
    }
#ifdef MXC_SPI0
    else if (spi == MXC_SPI0) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPI0);
    }
#endif

    else {
        return E_NO_DEVICE;
    }

    return E_NO_ERROR;
}

int MXC_SPI_ReadyForSleep(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA_ReadyForSleep((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_GetPeripheralClock(mxc_spi_regs_t *spi)
{
    int retval;

    if (spi == MXC_SPI1) {
        retval = PeripheralClock;
    }
#ifdef MXC_SPI0 // SPI0 is not accessible from the RISC core.
    else if (spi == MXC_SPI0) {
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
#if TARGET_NUM == 32655 || TARGET_NUM == 32680
        case MXC_SYS_CLOCK_ERFO:
            retval = ERFO_FREQ;
            break;
#endif
        default:
            return E_BAD_STATE;
        }
    }
#endif // MXC_SPI0
    else {
        return E_BAD_PARAM;
    }

    retval /= 2;

    return retval;
}

int MXC_SPI_SetFrequency(mxc_spi_regs_t *spi, unsigned int hz)
{
    return MXC_SPI_RevA_SetFrequency((mxc_spi_reva_regs_t *)spi, hz);
}

unsigned int MXC_SPI_GetFrequency(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA_GetFrequency((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_SetDataSize(mxc_spi_regs_t *spi, int dataSize)
{
    return MXC_SPI_RevA_SetDataSize((mxc_spi_reva_regs_t *)spi, dataSize);
}

int MXC_SPI_GetDataSize(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA_GetDataSize((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_SetSlave(mxc_spi_regs_t *spi, int ssIdx)
{
    return MXC_SPI_RevA_SetSlave((mxc_spi_reva_regs_t *)spi, ssIdx);
}

int MXC_SPI_GetSlave(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA_GetSlave((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_SetWidth(mxc_spi_regs_t *spi, mxc_spi_width_t spiWidth)
{
    return MXC_SPI_RevA_SetWidth((mxc_spi_reva_regs_t *)spi, spiWidth);
}

mxc_spi_width_t MXC_SPI_GetWidth(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA_GetWidth((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_SetMTMode(mxc_spi_regs_t *spi, int mtMode)
{
    return MXC_SPI_RevA_SetMTMode((mxc_spi_reva_regs_t *)spi, mtMode);
}

int MXC_SPI_GetMTMode(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA_GetMTMode((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_SetMode(mxc_spi_regs_t *spi, mxc_spi_mode_t spiMode)
{
    return MXC_SPI_RevA_SetMode((mxc_spi_reva_regs_t *)spi, spiMode);
}

mxc_spi_mode_t MXC_SPI_GetMode(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA_GetMode((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_StartTransmission(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA_StartTransmission((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_GetActive(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA_GetActive((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_AbortTransmission(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA_AbortTransmission((mxc_spi_reva_regs_t *)spi);
}

unsigned int MXC_SPI_ReadRXFIFO(mxc_spi_regs_t *spi, unsigned char *bytes, unsigned int len)
{
    return MXC_SPI_RevA_ReadRXFIFO((mxc_spi_reva_regs_t *)spi, bytes, len);
}

unsigned int MXC_SPI_GetRXFIFOAvailable(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA_GetRXFIFOAvailable((mxc_spi_reva_regs_t *)spi);
}

unsigned int MXC_SPI_WriteTXFIFO(mxc_spi_regs_t *spi, unsigned char *bytes, unsigned int len)
{
    return MXC_SPI_RevA_WriteTXFIFO((mxc_spi_reva_regs_t *)spi, bytes, len);
}

unsigned int MXC_SPI_GetTXFIFOAvailable(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA_GetTXFIFOAvailable((mxc_spi_reva_regs_t *)spi);
}

void MXC_SPI_ClearRXFIFO(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevA_ClearRXFIFO((mxc_spi_reva_regs_t *)spi);
}

void MXC_SPI_ClearTXFIFO(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevA_ClearTXFIFO((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_SetRXThreshold(mxc_spi_regs_t *spi, unsigned int numBytes)
{
    return MXC_SPI_RevA_SetRXThreshold((mxc_spi_reva_regs_t *)spi, numBytes);
}

unsigned int MXC_SPI_GetRXThreshold(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA_GetRXThreshold((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_SetTXThreshold(mxc_spi_regs_t *spi, unsigned int numBytes)
{
    return MXC_SPI_RevA_SetTXThreshold((mxc_spi_reva_regs_t *)spi, numBytes);
}

unsigned int MXC_SPI_GetTXThreshold(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA_GetTXThreshold((mxc_spi_reva_regs_t *)spi);
}

unsigned int MXC_SPI_GetFlags(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA_GetFlags((mxc_spi_reva_regs_t *)spi);
}

void MXC_SPI_ClearFlags(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevA_ClearFlags((mxc_spi_reva_regs_t *)spi);
}

void MXC_SPI_EnableInt(mxc_spi_regs_t *spi, unsigned int intEn)
{
    MXC_SPI_RevA_EnableInt((mxc_spi_reva_regs_t *)spi, intEn);
}

void MXC_SPI_DisableInt(mxc_spi_regs_t *spi, unsigned int intDis)
{
    MXC_SPI_RevA_DisableInt((mxc_spi_reva_regs_t *)spi, intDis);
}

int MXC_SPI_MasterTransaction(mxc_spi_req_t *req)
{
    return MXC_SPI_RevA_MasterTransaction((mxc_spi_reva_req_t *)req);
}

int MXC_SPI_MasterTransactionAsync(mxc_spi_req_t *req)
{
    return MXC_SPI_RevA_MasterTransactionAsync((mxc_spi_reva_req_t *)req);
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

    return MXC_SPI_RevA_MasterTransactionDMA((mxc_spi_reva_req_t *)req, reqselTx, reqselRx,
                                             MXC_DMA);
}

int MXC_SPI_SlaveTransaction(mxc_spi_req_t *req)
{
    return MXC_SPI_RevA_SlaveTransaction((mxc_spi_reva_req_t *)req);
}

int MXC_SPI_SlaveTransactionAsync(mxc_spi_req_t *req)
{
    return MXC_SPI_RevA_SlaveTransactionAsync((mxc_spi_reva_req_t *)req);
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

    return MXC_SPI_RevA_SlaveTransactionDMA((mxc_spi_reva_req_t *)req, reqselTx, reqselRx, MXC_DMA);
}

int MXC_SPI_SetDefaultTXData(mxc_spi_regs_t *spi, unsigned int defaultTXData)
{
    return MXC_SPI_RevA_SetDefaultTXData((mxc_spi_reva_regs_t *)spi, defaultTXData);
}

void MXC_SPI_AbortAsync(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevA_AbortAsync((mxc_spi_reva_regs_t *)spi);
}

void MXC_SPI_AsyncHandler(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevA_AsyncHandler((mxc_spi_reva_regs_t *)spi);
}
