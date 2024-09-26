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

#include "uart.h"
#include "mxc_device.h"
#include "mxc_pins.h"
#include "mxc_assert.h"
#include "uart_revb.h"
#include "uart_common.h"
#include "lpgcr_regs.h"
#include "dma.h"

void MXC_UART_DMACallback(int ch, int error)
{
    MXC_UART_RevB_DMACallback(ch, error);
}

int MXC_UART_AsyncCallback(mxc_uart_regs_t *uart, int retVal)
{
    return MXC_UART_RevB_AsyncCallback((mxc_uart_revb_regs_t *)uart, retVal);
}

int MXC_UART_AsyncStop(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevB_AsyncStop((mxc_uart_revb_regs_t *)uart);
}

int MXC_UART_Init(mxc_uart_regs_t *uart, unsigned int baud, mxc_uart_clock_t clock)
{
    int retval;

#ifndef MSDK_NO_GPIO_CLK_INIT
    retval = MXC_UART_Shutdown(uart);
    if (retval) {
        return retval;
    }

    switch (MXC_UART_GET_IDX(uart)) {
    case 0:
        MXC_GPIO_Config(&gpio_cfg_uart0);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_UART0);
        break;

    case 1:
        MXC_GPIO_Config(&gpio_cfg_uart1);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_UART1);
        break;

    case 2:
        MXC_GPIO_Config(&gpio_cfg_uart2);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_UART2);
        break;

    case 3:
        MXC_GPIO_Config(&gpio_cfg_uart3);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_UART3);
        break;

    default:
        return E_BAD_PARAM;
    }
#endif // MSDK_NO_GPIO_CLK_INIT

    retval = MXC_UART_SetClockSource(uart, clock);
    if (retval != E_NO_ERROR) {
        return retval;
    }
    return MXC_UART_RevB_Init((mxc_uart_revb_regs_t *)uart, baud, MXC_UART_GetClockSource(uart));
}

int MXC_UART_Shutdown(mxc_uart_regs_t *uart)
{
    switch (MXC_UART_GET_IDX(uart)) {
    case 0:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_UART0);
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_UART0);
        break;

    case 1:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_UART1);
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_UART1);
        break;

    case 2:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_UART2);
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_UART2);
        break;

    case 3:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_UART3);
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_UART3);
        break;

    default:
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

int MXC_UART_ReadyForSleep(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevB_ReadyForSleep((mxc_uart_revb_regs_t *)uart);
}

int MXC_UART_SetFrequency(mxc_uart_regs_t *uart, unsigned int baud, mxc_uart_clock_t clock)
{
    int freq;
    uint32_t clock_freq = 0;

    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    // Default OSR
    //  Setting LPUART Over-Sampling Rate in MXC_UART_RevB_SetFrequency function overwrites
    //  the sampling rate set below for the ERTCO.
    uart->osr = 5;

    switch (clock) {
    case MXC_UART_APB_CLK:
        clock_freq = SystemCoreClock / 2;
        break;

    case MXC_UART_IBRO_CLK:
        clock_freq = IBRO_FREQ;
        break;

    case MXC_UART_ERTCO_CLK:
        // Only UART3 (LPUART0) supports ERTCO clock source.
        if (uart != MXC_UART3) {
            return E_BAD_PARAM;
        }

        uart->ctrl |= MXC_F_UART_CTRL_FDM;

        if (baud > 2400) {
            uart->osr = 0;
        } else {
            uart->osr = 1;
        }

        clock_freq = ERTCO_FREQ * 2; // x2 to account for FDM.
        break;

    default:
        return E_BAD_PARAM;
    }

    freq = MXC_UART_RevB_SetFrequency((mxc_uart_revb_regs_t *)uart, clock_freq, baud);

    if (freq > 0) {
        // Enable baud clock and wait for it to become ready.
        uart->ctrl |= MXC_F_UART_CTRL_BCLKEN;
        while (((uart->ctrl & MXC_F_UART_CTRL_BCLKRDY) >> MXC_F_UART_CTRL_BCLKRDY_POS) == 0) {}
    }

    return freq;
}

int MXC_UART_GetFrequency(mxc_uart_regs_t *uart)
{
    int periphClock = 0;

    if (MXC_UART_GET_IDX(uart) < 0) {
        return E_BAD_PARAM;
    }

    // check if UART is LP UART
    if (uart == MXC_UART3) {
        if ((uart->ctrl & MXC_F_UART_CTRL_BCLKSRC) ==
            MXC_S_UART_REVB_CTRL_BCLKSRC_PERIPHERAL_CLOCK) {
            periphClock = IBRO_FREQ;
        } else if ((uart->ctrl & MXC_F_UART_CTRL_BCLKSRC) == MXC_S_UART_REVB_CTRL_BCLKSRC_CLK1) {
            periphClock = ERTCO_FREQ * 2;
        } else {
            return E_NOT_SUPPORTED;
        }
        return (periphClock / uart->clkdiv);
    } else {
        return MXC_UART_RevB_GetFrequency((mxc_uart_revb_regs_t *)uart);
    }
}

int MXC_UART_SetDataSize(mxc_uart_regs_t *uart, int dataSize)
{
    return MXC_UART_RevB_SetDataSize((mxc_uart_revb_regs_t *)uart, dataSize);
}

int MXC_UART_SetStopBits(mxc_uart_regs_t *uart, mxc_uart_stop_t stopBits)
{
    return MXC_UART_RevB_SetStopBits((mxc_uart_revb_regs_t *)uart, stopBits);
}

int MXC_UART_SetParity(mxc_uart_regs_t *uart, mxc_uart_parity_t parity)
{
    return MXC_UART_RevB_SetParity((mxc_uart_revb_regs_t *)uart, parity);
}

int MXC_UART_SetFlowCtrl(mxc_uart_regs_t *uart, mxc_uart_flow_t flowCtrl, int rtsThreshold)
{
    if (flowCtrl == MXC_UART_FLOW_EN) {
        switch (MXC_UART_GET_IDX(uart)) {
        case 0:
            MXC_GPIO_Config(&gpio_cfg_uart0_flow);
            break;

        case 1:
            MXC_GPIO_Config(&gpio_cfg_uart1_flow);
            break;

        case 2:
            MXC_GPIO_Config(&gpio_cfg_uart2_flow);
            break;

        default:
            return E_BAD_PARAM;
        }
    } else {
        switch (MXC_UART_GET_IDX(uart)) {
        case 0:
            MXC_GPIO_Config(&gpio_cfg_uart0_flow_disable);
            break;

        case 1:
            MXC_GPIO_Config(&gpio_cfg_uart1_flow_disable);
            break;

        case 2:
            MXC_GPIO_Config(&gpio_cfg_uart2_flow_disable);
            break;

        default:
            return E_BAD_PARAM;
        }
    }

    return MXC_UART_RevB_SetFlowCtrl((mxc_uart_revb_regs_t *)uart, flowCtrl, rtsThreshold);
}

int MXC_UART_SetClockSource(mxc_uart_regs_t *uart, mxc_uart_clock_t clock)
{
    int error = E_NO_ERROR;

    switch (MXC_UART_GET_IDX(uart)) {
    case 0:
    case 1:
    case 2:
        // UART0-2 support PCLK and IBRO
        switch (clock) {
        case MXC_UART_APB_CLK:
            MXC_UART_RevB_SetClockSource((mxc_uart_revb_regs_t *)uart, 0);
            break;

        case MXC_UART_IBRO_CLK:
#ifndef MSDK_NO_GPIO_CLK_INIT
            error = MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_IBRO);
#endif // MSDK_NO_GPIO_CLK_INIT
            MXC_UART_RevB_SetClockSource((mxc_uart_revb_regs_t *)uart, 2);
            break;

        default:
            return E_BAD_PARAM;
        }
        break;

    case 3:
        // UART3 (LPUART0) supports IBRO and ERTCO
        switch (clock) {
        case MXC_UART_IBRO_CLK:
#ifndef MSDK_NO_GPIO_CLK_INIT
            error = MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_IBRO);
#endif // MSDK_NO_GPIO_CLK_INIT
            MXC_UART_RevB_SetClockSource((mxc_uart_revb_regs_t *)uart, 0);
            break;

        case MXC_UART_ERTCO_CLK:
#ifndef MSDK_NO_GPIO_CLK_INIT
            error = MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_ERTCO);
#endif // MSDK_NO_GPIO_CLK_INIT
            MXC_UART_RevB_SetClockSource((mxc_uart_revb_regs_t *)uart, 1);
            break;

        default:
            return E_BAD_PARAM;
        }
        break;

    default:
        return E_BAD_PARAM;
    }

    return error;
}

mxc_uart_clock_t MXC_UART_GetClockSource(mxc_uart_regs_t *uart)
{
    unsigned int clock_option = MXC_UART_RevB_GetClockSource((mxc_uart_revb_regs_t *)uart);
    switch (MXC_UART_GET_IDX(uart)) {
    case 0:
    case 1:
    case 2:
        switch (clock_option) {
        case 0:
            return MXC_UART_APB_CLK;
        case 2:
            return MXC_UART_IBRO_CLK;
        default:
            return E_BAD_STATE;
        }
        break;
    case 3:
        switch (clock_option) {
        case 0:
            return MXC_UART_IBRO_CLK;
        case 1:
            return MXC_UART_ERTCO_CLK;
        default:
            return E_BAD_STATE;
        }
        break;
    default:
        return E_BAD_PARAM;
    }
}

int MXC_UART_GetActive(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevB_GetActive((mxc_uart_revb_regs_t *)uart);
}

int MXC_UART_AbortTransmission(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevB_AbortTransmission((mxc_uart_revb_regs_t *)uart);
}

int MXC_UART_ReadCharacterRaw(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevB_ReadCharacterRaw((mxc_uart_revb_regs_t *)uart);
}

int MXC_UART_WriteCharacterRaw(mxc_uart_regs_t *uart, uint8_t character)
{
    return MXC_UART_RevB_WriteCharacterRaw((mxc_uart_revb_regs_t *)uart, character);
}

int MXC_UART_ReadCharacter(mxc_uart_regs_t *uart)
{
    return MXC_UART_Common_ReadCharacter(uart);
}

int MXC_UART_WriteCharacter(mxc_uart_regs_t *uart, uint8_t character)
{
    return MXC_UART_Common_WriteCharacter(uart, character);
}

int MXC_UART_Read(mxc_uart_regs_t *uart, uint8_t *buffer, int *len)
{
    return MXC_UART_RevB_Read((mxc_uart_revb_regs_t *)uart, buffer, len);
}

int MXC_UART_Write(mxc_uart_regs_t *uart, const uint8_t *byte, int *len)
{
    return MXC_UART_RevB_Write((mxc_uart_revb_regs_t *)uart, byte, len);
}

unsigned int MXC_UART_ReadRXFIFO(mxc_uart_regs_t *uart, unsigned char *bytes, unsigned int len)
{
    return MXC_UART_RevB_ReadRXFIFO((mxc_uart_revb_regs_t *)uart, bytes, len);
}

int MXC_UART_ReadRXFIFODMA(mxc_uart_regs_t *uart, unsigned char *bytes, unsigned int len,
                           mxc_uart_dma_complete_cb_t callback)
{
    mxc_dma_config_t config;

    int uart_num = MXC_UART_GET_IDX(uart);

    switch (uart_num) {
    case 0:
        config.reqsel = MXC_DMA_REQUEST_UART0RX;
        break;

    case 1:
        config.reqsel = MXC_DMA_REQUEST_UART1RX;
        break;

    case 2:
        config.reqsel = MXC_DMA_REQUEST_UART2RX;
        break;

    case 3:
        config.reqsel = MXC_DMA_REQUEST_UART3RX;
        break;

    default:
        return E_BAD_PARAM;
        break;
    }

    return MXC_UART_RevB_ReadRXFIFODMA((mxc_uart_revb_regs_t *)uart, MXC_DMA, bytes, len, callback,
                                       config);
}

unsigned int MXC_UART_GetRXFIFOAvailable(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevB_GetRXFIFOAvailable((mxc_uart_revb_regs_t *)uart);
}

unsigned int MXC_UART_WriteTXFIFO(mxc_uart_regs_t *uart, const unsigned char *bytes,
                                  unsigned int len)
{
    return MXC_UART_RevB_WriteTXFIFO((mxc_uart_revb_regs_t *)uart, bytes, len);
}

int MXC_UART_WriteTXFIFODMA(mxc_uart_regs_t *uart, const unsigned char *bytes, unsigned int len,
                            mxc_uart_dma_complete_cb_t callback)
{
    mxc_dma_config_t config;

    int uart_num = MXC_UART_GET_IDX(uart);
    switch (uart_num) {
    case 0:
        config.reqsel = MXC_DMA_REQUEST_UART0TX;
        break;

    case 1:
        config.reqsel = MXC_DMA_REQUEST_UART1TX;
        break;

    case 2:
        config.reqsel = MXC_DMA_REQUEST_UART2TX;
        break;

    case 3:
        config.reqsel = MXC_DMA_REQUEST_UART3TX;
        break;

    default:
        return E_BAD_PARAM;
        break;
    }

    return MXC_UART_RevB_WriteTXFIFODMA((mxc_uart_revb_regs_t *)uart, MXC_DMA, bytes, len, callback,
                                        config);
}

unsigned int MXC_UART_GetTXFIFOAvailable(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevB_GetTXFIFOAvailable((mxc_uart_revb_regs_t *)uart);
}

int MXC_UART_ClearRXFIFO(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevB_ClearRXFIFO((mxc_uart_revb_regs_t *)uart);
}

int MXC_UART_ClearTXFIFO(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevB_ClearTXFIFO((mxc_uart_revb_regs_t *)uart);
}

int MXC_UART_SetRXThreshold(mxc_uart_regs_t *uart, unsigned int numBytes)
{
    return MXC_UART_RevB_SetRXThreshold((mxc_uart_revb_regs_t *)uart, numBytes);
}

unsigned int MXC_UART_GetRXThreshold(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevB_GetRXThreshold((mxc_uart_revb_regs_t *)uart);
}

int MXC_UART_SetTXThreshold(mxc_uart_regs_t *uart, unsigned int numBytes)
{
    return E_NOT_SUPPORTED;
}

unsigned int MXC_UART_GetTXThreshold(mxc_uart_regs_t *uart)
{
    return E_NOT_SUPPORTED;
}

unsigned int MXC_UART_GetFlags(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevB_GetFlags((mxc_uart_revb_regs_t *)uart);
}

int MXC_UART_ClearFlags(mxc_uart_regs_t *uart, unsigned int flags)
{
    return MXC_UART_RevB_ClearFlags((mxc_uart_revb_regs_t *)uart, flags);
}

int MXC_UART_EnableInt(mxc_uart_regs_t *uart, unsigned int intEn)
{
    return MXC_UART_RevB_EnableInt((mxc_uart_revb_regs_t *)uart, intEn);
}

int MXC_UART_DisableInt(mxc_uart_regs_t *uart, unsigned int intDis)
{
    return MXC_UART_RevB_DisableInt((mxc_uart_revb_regs_t *)uart, intDis);
}

unsigned int MXC_UART_GetStatus(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevB_GetStatus((mxc_uart_revb_regs_t *)uart);
}

int MXC_UART_Transaction(mxc_uart_req_t *req)
{
    return MXC_UART_RevB_Transaction((mxc_uart_revb_req_t *)req);
}

int MXC_UART_TransactionAsync(mxc_uart_req_t *req)
{
    return MXC_UART_RevB_TransactionAsync((mxc_uart_revb_req_t *)req);
}

int MXC_UART_TransactionDMA(mxc_uart_req_t *req)
{
    return MXC_UART_RevB_TransactionDMA((mxc_uart_revb_req_t *)req, MXC_DMA);
}

int MXC_UART_AbortAsync(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevB_AbortAsync((mxc_uart_revb_regs_t *)uart);
}

int MXC_UART_AsyncHandler(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevB_AsyncHandler((mxc_uart_revb_regs_t *)uart);
}

uint32_t MXC_UART_GetAsyncTXCount(mxc_uart_req_t *req)
{
    return req->txCnt;
}

uint32_t MXC_UART_GetAsyncRXCount(mxc_uart_req_t *req)
{
    return req->rxCnt;
}

int MXC_UART_SetAutoDMAHandlers(mxc_uart_regs_t *uart, bool enable)
{
    return MXC_UART_RevB_SetAutoDMAHandlers((mxc_uart_revb_regs_t *)uart, enable);
}

int MXC_UART_SetTXDMAChannel(mxc_uart_regs_t *uart, unsigned int channel)
{
    return MXC_UART_RevB_SetTXDMAChannel((mxc_uart_revb_regs_t *)uart, channel);
}

int MXC_UART_GetTXDMAChannel(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevB_GetTXDMAChannel((mxc_uart_revb_regs_t *)uart);
}

int MXC_UART_SetRXDMAChannel(mxc_uart_regs_t *uart, unsigned int channel)
{
    return MXC_UART_RevB_SetRXDMAChannel((mxc_uart_revb_regs_t *)uart, channel);
}

int MXC_UART_GetRXDMAChannel(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevB_GetTXDMAChannel((mxc_uart_revb_regs_t *)uart);
}
