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
#include "uart_revc.h"
#include "uart_common.h"
#include "dma.h"

void MXC_UART_DMACallback(int ch, int error)
{
    MXC_UART_RevC_DMACallback(ch, error);
}

int MXC_UART_AsyncCallback(mxc_uart_regs_t *uart, int retVal)
{
    return MXC_UART_RevC_AsyncCallback((mxc_uart_revc_regs_t *)uart, retVal);
}

int MXC_UART_AsyncStop(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevC_AsyncStop((mxc_uart_revc_regs_t *)uart);
}

int MXC_UART_Init(mxc_uart_regs_t *uart, unsigned int baud)
{
#ifndef MSDK_NO_GPIO_CLK_INIT
    int retval;

    retval = MXC_UART_Shutdown(uart);

    if (retval) {
        return retval;
    }

    switch (MXC_UART_GET_IDX(uart)) {
    case 0:
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_UART0);
        MXC_GPIO_Config(&gpio_cfg_uart0);
        break;

    default:
        return E_BAD_PARAM;
        break;
    }
#endif

    return MXC_UART_RevC_Init((mxc_uart_revc_regs_t *)uart, baud);
}

int MXC_UART_Shutdown(mxc_uart_regs_t *uart)
{
    switch (MXC_UART_GET_IDX(uart)) {
    case 0:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_UART0);
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_UART0);
        break;

    default:
        return E_BAD_PARAM;
        break;
    }

    return E_NO_ERROR;
}

int MXC_UART_ReadyForSleep(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevC_ReadyForSleep((mxc_uart_revc_regs_t *)uart);
}

int MXC_UART_SetFrequency(mxc_uart_regs_t *uart, unsigned int baud)
{
    return MXC_UART_RevC_SetFrequency((mxc_uart_revc_regs_t *)uart, baud);
}

int MXC_UART_GetFrequency(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevC_GetFrequency((mxc_uart_revc_regs_t *)uart);
}

int MXC_UART_SetDataSize(mxc_uart_regs_t *uart, int dataSize)
{
    return MXC_UART_RevC_SetDataSize((mxc_uart_revc_regs_t *)uart, dataSize);
}

int MXC_UART_SetStopBits(mxc_uart_regs_t *uart, mxc_uart_stop_t stopBits)
{
    return MXC_UART_RevC_SetStopBits((mxc_uart_revc_regs_t *)uart, stopBits);
}

int MXC_UART_SetParity(mxc_uart_regs_t *uart, mxc_uart_parity_t parity)
{
    return MXC_UART_RevC_SetParity((mxc_uart_revc_regs_t *)uart, parity);
}

int MXC_UART_GetActive(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevC_GetActive((mxc_uart_revc_regs_t *)uart);
}

int MXC_UART_AbortTransmission(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevC_AbortTransmission((mxc_uart_revc_regs_t *)uart);
}

int MXC_UART_ReadCharacterRaw(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevC_ReadCharacterRaw((mxc_uart_revc_regs_t *)uart);
}

int MXC_UART_WriteCharacterRaw(mxc_uart_regs_t *uart, uint8_t character)
{
    return MXC_UART_RevC_WriteCharacterRaw((mxc_uart_revc_regs_t *)uart, character);
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
    return MXC_UART_RevC_Read((mxc_uart_revc_regs_t *)uart, buffer, len);
}

int MXC_UART_Write(mxc_uart_regs_t *uart, uint8_t *byte, int *len)
{
    return MXC_UART_RevC_Write((mxc_uart_revc_regs_t *)uart, byte, len);
}

unsigned int MXC_UART_ReadRXFIFO(mxc_uart_regs_t *uart, unsigned char *bytes, unsigned int len)
{
    return MXC_UART_RevC_ReadRXFIFO((mxc_uart_revc_regs_t *)uart, bytes, len);
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

    default:
        return E_BAD_PARAM;
        break;
    }

    return MXC_UART_RevC_ReadRXFIFODMA((mxc_uart_revc_regs_t *)uart, bytes, len, callback, config);
}

unsigned int MXC_UART_GetRXFIFOAvailable(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevC_GetRXFIFOAvailable((mxc_uart_revc_regs_t *)uart);
}

unsigned int MXC_UART_WriteTXFIFO(mxc_uart_regs_t *uart, unsigned char *bytes, unsigned int len)
{
    return MXC_UART_RevC_WriteTXFIFO((mxc_uart_revc_regs_t *)uart, bytes, len);
}

int MXC_UART_WriteTXFIFODMA(mxc_uart_regs_t *uart, unsigned char *bytes, unsigned int len,
                            mxc_uart_dma_complete_cb_t callback)
{
    mxc_dma_config_t config;

    int uart_num = MXC_UART_GET_IDX(uart);

    switch (uart_num) {
    case 0:
        config.reqsel = MXC_DMA_REQUEST_UART0TX;
        break;

    default:
        return E_BAD_PARAM;
        break;
    }

    return MXC_UART_RevC_WriteTXFIFODMA((mxc_uart_revc_regs_t *)uart, bytes, len, callback, config);
}

unsigned int MXC_UART_GetTXFIFOAvailable(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevC_GetTXFIFOAvailable((mxc_uart_revc_regs_t *)uart);
}

int MXC_UART_ClearRXFIFO(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevC_ClearRXFIFO((mxc_uart_revc_regs_t *)uart);
}

int MXC_UART_ClearTXFIFO(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevC_ClearTXFIFO((mxc_uart_revc_regs_t *)uart);
}

int MXC_UART_SetRXThreshold(mxc_uart_regs_t *uart, unsigned int numBytes)
{
    return MXC_UART_RevC_SetRXThreshold((mxc_uart_revc_regs_t *)uart, numBytes);
}

unsigned int MXC_UART_GetRXThreshold(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevC_GetRXThreshold((mxc_uart_revc_regs_t *)uart);
}

unsigned int MXC_UART_GetFlags(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevC_GetFlags((mxc_uart_revc_regs_t *)uart);
}

int MXC_UART_ClearFlags(mxc_uart_regs_t *uart, unsigned int flags)
{
    return MXC_UART_RevC_ClearFlags((mxc_uart_revc_regs_t *)uart, flags);
}

int MXC_UART_EnableInt(mxc_uart_regs_t *uart, unsigned int mask)
{
    return MXC_UART_RevC_EnableInt((mxc_uart_revc_regs_t *)uart, mask);
}

int MXC_UART_DisableInt(mxc_uart_regs_t *uart, unsigned int mask)
{
    return MXC_UART_RevC_DisableInt((mxc_uart_revc_regs_t *)uart, mask);
}

unsigned int MXC_UART_GetStatus(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevC_GetStatus((mxc_uart_revc_regs_t *)uart);
}

int MXC_UART_Transaction(mxc_uart_req_t *req)
{
    return MXC_UART_RevC_Transaction((mxc_uart_revc_req_t *)req);
}

int MXC_UART_TransactionAsync(mxc_uart_req_t *req)
{
    return MXC_UART_RevC_TransactionAsync((mxc_uart_revc_req_t *)req);
}

int MXC_UART_TransactionDMA(mxc_uart_req_t *req)
{
    return MXC_UART_RevC_TransactionDMA((mxc_uart_revc_req_t *)req);
}

int MXC_UART_AbortAsync(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevC_AbortAsync((mxc_uart_revc_regs_t *)uart);
}

int MXC_UART_AsyncHandler(mxc_uart_regs_t *uart)
{
    return MXC_UART_RevC_AsyncHandler((mxc_uart_revc_regs_t *)uart);
}

uint32_t MXC_UART_GetAsyncTXCount(mxc_uart_req_t *req)
{
    return req->txCnt;
}

uint32_t MXC_UART_GetAsyncRXCount(mxc_uart_req_t *req)
{
    return req->rxCnt;
}
