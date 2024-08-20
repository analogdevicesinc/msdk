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
#include "mxc_device.h"
#include "mxc_assert.h"
#include "uart.h"
#include "uart_reva.h"
#include "dma.h"
#ifdef __arm__
#include "nvic_table.h"
#endif

// TODO(DMA): Fix multiple DMA instance handling.

/* **** Definitions **** */
#define MXC_UART_REVA_ERRINT_EN                                                       \
    (MXC_F_UART_REVA_INT_EN_RX_FRAME_ERROR | MXC_F_UART_REVA_INT_EN_RX_PARITY_ERROR | \
     MXC_F_UART_REVA_INT_EN_RX_OVERRUN)

#define MXC_UART_REVA_ERRINT_FL                                                       \
    (MXC_F_UART_REVA_INT_FL_RX_FRAME_ERROR | MXC_F_UART_REVA_INT_FL_RX_PARITY_ERROR | \
     MXC_F_UART_REVA_INT_FL_RX_OVERRUN)

/* **** Variable Declaration **** */
static void *TxAsyncRequests[MXC_UART_INSTANCES];
static void *RxAsyncRequests[MXC_UART_INSTANCES];

// Structure to save DMA state
typedef struct {
    mxc_uart_reva_req_t *tx_req;
    mxc_uart_reva_req_t *rx_req;
    int channelTx;
    int channelRx;
    bool auto_dma_handlers;
    mxc_dma_regs_t *dma;
} uart_reva_req_state_t;

// clang-format off
static uart_reva_req_state_t states[MXC_UART_INSTANCES] = {
    [0 ... MXC_UART_INSTANCES - 1] = {
        .tx_req = NULL,
        .rx_req = NULL,
        .channelTx = -1,
        .channelRx = -1,
        .auto_dma_handlers = false
    }
};
// clang-format on

/* **** Function Prototypes **** */

/* ************************************************************************* */
/* Control/Configuration functions                                           */
/* ************************************************************************* */
int MXC_UART_RevA_Init(mxc_uart_reva_regs_t *uart, unsigned int baud)
{
    int err;

    MXC_ASSERT(MXC_UART_GET_IDX((mxc_uart_regs_t *)uart) >= 0)

    // Initialize UART
    // Set RX threshold to 1 byte
    if ((err = (MXC_UART_SetRXThreshold((mxc_uart_regs_t *)uart, 1))) != E_NO_ERROR) {
        return err;
    }

    // Set TX threshold to 2 byte
    if ((err = (MXC_UART_SetTXThreshold((mxc_uart_regs_t *)uart, 2))) != E_NO_ERROR) {
        return err;
    }

    // Set Datasize to 8 bits
    if ((err = (MXC_UART_SetDataSize((mxc_uart_regs_t *)uart, 8))) != E_NO_ERROR) {
        return err;
    }

    if ((err = (MXC_UART_SetParity((mxc_uart_regs_t *)uart, MXC_UART_PARITY_DISABLE))) !=
        E_NO_ERROR) {
        return err;
    }

    if ((err = (MXC_UART_SetStopBits((mxc_uart_regs_t *)uart, MXC_UART_STOP_1))) != E_NO_ERROR) {
        return err;
    }

    uart->ctrl |= MXC_F_UART_REVA_CTRL_ENABLE;

    MXC_UART_SetFrequency((mxc_uart_regs_t *)uart, baud);

    // Initialize state struct
    unsigned int i = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);
    states[i].channelRx = -1;
    states[i].channelTx = -1;
    states[i].tx_req = NULL;
    states[i].rx_req = NULL;
    states[i].auto_dma_handlers = false;
    states[i].dma = NULL;

    return E_NO_ERROR;
}

int MXC_UART_RevA_ReadyForSleep(mxc_uart_reva_regs_t *uart)
{
    if (TxAsyncRequests[MXC_UART_GET_IDX((mxc_uart_regs_t *)uart)] != NULL) {
        return E_BUSY;
    }

    /* We can sleep if waiting for RX Async */

    return MXC_UART_GetActive((mxc_uart_regs_t *)uart);
}

int MXC_UART_RevA_SetFrequency(mxc_uart_reva_regs_t *uart, unsigned int baud)
{
    float uartDiv;
    int periphClock;
    int prescale;
    int decimalDiv;

    if (uart->ctrl & MXC_F_UART_REVA_CTRL_CLKSEL) {
#ifdef IBRO_FREQ
        periphClock = IBRO_FREQ;
#else
        return E_BAD_PARAM;
#endif
    } else {
        periphClock = PeripheralClock;
    }

    uartDiv = (float)periphClock / baud;

    // Find the largest value of prescale that keeps div > 1
    for (prescale = 8; prescale <= 128; prescale = prescale << 1) {
        if (uartDiv / (float)prescale < 1) {
            prescale = prescale >> 1;
            break;
        }
    }

    if (prescale > 128) {
        prescale = 128;
    }

    if (prescale < 8) {
        return E_BAD_PARAM;
    }

    uartDiv /= prescale;
    decimalDiv = (int)((uartDiv - (int)uartDiv) * 128);

    // Work around for Jira Bug: ME10-650
    if (decimalDiv > 3) {
        decimalDiv -= 3;
    } else {
        decimalDiv += 3;
    }

    switch (prescale) {
    case 8:
        prescale = 4;
        break;

    case 16:
        prescale = 3;
        break;

    case 32:
        prescale = 2;
        break;

    case 64:
        prescale = 1;
        break;

    case 128:
        prescale = 0;
        break;

    default:
        return E_UNKNOWN;
        break;
    }

    prescale <<= MXC_F_UART_REVA_BAUD0_FACTOR_POS;
    decimalDiv <<= MXC_F_UART_REVA_BAUD1_DBAUD_POS;

    MXC_SETFIELD(uart->baud0, MXC_F_UART_REVA_BAUD0_FACTOR, prescale);
    MXC_SETFIELD(uart->baud0, MXC_F_UART_REVA_BAUD0_IBAUD,
                 (((int)uartDiv) << MXC_F_UART_REVA_BAUD0_IBAUD_POS));
    MXC_SETFIELD(uart->baud1, MXC_F_UART_REVA_BAUD1_DBAUD, decimalDiv);

    return MXC_UART_GetFrequency((mxc_uart_regs_t *)uart);
}

int MXC_UART_RevA_GetFrequency(mxc_uart_reva_regs_t *uart)
{
    int periphClock = 0;
    float uartDiv = 0;
    float decimalDiv = 0;

    if (uart->ctrl & MXC_F_UART_REVA_CTRL_CLKSEL) {
#ifdef IBRO_FREQ
        periphClock = IBRO_FREQ;
#else
        return E_BAD_PARAM;
#endif
    } else {
        periphClock = PeripheralClock;
    }

    uartDiv += uart->baud0 & MXC_F_UART_REVA_BAUD0_IBAUD;
    decimalDiv = uart->baud1 & MXC_F_UART_REVA_BAUD1_DBAUD;

    // Based on work around for Jira Bug: ME10-650
    // No way to tell if the SetFrequency function added or
    //      subtracted 3 in this range
    if (decimalDiv > 3 && decimalDiv <= 6) {
        decimalDiv -= 3;
    } else {
        decimalDiv += 3;
    }

    uartDiv += decimalDiv / (float)128;
    uartDiv *= (1 << (7 - (uart->baud0 & MXC_F_UART_REVA_BAUD0_FACTOR)));

    return (int)((float)periphClock / uartDiv);
}

int MXC_UART_RevA_SetDataSize(mxc_uart_reva_regs_t *uart, int dataSize)
{
    if (dataSize < 5 || dataSize > 8) {
        return E_BAD_PARAM;
    }

    dataSize = (dataSize - 5) << MXC_F_UART_REVA_CTRL_CHAR_SIZE_POS;

    MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_CHAR_SIZE, dataSize);

    return E_NO_ERROR;
}

int MXC_UART_RevA_SetStopBits(mxc_uart_reva_regs_t *uart, mxc_uart_stop_t stopBits)
{
    switch (stopBits) {
    case MXC_UART_STOP_1:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_STOPBITS,
                     0 << MXC_F_UART_REVA_CTRL_STOPBITS_POS);
        break;

    case MXC_UART_STOP_2:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_STOPBITS,
                     1 << MXC_F_UART_REVA_CTRL_STOPBITS_POS);
        break;

    default:
        return E_BAD_PARAM;
        break;
    }

    return E_NO_ERROR;
}

int MXC_UART_RevA_SetParity(mxc_uart_reva_regs_t *uart, mxc_uart_parity_t parity)
{
    switch (parity) {
    case MXC_UART_PARITY_DISABLE:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARITY_EN,
                     0 << MXC_F_UART_REVA_CTRL_PARITY_EN_POS);
        break;

    case MXC_UART_PARITY_EVEN:
    case MXC_UART_PARITY_EVEN_0:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARITY_EN,
                     1 << MXC_F_UART_REVA_CTRL_PARITY_EN_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARITY, MXC_S_UART_REVA_CTRL_PARITY_EVEN);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARMD, 0 << MXC_F_UART_REVA_CTRL_PARMD_POS);
        break;

    case MXC_UART_PARITY_EVEN_1:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARITY_EN,
                     1 << MXC_F_UART_REVA_CTRL_PARITY_EN_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARITY, MXC_S_UART_REVA_CTRL_PARITY_EVEN);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARMD, 1 << MXC_F_UART_REVA_CTRL_PARMD_POS);
        break;

    case MXC_UART_PARITY_ODD:
    case MXC_UART_PARITY_ODD_0:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARITY_EN,
                     1 << MXC_F_UART_REVA_CTRL_PARITY_EN_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARITY, MXC_S_UART_REVA_CTRL_PARITY_ODD);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARMD, 0 << MXC_F_UART_REVA_CTRL_PARMD_POS);
        break;

    case MXC_UART_PARITY_ODD_1:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARITY_EN,
                     1 << MXC_F_UART_REVA_CTRL_PARITY_EN_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARITY, MXC_S_UART_REVA_CTRL_PARITY_ODD);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARMD, 1 << MXC_F_UART_REVA_CTRL_PARMD_POS);
        break;

    case MXC_UART_PARITY_MARK:
    case MXC_UART_PARITY_MARK_0:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARITY_EN,
                     1 << MXC_F_UART_REVA_CTRL_PARITY_EN_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARITY, MXC_S_UART_REVA_CTRL_PARITY_MARK);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARMD, 0 << MXC_F_UART_REVA_CTRL_PARMD_POS);
        break;

    case MXC_UART_PARITY_MARK_1:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARITY_EN,
                     1 << MXC_F_UART_REVA_CTRL_PARITY_EN_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARITY, MXC_S_UART_REVA_CTRL_PARITY_MARK);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARMD, 1 << MXC_F_UART_REVA_CTRL_PARMD_POS);
        break;

    case MXC_UART_PARITY_SPACE:
    case MXC_UART_PARITY_SPACE_0:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARITY_EN,
                     1 << MXC_F_UART_REVA_CTRL_PARITY_EN_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARITY, MXC_S_UART_REVA_CTRL_PARITY_SPACE);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARMD, 0 << MXC_F_UART_REVA_CTRL_PARMD_POS);
        break;

    case MXC_UART_PARITY_SPACE_1:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARITY_EN,
                     1 << MXC_F_UART_REVA_CTRL_PARITY_EN_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARITY, MXC_S_UART_REVA_CTRL_PARITY_SPACE);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_PARMD, 1 << MXC_F_UART_REVA_CTRL_PARMD_POS);
        break;

    default:
        return E_BAD_PARAM;
        break;
    }

    return E_NO_ERROR;
}

int MXC_UART_RevA_SetFlowCtrl(mxc_uart_reva_regs_t *uart, mxc_uart_flow_t flowCtrl,
                              int rtsThreshold)
{
    switch (flowCtrl) {
    case MXC_UART_FLOW_DIS:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_FLOW_CTRL,
                     0 << MXC_F_UART_REVA_CTRL_FLOW_CTRL_POS);
        break;

    case MXC_UART_FLOW_EN_LOW:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_FLOW_CTRL,
                     1 << MXC_F_UART_REVA_CTRL_FLOW_CTRL_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_FLOW_POL,
                     0 << MXC_F_UART_REVA_CTRL_FLOW_POL_POS);
        break;

    case MXC_UART_FLOW_EN_HIGH:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_FLOW_CTRL,
                     1 << MXC_F_UART_REVA_CTRL_FLOW_CTRL_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_FLOW_POL,
                     1 << MXC_F_UART_REVA_CTRL_FLOW_POL_POS);
        break;

    default:
        return E_BAD_PARAM;
        break;
    }

    if (rtsThreshold < 1 || rtsThreshold > MXC_UART_FIFO_DEPTH) {
        return E_BAD_PARAM;
    }

    rtsThreshold <<= MXC_F_UART_REVA_THRESH_CTRL_RTS_FIFO_THRESH_POS;
    MXC_SETFIELD(uart->thresh_ctrl, MXC_F_UART_REVA_THRESH_CTRL_RTS_FIFO_THRESH, rtsThreshold);

    return E_NO_ERROR;
}

int MXC_UART_RevA_SetClockSource(mxc_uart_reva_regs_t *uart, int usePCLK)
{
    int baudRate;

    baudRate = MXC_UART_GetFrequency((mxc_uart_regs_t *)uart);
    if (baudRate < 0) { // return error code
        return baudRate;
    }

    if (usePCLK) {
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_CLKSEL, 0 << MXC_F_UART_REVA_CTRL_CLKSEL_POS);
    } else {
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_CLKSEL, 1 << MXC_F_UART_REVA_CTRL_CLKSEL_POS);
    }

    return MXC_UART_SetFrequency((mxc_uart_regs_t *)uart, baudRate);
}

int MXC_UART_RevA_SetNullModem(mxc_uart_reva_regs_t *uart, int nullModem)
{
    nullModem = (nullModem > 0) << MXC_F_UART_REVA_CTRL_NULL_MODEM_POS;

    MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_NULL_MODEM, nullModem);

    return E_NO_ERROR;
}

int MXC_UART_RevA_SendBreak(mxc_uart_reva_regs_t *uart)
{
    MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVA_CTRL_BREAK, 1 << MXC_F_UART_REVA_CTRL_BREAK_POS);

    return E_NO_ERROR;
}

int MXC_UART_RevA_GetActive(mxc_uart_reva_regs_t *uart)
{
    if (uart->status & (MXC_F_UART_REVA_STATUS_TX_BUSY | MXC_F_UART_REVA_STATUS_RX_BUSY)) {
        return E_BUSY;
    }

    return E_NO_ERROR;
}

int MXC_UART_RevA_AbortTransmission(mxc_uart_reva_regs_t *uart)
{
    MXC_UART_ClearTXFIFO((mxc_uart_regs_t *)uart);
    return E_NO_ERROR;
}

int MXC_UART_RevA_ReadCharacterRaw(mxc_uart_reva_regs_t *uart)
{
    if (uart->status & MXC_F_UART_REVA_STATUS_RX_EMPTY) {
        return E_UNDERFLOW;
    }

    return uart->fifo;
}

int MXC_UART_RevA_WriteCharacterRaw(mxc_uart_reva_regs_t *uart, uint8_t character)
{
    // Return error if the FIFO is full
    if (uart->status & MXC_F_UART_REVA_STATUS_TX_FULL) {
        return E_OVERFLOW;
    }

    uart->fifo = character;

    return E_NO_ERROR;
}

int MXC_UART_RevA_Read(mxc_uart_reva_regs_t *uart, uint8_t *buffer, int *len)
{
    int read = 0;
    int retVal;

    for (; read < *len; read++) {
        retVal = MXC_UART_ReadCharacter((mxc_uart_regs_t *)uart);

        if (retVal < 0) {
            *len = read;
            return retVal;
        } else {
            buffer[read] = retVal;
        }
    }

    *len = read;
    return E_NO_ERROR;
}

int MXC_UART_RevA_Write(mxc_uart_reva_regs_t *uart, uint8_t *byte, int *len)
{
    int written = 0;
    int retVal;

    for (; written < *len; written++) {
        retVal = MXC_UART_WriteCharacter((mxc_uart_regs_t *)uart, byte[written]);

        if (retVal != E_NO_ERROR) {
            *len = written;
            return retVal;
        }
    }

    *len = written;
    return E_NO_ERROR;
}

unsigned int MXC_UART_RevA_ReadRXFIFO(mxc_uart_reva_regs_t *uart, unsigned char *bytes,
                                      unsigned int len)
{
    unsigned int read = 0;

    for (; read < len; read++) {
        if (uart->status & MXC_F_UART_REVA_STATUS_RX_EMPTY) {
            break;
        }

        bytes[read] = uart->fifo;
    }

    return read;
}

#if (MXC_DMA_INSTANCES > 1)

void MXC_UART_RevA_DMA0_Handler(void)
{
    MXC_DMA_Handler(MXC_DMA0);
}

void MXC_UART_RevA_DMA1_Handler(void)
{
    MXC_DMA_Handler(MXC_DMA1);
}

#endif

/* "Auto" handlers just need to call MXC_DMA_Handler with the correct
DMA instance.
*/
void MXC_UART_RevA_DMA_SetupAutoHandlers(mxc_dma_regs_t *dma_instance, unsigned int channel)
{
#ifdef __arm__
#if (MXC_DMA_INSTANCES > 1)
    NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(channel));

    /* (JC): This is not the cleanest or most scalable way to do this,
        but I tried defining default handler's in the system file.
        Some complications make this the most attractive short-term
        option.  We could handle multiple DMA instances better in the DMA API (See the mismatch between the size of "dma_resource" array and the number of channels per instance, to start)*/
    if (dma_instance == MXC_DMA0) {
        MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(channel), MXC_UART_RevA_DMA0_Handler);
    } else if (dma_instance == MXC_DMA1) {
        MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(channel), MXC_UART_RevA_DMA1_Handler);
    }
#else
    NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(channel));

    // Only one DMA instance, we can point direct to MXC_DMA_Handler
    MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(channel), MXC_DMA_Handler);
#endif // MXC_DMA_INSTANCES > 1

#else
    // TODO(JC): RISC-V

#endif // __arm__
}

int MXC_UART_RevA_ReadRXFIFODMA(mxc_uart_reva_regs_t *uart, mxc_dma_regs_t *dma,
                                unsigned char *bytes, unsigned int len,
                                mxc_uart_dma_complete_cb_t callback, mxc_dma_config_t config)
{
    uint8_t channel;
    mxc_dma_srcdst_t srcdst;

    int uart_num = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);

    if (bytes == NULL) {
        return E_NULL_PTR;
    }

    if (states[uart_num].auto_dma_handlers && states[uart_num].channelRx < 0) {
        /* Acquire channel if we don't have one already */
#if MXC_DMA_INSTANCES > 1
        channel = MXC_DMA_AcquireChannel(dma);
#else
        channel = MXC_DMA_AcquireChannel();
#endif
        MXC_UART_RevA_SetRXDMAChannel(uart, channel);
        /* (JC) Since we're automatically acquiring a channel here, we need the ISR for that channel to call MXC_DMA_Handler. */
        MXC_UART_RevA_DMA_SetupAutoHandlers(dma, channel);
    } else {
        /* Rely on application-defined handlers. */
        if (states[uart_num].channelRx < 0)
            return E_BAD_STATE;
        channel = states[uart_num].channelRx;
    }

    // states[uart_num].channelRx = channel;

    config.ch = channel;

    config.srcwd = MXC_DMA_WIDTH_BYTE;
    config.dstwd = MXC_DMA_WIDTH_BYTE;

    config.srcinc_en = 0;
    config.dstinc_en = 1;

    srcdst.ch = channel;
    srcdst.dest = bytes;
    srcdst.len = len;

    MXC_DMA_ConfigChannel(config, srcdst);
    MXC_DMA_SetCallback(channel, MXC_UART_DMACallback);

    MXC_DMA_EnableInt(channel);

    MXC_DMA_Start(channel);
    //MXC_DMA->ch[channel].ctrl |= MXC_F_DMA_CTRL_CTZ_IE;
    MXC_DMA_SetChannelInterruptEn(channel, 0, 1);
    uart->dma |= MXC_F_UART_REVA_DMA_RXDMA_EN;

    return E_NO_ERROR;
}

unsigned int MXC_UART_RevA_GetRXFIFOAvailable(mxc_uart_reva_regs_t *uart)
{
    return (uart->status & MXC_F_UART_REVA_STATUS_RX_FIFO_CNT) >>
           MXC_F_UART_REVA_STATUS_RX_FIFO_CNT_POS;
}

unsigned int MXC_UART_RevA_WriteTXFIFO(mxc_uart_reva_regs_t *uart, unsigned char *bytes,
                                       unsigned int len)
{
    unsigned int written = 0;

    for (; written < len; written++) {
        if (uart->status & MXC_F_UART_REVA_STATUS_TX_FULL) {
            break;
        }

        uart->fifo = bytes[written];
    }

    return written;
}

int MXC_UART_RevA_SetAutoDMAHandlers(mxc_uart_reva_regs_t *uart, bool enable)
{
    int n = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);

    states[n].auto_dma_handlers = enable;

    return E_NO_ERROR;
}

int MXC_UART_RevA_SetTXDMAChannel(mxc_uart_reva_regs_t *uart, unsigned int channel)
{
    int n = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);

    states[n].channelTx = channel;

    return E_NO_ERROR;
}

int MXC_UART_RevA_GetTXDMAChannel(mxc_uart_reva_regs_t *uart)
{
    int n = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);

    return states[n].channelTx;
}

int MXC_UART_RevA_SetRXDMAChannel(mxc_uart_reva_regs_t *uart, unsigned int channel)
{
    int n = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);

    states[n].channelRx = channel;

    return E_NO_ERROR;
}

int MXC_UART_RevA_GetRXDMAChannel(mxc_uart_reva_regs_t *uart)
{
    int n = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);

    return states[n].channelRx;
}

unsigned int MXC_UART_RevA_WriteTXFIFODMA(mxc_uart_reva_regs_t *uart, mxc_dma_regs_t *dma,
                                          unsigned char *bytes, unsigned int len,
                                          mxc_uart_dma_complete_cb_t callback,
                                          mxc_dma_config_t config)
{
    int channel = -1;
    mxc_dma_srcdst_t srcdst;

    int uart_num = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);

    if (bytes == NULL) {
        return E_NULL_PTR;
    }

    if (states[uart_num].auto_dma_handlers && states[uart_num].channelTx < 0) {
        /* Acquire channel if we don't have one already */
#if (TARGET_NUM == 32665)
        channel = MXC_DMA_AcquireChannel(dma);
#else
        channel = MXC_DMA_AcquireChannel();
#endif
        MXC_UART_RevA_SetTXDMAChannel(uart, channel); // Set state variable
        /* (JC) Since we're automatically acquiring a channel here, we need the ISR for that channel to call MXC_DMA_Handler.*/
        MXC_UART_RevA_DMA_SetupAutoHandlers(dma, channel);
    } else {
        /* Rely on application-defined handlers (from SetTXDMAChannel) */
        if (states[uart_num].channelTx < 0)
            return E_BAD_STATE;
        channel = MXC_UART_RevA_GetTXDMAChannel(uart);
    }

    config.ch = channel;

    config.srcwd = MXC_DMA_WIDTH_BYTE;
    config.dstwd = MXC_DMA_WIDTH_BYTE;

    config.srcinc_en = 1;
    config.dstinc_en = 0;

    srcdst.ch = channel;
    srcdst.source = bytes;
    srcdst.len = len;

    MXC_DMA_ConfigChannel(config, srcdst);
    MXC_DMA_SetCallback(channel, MXC_UART_DMACallback);

    MXC_DMA_EnableInt(channel);

    MXC_DMA_Start(channel);
    //MXC_DMA->ch[channel].ctrl |= MXC_F_DMA_CTRL_CTZ_IE;
    MXC_DMA_SetChannelInterruptEn(channel, 0, 1);

    uart->dma |= MXC_F_UART_REVA_DMA_TXDMA_EN;

    return E_NO_ERROR;
}

unsigned int MXC_UART_RevA_GetTXFIFOAvailable(mxc_uart_reva_regs_t *uart)
{
    int txCnt = (uart->status & MXC_F_UART_REVA_STATUS_TX_FIFO_CNT) >>
                MXC_F_UART_REVA_STATUS_TX_FIFO_CNT_POS;
    return MXC_UART_FIFO_DEPTH - txCnt;
}

int MXC_UART_RevA_ClearRXFIFO(mxc_uart_reva_regs_t *uart)
{
    uart->ctrl |= MXC_F_UART_REVA_CTRL_RX_FLUSH;

    while (uart->ctrl & MXC_F_UART_REVA_CTRL_RX_FLUSH) {}

    return E_NO_ERROR;
}

int MXC_UART_RevA_ClearTXFIFO(mxc_uart_reva_regs_t *uart)
{
    uart->ctrl |= MXC_F_UART_REVA_CTRL_TX_FLUSH;

    while (uart->ctrl & MXC_F_UART_REVA_CTRL_TX_FLUSH) {}

    return E_NO_ERROR;
}

int MXC_UART_RevA_SetRXThreshold(mxc_uart_reva_regs_t *uart, unsigned int numBytes)
{
    if (numBytes < 1 || numBytes > MXC_UART_FIFO_DEPTH) {
        return E_BAD_PARAM;
    }

    numBytes <<= MXC_F_UART_REVA_THRESH_CTRL_RX_FIFO_THRESH_POS;
    MXC_SETFIELD(uart->thresh_ctrl, MXC_F_UART_REVA_THRESH_CTRL_RX_FIFO_THRESH, numBytes);

    return E_NO_ERROR;
}

unsigned int MXC_UART_RevA_GetRXThreshold(mxc_uart_reva_regs_t *uart)
{
    return (uart->thresh_ctrl & MXC_F_UART_REVA_THRESH_CTRL_RX_FIFO_THRESH) >>
           MXC_F_UART_REVA_THRESH_CTRL_RX_FIFO_THRESH_POS;
}

int MXC_UART_RevA_SetTXThreshold(mxc_uart_reva_regs_t *uart, unsigned int numBytes)
{
    if (numBytes < 1 || numBytes > MXC_UART_FIFO_DEPTH) {
        return E_BAD_PARAM;
    }

    numBytes <<= MXC_F_UART_REVA_THRESH_CTRL_TX_FIFO_THRESH_POS;
    MXC_SETFIELD(uart->thresh_ctrl, MXC_F_UART_REVA_THRESH_CTRL_TX_FIFO_THRESH, numBytes);

    return E_NO_ERROR;
}

unsigned int MXC_UART_RevA_GetTXThreshold(mxc_uart_reva_regs_t *uart)
{
    return (uart->thresh_ctrl & MXC_F_UART_REVA_THRESH_CTRL_TX_FIFO_THRESH) >>
           MXC_F_UART_REVA_THRESH_CTRL_TX_FIFO_THRESH_POS;
}

unsigned int MXC_UART_RevA_GetFlags(mxc_uart_reva_regs_t *uart)
{
    return uart->int_fl;
}

int MXC_UART_RevA_ClearFlags(mxc_uart_reva_regs_t *uart, unsigned int flags)
{
    uart->int_fl = flags;

    return E_NO_ERROR;
}

int MXC_UART_RevA_EnableInt(mxc_uart_reva_regs_t *uart, unsigned int intEn)
{
    uart->int_en |= intEn;

    return E_NO_ERROR;
}

int MXC_UART_RevA_DisableInt(mxc_uart_reva_regs_t *uart, unsigned int intDis)
{
    uart->int_en &= ~intDis;

    return E_NO_ERROR;
}

unsigned int MXC_UART_RevA_GetStatus(mxc_uart_reva_regs_t *uart)
{
    return uart->status;
}

int MXC_UART_RevA_Busy(mxc_uart_reva_regs_t *uart)
{
    int uart_num =
        MXC_UART_GET_IDX((mxc_uart_regs_t *)uart); // Holds the current index of tx_states
    if ((uart->status & MXC_F_UART_REVA_STATUS_TX_BUSY) ||
        (uart->status & MXC_F_UART_REVA_STATUS_RX_BUSY)) {
        return E_BUSY;
    }
    // Check to see if there are any ongoing transactions and the UART has room in its FIFO
    if ((states[uart_num].tx_req == NULL) && (states[uart_num].rx_req == NULL) &&
        !(uart->status & MXC_F_UART_REVA_STATUS_TX_FULL)) {
        return E_NO_ERROR;
    }

    return E_BUSY;
}

int MXC_UART_RevA_Transaction(mxc_uart_reva_req_t *req)
{
    unsigned int numToWrite, numToRead;

    if (MXC_UART_GET_IDX((mxc_uart_regs_t *)(req->uart)) < 0) {
        return E_BAD_PARAM;
    }

    MXC_UART_DisableInt((mxc_uart_regs_t *)(req->uart), 0xFFFFFFFF);
    MXC_UART_ClearFlags((mxc_uart_regs_t *)(req->uart), 0xFFFFFFFF);

    MXC_UART_ClearRXFIFO((mxc_uart_regs_t *)(req->uart));
    MXC_UART_ClearTXFIFO((mxc_uart_regs_t *)(req->uart));

    req->txCnt = 0;
    req->rxCnt = 0;

    if (req->rxLen) {
        if (req->rxData == NULL) {
            return E_BAD_PARAM;
        }
    }

    if (req->txLen) {
        if (req->txData == NULL) {
            return E_BAD_PARAM;
        }

        numToWrite = MXC_UART_GetTXFIFOAvailable((mxc_uart_regs_t *)(req->uart));
        numToWrite = numToWrite > (req->txLen - req->txCnt) ? req->txLen - req->txCnt : numToWrite;
        req->txCnt += MXC_UART_WriteTXFIFO((mxc_uart_regs_t *)(req->uart), &req->txData[req->txCnt],
                                           numToWrite);

        while (req->txCnt < req->txLen) {
            while (!(MXC_UART_GetFlags((mxc_uart_regs_t *)(req->uart)) &
                     MXC_F_UART_REVA_INT_FL_TX_FIFO_THRESH)) {}

            numToWrite = MXC_UART_GetTXFIFOAvailable((mxc_uart_regs_t *)(req->uart));
            numToWrite = numToWrite > (req->txLen - req->txCnt) ? req->txLen - req->txCnt :
                                                                  numToWrite;
            req->txCnt += MXC_UART_WriteTXFIFO((mxc_uart_regs_t *)(req->uart),
                                               &req->txData[req->txCnt], numToWrite);
            MXC_UART_ClearFlags((mxc_uart_regs_t *)(req->uart),
                                MXC_F_UART_REVA_INT_FL_TX_FIFO_THRESH);
        }
    }

    if (req->rxLen) {
        numToRead = MXC_UART_GetRXFIFOAvailable((mxc_uart_regs_t *)(req->uart));
        numToRead = numToRead > (req->rxLen - req->rxCnt) ? req->rxLen - req->rxCnt : numToRead;
        req->rxCnt += MXC_UART_ReadRXFIFO((mxc_uart_regs_t *)(req->uart), &req->rxData[req->rxCnt],
                                          numToRead);

        while (req->rxCnt < req->rxLen) {
            while (!(MXC_UART_GetFlags((mxc_uart_regs_t *)(req->uart)) &
                     MXC_F_UART_REVA_INT_FL_RX_FIFO_THRESH)) {}

            numToRead = MXC_UART_GetRXFIFOAvailable((mxc_uart_regs_t *)(req->uart));
            numToRead = numToRead > (req->rxLen - req->rxCnt) ? req->rxLen - req->rxCnt : numToRead;
            req->rxCnt += MXC_UART_ReadRXFIFO((mxc_uart_regs_t *)(req->uart),
                                              &req->rxData[req->rxCnt], numToRead);
            MXC_UART_ClearFlags((mxc_uart_regs_t *)(req->uart),
                                MXC_F_UART_REVA_INT_FL_RX_FIFO_THRESH);
        }
    }

    return E_NO_ERROR;
}

int MXC_UART_RevA_TransactionAsync(mxc_uart_reva_req_t *req)
{
    unsigned int numToWrite, numToRead;
    int uart_num = MXC_UART_GET_IDX((mxc_uart_regs_t *)(req->uart));

    if (req->txLen) {
        if (TxAsyncRequests[uart_num] != NULL) {
            return E_BAD_STATE;
        } else if (req->txData == NULL) {
            return E_BAD_PARAM;
        }

        req->txCnt = 0;
        TxAsyncRequests[uart_num] = (void *)req;

        // Enable TX Threshold interrupt
        MXC_UART_EnableInt((mxc_uart_regs_t *)(req->uart), MXC_F_UART_REVA_INT_EN_TX_FIFO_THRESH);

        numToWrite = MXC_UART_GetTXFIFOAvailable((mxc_uart_regs_t *)(req->uart));
        numToWrite = numToWrite > (req->txLen - req->txCnt) ? req->txLen - req->txCnt : numToWrite;
        req->txCnt += MXC_UART_WriteTXFIFO((mxc_uart_regs_t *)(req->uart), &req->txData[req->txCnt],
                                           numToWrite);
    }

    if (req->rxLen) {
        if (RxAsyncRequests[uart_num] != NULL) {
            return E_BAD_STATE;
        } else if (req->rxData == NULL) {
            MXC_UART_DisableInt((mxc_uart_regs_t *)(req->uart), 0xFFFFFFFF);
            return E_BAD_PARAM;
        }

        req->rxCnt = 0;
        RxAsyncRequests[uart_num] = (void *)req;

        // All error interrupts are related to RX
        MXC_UART_EnableInt((mxc_uart_regs_t *)(req->uart), MXC_UART_REVA_ERRINT_EN);

        // Enable RX Threshold interrupt
        MXC_UART_EnableInt((mxc_uart_regs_t *)(req->uart), MXC_F_UART_REVA_INT_EN_RX_FIFO_THRESH);

        numToRead = MXC_UART_GetRXFIFOAvailable((mxc_uart_regs_t *)(req->uart));
        numToRead = numToRead > (req->rxLen - req->rxCnt) ? req->rxLen - req->rxCnt : numToRead;
        req->rxCnt += MXC_UART_ReadRXFIFO((mxc_uart_regs_t *)(req->uart), &req->rxData[req->rxCnt],
                                          numToRead);
        MXC_UART_ClearFlags((mxc_uart_regs_t *)(req->uart), MXC_F_UART_REVA_INT_FL_RX_FIFO_THRESH);
    }

    return E_NO_ERROR;
}

int MXC_UART_RevA_TransactionDMA(mxc_uart_reva_req_t *req, mxc_dma_regs_t *dma)
{
    int uart_num = MXC_UART_GET_IDX((mxc_uart_regs_t *)(req->uart));

    // Save DMA instance for DMA Callback.
    states[uart_num].dma = dma;

    if (req->txLen) {
        if (req->txData == NULL) {
            return E_BAD_PARAM;
        }
    }

    if (req->rxLen) {
        if (req->rxData == NULL) {
            return E_BAD_PARAM;
        }
    }

    MXC_UART_DisableInt((mxc_uart_regs_t *)(req->uart), 0xFFFFFFFF);
    MXC_UART_ClearFlags((mxc_uart_regs_t *)(req->uart), 0xFFFFFFFF);

    /* Clearing the RX FIFOs here makes RX-only or TX-only transactions half-duplex...
    Commenting out for now.*/
    // MXC_UART_ClearRXFIFO((mxc_uart_regs_t *)(req->uart));
    // MXC_UART_ClearTXFIFO((mxc_uart_regs_t *)(req->uart));

    (req->uart)->dma |=
        (1 << MXC_F_UART_REVA_DMA_RXDMA_LEVEL_POS); // Set RX DMA threshold to 1 byte
    (req->uart)->dma |=
        (2 << MXC_F_UART_REVA_DMA_TXDMA_LEVEL_POS); // Set TX DMA threshold to 2 bytes

#if (TARGET_NUM == 32665)
    MXC_DMA_Init(dma);
#else
    MXC_DMA_Init();
#endif

    // Reset rx/tx counters
    req->rxCnt = 0;
    req->txCnt = 0;

    //tx
    if ((req->txData != NULL) && (req->txLen)) {
        /* Save TX req, the DMA handler will use this later. */
        states[uart_num].tx_req = req;
#if (TARGET_NUM == 32665)
        if (MXC_UART_WriteTXFIFODMA((mxc_uart_regs_t *)(req->uart), dma, req->txData, req->txLen,
                                    NULL) != E_NO_ERROR) {
            return E_BAD_PARAM;
        }
#else
        if (MXC_UART_WriteTXFIFODMA((mxc_uart_regs_t *)(req->uart), req->txData, req->txLen,
                                    NULL) != E_NO_ERROR) {
            return E_BAD_PARAM;
        }
#endif
    }

    if ((req->rxData != NULL) && (req->rxLen)) {
        states[uart_num].rx_req = req;
#if (TARGET_NUM == 32665)
        if (MXC_UART_ReadRXFIFODMA((mxc_uart_regs_t *)(req->uart), dma, req->rxData, req->rxLen,
                                   NULL) != E_NO_ERROR) {
            return E_BAD_PARAM;
        }
#else
        if (MXC_UART_ReadRXFIFODMA((mxc_uart_regs_t *)(req->uart), req->rxData, req->rxLen, NULL) !=
            E_NO_ERROR) {
            return E_BAD_PARAM;
        }
#endif
    }

    return E_NO_ERROR;
}

void MXC_UART_RevA_DMACallback(int ch, int error)
{
    mxc_uart_reva_req_t *temp_req = NULL;

    for (int i = 0; i < MXC_UART_INSTANCES; i++) {
        if (states[i].channelTx == ch) {
            /* Populate txLen.  The number of "remainder" bytes is what's left on the
            DMA channel's count register. */
            states[i].tx_req->txCnt = states[i].tx_req->txLen - states[i].dma->ch[ch].cnt;

            temp_req = states[i].tx_req;

            if (states[i].auto_dma_handlers) {
                /* Release channel _before_ running callback in case
                user wants to start another transaction inside it */
                MXC_DMA_ReleaseChannel(ch);
                states[i].channelTx = -1;
            }

            if (temp_req->callback != NULL &&
                ((states[i].tx_req->rxCnt == states[i].tx_req->rxLen) ||
                 states[i].tx_req->rxData == NULL)) {
                /* Only call TX callback if RX component is complete/disabled. Note that
                we are checking the request associated with the _channel_ assignment, not
                the other side of the state struct. */
                temp_req->callback((mxc_uart_req_t *)temp_req, E_NO_ERROR);
            }
            break;
        } else if (states[i].channelRx == ch) {
            /* Same as above, but for RX */
            states[i].rx_req->rxCnt = states[i].rx_req->rxLen - states[i].dma->ch[ch].cnt;
            temp_req = states[i].rx_req;
            if (states[i].auto_dma_handlers) {
                MXC_DMA_ReleaseChannel(ch);
                states[i].channelRx = -1;
            }

            if (temp_req->callback != NULL &&
                ((states[i].rx_req->txCnt == states[i].rx_req->txLen) ||
                 states[i].rx_req->txData == NULL)) {
                temp_req->callback((mxc_uart_req_t *)temp_req, E_NO_ERROR);
            }
            break;
        }
    }
}

int MXC_UART_RevA_RxAsyncCallback(mxc_uart_reva_regs_t *uart, int retVal)
{
    mxc_uart_reva_req_t *req;
    int uart_num = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);

    if (RxAsyncRequests[uart_num] == NULL) {
        return E_BAD_STATE;
    }

    req = (mxc_uart_reva_req_t *)RxAsyncRequests[uart_num];

    if (req->callback != NULL) {
        req->callback((mxc_uart_req_t *)req, retVal);
    }

    return E_NO_ERROR;
}

int MXC_UART_RevA_TxAsyncCallback(mxc_uart_reva_regs_t *uart, int retVal)
{
    mxc_uart_reva_req_t *req;
    int uart_num = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);

    if (TxAsyncRequests[uart_num] == NULL) {
        return E_BAD_STATE;
    }

    req = (mxc_uart_reva_req_t *)TxAsyncRequests[uart_num];

    if (req->callback != NULL) {
        req->callback((mxc_uart_req_t *)req, retVal);
    }

    return E_NO_ERROR;
}

int MXC_UART_RevA_AsyncCallback(mxc_uart_reva_regs_t *uart, int retVal)
{
    int err;

    int uart_num = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);

    // Call TX callback
    err = MXC_UART_TxAsyncCallback((mxc_uart_regs_t *)uart, retVal);
    if (err != E_NO_ERROR) {
        return err;
    }

    // Call RX callback if the TX and RX requests are not the same request
    if (TxAsyncRequests[uart_num] != RxAsyncRequests[uart_num]) {
        err = MXC_UART_RxAsyncCallback((mxc_uart_regs_t *)uart, retVal);
    }

    return err;
}

int MXC_UART_RevA_TxAsyncStop(mxc_uart_reva_regs_t *uart)
{
    MXC_UART_DisableInt((mxc_uart_regs_t *)uart, MXC_F_UART_REVA_INT_EN_TX_FIFO_THRESH);
    TxAsyncRequests[MXC_UART_GET_IDX((mxc_uart_regs_t *)uart)] = NULL;

    return E_NO_ERROR;
}

int MXC_UART_RevA_RxAsyncStop(mxc_uart_reva_regs_t *uart)
{
    MXC_UART_DisableInt((mxc_uart_regs_t *)uart,
                        (MXC_UART_REVA_ERRINT_EN | MXC_F_UART_REVA_INT_EN_RX_FIFO_THRESH));
    RxAsyncRequests[MXC_UART_GET_IDX((mxc_uart_regs_t *)uart)] = NULL;

    return E_NO_ERROR;
}

int MXC_UART_RevA_AsyncStop(mxc_uart_reva_regs_t *uart)
{
    int uart_num = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);
    mxc_uart_reva_req_t *req;

    req = (mxc_uart_reva_req_t *)TxAsyncRequests[uart_num];
    if (req != NULL) {
        MXC_UART_TxAsyncStop((mxc_uart_regs_t *)uart);
    }

    req = (mxc_uart_reva_req_t *)RxAsyncRequests[uart_num];
    if (req != NULL) {
        MXC_UART_RxAsyncStop((mxc_uart_regs_t *)uart);
    }

    return E_NO_ERROR;
}

int MXC_UART_RevA_TxAbortAsync(mxc_uart_reva_regs_t *uart)
{
    int uart_num = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);

    mxc_uart_reva_req_t *req = (mxc_uart_reva_req_t *)TxAsyncRequests[uart_num];

    if (req != NULL) {
        MXC_UART_TxAsyncCallback((mxc_uart_regs_t *)uart, E_ABORT);
        MXC_UART_TxAsyncStop((mxc_uart_regs_t *)uart);
    }

    return E_NO_ERROR;
}

int MXC_UART_RevA_RxAbortAsync(mxc_uart_reva_regs_t *uart)
{
    int uart_num = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);

    mxc_uart_reva_req_t *req = (mxc_uart_reva_req_t *)RxAsyncRequests[uart_num];

    if (req != NULL) {
        MXC_UART_RxAsyncCallback((mxc_uart_regs_t *)uart, E_ABORT);
        MXC_UART_RxAsyncStop((mxc_uart_regs_t *)uart);
    }

    return E_NO_ERROR;
}

int MXC_UART_RevA_AbortAsync(mxc_uart_reva_regs_t *uart)
{
    int err;

    if (MXC_UART_GET_IDX((mxc_uart_regs_t *)uart) < 0) {
        return E_BAD_PARAM;
    }

    // Call appropriate callback
    err = MXC_UART_AsyncCallback((mxc_uart_regs_t *)uart, E_ABORT);
    if (err != E_NO_ERROR) {
        return err;
    }

    // Disable interrupts and clear async request
    return MXC_UART_AsyncStop((mxc_uart_regs_t *)uart);
}

int MXC_UART_RevA_AsyncHandler(mxc_uart_reva_regs_t *uart)
{
    int uart_num = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);
    unsigned int flags, numToWrite, numToRead;
    mxc_uart_reva_req_t *req;

    flags = MXC_UART_GetFlags((mxc_uart_regs_t *)uart);

    if (flags & MXC_UART_REVA_ERRINT_FL & uart->int_en) {
        MXC_UART_DisableInt((mxc_uart_regs_t *)uart,
                            (MXC_F_UART_REVA_INT_EN_TX_FIFO_THRESH | MXC_UART_REVA_ERRINT_EN |
                             MXC_F_UART_REVA_INT_EN_RX_FIFO_THRESH));

        MXC_UART_AsyncCallback((mxc_uart_regs_t *)uart, E_COMM_ERR);

        RxAsyncRequests[uart_num] = NULL;
        TxAsyncRequests[uart_num] = NULL;

        return E_INVALID;
    }

    req = (mxc_uart_reva_req_t *)TxAsyncRequests[uart_num];

    if ((flags & MXC_F_UART_REVA_INT_FL_TX_FIFO_THRESH) && (req != NULL) && (req->txLen)) {
        numToWrite = MXC_UART_GetTXFIFOAvailable((mxc_uart_regs_t *)(req->uart));
        numToWrite = numToWrite > (req->txLen - req->txCnt) ? req->txLen - req->txCnt : numToWrite;
        req->txCnt += MXC_UART_WriteTXFIFO((mxc_uart_regs_t *)(req->uart), &req->txData[req->txCnt],
                                           numToWrite);
        MXC_UART_ClearFlags((mxc_uart_regs_t *)(req->uart), MXC_F_UART_REVA_INT_FL_TX_FIFO_THRESH);
    }

    req = (mxc_uart_reva_req_t *)RxAsyncRequests[uart_num];

    if ((flags & MXC_F_UART_REVA_INT_FL_RX_FIFO_THRESH) && (req != NULL) && (req->rxLen)) {
        numToRead = MXC_UART_GetRXFIFOAvailable((mxc_uart_regs_t *)(req->uart));
        numToRead = numToRead > (req->rxLen - req->rxCnt) ? req->rxLen - req->rxCnt : numToRead;
        req->rxCnt += MXC_UART_ReadRXFIFO((mxc_uart_regs_t *)(req->uart), &req->rxData[req->rxCnt],
                                          numToRead);
        MXC_UART_ClearFlags((mxc_uart_regs_t *)(req->uart), MXC_F_UART_REVA_INT_FL_RX_FIFO_THRESH);
    }

    if (RxAsyncRequests[uart_num] == TxAsyncRequests[uart_num]) {
        if ((req != NULL) && (req->rxCnt == req->rxLen) && (req->txCnt == req->txLen)) {
            MXC_UART_DisableInt((mxc_uart_regs_t *)uart,
                                (MXC_F_UART_REVA_INT_EN_TX_FIFO_THRESH | MXC_UART_REVA_ERRINT_EN |
                                 MXC_F_UART_REVA_INT_EN_RX_FIFO_THRESH));

            RxAsyncRequests[uart_num] = NULL;
            TxAsyncRequests[uart_num] = NULL;

            if (req->callback != NULL) {
                req->callback((mxc_uart_req_t *)req, E_NO_ERROR);
            }
        }

        return E_NO_ERROR;
    }

    req = TxAsyncRequests[uart_num];
    if (req != NULL && req->txCnt == req->txLen) {
        MXC_UART_DisableInt((mxc_uart_regs_t *)uart, MXC_F_UART_REVA_INT_EN_TX_FIFO_THRESH);
        TxAsyncRequests[uart_num] = NULL;

        if (req->callback != NULL) {
            req->callback((mxc_uart_req_t *)req, E_NO_ERROR);
        }

        return E_NO_ERROR;
    }

    req = RxAsyncRequests[uart_num];
    if (req != NULL && req->rxCnt == req->rxLen) {
        MXC_UART_DisableInt((mxc_uart_regs_t *)uart,
                            (MXC_UART_REVA_ERRINT_EN | MXC_F_UART_REVA_INT_EN_RX_FIFO_THRESH));
        RxAsyncRequests[uart_num] = NULL;

        if (req->callback != NULL) {
            req->callback((mxc_uart_req_t *)req, E_NO_ERROR);
        }

        return E_NO_ERROR;
    }

    return E_NO_ERROR;
}
