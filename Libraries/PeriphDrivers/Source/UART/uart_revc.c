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
#include "uart_revc.h"
#include "dma.h"

/* **** Definitions **** */
#define MXC_UART_REVC_ERRINT_EN                                                       \
    (MXC_F_UART_REVC_INT_EN_RX_FRAME_ERROR | MXC_F_UART_REVC_INT_EN_RX_PARITY_ERROR | \
     MXC_F_UART_REVC_INT_EN_RX_OVERRUN)

#define MXC_UART_REVC_ERRINT_FL                                                       \
    (MXC_F_UART_REVC_INT_FL_RX_FRAME_ERROR | MXC_F_UART_REVC_INT_FL_RX_PARITY_ERROR | \
     MXC_F_UART_REVC_INT_FL_RX_OVERRUN)

/* **** Variable Declaration **** */
static void *AsyncRequests[MXC_UART_INSTANCES];
// static int baudRate;

// Structure to save DMA state
typedef struct {
    mxc_uart_revc_req_t *req;
    int channelTx;
    int channelRx;
} uart_revc_req_state_t;

static uart_revc_req_state_t states[MXC_UART_INSTANCES];

/* **** Function Prototypes **** */

/* ************************************************************************* */
/* Control/Configuration functions                                           */
/* ************************************************************************* */
int MXC_UART_RevC_Init(mxc_uart_revc_regs_t *uart, unsigned int baud)
{
    int err;

    MXC_ASSERT(MXC_UART_GET_IDX((mxc_uart_regs_t *)uart) >= 0)

    // Initialize UART
    // Set RX threshold to 1 byte
    if ((err = MXC_UART_SetRXThreshold((mxc_uart_regs_t *)uart, 1)) != E_NO_ERROR) {
        return err;
    }

    // Set Datasize to 8 bits
    if ((err = MXC_UART_SetDataSize((mxc_uart_regs_t *)uart, 8)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_UART_SetParity((mxc_uart_regs_t *)uart, MXC_UART_PARITY_DISABLE)) !=
        E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_UART_SetStopBits((mxc_uart_regs_t *)uart, MXC_UART_STOP_1)) != E_NO_ERROR) {
        return err;
    }

    // uart->ctrl |= MXC_F_UART_REVC_CTRL_ENABLE;
    MXC_UART_SetFrequency((mxc_uart_regs_t *)uart, baud);

    return E_NO_ERROR;
}

int MXC_UART_RevC_ReadyForSleep(mxc_uart_revc_regs_t *uart)
{
    if (AsyncRequests[MXC_UART_GET_IDX((mxc_uart_regs_t *)uart)] != NULL) {
        return E_BUSY;
    }

    return MXC_UART_GetActive((mxc_uart_regs_t *)uart);
}

int MXC_UART_RevC_SetFrequency(mxc_uart_revc_regs_t *uart, unsigned int baud)
{
    int div = 0, baud0 = 0, baud1 = 0;

    // Set the baud rate
    // Calculate divisor
    div = PeripheralClock / baud;
    // Search for integer and fractional baud rate registers based on divisor
    baud0 = div >> (7); // divide by 128 extract integer part
    baud1 = ((div) - (baud0 << 7)); //subtract factor corrected div - integer parts

    MXC_SETFIELD(uart->baud0, MXC_F_UART_REVC_BAUD0_IBAUD, baud0);
    MXC_SETFIELD(uart->baud1, MXC_F_UART_REVC_BAUD1_DBAUD, baud1);

    return MXC_UART_GetFrequency((mxc_uart_regs_t *)uart);
}

int MXC_UART_RevC_GetFrequency(mxc_uart_revc_regs_t *uart)
{
    int div = 0;

    div = uart->baud0 << 7;
    div += uart->baud1;

    return div * PeripheralClock;
}

int MXC_UART_RevC_SetDataSize(mxc_uart_revc_regs_t *uart, int dataSize)
{
    if (dataSize < 5 || dataSize > 8) {
        return E_BAD_PARAM;
    }

    dataSize = (dataSize - 5) << MXC_F_UART_REVC_CTRL_CHAR_SIZE_POS;

    MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVC_CTRL_CHAR_SIZE, dataSize);

    return E_NO_ERROR;
}

int MXC_UART_RevC_SetStopBits(mxc_uart_revc_regs_t *uart, mxc_uart_stop_t stopBits)
{
    switch (stopBits) {
    case MXC_UART_STOP_1:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVC_CTRL_STOPBITS,
                     0 << MXC_F_UART_REVC_CTRL_STOPBITS_POS);
        break;

    case MXC_UART_STOP_2:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVC_CTRL_STOPBITS,
                     1 << MXC_F_UART_REVC_CTRL_STOPBITS_POS);
        break;

    default:
        return E_BAD_PARAM;
        break;
    }

    return E_NO_ERROR;
}

int MXC_UART_RevC_SetParity(mxc_uart_revc_regs_t *uart, mxc_uart_parity_t parity)
{
    switch (parity) {
    case MXC_UART_PARITY_DISABLE:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVC_CTRL_PARITY_EN,
                     0 << MXC_F_UART_REVC_CTRL_PARITY_EN_POS);
        break;

    case MXC_UART_PARITY_EVEN:
    case MXC_UART_PARITY_EVEN_0:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVC_CTRL_PARITY_EN,
                     1 << MXC_F_UART_REVC_CTRL_PARITY_EN_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVC_CTRL_PARITY, 0 << MXC_F_UART_REVC_CTRL_PARITY_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVC_CTRL_PARMD, 0 << MXC_F_UART_REVC_CTRL_PARMD_POS);
        break;

    case MXC_UART_PARITY_EVEN_1:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVC_CTRL_PARITY_EN,
                     1 << MXC_F_UART_REVC_CTRL_PARITY_EN_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVC_CTRL_PARITY, 0 << MXC_F_UART_REVC_CTRL_PARITY_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVC_CTRL_PARMD, 1 << MXC_F_UART_REVC_CTRL_PARMD_POS);
        break;

    case MXC_UART_PARITY_ODD:
    case MXC_UART_PARITY_ODD_0:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVC_CTRL_PARITY_EN,
                     1 << MXC_F_UART_REVC_CTRL_PARITY_EN_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVC_CTRL_PARITY, 1 << MXC_F_UART_REVC_CTRL_PARITY_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVC_CTRL_PARMD, 0 << MXC_F_UART_REVC_CTRL_PARMD_POS);
        break;

    case MXC_UART_PARITY_ODD_1:
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVC_CTRL_PARITY_EN,
                     1 << MXC_F_UART_REVC_CTRL_PARITY_EN_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVC_CTRL_PARITY, 1 << MXC_F_UART_REVC_CTRL_PARITY_POS);
        MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVC_CTRL_PARMD, 1 << MXC_F_UART_REVC_CTRL_PARMD_POS);
        break;

    default:
        return E_BAD_PARAM;
        break;
    }

    return E_NO_ERROR;
}

int MXC_UART_RevC_GetActive(mxc_uart_revc_regs_t *uart)
{
    if (uart->status & (MXC_F_UART_REVC_STATUS_TX_BUSY | MXC_F_UART_REVC_STATUS_RX_BUSY)) {
        return E_BUSY;
    }

    return E_NO_ERROR;
}

int MXC_UART_RevC_AbortTransmission(mxc_uart_revc_regs_t *uart)
{
    return MXC_UART_ClearTXFIFO((mxc_uart_regs_t *)uart);
}

int MXC_UART_RevC_ReadCharacterRaw(mxc_uart_revc_regs_t *uart)
{
    if (uart->status & MXC_F_UART_REVC_STATUS_RX_EMPTY) {
        return E_UNDERFLOW;
    }

    return uart->fifo;
}

int MXC_UART_RevC_WriteCharacterRaw(mxc_uart_revc_regs_t *uart, uint8_t character)
{
    // Return error if the FIFO is full
    if (uart->status & MXC_F_UART_REVC_STATUS_TX_FULL) {
        return E_OVERFLOW;
    }

    uart->fifo = character;

    return E_NO_ERROR;
}

int MXC_UART_RevC_Read(mxc_uart_revc_regs_t *uart, uint8_t *buffer, int *len)
{
    int read = 0;

    for (; read < *len; read++) {
        int retVal = MXC_UART_ReadCharacter((mxc_uart_regs_t *)uart);

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

int MXC_UART_RevC_Write(mxc_uart_revc_regs_t *uart, uint8_t *byte, int *len)
{
    int written = 0;

    for (; written < *len; written++) {
        int retVal = MXC_UART_WriteCharacter((mxc_uart_regs_t *)uart, byte[written]);

        if (retVal != E_NO_ERROR) {
            *len = written;
            return retVal;
        }
    }

    *len = written;
    return E_NO_ERROR;
}

unsigned int MXC_UART_RevC_ReadRXFIFO(mxc_uart_revc_regs_t *uart, unsigned char *bytes,
                                      unsigned int len)
{
    int read = 0;

    for (; read < len; read++) {
        if (uart->status & MXC_F_UART_REVC_STATUS_RX_EMPTY) {
            break;
        }

        bytes[read] = uart->fifo;
    }

    return read;
}

int MXC_UART_RevC_ReadRXFIFODMA(mxc_uart_revc_regs_t *uart, unsigned char *bytes, unsigned int len,
                                mxc_uart_dma_complete_cb_t callback, mxc_dma_config_t config)
{
    uint8_t channel;
    mxc_dma_srcdst_t srcdst;

    int uart_num = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);

    channel = MXC_DMA_AcquireChannel();

    config.ch = channel;

    config.srcwd = MXC_DMA_WIDTH_BYTE;
    config.dstwd = MXC_DMA_WIDTH_BYTE;

    config.srcinc_en = 0;
    config.dstinc_en = 1;

    srcdst.ch = channel;
    srcdst.dest = bytes;
    srcdst.len = len;

    states[uart_num].channelRx = channel;
    MXC_DMA_ConfigChannel(config, srcdst);
    MXC_DMA_SetCallback(channel, MXC_UART_DMACallback);
    MXC_DMA_EnableInt(channel);
    MXC_DMA_Start(channel);
    //MXC_DMA->ch[channel].ctrl |= MXC_F_DMA_CTRL_CTZ_IE;
    MXC_DMA_SetChannelInterruptEn(channel, 0, 1);
    uart->dma |= MXC_F_UART_REVC_DMA_RXDMA_EN;

    return E_NO_ERROR;
}

unsigned int MXC_UART_RevC_GetRXFIFOAvailable(mxc_uart_revc_regs_t *uart)
{
    return (uart->status & MXC_F_UART_REVC_STATUS_RX_FIFO_CNT) >>
           MXC_F_UART_REVC_STATUS_RX_FIFO_CNT_POS;
}

unsigned int MXC_UART_RevC_WriteTXFIFO(mxc_uart_revc_regs_t *uart, unsigned char *bytes,
                                       unsigned int len)
{
    int written = 0;

    for (; written < len; written++) {
        if (uart->status & MXC_F_UART_REVC_STATUS_TX_FULL) {
            break;
        }

        uart->fifo = bytes[written];
    }

    return written;
}

int MXC_UART_RevC_WriteTXFIFODMA(mxc_uart_revc_regs_t *uart, unsigned char *bytes, unsigned int len,
                                 mxc_uart_dma_complete_cb_t callback, mxc_dma_config_t config)
{
    uint8_t channel;
    mxc_dma_srcdst_t srcdst;

    int uart_num = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);

    channel = MXC_DMA_AcquireChannel();

    config.ch = channel;

    config.srcwd = MXC_DMA_WIDTH_BYTE;
    config.dstwd = MXC_DMA_WIDTH_BYTE;

    config.srcinc_en = 1;
    config.dstinc_en = 0;

    srcdst.ch = channel;
    srcdst.source = bytes;
    srcdst.len = len;

    states[uart_num].channelTx = channel;
    MXC_DMA_ConfigChannel(config, srcdst);
    MXC_DMA_SetCallback(channel, MXC_UART_DMACallback);
    MXC_DMA_EnableInt(channel);
    MXC_DMA_Start(channel);
    //MXC_DMA->ch[channel].ctrl |= MXC_F_DMA_CTRL_CTZ_IE;
    MXC_DMA_SetChannelInterruptEn(channel, 0, 1);
    uart->dma |= MXC_F_UART_REVC_DMA_TXDMA_EN;

    return E_NO_ERROR;
}

unsigned int MXC_UART_RevC_GetTXFIFOAvailable(mxc_uart_revc_regs_t *uart)
{
    int txCnt = (uart->status & MXC_F_UART_REVC_STATUS_TX_FIFO_CNT) >>
                MXC_F_UART_REVC_STATUS_TX_FIFO_CNT_POS;
    return MXC_UART_FIFO_DEPTH - txCnt;
}

int MXC_UART_RevC_ClearRXFIFO(mxc_uart_revc_regs_t *uart)
{
    uart->ctrl |= MXC_F_UART_REVC_CTRL_RX_FLUSH;

    while (uart->ctrl & MXC_F_UART_REVC_CTRL_RX_FLUSH) {}

    return E_NO_ERROR;
}

int MXC_UART_RevC_ClearTXFIFO(mxc_uart_revc_regs_t *uart)
{
    uart->ctrl |= MXC_F_UART_REVC_CTRL_TX_FLUSH;

    while (uart->ctrl & MXC_F_UART_REVC_CTRL_TX_FLUSH) {}

    return E_NO_ERROR;
}

int MXC_UART_RevC_SetRXThreshold(mxc_uart_revc_regs_t *uart, unsigned int numBytes)
{
    if (numBytes < 1 || numBytes > MXC_UART_FIFO_DEPTH) {
        return E_BAD_PARAM;
    }

    numBytes <<= MXC_F_UART_REVC_CTRL_RXTHD_POS;
    MXC_SETFIELD(uart->ctrl, MXC_F_UART_REVC_CTRL_RXTHD, numBytes);

    return E_NO_ERROR;
}

unsigned int MXC_UART_RevC_GetRXThreshold(mxc_uart_revc_regs_t *uart)
{
    return (uart->ctrl & MXC_F_UART_REVC_CTRL_RXTHD) >> MXC_F_UART_REVC_CTRL_RXTHD_POS;
}

unsigned int MXC_UART_RevC_GetFlags(mxc_uart_revc_regs_t *uart)
{
    return uart->int_fl;
}

int MXC_UART_RevC_ClearFlags(mxc_uart_revc_regs_t *uart, unsigned int flags)
{
    uart->int_fl &= ~flags;

    return E_NO_ERROR;
}

int MXC_UART_RevC_EnableInt(mxc_uart_revc_regs_t *uart, unsigned int intEn)
{
    uart->int_en |= intEn;

    return E_NO_ERROR;
}

int MXC_UART_RevC_DisableInt(mxc_uart_revc_regs_t *uart, unsigned int intDis)
{
    uart->int_en &= ~intDis;

    return E_NO_ERROR;
}

unsigned int MXC_UART_RevC_GetStatus(mxc_uart_revc_regs_t *uart)
{
    return uart->status;
}

int MXC_UART_RevC_Transaction(mxc_uart_revc_req_t *req)
{
    int numToWrite, numToRead;

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
                     MXC_F_UART_REVC_INT_FL_TX_FIFO_HALF_EMPTY)) {}

            numToWrite = MXC_UART_GetTXFIFOAvailable((mxc_uart_regs_t *)(req->uart));
            numToWrite = numToWrite > (req->txLen - req->txCnt) ? req->txLen - req->txCnt :
                                                                  numToWrite;
            req->txCnt += MXC_UART_WriteTXFIFO((mxc_uart_regs_t *)(req->uart),
                                               &req->txData[req->txCnt], numToWrite);
            MXC_UART_ClearFlags((mxc_uart_regs_t *)(req->uart),
                                MXC_F_UART_REVC_INT_FL_TX_FIFO_HALF_EMPTY);
        }
    }

    if (req->rxLen) {
        numToRead = MXC_UART_GetRXFIFOAvailable((mxc_uart_regs_t *)(req->uart));
        numToRead = numToRead > (req->rxLen - req->rxCnt) ? req->rxLen - req->rxCnt : numToRead;
        req->rxCnt += MXC_UART_ReadRXFIFO((mxc_uart_regs_t *)(req->uart), &req->rxData[req->rxCnt],
                                          numToRead);

        while (req->rxCnt < req->rxLen) {
            while (!(MXC_UART_GetFlags((mxc_uart_regs_t *)(req->uart)) &
                     MXC_F_UART_REVC_INT_FL_RX_FIFO_THRESH)) {}

            numToRead = MXC_UART_GetRXFIFOAvailable((mxc_uart_regs_t *)(req->uart));
            numToRead = numToRead > (req->rxLen - req->rxCnt) ? req->rxLen - req->rxCnt : numToRead;
            req->rxCnt += MXC_UART_ReadRXFIFO((mxc_uart_regs_t *)(req->uart),
                                              &req->rxData[req->rxCnt], numToRead);
            MXC_UART_ClearFlags((mxc_uart_regs_t *)(req->uart),
                                MXC_F_UART_REVC_INT_FL_RX_FIFO_THRESH);
        }
    }

    return E_NO_ERROR;
}

int MXC_UART_RevC_TransactionAsync(mxc_uart_revc_req_t *req)
{
    unsigned int intEn;
    int numToWrite, numToRead;

    if (MXC_UART_GET_IDX((mxc_uart_regs_t *)(req->uart)) < 0) {
        return E_BAD_PARAM;
    }

    MXC_UART_DisableInt((mxc_uart_regs_t *)(req->uart), 0xFFFFFFFF);
    MXC_UART_ClearFlags((mxc_uart_regs_t *)(req->uart), 0xFFFFFFFF);

    MXC_UART_ClearRXFIFO((mxc_uart_regs_t *)(req->uart));
    MXC_UART_ClearTXFIFO((mxc_uart_regs_t *)(req->uart));

    req->txCnt = 0;
    req->rxCnt = 0;

    // Check for bad requests.
    if (req->txLen) {
        if (req->txData == NULL) {
            MXC_UART_DisableInt((mxc_uart_regs_t *)(req->uart), 0xFFFFFFFF);
            return E_BAD_PARAM;
        }
    }

    if (req->rxLen) {
        if (req->rxData == NULL) {
            MXC_UART_DisableInt((mxc_uart_regs_t *)(req->uart), 0xFFFFFFFF);
            return E_BAD_PARAM;
        }
    }

    // Register request.
    MXC_ASSERT(AsyncRequests[MXC_UART_GET_IDX((mxc_uart_regs_t *)(req->uart))] == NULL);
    AsyncRequests[MXC_UART_GET_IDX((mxc_uart_regs_t *)(req->uart))] = (void *)req;

    // Select interrupts to enable atomically.
    // returning success by default.
    intEn = 0;

    // Be it a tx fifo interrupt,
    if (req->txLen) {
        intEn |= MXC_F_UART_REVC_INT_EN_TX_FIFO_HALF_EMPTY;
        numToWrite = MXC_UART_GetTXFIFOAvailable((mxc_uart_regs_t *)(req->uart));
        numToWrite = numToWrite > (req->txLen - req->txCnt) ? req->txLen - req->txCnt : numToWrite;
        req->txCnt += MXC_UART_WriteTXFIFO((mxc_uart_regs_t *)(req->uart), &req->txData[req->txCnt],
                                           numToWrite);
    }

    // Or an rx interrupt.
    if (req->rxLen) {
        // All error interrupts are related to RX
        intEn |= MXC_UART_REVC_ERRINT_EN;
        intEn |= MXC_F_UART_REVC_INT_EN_RX_FIFO_THRESH;
        numToRead = MXC_UART_GetRXFIFOAvailable((mxc_uart_regs_t *)(req->uart));
        numToRead = numToRead > (req->rxLen - req->rxCnt) ? req->rxLen - req->rxCnt : numToRead;
        req->rxCnt += MXC_UART_ReadRXFIFO((mxc_uart_regs_t *)(req->uart), &req->rxData[req->rxCnt],
                                          numToRead);
        MXC_UART_ClearFlags((mxc_uart_regs_t *)(req->uart), MXC_F_UART_REVC_INT_FL_RX_FIFO_THRESH);
    }

    // Enable all interrupts at the same time.
    MXC_UART_EnableInt((mxc_uart_regs_t *)(req->uart), intEn);

    return E_NO_ERROR;
}

int MXC_UART_RevC_TransactionDMA(mxc_uart_revc_req_t *req)
{
    int uart_num = MXC_UART_GET_IDX((mxc_uart_regs_t *)(req->uart));

    MXC_ASSERT(uart_num >= 0)

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

    MXC_UART_ClearRXFIFO((mxc_uart_regs_t *)(req->uart));
    MXC_UART_ClearTXFIFO((mxc_uart_regs_t *)(req->uart));

    (req->uart)->dma |=
        (1 << MXC_F_UART_REVC_DMA_RXDMA_LEVEL_POS); // Set RX DMA threshold to 1 byte
    (req->uart)->dma |=
        (2 << MXC_F_UART_REVC_DMA_TXDMA_LEVEL_POS); // Set TX DMA threshold to 2 bytes

    MXC_DMA_Init();

    //tx
    if ((req->txData != NULL) && (req->txLen)) {
        if (MXC_UART_WriteTXFIFODMA((mxc_uart_regs_t *)(req->uart), req->txData, req->txLen,
                                    NULL) != E_NO_ERROR) {
            return E_BAD_PARAM;
        }
    }

    if ((req->rxData != NULL) && (req->rxLen)) {
        if (MXC_UART_ReadRXFIFODMA((mxc_uart_regs_t *)(req->uart), req->rxData, req->rxLen, NULL) !=
            E_NO_ERROR) {
            return E_BAD_PARAM;
        }
    }

    return E_NO_ERROR;
}

void MXC_UART_RevC_DMACallback(int ch, int error)
{
    mxc_uart_revc_req_t *temp_req;

    for (int i = 0; i < MXC_UART_INSTANCES; i++) {
        if (states[i].channelTx == ch) {
            //save the request
            temp_req = states[i].req;
            // Callback if not NULL
            if (temp_req->callback != NULL) {
                temp_req->callback((mxc_uart_req_t *)temp_req, E_NO_ERROR);
            }
            break;
        } else if (states[i].channelRx == ch) {
            //save the request
            temp_req = states[i].req;
            // Callback if not NULL
            if (temp_req->callback != NULL) {
                temp_req->callback((mxc_uart_req_t *)temp_req, E_NO_ERROR);
            }
            break;
        }
    }
}

int MXC_UART_RevC_AsyncCallback(mxc_uart_revc_regs_t *uart, int retVal)
{
    mxc_uart_revc_req_t *req =
        (mxc_uart_revc_req_t *)AsyncRequests[MXC_UART_GET_IDX((mxc_uart_regs_t *)uart)];

    if (req->callback != NULL) {
        req->callback((mxc_uart_req_t *)req, retVal);
    }

    return E_NO_ERROR;
}

int MXC_UART_RevC_AsyncStop(mxc_uart_revc_regs_t *uart)
{
    MXC_UART_DisableInt((mxc_uart_regs_t *)uart, 0xFFFFFFFF);
    AsyncRequests[MXC_UART_GET_IDX((mxc_uart_regs_t *)uart)] = NULL;

    return E_NO_ERROR;
}

int MXC_UART_RevC_AbortAsync(mxc_uart_revc_regs_t *uart)
{
    MXC_UART_AsyncCallback((mxc_uart_regs_t *)uart, E_ABORT);
    MXC_UART_AsyncStop((mxc_uart_regs_t *)uart);

    return E_NO_ERROR;
}

int MXC_UART_RevC_AsyncHandler(mxc_uart_revc_regs_t *uart)
{
    int uart_num = MXC_UART_GET_IDX((mxc_uart_regs_t *)uart);
    int flags, numToWrite, numToRead;
    mxc_uart_revc_req_t *req;

    MXC_ASSERT(uart_num >= 0)

    req = (mxc_uart_revc_req_t *)AsyncRequests[uart_num];

    flags = MXC_UART_GetFlags((mxc_uart_regs_t *)uart);

    if (flags & MXC_UART_REVC_ERRINT_FL & uart->int_en) {
        MXC_UART_AsyncCallback((mxc_uart_regs_t *)uart, E_COMM_ERR);
        MXC_UART_AsyncStop((mxc_uart_regs_t *)uart);
        return E_INVALID;
    }

    if ((flags & MXC_F_UART_REVC_INT_FL_TX_FIFO_HALF_EMPTY) && (req->txLen)) {
        numToWrite = MXC_UART_GetTXFIFOAvailable((mxc_uart_regs_t *)(req->uart));
        numToWrite = numToWrite > (req->txLen - req->txCnt) ? req->txLen - req->txCnt : numToWrite;
        req->txCnt += MXC_UART_WriteTXFIFO((mxc_uart_regs_t *)(req->uart), &req->txData[req->txCnt],
                                           numToWrite);
        MXC_UART_ClearFlags((mxc_uart_regs_t *)(req->uart),
                            MXC_F_UART_REVC_INT_FL_TX_FIFO_HALF_EMPTY);
    }

    if ((flags & MXC_F_UART_REVC_INT_FL_RX_FIFO_THRESH) && (req->rxLen)) {
        numToRead = MXC_UART_GetRXFIFOAvailable((mxc_uart_regs_t *)(req->uart));
        numToRead = numToRead > (req->rxLen - req->rxCnt) ? req->rxLen - req->rxCnt : numToRead;
        req->rxCnt += MXC_UART_ReadRXFIFO((mxc_uart_regs_t *)(req->uart), &req->rxData[req->rxCnt],
                                          numToRead);

        if ((req->rxLen - req->rxCnt) < MXC_UART_GetRXThreshold((mxc_uart_regs_t *)(req->uart))) {
            MXC_UART_SetRXThreshold((mxc_uart_regs_t *)(req->uart), req->rxLen - req->rxCnt);
        }

        MXC_UART_ClearFlags((mxc_uart_regs_t *)(req->uart), MXC_F_UART_REVC_INT_FL_RX_FIFO_THRESH);
    }

    if ((req->rxCnt == req->rxLen) && (req->txCnt == req->txLen)) {
        MXC_UART_AsyncCallback((mxc_uart_regs_t *)uart, E_NO_ERROR);
        MXC_UART_AsyncStop((mxc_uart_regs_t *)uart);
    }

    return E_NO_ERROR;
}
