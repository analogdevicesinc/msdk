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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_UART_UART_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_UART_UART_REVA_H_

#include "mxc_device.h"
#include "mxc_assert.h"
#include "dma.h"
#include "uart_reva_regs.h"
#include "uart_regs.h"

typedef struct _mxc_uart_reva_req_t mxc_uart_reva_req_t;

struct _mxc_uart_reva_req_t {
    mxc_uart_reva_regs_t *uart;
    uint8_t *txData;
    uint8_t *rxData;
    uint32_t txLen;
    uint32_t rxLen;
    uint32_t txCnt;
    uint32_t rxCnt;
    mxc_uart_complete_cb_t callback;
};

int MXC_UART_RevA_Init(mxc_uart_reva_regs_t *uart, unsigned int baud);
int MXC_UART_RevA_Shutdown(mxc_uart_reva_regs_t *uart);
int MXC_UART_RevA_ReadyForSleep(mxc_uart_reva_regs_t *uart);
int MXC_UART_RevA_SetFrequency(mxc_uart_reva_regs_t *uart, unsigned int baud);
int MXC_UART_RevA_GetFrequency(mxc_uart_reva_regs_t *uart);
int MXC_UART_RevA_SetDataSize(mxc_uart_reva_regs_t *uart, int dataSize);
int MXC_UART_RevA_SetStopBits(mxc_uart_reva_regs_t *uart, mxc_uart_stop_t stopBits);
int MXC_UART_RevA_SetParity(mxc_uart_reva_regs_t *uart, mxc_uart_parity_t parity);
int MXC_UART_RevA_SetFlowCtrl(mxc_uart_reva_regs_t *uart, mxc_uart_flow_t flowCtrl,
                              int rtsThreshold);
int MXC_UART_RevA_SetClockSource(mxc_uart_reva_regs_t *uart, int usePCLK);
int MXC_UART_RevA_SetNullModem(mxc_uart_reva_regs_t *uart, int nullModem);
int MXC_UART_RevA_SendBreak(mxc_uart_reva_regs_t *uart);
int MXC_UART_RevA_GetActive(mxc_uart_reva_regs_t *uart);
int MXC_UART_RevA_AbortTransmission(mxc_uart_reva_regs_t *uart);
int MXC_UART_RevA_ReadCharacterRaw(mxc_uart_reva_regs_t *uart);
int MXC_UART_RevA_WriteCharacterRaw(mxc_uart_reva_regs_t *uart, uint8_t character);
int MXC_UART_RevA_Read(mxc_uart_reva_regs_t *uart, uint8_t *buffer, int *len);
int MXC_UART_RevA_Write(mxc_uart_reva_regs_t *uart, uint8_t *byte, int *len);
unsigned int MXC_UART_RevA_ReadRXFIFO(mxc_uart_reva_regs_t *uart, unsigned char *bytes,
                                      unsigned int len);
int MXC_UART_RevA_ReadRXFIFODMA(mxc_uart_reva_regs_t *uart, mxc_dma_regs_t *dma,
                                unsigned char *bytes, unsigned int len,
                                mxc_uart_dma_complete_cb_t callback, mxc_dma_config_t config);
unsigned int MXC_UART_RevA_GetRXFIFOAvailable(mxc_uart_reva_regs_t *uart);
unsigned int MXC_UART_RevA_WriteTXFIFO(mxc_uart_reva_regs_t *uart, unsigned char *bytes,
                                       unsigned int len);
unsigned int MXC_UART_RevA_WriteTXFIFODMA(mxc_uart_reva_regs_t *uart, mxc_dma_regs_t *dma,
                                          unsigned char *bytes, unsigned int len,
                                          mxc_uart_dma_complete_cb_t callback,
                                          mxc_dma_config_t config);
unsigned int MXC_UART_RevA_GetTXFIFOAvailable(mxc_uart_reva_regs_t *uart);
int MXC_UART_RevA_ClearRXFIFO(mxc_uart_reva_regs_t *uart);
int MXC_UART_RevA_ClearTXFIFO(mxc_uart_reva_regs_t *uart);
int MXC_UART_RevA_SetRXThreshold(mxc_uart_reva_regs_t *uart, unsigned int numBytes);
unsigned int MXC_UART_RevA_GetRXThreshold(mxc_uart_reva_regs_t *uart);
int MXC_UART_RevA_SetTXThreshold(mxc_uart_reva_regs_t *uart, unsigned int numBytes);
unsigned int MXC_UART_RevA_GetTXThreshold(mxc_uart_reva_regs_t *uart);
unsigned int MXC_UART_RevA_GetFlags(mxc_uart_reva_regs_t *uart);
int MXC_UART_RevA_ClearFlags(mxc_uart_reva_regs_t *uart, unsigned int flags);
int MXC_UART_RevA_EnableInt(mxc_uart_reva_regs_t *uart, unsigned int mask);
int MXC_UART_RevA_DisableInt(mxc_uart_reva_regs_t *uart, unsigned int mask);
unsigned int MXC_UART_RevA_GetStatus(mxc_uart_reva_regs_t *uart);
int MXC_UART_RevA_Busy(mxc_uart_reva_regs_t *uart);
int MXC_UART_RevA_Transaction(mxc_uart_reva_req_t *req);
int MXC_UART_RevA_TransactionAsync(mxc_uart_reva_req_t *req);
int MXC_UART_RevA_TransactionDMA(mxc_uart_reva_req_t *req, mxc_dma_regs_t *dma);
int MXC_UART_RevA_TxAbortAsync(mxc_uart_reva_regs_t *uart);
int MXC_UART_RevA_RxAbortAsync(mxc_uart_reva_regs_t *uart);
int MXC_UART_RevA_AbortAsync(mxc_uart_reva_regs_t *uart);
int MXC_UART_RevA_AsyncHandler(mxc_uart_reva_regs_t *uart);
int MXC_UART_RevA_TxAsyncStop(mxc_uart_reva_regs_t *uart);
int MXC_UART_RevA_RxAsyncStop(mxc_uart_reva_regs_t *uart);
int MXC_UART_RevA_AsyncStop(mxc_uart_reva_regs_t *uart);
int MXC_UART_RevA_AsyncCallback(mxc_uart_reva_regs_t *uart, int retVal);
int MXC_UART_RevA_TxAsyncCallback(mxc_uart_reva_regs_t *uart, int retVal);
int MXC_UART_RevA_RxAsyncCallback(mxc_uart_reva_regs_t *uart, int retVal);
void MXC_UART_RevA_DMACallback(int ch, int error);

int MXC_UART_RevA_SetAutoDMAHandlers(mxc_uart_reva_regs_t *uart, bool enable);
int MXC_UART_RevA_SetTXDMAChannel(mxc_uart_reva_regs_t *uart, unsigned int channel);
int MXC_UART_RevA_GetTXDMAChannel(mxc_uart_reva_regs_t *uart);
int MXC_UART_RevA_SetRXDMAChannel(mxc_uart_reva_regs_t *uart, unsigned int channel);
int MXC_UART_RevA_GetRXDMAChannel(mxc_uart_reva_regs_t *uart);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_UART_UART_REVA_H_
