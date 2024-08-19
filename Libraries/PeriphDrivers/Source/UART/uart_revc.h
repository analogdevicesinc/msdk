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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_UART_UART_REVC_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_UART_UART_REVC_H_

#include "mxc_device.h"
#include "mxc_assert.h"
#include "dma.h"
#include "uart_revc_regs.h"
#include "uart_regs.h"

typedef struct _mxc_uart_revc_req_t mxc_uart_revc_req_t;

struct _mxc_uart_revc_req_t {
    mxc_uart_revc_regs_t *uart;
    uint8_t *txData;
    uint8_t *rxData;
    uint32_t txLen;
    uint32_t rxLen;
    uint32_t txCnt;
    uint32_t rxCnt;
    mxc_uart_complete_cb_t callback;
};

int MXC_UART_RevC_Init(mxc_uart_revc_regs_t *uart, unsigned int baud);
int MXC_UART_RevC_Shutdown(mxc_uart_revc_regs_t *uart);
int MXC_UART_RevC_ReadyForSleep(mxc_uart_revc_regs_t *uart);
int MXC_UART_RevC_SetFrequency(mxc_uart_revc_regs_t *uart, unsigned int baud);
int MXC_UART_RevC_GetFrequency(mxc_uart_revc_regs_t *uart);
int MXC_UART_RevC_SetDataSize(mxc_uart_revc_regs_t *uart, int dataSize);
int MXC_UART_RevC_SetStopBits(mxc_uart_revc_regs_t *uart, mxc_uart_stop_t stopBits);
int MXC_UART_RevC_SetParity(mxc_uart_revc_regs_t *uart, mxc_uart_parity_t parity);
int MXC_UART_RevC_GetActive(mxc_uart_revc_regs_t *uart);
int MXC_UART_RevC_AbortTransmission(mxc_uart_revc_regs_t *uart);
int MXC_UART_RevC_ReadCharacterRaw(mxc_uart_revc_regs_t *uart);
int MXC_UART_RevC_WriteCharacterRaw(mxc_uart_revc_regs_t *uart, uint8_t character);
int MXC_UART_RevC_Read(mxc_uart_revc_regs_t *uart, uint8_t *buffer, int *len);
int MXC_UART_RevC_Write(mxc_uart_revc_regs_t *uart, uint8_t *byte, int *len);
unsigned int MXC_UART_RevC_ReadRXFIFO(mxc_uart_revc_regs_t *uart, unsigned char *bytes,
                                      unsigned int len);
int MXC_UART_RevC_ReadRXFIFODMA(mxc_uart_revc_regs_t *uart, unsigned char *bytes, unsigned int len,
                                mxc_uart_dma_complete_cb_t callback, mxc_dma_config_t config);
unsigned int MXC_UART_RevC_GetRXFIFOAvailable(mxc_uart_revc_regs_t *uart);
unsigned int MXC_UART_RevC_WriteTXFIFO(mxc_uart_revc_regs_t *uart, unsigned char *bytes,
                                       unsigned int len);
int MXC_UART_RevC_WriteTXFIFODMA(mxc_uart_revc_regs_t *uart, unsigned char *bytes, unsigned int len,
                                 mxc_uart_dma_complete_cb_t callback, mxc_dma_config_t config);
unsigned int MXC_UART_RevC_GetTXFIFOAvailable(mxc_uart_revc_regs_t *uart);
int MXC_UART_RevC_ClearRXFIFO(mxc_uart_revc_regs_t *uart);
int MXC_UART_RevC_ClearTXFIFO(mxc_uart_revc_regs_t *uart);
int MXC_UART_RevC_SetRXThreshold(mxc_uart_revc_regs_t *uart, unsigned int numBytes);
unsigned int MXC_UART_RevC_GetRXThreshold(mxc_uart_revc_regs_t *uart);
unsigned int MXC_UART_RevC_GetFlags(mxc_uart_revc_regs_t *uart);
int MXC_UART_RevC_ClearFlags(mxc_uart_revc_regs_t *uart, unsigned int flags);
int MXC_UART_RevC_EnableInt(mxc_uart_revc_regs_t *uart, unsigned int mask);
int MXC_UART_RevC_DisableInt(mxc_uart_revc_regs_t *uart, unsigned int mask);
unsigned int MXC_UART_RevC_GetStatus(mxc_uart_revc_regs_t *uart);
int MXC_UART_RevC_Transaction(mxc_uart_revc_req_t *req);
int MXC_UART_RevC_TransactionAsync(mxc_uart_revc_req_t *req);
int MXC_UART_RevC_TransactionDMA(mxc_uart_revc_req_t *req);
int MXC_UART_RevC_AbortAsync(mxc_uart_revc_regs_t *uart);
int MXC_UART_RevC_AsyncHandler(mxc_uart_revc_regs_t *uart);
int MXC_UART_RevC_AsyncStop(mxc_uart_revc_regs_t *uart);
int MXC_UART_RevC_AsyncCallback(mxc_uart_revc_regs_t *uart, int retVal);
void MXC_UART_RevC_DMACallback(int ch, int error);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_UART_UART_REVC_H_
