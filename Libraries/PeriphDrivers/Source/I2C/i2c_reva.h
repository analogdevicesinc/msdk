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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_I2C_I2C_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_I2C_I2C_REVA_H_

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "i2c_regs.h"
#include "i2c_reva_regs.h"
#include "dma.h"
#include "dma_reva_regs.h"

/* **** Definitions **** */
#define MXC_I2C_REVA_MAX_ADDR_WIDTH 0x7F
#define MXC_I2C_REVA_STD_MODE 100000
#define MXC_I2C_REVA_FAST_SPEED 400000
#define MXC_I2C_REVA_FASTPLUS_SPEED 1000000
#define MXC_I2C_REVA_HIGH_SPEED 3400000

#define MXC_I2C_REVA_INTFL0_MASK 0x00FFFFFF
#define MXC_I2C_REVA_INTFL1_MASK 0x00000007

#define MXC_I2C_REVA_MAX_FIFO_TRANSACTION 256

#define MXC_I2C_REVA_ERROR                                                  \
    (MXC_F_I2C_REVA_INTFL0_ARB_ERR | MXC_F_I2C_REVA_INTFL0_TO_ERR |         \
     MXC_F_I2C_REVA_INTFL0_ADDR_NACK_ERR | MXC_F_I2C_REVA_INTFL0_DATA_ERR | \
     MXC_F_I2C_REVA_INTFL0_DNR_ERR | MXC_F_I2C_REVA_INTFL0_START_ERR |      \
     MXC_F_I2C_REVA_INTFL0_STOP_ERR)

typedef struct _i2c_reva_req_t mxc_i2c_reva_req_t;
typedef int (*mxc_i2c_reva_getAck_t)(mxc_i2c_reva_regs_t *i2c, unsigned char byte);
typedef void (*mxc_i2c_reva_complete_cb_t)(mxc_i2c_reva_req_t *req, int result);
typedef void (*mxc_i2c_reva_dma_complete_cb_t)(int len, int result);
struct _i2c_reva_req_t {
    mxc_i2c_reva_regs_t *i2c;
    unsigned int addr;
    unsigned char *tx_buf;
    unsigned int tx_len;
    unsigned char *rx_buf;
    unsigned int rx_len;
    int restart;
    mxc_i2c_reva_complete_cb_t callback;
};
typedef enum {
    MXC_I2C_REVA_EVT_MASTER_WR,
    MXC_I2C_REVA_EVT_MASTER_RD,
    MXC_I2C_REVA_EVT_RX_THRESH,
    MXC_I2C_REVA_EVT_TX_THRESH,
    MXC_I2C_REVA_EVT_TRANS_COMP,
    MXC_I2C_REVA_EVT_UNDERFLOW,
    MXC_I2C_REVA_EVT_OVERFLOW,
} mxc_i2c_reva_slave_event_t;
typedef int (*mxc_i2c_reva_slave_handler_t)(mxc_i2c_reva_regs_t *i2c,
                                            mxc_i2c_reva_slave_event_t event, void *data);
/* **** Variable Declaration **** */

extern void *AsyncRequests[MXC_I2C_INSTANCES];

/* **** Function Prototypes **** */

/* ************************************************************************* */
/* Control/Configuration functions                                           */
/* ************************************************************************* */
int MXC_I2C_RevA_Init(mxc_i2c_reva_regs_t *i2c, int masterMode, unsigned int slaveAddr);
int MXC_I2C_RevA_SetSlaveAddr(mxc_i2c_reva_regs_t *i2c, unsigned int slaveAddr, int idx);
int MXC_I2C_RevA_Shutdown(mxc_i2c_reva_regs_t *i2c);
int MXC_I2C_RevA_SetFrequency(mxc_i2c_reva_regs_t *i2c, unsigned int hz);
unsigned int MXC_I2C_RevA_GetFrequency(mxc_i2c_reva_regs_t *i2c);
int MXC_I2C_RevA_ReadyForSleep(mxc_i2c_reva_regs_t *i2c);
int MXC_I2C_RevA_SetClockStretching(mxc_i2c_reva_regs_t *i2c, int enable);
int MXC_I2C_RevA_GetClockStretching(mxc_i2c_reva_regs_t *i2c);
int MXC_I2C_RevA_DMA_Init(mxc_i2c_reva_regs_t *i2c, mxc_dma_reva_regs_t *dma, bool use_dma_tx,
                          bool use_dma_rx);
int MXC_I2C_RevA_DMA_GetTXChannel(mxc_i2c_reva_regs_t *i2c);
int MXC_I2C_RevA_DMA_GetRXChannel(mxc_i2c_reva_regs_t *i2c);
int MXC_I2C_RevA_DMA_SetRequestSelect(mxc_i2c_reva_regs_t *i2c, mxc_dma_reva_regs_t *dma,
                                      uint32_t txReqSel, uint32_t rxReqSel);

/* ************************************************************************* */
/* Low-level functions                                                       */
/* ************************************************************************* */
int MXC_I2C_RevA_Start(mxc_i2c_reva_regs_t *i2c);
int MXC_I2C_RevA_Stop(mxc_i2c_reva_regs_t *i2c);
int MXC_I2C_RevA_WriteByte(mxc_i2c_reva_regs_t *i2c, unsigned char byte);
int MXC_I2C_RevA_ReadByte(mxc_i2c_reva_regs_t *i2c, unsigned char *byte, int ack);
int MXC_I2C_RevA_ReadByteInteractive(mxc_i2c_reva_regs_t *i2c, unsigned char *byte,
                                     mxc_i2c_reva_getAck_t getAck);
int MXC_I2C_RevA_Write(mxc_i2c_reva_regs_t *i2c, unsigned char *bytes, unsigned int *len);
int MXC_I2C_RevA_Read(mxc_i2c_reva_regs_t *i2c, unsigned char *bytes, unsigned int *len, int ack);
int MXC_I2C_RevA_ReadRXFIFO(mxc_i2c_reva_regs_t *i2c, volatile unsigned char *bytes,
                            unsigned int len);
int MXC_I2C_RevA_ReadRXFIFODMA(mxc_i2c_reva_regs_t *i2c, unsigned char *bytes, unsigned int len,
                               mxc_dma_regs_t *dma);
int MXC_I2C_RevA_GetRXFIFOAvailable(mxc_i2c_reva_regs_t *i2c);
int MXC_I2C_RevA_WriteTXFIFO(mxc_i2c_reva_regs_t *i2c, volatile unsigned char *bytes,
                             unsigned int len);
int MXC_I2C_RevA_WriteTXFIFODMA(mxc_i2c_reva_regs_t *i2c, unsigned char *bytes, unsigned int len,
                                mxc_dma_regs_t *dma);
int MXC_I2C_RevA_GetTXFIFOAvailable(mxc_i2c_reva_regs_t *i2c);
void MXC_I2C_RevA_ClearRXFIFO(mxc_i2c_reva_regs_t *i2c);
void MXC_I2C_RevA_ClearTXFIFO(mxc_i2c_reva_regs_t *i2c);
int MXC_I2C_RevA_GetFlags(mxc_i2c_reva_regs_t *i2c, unsigned int *flags0, unsigned int *flags1);
void MXC_I2C_RevA_ClearFlags(mxc_i2c_reva_regs_t *i2c, unsigned int flags0, unsigned int flags1);
void MXC_I2C_RevA_EnableInt(mxc_i2c_reva_regs_t *i2c, unsigned int flags0, unsigned int flags1);
void MXC_I2C_RevA_DisableInt(mxc_i2c_reva_regs_t *i2c, unsigned int flags0, unsigned int flags1);
void MXC_I2C_RevA_EnablePreload(mxc_i2c_reva_regs_t *i2c);
void MXC_I2C_RevA_DisablePreload(mxc_i2c_reva_regs_t *i2c);
void MXC_I2C_RevA_EnableGeneralCall(mxc_i2c_reva_regs_t *i2c);
void MXC_I2C_RevA_DisableGeneralCall(mxc_i2c_reva_regs_t *i2c);
void MXC_I2C_RevA_SetTimeout(mxc_i2c_reva_regs_t *i2c, unsigned int timeout);
unsigned int MXC_I2C_RevA_GetTimeout(mxc_i2c_reva_regs_t *i2c);
int MXC_I2C_RevA_Recover(mxc_i2c_reva_regs_t *i2c, unsigned int retries);

/* ************************************************************************* */
/* Transaction level functions                                               */
/* ************************************************************************* */
int MXC_I2C_RevA_MasterTransaction(mxc_i2c_reva_req_t *req);
int MXC_I2C_RevA_MasterTransactionAsync(mxc_i2c_reva_req_t *req);
int MXC_I2C_RevA_MasterTransactionDMA(mxc_i2c_reva_req_t *req, mxc_dma_regs_t *dma);
int MXC_I2C_RevA_SlaveTransaction(mxc_i2c_reva_regs_t *i2c, mxc_i2c_reva_slave_handler_t callback,
                                  uint32_t interruptCheck);
int MXC_I2C_RevA_SlaveTransactionAsync(mxc_i2c_reva_regs_t *i2c,
                                       mxc_i2c_reva_slave_handler_t callback,
                                       uint32_t interruptCheck);
int MXC_I2C_RevA_SetRXThreshold(mxc_i2c_reva_regs_t *i2c, unsigned int numBytes);
unsigned int MXC_I2C_RevA_GetRXThreshold(mxc_i2c_reva_regs_t *i2c);
int MXC_I2C_RevA_SetTXThreshold(mxc_i2c_reva_regs_t *i2c, unsigned int numBytes);
unsigned int MXC_I2C_RevA_GetTXThreshold(mxc_i2c_reva_regs_t *i2c);
void MXC_I2C_RevA_AsyncCallback(mxc_i2c_reva_regs_t *i2c, int retVal);
void MXC_I2C_RevA_AsyncStop(mxc_i2c_reva_regs_t *i2c);
void MXC_I2C_RevA_AbortAsync(mxc_i2c_reva_regs_t *i2c);
void MXC_I2C_RevA_MasterAsyncHandler(int i2cNum);
void MXC_I2C_RevA_SlaveAsyncHandler(mxc_i2c_reva_regs_t *i2c, mxc_i2c_reva_slave_handler_t callback,
                                    uint32_t *int_en, int *retVal);
void MXC_I2C_RevA_AsyncHandler(mxc_i2c_reva_regs_t *i2c, uint32_t interruptCheck);
void MXC_I2C_RevA_DMACallback(int ch, int error);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_I2C_I2C_REVA_H_
