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
#include <stddef.h>
#include <stdint.h>
#include "mxc_errors.h"
#include "mxc_delay.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "i2c_revb.h"
#include "i2c_reva.h"

/* **** Definitions **** */

/* **** Variable Declaration **** */

// Saves the state of the non-blocking requests
typedef struct {
    mxc_i2c_revb_req_t *req;
    int master; // 1 for Master, 0 for slave
    int channelTx; // DMA channel for TX transaction
    int channelRx; // DMA channel for RX transaction
    int writeDone; // Write done flag
    int readDone; // Flag done flag
} mxc_i2c_revb_req_state_t;

/* ************************************************************************* */
/* Control/Configuration functions                                           */
/* ************************************************************************* */

int MXC_I2C_RevB_Init(mxc_i2c_revb_regs_t *i2c, int masterMode, unsigned int slaveAddr)
{
    return MXC_I2C_RevA_Init((mxc_i2c_reva_regs_t *)i2c, masterMode, slaveAddr);
}

int MXC_I2C_RevB_SetSlaveAddr(mxc_i2c_revb_regs_t *i2c, unsigned int slaveAddr, int idx)
{
    if (i2c == NULL || slaveAddr > MXC_F_I2C_REVB_SLV_ADDR_SLA || idx != 0) {
        return E_BAD_PARAM;
    }

    i2c->slv_addr = 0;

    if (slaveAddr > MXC_I2C_REVB_MAX_ADDR_WIDTH) {
        i2c->slv_addr |= MXC_S_I2C_REVB_SLV_ADDR_EA_10BIT_ADDR;
    }

    i2c->slv_addr |= slaveAddr & MXC_F_I2C_REVB_SLV_ADDR_SLA;

    return E_NO_ERROR;
}

int MXC_I2C_RevB_Shutdown(mxc_i2c_revb_regs_t *i2c)
{
    return MXC_I2C_RevA_Shutdown((mxc_i2c_reva_regs_t *)i2c);
}

int MXC_I2C_RevB_SetFrequency(mxc_i2c_revb_regs_t *i2c, unsigned int hz)
{
    return MXC_I2C_RevA_SetFrequency((mxc_i2c_reva_regs_t *)i2c, hz);
}

unsigned int MXC_I2C_RevB_GetFrequency(mxc_i2c_revb_regs_t *i2c)
{
    return MXC_I2C_RevA_GetFrequency((mxc_i2c_reva_regs_t *)i2c);
}

int MXC_I2C_RevB_ReadyForSleep(mxc_i2c_revb_regs_t *i2c)
{
    return MXC_I2C_RevA_ReadyForSleep((mxc_i2c_reva_regs_t *)i2c);
}

int MXC_I2C_RevB_SetClockStretching(mxc_i2c_revb_regs_t *i2c, int enable)
{
    return MXC_I2C_RevA_SetClockStretching((mxc_i2c_reva_regs_t *)i2c, enable);
}

int MXC_I2C_RevB_GetClockStretching(mxc_i2c_revb_regs_t *i2c)
{
    return MXC_I2C_RevA_GetClockStretching((mxc_i2c_reva_regs_t *)i2c);
}

/* ************************************************************************* */
/* Low-level functions                                                       */
/* ************************************************************************* */

int MXC_I2C_RevB_Start(mxc_i2c_revb_regs_t *i2c)
{
    return MXC_I2C_RevA_Start((mxc_i2c_reva_regs_t *)i2c);
}

int MXC_I2C_RevB_Stop(mxc_i2c_revb_regs_t *i2c)
{
    return MXC_I2C_RevA_Stop((mxc_i2c_reva_regs_t *)i2c);
}

int MXC_I2C_RevB_WriteByte(mxc_i2c_revb_regs_t *i2c, unsigned char byte)
{
    return MXC_I2C_RevA_WriteByte((mxc_i2c_reva_regs_t *)i2c, byte);
}

int MXC_I2C_RevB_ReadByte(mxc_i2c_revb_regs_t *i2c, unsigned char *byte, int ack)
{
    return MXC_I2C_RevA_ReadByte((mxc_i2c_reva_regs_t *)i2c, byte, ack);
}

int MXC_I2C_RevB_ReadByteInteractive(mxc_i2c_revb_regs_t *i2c, unsigned char *byte,
                                     mxc_i2c_revb_getAck_t getAck)
{
    return MXC_I2C_RevA_ReadByteInteractive((mxc_i2c_reva_regs_t *)i2c, byte,
                                            (mxc_i2c_reva_getAck_t)getAck);
}

int MXC_I2C_RevB_Write(mxc_i2c_revb_regs_t *i2c, unsigned char *bytes, unsigned int *len)
{
    return MXC_I2C_RevA_Write((mxc_i2c_reva_regs_t *)i2c, bytes, len);
}

int MXC_I2C_RevB_Read(mxc_i2c_revb_regs_t *i2c, unsigned char *bytes, unsigned int *len, int ack)
{
    return MXC_I2C_RevA_Read((mxc_i2c_reva_regs_t *)i2c, bytes, len, ack);
}

int MXC_I2C_RevB_ReadRXFIFO(mxc_i2c_revb_regs_t *i2c, volatile unsigned char *bytes,
                            unsigned int len)
{
    return MXC_I2C_RevA_ReadRXFIFO((mxc_i2c_reva_regs_t *)i2c, bytes, len);
}

int MXC_I2C_RevB_ReadRXFIFODMA(mxc_i2c_revb_regs_t *i2c, unsigned char *bytes, unsigned int len,
                               mxc_dma_regs_t *dma)
{
    return MXC_I2C_RevA_ReadRXFIFODMA((mxc_i2c_reva_regs_t *)i2c, bytes, len, dma);
}

int MXC_I2C_RevB_GetRXFIFOAvailable(mxc_i2c_revb_regs_t *i2c)
{
    return MXC_I2C_RevA_GetRXFIFOAvailable((mxc_i2c_reva_regs_t *)i2c);
}

int MXC_I2C_RevB_WriteTXFIFO(mxc_i2c_revb_regs_t *i2c, volatile unsigned char *bytes,
                             unsigned int len)
{
    return MXC_I2C_RevA_WriteTXFIFO((mxc_i2c_reva_regs_t *)i2c, bytes, len);
}

int MXC_I2C_RevB_WriteTXFIFODMA(mxc_i2c_revb_regs_t *i2c, unsigned char *bytes, unsigned int len,
                                mxc_dma_regs_t *dma)
{
    return MXC_I2C_RevA_WriteTXFIFODMA((mxc_i2c_reva_regs_t *)i2c, bytes, len, dma);
}

int MXC_I2C_RevB_GetTXFIFOAvailable(mxc_i2c_revb_regs_t *i2c)
{
    return MXC_I2C_RevA_GetTXFIFOAvailable((mxc_i2c_reva_regs_t *)i2c);
}

void MXC_I2C_RevB_ClearRXFIFO(mxc_i2c_revb_regs_t *i2c)
{
    MXC_I2C_RevA_ClearRXFIFO((mxc_i2c_reva_regs_t *)i2c);
}

void MXC_I2C_RevB_ClearTXFIFO(mxc_i2c_revb_regs_t *i2c)
{
    MXC_I2C_RevA_ClearTXFIFO((mxc_i2c_reva_regs_t *)i2c);
}

int MXC_I2C_RevB_GetFlags(mxc_i2c_revb_regs_t *i2c, unsigned int *flags0, unsigned int *flags1)
{
    return MXC_I2C_RevA_GetFlags((mxc_i2c_reva_regs_t *)i2c, flags0, flags1);
}

void MXC_I2C_RevB_ClearFlags(mxc_i2c_revb_regs_t *i2c, unsigned int flags0, unsigned int flags1)
{
    MXC_I2C_RevA_ClearFlags((mxc_i2c_reva_regs_t *)i2c, flags0, flags1);
}

void MXC_I2C_RevB_EnableInt(mxc_i2c_revb_regs_t *i2c, unsigned int flags0, unsigned int flags1)
{
    MXC_I2C_RevA_EnableInt((mxc_i2c_reva_regs_t *)i2c, flags0, flags1);
}

void MXC_I2C_RevB_DisableInt(mxc_i2c_revb_regs_t *i2c, unsigned int flags0, unsigned int flags1)
{
    MXC_I2C_RevA_DisableInt((mxc_i2c_reva_regs_t *)i2c, flags0, flags1);
}

void MXC_I2C_RevB_EnablePreload(mxc_i2c_revb_regs_t *i2c)
{
    MXC_I2C_RevA_EnablePreload((mxc_i2c_reva_regs_t *)i2c);
}

void MXC_I2C_RevB_DisablePreload(mxc_i2c_revb_regs_t *i2c)
{
    MXC_I2C_RevA_DisablePreload((mxc_i2c_reva_regs_t *)i2c);
}

void MXC_I2C_RevB_EnableGeneralCall(mxc_i2c_revb_regs_t *i2c)
{
    i2c->ctrl0 |= MXC_F_I2C_REVB_CTRL0_GCEN;
}

void MXC_I2C_RevB_DisableGeneralCall(mxc_i2c_revb_regs_t *i2c)
{
    i2c->ctrl0 &= ~MXC_F_I2C_REVB_CTRL0_GCEN;
}

void MXC_I2C_RevB_SetTimeout(mxc_i2c_revb_regs_t *i2c, unsigned int timeout)
{
    MXC_I2C_RevA_SetTimeout((mxc_i2c_reva_regs_t *)i2c, timeout);
}

unsigned int MXC_I2C_RevB_GetTimeout(mxc_i2c_revb_regs_t *i2c)
{
    return MXC_I2C_RevA_GetTimeout((mxc_i2c_reva_regs_t *)i2c);
}

int MXC_I2C_RevB_Recover(mxc_i2c_revb_regs_t *i2c, unsigned int retries)
{
    return MXC_I2C_RevA_Recover((mxc_i2c_reva_regs_t *)i2c, retries);
}

/* ************************************************************************* */
/* Transaction level functions                                               */
/* ************************************************************************* */

int MXC_I2C_RevB_MasterTransaction(mxc_i2c_revb_req_t *req)
{
    return MXC_I2C_RevA_MasterTransaction((mxc_i2c_reva_req_t *)req);
}

int MXC_I2C_RevB_MasterTransactionAsync(mxc_i2c_revb_req_t *req)
{
    return MXC_I2C_RevA_MasterTransactionAsync((mxc_i2c_reva_req_t *)req);
}

int MXC_I2C_RevB_MasterTransactionDMA(mxc_i2c_revb_req_t *req, mxc_dma_regs_t *dma)
{
    return MXC_I2C_RevA_MasterTransactionDMA((mxc_i2c_reva_req_t *)req, dma);
}

int MXC_I2C_RevB_SlaveTransaction(mxc_i2c_revb_regs_t *i2c, mxc_i2c_revb_slave_handler_t callback,
                                  uint32_t interruptCheck)
{
    return MXC_I2C_RevA_SlaveTransaction((mxc_i2c_reva_regs_t *)i2c,
                                         (mxc_i2c_reva_slave_handler_t)callback, interruptCheck);
}

int MXC_I2C_RevB_SlaveTransactionAsync(mxc_i2c_revb_regs_t *i2c,
                                       mxc_i2c_revb_slave_handler_t callback,
                                       uint32_t interruptCheck)
{
    return MXC_I2C_RevA_SlaveTransactionAsync(
        (mxc_i2c_reva_regs_t *)i2c, (mxc_i2c_reva_slave_handler_t)callback, interruptCheck);
}

int MXC_I2C_RevB_SetRXThreshold(mxc_i2c_revb_regs_t *i2c, unsigned int numBytes)
{
    return MXC_I2C_RevA_SetRXThreshold((mxc_i2c_reva_regs_t *)i2c, numBytes);
}

unsigned int MXC_I2C_RevB_GetRXThreshold(mxc_i2c_revb_regs_t *i2c)
{
    return MXC_I2C_RevA_GetRXThreshold((mxc_i2c_reva_regs_t *)i2c);
}

int MXC_I2C_RevB_SetTXThreshold(mxc_i2c_revb_regs_t *i2c, unsigned int numBytes)
{
    return MXC_I2C_RevA_SetTXThreshold((mxc_i2c_reva_regs_t *)i2c, numBytes);
}

unsigned int MXC_I2C_RevB_GetTXThreshold(mxc_i2c_revb_regs_t *i2c)
{
    return MXC_I2C_RevA_GetTXThreshold((mxc_i2c_reva_regs_t *)i2c);
}

void MXC_I2C_RevB_AsyncCallback(mxc_i2c_revb_regs_t *i2c, int retVal)
{
    MXC_I2C_RevA_AsyncCallback((mxc_i2c_reva_regs_t *)i2c, retVal);
}

void MXC_I2C_RevB_AsyncStop(mxc_i2c_revb_regs_t *i2c)
{
    MXC_I2C_RevA_AsyncStop((mxc_i2c_reva_regs_t *)i2c);
}

void MXC_I2C_RevB_AbortAsync(mxc_i2c_revb_regs_t *i2c)
{
    MXC_I2C_RevA_AbortAsync((mxc_i2c_reva_regs_t *)i2c);
}

void MXC_I2C_RevB_MasterAsyncHandler(int i2cNum)
{
    MXC_I2C_RevA_MasterAsyncHandler(i2cNum);
}

unsigned int MXC_I2C_RevB_SlaveAsyncHandler(mxc_i2c_revb_regs_t *i2c,
                                            mxc_i2c_revb_slave_handler_t callback,
                                            unsigned int interruptEnables, int *retVal)
{
    *retVal = E_NO_ERROR;

    // Callback called on
    // Slave Address Match (distinguish read/write)
    // RX Threshold
    // TX Threshold
    // Done
    // TX Underflow
    // RX Overflow
    //
    // Event Codes
    // I2C_EVT_MASTER_WR
    // I2C_EVT_MASTER_RD
    // I2C_EVT_RX_THRESH
    // I2C_EVT_TX_THRESH
    // I2C_EVT_TRANS_COMP
    // I2C_EVT_UNDERFLOW
    // I2C_EVT_OVERFLOW
    if (!(interruptEnables & (MXC_F_I2C_REVB_INT_FL0_AMI))) {
        // The STOPERR/STARTERR interrupt that's enabled here could fire before we are addressed
        // (fires anytime a stop/start is detected out of sequence).
        if (i2c->int_fl0 & MXC_I2C_REVB_ERROR) {
            *retVal = E_COMM_ERR;
            callback(i2c, MXC_I2C_REVB_EVT_TRANS_COMP, retVal);
            MXC_I2C_ClearFlags((mxc_i2c_regs_t *)i2c, MXC_I2C_REVB_INTFL0_MASK,
                               MXC_I2C_REVB_INTFL1_MASK); // Clear all I2C Interrupts
            MXC_I2C_ClearTXFIFO((mxc_i2c_regs_t *)i2c);
            MXC_I2C_ClearRXFIFO((mxc_i2c_regs_t *)i2c);
            interruptEnables = 0;
            AsyncRequests[MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c)] = NULL;
        }

        if (interruptEnables & (MXC_F_I2C_REVB_INT_FL0_RXTHI | MXC_F_I2C_REVB_INT_FL1_RXOFI)) {
            if (i2c->int_fl0 & MXC_F_I2C_REVB_INT_FL0_RXTHI) {
                callback(i2c, MXC_I2C_REVB_EVT_RX_THRESH, NULL);
                i2c->int_fl0 = MXC_F_I2C_REVB_INT_FL0_RXTHI;
            }

            if (i2c->int_fl1 & MXC_F_I2C_REVB_INT_FL1_RXOFI) {
                callback(i2c, MXC_I2C_REVB_EVT_OVERFLOW, NULL);
                i2c->int_fl1 = MXC_F_I2C_REVB_INT_FL1_RXOFI;
            }
        }

        if (interruptEnables & (MXC_F_I2C_REVB_INT_FL0_TXTHI | MXC_F_I2C_REVB_INT_FL1_TXUFI |
                                MXC_F_I2C_REVB_INT_FL0_TXLOI)) {
            if (i2c->int_fl0 & MXC_F_I2C_REVB_INT_FL0_TXTHI) {
                callback(i2c, MXC_I2C_REVB_EVT_TX_THRESH, NULL);
                i2c->int_fl0 = MXC_F_I2C_REVB_INT_FL0_TXTHI;
            }

            if (i2c->int_fl1 & MXC_F_I2C_REVB_INT_FL1_TXUFI) {
                callback(i2c, MXC_I2C_REVB_EVT_UNDERFLOW, NULL);
                i2c->int_fl1 = MXC_F_I2C_REVB_INT_FL1_TXUFI;
            }

            if (i2c->int_fl0 & MXC_F_I2C_REVB_INT_FL0_TXLOI) {
                *retVal = E_NO_ERROR;
                callback(i2c, MXC_I2C_REVB_EVT_TRANS_COMP, retVal);
                i2c->int_fl0 = MXC_F_I2C_REVB_INT_FL0_TXLOI;
                interruptEnables = 0;
                AsyncRequests[MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c)] = NULL;
            }
        }

        if (i2c->int_fl0 & MXC_F_I2C_REVB_INT_FL0_STOPI) {
            *retVal = E_NO_ERROR;
            callback(i2c, MXC_I2C_REVB_EVT_TRANS_COMP, retVal);
            i2c->int_fl0 = MXC_F_I2C_REVB_INT_FL0_STOPI;
            interruptEnables = 0;
            AsyncRequests[MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c)] = NULL;
        }
    }

    if (i2c->int_fl0 & MXC_F_I2C_REVB_INT_FL0_AMI) {
        if (i2c->ctrl0 & MXC_F_I2C_REVB_CTRL0_READ) {
            callback(i2c, MXC_I2C_REVB_EVT_MASTER_RD, NULL);
            i2c->int_fl0 = MXC_F_I2C_REVB_INT_FL0_AMI;
            i2c->int_fl0 = MXC_F_I2C_REVB_INT_FL0_AMI;
            i2c->int_fl0 = MXC_F_I2C_REVB_INT_FL0_TXLOI;
            interruptEnables = MXC_F_I2C_REVB_INT_FL0_TXTHI | MXC_F_I2C_REVB_INT_FL1_TXUFI |
                               MXC_F_I2C_REVB_INT_FL0_TXLOI | MXC_I2C_REVB_ERROR;
        } else {
            callback(i2c, MXC_I2C_REVB_EVT_MASTER_WR, NULL);
            i2c->int_fl0 = MXC_F_I2C_REVB_INT_FL0_AMI;
            i2c->int_fl0 = MXC_F_I2C_REVB_INT_FL0_AMI;
            interruptEnables = MXC_F_I2C_REVB_INT_FL0_RXTHI | MXC_F_I2C_REVB_INT_FL1_RXOFI |
                               MXC_I2C_REVB_ERROR;
        }
    } else if (i2c->int_fl0 & MXC_I2C_REVB_ERROR) {
        *retVal = E_COMM_ERR;
        callback(i2c, MXC_I2C_REVB_EVT_TRANS_COMP, retVal);
        MXC_I2C_RevB_ClearFlags(i2c, MXC_I2C_REVB_INTFL0_MASK,
                                MXC_I2C_REVB_INTFL1_MASK); // clear all i2c interrupts
        MXC_I2C_RevB_ClearTXFIFO(i2c);
        MXC_I2C_RevB_ClearRXFIFO(i2c);
        interruptEnables = 0;
        AsyncRequests[MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c)] = NULL;
    }

    return interruptEnables;
}

void MXC_I2C_RevB_AsyncHandler(mxc_i2c_revb_regs_t *i2c, uint32_t interruptCheck)
{
    int i2cNum = MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c);
    int slaveRetVal;

    if (i2cNum < 0) {
        return;
    }

    if (i2c->ctrl0 & MXC_F_I2C_REVB_CTRL0_MST) {
        MXC_I2C_RevB_MasterAsyncHandler(i2cNum);
    } else {
        mxc_i2c_revb_slave_handler_t callback = (mxc_i2c_revb_slave_handler_t)AsyncRequests[i2cNum];
        i2c->int_en0 = MXC_I2C_RevB_SlaveAsyncHandler(i2c, callback, i2c->int_en0, &slaveRetVal);
    }
}

void MXC_I2C_RevB_DMACallback(int ch, int error)
{
    MXC_I2C_RevA_DMACallback(ch, error);
}
