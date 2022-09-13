/* ****************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
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
 * $Date: 2020-02-03 10:33:50 -0600 (Mon, 03 Feb 2020) $
 * $Revision: 51326 $
 *
 *************************************************************************** */

#include <stddef.h>
#include <stdint.h>
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "i2c.h"
#include "i2c_revb.h"
#include <stdio.h>
#include "mxc_delay.h"

/* **** Definitions **** */
#define I2C_ERROR                                                                                \
    (MXC_F_I2C_INT_FL0_ARB_ER | MXC_F_I2C_INT_FL0_TO_ER | MXC_F_I2C_INT_FL0_ADDR_NACK_ER |       \
     MXC_F_I2C_INT_FL0_DATA_ER | MXC_F_I2C_INT_FL0_DO_NOT_RESP_ER | MXC_F_I2C_INT_FL0_START_ER | \
     MXC_F_I2C_INT_FL0_STOP_ER)
#define MASTER 1
#define SLAVE 0

#define MXC_I2C_MAX_ADDR_WIDTH 0x7F
#define MXC_I2C_STD_MODE 100000
#define MXC_I2C_FAST_SPEED 400000

/* **** Variable Declaration **** */

/* **** Function Prototypes **** */

/* ************************************************************************** */
int MXC_I2C_Init(mxc_i2c_regs_t *i2c, int masterMode, unsigned int slaveAddr)
{
    int err;
    int idx = MXC_I2C_GET_IDX(i2c);

    switch (idx) {
    case 0:
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_I2C0);
        MXC_GPIO_Config(&gpio_cfg_i2c0);
        break;
    case 1:
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_I2C1);
        MXC_GPIO_Config(&gpio_cfg_i2c1);
        break;
    default:
        return E_BAD_PARAM;
    }

    if ((err = MXC_I2C_RevB_Init((mxc_i2c_revb_regs_t *)i2c, masterMode, slaveAddr)) !=
        E_NO_ERROR) {
        return err;
    }

    return E_NO_ERROR;
}

/* ************************************************************************** */
int MXC_I2C_SetSlaveAddr(mxc_i2c_regs_t *i2c, unsigned int slaveAddr, int idx)
{
    return MXC_I2C_RevB_SetSlaveAddr((mxc_i2c_revb_regs_t *)i2c, slaveAddr, idx);
}

/* ************************************************************************** */
int MXC_I2C_Shutdown(mxc_i2c_regs_t *i2c)
{
    switch (MXC_I2C_GET_IDX(i2c)) {
    case 0:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_I2C0);
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_I2C0);
        break;
    case 1:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_I2C1);
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_I2C1);
        break;
    default:
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

/* ************************************************************************** */
int MXC_I2C_SetFrequency(mxc_i2c_regs_t *i2c, unsigned int hz)
{
    if (hz > MXC_I2C_FAST_SPEED) {
        return E_NOT_SUPPORTED;
    }

    return MXC_I2C_RevB_SetFrequency((mxc_i2c_revb_regs_t *)i2c, hz);
}

/* ************************************************************************** */
unsigned int MXC_I2C_GetFrequency(mxc_i2c_regs_t *i2c)
{
    return MXC_I2C_RevB_GetFrequency((mxc_i2c_revb_regs_t *)i2c);
}

/* ************************************************************************** */
int MXC_I2C_ReadyForSleep(mxc_i2c_regs_t *i2c)
{
    if (i2c->stat & MXC_F_I2C_REVB_STAT_BUSY) {
        return E_BUSY;
    }
    return MXC_I2C_RevB_ReadyForSleep((mxc_i2c_revb_regs_t *)i2c);
}

/* ************************************************************************** */
int MXC_I2C_SetClockStretching(mxc_i2c_regs_t *i2c, int enable)
{
    return MXC_I2C_RevB_SetClockStretching((mxc_i2c_revb_regs_t *)i2c, enable);
}

/* ************************************************************************** */
int MXC_I2C_GetClockStretching(mxc_i2c_regs_t *i2c)
{
    return MXC_I2C_RevB_GetClockStretching((mxc_i2c_revb_regs_t *)i2c);
}

/* ************************************************************************** */
int MXC_I2C_Start(mxc_i2c_regs_t *i2c)
{
    return MXC_I2C_RevB_Start((mxc_i2c_revb_regs_t *)i2c);
}

/* ************************************************************************** */
int MXC_I2C_Stop(mxc_i2c_regs_t *i2c)
{
    return MXC_I2C_RevB_Stop((mxc_i2c_revb_regs_t *)i2c);
}

/* ************************************************************************** */
int MXC_I2C_WriteByte(mxc_i2c_regs_t *i2c, unsigned char byte)
{
    return MXC_I2C_RevB_WriteByte((mxc_i2c_revb_regs_t *)i2c, byte);
}

/* ************************************************************************** */
int MXC_I2C_ReadByte(mxc_i2c_regs_t *i2c, unsigned char *byte, int ack)
{
    return MXC_I2C_RevB_ReadByte((mxc_i2c_revb_regs_t *)i2c, byte, ack);
}

/* ************************************************************************** */
int MXC_I2C_ReadByteInteractive(mxc_i2c_regs_t *i2c, unsigned char *byte, mxc_i2c_getAck_t getAck)
{
    return MXC_I2C_RevB_ReadByteInteractive((mxc_i2c_revb_regs_t *)i2c, byte,
                                            (mxc_i2c_revb_getAck_t)getAck);
}

/* ************************************************************************** */
int MXC_I2C_Write(mxc_i2c_regs_t *i2c, unsigned char *bytes, unsigned int *len)
{
    return MXC_I2C_RevB_Write((mxc_i2c_revb_regs_t *)i2c, bytes, len);
}

/* ************************************************************************** */
int MXC_I2C_Read(mxc_i2c_regs_t *i2c, unsigned char *bytes, unsigned int *len, int ack)
{
    return MXC_I2C_RevB_Read((mxc_i2c_revb_regs_t *)i2c, bytes, len, ack);
}

/* ************************************************************************** */
int MXC_I2C_ReadRXFIFO(mxc_i2c_regs_t *i2c, volatile unsigned char *bytes, unsigned int len)
{
    return MXC_I2C_RevB_ReadRXFIFO((mxc_i2c_revb_regs_t *)i2c, bytes, len);
}

/* ************************************************************************** */
int MXC_I2C_ReadRXFIFODMA(mxc_i2c_regs_t *i2c, unsigned char *bytes, unsigned int len,
                          mxc_i2c_dma_complete_cb_t callback)
{
    mxc_dma_config_t config;

    switch (MXC_I2C_GET_IDX(i2c)) {
    case 0:
        config.reqsel = MXC_DMA_REQUEST_I2C0RX;
        break;
    case 1:
        config.reqsel = MXC_DMA_REQUEST_I2C1RX;
        break;
    default:
        return E_BAD_PARAM;
    }

    return MXC_I2C_RevB_ReadRXFIFODMA((mxc_i2c_revb_regs_t *)i2c, bytes, len, callback, config,
                                      MXC_DMA);
}

/* ************************************************************************** */
int MXC_I2C_GetRXFIFOAvailable(mxc_i2c_regs_t *i2c)
{
    return MXC_I2C_RevB_GetRXFIFOAvailable((mxc_i2c_revb_regs_t *)i2c);
}

/* ************************************************************************** */
int MXC_I2C_WriteTXFIFO(mxc_i2c_regs_t *i2c, volatile unsigned char *bytes, unsigned int len)
{
    return MXC_I2C_RevB_WriteTXFIFO((mxc_i2c_revb_regs_t *)i2c, bytes, len);
}

/* ************************************************************************** */
int MXC_I2C_WriteTXFIFODMA(mxc_i2c_regs_t *i2c, unsigned char *bytes, unsigned int len,
                           mxc_i2c_dma_complete_cb_t callback)
{
    mxc_dma_config_t config;

    switch (MXC_I2C_GET_IDX(i2c)) {
    case 0:
        config.reqsel = MXC_DMA_REQUEST_I2C0TX;
        break;
    case 1:
        config.reqsel = MXC_DMA_REQUEST_I2C1TX;
        break;
    default:
        return E_BAD_PARAM;
    }

    return MXC_I2C_RevB_WriteTXFIFODMA((mxc_i2c_revb_regs_t *)i2c, bytes, len, callback, config,
                                       MXC_DMA);
}

/* ************************************************************************** */
int MXC_I2C_GetTXFIFOAvailable(mxc_i2c_regs_t *i2c)
{
    return MXC_I2C_RevB_GetTXFIFOAvailable((mxc_i2c_revb_regs_t *)i2c);
}

/* ************************************************************************** */
void MXC_I2C_ClearRXFIFO(mxc_i2c_regs_t *i2c)
{
    MXC_I2C_RevB_ClearRXFIFO((mxc_i2c_revb_regs_t *)i2c);
}

/* ************************************************************************** */
void MXC_I2C_ClearTXFIFO(mxc_i2c_regs_t *i2c)
{
    MXC_I2C_RevB_ClearTXFIFO((mxc_i2c_revb_regs_t *)i2c);
}

/* ************************************************************************** */
int MXC_I2C_GetFlags(mxc_i2c_regs_t *i2c, unsigned int *flags0, unsigned int *flags1)
{
    return MXC_I2C_RevB_GetFlags((mxc_i2c_revb_regs_t *)i2c, flags0, flags1);
}

/* ************************************************************************** */
void MXC_I2C_ClearFlags(mxc_i2c_regs_t *i2c, unsigned int flags0, unsigned int flags1)
{
    MXC_I2C_RevB_ClearFlags((mxc_i2c_revb_regs_t *)i2c, flags0, flags1);
}

/* ************************************************************************** */
void MXC_I2C_EnableInt(mxc_i2c_regs_t *i2c, unsigned int flags0, unsigned int flags1)
{
    MXC_I2C_RevB_EnableInt((mxc_i2c_revb_regs_t *)i2c, flags0, flags1);
}

/* ************************************************************************** */
void MXC_I2C_DisableInt(mxc_i2c_regs_t *i2c, unsigned int flags0, unsigned int flags1)
{
    MXC_I2C_RevB_DisableInt((mxc_i2c_revb_regs_t *)i2c, flags0, flags1);
}

/* ************************************************************************** */
void MXC_I2C_EnablePreload(mxc_i2c_regs_t *i2c)
{
    MXC_I2C_RevB_EnablePreload((mxc_i2c_revb_regs_t *)i2c);
}

/* ************************************************************************** */
void MXC_I2C_DisablePreload(mxc_i2c_regs_t *i2c)
{
    MXC_I2C_RevB_DisablePreload((mxc_i2c_revb_regs_t *)i2c);
}

/* ************************************************************************** */
void MXC_I2C_EnableGeneralCall(mxc_i2c_regs_t *i2c)
{
    MXC_I2C_RevB_EnableGeneralCall((mxc_i2c_revb_regs_t *)i2c);
}

/* ************************************************************************** */
void MXC_I2C_DisableGeneralCall(mxc_i2c_regs_t *i2c)
{
    MXC_I2C_RevB_DisableGeneralCall((mxc_i2c_revb_regs_t *)i2c);
}

/* ************************************************************************** */
void MXC_I2C_SetTimeout(mxc_i2c_regs_t *i2c, unsigned int timeout)
{
    MXC_I2C_RevB_SetTimeout((mxc_i2c_revb_regs_t *)i2c, timeout);
}

/* ************************************************************************** */
unsigned int MXC_I2C_GetTimeout(mxc_i2c_regs_t *i2c)
{
    return MXC_I2C_RevB_GetTimeout((mxc_i2c_revb_regs_t *)i2c);
}

/* ************************************************************************** */
int MXC_I2C_Recover(mxc_i2c_regs_t *i2c, unsigned int retries)
{
    return MXC_I2C_RevB_Recover((mxc_i2c_revb_regs_t *)i2c, retries);
}

/* ************************************************************************* */
int MXC_I2C_MasterTransaction(mxc_i2c_req_t *req)
{
    return MXC_I2C_RevB_MasterTransaction((mxc_i2c_revb_req_t *)req);
}

/* ************************************************************************* */
int MXC_I2C_MasterTransactionAsync(mxc_i2c_req_t *req)
{
    return MXC_I2C_RevB_MasterTransactionAsync((mxc_i2c_revb_req_t *)req);
}

/* ************************************************************************* */
int MXC_I2C_MasterTransactionDMA(mxc_i2c_req_t *req)
{
    return MXC_I2C_RevB_MasterTransactionDMA((mxc_i2c_revb_req_t *)req, MXC_DMA);
}

/* ************************************************************************* */
int MXC_I2C_SlaveTransaction(mxc_i2c_regs_t *i2c, mxc_i2c_slave_handler_t callback)
{
    return MXC_I2C_RevB_SlaveTransaction(
        (mxc_i2c_revb_regs_t *)i2c, (mxc_i2c_revb_slave_handler_t)callback, MXC_F_I2C_INT_FL0_AMI);
}

/* ************************************************************************* */
int MXC_I2C_SlaveTransactionAsync(mxc_i2c_regs_t *i2c, mxc_i2c_slave_handler_t callback)
{
    return MXC_I2C_RevB_SlaveTransactionAsync(
        (mxc_i2c_revb_regs_t *)i2c, (mxc_i2c_revb_slave_handler_t)callback, MXC_F_I2C_INT_FL0_AMI);
}

/* ************************************************************************* */
int MXC_I2C_SetRXThreshold(mxc_i2c_regs_t *i2c, unsigned int numBytes)
{
    return MXC_I2C_RevB_SetRXThreshold((mxc_i2c_revb_regs_t *)i2c, numBytes);
}

/* ************************************************************************* */
unsigned int MXC_I2C_GetRXThreshold(mxc_i2c_regs_t *i2c)
{
    return MXC_I2C_RevB_GetRXThreshold((mxc_i2c_revb_regs_t *)i2c);
}

/* ************************************************************************* */
int MXC_I2C_SetTXThreshold(mxc_i2c_regs_t *i2c, unsigned int numBytes)
{
    return MXC_I2C_RevB_SetTXThreshold((mxc_i2c_revb_regs_t *)i2c, numBytes);
}

/* ************************************************************************* */
unsigned int MXC_I2C_GetTXThreshold(mxc_i2c_regs_t *i2c)
{
    return MXC_I2C_RevB_GetTXThreshold((mxc_i2c_revb_regs_t *)i2c);
}

/* ************************************************************************* */
void MXC_I2C_AsyncCallback(mxc_i2c_regs_t *i2c, int retVal)
{
    MXC_I2C_RevB_AsyncCallback((mxc_i2c_revb_regs_t *)i2c, retVal);
}

/* ************************************************************************* */
void MXC_I2C_AsyncStop(mxc_i2c_regs_t *i2c)
{
    MXC_I2C_RevB_AsyncStop((mxc_i2c_revb_regs_t *)i2c);
}

/* ************************************************************************* */
void MXC_I2C_AbortAsync(mxc_i2c_regs_t *i2c)
{
    MXC_I2C_RevB_AbortAsync((mxc_i2c_revb_regs_t *)i2c);
}

/* ************************************************************************* */
void MXC_I2C_MasterAsyncHandler(int i2cNum)
{
    MXC_I2C_RevB_MasterAsyncHandler(i2cNum);
}

/* ************************************************************************* */
unsigned int MXC_I2C_SlaveAsyncHandler(mxc_i2c_regs_t *i2c, mxc_i2c_slave_handler_t callback,
                                       unsigned int interruptEnables, int *retVal)
{
    return MXC_I2C_RevB_SlaveAsyncHandler((mxc_i2c_revb_regs_t *)i2c,
                                          (mxc_i2c_revb_slave_handler_t)callback, interruptEnables,
                                          retVal);
}

/* ************************************************************************* */
void MXC_I2C_AsyncHandler(mxc_i2c_regs_t *i2c)
{
    MXC_I2C_RevB_AsyncHandler((mxc_i2c_revb_regs_t *)i2c, MXC_F_I2C_INT_FL0_AMI);
}

/* ************************************************************************* */
void MXC_I2C_DMACallback(int ch, int error)
{
    MXC_I2C_RevB_DMACallback(ch, error);
}
