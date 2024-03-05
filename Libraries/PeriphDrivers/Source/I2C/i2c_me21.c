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
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "i2c_regs.h"
#include "dma_regs.h"
#include "i2c.h"
#include "i2c_reva.h"

/* **** Definitions **** */
#define MXC_I2C_MAX_ADDR_WIDTH 0x7F

/*
 *  These definitions are used to set the slave address register
 *  for MAX32672 Rev. A parts. MAX32672 Rev.B was updated to use
 *  the i2c_reva_regs.h register set.
 */
#define SLAVE_A1_REG_OFFSET ((uint32_t)0x44UL) /**< SLAVE_A1_REG_OFFSET */

#define SLAVE_A1_ADDR_POS 0 /**< SLAVE_A1_ADDR Position */
#define SLAVE_A1_ADDR ((uint32_t)(0x3FFUL << SLAVE_A1_ADDR_POS)) /**< SLAVE_A1_ADDR Mask */

#define SLAVE_A1_DIS_POS 10 /**< SLAVE_A1_DIS Position */
#define SLAVE_A1_DIS ((uint32_t)(0x1UL << SLAVE_A1_DIS_POS)) /**< SLAVE_A1_DIS Mask */

#define SLAVE_A1_IDX_POS 11 /**< SLAVE_A1_IDX Position */
#define SLAVE_A1_IDX ((uint32_t)(0x3UL << SLAVE_A1_IDX_POS)) /**< SLAVE_A1_IDX Mask */

#define SLAVE_A1_EXT_ADDR_EN_POS 15 /**< SLAVE_A1_EXT_ADDR_EN Position */
#define SLAVE_A1_EXT_ADDR_EN \
    ((uint32_t)(0x1UL << SLAVE_A1_EXT_ADDR_EN_POS)) /**< SLAVE_A1_EXT_ADDR_EN Mask */

/* **** Variable Declaration **** */
uint32_t interruptCheck = MXC_F_I2C_INTFL0_ADDR_MATCH | MXC_F_I2C_INTFL0_DNR_ERR;

/* **** Function Prototypes **** */

/* ************************************************************************* */
/* Control/Configuration functions                                           */
/* ************************************************************************* */
int MXC_I2C_Init(mxc_i2c_regs_t *i2c, int masterMode, unsigned int slaveAddr)
{
    if (i2c == NULL) {
        return E_NULL_PTR;
    }

#ifndef MSDK_NO_GPIO_CLK_INIT
    MXC_I2C_Shutdown(i2c); // Clear everything out

    if (i2c == MXC_I2C0) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_I2C0);
        MXC_GPIO_Config(&gpio_cfg_i2c0);
    } else if (i2c == MXC_I2C1) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_I2C1);
        MXC_GPIO_Config(&gpio_cfg_i2c1);
    } else if (i2c == MXC_I2C2) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_I2C2);
        MXC_GPIO_Config(&gpio_cfg_i2c2);
    } else {
        return E_NO_DEVICE;
    }
#endif // MSDK_NO_GPIO_CLK_INIT

    return MXC_I2C_RevA_Init((mxc_i2c_reva_regs_t *)i2c, masterMode, slaveAddr);
}

int MXC_I2C_SetSlaveAddr(mxc_i2c_regs_t *i2c, unsigned int slaveAddr, int idx)
{
    if ((MXC_SYS_GetRevision() & 0xF0) == 0xB0) {
        // Use multi-slave for MAX32672 Rev. B parts
        return MXC_I2C_RevA_SetSlaveAddr((mxc_i2c_reva_regs_t *)i2c, slaveAddr, idx);
    } else {
        // Use single-slave for MAX32672 Rev. A parts
        if (i2c == NULL) {
            return E_NULL_PTR;
        }

        if (idx >= MXC_I2C_NUM_TARGET_ADDR) {
            return E_NOT_SUPPORTED;
        }

        if (slaveAddr > SLAVE_A1_ADDR) {
            // Only support addresses up to 10 bits
            return E_BAD_PARAM;
        }

        uint32_t *slave_a1 = (uint32_t *)((uint32_t)i2c + SLAVE_A1_REG_OFFSET);

        // Set the slave address to operate on
        MXC_SETFIELD((*slave_a1), SLAVE_A1_IDX, (idx << SLAVE_A1_IDX_POS));

        if (slaveAddr > MXC_I2C_MAX_ADDR_WIDTH) {
            // Set for 10bit addressing mode
            *slave_a1 |= SLAVE_A1_EXT_ADDR_EN;
        } else {
            // Clear for 7bit addressing mode
            *slave_a1 &= ~SLAVE_A1_EXT_ADDR_EN;
        }

        // Set the slave address
        MXC_SETFIELD((*slave_a1), SLAVE_A1_ADDR, (slaveAddr << SLAVE_A1_ADDR_POS));

        // Enable the slave address
        *slave_a1 &= ~SLAVE_A1_DIS;
    }

    return E_NO_ERROR;
}

int MXC_I2C_Shutdown(mxc_i2c_regs_t *i2c)
{
    // Configure GPIO for I2C
    if (i2c == MXC_I2C0) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_I2C0);
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_I2C0);
    } else if (i2c == MXC_I2C1) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_I2C1);
        MXC_SYS_Reset_Periph(MXC_SYS_RESET1_I2C1);
    } else if (i2c == MXC_I2C2) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_I2C2);
        MXC_SYS_Reset_Periph(MXC_SYS_RESET1_I2C2);
    } else {
        return E_NO_DEVICE;
    }

    return MXC_I2C_RevA_Shutdown((mxc_i2c_reva_regs_t *)i2c);
}

int MXC_I2C_Reset(mxc_i2c_regs_t *i2c)
{
    // Configure GPIO for I2C
    if (i2c == MXC_I2C0) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_I2C0);
    } else if (i2c == MXC_I2C1) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET1_I2C1);
    } else if (i2c == MXC_I2C2) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET1_I2C2);
    } else {
        return E_NO_DEVICE;
    }

    return E_NO_ERROR;
}

int MXC_I2C_SetFrequency(mxc_i2c_regs_t *i2c, unsigned int hz)
{
    return MXC_I2C_RevA_SetFrequency((mxc_i2c_reva_regs_t *)i2c, hz);
}

unsigned int MXC_I2C_GetFrequency(mxc_i2c_regs_t *i2c)
{
    return MXC_I2C_RevA_GetFrequency((mxc_i2c_reva_regs_t *)i2c);
}

int MXC_I2C_ReadyForSleep(mxc_i2c_regs_t *i2c)
{
    return MXC_I2C_RevA_ReadyForSleep((mxc_i2c_reva_regs_t *)i2c);
}

int MXC_I2C_SetClockStretching(mxc_i2c_regs_t *i2c, int enable)
{
    return MXC_I2C_RevA_SetClockStretching((mxc_i2c_reva_regs_t *)i2c, enable);
}

int MXC_I2C_GetClockStretching(mxc_i2c_regs_t *i2c)
{
    return MXC_I2C_RevA_GetClockStretching((mxc_i2c_reva_regs_t *)i2c);
}

int MXC_I2C_DMA_Init(mxc_i2c_regs_t *i2c, mxc_dma_regs_t *dma, bool use_dma_tx, bool use_dma_rx)
{
    return MXC_I2C_RevA_DMA_Init((mxc_i2c_reva_regs_t *)i2c, (mxc_dma_reva_regs_t *)dma, use_dma_tx,
                                 use_dma_rx);
}

int MXC_I2C_DMA_GetTXChannel(mxc_i2c_regs_t *i2c)
{
    return MXC_I2C_RevA_DMA_GetTXChannel((mxc_i2c_reva_regs_t *)i2c);
}

int MXC_I2C_DMA_GetRXChannel(mxc_i2c_regs_t *i2c)
{
    return MXC_I2C_RevA_DMA_GetRXChannel((mxc_i2c_reva_regs_t *)i2c);
}

int MXC_I2C_DMA_SetRequestSelect(mxc_i2c_regs_t *i2c, uint8_t *txData, uint8_t *rxData)
{
    int i2cNum;
    int txReqSel = -1;
    int rxReqSel = -1;

    if (i2c == NULL) {
        return E_NULL_PTR;
    }

    i2cNum = MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c);

    if (txData != NULL) {
        switch (i2cNum) {
        case 0:
            txReqSel = MXC_DMA_REQUEST_I2C0TX;
            break;

        case 1:
            txReqSel = MXC_DMA_REQUEST_I2C1TX;
            break;

        case 2:
            txReqSel = MXC_DMA_REQUEST_I2C2TX;
            break;

        default:
            return E_BAD_PARAM;
        }
    }

    if (rxData != NULL) {
        switch (i2cNum) {
        case 0:
            rxReqSel = MXC_DMA_REQUEST_I2C0RX;
            break;

        case 1:
            rxReqSel = MXC_DMA_REQUEST_I2C1RX;
            break;

        case 2:
            rxReqSel = MXC_DMA_REQUEST_I2C2RX;
            break;

        default:
            return E_BAD_PARAM;
        }
    }

    return MXC_I2C_RevA_DMA_SetRequestSelect((mxc_i2c_reva_regs_t *)i2c,
                                             (mxc_dma_reva_regs_t *)MXC_DMA, txReqSel, rxReqSel);
}

/* ************************************************************************* */
/* Low-level functions                                                       */
/* ************************************************************************* */
int MXC_I2C_Start(mxc_i2c_regs_t *i2c)
{
    return MXC_I2C_RevA_Start((mxc_i2c_reva_regs_t *)i2c);
}

int MXC_I2C_Stop(mxc_i2c_regs_t *i2c)
{
    return MXC_I2C_RevA_Stop((mxc_i2c_reva_regs_t *)i2c);
}

int MXC_I2C_WriteByte(mxc_i2c_regs_t *i2c, unsigned char byte)
{
    return MXC_I2C_RevA_WriteByte((mxc_i2c_reva_regs_t *)i2c, byte);
}

int MXC_I2C_ReadByte(mxc_i2c_regs_t *i2c, unsigned char *byte, int ack)
{
    return MXC_I2C_RevA_ReadByte((mxc_i2c_reva_regs_t *)i2c, byte, ack);
}

int MXC_I2C_ReadByteInteractive(mxc_i2c_regs_t *i2c, unsigned char *byte, mxc_i2c_getAck_t getAck)
{
    return MXC_I2C_RevA_ReadByteInteractive((mxc_i2c_reva_regs_t *)i2c, byte,
                                            (mxc_i2c_reva_getAck_t)getAck);
}

int MXC_I2C_Write(mxc_i2c_regs_t *i2c, unsigned char *bytes, unsigned int *len)
{
    return MXC_I2C_RevA_Write((mxc_i2c_reva_regs_t *)i2c, bytes, len);
}

int MXC_I2C_Read(mxc_i2c_regs_t *i2c, unsigned char *bytes, unsigned int *len, int ack)
{
    return MXC_I2C_RevA_Read((mxc_i2c_reva_regs_t *)i2c, bytes, len, ack);
}

int MXC_I2C_ReadRXFIFO(mxc_i2c_regs_t *i2c, volatile unsigned char *bytes, unsigned int len)
{
    return MXC_I2C_RevA_ReadRXFIFO((mxc_i2c_reva_regs_t *)i2c, bytes, len);
}

int MXC_I2C_ReadRXFIFODMA(mxc_i2c_regs_t *i2c, unsigned char *bytes, unsigned int len,
                          mxc_i2c_dma_complete_cb_t callback)
{
    // The callback parameter was previously unused but keeping it for backwards-compatibility.
    return MXC_I2C_RevA_ReadRXFIFODMA((mxc_i2c_reva_regs_t *)i2c, bytes, len, MXC_DMA);
}

int MXC_I2C_GetRXFIFOAvailable(mxc_i2c_regs_t *i2c)
{
    return MXC_I2C_RevA_GetRXFIFOAvailable((mxc_i2c_reva_regs_t *)i2c);
}

int MXC_I2C_WriteTXFIFO(mxc_i2c_regs_t *i2c, volatile unsigned char *bytes, unsigned int len)
{
    return MXC_I2C_RevA_WriteTXFIFO((mxc_i2c_reva_regs_t *)i2c, bytes, len);
}

int MXC_I2C_WriteTXFIFODMA(mxc_i2c_regs_t *i2c, unsigned char *bytes, unsigned int len,
                           mxc_i2c_dma_complete_cb_t callback)
{
    // The callback parameter was previously unused but keeping it for backwards-compatibility.
    return MXC_I2C_RevA_WriteTXFIFODMA((mxc_i2c_reva_regs_t *)i2c, bytes, len, MXC_DMA);
}

int MXC_I2C_GetTXFIFOAvailable(mxc_i2c_regs_t *i2c)
{
    return MXC_I2C_RevA_GetTXFIFOAvailable((mxc_i2c_reva_regs_t *)i2c);
}

void MXC_I2C_ClearRXFIFO(mxc_i2c_regs_t *i2c)
{
    MXC_I2C_RevA_ClearRXFIFO((mxc_i2c_reva_regs_t *)i2c);
}

void MXC_I2C_ClearTXFIFO(mxc_i2c_regs_t *i2c)
{
    MXC_I2C_RevA_ClearTXFIFO((mxc_i2c_reva_regs_t *)i2c);
}

int MXC_I2C_GetFlags(mxc_i2c_regs_t *i2c, unsigned int *flags0, unsigned int *flags1)
{
    return MXC_I2C_RevA_GetFlags((mxc_i2c_reva_regs_t *)i2c, flags0, flags1);
}

void MXC_I2C_ClearFlags(mxc_i2c_regs_t *i2c, unsigned int flags0, unsigned int flags1)
{
    MXC_I2C_RevA_ClearFlags((mxc_i2c_reva_regs_t *)i2c, flags0, flags1);
}

void MXC_I2C_EnableInt(mxc_i2c_regs_t *i2c, unsigned int flags0, unsigned int flags1)
{
    MXC_I2C_RevA_EnableInt((mxc_i2c_reva_regs_t *)i2c, flags0, flags1);
}

void MXC_I2C_DisableInt(mxc_i2c_regs_t *i2c, unsigned int flags0, unsigned int flags1)
{
    MXC_I2C_RevA_DisableInt((mxc_i2c_reva_regs_t *)i2c, flags0, flags1);
}

void MXC_I2C_EnablePreload(mxc_i2c_regs_t *i2c)
{
    MXC_I2C_RevA_EnablePreload((mxc_i2c_reva_regs_t *)i2c);
}

void MXC_I2C_DisablePreload(mxc_i2c_regs_t *i2c)
{
    MXC_I2C_RevA_DisablePreload((mxc_i2c_reva_regs_t *)i2c);
}

void MXC_I2C_EnableGeneralCall(mxc_i2c_regs_t *i2c)
{
    MXC_I2C_RevA_EnableGeneralCall((mxc_i2c_reva_regs_t *)i2c);
}

void MXC_I2C_DisableGeneralCall(mxc_i2c_regs_t *i2c)
{
    MXC_I2C_RevA_DisableGeneralCall((mxc_i2c_reva_regs_t *)i2c);
}

void MXC_I2C_SetTimeout(mxc_i2c_regs_t *i2c, unsigned int timeout)
{
    MXC_I2C_RevA_SetTimeout((mxc_i2c_reva_regs_t *)i2c, timeout);
}

unsigned int MXC_I2C_GetTimeout(mxc_i2c_regs_t *i2c)
{
    return MXC_I2C_RevA_GetTimeout((mxc_i2c_reva_regs_t *)i2c);
}

int MXC_I2C_Recover(mxc_i2c_regs_t *i2c, unsigned int retries)
{
    return MXC_I2C_RevA_Recover((mxc_i2c_reva_regs_t *)i2c, retries);
}

/* ************************************************************************* */
/* Transaction level functions                                               */
/* ************************************************************************* */

int MXC_I2C_MasterTransaction(mxc_i2c_req_t *req)
{
    if (!(req->i2c->status & MXC_F_I2C_STATUS_TX_EM)) {
        MXC_I2C_ClearTXFIFO(req->i2c);
    }
    if (!(req->i2c->status & MXC_F_I2C_STATUS_RX_EM)) {
        MXC_I2C_ClearRXFIFO(req->i2c);
    }

    return MXC_I2C_RevA_MasterTransaction((mxc_i2c_reva_req_t *)req);
}

int MXC_I2C_MasterTransactionAsync(mxc_i2c_req_t *req)
{
    MXC_I2C_ClearRXFIFO(req->i2c);
    MXC_I2C_ClearTXFIFO(req->i2c);

    return MXC_I2C_RevA_MasterTransactionAsync((mxc_i2c_reva_req_t *)req);
}

int MXC_I2C_MasterTransactionDMA(mxc_i2c_req_t *req)
{
    MXC_I2C_ClearRXFIFO(req->i2c);
    MXC_I2C_ClearTXFIFO(req->i2c);

    return MXC_I2C_RevA_MasterTransactionDMA((mxc_i2c_reva_req_t *)req, MXC_DMA);
}

int MXC_I2C_SlaveTransaction(mxc_i2c_regs_t *i2c, mxc_i2c_slave_handler_t callback)
{
    MXC_I2C_ClearRXFIFO(i2c);
    MXC_I2C_ClearTXFIFO(i2c);

    return MXC_I2C_RevA_SlaveTransaction((mxc_i2c_reva_regs_t *)i2c,
                                         (mxc_i2c_reva_slave_handler_t)callback, interruptCheck);
}

int MXC_I2C_SlaveTransactionAsync(mxc_i2c_regs_t *i2c, mxc_i2c_slave_handler_t callback)
{
    MXC_I2C_ClearRXFIFO(i2c);
    MXC_I2C_ClearTXFIFO(i2c);

    return MXC_I2C_RevA_SlaveTransactionAsync(
        (mxc_i2c_reva_regs_t *)i2c, (mxc_i2c_reva_slave_handler_t)callback, interruptCheck);
}

int MXC_I2C_SetRXThreshold(mxc_i2c_regs_t *i2c, unsigned int numBytes)
{
    return MXC_I2C_RevA_SetRXThreshold((mxc_i2c_reva_regs_t *)i2c, numBytes);
}

unsigned int MXC_I2C_GetRXThreshold(mxc_i2c_regs_t *i2c)
{
    return MXC_I2C_RevA_GetRXThreshold((mxc_i2c_reva_regs_t *)i2c);
}

int MXC_I2C_SetTXThreshold(mxc_i2c_regs_t *i2c, unsigned int numBytes)
{
    return MXC_I2C_RevA_SetTXThreshold((mxc_i2c_reva_regs_t *)i2c, numBytes);
}

unsigned int MXC_I2C_GetTXThreshold(mxc_i2c_regs_t *i2c)
{
    return MXC_I2C_RevA_GetTXThreshold((mxc_i2c_reva_regs_t *)i2c);
}

void MXC_I2C_AsyncStop(mxc_i2c_regs_t *i2c)
{
    MXC_I2C_RevA_AsyncStop((mxc_i2c_reva_regs_t *)i2c);
}

void MXC_I2C_AbortAsync(mxc_i2c_regs_t *i2c)
{
    MXC_I2C_RevA_AbortAsync((mxc_i2c_reva_regs_t *)i2c);
}

void MXC_I2C_AsyncHandler(mxc_i2c_regs_t *i2c)
{
    MXC_I2C_RevA_AsyncHandler((mxc_i2c_reva_regs_t *)i2c, interruptCheck);
}

void MXC_I2C_DMACallback(int ch, int error)
{
    MXC_I2C_RevA_DMACallback(ch, error);
}
