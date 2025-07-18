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
#include <stdbool.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "i2c_regs.h"
#include "i2c.h"
#include "i2c_reva.h"
#include "dma.h"
#include "dma_reva.h"
#include "nvic_table.h"

/* **** Variable Declaration **** */
typedef struct {
    mxc_i2c_reva_req_t *req;
    int master; // 1 for Master, 0 for slave
    int channelTx; // DMA channel for TX transaction
    int channelRx; // DMA channel for RX transaction
    volatile int writeDone; // Write done flag
    volatile int readDone; // Flag done flag
    bool dma_initialized; // Check to see whether DMA was initialized
    mxc_dma_reva_regs_t *dma; // Save DMA Instance
} mxc_i2c_reva_req_state_t;

static mxc_i2c_reva_req_state_t states[MXC_I2C_INSTANCES];

void *AsyncRequests[MXC_I2C_INSTANCES];
unsigned int AsyncWritten[MXC_I2C_INSTANCES];
unsigned int AsyncRead[MXC_I2C_INSTANCES];

/* **** Function Prototypes **** */
void MXC_I2C_RevA_AsyncCallback(mxc_i2c_reva_regs_t *i2c, int retVal);
void MXC_I2C_RevA_AsyncStop(mxc_i2c_reva_regs_t *i2c);
void MXC_I2C_RevA_AbortAsync(mxc_i2c_reva_regs_t *i2c);
void MXC_I2C_RevA_MasterAsyncHandler(int i2cNum);
int MXC_I2C_RevA_DMAHandler(mxc_i2c_reva_req_t *req);
void MXC_I2C_RevA_SlaveAsyncHandler(mxc_i2c_reva_regs_t *i2c, mxc_i2c_reva_slave_handler_t callback,
                                    uint32_t *int_en, int *retVal);

/* ************************************************************************* */
/* Control/Configuration functions                                           */
/* ************************************************************************* */
int MXC_I2C_RevA_Init(mxc_i2c_reva_regs_t *i2c, int masterMode, unsigned int slaveAddr)
{
    int err;
    int8_t i2cNum;

    if (i2c == NULL) {
        return E_NULL_PTR;
    }

    i2cNum = MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c);

    if ((err = MXC_I2C_Recover((mxc_i2c_regs_t *)i2c, 16)) != E_NO_ERROR) {
        return err;
    }

    i2c->ctrl |= MXC_F_I2C_REVA_CTRL_EN;

    MXC_I2C_ClearRXFIFO((mxc_i2c_regs_t *)i2c);
    MXC_I2C_ClearTXFIFO((mxc_i2c_regs_t *)i2c);
    // Set the thresholds here and allow the user to change them as needed
    MXC_I2C_SetTXThreshold((mxc_i2c_regs_t *)i2c, 2); // set TX threshold to 2 bytes
    MXC_I2C_SetRXThreshold((mxc_i2c_regs_t *)i2c, 6); // set RX threshold to 6 bytes

    if (!masterMode) {
        MXC_I2C_SetSlaveAddr((mxc_i2c_regs_t *)i2c, slaveAddr, 0);
        states[MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c)].master = 0;
    } else {
        i2c->ctrl |= MXC_F_I2C_REVA_CTRL_MST_MODE;
        states[MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c)].master = 1;
    }

    // Prepare I2C instance state.
    states[i2cNum].channelTx = E_NO_DEVICE;
    states[i2cNum].channelRx = E_NO_DEVICE;
    states[i2cNum].writeDone = 0;
    states[i2cNum].readDone = 0;
    states[i2cNum].dma_initialized = false;

    return E_NO_ERROR;
}

int MXC_I2C_RevA_SetSlaveAddr(mxc_i2c_reva_regs_t *i2c, unsigned int slaveAddr, int idx)
{
    if (i2c == NULL) {
        return E_NULL_PTR;
    }

    if (idx >= (sizeof(i2c->slave_multi) / sizeof(uint32_t))) {
        return E_NOT_SUPPORTED;
    }

    if (slaveAddr > MXC_F_I2C_REVA_SLAVE_MULTI_ADDR) {
        // Only support addresses up to 10 bits
        return E_BAD_PARAM;
    }

    i2c->slave_multi[idx] = 0;

    if (slaveAddr > MXC_I2C_REVA_MAX_ADDR_WIDTH) {
        // Set for 10bit addressing mode
        i2c->slave_multi[idx] = MXC_F_I2C_REVA_SLAVE_MULTI_EXT_ADDR_EN;
    }

    i2c->slave_multi[idx] |= slaveAddr;

    return E_NO_ERROR;
}

int MXC_I2C_RevA_Shutdown(mxc_i2c_reva_regs_t *i2c)
{
    int8_t i2cNum;

    if (i2c == NULL) {
        return E_NULL_PTR;
    }

    i2cNum = MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c);

    i2c->ctrl = 0;
    i2c->inten0 = 0;
    i2c->inten1 = 0;
    i2c->intfl0 = i2c->intfl0;
    i2c->intfl1 = i2c->intfl1;
    i2c->rxctrl0 = 0;
    i2c->rxctrl1 = 0;
    i2c->txctrl0 = 0;
    i2c->txctrl1 = 0;

    states[i2cNum] = (const mxc_i2c_reva_req_state_t){ 0 };

    MXC_I2C_ClearRXFIFO((mxc_i2c_regs_t *)i2c);
    MXC_I2C_ClearTXFIFO((mxc_i2c_regs_t *)i2c);

    if (states[i2cNum].dma_initialized == true) {
#if TARGET_NUM == 32665
        MXC_DMA_DeInit((mxc_dma_regs_t *)(states[i2cNum].dma));
#else
        MXC_DMA_DeInit();
#endif
        // Release any acquired DMA channels.
        if (states[i2cNum].channelTx >= 0) {
            MXC_DMA_ReleaseChannel(states[i2cNum].channelTx);
            states[i2cNum].channelTx = E_NO_DEVICE;
        }
        if (states[i2cNum].channelRx >= 0) {
            MXC_DMA_ReleaseChannel(states[i2cNum].channelRx);
            states[i2cNum].channelRx = E_NO_DEVICE;
        }
    }

    return E_NO_ERROR;
}

int MXC_I2C_RevA_SetFrequency(mxc_i2c_reva_regs_t *i2c, unsigned int hz)
{
    unsigned int ticksTotal, hiClks, lowClks;

    if (i2c == NULL) {
        return E_NULL_PTR;
    }

    if (hz > MXC_I2C_REVA_FASTPLUS_SPEED && hz <= MXC_I2C_REVA_HIGH_SPEED) {
        // Enable high speed mode
        int hsLowClks, hsHiClks;

        // Calculate the period of SCL and set up 33% duty cycle
        ticksTotal = PeripheralClock / hz;
        hsLowClks = (ticksTotal * 2) / 3 - 1;
        hsHiClks = ticksTotal / 3 - 1;

        // For rounding errors, adjust by 1 clock tick
        if (ticksTotal % 2) {
            hsHiClks++;
        }

        // If we're too slow for high speed, bail out
        if ((hsHiClks > 0xF) || (hsLowClks > 0xF)) {
            return E_BAD_PARAM;
        }

        hsLowClks = (hsLowClks << MXC_F_I2C_REVA_HSCLK_LO_POS) & MXC_F_I2C_REVA_HSCLK_LO;
        hsHiClks = (hsHiClks << MXC_F_I2C_REVA_HSCLK_HI_POS) & MXC_F_I2C_REVA_HSCLK_HI;

        i2c->hsclk = (hsLowClks | hsHiClks);

        i2c->ctrl |= MXC_F_I2C_REVA_CTRL_HS_EN;

        hz = MXC_I2C_REVA_FAST_SPEED; // High speed preambles will be sent at 400kHz

    } else if (hz > MXC_I2C_REVA_HIGH_SPEED) {
        return E_BAD_PARAM;
    }

    // Calculate the period of SCL, 50% duty cycle
    ticksTotal = PeripheralClock / hz;
    hiClks = (ticksTotal >> 1) - 1;
    lowClks = (ticksTotal >> 1) - 1;

    // Adjust for rounding errors
    if (ticksTotal % 2) {
        hiClks++;
    }

    // Check for maximum/minimum supported speeds
    if ((hiClks > MXC_F_I2C_REVA_CLKHI_HI) || (lowClks == 0)) {
        return E_BAD_PARAM;
    }

    i2c->clklo = lowClks & MXC_F_I2C_REVA_CLKLO_LO;
    i2c->clkhi = hiClks & MXC_F_I2C_REVA_CLKHI_HI;

    // Return the actual speed set, since it won't be exactly what's requested
    return MXC_I2C_GetFrequency((mxc_i2c_regs_t *)i2c);
}

unsigned int MXC_I2C_RevA_GetFrequency(mxc_i2c_reva_regs_t *i2c)
{
    unsigned int sclCycles = 2;
    // sclCycles Initialized to 2 b/c formula is sclCycles = (lo_clks + 1) + (hi_clks + 1)

    if (i2c->ctrl & MXC_F_I2C_REVA_CTRL_HS_EN) {
        // HS-Mode enabled, calculate HS Frequency
        sclCycles += (i2c->hsclk & MXC_F_I2C_REVA_HSCLK_LO) >> MXC_F_I2C_REVA_HSCLK_LO_POS;
        sclCycles += (i2c->hsclk & MXC_F_I2C_REVA_HSCLK_HI) >> MXC_F_I2C_REVA_HSCLK_HI_POS;
    } else {
        // HS-Mode not enabled, calculate nominal frequency
        sclCycles += (i2c->clklo & MXC_F_I2C_REVA_CLKLO_LO);
        sclCycles += (i2c->clkhi & MXC_F_I2C_REVA_CLKHI_HI);
    }

    return PeripheralClock / sclCycles;
}

int MXC_I2C_RevA_ReadyForSleep(mxc_i2c_reva_regs_t *i2c)
{
    if (MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c) < 0) {
        return E_BAD_PARAM;
    }

    if (AsyncRequests[MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c)] != NULL) {
        return E_BUSY;
    }

    return E_NO_ERROR;
}

int MXC_I2C_RevA_SetClockStretching(mxc_i2c_reva_regs_t *i2c, int enable)
{
    if (i2c == NULL) {
        return E_NULL_PTR;
    }

    if (enable) {
        i2c->ctrl &= ~MXC_F_I2C_REVA_CTRL_CLKSTR_DIS;
    } else {
        i2c->ctrl |= MXC_F_I2C_REVA_CTRL_CLKSTR_DIS;
    }

    return E_NO_ERROR;
}

int MXC_I2C_RevA_GetClockStretching(mxc_i2c_reva_regs_t *i2c)
{
    if (i2c == NULL) {
        return E_NULL_PTR;
    }

    return !((i2c->ctrl & MXC_F_I2C_REVA_CTRL_CLKSTR_DIS) >> MXC_F_I2C_REVA_CTRL_CLKSTR_DIS_POS);
}

int MXC_I2C_RevA_DMA_Init(mxc_i2c_reva_regs_t *i2c, mxc_dma_reva_regs_t *dma, bool use_dma_tx,
                          bool use_dma_rx)
{
    int8_t i2cNum;
    int8_t rxChannel = -1;
    int8_t txChannel = -1;

    if (i2c == NULL || dma == NULL) {
        return E_NULL_PTR;
    }

    i2cNum = MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c);

    if (states[i2cNum].dma_initialized == false) {
#if TARGET_NUM == 32665
        MXC_DMA_Init((mxc_dma_regs_t *)dma);
#else
        MXC_DMA_Init();
#endif
    }

    // Release any acquire DMA TX channels before configuring.
    if (states[i2cNum].channelTx != E_NO_DEVICE) {
        MXC_DMA_ReleaseChannel(states[i2cNum].channelTx);
    }

    // Release any acquire DMA RX channels before configuring.
    if (states[i2cNum].channelRx != E_NO_DEVICE) {
        MXC_DMA_ReleaseChannel(states[i2cNum].channelRx);
    }

    // Set up I2C DMA TX.
    if (use_dma_tx == true) {
#if TARGET_NUM == 32665
        txChannel = MXC_DMA_AcquireChannel((mxc_dma_regs_t *)dma);
#else
        txChannel = MXC_DMA_AcquireChannel();
#endif

        // Set Source and Destination Widths.
        MXC_SETFIELD(dma->ch[txChannel].ctrl, MXC_F_DMA_REVA_CTRL_SRCWD,
                     (MXC_DMA_WIDTH_BYTE << MXC_F_DMA_REVA_CTRL_SRCWD_POS));
        MXC_SETFIELD(dma->ch[txChannel].ctrl, MXC_F_DMA_REVA_CTRL_DSTWD,
                     (MXC_DMA_WIDTH_BYTE << MXC_F_DMA_REVA_CTRL_DSTWD_POS));

        // Set Source and Destination Increment.
        MXC_SETFIELD(dma->ch[txChannel].ctrl, MXC_F_DMA_REVA_CTRL_SRCINC,
                     (1 << MXC_F_DMA_REVA_CTRL_SRCINC_POS));
        MXC_SETFIELD(dma->ch[txChannel].ctrl, MXC_F_DMA_REVA_CTRL_DSTINC,
                     (0 << MXC_F_DMA_REVA_CTRL_DSTINC_POS));

        if (states[i2cNum].master) {
            MXC_DMA_SetCallback(txChannel, MXC_I2C_RevA_DMACallback);
        } else {
            MXC_DMA_SetCallback(txChannel, NULL);
        }

        MXC_DMA_EnableInt(txChannel);
        MXC_DMA_SetChannelInterruptEn(txChannel, 0, 1);

        states[i2cNum].channelTx = txChannel;
#ifdef __arm__
        NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(txChannel));
#if TARGET_NUM == 32665
        MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(txChannel),
                           MXC_DMA_Get_DMA_Handler((mxc_dma_regs_t *)dma));
#else
        MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(txChannel), MXC_DMA_Handler);
#endif
#endif
    }

    // Set up I2C DMA RX.
    if (use_dma_rx == true) {
#if TARGET_NUM == 32665
        rxChannel = MXC_DMA_AcquireChannel((mxc_dma_regs_t *)dma);
#else
        rxChannel = MXC_DMA_AcquireChannel();
#endif

        // Set Source and Destination Widths.
        MXC_SETFIELD(dma->ch[rxChannel].ctrl, MXC_F_DMA_REVA_CTRL_SRCWD,
                     (MXC_DMA_WIDTH_BYTE << MXC_F_DMA_REVA_CTRL_SRCWD_POS));
        MXC_SETFIELD(dma->ch[rxChannel].ctrl, MXC_F_DMA_REVA_CTRL_DSTWD,
                     (MXC_DMA_WIDTH_BYTE << MXC_F_DMA_REVA_CTRL_DSTWD_POS));

        // Set Source and Destination Increment.
        MXC_SETFIELD(dma->ch[rxChannel].ctrl, MXC_F_DMA_REVA_CTRL_SRCINC,
                     (0 << MXC_F_DMA_REVA_CTRL_SRCINC_POS));
        MXC_SETFIELD(dma->ch[rxChannel].ctrl, MXC_F_DMA_REVA_CTRL_DSTINC,
                     (1 << MXC_F_DMA_REVA_CTRL_DSTINC_POS));

        if (states[i2cNum].master) {
            MXC_DMA_SetCallback(rxChannel, MXC_I2C_RevA_DMACallback);
        } else {
            MXC_DMA_SetCallback(rxChannel, NULL);
        }

        MXC_DMA_EnableInt(rxChannel);
        MXC_DMA_SetChannelInterruptEn(rxChannel, 0, 1);
#ifdef __arm__
        NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(rxChannel));
#if TARGET_NUM == 32665
        MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(txChannel),
                           MXC_DMA_Get_DMA_Handler((mxc_dma_regs_t *)dma));
#else
        MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(txChannel), MXC_DMA_Handler);
#endif
#endif

        states[i2cNum].channelRx = rxChannel;
    }

    states[i2cNum].dma_initialized = true;
    states[i2cNum].dma = dma;

    return E_NO_ERROR;
}

int MXC_I2C_RevA_DMA_GetTXChannel(mxc_i2c_reva_regs_t *i2c)
{
    int i2cNum;

    if (i2c == NULL) {
        return E_NULL_PTR;
    }

    i2cNum = MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c);

    return states[i2cNum].channelTx;
}

int MXC_I2C_RevA_DMA_GetRXChannel(mxc_i2c_reva_regs_t *i2c)
{
    int i2cNum;

    if (i2c == NULL) {
        return E_NULL_PTR;
    }

    i2cNum = MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c);

    return states[i2cNum].channelRx;
}

int MXC_I2C_RevA_DMA_SetRequestSelect(mxc_i2c_reva_regs_t *i2c, mxc_dma_reva_regs_t *dma,
                                      uint32_t txReqSel, uint32_t rxReqSel)
{
    int i2cNum;
    uint32_t txChannel;
    uint32_t rxChannel;

    if (i2c == NULL || dma == NULL) {
        return E_NULL_PTR;
    }

    i2cNum = MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c);

    txChannel = states[i2cNum].channelTx;
    rxChannel = states[i2cNum].channelRx;

    // This function will overwrite the current DMA TX/RX Request Selects.
    if (txReqSel != -1) {
        MXC_SETFIELD(dma->ch[txChannel].ctrl, MXC_F_DMA_REVA_CTRL_REQUEST, txReqSel);
    }

    if (rxReqSel != -1) {
        MXC_SETFIELD(dma->ch[rxChannel].ctrl, MXC_F_DMA_REVA_CTRL_REQUEST, rxReqSel);
    }

    return E_NO_ERROR;
}

/* ************************************************************************* */
/* Low-level functions                                                       */
/* ************************************************************************* */
int MXC_I2C_RevA_Start(mxc_i2c_reva_regs_t *i2c)
{
    if (i2c == NULL) {
        return E_NULL_PTR;
    }

    // If we have an incomplete transfer, we need to do a restart
    if (i2c->mstctrl & MXC_F_I2C_REVA_MSTCTRL_START) {
        i2c->mstctrl |= MXC_F_I2C_REVA_MSTCTRL_RESTART;
    } else {
        i2c->mstctrl |= MXC_F_I2C_REVA_MSTCTRL_START; // No check for start generation
    }

    return E_NO_ERROR;
}

int MXC_I2C_RevA_Stop(mxc_i2c_reva_regs_t *i2c)
{
    if (i2c == NULL) {
        return E_NULL_PTR;
    }

    i2c->mstctrl |= MXC_F_I2C_REVA_MSTCTRL_STOP;

    while (i2c->mstctrl & MXC_F_I2C_REVA_MSTCTRL_STOP) {}

    return E_NO_ERROR;
}

int MXC_I2C_RevA_WriteByte(mxc_i2c_reva_regs_t *i2c, unsigned char byte)
{
    if (i2c == NULL) {
        return E_NULL_PTR;
    }

    if (!(i2c->status & MXC_F_I2C_REVA_STATUS_TX_EM)) {
        return E_OVERFLOW;
    }

    // I'm depending on an interrupt flag here
    // This might cause issues with the transaction level functions to come
    MXC_I2C_ClearFlags((mxc_i2c_regs_t *)i2c, MXC_I2C_REVA_INTFL0_MASK, MXC_I2C_REVA_INTFL1_MASK);
    i2c->fifo = byte;

    while (!(i2c->status & MXC_F_I2C_REVA_STATUS_TX_EM)) {}

    return i2c->intfl0 & MXC_F_I2C_REVA_INTFL0_DATA_ERR;
}

int MXC_I2C_RevA_ReadByte(mxc_i2c_reva_regs_t *i2c, unsigned char *byte, int ack)
{
    if ((i2c == NULL) || (byte == NULL)) {
        return E_NULL_PTR;
    }

    if (i2c->status & MXC_F_I2C_REVA_STATUS_RX_EM) {
        return E_UNDERFLOW;
    }

    *byte = (uint8_t)(i2c->fifo & MXC_F_I2C_REVA_FIFO_DATA);

    if (ack) {
        i2c->ctrl &= ~MXC_F_I2C_REVA_CTRL_IRXM_ACK;
    } else {
        i2c->ctrl |= MXC_F_I2C_REVA_CTRL_IRXM_ACK;
    }

    return E_NO_ERROR;
}

int MXC_I2C_RevA_ReadByteInteractive(mxc_i2c_reva_regs_t *i2c, unsigned char *byte,
                                     mxc_i2c_reva_getAck_t getAck)
{
    if ((i2c == NULL) || (byte == NULL)) {
        return E_NULL_PTR;
    }

    if (!(i2c->status & MXC_F_I2C_REVA_STATUS_RX_EM)) {
        return E_UNDERFLOW;
    }

    *byte = (uint8_t)(i2c->fifo & MXC_F_I2C_REVA_FIFO_DATA);

    if (getAck == NULL) {
        i2c->ctrl &= ~MXC_F_I2C_REVA_CTRL_IRXM_ACK_POS;
    } else {
        i2c->ctrl |= (!!getAck((mxc_i2c_reva_regs_t *)i2c, *byte))
                     << MXC_F_I2C_REVA_CTRL_IRXM_ACK_POS;
    }

    return E_NO_ERROR;
}

int MXC_I2C_RevA_Write(mxc_i2c_reva_regs_t *i2c, unsigned char *bytes, unsigned int *len)
{
    int notAcked = 0;
    unsigned written = 0;

    if (i2c == NULL) {
        return E_NULL_PTR;
    }

    if ((bytes == NULL) || (len == NULL)) {
        return E_NULL_PTR;
    }

    for (; written < *len; written++) {
        int retVal = MXC_I2C_WriteByte((mxc_i2c_regs_t *)i2c, bytes[written]);

        if (retVal >= 0) {
            notAcked += retVal;
        } else {
            *len = written;
            return retVal;
        }
    }

    *len = written;
    notAcked = (notAcked > 0) ? 1 : 0;
    return notAcked;
}

int MXC_I2C_RevA_Read(mxc_i2c_reva_regs_t *i2c, unsigned char *bytes, unsigned int *len, int ack)
{
    unsigned read = 0;

    if (i2c == NULL) {
        return E_NULL_PTR;
    }

    if ((bytes == NULL) || (len == NULL)) {
        return E_NULL_PTR;
    }

    for (; read < *len - 1; read++) {
        int retVal = MXC_I2C_ReadByte((mxc_i2c_regs_t *)i2c, &(bytes[read]), 1);

        if (retVal != E_NO_ERROR) {
            *len = read;
            return retVal;
        }
    }

    read++;
    *len = read;
    return MXC_I2C_ReadByte((mxc_i2c_regs_t *)i2c, &(bytes[read]), ack);
}

int MXC_I2C_RevA_ReadRXFIFO(mxc_i2c_reva_regs_t *i2c, volatile unsigned char *bytes,
                            unsigned int len)
{
    unsigned read = 0;

    if ((i2c == NULL) || (bytes == NULL)) {
        return E_NULL_PTR;
    }

    while ((len > read) && (!(i2c->status & MXC_F_I2C_REVA_STATUS_RX_EM))) {
        bytes[read++] = i2c->fifo;
    }

    return read;
}

int MXC_I2C_RevA_ReadRXFIFODMA(mxc_i2c_reva_regs_t *i2c, unsigned char *bytes, unsigned int len,
                               mxc_dma_regs_t *dma)
{
    uint8_t i2cNum;
    mxc_dma_srcdst_t srcdst;

    if ((i2c == NULL) || (bytes == NULL)) {
        return E_NULL_PTR;
    }

    i2cNum = MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c);

    if (states[i2cNum].channelRx == E_NO_DEVICE) {
        return E_BAD_STATE;
    }

    srcdst.ch = states[i2cNum].channelRx;
    srcdst.dest = bytes;
    srcdst.len = len;

    MXC_DMA_SetSrcDst(srcdst);

    MXC_DMA_Start(states[i2cNum].channelRx);
    i2c->dma |= MXC_F_I2C_REVA_DMA_RX_EN;

    return E_NO_ERROR;
}

int MXC_I2C_RevA_GetRXFIFOAvailable(mxc_i2c_reva_regs_t *i2c)
{
    if (i2c == NULL) {
        return E_NULL_PTR;
    }

    return (i2c->rxctrl1 & MXC_F_I2C_REVA_RXCTRL1_LVL) >> MXC_F_I2C_REVA_RXCTRL1_LVL_POS;
}

int MXC_I2C_RevA_WriteTXFIFO(mxc_i2c_reva_regs_t *i2c, volatile unsigned char *bytes,
                             unsigned int len)
{
    unsigned written = 0;

    if ((i2c == NULL) || (bytes == NULL)) {
        return E_NULL_PTR;
    }

    while ((len > written) && (!(i2c->status & MXC_F_I2C_REVA_STATUS_TX_FULL))) {
        i2c->fifo = bytes[written++];
    }

    return written;
}

int MXC_I2C_RevA_WriteTXFIFODMA(mxc_i2c_reva_regs_t *i2c, unsigned char *bytes, unsigned int len,
                                mxc_dma_regs_t *dma)
{
    int8_t i2cNum;
    mxc_dma_srcdst_t srcdst;

    if ((i2c == NULL) || (bytes == NULL)) {
        return E_NULL_PTR;
    }

    i2cNum = MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c);

    i2c->mstctrl |= MXC_F_I2C_REVA_MSTCTRL_START;

    if (states[i2cNum].channelTx == E_NO_DEVICE) {
        return E_BAD_STATE;
    }

    srcdst.ch = states[i2cNum].channelTx;
    srcdst.source = bytes;
    srcdst.len = len;

    MXC_DMA_SetSrcDst(srcdst);

    MXC_DMA_Start(states[i2cNum].channelTx);
    i2c->dma |= MXC_F_I2C_REVA_DMA_TX_EN;

    return E_NO_ERROR;
}

int MXC_I2C_RevA_GetTXFIFOAvailable(mxc_i2c_reva_regs_t *i2c)
{
    if (i2c == NULL) {
        return E_NULL_PTR;
    }

    int txFIFOlen = (i2c->fifolen & MXC_F_I2C_REVA_FIFOLEN_TX_DEPTH) >>
                    MXC_F_I2C_REVA_FIFOLEN_TX_DEPTH_POS;
    return txFIFOlen -
           ((i2c->txctrl1 & MXC_F_I2C_REVA_TXCTRL1_LVL) >> MXC_F_I2C_REVA_TXCTRL1_LVL_POS);
}

void MXC_I2C_RevA_ClearRXFIFO(mxc_i2c_reva_regs_t *i2c)
{
    i2c->rxctrl0 |= MXC_F_I2C_REVA_RXCTRL0_FLUSH;

    while (i2c->rxctrl0 & MXC_F_I2C_REVA_RXCTRL0_FLUSH) {}
}

void MXC_I2C_RevA_ClearTXFIFO(mxc_i2c_reva_regs_t *i2c)
{
    i2c->txctrl0 |= MXC_F_I2C_REVA_TXCTRL0_FLUSH;

    while (i2c->txctrl0 & MXC_F_I2C_REVA_TXCTRL0_FLUSH) {}
}

int MXC_I2C_RevA_GetFlags(mxc_i2c_reva_regs_t *i2c, unsigned int *flags0, unsigned int *flags1)
{
    if (i2c == NULL) {
        return E_NULL_PTR;
    }

    if ((flags0 == NULL) || (flags1 == NULL)) {
        return E_BAD_PARAM;
    }

    *flags0 = i2c->intfl0;
    *flags1 = i2c->intfl1;

    return E_NO_ERROR;
}

void MXC_I2C_RevA_ClearFlags(mxc_i2c_reva_regs_t *i2c, unsigned int flags0, unsigned int flags1)
{
    i2c->intfl0 = flags0;
    i2c->intfl1 = flags1;
}

void MXC_I2C_RevA_EnableInt(mxc_i2c_reva_regs_t *i2c, unsigned int flags0, unsigned int flags1)
{
    i2c->inten0 |= flags0;
    i2c->inten1 |= flags1;
}

void MXC_I2C_RevA_DisableInt(mxc_i2c_reva_regs_t *i2c, unsigned int flags0, unsigned int flags1)
{
    i2c->inten0 &= ~flags0;
    i2c->inten1 &= ~flags1;
}

int MXC_I2C_RevA_Recover(mxc_i2c_reva_regs_t *i2c, unsigned int retries)
{
    int err;
    unsigned int i;

    if (i2c == NULL) {
        return E_NULL_PTR;
    }

    err = E_COMM_ERR;

    i2c->ctrl |= MXC_F_I2C_REVA_CTRL_EN;
    int swBit = i2c->ctrl & MXC_F_I2C_REVA_CTRL_BB_MODE;

    if (i2c == NULL) {
        return E_NULL_PTR;
    }

    i2c->ctrl |= MXC_F_I2C_REVA_CTRL_BB_MODE;

    // Follow the procedure detailed in the header file
    // Delay 10uS between each step to give the line/slaves time to react
    for (i = 0; i < retries; i++) {
        MXC_Delay(10);
        i2c->ctrl &= ~MXC_F_I2C_REVA_CTRL_SCL_OUT;

        MXC_Delay(10);

        if (i2c->ctrl & MXC_F_I2C_REVA_CTRL_SCL) {
            i2c->ctrl |= MXC_F_I2C_REVA_CTRL_SCL_OUT | MXC_F_I2C_REVA_CTRL_SDA_OUT;
            continue; // Give up and try again
        }

        MXC_Delay(10);
        i2c->ctrl &= ~MXC_F_I2C_REVA_CTRL_SDA_OUT;

        MXC_Delay(10);

        if (i2c->ctrl & MXC_F_I2C_REVA_CTRL_SDA) {
            i2c->ctrl |= MXC_F_I2C_REVA_CTRL_SCL_OUT | MXC_F_I2C_REVA_CTRL_SDA_OUT;
            continue; // Give up and try again
        }

        MXC_Delay(10);
        i2c->ctrl |= MXC_F_I2C_REVA_CTRL_SDA_OUT;

        MXC_Delay(10);

        if (!(i2c->ctrl & MXC_F_I2C_REVA_CTRL_SDA)) {
            i2c->ctrl |= MXC_F_I2C_REVA_CTRL_SCL_OUT | MXC_F_I2C_REVA_CTRL_SDA_OUT;
            continue; // Give up and try again
        }

        MXC_Delay(10);
        i2c->ctrl |= MXC_F_I2C_REVA_CTRL_SCL_OUT;

        MXC_Delay(10);

        if (i2c->ctrl & MXC_F_I2C_REVA_CTRL_SCL) {
            err = E_NO_ERROR; // We have control
            break;
        }
    }

    if (swBit == 0) {
        i2c->ctrl &= ~MXC_F_I2C_REVA_CTRL_BB_MODE;
    }

    i2c->ctrl &= ~MXC_F_I2C_REVA_CTRL_EN;

    return err;
}

void MXC_I2C_RevA_EnablePreload(mxc_i2c_reva_regs_t *i2c)
{
    i2c->txctrl0 |= MXC_F_I2C_REVA_TXCTRL0_PRELOAD_MODE;
}

void MXC_I2C_RevA_DisablePreload(mxc_i2c_reva_regs_t *i2c)
{
    i2c->txctrl0 &= ~MXC_F_I2C_REVA_TXCTRL0_PRELOAD_MODE;
}

void MXC_I2C_RevA_EnableGeneralCall(mxc_i2c_reva_regs_t *i2c)
{
    i2c->ctrl |= MXC_F_I2C_REVA_CTRL_GC_ADDR_EN;
}

void MXC_I2C_RevA_DisableGeneralCall(mxc_i2c_reva_regs_t *i2c)
{
    i2c->ctrl &= ~MXC_F_I2C_REVA_CTRL_GC_ADDR_EN;
}

void MXC_I2C_RevA_SetTimeout(mxc_i2c_reva_regs_t *i2c, unsigned int timeout)
{
    i2c->timeout |= (timeout & 0xFFFF);
}

unsigned int MXC_I2C_RevA_GetTimeout(mxc_i2c_reva_regs_t *i2c)
{
    return (i2c->timeout & 0xFFFF);
}

/* ************************************************************************* */
/* Transaction level functions                                               */
/* ************************************************************************* */

int MXC_I2C_RevA_MasterTransaction(mxc_i2c_reva_req_t *req)
{
    mxc_i2c_reva_regs_t *i2c = req->i2c; // Save off pointer for faster access
    unsigned int written = 0;
    unsigned int read = 0;

    if (req->addr > MXC_I2C_REVA_MAX_ADDR_WIDTH) {
        return E_NOT_SUPPORTED;
    }

    if (MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c) < 0) {
        return E_BAD_PARAM;
    }

    if (!(i2c->ctrl & MXC_F_I2C_REVA_CTRL_MST_MODE)) {
        return E_BAD_STATE;
    }

    // Check and return BUSY if the Bus is not ready
    if (i2c->status & MXC_F_I2C_REVA_STATUS_BUSY) {
        return E_BUSY;
    }

    // if(!read | write)
    //  Start
    //  send addr w/ write bit
    // if(Write)
    //  send tx_len data
    //  return if error (or NACK)
    // if(Read)
    //  if(Write)
    //   send restart
    //  else
    //   send start
    //  send addr w/ read bit
    //  read rx_len bytes acking all
    // stop or restart
    // return good or error

    MXC_I2C_ClearFlags((mxc_i2c_regs_t *)i2c, MXC_I2C_REVA_INTFL0_MASK,
                       MXC_I2C_REVA_INTFL1_MASK); // Clear all I2C Interrupts
    MXC_I2C_ClearTXFIFO((mxc_i2c_regs_t *)i2c);
    MXC_I2C_ClearRXFIFO((mxc_i2c_regs_t *)i2c);
    i2c->inten0 = 0;
    i2c->inten1 = 0;

    if ((req->rx_len == 0) || (req->tx_len != 0)) {
        // Load the slave address with write bit set
        i2c->fifo = (req->addr << 1) & ~0x1;
        i2c->mstctrl |= MXC_F_I2C_REVA_MSTCTRL_START;
    }

    while (req->tx_len > written) {
        if (i2c->intfl0 & MXC_F_I2C_REVA_INTFL0_TX_THD) {
            written += MXC_I2C_WriteTXFIFO((mxc_i2c_regs_t *)i2c, &req->tx_buf[written],
                                           req->tx_len - written);
            i2c->intfl0 = MXC_F_I2C_REVA_INTFL0_TX_THD;
        }

        if (i2c->intfl0 & MXC_I2C_REVA_ERROR) {
            req->tx_len = written;
            MXC_I2C_Stop((mxc_i2c_regs_t *)i2c);
            return E_COMM_ERR;
        }
    }

    MXC_I2C_ClearFlags((mxc_i2c_regs_t *)i2c,
                       MXC_F_I2C_REVA_INTFL0_DONE | MXC_F_I2C_REVA_INTFL0_RX_THD, 0);

    if (req->rx_len != 0) {
        if (req->rx_len > MXC_I2C_REVA_MAX_FIFO_TRANSACTION) {
            i2c->rxctrl1 = 0;
        } else {
            i2c->rxctrl1 = req->rx_len; // 0 for 256, otherwise number of bytes to read
        }

        MXC_I2C_Start((mxc_i2c_regs_t *)i2c); // Start or Restart as needed

        while (i2c->mstctrl & MXC_F_I2C_REVA_MSTCTRL_RESTART) {}

        i2c->fifo = (req->addr << 1) | 0x1; // Load slave address with read bit.
    }

    if ((req->rx_len != 0) && (req->tx_len != 0)) {
        while (!(i2c->intfl0 & MXC_F_I2C_REVA_INTFL0_DONE)) {}
        // Wait for Transaction to finish

        i2c->intfl0 |= MXC_F_I2C_REVA_INTFL0_DONE;
    }

    while (req->rx_len > read) {
        if (i2c->intfl0 & (MXC_F_I2C_REVA_INTFL0_RX_THD | MXC_F_I2C_REVA_INTFL0_DONE)) {
            read +=
                MXC_I2C_ReadRXFIFO((mxc_i2c_regs_t *)i2c, &req->rx_buf[read], req->rx_len - read);
            i2c->intfl0 = MXC_F_I2C_REVA_INTFL0_RX_THD;
        }

        if (i2c->intfl0 & MXC_I2C_REVA_ERROR) {
            req->rx_len = read;
            MXC_I2C_Stop((mxc_i2c_regs_t *)i2c);
            return E_COMM_ERR;
        }

        if ((i2c->intfl0 & MXC_F_I2C_REVA_INTFL0_DONE) && (req->rx_len > read) &&
            (MXC_I2C_RevA_GetRXFIFOAvailable(i2c) == 0)) {
            if ((req->rx_len - read) > MXC_I2C_REVA_MAX_FIFO_TRANSACTION) {
                i2c->rxctrl1 = 0;
            } else {
                i2c->rxctrl1 = (req->rx_len - read); // 0 for 256, otherwise number of bytes to read
            }

            i2c->mstctrl |= MXC_F_I2C_REVA_MSTCTRL_RESTART;
            i2c->intfl0 = MXC_F_I2C_REVA_INTFL0_DONE;
            i2c->fifo = (req->addr << 1) | 0x1; // Load slave address with read bit.
        }
    }

    if (req->restart) {
        i2c->mstctrl |= MXC_F_I2C_REVA_MSTCTRL_RESTART;
    } else {
        i2c->mstctrl |= MXC_F_I2C_REVA_MSTCTRL_STOP;

        while ((i2c->mstctrl & MXC_F_I2C_REVA_MSTCTRL_STOP)) {}
        // Wait for Transaction to finish
    }

    while (!(i2c->intfl0 & MXC_F_I2C_REVA_INTFL0_DONE)) {}
    // Wait for Transaction to finish

    i2c->intfl0 = MXC_F_I2C_REVA_INTFL0_DONE | MXC_F_I2C_REVA_INTFL0_STOP;

    if (i2c->intfl0 & MXC_I2C_REVA_ERROR) {
        return E_COMM_ERR;
    }

    return E_NO_ERROR;
}

int MXC_I2C_RevA_MasterTransactionAsync(mxc_i2c_reva_req_t *req)
{
    int i2cNum = MXC_I2C_GET_IDX((mxc_i2c_regs_t *)(req->i2c));
    mxc_i2c_reva_regs_t *i2c = req->i2c;

    if (i2cNum < 0) {
        return E_BAD_PARAM;
    }

    if (!(i2c->ctrl & MXC_F_I2C_REVA_CTRL_MST_MODE)) {
        return E_BAD_STATE;
    }

    if (AsyncRequests[i2cNum] == NULL) {
        if (req->addr > MXC_I2C_REVA_MAX_ADDR_WIDTH) {
            return E_NOT_SUPPORTED;
        }

        AsyncRequests[i2cNum] = (void *)req;
        AsyncWritten[i2cNum] = 0;
        AsyncRead[i2cNum] = 0;
        MXC_I2C_ClearFlags((mxc_i2c_regs_t *)i2c, MXC_I2C_REVA_INTFL0_MASK,
                           MXC_I2C_REVA_INTFL1_MASK); // Clear all I2C Interrupts
        MXC_I2C_ClearTXFIFO((mxc_i2c_regs_t *)i2c);
        MXC_I2C_ClearRXFIFO((mxc_i2c_regs_t *)i2c);

        i2c->inten0 = MXC_I2C_REVA_ERROR;

        if (req->tx_len) {
            i2c->fifo = (req->addr << 1) & ~0x1; // Load the slave address with write bit set
        } else if (req->rx_len) {
            i2c->fifo = (req->addr << 1) | 0x1; // Load the slave address with read bit set

            /* Set the number of bytes to read */
            if (req->rx_len > MXC_I2C_REVA_MAX_FIFO_TRANSACTION) {
                i2c->rxctrl1 = 0;
            } else {
                i2c->rxctrl1 = req->rx_len; // 0 for 256, otherwise number of bytes to read
            }

            /* Enable RX Threshold interrupt for when the FIFO is full */
            i2c->inten0 |= (MXC_F_I2C_REVA_INTEN0_RX_THD | MXC_F_I2C_REVA_INTEN0_DONE);
        } else {
            /* Must have tx_len and/or rx_len */
            return E_BAD_PARAM;
        }

        MXC_I2C_Start((mxc_i2c_regs_t *)i2c);

        /* Fill the FIFO as nessary */
        MXC_I2C_RevA_MasterAsyncHandler(i2cNum);

        return E_NO_ERROR;
    } else {
        return E_BUSY;
    }
}

int MXC_I2C_RevA_MasterTransactionDMA(mxc_i2c_reva_req_t *req, mxc_dma_regs_t *dma)
{
    int error;
    int8_t i2cNum;

    mxc_i2c_reva_regs_t *i2c = req->i2c; // Save off pointer for faster access
    i2cNum = MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c);

    if (req->addr > MXC_I2C_REVA_MAX_ADDR_WIDTH) {
        return E_NOT_SUPPORTED;
    }

    if (i2cNum < 0) {
        return E_BAD_PARAM;
    }

    if (!(i2c->ctrl & MXC_F_I2C_REVA_CTRL_MST_MODE)) {
        return E_BAD_STATE;
    }

    if (req->rx_len > MXC_I2C_REVA_MAX_FIFO_TRANSACTION) {
        return E_BAD_PARAM;
    }

    MXC_I2C_ClearFlags((mxc_i2c_regs_t *)i2c, MXC_I2C_REVA_INTFL0_MASK,
                       MXC_I2C_REVA_INTFL1_MASK); // Clear all I2C Interrupts
    MXC_I2C_ClearTXFIFO((mxc_i2c_regs_t *)i2c);
    MXC_I2C_ClearRXFIFO((mxc_i2c_regs_t *)i2c);

    MXC_I2C_SetTXThreshold((mxc_i2c_regs_t *)i2c, 2);
    MXC_I2C_SetRXThreshold((mxc_i2c_regs_t *)i2c, 1);

    states[i2cNum].req = req;
    states[i2cNum].writeDone = (req->tx_len == 0);
    states[i2cNum].readDone = (req->rx_len == 0);

    // If MXC_I2C_DMA_Init(...) was not already called, then configure both DMA TX/RXchannels by default.
    if (states[i2cNum].dma_initialized == false) {
        error = MXC_I2C_DMA_Init((mxc_i2c_regs_t *)i2c, (mxc_dma_regs_t *)dma, true, true);
        if (error != E_NO_ERROR) {
            return error;
        }
    }

    error = MXC_I2C_DMA_SetRequestSelect((mxc_i2c_regs_t *)i2c, req->tx_buf, req->rx_buf);
    if (error != E_NO_ERROR) {
        return error;
    }

    //tx
    if ((req->tx_buf != NULL) && !(states[i2cNum].writeDone)) {
        i2c->fifo = ((req->addr) << 1) & ~0x1; // Load the slave address with write bit set

#if TARGET_NUM == 32665
        MXC_I2C_WriteTXFIFODMA((mxc_i2c_regs_t *)i2c, req->tx_buf, req->tx_len, NULL, dma);
#else
        MXC_I2C_WriteTXFIFODMA((mxc_i2c_regs_t *)i2c, req->tx_buf, req->tx_len, NULL);
#endif
    } else {
        states[i2cNum].writeDone = 1;
    }

    if (req->rx_buf != NULL) {
        while (states[i2cNum].writeDone != 1) {}
        //Ensure DMA transmission has finished before attempting to receive

        if ((states[i2cNum].writeDone) && (!states[i2cNum].readDone)) {
            if (req->rx_len > MXC_I2C_REVA_MAX_FIFO_TRANSACTION) {
                i2c->rxctrl1 = 0;
            } else {
                i2c->rxctrl1 = req->rx_len; // 0 for 256, otherwise number of bytes to read
            }

#if TARGET_NUM == 32665
            MXC_I2C_ReadRXFIFODMA((mxc_i2c_regs_t *)i2c, req->rx_buf, req->rx_len, NULL, dma);
#else
            MXC_I2C_ReadRXFIFODMA((mxc_i2c_regs_t *)i2c, req->rx_buf, req->rx_len, NULL);
#endif

            MXC_I2C_Start((mxc_i2c_regs_t *)i2c); // Start or Restart as needed

            while (i2c->mstctrl & MXC_F_I2C_REVA_MSTCTRL_RESTART) {}

            i2c->fifo = ((req->addr) << 1) | 0x1; // Load the slave address with write bit set
            while (!((i2c->intfl0 & MXC_F_I2C_REVA_INTFL0_ADDR_ACK) ||
                     (i2c->intfl0 & MXC_F_I2C_REVA_INTFL0_ADDR_NACK_ERR))) {
                // Wait for an ACK or NACK from the slave
            }
            if (!(i2c->intfl0 & MXC_F_I2C_REVA_INTFL0_ADDR_ACK)) {
                // If we did not get an ACK, then something went wrong.
                // Abort the transaction and signal the user's callback
                MXC_I2C_RevA_Stop(i2c);
                MXC_DMA_Stop(states[i2cNum].channelRx);
                if (states[i2cNum].req->callback != NULL) {
                    states[i2cNum].req->callback(states[i2cNum].req, E_COMM_ERR);
                }
                return E_COMM_ERR;
            }
        }
    } else {
        states[i2cNum].readDone = 1;
    }

    return E_NO_ERROR;
}

void MXC_I2C_RevA_DMACallback(int ch, int error)
{
    mxc_i2c_reva_req_t *temp_req;

    for (int i = 0; i < MXC_I2C_INSTANCES; i++) {
        if (states[i].channelTx == ch) {
            //save the request
            temp_req = states[i].req;
            states[i].writeDone = 1;

            if (states[i].readDone) {
                if (temp_req->restart) {
                    (temp_req->i2c)->mstctrl |= MXC_F_I2C_REVA_MSTCTRL_RESTART;
                } else {
                    (temp_req->i2c)->mstctrl |= MXC_F_I2C_REVA_MSTCTRL_STOP;
                }

                // Callback if not NULL
                if (temp_req->callback != NULL) {
                    temp_req->callback(temp_req, E_NO_ERROR);
                }
            }
        } else if (states[i].channelRx == ch) {
            //save the request
            states[i].readDone = 1;
            temp_req = states[i].req;

            if (states[i].writeDone) {
                if (temp_req->restart) {
                    (temp_req->i2c)->mstctrl |= MXC_F_I2C_REVA_MSTCTRL_RESTART;
                } else {
                    (temp_req->i2c)->mstctrl |= MXC_F_I2C_REVA_MSTCTRL_STOP;
                }

                // Callback if not NULL
                if (temp_req->callback != NULL) {
                    temp_req->callback(temp_req, E_NO_ERROR);
                }
            }
        }
    }
}

int MXC_I2C_RevA_SlaveTransaction(mxc_i2c_reva_regs_t *i2c, mxc_i2c_reva_slave_handler_t callback,
                                  uint32_t interruptCheck)
{
    int retVal = E_NO_ERROR;

    uint32_t int_en[2];
    int_en[0] = interruptCheck;
    int_en[1] = 0;

    if (MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c) < 0) {
        return E_BAD_PARAM;
    }

    if (i2c->ctrl & MXC_F_I2C_REVA_CTRL_MST_MODE) {
        return E_BAD_STATE;
    }

    MXC_I2C_ClearFlags((mxc_i2c_regs_t *)i2c, MXC_I2C_REVA_INTFL0_MASK,
                       MXC_I2C_REVA_INTFL1_MASK); // Clear all I2C Interrupts
    MXC_I2C_ClearTXFIFO((mxc_i2c_regs_t *)i2c);
    MXC_I2C_ClearRXFIFO((mxc_i2c_regs_t *)i2c);

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

    while (int_en[0] > 0 || int_en[1] > 0) {
        MXC_I2C_RevA_SlaveAsyncHandler(i2c, callback, int_en, &retVal);
    }

    return retVal;
}

int MXC_I2C_RevA_SlaveTransactionAsync(mxc_i2c_reva_regs_t *i2c,
                                       mxc_i2c_reva_slave_handler_t callback,
                                       uint32_t interruptCheck)
{
    int i2cnum = MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c);

    if (i2cnum < 0) {
        return E_BAD_PARAM;
    }

    if (i2c->ctrl & MXC_F_I2C_REVA_CTRL_MST_MODE) {
        return E_BAD_STATE;
    }

    if (AsyncRequests[i2cnum] != NULL) {
        return E_BUSY;
    }

    MXC_I2C_ClearFlags((mxc_i2c_regs_t *)i2c, MXC_I2C_REVA_INTFL0_MASK,
                       MXC_I2C_REVA_INTFL1_MASK); // Clear all I2C Interrupts
    MXC_I2C_ClearTXFIFO((mxc_i2c_regs_t *)i2c);
    MXC_I2C_ClearRXFIFO((mxc_i2c_regs_t *)i2c);
    MXC_I2C_SetTXThreshold((mxc_i2c_regs_t *)i2c, 1); // set TX threshold to 2 bytes
    MXC_I2C_SetRXThreshold((mxc_i2c_regs_t *)i2c, 1); // set RX threshold to 6 bytes
    AsyncRequests[i2cnum] = (void *)callback;

    i2c->inten0 = interruptCheck;

    return E_NO_ERROR;
}

int MXC_I2C_RevA_SetRXThreshold(mxc_i2c_reva_regs_t *i2c, unsigned int numBytes)
{
    unsigned int rxFIFOlen = (i2c->fifolen & MXC_F_I2C_REVA_FIFOLEN_RX_DEPTH) >>
                             MXC_F_I2C_REVA_FIFOLEN_RX_DEPTH_POS;

    if (numBytes > rxFIFOlen) {
        return E_BAD_PARAM;
    }

    i2c->rxctrl0 = (i2c->rxctrl0 & ~MXC_F_I2C_REVA_RXCTRL0_THD_LVL) |
                   (numBytes << MXC_F_I2C_REVA_RXCTRL0_THD_LVL_POS);
    return E_NO_ERROR;
}

unsigned int MXC_I2C_RevA_GetRXThreshold(mxc_i2c_reva_regs_t *i2c)
{
    return (i2c->rxctrl0 & MXC_F_I2C_REVA_RXCTRL0_THD_LVL) >> MXC_F_I2C_REVA_RXCTRL0_THD_LVL_POS;
}

int MXC_I2C_RevA_SetTXThreshold(mxc_i2c_reva_regs_t *i2c, unsigned int numBytes)
{
    unsigned int txFIFOlen = (i2c->fifolen & MXC_F_I2C_REVA_FIFOLEN_TX_DEPTH) >>
                             MXC_F_I2C_REVA_FIFOLEN_TX_DEPTH_POS;

    if (numBytes > txFIFOlen) {
        return E_BAD_PARAM;
    }

    i2c->txctrl0 = (i2c->txctrl0 & ~MXC_F_I2C_REVA_TXCTRL0_THD_LVL) |
                   (numBytes << MXC_F_I2C_REVA_TXCTRL0_THD_LVL_POS);
    return E_NO_ERROR;
}

unsigned int MXC_I2C_RevA_GetTXThreshold(mxc_i2c_reva_regs_t *i2c)
{
    return (i2c->txctrl0 & MXC_F_I2C_REVA_TXCTRL0_THD_LVL) >> MXC_F_I2C_REVA_TXCTRL0_THD_LVL_POS;
}

void MXC_I2C_RevA_AsyncCallback(mxc_i2c_reva_regs_t *i2c, int retVal)
{
    // Don't need to check for return value as this function is not accessible to user
    // i2c is already cheked for NULL from where this function is being called
    mxc_i2c_reva_req_t *req =
        (mxc_i2c_reva_req_t *)AsyncRequests[MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c)];

    if (req->callback != NULL) {
        req->callback(req, retVal);
    }
}

void MXC_I2C_RevA_AsyncStop(mxc_i2c_reva_regs_t *i2c)
{
    /* Disable and clear interrupts */
    i2c->inten0 = 0;
    i2c->inten1 = 0;

    i2c->intfl0 = i2c->intfl0;
    i2c->intfl1 = i2c->intfl1;

    // Don't need to check for return value as this function is not accessible to user
    // i2c is already cheked for NULL from where this function is being called
    AsyncRequests[MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c)] = NULL;
}

void MXC_I2C_RevA_AbortAsync(mxc_i2c_reva_regs_t *i2c)
{
    // Don't need to check for return value as this function is not accessible to user
    // i2c is already cheked for NULL from where this function is being called
    MXC_I2C_RevA_AsyncCallback(i2c, E_ABORT);
    MXC_I2C_RevA_AsyncStop(i2c);
}

void MXC_I2C_RevA_MasterAsyncHandler(int i2cNum)
{
    unsigned int written = AsyncWritten[i2cNum];
    unsigned int read = AsyncRead[i2cNum];
    mxc_i2c_reva_regs_t *i2c = (mxc_i2c_reva_regs_t *)MXC_I2C_GET_BASE(i2cNum);
    mxc_i2c_reva_req_t *req = (mxc_i2c_reva_req_t *)AsyncRequests[i2cNum];

    /* Check for errors */
    if (i2c->intfl0 & MXC_I2C_REVA_ERROR) {
        /* Clear and disable interrupts */
        i2c->intfl0 = i2c->intfl0;
        i2c->intfl1 = i2c->intfl1;
        i2c->inten0 = 0;
        i2c->inten1 = 0;

        MXC_I2C_Stop((mxc_i2c_regs_t *)i2c);
        MXC_I2C_RevA_AsyncCallback(i2c, E_COMM_ERR);
        MXC_I2C_RevA_AsyncStop(i2c);

        return;
    }

    /* Write data to the TX FIFO */
    if (req->tx_len > written) {
        if (i2c->intfl0 & MXC_F_I2C_REVA_INTFL0_TX_THD) {
            written += MXC_I2C_WriteTXFIFO((mxc_i2c_regs_t *)i2c, &req->tx_buf[written],
                                           req->tx_len - written);
            i2c->intfl0 = MXC_F_I2C_REVA_INTFL0_TX_THD;
        }

        /* Enable the TX Threshold interrupt if we still need to write to the TX FIFO */
        if (written < req->tx_len) {
            i2c->inten0 |= MXC_F_I2C_REVA_INTEN0_TX_THD;
        } else {
            i2c->inten0 &= ~(MXC_F_I2C_REVA_INTEN0_TX_THD);
        }

        /* Send a restart if we're reading after writing */
        if ((req->tx_len == written) && (req->rx_len)) {
            i2c->mstctrl |= MXC_F_I2C_REVA_MSTCTRL_RESTART;
            i2c->intfl0 = MXC_F_I2C_REVA_INTFL0_DONE;
            i2c->inten0 |= (MXC_F_I2C_REVA_INTEN0_DONE);
        }
    }

    /* Read data in the RX FIFO */
    if (req->rx_len > read) {
        if (i2c->intfl0 & (MXC_F_I2C_REVA_INTFL0_RX_THD | MXC_F_I2C_REVA_INTFL0_DONE)) {
            read +=
                MXC_I2C_ReadRXFIFO((mxc_i2c_regs_t *)i2c, &req->rx_buf[read], req->rx_len - read);
            i2c->intfl0 = MXC_F_I2C_REVA_INTFL0_RX_THD;
        }
    }

    /* Done writing, still reading */
    if ((req->tx_len == written) && (req->rx_len - read) &&
        (i2c->intfl0 & MXC_F_I2C_REVA_INTFL0_DONE)) {
        i2c->intfl0 = MXC_F_I2C_REVA_INTFL0_DONE;

        /* First done interrupt after completing writes to the TX FIFO */
        if (read == 0) {
            i2c->fifo = (req->addr << 1) | 0x1; // Load slave address with read bit.
        }

        /* Set the number of bytes to read */
        if ((req->rx_len - read) > MXC_I2C_REVA_MAX_FIFO_TRANSACTION) {
            i2c->rxctrl1 = 0;
        } else {
            i2c->rxctrl1 = (req->rx_len - read); // 0 for 256, otherwise number of bytes to read
        }

        /* Enable RX Threshold interrupt for when the FIFO is full */
        if (read < req->rx_len) {
            i2c->inten0 |= (MXC_F_I2C_REVA_INTEN0_RX_THD | MXC_F_I2C_REVA_INTEN0_DONE);
        } else {
            i2c->inten0 &= ~(MXC_F_I2C_REVA_INTEN0_RX_THD | MXC_F_I2C_REVA_INTEN0_DONE);
        }
    }

    /* Done reading and writing */
    if ((req->tx_len == written) && (req->rx_len == read)) {
        /* Disable and clear interrupts */
        i2c->inten0 = 0;
        i2c->inten1 = 0;
        i2c->intfl0 = i2c->intfl0;
        i2c->intfl1 = i2c->intfl1;

        /* Send a restart or stop at the end of the transaction */
        if (req->restart) {
            i2c->mstctrl |= MXC_F_I2C_REVA_MSTCTRL_RESTART;
        } else {
            i2c->mstctrl |= MXC_F_I2C_REVA_MSTCTRL_STOP;
        }

        /* Call the callback */
        MXC_I2C_RevA_AsyncCallback(i2c, E_NO_ERROR);

        /* Clear the async state */
        MXC_I2C_RevA_AsyncStop(i2c);

    } else {
        AsyncWritten[i2cNum] = written;
        AsyncRead[i2cNum] = read;
    }
}

void MXC_I2C_RevA_SlaveAsyncHandler(mxc_i2c_reva_regs_t *i2c, mxc_i2c_reva_slave_handler_t callback,
                                    uint32_t *int_en, int *retVal)
{
    uint32_t tFlags = i2c->intfl0;
    *retVal = E_NO_ERROR;

    // Check for errors
    if (tFlags & MXC_I2C_REVA_ERROR) {
        // Error occurred, notify callback function and end transaction
        *retVal = E_COMM_ERR;

        if (callback != NULL) {
            callback(i2c, MXC_I2C_REVA_EVT_TRANS_COMP, retVal);
        }

        MXC_I2C_RevA_ClearFlags(i2c, MXC_I2C_REVA_INTFL0_MASK,
                                MXC_I2C_REVA_INTFL1_MASK); // clear all i2c interrupts
        MXC_I2C_RevA_ClearTXFIFO(i2c);
        MXC_I2C_RevA_ClearRXFIFO(i2c);
        int_en[0] = 0;
        int_en[1] = 0;
        AsyncRequests[MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c)] = NULL;
    }

    // Check whether data is available if we received an interrupt occurred while receiving
    if (int_en[0] & MXC_F_I2C_REVA_INTFL0_RX_THD || int_en[1] & MXC_F_I2C_REVA_INTFL1_RX_OV) {
        if (tFlags & MXC_F_I2C_REVA_INTFL0_RX_THD) {
            if (callback != NULL) {
                callback(i2c, MXC_I2C_REVA_EVT_RX_THRESH, NULL);
            }

            i2c->intfl0 = MXC_F_I2C_REVA_INTFL0_RX_THD;
        }

        if (i2c->intfl1 & MXC_F_I2C_REVA_INTFL1_RX_OV) {
            if (callback != NULL) {
                callback(i2c, MXC_I2C_REVA_EVT_OVERFLOW, NULL);
            }

            i2c->intfl1 = MXC_F_I2C_REVA_INTFL1_RX_OV;
        }
    }

    // Check whether TX FIFO needs to be refilled if interrupt ocurred while transmitting
    if (int_en[0] & (MXC_F_I2C_REVA_INTFL0_TX_THD | MXC_F_I2C_REVA_INTFL0_TX_LOCKOUT) ||
        int_en[1] & MXC_F_I2C_REVA_INTFL1_TX_UN) {
        if (tFlags & MXC_F_I2C_REVA_INTFL0_TX_THD) {
            if (callback != NULL) {
                callback(i2c, MXC_I2C_REVA_EVT_TX_THRESH, NULL);
            }

            i2c->intfl0 = MXC_F_I2C_REVA_INTFL0_TX_THD;
        }

        if (i2c->intfl1 & MXC_F_I2C_REVA_INTFL1_TX_UN) {
            if (callback != NULL) {
                callback(i2c, MXC_I2C_REVA_EVT_UNDERFLOW, NULL);
            }

            i2c->intfl1 = MXC_F_I2C_REVA_INTFL1_TX_UN;
        }

        if (tFlags & MXC_F_I2C_REVA_INTFL0_TX_LOCKOUT) {
            *retVal = E_NO_ERROR;

            if (callback != NULL) {
                callback(i2c, MXC_I2C_REVA_EVT_TRANS_COMP, retVal);
            }

            i2c->intfl0 = MXC_F_I2C_REVA_INTFL0_TX_LOCKOUT;
            int_en[0] = 0;
            int_en[1] = 0;
            AsyncRequests[MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c)] = NULL;
        }
    }

    // Check if transaction completed or restart occurred
    if (int_en[0] & MXC_F_I2C_REVA_INTFL0_DONE) {
        if (tFlags & MXC_F_I2C_REVA_INTFL0_STOP) {
            // Stop/NACK condition occurred, transaction complete
            *retVal = E_NO_ERROR;

            if (callback != NULL) {
                callback(i2c, MXC_I2C_REVA_EVT_TRANS_COMP, retVal);
            }

            i2c->intfl0 = MXC_F_I2C_REVA_INTFL0_STOP;
            int_en[0] = 0;
            int_en[1] = 0;
            AsyncRequests[MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c)] = NULL;
        } else if (tFlags & MXC_F_I2C_REVA_INTFL0_DONE) {
            // Restart detected, re-arm address match interrupt
            i2c->intfl0 = MXC_F_I2C_REVA_INTFL0_DONE;
            int_en[0] = MXC_F_I2C_REVA_INTFL0_ADDR_MATCH;
        }
    }

    // Check for address match interrupt
    if (int_en[0] & MXC_F_I2C_REVA_INTFL0_ADDR_MATCH) {
        if (tFlags & MXC_F_I2C_REVA_INTFL0_ADDR_MATCH) {
            // Address match occurred, prepare for transaction
            if (tFlags & MXC_F_I2C_REVA_INTFL0_STOP && !(tFlags & MXC_F_I2C_REVA_INTFL0_DONE)) {
                // Clear stop flag if it was asserted in a previous transaction
                i2c->intfl0 = MXC_F_I2C_REVA_INTFL0_STOP;
            }

            if (i2c->ctrl & MXC_F_I2C_REVA_CTRL_READ) {
                // Read request received from the master
                if (callback != NULL) {
                    callback(i2c, MXC_I2C_REVA_EVT_MASTER_RD, NULL);
                }

                i2c->intfl0 = MXC_F_I2C_REVA_INTFL0_RD_ADDR_MATCH;
                i2c->intfl0 = MXC_F_I2C_REVA_INTFL0_ADDR_MATCH;
                i2c->intfl0 = MXC_F_I2C_REVA_INTFL0_TX_LOCKOUT;

                int_en[0] = MXC_F_I2C_REVA_INTFL0_TX_THD | MXC_F_I2C_REVA_INTFL0_TX_LOCKOUT |
                            MXC_F_I2C_REVA_INTFL0_DONE | MXC_I2C_REVA_ERROR;
                int_en[1] = MXC_F_I2C_REVA_INTFL1_TX_UN;
            } else {
                // Write request received from the master
                if (callback != NULL) {
                    callback(i2c, MXC_I2C_REVA_EVT_MASTER_WR, NULL);
                }

                i2c->intfl0 = MXC_F_I2C_REVA_INTFL0_WR_ADDR_MATCH;
                i2c->intfl0 = MXC_F_I2C_REVA_INTFL0_ADDR_MATCH;
                int_en[0] = MXC_F_I2C_REVA_INTFL0_RX_THD | MXC_F_I2C_REVA_INTFL0_DONE |
                            MXC_I2C_REVA_ERROR;
                int_en[1] = MXC_F_I2C_REVA_INTFL1_RX_OV;
            }
        }
    }
}

void MXC_I2C_RevA_AsyncHandler(mxc_i2c_reva_regs_t *i2c, uint32_t interruptCheck)
{
    int i2cNum = MXC_I2C_GET_IDX((mxc_i2c_regs_t *)i2c);
    int slaveRetVal;
    uint32_t int_en[2];

    if (i2cNum < 0) {
        return;
    }

    if (i2c->ctrl & MXC_F_I2C_REVA_CTRL_MST_MODE) {
        MXC_I2C_RevA_MasterAsyncHandler(i2cNum);
    } else {
        mxc_i2c_reva_slave_handler_t callback = (mxc_i2c_reva_slave_handler_t)AsyncRequests[i2cNum];

        int_en[0] = i2c->inten0;
        int_en[1] = i2c->inten1;

        MXC_I2C_RevA_SlaveAsyncHandler(i2c, callback, int_en, &slaveRetVal);

        i2c->inten0 = int_en[0];
        i2c->inten1 = int_en[1];
    }
}
