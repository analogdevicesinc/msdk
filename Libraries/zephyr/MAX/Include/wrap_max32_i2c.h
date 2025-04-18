/******************************************************************************
 *
 * Copyright (C) 2023-2025 Analog Devices, Inc.
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

#ifndef LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_I2C_H_
#define LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_I2C_H_

/***** Includes *****/
#include <i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  MAX32665, MAX32666 related mapping
 */
#if defined(CONFIG_SOC_MAX32665) || defined(CONFIG_SOC_MAX32666) || defined(CONFIG_SOC_MAX32650)

/*
 *  Control register bits
 */
#define ADI_MAX32_I2C_CTRL_MASTER MXC_F_I2C_CTRL_MST

/*
 *  Interrupt enable bits
 */
#if defined(CONFIG_SOC_MAX32650)
#define ADI_MAX32_I2C_INT_EN0_RX_THD MXC_F_I2C_INT_EN0_RXTHIE
#define ADI_MAX32_I2C_INT_EN0_TX_LOCK_OUT MXC_F_I2C_INT_EN0_TXLOIE
#define ADI_MAX32_I2C_INT_EN0_TX_THD MXC_F_I2C_INT_EN0_TXTHIE
#define ADI_MAX32_I2C_INT_EN0_DONE MXC_F_I2C_INT_EN0_DONEIE
#define ADI_MAX32_I2C_INT_EN0_STOP MXC_F_I2C_INT_EN0_STOPIE
#define ADI_MAX32_I2C_INT_EN0_ADDR_MATCH MXC_F_I2C_INT_EN0_AMIE
#define ADI_MAX32_I2C_INT_EN0_ADDR_ACK MXC_F_I2C_INT_EN0_ADRACKIE
#define ADI_MAX32_I2C_INT_EN1_RX_OVERFLOW MXC_F_I2C_INT_EN1_RXOFIE
#define ADI_MAX32_I2C_INT_EN1_TX_UNDERFLOW MXC_F_I2C_INT_EN1_TXUFIE
#define ADI_MAX32_I2C_INT_EN0_ERR                                                          \
    (MXC_F_I2C_INT_EN0_ARBERIE | MXC_F_I2C_INT_EN0_TOERIE | MXC_F_I2C_INT_EN0_ADRERIE |    \
     MXC_F_I2C_INT_EN0_DATAERIE | MXC_F_I2C_INT_EN0_DNRERIE | MXC_F_I2C_INT_EN0_STRTERIE | \
     MXC_F_I2C_INT_EN0_STOPERIE)
#else
#define ADI_MAX32_I2C_INT_EN0_RX_THD MXC_F_I2C_INT_EN0_RX_THRESH
#define ADI_MAX32_I2C_INT_EN0_TX_LOCK_OUT MXC_F_I2C_INT_EN0_TX_LOCK_OUT
#define ADI_MAX32_I2C_INT_EN0_TX_THD MXC_F_I2C_INT_EN0_TX_THRESH
#define ADI_MAX32_I2C_INT_EN0_DONE MXC_F_I2C_INT_EN0_DONE
#define ADI_MAX32_I2C_INT_EN0_STOP MXC_F_I2C_INT_EN0_STOP
#define ADI_MAX32_I2C_INT_EN0_ADDR_MATCH MXC_F_I2C_INT_EN0_ADDR_MATCH
#define ADI_MAX32_I2C_INT_EN0_ADDR_ACK MXC_F_I2C_INT_EN0_ADDR_ACK
#define ADI_MAX32_I2C_INT_EN1_RX_OVERFLOW MXC_F_I2C_INT_EN1_RX_OVERFLOW
#define ADI_MAX32_I2C_INT_EN1_TX_UNDERFLOW MXC_F_I2C_INT_EN1_TX_UNDERFLOW
#define ADI_MAX32_I2C_INT_EN0_ERR                                                                \
    (MXC_F_I2C_INT_EN0_ARB_ER | MXC_F_I2C_INT_EN0_TO_ER | MXC_F_I2C_INT_EN0_ADDR_NACK_ER |       \
     MXC_F_I2C_INT_EN0_DATA_ER | MXC_F_I2C_INT_EN0_DO_NOT_RESP_ER | MXC_F_I2C_INT_EN0_START_ER | \
     MXC_F_I2C_INT_EN0_STOP_ER)
#endif

/*
 *  Interrupt flags
 */
#if defined(CONFIG_SOC_MAX32650)
#define ADI_MAX32_I2C_INT_FL0_RX_THD MXC_F_I2C_INT_FL0_RXTHI
#define ADI_MAX32_I2C_INT_FL0_TX_LOCK_OUT MXC_F_I2C_INT_FL0_TXLOI
#define ADI_MAX32_I2C_INT_FL0_TX_THD MXC_F_I2C_INT_FL0_TXTHI
#define ADI_MAX32_I2C_INT_FL0_ADDR_MATCH MXC_F_I2C_INT_FL0_AMI
#define ADI_MAX32_I2C_INT_FL0_ADDR_ACK MXC_F_I2C_INT_FL0_ADRACKI
#define ADI_MAX32_I2C_INT_FL0_STOP MXC_F_I2C_INT_FL0_STOPI
#define ADI_MAX32_I2C_INT_FL0_DONE MXC_F_I2C_INT_FL0_DONEI
#define ADI_MAX32_I2C_INT_FL1_RX_OVERFLOW MXC_F_I2C_INT_FL1_RXOFI
#define ADI_MAX32_I2C_INT_FL0_ERR                                                       \
    (MXC_F_I2C_INT_FL0_ARBERI | MXC_F_I2C_INT_FL0_TOERI | MXC_F_I2C_INT_FL0_ADRERI |    \
     MXC_F_I2C_INT_FL0_DATAERI | MXC_F_I2C_INT_FL0_DNRERI | MXC_F_I2C_INT_FL0_STRTERI | \
     MXC_F_I2C_INT_FL0_STOPERI)
#else
#define ADI_MAX32_I2C_INT_FL0_RX_THD MXC_F_I2C_INT_FL0_RX_THRESH
#define ADI_MAX32_I2C_INT_FL0_TX_LOCK_OUT MXC_F_I2C_INT_FL0_TX_LOCK_OUT
#define ADI_MAX32_I2C_INT_FL0_TX_THD MXC_F_I2C_INT_FL0_TX_THRESH
#define ADI_MAX32_I2C_INT_FL0_ADDR_MATCH MXC_F_I2C_INT_FL0_ADDR_MATCH
#define ADI_MAX32_I2C_INT_FL0_ADDR_ACK MXC_F_I2C_INT_FL0_ADDR_ACK
#define ADI_MAX32_I2C_INT_FL0_STOP MXC_F_I2C_INT_FL0_STOP
#define ADI_MAX32_I2C_INT_FL0_DONE MXC_F_I2C_INT_FL0_DONE
#define ADI_MAX32_I2C_INT_FL1_RX_OVERFLOW MXC_F_I2C_INT_FL1_RX_OVERFLOW
#define ADI_MAX32_I2C_INT_FL0_ERR                                                                \
    (MXC_F_I2C_INT_FL0_ARB_ER | MXC_F_I2C_INT_FL0_TO_ER | MXC_F_I2C_INT_FL0_ADDR_NACK_ER |       \
     MXC_F_I2C_INT_FL0_DATA_ER | MXC_F_I2C_INT_FL0_DO_NOT_RESP_ER | MXC_F_I2C_INT_FL0_START_ER | \
     MXC_F_I2C_INT_FL0_STOP_ER)
#endif

/*
 *  DMA enable bits
 */
#define ADI_MAX32_I2C_DMA_RX_EN MXC_F_I2C_DMA_RXEN
#define ADI_MAX32_I2C_DMA_TX_EN MXC_F_I2C_DMA_TXEN

static inline void Wrap_MXC_I2C_GetIntEn(mxc_i2c_regs_t *i2c, unsigned int *int_en0,
                                         unsigned int *int_en1)
{
    *int_en0 = i2c->int_en0;
    *int_en1 = i2c->int_en1;
}

static inline void Wrap_MXC_I2C_SetIntEn(mxc_i2c_regs_t *i2c, unsigned int int_en0,
                                         unsigned int int_en1)
{
    i2c->int_en0 = int_en0;
    i2c->int_en1 = int_en1;
}

static inline uint32_t Wrap_MXC_I2C_GetTxFIFOLevel(mxc_i2c_regs_t *i2c)
{
    return (i2c->tx_ctrl1 & MXC_F_I2C_TX_CTRL1_TXFIFO) >> MXC_F_I2C_TX_CTRL1_TXFIFO_POS;
}

static inline void Wrap_MXC_I2C_SetRxCount(mxc_i2c_regs_t *i2c, unsigned int len)
{
    if (len < 256) {
        i2c->rx_ctrl1 = len;
    } else {
        i2c->rx_ctrl1 = 0;
    }
}

static inline void Wrap_MXC_I2C_WaitForRestart(mxc_i2c_regs_t *i2c)
{
#if defined(CONFIG_SOC_MAX32650)
    while (i2c->mstr_mode & MXC_F_I2C_MSTR_MODE_RESTART) {}
#else
    while (i2c->master_ctrl & MXC_F_I2C_MASTER_CTRL_RESTART) {}
#endif
}

static inline void Wrap_MXC_I2C_Start(mxc_i2c_regs_t *i2c)
{
#if defined(CONFIG_SOC_MAX32650)
    i2c->mstr_mode |= MXC_F_I2C_MSTR_MODE_START;
#else
    i2c->master_ctrl |= MXC_F_I2C_MASTER_CTRL_START;
#endif
}

static inline void Wrap_MXC_I2C_Restart(mxc_i2c_regs_t *i2c)
{
#if defined(CONFIG_SOC_MAX32650)
    i2c->mstr_mode |= MXC_F_I2C_MSTR_MODE_RESTART;
#else
    i2c->master_ctrl |= MXC_F_I2C_MASTER_CTRL_RESTART;
#endif
}

static inline void Wrap_MXC_I2C_Stop(mxc_i2c_regs_t *i2c)
{
#if defined(CONFIG_SOC_MAX32650)
    i2c->mstr_mode |= MXC_F_I2C_MSTR_MODE_STOP;
#else
    i2c->master_ctrl |= MXC_F_I2C_MASTER_CTRL_STOP;
#endif
}

static inline void Wrap_MXC_I2C_WaitForBusyClear(mxc_i2c_regs_t *i2c)
{
#if defined(CONFIG_SOC_MAX32650)
    while (i2c->stat & MXC_F_I2C_STAT_CKMD) {}
#else
    while (i2c->status & MXC_F_I2C_STATUS_CLK_MODE) {}
#endif
}

static inline void Wrap_MXC_I2C_GetCtrl(mxc_i2c_regs_t *i2c, unsigned int *ctrl)
{
#if defined(CONFIG_SOC_MAX32650)
    *ctrl = i2c->ctrl0;
#else
    *ctrl = i2c->ctrl;
#endif
}

static inline uint32_t Wrap_MXC_I2C_GetReadWriteBitStatus(mxc_i2c_regs_t *i2c)
{
#if defined(CONFIG_SOC_MAX32650)
    return i2c->ctrl0 & MXC_F_I2C_CTRL0_READ;
#else
    return i2c->ctrl & MXC_F_I2C_CTRL_READ;
#endif
}

/*
 *  MAX32690, MAX32655 related mapping
 */
#elif defined(CONFIG_SOC_MAX32690) || defined(CONFIG_SOC_MAX32655) || \
    defined(CONFIG_SOC_MAX32670) || defined(CONFIG_SOC_MAX32672) ||   \
    defined(CONFIG_SOC_MAX32662) || defined(CONFIG_SOC_MAX32675) ||   \
    defined(CONFIG_SOC_MAX32680) || defined(CONFIG_SOC_MAX78002) || defined(CONFIG_SOC_MAX78000)

/*
 *  Control register bits
 */
#define ADI_MAX32_I2C_CTRL_MASTER MXC_F_I2C_CTRL_MST_MODE

/*
 *  Interrupt enable bits
 */
#define ADI_MAX32_I2C_INT_EN0_RX_THD MXC_F_I2C_INTEN0_RX_THD
#define ADI_MAX32_I2C_INT_EN0_TX_LOCK_OUT MXC_F_I2C_INTEN0_TX_LOCKOUT
#define ADI_MAX32_I2C_INT_EN0_TX_THD MXC_F_I2C_INTEN0_TX_THD
#define ADI_MAX32_I2C_INT_EN0_DONE MXC_F_I2C_INTEN0_DONE
#define ADI_MAX32_I2C_INT_EN0_STOP MXC_F_I2C_INTEN0_STOP
#define ADI_MAX32_I2C_INT_EN0_ADDR_MATCH MXC_F_I2C_INTEN0_ADDR_MATCH
#define ADI_MAX32_I2C_INT_EN0_ADDR_ACK MXC_F_I2C_INTEN0_ADDR_ACK
#define ADI_MAX32_I2C_INT_EN1_RX_OVERFLOW MXC_F_I2C_INTEN1_RX_OV
#define ADI_MAX32_I2C_INT_EN1_TX_UNDERFLOW MXC_F_I2C_INTEN1_TX_UN

#define ADI_MAX32_I2C_INT_EN0_ERR                                                          \
    (MXC_F_I2C_INTEN0_ARB_ERR | MXC_F_I2C_INTEN0_TO_ERR | MXC_F_I2C_INTEN0_ADDR_NACK_ERR | \
     MXC_F_I2C_INTEN0_DATA_ERR | MXC_F_I2C_INTEN0_DNR_ERR | MXC_F_I2C_INTEN0_START_ERR |   \
     MXC_F_I2C_INTEN0_STOP_ERR)

/*
 *  Interrupt flags
 */
#define ADI_MAX32_I2C_INT_FL0_RX_THD MXC_F_I2C_INTFL0_RX_THD
#define ADI_MAX32_I2C_INT_FL0_TX_LOCK_OUT MXC_F_I2C_INTFL0_TX_LOCKOUT
#define ADI_MAX32_I2C_INT_FL0_TX_THD MXC_F_I2C_INTFL0_TX_THD
#define ADI_MAX32_I2C_INT_FL0_ADDR_MATCH MXC_F_I2C_INTFL0_ADDR_MATCH
#define ADI_MAX32_I2C_INT_FL0_ADDR_ACK MXC_F_I2C_INTFL0_ADDR_ACK
#define ADI_MAX32_I2C_INT_FL0_STOP MXC_F_I2C_INTFL0_STOP
#define ADI_MAX32_I2C_INT_FL0_DONE MXC_F_I2C_INTFL0_DONE
#define ADI_MAX32_I2C_INT_FL1_RX_OVERFLOW MXC_F_I2C_INTFL1_RX_OV

#define ADI_MAX32_I2C_INT_FL0_ERR                                                          \
    (MXC_F_I2C_INTFL0_ARB_ERR | MXC_F_I2C_INTFL0_TO_ERR | MXC_F_I2C_INTFL0_ADDR_NACK_ERR | \
     MXC_F_I2C_INTFL0_DATA_ERR | MXC_F_I2C_INTFL0_DNR_ERR | MXC_F_I2C_INTFL0_START_ERR |   \
     MXC_F_I2C_INTFL0_STOP_ERR)

/*
 *  DMA enable bits
 */
#define ADI_MAX32_I2C_DMA_RX_EN MXC_F_I2C_DMA_RX_EN
#define ADI_MAX32_I2C_DMA_TX_EN MXC_F_I2C_DMA_TX_EN

static inline void Wrap_MXC_I2C_GetIntEn(mxc_i2c_regs_t *i2c, unsigned int *int_en0,
                                         unsigned int *int_en1)
{
    *int_en0 = i2c->inten0;
    *int_en1 = i2c->inten1;
}

static inline void Wrap_MXC_I2C_SetIntEn(mxc_i2c_regs_t *i2c, unsigned int int_en0,
                                         unsigned int int_en1)
{
    i2c->inten0 = int_en0;
    i2c->inten1 = int_en1;
}

static inline uint32_t Wrap_MXC_I2C_GetTxFIFOLevel(mxc_i2c_regs_t *i2c)
{
    return (i2c->txctrl1 & MXC_F_I2C_TXCTRL1_LVL) >> MXC_F_I2C_TXCTRL1_LVL_POS;
}

static inline void Wrap_MXC_I2C_SetRxCount(mxc_i2c_regs_t *i2c, unsigned int len)
{
    if (len < 256) {
        i2c->rxctrl1 = len;
    } else {
        i2c->rxctrl1 = 0;
    }
}

static inline void Wrap_MXC_I2C_WaitForRestart(mxc_i2c_regs_t *i2c)
{
    while (i2c->mstctrl & MXC_F_I2C_MSTCTRL_RESTART) {}
}

static inline void Wrap_MXC_I2C_Start(mxc_i2c_regs_t *i2c)
{
    i2c->mstctrl |= MXC_F_I2C_MSTCTRL_START;
}

static inline void Wrap_MXC_I2C_Restart(mxc_i2c_regs_t *i2c)
{
    i2c->mstctrl |= MXC_F_I2C_MSTCTRL_RESTART;
}

static inline void Wrap_MXC_I2C_Stop(mxc_i2c_regs_t *i2c)
{
    i2c->mstctrl |= MXC_F_I2C_MSTCTRL_STOP;
}

static inline void Wrap_MXC_I2C_WaitForBusyClear(mxc_i2c_regs_t *i2c)
{
    while (i2c->status & MXC_F_I2C_STATUS_MST_BUSY) {}
}

static inline void Wrap_MXC_I2C_GetCtrl(mxc_i2c_regs_t *i2c, unsigned int *ctrl)
{
    *ctrl = i2c->ctrl;
}

static inline uint32_t Wrap_MXC_I2C_GetReadWriteBitStatus(mxc_i2c_regs_t *i2c)
{
    return i2c->ctrl & MXC_F_I2C_CTRL_READ;
}

#endif

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_I2C_H_
