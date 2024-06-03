/**
* @file     i3c.h
* @brief    Improved Inter Integrated Circuit (I3C) communications interface driver.
*/

/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_I3C_I3C_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_I3C_I3C_REVA_H_

/* **** Includes **** */
#include <stdint.h>
#include "i3c.h"
#include "i3c_reva.h"
#include "i3c_reva_regs.h"
#include "mxc_sys.h"

/****** Definitions *****/
typedef struct _i3c_reva_target_t mxc_i3c_reva_target_t;
typedef struct _i2c_reva_target_t mxc_i3c_reva_i2c_target_t;

/**
 * @brief Maximum supported IBI bytes.
 *
 */
#define MXC_I3C_REVA_MAX_IBI_BYTES 7U

/**
 * @brief   The list of high-keeper options.
 *
 * This setting should match the high-keeper implementation of the device.
 */
typedef enum {
    MXC_I3C_REVA_HIGH_KEEPER_OFF = MXC_S_I3C_REVA_CONT_CTRL0_HKEEP_OFF, ///< No high-keeper support
    MXC_I3C_REVA_HIGH_KEEPER_ON_CHIP =
        MXC_S_I3C_REVA_CONT_CTRL0_HKEEP_ON_CHIP, ///< SCL and SDA pads
    ///< have weak pull-ups
    MXC_I3C_REVA_HIGH_KEEPER_EXT_SDA =
        MXC_S_I3C_REVA_CONT_CTRL0_HKEEP_EXT_SDA, ///< External high-keeper
    ///< support for SDA signal
    MXC_I3C_REVA_HIGH_KEEPER_EXT_SCL_SDA = MXC_S_I3C_REVA_CONT_CTRL0_HKEEP_EXT_SCL_SDA,
    ///< External high-keeper support for SCL and SDA signals
} mxc_i3c_reva_high_keeper_t;

/**
 * @brief   The list of receive FIFO trigger levels.
 */
typedef enum {
    MXC_I3C_REVA_RX_TH_NOT_EMPTY = MXC_V_I3C_REVA_CONT_FIFOCTRL_RX_THD_LVL_NOT_EMPTY, ///<
    MXC_I3C_REVA_RX_TH_QUARTER_FULL = MXC_V_I3C_REVA_CONT_FIFOCTRL_RX_THD_LVL_QUARTER_FULL, ///<
    MXC_I3C_REVA_RX_TH_HALF_FULL = MXC_V_I3C_REVA_CONT_FIFOCTRL_RX_THD_LVL_HALF_FULL, ///<
    MXC_I3C_REVA_RX_TH_3_QUARTER_FULL = MXC_V_I3C_REVA_CONT_FIFOCTRL_RX_THD_LVL_3_QUARTER_FULL, ///<
} mxc_i3c_reva_rx_threshold_t;

/**
 * @brief   The list of transmit FIFO trigger levels.
 */
typedef enum {
    MXC_I3C_REVA_TX_TH_EMPTY = MXC_V_I3C_REVA_CONT_FIFOCTRL_TX_THD_LVL_EMPTY, ///<
    MXC_I3C_REVA_TX_TH_QUARTER_FULL = MXC_V_I3C_REVA_CONT_FIFOCTRL_TX_THD_LVL_QUARTER_FULL, ///<
    MXC_I3C_REVA_TX_TH_HALF_FULL = MXC_V_I3C_REVA_CONT_FIFOCTRL_TX_THD_LVL_HALF_FULL, ///<
    MXC_I3C_REVA_TX_TH_ALMOST_FULL = MXC_V_I3C_REVA_CONT_FIFOCTRL_TX_THD_LVL_ALMOST_FULL, ///<
} mxc_i3c_reva_tx_threshold_t;

/**
 * @brief IBI types.
 *
 */
typedef enum {
    MXC_I3C_REVA_IBI_TYPE_NONE = MXC_V_I3C_REVA_CONT_STATUS_IBITYPE_NONE, ///<
    MXC_I3C_REVA_IBI_TYPE_IBI = MXC_V_I3C_REVA_CONT_STATUS_IBITYPE_IBI, ///<
    MXC_I3C_REVA_IBI_TYPE_CONTROLLER_REQ = MXC_V_I3C_REVA_CONT_STATUS_IBITYPE_CONT_REQ, ///<
    MXC_I3C_REVA_IBI_TYPE_HOTJOIN_REQ = MXC_V_I3C_REVA_CONT_STATUS_IBITYPE_HOTJOIN_REQ, ///<
} mxc_i3c_reva_ibi_type_t;

/**
 * @brief   IBI callback.
 *
 * When a target wins address arbitration and generates an IBI, this callback
 * function is called to get the application decision to ACK/NACK the IBI.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   dynAddr     The byte received.
 * @param   ibiType     IBI type. See \ref mxc_i3c_ibi_type_t for possible values.
 *
 * @return  0 if the IBI should not be acknowledged (NACK), non-zero to
 *          acknowledge the IBI.
 */
typedef int (*mxc_i3c_reva_ibi_ack_t)(mxc_i3c_reva_regs_t *i3c, unsigned char dynAddr,
                                      mxc_i3c_reva_ibi_type_t ibiType);

/**
 * @brief   IBI request callback. Called after an IBI is acknowledged by the application
 * and mandatory and additional data bytes are read.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   target      Pointer to I3C target requesting an IBI.
 *
 */
typedef void (*mxc_i3c_reva_ibi_req_t)(mxc_i3c_reva_regs_t *i3c, mxc_i3c_reva_target_t *target);

/**
 * @brief   IBI payload request callback. Write additional byte to \a byte.
 *
 * This function will be called as long as non-zero is returned.
 *
 * @return  Non-zero if a byte is written to \a byte, 0 to indicate no more additional
 * bytes left to send.
 */
typedef int (*mxc_i3c_reva_ibi_getbyte_t)(mxc_i3c_reva_regs_t *i3c, unsigned char *byte);

/**
 * @brief   CCC request callback.
 *
 * Called when the received CCC is not handled automatically.
 */
typedef void (*mxc_i3c_reva_ccc_cb_t)(mxc_i3c_reva_regs_t *i3c, unsigned char ccc);

/**
 * @brief   The information required to perform a complete I2C transaction as
 *          the bus master.
 *
 * The information required to perform a complete I2C transaction as the bus
 * master. This structure is used by the MXC_I2C_MasterTransaction() and
 * MXC_I2C_MasterTransactionAsync() functions.
 */
struct _i3c_reva_target_t {
    uint8_t dynAddr; ///< Dynamic address of the I3C target.
    uint8_t staticAddr; ///< Static address of the I3C target. Set to 0 if target does
    ///< not have an I2C-style static address.
    uint64_t pid; ///< Provisioned ID.
    uint8_t bcr; ///< Bus characteristics register.
    uint8_t dcr; ///< Device characteristics register.
    uint8_t data[1 + MXC_I3C_MAX_IBI_BYTES]; ///< Mandatory byte plus additional bytes.
    uint8_t numBytes; ///< Number of data bytes.
};

struct _i2c_reva_target_t {
    uint8_t staticAddr; ///< Target address of the I2C target.
};

/* **** Function Prototypes **** */

/* ************************************************************************* */
/* Control/Configuration functions                                           */
/* ************************************************************************* */

int MXC_I3C_RevA_Init(mxc_i3c_reva_regs_t *i3c, int targetMode, uint8_t staticAddr);
int MXC_I3C_RevA_Shutdown(mxc_i3c_reva_regs_t *i3c);
int MXC_I3C_RevA_SetPPFrequency(mxc_i3c_reva_regs_t *i3c, unsigned int frequency);
unsigned int MXC_I3C_RevA_GetPPFrequency(mxc_i3c_reva_regs_t *i3c);
int MXC_I3C_RevA_SetODFrequency(mxc_i3c_reva_regs_t *i3c, unsigned int frequency, bool highPP);
unsigned int MXC_I3C_RevA_GetODFrequency(mxc_i3c_reva_regs_t *i3c);
int MXC_I3C_RevA_SetI2CFrequency(mxc_i3c_reva_regs_t *i3c, unsigned int frequency);
unsigned int MXC_I3C_RevA_GetI2CFrequency(mxc_i3c_reva_regs_t *i3c);
uint8_t MXC_I3C_RevA_GetDynamicAddress(mxc_i3c_reva_regs_t *i3c);
int MXC_I3C_RevA_SetSkew(mxc_i3c_reva_regs_t *i3c, uint8_t skew);
int MXC_I3C_RevA_SetHighKeeperMode(mxc_i3c_reva_regs_t *i3c, mxc_i3c_reva_high_keeper_t hkeep);
int MXC_I3C_RevA_SetI3CTargets(mxc_i3c_reva_regs_t *i3c, mxc_i3c_reva_target_t *targets,
                               uint8_t numTargets);
int MXC_I3C_RevA_SetI2CTargets(mxc_i3c_reva_regs_t *i3c, mxc_i3c_reva_i2c_target_t *targets,
                               uint8_t numTargets);

/* ************************************************************************* */
/* Protocol functions                                                       */
/* ************************************************************************* */
int MXC_I3C_RevA_Start(mxc_i3c_reva_regs_t *i3c, bool i2c, uint8_t readWrite, uint8_t addr,
                       uint8_t readCount);
int MXC_I3C_RevA_BroadcastCCC(mxc_i3c_reva_regs_t *i3c, unsigned char ccc, int defByte,
                              unsigned char *data, int len);
int MXC_I3C_RevA_PerformDAA(mxc_i3c_reva_regs_t *i3c);
int MXC_I3C_RevA_HotJoin(mxc_i3c_reva_regs_t *i3c);
int MXC_I3C_RevA_RequestIBI(mxc_i3c_reva_regs_t *i3c, unsigned char mdb,
                            mxc_i3c_reva_ibi_getbyte_t getByteCb);
int MXC_I3C_RevA_Standby(mxc_i3c_reva_regs_t *i3c);
int MXC_I3C_RevA_Wakeup(mxc_i3c_reva_regs_t *i3c);
void MXC_I3C_RevA_SetIBICallback(mxc_i3c_reva_regs_t *i3c, mxc_i3c_reva_ibi_ack_t ackCb,
                                 mxc_i3c_reva_ibi_req_t reqCb);
void MXC_I3C_RevA_SetIBIPayloadCallback(mxc_i3c_reva_regs_t *i3c,
                                        mxc_i3c_reva_ibi_getbyte_t payloadCb);
void MXC_I3C_RevA_SetCCCCallback(mxc_i3c_reva_regs_t *i3c, mxc_i3c_reva_ccc_cb_t cccCb);
int MXC_I3C_RevA_ReadI2CBlocking(mxc_i3c_reva_regs_t *i3c, unsigned char staticAddr,
                                 unsigned char *bytes, unsigned int *len);
int MXC_I3C_RevA_WriteI2CBlocking(mxc_i3c_reva_regs_t *i3c, unsigned char staticAddr,
                                  unsigned char *bytes, unsigned int *len);
int MXC_I3C_RevA_ReadSDRBlocking(mxc_i3c_reva_regs_t *i3c, unsigned char dynAddr,
                                 unsigned char *bytes, unsigned int *len);
int MXC_I3C_RevA_WriteSDRBlocking(mxc_i3c_reva_regs_t *i3c, unsigned char dynAddr,
                                  unsigned char *bytes, unsigned int *len);

/* ************************************************************************* */
/* Low-level functions                                                       */
/* ************************************************************************* */
int MXC_I3C_RevA_GetError(mxc_i3c_reva_regs_t *i3c);
void MXC_I3C_RevA_ClearError(mxc_i3c_reva_regs_t *i3c);

static inline void MXC_I3C_RevA_Stop(mxc_i3c_reva_regs_t *i3c)
{
    /* Configure MCTRL register for STOP */
    i3c->cont_ctrl1 &= ~(MXC_F_I3C_REVA_CONT_CTRL1_REQ | MXC_F_I3C_REVA_CONT_CTRL1_RDWR_DIR |
                         MXC_F_I3C_REVA_CONT_CTRL1_TERM_RD);
    i3c->cont_ctrl1 |= MXC_S_I3C_REVA_CONT_CTRL1_REQ_EMIT_STOP;
    /* Wait for MCTRL_DONE */
    while (!(i3c->cont_status & MXC_F_I3C_REVA_CONT_STATUS_REQ_DONE)) {}
}
static inline void MXC_I3C_RevA_I2CStop(mxc_i3c_reva_regs_t *i3c)
{
    /* Configure MCTRL register for STOP */
    i3c->cont_ctrl1 = MXC_S_I3C_REVA_CONT_CTRL1_REQ_EMIT_STOP |
                      (1 << MXC_F_I3C_REVA_CONT_CTRL1_TYPE_POS);
    /* Wait for MCTRL_DONE */
    while (!(i3c->cont_status & MXC_F_I3C_REVA_CONT_STATUS_REQ_DONE)) {}
}
int MXC_I3C_RevA_SetRXTXThreshold(mxc_i3c_reva_regs_t *i3c, mxc_i3c_reva_rx_threshold_t rxth,
                                  mxc_i3c_reva_tx_threshold_t txth);
int MXC_I3C_RevA_ReadRXFIFO(mxc_i3c_reva_regs_t *i3c, volatile unsigned char *bytes,
                            unsigned int len);
int MXC_I3C_RevA_WriteTXFIFO(mxc_i3c_reva_regs_t *i3c, volatile unsigned char *bytes,
                             unsigned int len);
void MXC_I3C_RevA_IRQHandler(mxc_i3c_reva_regs_t *i3c);

static inline void MXC_I3C_RevA_ClearRXFIFO(mxc_i3c_reva_regs_t *i3c)
{
    i3c->cont_fifoctrl |= MXC_F_I3C_REVA_CONT_FIFOCTRL_RX_FLUSH;
}

static inline void MXC_I3C_RevA_ClearTXFIFO(mxc_i3c_reva_regs_t *i3c)
{
    i3c->cont_fifoctrl |= MXC_F_I3C_REVA_CONT_FIFOCTRL_TX_FLUSH;
}

static inline unsigned int MXC_I3C_RevA_ControllerGetRXCount(mxc_i3c_reva_regs_t *i3c)
{
    return ((i3c->cont_fifoctrl & MXC_F_I3C_REVA_CONT_FIFOCTRL_RX_LVL) >>
            MXC_F_I3C_REVA_CONT_FIFOCTRL_RX_LVL_POS);
}

static inline unsigned int MXC_I3C_RevA_ControllerGetTXCount(mxc_i3c_reva_regs_t *i3c)
{
    return ((i3c->cont_fifoctrl & MXC_F_I3C_REVA_CONT_FIFOCTRL_TX_LVL) >>
            MXC_F_I3C_REVA_CONT_FIFOCTRL_TX_LVL_POS);
}

static inline void MXC_I3C_RevA_ControllerEnableInt(mxc_i3c_reva_regs_t *i3c, uint32_t mask)
{
    i3c->cont_inten |= mask;
}

static inline void MXC_I3C_RevA_ControllerDisableInt(mxc_i3c_reva_regs_t *i3c, uint32_t mask)
{
    i3c->cont_intclr |= mask;
}

static inline unsigned int MXC_I3C_RevA_ControllerGetFlags(mxc_i3c_reva_regs_t *i3c)
{
    return i3c->cont_intfl;
}

static inline void MXC_I3C_RevA_ControllerClearFlags(mxc_i3c_reva_regs_t *i3c, uint32_t mask)
{
    i3c->cont_status |= mask;
}

static inline void MXC_I3C_RevA_TargetEnableInt(mxc_i3c_reva_regs_t *i3c, uint32_t mask)
{
    i3c->targ_inten |= mask;
}

static inline void MXC_I3C_RevA_TargetDisableInt(mxc_i3c_reva_regs_t *i3c, uint32_t mask)
{
    i3c->targ_intclr |= mask;
}

static inline int MXC_I3C_RevA_TargetGetFlags(mxc_i3c_reva_regs_t *i3c)
{
    return i3c->targ_intfl;
}

static inline void MXC_I3C_RevA_TargetClearFlags(mxc_i3c_reva_regs_t *i3c, uint32_t mask)
{
    i3c->targ_status |= mask;
}

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_I3C_I3C_REVA_H_
