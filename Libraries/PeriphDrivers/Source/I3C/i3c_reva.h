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

/**
 * @brief Maximum supported IBI bytes.
 *
 */
#define MXC_I3C_REVA_MAX_IBI_BYTES 7U

typedef struct _i3c_reva_ccc_req_t mxc_i3c_reva_ccc_req_t;
typedef struct _i3c_reva_req_t mxc_i3c_reva_req_t;

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
 * @brief   IBI types.
 *
 */
typedef enum {
    MXC_I3C_REVA_IBI_TYPE_NONE = MXC_V_I3C_REVA_CONT_STATUS_IBITYPE_NONE, ///<
    MXC_I3C_REVA_IBI_TYPE_IBI = MXC_V_I3C_REVA_CONT_STATUS_IBITYPE_IBI, ///<
    MXC_I3C_REVA_IBI_TYPE_CONTROLLER_REQ = MXC_V_I3C_REVA_CONT_STATUS_IBITYPE_CONT_REQ, ///<
    MXC_I3C_REVA_IBI_TYPE_HOTJOIN_REQ = MXC_V_I3C_REVA_CONT_STATUS_IBITYPE_HOTJOIN_REQ, ///<
} mxc_i3c_reva_ibi_type_t;

/**
 * @brief Common Command Code request structure.
 *
 * Broadcast and direct commands are distinguished by MSB, i.e. if MSB being set
 * implies a direct CCC while MSB being 0 implies a broadcast CCC.
 *
 */
struct _i3c_reva_ccc_req_t {
    uint8_t ccc; ///< CCC command to send.
    uint8_t target_addr; ///< Target address if CCC is a direct command. Ignored
    ///< if CCC is a broadcast command.
    mxc_i3c_transfer_type_t xfer_type; ///< Transfer type. Ignored for broadcast CCCs.
    uint8_t flags; ///< See request flags above.
    uint8_t def_byte; ///< Optional defining byte. Defined by CCC.
    uint8_t sub_cmd; ///< Optional sub-command. Defined by CCC. Only used by direct
    ///< CCCs.
    unsigned char *tx_buf; ///< Optional data bytes to send.
    uint8_t tx_len; ///< Length of optional data.
    unsigned char *rx_buf; ///< Optional data bytes to read.
    uint8_t rx_len; ///< Length of optional data.
};

/**
 * @brief Private SDR request structure.
 *
 * SDR read and write to an I3C target.
 *
 */
struct _i3c_reva_req_t {
    uint8_t target_addr; ///< Target address.
    bool is_i2c; ///< If this is a legacy I2C transfer.
    bool stop; ///< Send a STOP after the transaction.
    unsigned char *tx_buf; ///< Optional data bytes to send.
    uint16_t tx_len; ///< Length of optional data.
    unsigned char *rx_buf; ///< Optional data bytes to read.
    uint16_t rx_len; ///< Length of optional data.
};

/**
 * @brief   IBI payload request callback. Write additional byte to \a byte.
 *
 * This function will be called as long as non-zero is returned.
 *
 * @return  Non-zero if a byte is written to \a byte, 0 to indicate no more additional
 * bytes left to send.
 */
typedef int (*mxc_i3c_reva_ibi_getbyte_t)(mxc_i3c_reva_regs_t *i3c, unsigned char *byte);

/* **** Function Prototypes **** */

/* ************************************************************************* */
/* Control/Configuration functions                                           */
/* ************************************************************************* */

int MXC_I3C_RevA_Init(mxc_i3c_reva_regs_t *i3c, bool targetMode, uint8_t staticAddr);
int MXC_I3C_RevA_Recover(mxc_i3c_reva_regs_t *i3c);
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

/* ************************************************************************* */
/* Protocol functions                                                       */
/* ************************************************************************* */
int MXC_I3C_RevA_EmitStart(mxc_i3c_reva_regs_t *i3c, bool isI2C, mxc_i3c_transfer_type_t xferType,
                           uint8_t addr, uint8_t readCount);
int MXC_I3C_RevA_ResetTarget(mxc_i3c_reva_regs_t *i3c);
void MXC_I3C_RevA_EmitStop(mxc_i3c_reva_regs_t *i3c);
void MXC_I3C_RevA_EmitI2CStop(mxc_i3c_reva_regs_t *i3c);
int MXC_I3C_RevA_Controller_CCC(mxc_i3c_reva_regs_t *i3c, const mxc_i3c_reva_ccc_req_t *req);
int MXC_I3C_RevA_Controller_Transaction(mxc_i3c_reva_regs_t *i3c, const mxc_i3c_reva_req_t *req);
int MXC_I3C_RevA_Controller_DAA(mxc_i3c_reva_regs_t *i3c, uint8_t addr, uint8_t *pid, uint8_t *bcr,
                                uint8_t *dcr);
int MXC_I3C_RevA_AutoIBI(mxc_i3c_reva_regs_t *i3c);
int MXC_I3C_RevA_HotJoin(mxc_i3c_reva_regs_t *i3c);
int MXC_I3C_RevA_RequestIBI(mxc_i3c_reva_regs_t *i3c, unsigned char mdb,
                            mxc_i3c_reva_ibi_getbyte_t getByteCb);
int MXC_I3C_RevA_Standby(mxc_i3c_reva_regs_t *i3c);
int MXC_I3C_RevA_Wakeup(mxc_i3c_reva_regs_t *i3c);

/* ************************************************************************* */
/* Low-level functions                                                       */
/* ************************************************************************* */
int MXC_I3C_RevA_Controller_GetError(mxc_i3c_reva_regs_t *i3c);
void MXC_I3C_RevA_Controller_ClearError(mxc_i3c_reva_regs_t *i3c);

int MXC_I3C_RevA_SetRXTXThreshold(mxc_i3c_reva_regs_t *i3c, mxc_i3c_reva_rx_threshold_t rxth,
                                  mxc_i3c_reva_tx_threshold_t txth);
int MXC_I3C_RevA_ReadRXFIFO(mxc_i3c_reva_regs_t *i3c, volatile unsigned char *bytes,
                            unsigned int len, int timeout);
int MXC_I3C_RevA_WriteTXFIFO(mxc_i3c_reva_regs_t *i3c, const unsigned char *bytes, unsigned int len,
                             bool end, int timeout);
void MXC_I3C_RevA_IRQHandler(mxc_i3c_reva_regs_t *i3c);

static inline void MXC_I3C_RevA_ClearRXFIFO(mxc_i3c_reva_regs_t *i3c)
{
    i3c->cont_fifoctrl |= MXC_F_I3C_REVA_CONT_FIFOCTRL_RX_FLUSH;
}

static inline void MXC_I3C_RevA_ClearTXFIFO(mxc_i3c_reva_regs_t *i3c)
{
    i3c->cont_fifoctrl |= MXC_F_I3C_REVA_CONT_FIFOCTRL_TX_FLUSH;
}

static inline unsigned int MXC_I3C_RevA_Controller_GetRXCount(mxc_i3c_reva_regs_t *i3c)
{
    return ((i3c->cont_fifoctrl & MXC_F_I3C_REVA_CONT_FIFOCTRL_RX_LVL) >>
            MXC_F_I3C_REVA_CONT_FIFOCTRL_RX_LVL_POS);
}

static inline unsigned int MXC_I3C_RevA_Controller_GetTXCount(mxc_i3c_reva_regs_t *i3c)
{
    return ((i3c->cont_fifoctrl & MXC_F_I3C_REVA_CONT_FIFOCTRL_TX_LVL) >>
            MXC_F_I3C_REVA_CONT_FIFOCTRL_TX_LVL_POS);
}

static inline void MXC_I3C_RevA_Controller_EnableInt(mxc_i3c_reva_regs_t *i3c, uint32_t mask)
{
    i3c->cont_inten |= mask;
}

static inline void MXC_I3C_RevA_Controller_DisableInt(mxc_i3c_reva_regs_t *i3c, uint32_t mask)
{
    i3c->cont_intclr |= mask;
}

static inline unsigned int MXC_I3C_RevA_Controller_GetFlags(mxc_i3c_reva_regs_t *i3c)
{
    return i3c->cont_intfl;
}

static inline void MXC_I3C_RevA_Controller_ClearFlags(mxc_i3c_reva_regs_t *i3c, uint32_t mask)
{
    i3c->cont_status |= mask;
}

static inline void MXC_I3C_RevA_Target_EnableInt(mxc_i3c_reva_regs_t *i3c, uint32_t mask)
{
    i3c->targ_inten |= mask;
}

static inline void MXC_I3C_RevA_Target_DisableInt(mxc_i3c_reva_regs_t *i3c, uint32_t mask)
{
    i3c->targ_intclr |= mask;
}

static inline int MXC_I3C_RevA_Target_GetFlags(mxc_i3c_reva_regs_t *i3c)
{
    return i3c->targ_intfl;
}

static inline void MXC_I3C_RevA_Target_ClearFlags(mxc_i3c_reva_regs_t *i3c, uint32_t mask)
{
    i3c->targ_status |= mask;
}

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_I3C_I3C_REVA_H_
