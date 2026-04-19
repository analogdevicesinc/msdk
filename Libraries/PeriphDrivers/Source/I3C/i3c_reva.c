/******************************************************************************
 *
 * Copyright (C) 2024-2025 Analog Devices, Inc.
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

/* **** Includes **** */
#include <string.h>
#include "mxc_errors.h"
#include "mxc_sys.h"
#include "i3c_regs.h"
#include "i3c.h"
#include "i3c_reva.h"
#include "i3c_ccc.h"

/* **** Definitions **** */
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x)[0])
#endif

#define MXC_I3C_REVA_MAX_FIFO_TRANSACTION 255U

#define GET_FIELD(reg, field) ((i3c->reg & MXC_F_I3C_REVA_##field) >> MXC_F_I3C_REVA_##field##_POS)
#define SET_FIELD(reg, field, value)                  \
    do {                                              \
        i3c->reg &= ~MXC_F_I3C_REVA_##field;          \
        i3c->reg |= value << MXC_F_I3C_##field##_POS; \
    } while (0);

#define MXC_F_I3C_REVA_CONT_ERRWARN_MASK                                         \
    (MXC_F_I3C_REVA_CONT_ERRWARN_NACK | MXC_F_I3C_REVA_CONT_ERRWARN_TX_ABT |     \
     MXC_F_I3C_REVA_CONT_ERRWARN_RX_TERM | MXC_F_I3C_REVA_CONT_ERRWARN_HDR_PAR | \
     MXC_F_I3C_REVA_CONT_ERRWARN_HDR_CRC | MXC_F_I3C_REVA_CONT_ERRWARN_TX_OVR |  \
     MXC_F_I3C_REVA_CONT_ERRWARN_MSG | MXC_F_I3C_REVA_CONT_ERRWARN_INV_REQ |     \
     MXC_F_I3C_REVA_CONT_ERRWARN_TO)

#define MXC_F_I3C_REVA_CONT_STATUS_MASK                                           \
    (MXC_F_I3C_REVA_CONT_STATUS_IBI_WON | MXC_F_I3C_REVA_CONT_STATUS_CONT_TRANS | \
     MXC_F_I3C_REVA_CONT_STATUS_DONE | MXC_F_I3C_REVA_CONT_STATUS_NACK |          \
     MXC_F_I3C_REVA_CONT_STATUS_REQ_DONE | MXC_F_I3C_REVA_CONT_STATUS_TARG_START)

/* MCTRL_TYPE values if request is EmitStartAddr */
typedef enum {
    MXC_I3C_REVA_START_TYPE_SDR = 0,
    MXC_I3C_REVA_START_TYPE_I2C = 1
} mxc_i3c_reva_start_type_t;

/* MCTRL_IBIRESP values if request is EmitStartAddr and AutoIBI */
#define MXC_I3C_REVA_CONT_CTRL1_IBIRESP_ACK 0x00U
#define MXC_I3C_REVA_CONT_CTRL1_IBIRESP_NACK 0x01U
#define MXC_I3C_REVA_CONT_CTRL1_IBIRESP_ACK_WITH_MB 0x02U
#define MXC_I3C_REVA_CONT_CTRL1_IBIRESP_MANUAL 0x03U

/* Provisioned ID length */
#define MXC_I3C_PROVISIONED_ID_LEN 6U

/* Broadcast address */
#define MXC_I3C_BROADCAST_ADDR 0x7EU

typedef struct {
    mxc_i3c_reva_regs_t *regs; ///< Pointer to regs of this I3C instance.
    mxc_i3c_reva_ibi_getbyte_t ibiGetByteCB; ///< IBI additional data callback.
} mxc_i3c_reva_controller_t;

typedef enum {
    MXC_I3C_REVA_IBIRESP_ACK = 0,
    MXC_I3C_REVA_IBIRESP_NACK = 1,
    MXC_I3C_REVA_IBIRESP_ACK_WITH_MB = 2,
    MXC_I3C_REVA_IBIRESP_MANUAL = 3
} mxc_i3c_reva_ibiresp_t;

#define MXC_I3C_REVA_ADDR_INVALID 0x00U

/* **** Globals **** */
/**
 * @brief I3C controller instances.
 *
 */
mxc_i3c_reva_controller_t controller[MXC_CFG_I3C_INSTANCES];

/* **** Functions **** */
int MXC_I3C_RevA_Init(mxc_i3c_reva_regs_t *i3c, bool targetMode, uint8_t staticAddr)
{
    int ret = E_NO_ERROR;
    int idx;
    uint8_t val;

    idx = MXC_I3C_GET_IDX((mxc_i3c_regs_t *)i3c);
    if (idx < 0) {
        return E_BAD_PARAM;
    }

    if (!targetMode) {
        /* Controller mode initialization */

        /* SCL-to-SDA skew */
        ret = MXC_I3C_RevA_SetSkew(i3c, 0);
        if (ret < 0) {
            return ret;
        }

        /* High-Keeper implementation selection */
        ret = MXC_I3C_RevA_SetHighKeeperMode(i3c, MXC_I3C_HIGH_KEEPER_OFF);
        if (ret < 0) {
            return ret;
        }

        /* 2. If using fifos, set tx rx fifo trigger levels */
        if (i3c->targ_cap1 & MXC_F_I3C_REVA_TARG_CAP1_RXFIFO_CFG) {
            MXC_I3C_RevA_SetRXTXThreshold(i3c, MXC_I3C_REVA_RX_TH_NOT_EMPTY,
                                          MXC_I3C_REVA_TX_TH_ALMOST_FULL);
        }

        /* 3. Optionally write IBIRULES reg to optimize response to incoming IBIs */

        /* 4. Enable controller mode */
        SET_FIELD(cont_ctrl0, CONT_CTRL0_EN, MXC_V_I3C_REVA_CONT_CTRL0_EN_ON);
    } else {
        /* Target mode initialization */
        /* 1. After reset, write setup registers needed for optional features */

        if (i3c->targ_cap1 & MXC_F_I3C_REVA_TARG_CAP1_RXFIFO_CFG) {
            MXC_I3C_RevA_SetRXTXThreshold(i3c, MXC_I3C_REVA_RX_TH_NOT_EMPTY,
                                          MXC_I3C_REVA_TX_TH_ALMOST_FULL);
        }

        /* Set BCR, DCR */
        val = 0 << 6; /* Device role: target */
        if (i3c->targ_cap1 & MXC_S_I3C_REVA_TARG_CAP1_CCCH_LIMITS) {
            val |= 1 << 0; /* Max data speed limitation */
        }
        if (i3c->targ_cap1 & MXC_S_I3C_REVA_TARG_CAP1_CCCH_BASIC) {
            val |= 1 << 5; /* Supports advanced capabiliries */
        }
        if (i3c->targ_cap1 & MXC_S_I3C_REVA_TARG_CAP1_IBI_EVENTS_IBI) {
            val |= 1 << 1; /* Supports IBI generation */
        }
        if (i3c->targ_cap1 & MXC_S_I3C_REVA_TARG_CAP1_IBI_EVENTS_PAYLOAD) {
            val |= 1 << 2; /* MDB and additional data bytes may follow the IBI */
        }
        val |= 1 << 3; /* Offline capable */
        SET_FIELD(targ_idext, TARG_IDEXT_BUSCHAR, val);

        /* Set DCR to generic device */
        SET_FIELD(targ_idext, TARG_IDEXT_DEVCHAR, 0);

        /* Enable CCC handling */
        MXC_I3C_RevA_Target_EnableInt(i3c, MXC_F_I3C_REVA_TARG_INTEN_CCC);

        /* 2. Write 1 to CONFIG.TGTENA */
        SET_FIELD(targ_ctrl0, TARG_CTRL0_EN, 1);

        /* Clear STOP bit */
        SET_FIELD(targ_status, TARG_STATUS_STOP, 0);
    }

    controller[idx].regs = i3c;

    return E_NO_ERROR;
}

static inline uint8_t MXC_I3C_RevA_Controller_GetState(mxc_i3c_reva_regs_t *i3c)
{
    return (uint8_t)GET_FIELD(cont_status, CONT_STATUS_STATE);
}

static inline int MXC_I3C_RevA_Controller_WaitForDone(mxc_i3c_reva_regs_t *i3c, int timeout)
{
    while (!(i3c->cont_status & MXC_F_I3C_REVA_CONT_STATUS_DONE)) {
        if (timeout == 0) {
            return E_TIME_OUT;
        }
        if (timeout > 0) {
            timeout--;
        }
    }

    return E_SUCCESS;
}

static inline int MXC_I3C_RevA_Controller_WaitForReqDone(mxc_i3c_reva_regs_t *i3c, int timeout)
{
    while (!(i3c->cont_status & MXC_F_I3C_REVA_CONT_STATUS_REQ_DONE)) {
        if (timeout == 0) {
            return E_TIME_OUT;
        }
        if (timeout > 0) {
            timeout--;
        }
    }

    return E_SUCCESS;
}

static inline int MXC_I3C_RevA_Controller_WaitForIdle(mxc_i3c_reva_regs_t *i3c, int timeout)
{
    while ((i3c->cont_status & MXC_F_I3C_REVA_CONT_STATUS_STATE) !=
           MXC_S_I3C_REVA_CONT_STATUS_STATE_IDLE) {
        if (timeout == 0) {
            return E_TIME_OUT;
        }
        if (timeout > 0) {
            timeout--;
        }
    }

    return E_SUCCESS;
}

int MXC_I3C_RevA_Recover(mxc_i3c_reva_regs_t *i3c)
{
    int retries = 3;

    while (retries &&
           MXC_I3C_RevA_Controller_GetState(i3c) == MXC_V_I3C_REVA_CONT_STATUS_STATE_SDR_NORM) {
        MXC_I3C_RevA_EmitStop(i3c);
        retries--;
    }

    while (GET_FIELD(cont_status, CONT_STATUS_TARG_START)) {
        SET_FIELD(cont_status, CONT_STATUS_TARG_START, 1);
        MXC_I3C_RevA_AutoIBI(i3c);
        MXC_I3C_RevA_Controller_WaitForDone(i3c, 1000);
        MXC_I3C_RevA_ClearRXFIFO(i3c);
    }

    retries = 1000;
    while (retries &&
           MXC_I3C_RevA_Controller_GetState(i3c) != MXC_V_I3C_REVA_CONT_STATUS_STATE_IDLE) {
        retries--;
    }

    if (retries == 0) {
        return E_TIME_OUT;
    }

    return 0;
}

int MXC_I3C_RevA_Shutdown(mxc_i3c_reva_regs_t *i3c)
{
    int idx;

    idx = MXC_I3C_GET_IDX((mxc_i3c_regs_t *)i3c);
    if (idx < 0) {
        return E_BAD_PARAM;
    }

    if (GET_FIELD(cont_ctrl0, CONT_CTRL0_EN)) {
        SET_FIELD(cont_ctrl0, CONT_CTRL0_EN, MXC_V_I3C_REVA_CONT_CTRL0_EN_OFF);
        MXC_I3C_RevA_Controller_DisableInt(i3c, i3c->cont_inten);
        MXC_I3C_RevA_Controller_ClearFlags(i3c, i3c->cont_intfl);
    } else if (GET_FIELD(targ_ctrl0, TARG_CTRL0_EN)) {
        SET_FIELD(targ_ctrl0, TARG_CTRL0_EN, 0);
        MXC_I3C_RevA_Target_DisableInt(i3c, i3c->targ_inten);
        MXC_I3C_RevA_Target_ClearFlags(i3c, i3c->targ_intfl);
    }

    MXC_I3C_RevA_ClearRXFIFO(i3c);
    MXC_I3C_RevA_ClearTXFIFO(i3c);
    controller[idx].ibiGetByteCB = NULL;

    return E_SUCCESS;
}

int MXC_I3C_RevA_SetPPFrequency(mxc_i3c_reva_regs_t *i3c, unsigned int frequency)
{
    uint32_t ticks, highPeriod, lowPeriod;
    uint8_t ticks_min, ticks_max;

    /* Minimum tick count for SCL high and low is (PP_BAUD_MIN + 1) */
    ticks_min = (MXC_V_I3C_REVA_CONT_CTRL0_PP_BAUD_1_FCLK + 1) << 1;
    /* Maximum tick count for SCL high is (PP_BAUD_MAX + 1), and for
     * SCL low is (PP_BAUD_MAX + 1) + (PP_ADD_LBAUD_MAX) */
    ticks_max = ((MXC_V_I3C_REVA_CONT_CTRL0_PP_BAUD_16_FCLK + 1) << 1) +
                MXC_V_I3C_REVA_CONT_CTRL0_PP_ADD_LBAUD_15_FCLK;

    /* Divide and round closest */
    ticks = (PeripheralClock + (frequency >> 1)) / frequency;
    if (ticks < ticks_min || ticks > ticks_max) {
        return E_BAD_PARAM;
    }

    highPeriod = ticks / 2;
    lowPeriod = ticks / 2;

    while (highPeriod > (MXC_V_I3C_REVA_CONT_CTRL0_PP_BAUD_16_FCLK + 1)) {
        highPeriod--;
        lowPeriod++;
    }

    if (ticks % 2) {
        lowPeriod++;
    }

    i3c->cont_ctrl0 &= ~MXC_F_I3C_REVA_CONT_CTRL0_PP_BAUD;
    i3c->cont_ctrl0 &= ~MXC_F_I3C_REVA_CONT_CTRL0_PP_ADD_LBAUD;
    i3c->cont_ctrl0 |= (highPeriod - 1) << MXC_F_I3C_REVA_CONT_CTRL0_PP_BAUD_POS;
    i3c->cont_ctrl0 |= (lowPeriod - highPeriod) << MXC_F_I3C_REVA_CONT_CTRL0_PP_ADD_LBAUD_POS;

    return (int)MXC_I3C_RevA_GetPPFrequency(i3c);
}

unsigned int MXC_I3C_RevA_GetPPFrequency(mxc_i3c_reva_regs_t *i3c)
{
    uint8_t highPeriod, lowPeriod;

    highPeriod = 1 + ((i3c->cont_ctrl0 & MXC_F_I3C_REVA_CONT_CTRL0_PP_BAUD) >>
                      MXC_F_I3C_REVA_CONT_CTRL0_PP_BAUD_POS);
    lowPeriod = highPeriod + ((i3c->cont_ctrl0 & MXC_F_I3C_REVA_CONT_CTRL0_PP_ADD_LBAUD) >>
                              MXC_F_I3C_REVA_CONT_CTRL0_PP_ADD_LBAUD_POS);

    return PeripheralClock / (highPeriod + lowPeriod);
}

int MXC_I3C_RevA_SetODFrequency(mxc_i3c_reva_regs_t *i3c, unsigned int frequency, bool highPP)
{
    uint32_t ticks, lowPeriod, odInClk, ticks_min;
    uint8_t ppBaud;

    ppBaud = (i3c->cont_ctrl0 & MXC_F_I3C_REVA_CONT_CTRL0_PP_BAUD) >>
             MXC_F_I3C_REVA_CONT_CTRL0_PP_BAUD_POS;
    ppBaud = ppBaud + 1;
    odInClk = PeripheralClock / ppBaud;

    /* Minimum low period is 200ns
     * ticks_min = (200 * odInClk) / 1000000000;
     * Eliminate two 0s in above equation to avoid exceeding UINT32_MAX.
     */
    ticks_min = (2 * odInClk) / 10000000;
    ticks = (odInClk + (frequency >> 1)) / frequency;
    if (ticks < 2) {
        return E_BAD_PARAM;
    }

    if (highPP) {
        lowPeriod = ticks - 1;
    } else {
        lowPeriod = ticks / 2;
    }

    if (lowPeriod < ticks_min) {
        lowPeriod = ticks_min;
    }

    lowPeriod -= 1;

    i3c->cont_ctrl0 &= ~(MXC_F_I3C_REVA_CONT_CTRL0_OD_LBAUD | MXC_F_I3C_REVA_CONT_CTRL0_OD_HP);
    i3c->cont_ctrl0 |= lowPeriod << MXC_F_I3C_REVA_CONT_CTRL0_OD_LBAUD_POS;
    i3c->cont_ctrl0 |= (highPP ? 1 : 0) << MXC_F_I3C_REVA_CONT_CTRL0_OD_HP_POS;

    return (int)MXC_I3C_RevA_GetODFrequency(i3c);
}

unsigned int MXC_I3C_RevA_GetODFrequency(mxc_i3c_reva_regs_t *i3c)
{
    uint8_t highPeriod, lowPeriod, odBaud, ppBaud;

    ppBaud = (i3c->cont_ctrl0 & MXC_F_I3C_REVA_CONT_CTRL0_PP_BAUD) >>
             MXC_F_I3C_REVA_CONT_CTRL0_PP_BAUD_POS;
    odBaud = (i3c->cont_ctrl0 & MXC_F_I3C_REVA_CONT_CTRL0_OD_LBAUD) >>
             MXC_F_I3C_REVA_CONT_CTRL0_OD_LBAUD_POS;
    lowPeriod = (odBaud + 1) * (ppBaud + 1);
    if (i3c->cont_ctrl0 & MXC_F_I3C_REVA_CONT_CTRL0_OD_HP) {
        highPeriod = ppBaud + 1;
    } else {
        highPeriod = lowPeriod;
    }

    return PeripheralClock / (highPeriod + lowPeriod);
}

int MXC_I3C_RevA_SetI2CFrequency(mxc_i3c_reva_regs_t *i3c, unsigned int frequency)
{
    uint32_t highPeriod, i2cInClk, ticks;
    uint8_t odBaud, ppBaud;

    ppBaud = (i3c->cont_ctrl0 & MXC_F_I3C_REVA_CONT_CTRL0_PP_BAUD) >>
             MXC_F_I3C_REVA_CONT_CTRL0_PP_BAUD_POS;
    odBaud = (i3c->cont_ctrl0 & MXC_F_I3C_REVA_CONT_CTRL0_OD_LBAUD) >>
             MXC_F_I3C_REVA_CONT_CTRL0_OD_LBAUD_POS;

    i2cInClk = PeripheralClock / ((odBaud + 1) * (ppBaud + 1));
    ticks = (i2cInClk + (frequency >> 1)) / frequency;
    highPeriod = ticks / 2;

    highPeriod = (highPeriod - 1) << 1;
    if (ticks % 2) {
        highPeriod++;
    }

    if (highPeriod > 0xE) {
        highPeriod = 0xE;
    }

    i3c->cont_ctrl0 &= ~MXC_F_I3C_REVA_CONT_CTRL0_I2C_BAUD;
    i3c->cont_ctrl0 |= highPeriod << MXC_F_I3C_REVA_CONT_CTRL0_I2C_BAUD_POS;

    return (int)MXC_I3C_RevA_GetI2CFrequency(i3c);
}

unsigned int MXC_I3C_RevA_GetI2CFrequency(mxc_i3c_reva_regs_t *i3c)
{
    uint8_t i2cBaud, odBaud, ppBaud;
    uint32_t lowPeriod, highPeriod;

    i2cBaud = (i3c->cont_ctrl0 & MXC_F_I3C_REVA_CONT_CTRL0_I2C_BAUD) >>
              MXC_F_I3C_REVA_CONT_CTRL0_I2C_BAUD_POS;
    ppBaud = (i3c->cont_ctrl0 & MXC_F_I3C_REVA_CONT_CTRL0_PP_BAUD) >>
             MXC_F_I3C_REVA_CONT_CTRL0_PP_BAUD_POS;
    odBaud = (i3c->cont_ctrl0 & MXC_F_I3C_REVA_CONT_CTRL0_OD_LBAUD) >>
             MXC_F_I3C_REVA_CONT_CTRL0_OD_LBAUD_POS;

    highPeriod = (i2cBaud >> 1) + 1;
    lowPeriod = highPeriod;
    if ((i2cBaud % 2) != 0) {
        lowPeriod++;
    }

    return PeripheralClock / ((lowPeriod + highPeriod) * ((odBaud + 1) * (ppBaud + 1)));
}

uint8_t MXC_I3C_RevA_GetDynamicAddress(mxc_i3c_reva_regs_t *i3c)
{
    if (GET_FIELD(targ_ctrl0, TARG_CTRL0_EN) && GET_FIELD(targ_dynaddr, TARG_DYNADDR_VALID)) {
        return GET_FIELD(targ_dynaddr, TARG_DYNADDR_ADDR);
    }

    return MXC_I3C_ADDR_INVALID;
}

int MXC_I3C_RevA_SetSkew(mxc_i3c_reva_regs_t *i3c, uint8_t skew)
{
    unsigned int ppFreq;

    if (skew > 7) {
        return E_BAD_PARAM;
    }

    if (skew > 0) {
        ppFreq = MXC_I3C_RevA_GetPPFrequency(i3c);
        if ((PeripheralClock / ppFreq) < 4) {
            return E_BAD_STATE;
        }
    }

    i3c->cont_ctrl0 &= ~MXC_F_I3C_REVA_CONT_CTRL0_PP_SKEW;
    i3c->cont_ctrl0 |= (uint32_t)skew << MXC_F_I3C_REVA_CONT_CTRL0_PP_SKEW_POS;

    return E_SUCCESS;
}

int MXC_I3C_RevA_SetHighKeeperMode(mxc_i3c_reva_regs_t *i3c, mxc_i3c_reva_high_keeper_t hkeep)
{
    if (hkeep > MXC_I3C_REVA_HIGH_KEEPER_EXT_SCL_SDA) {
        return E_BAD_PARAM;
    }

    i3c->cont_ctrl0 &= ~MXC_F_I3C_REVA_CONT_CTRL0_HKEEP;
    i3c->cont_ctrl0 |= hkeep;

    return E_SUCCESS;
}

/**
 * @brief   Request an I3C or I2C bus operation.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   request     Request type. e.g. MXC_V_I3C_REVA_CONT_CTRL1_REQUEST_EMIT_START_ADDR.
 * @param   type        Request type if EmitStartAddr is requested. For other requests, pass 0.
 * @param   xferType    1 if this is a read request, 0 if write.
 * @param   addr        Target address. This is usually 7'h7E for I3C broadcast messages,
 *                      target's static address for I2C messages and target's dynamic
 *                      address for I3C SDR messages.
 * @param   readCount   Maximum bytes allowed to be read from the target. Set to 0 if you wish
 *                      to allow the read until the target stops sending.
 */
static void MXC_I3C_RevA_Controller_Request(mxc_i3c_reva_regs_t *i3c, uint8_t request,
                                            mxc_i3c_reva_start_type_t type,
                                            mxc_i3c_transfer_type_t xferType, uint8_t addr,
                                            uint8_t readCount)
{
    uint32_t cont1;

    cont1 = (request << MXC_F_I3C_CONT_CTRL1_REQ_POS) | (addr << MXC_F_I3C_CONT_CTRL1_ADDR_POS) |
            (type << MXC_F_I3C_CONT_CTRL1_TYPE_POS) |
            (MXC_I3C_REVA_CONT_CTRL1_IBIRESP_NACK << MXC_F_I3C_CONT_CTRL1_IBIRESP_POS);
    if (xferType == MXC_I3C_TRANSFER_TYPE_READ) {
        cont1 |= 1U << MXC_F_I3C_CONT_CTRL1_RDWR_DIR_POS;
        cont1 |= readCount << MXC_F_I3C_CONT_CTRL1_TERM_RD_POS;
    }

    i3c->cont_ctrl1 = cont1;
}

int MXC_I3C_RevA_EmitStart(mxc_i3c_reva_regs_t *i3c, bool isI2C, mxc_i3c_transfer_type_t xferType,
                           uint8_t addr, uint8_t readCount)
{
    MXC_I3C_RevA_Controller_ClearError(i3c);
    MXC_I3C_RevA_Controller_ClearFlags(i3c, MXC_F_I3C_REVA_CONT_STATUS_MASK);

    MXC_I3C_RevA_Controller_Request(i3c, MXC_V_I3C_REVA_CONT_CTRL1_REQ_EMIT_START,
                                    isI2C ? MXC_I3C_REVA_START_TYPE_I2C :
                                            MXC_I3C_REVA_START_TYPE_SDR,
                                    xferType, addr, readCount);

    if (MXC_I3C_RevA_Controller_WaitForReqDone(i3c, 1000) < 0) {
        return E_TIME_OUT;
    }

    return MXC_I3C_RevA_Controller_GetError(i3c);
}

int MXC_I3C_RevA_ResetTarget(mxc_i3c_reva_regs_t *i3c)
{
    MXC_I3C_RevA_Controller_ClearError(i3c);
    MXC_I3C_RevA_Controller_ClearFlags(i3c, MXC_F_I3C_REVA_CONT_STATUS_MASK);

    MXC_I3C_RevA_Controller_Request(i3c, MXC_V_I3C_REVA_CONT_CTRL1_REQ_EXIT_RST,
                                    MXC_I3C_REVA_START_TYPE_SDR, 0, 0, 0);

    if (MXC_I3C_RevA_Controller_WaitForReqDone(i3c, 1000) < 0) {
        return E_TIME_OUT;
    }

    return MXC_I3C_RevA_Controller_GetError(i3c);
}

void MXC_I3C_RevA_EmitStop(mxc_i3c_reva_regs_t *i3c)
{
    /* Configure MCTRL register for STOP */
    i3c->cont_ctrl1 &= ~(MXC_F_I3C_REVA_CONT_CTRL1_REQ | MXC_F_I3C_REVA_CONT_CTRL1_RDWR_DIR |
                         MXC_F_I3C_REVA_CONT_CTRL1_TERM_RD);
    i3c->cont_ctrl1 |= MXC_S_I3C_REVA_CONT_CTRL1_REQ_EMIT_STOP;
    /* Wait for IDLE state */
    MXC_I3C_RevA_Controller_WaitForIdle(i3c, 1000);
}

void MXC_I3C_RevA_EmitI2CStop(mxc_i3c_reva_regs_t *i3c)
{
    /* Configure MCTRL register for STOP */
    i3c->cont_ctrl1 = MXC_S_I3C_REVA_CONT_CTRL1_REQ_EMIT_STOP |
                      (1 << MXC_F_I3C_REVA_CONT_CTRL1_TYPE_POS);
    /* Wait for IDLE state */
    MXC_I3C_RevA_Controller_WaitForIdle(i3c, 1000);
}

int MXC_I3C_RevA_Controller_CCC(mxc_i3c_reva_regs_t *i3c, const mxc_i3c_reva_ccc_req_t *req)
{
    int ret;

    MXC_I3C_RevA_ClearTXFIFO(i3c);
    MXC_I3C_RevA_ClearRXFIFO(i3c);
    MXC_I3C_RevA_EmitStart(i3c, false, MXC_I3C_TRANSFER_TYPE_WRITE, MXC_I3C_BROADCAST_ADDR, 0);

    MXC_I3C_RevA_WriteTXFIFO(i3c, &req->ccc, 1, req->flags == 0, 100);
    if (req->flags & MXC_I3C_CCC_HAS_DEFINING_BYTE) {
        MXC_I3C_RevA_WriteTXFIFO(i3c, &req->def_byte, 1, req->tx_len == 0, 100);
    }
    MXC_I3C_RevA_Controller_WaitForDone(i3c, 1000);

    if (req->ccc & 0x80) {
        /* Direct CCC */
        ret = MXC_I3C_RevA_EmitStart(i3c, false, req->xfer_type, req->target_addr, req->rx_len);
        if (ret < 0) {
            goto err;
        }
        if (req->xfer_type == MXC_I3C_TRANSFER_TYPE_WRITE) {
            /* Write message */
            if (req->flags & MXC_I3C_CCC_HAS_SUB_COMMAND) {
                MXC_I3C_RevA_WriteTXFIFO(i3c, &req->sub_cmd, 1, (req->tx_len == 0), 100);
            }
            if (req->tx_len) {
                MXC_I3C_RevA_WriteTXFIFO(i3c, req->tx_buf, req->tx_len, true, 100);
            }
        } else {
            /* Read message */
            MXC_I3C_RevA_ReadRXFIFO(i3c, req->rx_buf, req->rx_len, -1);
        }
    } else {
        /* Broadcast CCC */
        if (req->tx_len) {
            MXC_I3C_RevA_WriteTXFIFO(i3c, req->tx_buf, req->tx_len, true, 100);
        }
    }
    MXC_I3C_RevA_Controller_WaitForDone(i3c, 1000);

    /* Check for errors */
    ret = MXC_I3C_RevA_Controller_GetError(i3c);
err:
    MXC_I3C_RevA_EmitStop(i3c);

    return ret;
}

int MXC_I3C_RevA_Controller_Transaction(mxc_i3c_reva_regs_t *i3c, const mxc_i3c_reva_req_t *req)
{
    int ret;
    uint8_t readCount;
    uint16_t remaining;
    uint32_t timeout;
    uint32_t freq;

    if (MXC_I3C_RevA_Controller_GetState(i3c) != MXC_V_I3C_REVA_CONT_STATUS_STATE_IDLE &&
        MXC_I3C_RevA_Controller_GetState(i3c) != MXC_V_I3C_REVA_CONT_STATUS_STATE_SDR_NORM) {
        return E_BUSY;
    }

    if (req->tx_len == 0 && req->rx_len == 0) {
        return E_BAD_PARAM;
    }

    MXC_I3C_RevA_ClearTXFIFO(i3c);
    MXC_I3C_RevA_ClearRXFIFO(i3c);

    if (!req->is_i2c) {
        ret = MXC_I3C_RevA_EmitStart(i3c, req->is_i2c, MXC_I3C_TRANSFER_TYPE_WRITE,
                                     MXC_I3C_BROADCAST_ADDR, 0);
        if (ret < 0) {
            goto err;
        }
        freq = MXC_I3C_RevA_GetPPFrequency(i3c);
    } else {
        freq = MXC_I3C_RevA_GetI2CFrequency(i3c);
    }

    /* Restart with write */
    if (req->tx_len) {
        ret = MXC_I3C_RevA_EmitStart(i3c, req->is_i2c, MXC_I3C_TRANSFER_TYPE_WRITE,
                                     req->target_addr, 0);
        if (ret < 0) {
            goto err;
        }

        /* A simple linear estimation to find a reasonable write timeout value,
           proportional to clock period and buffer size. Coefficient value has
           been found by trial-and-error.
        */
        timeout = (uint32_t)(40 * 1000000 / freq) * req->tx_len;

        ret = MXC_I3C_RevA_WriteTXFIFO(i3c, req->tx_buf, req->tx_len, true, timeout);
        if (ret < 0) {
            goto err;
        }

        MXC_I3C_RevA_Controller_WaitForDone(i3c, 1000);
    }

    if (req->rx_len) {
        remaining = req->rx_len;
        while (remaining > 0) {
            readCount = (remaining < MXC_I3C_REVA_MAX_FIFO_TRANSACTION) ?
                            remaining :
                            MXC_I3C_REVA_MAX_FIFO_TRANSACTION;

            ret = MXC_I3C_RevA_EmitStart(i3c, req->is_i2c, MXC_I3C_TRANSFER_TYPE_READ,
                                         req->target_addr, readCount);
            if (ret < 0) {
                goto err;
            }

            /* A simple linear estimation to find a reasonable read timeout value,
               proportional to clock period and buffer size. Coefficient value has
               been found by trial-and-error.
            */
            timeout = (uint32_t)(80 * 1000000 / freq) * readCount;

            ret = MXC_I3C_RevA_ReadRXFIFO(i3c, req->rx_buf + (req->rx_len - remaining), readCount,
                                          timeout);
            if (ret == readCount) {
                remaining -= readCount;
            } else {
                ret = MXC_I3C_RevA_Controller_GetError(i3c);
                goto err;
            }
        }

        MXC_I3C_RevA_Controller_WaitForDone(i3c, 1000);
    }

    /* Check for errors */
    ret = MXC_I3C_RevA_Controller_GetError(i3c);
err:
    if (req->stop || ret != E_SUCCESS) {
        MXC_I3C_RevA_EmitStop(i3c);
    }

    return ret;
}

int MXC_I3C_RevA_Controller_DAA(mxc_i3c_reva_regs_t *i3c, uint8_t addr, uint8_t *pid, uint8_t *bcr,
                                uint8_t *dcr)
{
    int ret;

    MXC_I3C_RevA_ClearTXFIFO(i3c);
    MXC_I3C_RevA_ClearRXFIFO(i3c);

    if (MXC_I3C_RevA_Controller_GetState(i3c) == MXC_V_I3C_REVA_CONT_STATUS_STATE_DAA &&
        GET_FIELD(cont_status, CONT_STATUS_WAIT)) {
        MXC_I3C_RevA_WriteTXFIFO(i3c, &addr, 1, true, 10);
        MXC_I3C_RevA_Controller_WaitForDone(i3c, 1000);
    }

    MXC_I3C_RevA_Controller_Request(i3c, MXC_V_I3C_REVA_CONT_CTRL1_REQ_PROCESS_DAA,
                                    MXC_I3C_REVA_START_TYPE_SDR, 0, 0, 0);
    ret = MXC_I3C_RevA_Controller_WaitForReqDone(i3c, 1000);

    if (ret == E_SUCCESS) {
        if (MXC_I3C_RevA_Controller_GetState(i3c) == MXC_V_I3C_REVA_CONT_STATUS_STATE_DAA &&
            GET_FIELD(cont_status, CONT_STATUS_WAIT)) {
            MXC_I3C_RevA_ReadRXFIFO(i3c, pid, MXC_I3C_PROVISIONED_ID_LEN, 10);
            MXC_I3C_RevA_ReadRXFIFO(i3c, bcr, 1, 1);
            MXC_I3C_RevA_ReadRXFIFO(i3c, dcr, 1, 1);
        } else if (MXC_I3C_RevA_Controller_GetState(i3c) == MXC_V_I3C_REVA_CONT_STATUS_STATE_IDLE &&
                   GET_FIELD(cont_status, CONT_STATUS_DONE)) {
            ret = E_SHUTDOWN;
        } else {
            ret = E_FAIL;
        }
    }

    return ret;
}

int MXC_I3C_RevA_AutoIBI(mxc_i3c_reva_regs_t *i3c)
{
    int timeout = 500;
    i3c->cont_ctrl1 &= ~(MXC_F_I3C_CONT_CTRL1_REQ | MXC_F_I3C_CONT_CTRL1_IBIRESP);
    i3c->cont_ctrl1 |= MXC_S_I3C_REVA_CONT_CTRL1_REQ_AUTO_IBI |
                       (MXC_I3C_REVA_IBIRESP_ACK << MXC_F_I3C_REVA_CONT_CTRL1_IBIRESP_POS);

    while (!GET_FIELD(cont_status, CONT_STATUS_IBI_WON)) {
        if (--timeout == 0) {
            return E_TIME_OUT;
        }
    }
    SET_FIELD(cont_status, CONT_STATUS_IBI_WON, 0);

    return 0;
}

int MXC_I3C_RevA_HotJoin(mxc_i3c_reva_regs_t *i3c)
{
    if (!(i3c->targ_cap1 & MXC_S_I3C_REVA_TARG_CAP1_IBI_EVENTS_HJ)) {
        return E_NOT_SUPPORTED;
    }

    if ((MXC_I3C_RevA_GetDynamicAddress(i3c) != MXC_I3C_ADDR_INVALID) ||
        (i3c->targ_status & MXC_F_I3C_REVA_TARG_STATUS_HJ_DIS)) {
        return E_BAD_STATE;
    }

    SET_FIELD(targ_ctrl0, TARG_CTRL0_EN, 0);
    SET_FIELD(targ_ctrl1, TARG_CTRL1_EVENT, MXC_V_I3C_REVA_TARG_CTRL1_EVENT_HJ);
    SET_FIELD(targ_ctrl0, TARG_CTRL0_EN, 1);

    return E_SUCCESS;
}

int MXC_I3C_RevA_RequestIBI(mxc_i3c_reva_regs_t *i3c, unsigned char mdb,
                            mxc_i3c_reva_ibi_getbyte_t getByteCb)
{
    int ret, idx;
    unsigned char byte;

    idx = MXC_I3C_GET_IDX((mxc_i3c_regs_t *)i3c);
    if (idx < 0) {
        return E_BAD_PARAM;
    }

    if (!(i3c->targ_cap1 & MXC_S_I3C_REVA_TARG_CAP1_IBI_EVENTS_HJ)) {
        return E_NOT_SUPPORTED;
    }

    if ((MXC_I3C_RevA_GetDynamicAddress(i3c) == MXC_I3C_ADDR_INVALID) ||
        (i3c->targ_status & MXC_F_I3C_REVA_TARG_STATUS_IBI_DIS)) {
        return E_BAD_STATE;
    }

    if (i3c->targ_status & MXC_F_I3C_REVA_TARG_STATUS_EVENT) {
        return E_BUSY;
    }

    /* Write MDB and additional data bytes to TX FIFO */
    controller[idx].ibiGetByteCB = getByteCb;
    if (i3c->targ_cap1 & MXC_S_I3C_REVA_TARG_CAP1_IBI_EVENTS_PAYLOAD) {
        if (getByteCb) {
            while ((i3c->targ_status & MXC_F_I3C_REVA_TARG_STATUS_TX_NFULL)) {
                ret = getByteCb(i3c, &byte);
                if (ret) {
                    i3c->targ_txfifo8 = byte;
                } else {
                    break;
                }
            }
        }
        SET_FIELD(targ_ctrl1, TARG_CTRL1_IBIDATA, mdb);
        SET_FIELD(targ_ctrl1, TARG_CTRL1_EXTIBI,
                  !!(i3c->targ_fifoctrl & MXC_F_I3C_REVA_TARG_FIFOCTRL_TX_LVL));
    }

    MXC_I3C_RevA_Target_EnableInt(i3c, MXC_F_I3C_REVA_TARG_INTEN_EVENT_REQ |
                                           MXC_F_I3C_REVA_TARG_INTEN_TX_NFULL |
                                           MXC_F_I3C_REVA_TARG_INTEN_STOP);

    SET_FIELD(targ_ctrl1, TARG_CTRL1_EVENT, MXC_V_I3C_REVA_TARG_CTRL1_EVENT_IBI);

    return E_SUCCESS;
}

int MXC_I3C_RevA_Standby(mxc_i3c_reva_regs_t *i3c)
{
    if (MXC_I3C_RevA_GetDynamicAddress(i3c) == MXC_I3C_ADDR_INVALID) {
        return E_BAD_STATE;
    }

    SET_FIELD(targ_ctrl0, TARG_CTRL0_EN, 0);

    return E_SUCCESS;
}

int MXC_I3C_RevA_Wakeup(mxc_i3c_reva_regs_t *i3c)
{
    if ((MXC_I3C_RevA_GetDynamicAddress(i3c) == MXC_I3C_ADDR_INVALID) ||
        (GET_FIELD(targ_ctrl0, TARG_CTRL0_EN))) {
        return E_BAD_STATE;
    }

    i3c->targ_ctrl0 |= (MXC_F_I3C_REVA_TARG_CTRL0_EN | MXC_F_I3C_REVA_TARG_CTRL0_OFFLINE);

    return E_SUCCESS;
}

void MXC_I3C_RevA_SetIBIPayloadCallback(mxc_i3c_reva_regs_t *i3c,
                                        mxc_i3c_reva_ibi_getbyte_t payloadCb)
{
    int idx;

    idx = MXC_I3C_GET_IDX((mxc_i3c_regs_t *)i3c);
    if (idx >= 0) {
        controller[idx].ibiGetByteCB = payloadCb;
    }
}

int MXC_I3C_RevA_Controller_GetError(mxc_i3c_reva_regs_t *i3c)
{
    uint32_t errFlags = i3c->cont_errwarn & MXC_F_I3C_REVA_CONT_ERRWARN_MASK;

    if (errFlags & MXC_F_I3C_REVA_CONT_ERRWARN_NACK) {
        return E_NO_RESPONSE;
    } else if (errFlags & MXC_F_I3C_REVA_CONT_ERRWARN_TX_ABT) {
        return E_ABORT;
    } else if (errFlags & (MXC_F_I3C_REVA_CONT_ERRWARN_INV_REQ | MXC_F_I3C_REVA_CONT_ERRWARN_MSG)) {
        return E_BAD_STATE;
    } else if (errFlags & MXC_F_I3C_REVA_CONT_ERRWARN_TO) {
        return E_TIME_OUT;
    } else if (errFlags &
               (MXC_F_I3C_REVA_CONT_ERRWARN_RX_UNR | MXC_F_I3C_REVA_CONT_ERRWARN_TX_OVR)) {
        return E_OVERFLOW;
    }

    return E_NO_ERROR;
}

void MXC_I3C_RevA_Controller_ClearError(mxc_i3c_reva_regs_t *i3c)
{
    i3c->cont_errwarn = MXC_F_I3C_REVA_CONT_ERRWARN_MASK;
}

int MXC_I3C_RevA_SetRXTXThreshold(mxc_i3c_reva_regs_t *i3c, mxc_i3c_reva_rx_threshold_t rxth,
                                  mxc_i3c_reva_tx_threshold_t txth)
{
    if (rxth > MXC_V_I3C_REVA_CONT_FIFOCTRL_RX_THD_LVL_3_QUARTER_FULL ||
        txth > MXC_V_I3C_REVA_CONT_FIFOCTRL_TX_THD_LVL_ALMOST_FULL) {
        return E_BAD_PARAM;
    }

    i3c->cont_fifoctrl &=
        ~(MXC_F_I3C_REVA_CONT_FIFOCTRL_RX_THD_LVL | MXC_F_I3C_REVA_CONT_FIFOCTRL_TX_THD_LVL);
    i3c->cont_fifoctrl = (rxth << MXC_F_I3C_REVA_CONT_FIFOCTRL_RX_THD_LVL_POS) |
                         (txth << MXC_F_I3C_REVA_CONT_FIFOCTRL_TX_THD_LVL_POS) |
                         MXC_F_I3C_REVA_CONT_FIFOCTRL_UNLOCK;

    return E_SUCCESS;
}

int MXC_I3C_RevA_ReadRXFIFO(mxc_i3c_reva_regs_t *i3c, volatile unsigned char *bytes,
                            unsigned int len, int timeout)
{
    unsigned int readb = 0;

    if (timeout == 0) {
        timeout = 1;
    } else if ((timeout > 0) && (timeout < (len + 1))) {
        timeout = len + 1;
    }

    while ((len > readb) && (timeout != 0)) {
        if (i3c->cont_status & MXC_F_I3C_REVA_CONT_STATUS_RX_RDY) {
            bytes[readb++] = i3c->cont_rxfifo8;
        }
        if (timeout > 0) {
            timeout--;
        }
    }

    return readb;
}

int MXC_I3C_RevA_WriteTXFIFO(mxc_i3c_reva_regs_t *i3c, const unsigned char *bytes, unsigned int len,
                             bool end, int timeout)
{
    unsigned int written = 0;

    if (len == 0) {
        return 0;
    }

    if (timeout == 0) {
        timeout = 1;
    } else if ((timeout > 0) && (timeout < (len + 1))) {
        timeout = len + 1;
    }

    while ((len - 1 > written) && (timeout != 0)) {
        if (i3c->cont_status & MXC_F_I3C_REVA_CONT_STATUS_TX_NFULL) {
            i3c->cont_txfifo8o = bytes[written++];
        }
        if (timeout > 0) {
            timeout--;
        }
    }

    while ((len == written + 1) && (timeout != 0)) {
        if (i3c->cont_status & MXC_F_I3C_REVA_CONT_STATUS_TX_NFULL) {
            if (end) {
                i3c->cont_txfifo8e = bytes[written++];
            } else {
                i3c->cont_txfifo8o = bytes[written++];
            }
        }
        if (timeout > 0) {
            timeout--;
        }
    }

    return written;
}

void MXC_I3C_RevA_IRQHandler(mxc_i3c_reva_regs_t *i3c)
{
    int idx;

    idx = MXC_I3C_GET_IDX((mxc_i3c_regs_t *)i3c);
    if (idx < 0) {
        return;
    }
}
