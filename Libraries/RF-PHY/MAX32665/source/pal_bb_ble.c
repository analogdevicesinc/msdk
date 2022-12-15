/*************************************************************************************************/
/*!
 *  Copyright (c) 2013-2019 Arm Ltd. All Rights Reserved.
 *
 *  Copyright (c) 2019-2020 Packetcraft, Inc.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
/*************************************************************************************************/
/* *****************************************************************************
 * Copyright (C) Analog Devices, All rights Reserved.
 *
 * This software is protected by copyright laws of the United States and
 * of foreign countries. This material may also be protected by patent laws
 * and technology transfer regulations of the United States and of foreign
 * countries. This software is furnished under a license agreement and/or a
 * nondisclosure agreement and may only be used or reproduced in accordance
 * with the terms of those agreements. Dissemination of this information to
 * any party or parties not specified in the license agreement and/or
 * nondisclosure agreement is expressly prohibited.
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
 **************************************************************************** */

/**
 * @file    pal_bb_ble.c
 * @brief   Platform Adaption Layer for Bluetooth Baseband.
 * @details
 *
 * TODO:
 * Inline encryption/decryption.
 * Handling timing requirements for coded PHY switching.
 *
 */

/**************************************************************************************************
  Includes
**************************************************************************************************/
#include <string.h>

#include "dbb_registers.h"

#include "pal_types.h"
#include "pal_bb_ble.h"
#include "pal_crypto.h"
#include "pal_bb_phy.h"
#include "pal_bb_afe.h"
#include "pal_bb_dbb.h"
#include "pal_led.h"

#include "pal_bb_cfg.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

#define PAL_BB_BLE_AAE_MASK 0x1C
#define PAL_BB_BLE_GP_RX_OFFSET 145
#define PAL_BB_BLE_TX_PHR_DEF 0xFFFF
#define PAL_BB_BLE_PHR_LEN_POS 0x8
#define PAL_BB_BLE_PHR_LEN_MASK 0xFF00
#define PAL_BB_BLE_PHR_FLAG_MASK 0x00FF
#define PAL_BB_BLE_MIC_LEN 0x04

/*! \brief      TIFS operation types. */
typedef enum {
    TIFS_NONE = 0,
    TIFS_TX,
    TIFS_RX,
} palBbBlePendingTifs_t;

/*! \brief      GP Event types types. */
typedef enum {
    GP_NONE = 0,
    GP_TX_START,
    GP_RX_START,
} palBbBleGpEvent_t;

/*! \brief      PalBbBle control block. */
typedef struct {
    PalBbBleRxIsr_t rxCallback;
    PalBbBleTxIsr_t txCallback;
    palBbBlePendingTifs_t pendingTifs;
    palBbBleGpEvent_t gpEvent;

    PalBbBleChan_t chanParams;
    PalBbBleDataParam_t dataParams;
    PalBbBleOpParam_t opParams;

    PalCryptoEnc_t palBbBleEnc;
    uint64_t txPktCnt;
    uint64_t rxPktCnt;

    /* TODO: Do we need the valid flags */
    bool_t txPktCntValid;
    bool_t rxPktCntValid;

    uint8_t *rxDataBuf;
    uint16_t rxDataBufLen;
} palBbBleCb_t;

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

static palBbBleCb_t palBbBleCb;

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/**************************************************************************************************
  Functions
**************************************************************************************************/

/*************************************************************************************************/
static void palBbGpStart(uint32_t due, palBbBleGpEvent_t event)
{
#if BB_LED_ENA

    /* Set the start time */
    MXC_DBB_CTRL->event_timing_gp_event_time = due;

    /* Start the timer */
    MXC_DBB_CTRL->event_timing_ctrl = (MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_ENABLE |
                                       MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_SET_GP_EVENT_TIME);

    palBbBleCb.gpEvent = event;
#endif
}

/*************************************************************************************************/
/*!
 *  \brief      Initialize the BLE baseband driver.
 *
 *  \return     None.
 *
 *  One-time initialization of BLE baseband driver.
 */
/*************************************************************************************************/
void PalBbBleInit(void)
{
    memset(&palBbBleCb, 0, sizeof(palBbBleCb_t)); // NOLINT
}

/*************************************************************************************************/
/*!
 *  \brief      Enable the BB hardware.
 *
 *  \return     None.
 *
 *  Wake the BB hardware out of sleep and enable for operation. All BB
 *  functionality is available when this routine completes. BB clock is set to
 *  zero and started.
 */
/*************************************************************************************************/
void PalBbBleEnable(void)
{
    volatile uint32_t flags;

    /* Clear any pending interrupts, save the current status */
    flags = MXC_DBB_CTRL->events_status;
    MXC_DBB_CTRL->events_status = 0;
    (void)flags;

    NVIC_ClearPendingIRQ(BTLE_TX_DONE_IRQn);
    NVIC_ClearPendingIRQ(BTLE_RX_RCVD_IRQn);
    NVIC_ClearPendingIRQ(BTLE_SFD_TO_IRQn);
    NVIC_ClearPendingIRQ(BTLE_GP_EVENT_IRQn);

    NVIC_EnableIRQ(BTLE_TX_DONE_IRQn);
    NVIC_EnableIRQ(BTLE_RX_RCVD_IRQn);
    NVIC_EnableIRQ(BTLE_SFD_TO_IRQn);
    NVIC_EnableIRQ(BTLE_GP_EVENT_IRQn);
}

/*************************************************************************************************/
/*!
 *  \brief      Disable the BB hardware.
 *
 *  \return     None.
 *
 *  Disable the baseband and put radio hardware to sleep. Must be called from an
 *  idle state. A radio operation cannot be in progress.
 */
/*************************************************************************************************/
void PalBbBleDisable(void)
{
    NVIC_DisableIRQ(BTLE_TX_DONE_IRQn);
    NVIC_DisableIRQ(BTLE_RX_RCVD_IRQn);
    NVIC_DisableIRQ(BTLE_SFD_TO_IRQn);
    NVIC_DisableIRQ(BTLE_GP_EVENT_IRQn);
}

/*************************************************************************************************/
/*!
 *  \brief      Set the data packet exchange parameters.
 *
 *  \param      pParam      Data exchange parameters.
 *
 *  \return     None.
 *
 *  Calling this routine will set parameters for all future transmit and receive
 *  operations until this routine is called again providing new parameters.
 */
/*************************************************************************************************/
void PalBbBleSetDataParams(const PalBbBleDataParam_t *pParam)
{
    palBbBleCb.dataParams = *pParam;
}

/*************************************************************************************************/
/*!
 *  \brief      Set the operation parameters.
 *
 *  \param      pOpParam    Operations parameters.
 *
 *  \return     None.
 *
 *  Calling this routine will set parameters for the next transmit or receive
 *  operations.
 */
/*************************************************************************************************/
void PalBbBleSetOpParams(const PalBbBleOpParam_t *pOpParam)
{
    palBbBleCb.opParams = *pOpParam;
}

/*************************************************************************************************/
/*!
 *  \brief      Set channelization parameters.
 *
 *  \param      pChan      Channelization parameters.
 *
 *  \return     None.
 *
 *  Calling this routine will set these parameters for all future transmit and
 *  receive operations until this routine is called again providing new
 *  parameters.
 *
 *  The setting of channelization parameters influence the operations of the
 *  following listed routines. Therefore, this routine is called to set the
 *  channel characteristics before the use of these listed packet routines.
 *
 *  - \a BbDrvTx()
 *  - \a BbDrvRx()
 *  - \a BbDrvTxTifs()
 *  - \a BbDrvRxTifs()
 *
 *  \note       The \a pParam contents are not guaranteed to be static and is
 *  only valid in the context of the call to this routine. Therefore parameters
 *  requiring persistence should be copied.
 */
/*************************************************************************************************/
void PalBbBleSetChannelParam(PalBbBleChan_t *pChan)
{
    palBbBleCb.chanParams = *pChan;
}

/*************************************************************************************************/
/*!
 *  \brief      Enable or disable data whitening.
 *
 *  \param      enable       Flag to indicate data whitening.
 *
 *  \return     None.
 *
 *  Sets an internal variable that indicates if data whitening is enabled or
 *  not.
 */
/*************************************************************************************************/
void PalBbBleEnableDataWhitening(bool_t enable)
{
    PAL_BB_SETFIELD(MXC_DBB_RX->rx_dl_btle_settings_whit,
                    MXC_F_DBB_RX_RX_DL_BTLE_SETTINGS_WHIT_BYPASS,
                    (!enable << MXC_F_DBB_RX_RX_DL_BTLE_SETTINGS_WHIT_BYPASS_POS));

    PAL_BB_SETFIELD(MXC_DBB_TX->tx_dl_btle_settings_whit,
                    MXC_F_DBB_TX_TX_DL_BTLE_SETTINGS_WHIT_BYPASS,
                    (!enable << MXC_F_DBB_TX_TX_DL_BTLE_SETTINGS_WHIT_BYPASS_POS));
}

/*************************************************************************************************/
/*!
 *  \brief      Enable or disable PRBS15.
 *
 *  \param      enable       Flag to indicate PRBS15.
 *
 *  \return     None.
 *
 *  Immediately enable or disable continuous PRBS15 bitstream. Setting the
 *  channelization parameters with \a PalBbBleSetChannelParam() must precede
 *  enabling PRBS15.
 *
 *  Use of \a PAL_BB_BLE_DATA routines is not allowed while PRBS15 is enabled.
 */
/*************************************************************************************************/
void PalBbBleEnablePrbs15(bool_t enable) {}

/*************************************************************************************************/
/*!
 *  \brief      Transmit a packet.
 *
 *  \param      descs       Array of transmit buffer descriptor.
 *  \param      cnt         Number of descriptors.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalBbBleTxData(PalBbBleTxBufDesc_t descs[], uint8_t cnt)
{
    uint16_t pduHdr;
    uint8_t *pBuf;
    uint32_t due, now, txStartupUsec;
    bool_t nowValid;
    unsigned payloadOffset, iDescs;

    static const uint32_t txSetupMargin = 15;

    PalBbDbbSetAccAddr(palBbBleCb.chanParams.accAddr);
    PalBbDbbSetCrcInit(palBbBleCb.chanParams.crcInit);
    PalBbPhySet(palBbBleCb.chanParams.txPhy, palBbBleCb.chanParams.initTxPhyOptions);
    PalBbDbbSetChannel(palBbBleCb.chanParams.chanIdx);

    /* Store the callback */
    palBbBleCb.txCallback = palBbBleCb.dataParams.txCback;

    /* Extract the packet header */
    pBuf = descs[0].pBuf;
    pduHdr = pBuf[0] | (pBuf[1] << 8); // NOLINT
    pBuf += 2;

    PalBbAfeTxSetup();

    /* Setup the encryption params */
    if (palBbBleCb.palBbBleEnc.enaEncrypt && palBbBleCb.txPktCntValid) {
        uint8_t len;
        uint8_t aad;

        /* Extract the length */
        len = (uint8_t)((pduHdr & PAL_BB_BLE_PHR_LEN_MASK) >> PAL_BB_BLE_PHR_LEN_POS);

        /* Extract the additional authentication data */
        /* The Data Channel PDU header’s first octet with NESN, SN, */
        /*   and MD bits masked to 0 */
        aad = (uint8_t)((pduHdr & PAL_BB_BLE_PHR_FLAG_MASK) & ~(PAL_BB_BLE_AAE_MASK));

        PalBbDbbEnableEncryption(palBbBleCb.txPktCnt, len, aad, palBbBleCb.palBbBleEnc.dir);
    } else {
        PalBbDbbDisableEncryption();
    }
    palBbBleCb.txPktCntValid = FALSE;

    /* TODO: Determine if we need more margin */
    due = palBbBleCb.dataParams.dueUsec;

    /* Adjust the due time for the enable time */
    txStartupUsec = PalBbPhyGetTxStartup();
    due -= (((MXC_DBB_CTRL->tx_ctrl_rffe_en_dly[0] + MXC_DBB_CTRL->tx_ctrl_rffe_en_dly[1]) /
             (DBB_CLK_RATE_HZ / BB_CLK_RATE_HZ)) +
            txStartupUsec);

    /* Determine if we can meet the due time */
    nowValid = PalBbGetTimestamp(&now);
    if (!nowValid || ((due - now - txSetupMargin) > TIMESTAMP_WRAP_DIFF)) {
        /* Call the callback with error status if we can't make the due time */
        if (palBbBleCb.txCallback != NULL) {
            palBbBleCb.txCallback(BB_STATUS_FAILED);
        }
        return;
    }

    /* Set TX start time */
    MXC_DBB_CTRL->event_timing_tx_enable_time = due;

    /* Enable RX start */
    MXC_DBB_CTRL->event_timing_ctrl = (MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_ENABLE |
                                       MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_SET_TX_ENABLE_TIME);

    /* Set the header */
    MXC_DBB_TX->tx_dl_phr = pduHdr;

    /* Copy the rest of the first payload, -2 for the 2 byte header */
    unsigned copyLen = descs[0].len - 2;
    if ((copyLen & 0x3) == 0) {
        copyLen++;
    }
    memcpy((void *)&MXC_DBB_TX->tx_pld_mem[0], pBuf, copyLen); // NOLINT

    /* Copy the rest of the payloads */
    payloadOffset = descs[0].len - 2;
    iDescs = 1;
    while (--cnt) {
        pBuf = descs[iDescs].pBuf;

        copyLen = descs[iDescs].len;
        if ((copyLen & 0x3) == 0) {
            copyLen++;
        }
        memcpy((void *)&MXC_DBB_TX->tx_pld_mem[payloadOffset], pBuf, copyLen); // NOLINT

        payloadOffset += descs[iDescs].len;
        iDescs++;
    }

    /* Configure the TIFS timers */
    switch (palBbBleCb.opParams.ifsMode) {
    default:
    case PAL_BB_IFS_MODE_CLR:
    case PAL_BB_IFS_MODE_SAME_ABS:
        PalBbDbbDisableTIFS();
        palBbBleCb.pendingTifs = TIFS_NONE;
        break;
    case PAL_BB_IFS_MODE_TOGGLE_TIFS:
        PalBbDbbEnableTIFS();
        palBbBleCb.pendingTifs = TIFS_RX;
        break;
    }

    palBbGpStart(palBbBleCb.dataParams.dueUsec, GP_TX_START);
}

/*************************************************************************************************/
/*!
 *  \brief      Transmit packet on TIFS timing.
 *
 *  \param      descs       Transmit data buffer descriptor.
 *  \param      cnt         Transmit data count.
 *
 *  \return     None.
 *
 *  If possible, the transmit will occur at the TIFS timing. If not possible,
 *  the callback status will indicate this.
 */
/*************************************************************************************************/
void PalBbBleTxTifsData(PalBbBleTxBufDesc_t descs[], uint8_t cnt)
{
    uint16_t pduHdr;
    uint8_t *pBuf;
    unsigned payloadOffset, iDescs;

    /* Store the callback */
    palBbBleCb.txCallback = palBbBleCb.dataParams.txCback;

    /* Extract the packet header */
    pBuf = descs[0].pBuf;
    pduHdr = pBuf[0] | (pBuf[1] << 8); // NOLINT
    pBuf += 2;

    PalBbAfeTxSetup();

    /* Setup the encryption params */
    if (palBbBleCb.palBbBleEnc.enaEncrypt && palBbBleCb.txPktCntValid) {
        uint8_t len;
        uint8_t aad;

        /* Extract the length */
        len = (uint8_t)((pduHdr & PAL_BB_BLE_PHR_LEN_MASK) >> PAL_BB_BLE_PHR_LEN_POS);

        /* Extract the additional authentication data */
        /* The Data Channel PDU header’s first octet with NESN, SN, */
        /*   and MD bits masked to 0 */
        aad = (uint8_t)((pduHdr & PAL_BB_BLE_PHR_FLAG_MASK) & ~(PAL_BB_BLE_AAE_MASK));

        PalBbDbbEnableEncryption(palBbBleCb.txPktCnt, len, aad, palBbBleCb.palBbBleEnc.dir);
    } else {
        PalBbDbbDisableEncryption();
    }
    palBbBleCb.txPktCntValid = FALSE;

    /* Use the encoding of the received packet to determine if we're using S2 or
     * S8 */
    if (palBbBleCb.chanParams.rxPhy == BB_PHY_BLE_CODED) {
        if (MXC_DBB_RX->rx_dl_out_ci & MXC_S_DBB_RX_RX_DL_OUT_CI_CI_S2) {
            MXC_DBB_TX->tx_dl_btle_longrange &= ~MXC_F_DBB_TX_TX_DL_BTLE_LONGRANGE_S_EQ_8;
        } else {
            MXC_DBB_TX->tx_dl_btle_longrange |= MXC_F_DBB_TX_TX_DL_BTLE_LONGRANGE_S_EQ_8;
        }
    }

    /* TODO: Handle absolute timing */

    /* Set the header */
    MXC_DBB_TX->tx_dl_phr = pduHdr;

    /* Copy the rest of the first payload, -2 for the 2 byte header */
    memcpy((void *)&MXC_DBB_TX->tx_pld_mem[0], pBuf, descs[0].len - 2); // NOLINT

    /* Copy the rest of the payloads */
    payloadOffset = descs[0].len - 2;
    iDescs = 1;
    while (--cnt) {
        pBuf = descs[iDescs].pBuf;
        memcpy((void *)&MXC_DBB_TX->tx_pld_mem[payloadOffset], pBuf, descs[iDescs].len); // NOLINT

        payloadOffset += descs[iDescs].len;
        iDescs++;
    }

    /* Configure the TIFS timers */
    switch (palBbBleCb.opParams.ifsMode) {
    default:
    case PAL_BB_IFS_MODE_CLR:
    case PAL_BB_IFS_MODE_SAME_ABS:
        PalBbDbbDisableTIFS();
        palBbBleCb.pendingTifs = TIFS_NONE;
        break;
    case PAL_BB_IFS_MODE_TOGGLE_TIFS:
        PalBbDbbEnableTIFS();
        palBbBleCb.pendingTifs = TIFS_RX;
        break;
    }

    /* TODO: Account for different TIFS timings */
    palBbGpStart(MXC_DBB_CTRL->event_timing_timestamp_rx_received + PAL_BB_BLE_GP_RX_OFFSET,
                 GP_TX_START);
}

/*************************************************************************************************/
/*!
 *  \brief      Receive packet at the given due time.
 *
 *  \param      pBuf        Transmit data buffer.
 *  \param      len         Length of data buffer.
 *
 *  \return     None.
 *
 *  The receiver is kept on for the amount of time previously configured by
 *  function call.
 */
/*************************************************************************************************/
void PalBbBleRxData(uint8_t *pBuf, uint16_t len)
{
    uint32_t due, now;
    bool_t nowValid;
    static const uint32_t rxSetupMargin = 15;

    PalBbDbbSetAccAddr(palBbBleCb.chanParams.accAddr);
    PalBbDbbSetCrcInit(palBbBleCb.chanParams.crcInit);
    PalBbPhySet(palBbBleCb.chanParams.rxPhy, BB_PHY_OPTIONS_DEFAULT);
    PalBbDbbSetChannel(palBbBleCb.chanParams.chanIdx);

    /* Store the callback */
    palBbBleCb.rxCallback = palBbBleCb.dataParams.rxCback;

    /* Store the RX buffer */
    palBbBleCb.rxDataBuf = pBuf;
    palBbBleCb.rxDataBufLen = len;

    /* Set the decryption params */
    if (palBbBleCb.palBbBleEnc.enaDecrypt && palBbBleCb.rxPktCntValid) {
        PalBbDbbEnableDecryption(palBbBleCb.rxPktCnt, !palBbBleCb.palBbBleEnc.dir);
    } else {
        PalBbDbbDisableDecryption();
    }
    palBbBleCb.rxPktCntValid = FALSE;

    /* TODO: Determine if we need more margin */
    due = palBbBleCb.dataParams.dueUsec;

    /* Adjust the due time for the enable time */
    due -= ((MXC_DBB_CTRL->rx_ctrl_rffe_en_dly[0] + MXC_DBB_CTRL->rx_ctrl_rffe_en_dly[1]) /
            (DBB_CLK_RATE_HZ / BB_CLK_RATE_HZ));

    /* Determine if we can meet the due time */
    nowValid = PalBbGetTimestamp(&now);
    if (!nowValid || ((due - now - rxSetupMargin) > TIMESTAMP_WRAP_DIFF)) {
        /* Call the callback with error status if we can't make the due time */
        if (palBbBleCb.rxCallback != NULL) {
            palBbBleCb.rxCallback(BB_STATUS_FAILED, 0, 0, due, 0);
        }
        return;
    }

    PalBbAfeRxSetup();

    /* Set RX start time */
    MXC_DBB_CTRL->event_timing_rx_enable_time = due;

    /* Enable RX start */
    MXC_DBB_CTRL->event_timing_ctrl = (MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_ENABLE |
                                       MXC_F_DBB_CTRL_EVENT_TIMING_CTRL_SET_RX_ENABLE_TIME);

    /* Configure the TIFS timers */
    switch (palBbBleCb.opParams.ifsMode) {
    default:
    case PAL_BB_IFS_MODE_CLR:
    case PAL_BB_IFS_MODE_SAME_ABS:
        PalBbDbbDisableTIFS();
        palBbBleCb.pendingTifs = TIFS_NONE;
        break;
    case PAL_BB_IFS_MODE_TOGGLE_TIFS:
        PalBbDbbEnableTIFS();
        palBbBleCb.pendingTifs = TIFS_TX;
        break;
    }

    PalBbDbbSetRxTimeout(palBbBleCb.dataParams.rxTimeoutUsec, palBbBleCb.chanParams.rxPhy);

    /* Use the GP timer to indicate the RX */
    palBbGpStart(palBbBleCb.dataParams.dueUsec, GP_RX_START);
}

/*************************************************************************************************/
/*!
 *  \brief      Receive packet on TIFS timing.
 *
 *  \param      pBuf        Receive data buffer.
 *  \param      len         Length of data buffer.
 *
 *  \return     None.
 *
 *  The receiver is left on for the minimum amount of time to recognize a
 *  receive.
 *
 *  If possible, the receive will occur on the TIFS timing. If not possible, the
 *  callback status will indicate this.
 */
/*************************************************************************************************/
void PalBbBleRxTifsData(uint8_t *pBuf, uint16_t len)
{
    /* Store the callback */
    palBbBleCb.rxCallback = palBbBleCb.dataParams.rxCback;

    /* TODO: Rx Setup here is causing advertising failure, call from TX Done IRQ
     * handler */
    /* PalBbAfeRxSetup(); */

    PalBbDbbSetRxTimeout(palBbBleCb.dataParams.rxTimeoutUsec, palBbBleCb.chanParams.rxPhy);

    /* Save the RX buffer */
    palBbBleCb.rxDataBuf = pBuf;
    palBbBleCb.rxDataBufLen = len;

    /* Set the decryption params */
    if (palBbBleCb.palBbBleEnc.enaDecrypt && palBbBleCb.rxPktCntValid) {
        PalBbDbbEnableDecryption(palBbBleCb.rxPktCnt, !palBbBleCb.palBbBleEnc.dir);
    } else {
        PalBbDbbDisableDecryption();
    }
    palBbBleCb.rxPktCntValid = FALSE;

    /* Configure the TIFS timers */
    switch (palBbBleCb.opParams.ifsMode) {
    default:
    case PAL_BB_IFS_MODE_CLR:
    case PAL_BB_IFS_MODE_SAME_ABS:
        PalBbDbbDisableTIFS();
        palBbBleCb.pendingTifs = TIFS_NONE;
        break;
    case PAL_BB_IFS_MODE_TOGGLE_TIFS:
        PalBbDbbEnableTIFS();
        palBbBleCb.pendingTifs = TIFS_TX;
        break;
    }

    /* Use the GP timer to indicate the RX */
    /* TODO: Match TIFS timing */
    palBbGpStart(MXC_DBB_CTRL->event_timing_timestamp_tx_done + PAL_BB_BLE_GP_RX_OFFSET,
                 GP_RX_START);
}

/*************************************************************************************************/
/*!
 *  \brief      Cancel TIFS timer.
 *
 *  \return     None.
 *
 *  This stops any active TIFS timer operation. This routine is always called in
 *  the callback (i.e. ISR) context.
 */
/*************************************************************************************************/
void PalBbBleCancelTifs(void)
{
    /* Only cancel a pending TIFS operation */
    if (palBbBleCb.pendingTifs == TIFS_TX) {
        PalBbDbbTxCancel();
    } else if (palBbBleCb.pendingTifs == TIFS_RX) {
        PalBbDbbRxCancel();
    }

    palBbBleCb.pendingTifs = TIFS_NONE;
}

/*************************************************************************************************/
/*!
 *  \brief      Cancel a pending transmit or receive.
 *
 *  \return     None.
 *
 *  This stops any active radio operation. This routine is never called in the
 *  callback (i.e. ISR) context.
 */
/*************************************************************************************************/
void PalBbBleCancelData(void)
{
    PalBbDbbRxCancel();
    PalBbDbbTxCancel();

    /* Clear the callback state */
    palBbBleCb.rxCallback = NULL;
    palBbBleCb.rxDataBuf = NULL;
    palBbBleCb.rxDataBufLen = 0;
    palBbBleCb.txCallback = NULL;
}

/*************************************************************************************************/
static uint32_t palBbBleGetCRC(void)
{
    uint32_t crc = 0;
    uint8_t *pld_mem = (uint8_t *)MXC_DBB_RX->rx_pld_mem;

    /* Get the CRC value from the payload memory */
    // NOLINTBEGIN(*)
    memcpy(&crc, (void *)&(pld_mem[(MXC_DBB_RX->rx_dl_out_phr >> 8) & 0xFF]),
           ((MXC_DBB_RX->rx_dl_crc_mode & MXC_F_DBB_RX_RX_DL_CRC_MODE_PLD) >>
            MXC_F_DBB_RX_RX_DL_CRC_MODE_PLD_POS));
    // NOLINTEND(*)

    return crc;
}

/*************************************************************************************************/
static void palBbBleTsDelay(void)
{
    /* TODO: Not sure how necessary this is */
    return;

    /* Delay to allow for the clocks to synchronize. Interrupt occurs at fast
     * clock while time stamp registers are updated with a slower clock.
     */
    volatile int i;
    static const int DELAY_LEN = 64;
    for (i = 0; i < DELAY_LEN; i++) {}
}

/*************************************************************************************************/
void BTLE_TX_DONE_IRQHandler(void)
{
    static uint32_t old_tstamp = 0;
    uint32_t current_tstamp = MXC_DBB_CTRL->event_timing_timestamp_tx_done;
    volatile uint32_t flags;

    BB_LED_ON(BB_LED_PIN_ISR);

    palBbBleTsDelay();

    /* TODO: Fixes a hardware spurious interrupt bug */
    static const uint32_t VALID_TS_DIFF = 8;
    if ((current_tstamp - old_tstamp) < VALID_TS_DIFF) {
        old_tstamp = current_tstamp;
        BB_LED_OFF(BB_LED_PIN_ISR);

        return;
    }

    /* Clear any pending interrupts, save the current status */
    flags = MXC_DBB_CTRL->events_status;
    MXC_DBB_CTRL->events_status = 0;
    (void)flags;

    BB_LED_OFF(BB_LED_PIN_TX);
    old_tstamp = current_tstamp;

    /* Set to max length until ready to transmit */
    MXC_DBB_TX->tx_dl_phr = PAL_BB_BLE_TX_PHR_DEF;
    MXC_DBB_TX->tx_pld_mem[0] = 0;

    /* TODO: Test if we need to cancel TIFS operation */

    /* TODO: Correct TX done timestamp */

    if (palBbBleCb.txCallback != NULL) {
        palBbBleCb.txCallback(BB_STATUS_SUCCESS);
    }

    /* TODO: May need some timing here. Calling after the callback to prevent
    any AFE changes until after the packet has been transmitted OTA.
    Not working when we call this in the TIFS case. */
    /* PalBbAfeTxDone(); */

    /* TODO: Start GP timer to complete TX debug */

    BB_LED_OFF(BB_LED_PIN_ISR);
}

/*************************************************************************************************/
void BTLE_RX_RCVD_IRQHandler(void)
{
    uint32_t rxStartTime, crc;
    volatile uint32_t flags;
    uint8_t phy, phyOptions, status, rxLen;
    int8_t rssi;
    uint16_t phr;
    bool_t crcErr;

    BB_LED_ON(BB_LED_PIN_ISR);
    BB_LED_OFF(BB_LED_PIN_RX);

    palBbBleTsDelay();

    /* TODO: Maybe set this to 0 in the event that we don't have any data to TX
     */
    /* set to max length until ready to transmit */
    MXC_DBB_TX->tx_dl_phr = PAL_BB_BLE_TX_PHR_DEF;

    MXC_DBB_TX->tx_pld_mem[0] = 0;

    /* Clear any pending interrupts, save the current status */
    flags = MXC_DBB_CTRL->events_status;
    MXC_DBB_CTRL->events_status = 0;
    (void)flags;

    PalBbAfeRxDone();

    rxStartTime = MXC_DBB_CTRL->event_timing_timestamp_rx_sfd_det;
    phr = MXC_DBB_RX->rx_dl_out_phr;
    crc = palBbBleGetCRC();
    rssi = MXC_DBB_RX->rx_phy_rssi_out_pwr;

    /* Adjust timestamp based on PHY to the start of the access address */
    phy = PalBbPhyGetPhy();
    switch (phy) {
    default:
    case BB_PHY_BLE_1M:
        rxStartTime -= BB_BLE_PREAM_TO_ACCESS_1M;
        break;
    case BB_PHY_BLE_2M:
        rxStartTime -= BB_BLE_PREAM_TO_ACCESS_2M;
        break;
    case BB_PHY_BLE_CODED:
        rxStartTime -= BB_BLE_PREAM_TO_ACCESS_CODED;
        break;
    }

    /* Exctract the PHY options if using coded mode */
    if (phy == BB_PHY_BLE_CODED) {
        if (MXC_DBB_RX->rx_dl_out_ci & MXC_S_DBB_RX_RX_DL_OUT_CI_CI_S2) {
            phyOptions = BB_PHY_OPTIONS_BLE_S2;
        } else {
            phyOptions = BB_PHY_OPTIONS_BLE_S8;
        }
    } else {
        phyOptions = BB_PHY_OPTIONS_DEFAULT;
    }

    /* when no buffer specified, prepare the user callback and start the RX
     * finished chain */
    if ((palBbBleCb.rxDataBuf == NULL) || (palBbBleCb.rxDataBufLen < 2)) {
        /* when defined, call the callback */
        if (palBbBleCb.rxCallback) {
            palBbBleCb.rxCallback(BB_STATUS_FAILED, rssi, crc, rxStartTime, 0);
        }
        return;
    }

    /* Get the packet length from the header */
    rxLen = (phr & PAL_BB_BLE_PHR_LEN_MASK) >> PAL_BB_BLE_PHR_LEN_POS;

    /* If RX encryption is enabled, subtract the MIC bytes from the length in
     * the phy header */
    if (MXC_DBB_RX->rx_general_control & MXC_F_DBB_RX_RX_GENERAL_CONTROL_ENCRYPTION_ENABLE) {
        if (rxLen >= PAL_BB_BLE_MIC_LEN) {
            rxLen -= PAL_BB_BLE_MIC_LEN;
        }
    }

    /* Check for CRC errors or decrypt failures */
    if ((MXC_DBB_RX->rx_dl_out_crc_stat &
         (MXC_F_DBB_RX_RX_DL_OUT_CRC_STAT_PHR_NOK | MXC_F_DBB_RX_RX_DL_OUT_CRC_STAT_PLD_NOK)) ||
        (flags & MXC_F_DBB_CTRL_EVENTS_IRQ_TEST_DECRYPT_FAIL)) {
        crcErr = TRUE;
        rxLen = 0;

    } else {
        crcErr = FALSE;
    }

    /* Add the header to the buffer */
    memcpy(palBbBleCb.rxDataBuf, &phr, 2); // NOLINT
    palBbBleCb.rxDataBuf += 2;
    palBbBleCb.rxDataBufLen -= 2;

    /* Check the buffer length */
    if (rxLen > palBbBleCb.rxDataBufLen) {
        rxLen = palBbBleCb.rxDataBufLen;
    }

    /* Copy the data from the payload memory into the RX buffer */
    if (rxLen > 0) {
        memcpy((void *)palBbBleCb.rxDataBuf, (void *)MXC_DBB_RX->rx_pld_mem, rxLen); // NOLINT
    }

    /* Determine the RX status */
    if (crcErr) {
        status = BB_STATUS_CRC_FAILED;
        BB_LED_ON(BB_LED_PIN_RX_CRC);
    } else {
        status = BB_STATUS_SUCCESS;
        BB_LED_ON(BB_LED_PIN_RX_OK);
    }

    /* Clear the current RX buffer */
    palBbBleCb.rxDataBuf = NULL;
    palBbBleCb.rxDataBufLen = 0;

    /* when defined, call the callback */
    if (palBbBleCb.rxCallback) {
        palBbBleCb.rxCallback(status, rssi, crc, rxStartTime, phyOptions);
    }

    BB_LED_OFF(BB_LED_PIN_RX_OK);
    BB_LED_OFF(BB_LED_PIN_RX_CRC);
    BB_LED_OFF(BB_LED_PIN_ISR);
}

void BTLE_SFD_TO_IRQHandler(void)
{
    uint32_t rxTimemoutTimestamp;
    volatile uint32_t flags;

    BB_LED_ON(BB_LED_PIN_ISR);
    BB_LED_ON(BB_LED_PIN_RX_TO);
    BB_LED_OFF(BB_LED_PIN_RX);

    /* Clear any pending interrupts, save the current status */
    flags = MXC_DBB_CTRL->events_status;
    MXC_DBB_CTRL->events_status = 0;
    (void)flags;

    palBbBleTsDelay();

    /* set to max length until ready to transmit */
    MXC_DBB_TX->tx_dl_phr = PAL_BB_BLE_TX_PHR_DEF;
    MXC_DBB_TX->tx_pld_mem[0] = 0;

    rxTimemoutTimestamp = MXC_DBB_CTRL->event_timing_timestamp_rx_sfd_to;

    /* When defined, call the callback */
    if (palBbBleCb.rxCallback) {
        palBbBleCb.rxCallback(BB_STATUS_RX_TIMEOUT, 0, 0, rxTimemoutTimestamp, 0);
    }

    BB_LED_OFF(BB_LED_PIN_RX_TO);
    BB_LED_OFF(BB_LED_PIN_ISR);
}

void BTLE_GP_EVENT_IRQHandler(void)
{
    volatile uint32_t flags;

    /* Clear any pending interrupts, save the current status */
    flags = MXC_DBB_CTRL->events_status;
    MXC_DBB_CTRL->events_status = 0;
    (void)flags;

#if BB_LED_ENA
    BB_LED_ON(BB_LED_PIN_ISR);

    switch (palBbBleCb.gpEvent) {
    default:
    case GP_NONE:
        break;
    case GP_TX_START:
        BB_LED_ON(BB_LED_PIN_TX);
        break;
    case GP_RX_START:
        BB_LED_ON(BB_LED_PIN_RX);
        break;
    }

    BB_LED_OFF(BB_LED_PIN_ISR);
#endif
}

void BTLE_SFD_DET_IRQHandler(void)
{
    BB_LED_ON(BB_LED_PIN_ISR);

    BB_LED_OFF(BB_LED_PIN_ISR);
}

void BTLE_RX_ENG_DET_IRQHandler(void)
{
    BB_LED_ON(BB_LED_PIN_ISR);

    BB_LED_OFF(BB_LED_PIN_ISR);
}

void BTLE_RFFE_SPIM_IRQHandler(void)
{
    BB_LED_ON(BB_LED_PIN_ISR);

    BB_LED_OFF(BB_LED_PIN_ISR);
}

void BTLE_CFO_IRQHandler(void)
{
    BB_LED_ON(BB_LED_PIN_ISR);

    BB_LED_OFF(BB_LED_PIN_ISR);
}

void BTLE_SIG_DET_IRQHandler(void)
{
    BB_LED_ON(BB_LED_PIN_ISR);

    BB_LED_OFF(BB_LED_PIN_ISR);
}

void BTLE_AGC_EVENT_IRQHandler(void)
{
    BB_LED_ON(BB_LED_PIN_ISR);

    BB_LED_OFF(BB_LED_PIN_ISR);
}

void BTLE_TX_AES_IRQHandler(void)
{
    BB_LED_ON(BB_LED_PIN_ISR);

    BB_LED_OFF(BB_LED_PIN_ISR);
}

void BTLE_RX_AES_IRQHandler(void)
{
    BB_LED_ON(BB_LED_PIN_ISR);

    BB_LED_OFF(BB_LED_PIN_ISR);
}

void BTLE_INV_APB_ADDR_IRQHandler(void)
{
    BB_LED_ON(BB_LED_PIN_ISR);

    BB_LED_OFF(BB_LED_PIN_ISR);
}

void BTLE_IQ_DATA_VALID_IRQHandler(void)
{
    BB_LED_ON(BB_LED_PIN_ISR);

    BB_LED_OFF(BB_LED_PIN_ISR);
}

/*************************************************************************************************/
/*!
 *  \brief      Low power operation.
 *
 *  \return     None.
 *
 *  \note       Called by upper baseband code.
 */
/*************************************************************************************************/
void PalBbBleLowPower(void) {}

/*************************************************************************************************/
/*!
 *  \brief  Enable AES cipher block.
 *
 *  \param  pEnc        Encryption parameters.
 *  \param  id          Context ID.
 *  \param  localDir    Direction bit of local device (0=slave, 1=master).
 *
 *  \return None.
 */
/*************************************************************************************************/
void __attribute__((weak)) PalCryptoAesEnable(PalCryptoEnc_t *pEnc, uint8_t id, uint8_t dir)
{
    palBbBleCb.palBbBleEnc = *pEnc;
    palBbBleCb.txPktCntValid = FALSE;
    palBbBleCb.rxPktCntValid = FALSE;

    /* Use Local Key */
    MXC_DBB_CTRL->aes_ctrl |= 1;

    /* copy the key to the AES block */
    memcpy((void *)&(MXC_DBB_CTRL->aes_key), (void *)palBbBleCb.palBbBleEnc.sk, 16); // NOLINT

    /* Write IV and constant into AES counter block */
    static const uint16_t AES_MAGIC_VALUE = 0x49;
    MXC_DBB_CTRL->aes_ctr_blk[0] = AES_MAGIC_VALUE;

    /* Nonce is 13bytes long and IV is 8bytes long.  To left justify in */
    /* the register, We start with 6 offset occupying 6-13 octets (8 bytes
     * long). */
    memcpy((void *)&(MXC_DBB_CTRL->aes_ctr_blk[6]), (void *)palBbBleCb.palBbBleEnc.iv, 8); // NOLINT
}

/*************************************************************************************************/
/*!
 *  \brief  Disable AES cipher block.
 *
 *  \param  pEnc        Encryption parameters.
 *  \param  id          Context ID.
 *  \param  localDir    Direction bit of local device (0=slave, 1=master).
 *
 *  \return None.
 */
/*************************************************************************************************/
void __attribute__((weak)) PalCryptoAesDisable(PalCryptoEnc_t *pEnc, uint8_t id, uint8_t dir)
{
    palBbBleCb.palBbBleEnc = *pEnc;
    palBbBleCb.txPktCntValid = FALSE;
    palBbBleCb.rxPktCntValid = FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief  Set the encrypt nonce packet counter field.
 *
 *  \param  pEnc        Encryption parameters.
 *  \param  pktCnt      Counter value.
 *
 *  \return None.
 */
/*************************************************************************************************/
void PalCryptoSetEncryptPacketCount(PalCryptoEnc_t *pEnc, uint64_t pktCnt)
{
    palBbBleCb.palBbBleEnc = *pEnc;
    palBbBleCb.txPktCnt = pktCnt;
    palBbBleCb.txPktCntValid = TRUE;
}

/*************************************************************************************************/
/*!
 *  \brief  Set the decrypt nonce packet counter field.
 *
 *  \param  pEnc        Encryption parameters.
 *  \param  pktCnt      Counter value.
 *
 *  \return None.
 */
/*************************************************************************************************/
void PalCryptoSetDecryptPacketCount(PalCryptoEnc_t *pEnc, uint64_t pktCnt)
{
    palBbBleCb.palBbBleEnc = *pEnc;
    palBbBleCb.rxPktCnt = pktCnt;
    palBbBleCb.rxPktCntValid = TRUE;
}
