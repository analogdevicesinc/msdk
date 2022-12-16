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
 **************************************************************************** */

/**
 * @file    pal_bb.c
 * @brief   Platform Adaption Layer for Baseband.
 * @details
 */

/**************************************************************************************************
  Includes
**************************************************************************************************/

#include <string.h>
#include "pal_types.h"
#include "pal_bb.h"
#include "pal_bb_ble.h"

#include "gcr_regs.h"
#include "flc_regs.h"

#include "dbb_registers.h"

#include "pal_bb_phy.h"
#include "pal_bb_afe.h"

#include "pal_bb_cfg.h"
#include "pal_bb_dbb.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! \brief      PalBbAfe control block. */
typedef struct {
    uint32_t enableCount;
} palBbCb_t;

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

static palBbCb_t palBbCb;

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/**************************************************************************************************
  Functions
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief      Initialize the baseband driver.
 *
 *  \return     None.
 *
 *  One-time initialization of baseband resources. This routine can be used to
 *  setup baseband resources, load RF trim parameters and execute RF
 * calibrations and seed the random number generator.
 *
 *  This routine should block until the BB hardware is completely initialized.
 */
/*************************************************************************************************/
void PalBbInit(void)
{
    memset(&palBbCb, 0, sizeof(palBbCb_t)); // NOLINT

    PalBbAfeInit();

    palBbCb.enableCount++;

    PalBbAfeEnable();

    /* Wait for the 32 MHz clock to power up */
    while (!(MXC_GCR->clkcn & MXC_F_GCR_CLKCN_X32M_RDY)) {}

    /* Disable BTLE clock gate */
    MXC_GCR->perckcn1 &= ~(MXC_F_GCR_PERCKCN1_BTLED);

    PalBbAfeSetTxPower(BB_DEFAULT_TX_POWER);

    PalBbDbbInit();

    PalBbPhyInit(BB_PHY_BLE_1M, BB_PHY_OPTIONS_DEFAULT);

    PalBbDisable();
}

/*************************************************************************************************/
/*!
 *  \brief      Enable the BB hardware.
 *
 *  \return     None.
 *
 *  This routine brings the BB hardware out of low power (enable power and
 *  clocks) just before a first BB operation is executed.
 */
/*************************************************************************************************/
void PalBbEnable(void)
{
    palBbCb.enableCount++;

    PalBbAfeEnable();

    /* Disable BTLE clock gate */
    MXC_GCR->perckcn1 &= ~(MXC_F_GCR_PERCKCN1_BTLED);

    /* Determine if we went to DS and need to do a restore */
    if (MXC_DBB_RFFE->rffe_ifc_rffe_version == 0) {
        PalBbDbbRestore();
    }

    MXC_DBB_CTRL->dbb_ctrl_rst |= 1;

    PalBbBleEnable();
}

/*************************************************************************************************/
/*!
 *  \brief      Disable the BB hardware.
 *
 *  \return     None.
 *
 *  This routine signals the BB hardware to go into low power (disable power and
 *  clocks) after all BB operations have been disabled.
 */
/*************************************************************************************************/
void PalBbDisable(void)
{
    if (palBbCb.enableCount) {
        palBbCb.enableCount--;
    }

    PalBbBleDisable();

    /* Stop the control of the TX and RX to be sure both blocks are in the
     * correct state */
    MXC_DBB_CTRL->tx_ctrl_cmd |= MXC_F_DBB_CTRL_TX_CTRL_CMD_STOP;
    MXC_DBB_CTRL->rx_ctrl_cmd |= MXC_F_DBB_CTRL_RX_CTRL_CMD_STOP;

    PalBbAfeDisable();

    /* Reset the DBB to obtain a known state */
    MXC_DBB_CTRL->dbb_ctrl_rst |= 1;

    /* Enable BTLE clock gate */
    MXC_GCR->perckcn1 |= MXC_F_GCR_PERCKCN1_BTLED;
}

/*************************************************************************************************/
/*!
 *  \brief      Load BB timing configuration.
 *
 *  \param      pCfg                Return configuration values.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalBbLoadCfg(PalBbCfg_t *pCfg)
{
    // Sleep clock has 20 ppm accuracy
    static const uint16_t clkPpm = 20;

    // Scheduler setup time needs to be at least 200 us.
    static const uint16_t schSetupDelayUsec = 200;

    // The baseband timer rolls over after 32 bits;
    static const uint32_t BbTimerBoundaryUsec = 0xFFFFFFFF;

    pCfg->clkPpm = clkPpm;
    pCfg->rfSetupDelayUsec = BB_RF_SETUP_DELAY_US;
    pCfg->maxScanPeriodMsec = BB_MAX_SCAN_PERIOD_MS;
    pCfg->schSetupDelayUsec = schSetupDelayUsec;
    pCfg->BbTimerBoundaryUsec = BbTimerBoundaryUsec;
}

/*************************************************************************************************/
/*!
 *  \brief      Get the current BB clock value in microseconds.
 *
 *  \return     Current BB clock value, units are microseconds.
 *
 *  This routine reads the current value from the BB clock and returns its
 *  value.
 */
/*************************************************************************************************/
uint32_t PalBbGetCurrentTime(void)
{
    volatile uint32_t ts0, ts1;

    /* Prevent reading of the DBB registers when disabled */
    if (!palBbCb.enableCount) {
        return 0;
    }

    /* Prevent glitched timer values */
    do {
        ts0 = MXC_DBB_CTRL->event_timing_cntr_val;
        ts1 = MXC_DBB_CTRL->event_timing_cntr_val;
    } while (ts0 != ts1);

    return ts0;
}

/*************************************************************************************************/
/*!
 *  \brief      Get the current FRC time.
 *
 *  \param      pTime   Pointer to return the current time.
 *
 *  \return     TRUE if time is valid, FALSE otherwise.
 *
 *  Get the current FRC time.
 *
 *  \note       FRC is limited to the same bit-width as the BB clock. Return
 *  value is available only when the BB is active.
 */
/*************************************************************************************************/
bool_t PalBbGetTimestamp(uint32_t *pTime)
{
    if (palBbCb.enableCount) {
        *pTime = PalBbGetCurrentTime();
        return TRUE;
    }

    return FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief      Called to register a protocol's Radio and Timer IRQ callback
 *  functions.
 *
 *  \param      protId      Protocol ID.
 *  \param      timerCback  Timer IRQ callback.
 *  \param      radioCback  Timer IRQ callback.
 *
 *  \return     None.
 */
/*************************************************************************************************/
// void PalBbRegisterProtIrq(uint8_t protId, bbDrvIrqCback_t timerCback, bbDrvIrqCback_t radioCback)
// {}

/*************************************************************************************************/
/*!
 *  \brief      Set protocol ID.
 *
 *  \param      protId      Protocol ID.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalBbSetProtId(uint8_t protId) {}
