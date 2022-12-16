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
 * @file    pal_bb_ble_rf.c
 * @brief   Platform Adaption Layer for Baseband Bluetooth RF functions.
 * @details
 */

/**************************************************************************************************
  Includes
**************************************************************************************************/

#include "pal_types.h"
#include "pal_bb_afe.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! \brief      Minimum RF path compensation (in 0.1dBm units). */
#define BB_BLE_MIN_PATH_COMP (-1280)

/*! \brief      Minimum RF path compensation (in 1dBm units). */
#define BB_BLE_MIN_PATH_COMP_DBM (-128)

/*! \brief      Maximum RF path compensation (in 0.1dBm units). */
#define BB_BLE_MAX_PATH_COMP 1280

/*! \brief      Binary divide with 10 divisor (n[max]=0xFFFFFFFF). */
#define BB_BLE_MATH_DIV_10(n) ((uint32_t)(((uint64_t)(n)*UINT64_C(419431)) >> 22))

/**************************************************************************************************
  Constants
**************************************************************************************************/

static const int8_t bbBleMinTxPwr = -10; /* -30dBm */
static const int8_t bbBleMaxTxPwr = 4; /*  +4dBm */

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! \brief      Current Tx path compensation value. */
int16_t bbBleTxPathComp = BB_BLE_MIN_PATH_COMP;

/*! \brief      Current Rx path compensation value. */
int16_t bbBleRxPathComp = BB_BLE_MIN_PATH_COMP;

/*************************************************************************************************/
/*!
 *  \brief      Get receive RF path compensation.
 *
 *  \return     Transmit RF path compensation (in 1-dBm units).
 */
/*************************************************************************************************/
int8_t PalRadioGetRxRfPathComp(void)
{
    uint16_t pathCompUnsigned = (uint16_t)(bbBleRxPathComp - BB_BLE_MIN_PATH_COMP);

    int8_t retval = BB_BLE_MATH_DIV_10(pathCompUnsigned) + BB_BLE_MIN_PATH_COMP_DBM; // NOLINT

    return retval;
}

/*************************************************************************************************/
/*!
 *  \brief      Initialize RF path compensation.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalRadioInitPathComp(void)
{
    bbBleTxPathComp = 0;
    bbBleRxPathComp = 0;
}

/*************************************************************************************************/
/*!
 *  \brief      Get supported transmit power.
 *
 *  \param      pMinTxPwr   Return buffer for minimum transmit power (expressed
 *  in 1dBm units). \param      pMaxTxPwr   Return buffer for maximum transmit
 *  power (expressed in 1dBm units).
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalRadioGetSupTxPower(int8_t *pMinTxPwr, int8_t *pMaxTxPwr)
{
    *pMinTxPwr = bbBleMinTxPwr;
    *pMaxTxPwr = bbBleMaxTxPwr;
}

/*************************************************************************************************/
/*!
 *  \brief      Get RF path compensation.
 *
 *  \param      pTxPathComp Return buffer for RF transmit path compensation
 *  value (expressed in 0.1dBm units). \param      pRxPathComp Return buffer for
 *  RF receive path compensation value (expressed in 0.1dBm units).
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalRadioReadRfPathComp(int16_t *pTxPathComp, int16_t *pRxPathComp)
{
    *pTxPathComp = bbBleTxPathComp;
    *pRxPathComp = bbBleRxPathComp;
}

/*************************************************************************************************/
/*!
 *  \brief      Set RF path compensation.
 *
 *  \param      txPathComp      RF transmit path compensation value (expressed
 *  in 0.1dBm units). \param      rxPathComp      RF receive path compensation
 *  value (expressed in 0.1dBm units).
 *
 *  \return     TRUE if successful, FALSE otherwise.
 */
/*************************************************************************************************/
bool_t PalRadioWriteRfPathComp(int16_t txPathComp, int16_t rxPathComp)
{
    bbBleTxPathComp = txPathComp;
    bbBleRxPathComp = rxPathComp;

    return TRUE;
}

static int8_t palRadioGetClosestTxPower(int8_t txPwr)
{
    unsigned levelIndex;

    // Initialize to the lowest power setting
    int8_t actualTxPwr = PalBbAfeGetIndexTxPower(0);

    // Find the closest power level
    for (levelIndex = (PalBbAfeGetTxPowerLevels() - 1); levelIndex >= 0; levelIndex--) {
        int8_t currentPower = PalBbAfeGetIndexTxPower(levelIndex);
        if (txPwr >= currentPower) {
            actualTxPwr = currentPower;
            break;
        }
    }

    return actualTxPwr;
}

/*************************************************************************************************/
/*!
 *  \brief      Get the actual Tx power at the antenna (expressed in 1dBm units).
 *
 *  \param      txPwr           Tx power provided by the host (expressed in 1dBm units).
 *  \param      compFlag        Flag to apply Tx path compensation or not.
 *
 *  \return     Actual Tx power at the antenna (expressed in 1dBm units).
 *
 *  Tx path compensation is only used for extended ADV header.
 *  Compensation is not considered when filling in HCI events.
 */
/*************************************************************************************************/
int8_t PalRadioGetActualTxPower(int8_t txPwr, bool_t compFlag)
{
    // Find the closest TxPower setting
    int8_t actualTxPwr = palRadioGetClosestTxPower(txPwr);

    // Adjust for local TX loss
    if (compFlag) {
        actualTxPwr += (int8_t)bbBleTxPathComp; // NOLINT
    }

    return actualTxPwr;
}

/*************************************************************************************************/
/*!
 *  \brief      Request an increase in power.

 *  \param      reqPwr           Requested Power.
 *  \param      delta            Delta
 *
 *  \return     TxPower to be set
 *
 *  If increasing power: the controller will increase one step if possible.
 *  If decreasing power: the controller will only decrease to the ceiling step.
 */
/*************************************************************************************************/
int8_t PalRadioIncreasePower(int8_t reqPwr, int8_t delta)
{
    // Find the closest TxPower setting
    int8_t adjustedPower = reqPwr + delta; // NOLINT
    int8_t actualTxPwr = palRadioGetClosestTxPower(adjustedPower);

    return actualTxPwr;
}

/*************************************************************************************************/
/*!
 *  \brief      Get the next acceptable power reduction step.
 *
 *  \param      txPwr           Tx Power(expressed in 1dBm units).
 *
 *  \return     Lowest acceptable power reduction size.
 *
 */
/*************************************************************************************************/
// int8_t PalRadioGetAcceptablePowerReduction(int8_t txPwr) {}
