/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Maxim Custom service server.
 *
 *  Copyright (c) 2012-2018 Arm Ltd. All Rights Reserved.
 *
 *  Copyright (c) 2022-2023 Analog Devices, Inc.
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

#ifndef EXAMPLES_MAX32665_BLUETOOTH_BLE_MCS_PROFILES_MCS_MCS_API_H_
#define EXAMPLES_MAX32665_BLUETOOTH_BLE_MCS_PROFILES_MCS_MCS_API_H_

#include "wsf_types.h"
#include "att_api.h"
#include "app_api.h"
#include "svc_mcs.h"
#include "board.h"
#include "led.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
  Data Types
**************************************************************************************************/
/*! \brief Maxim custom service configurable parameters */
typedef struct {
    wsfTimerTicks_t period; /*!< \brief Button timer expiration period in seconds */
    uint16_t count; /*!< \brief Perform button after this many timer periods */
    uint8_t threshold; /*!< \brief Send button notification to peer when below this level. */
} mcsCfg_t;

/*************************************************************************************************/
/*!
 *  \brief  ATTS write callback for maxim custom service Use this function as a parameter
 *          to SvcMcsCbackRegister().
 *
 *  \param  connId      DM connection identifier.
 *  \param  handle      ATT handle.
 *  \param  operation   ATT operation.
 *  \param  offset      Write offset.
 *  \param  len         Write length.
 *  \param  pValue      Value to write.
 *  \param  pAttr       Attribute to write.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
uint8_t McsWriteCback(dmConnId_t connId, uint16_t handle, uint8_t operation, uint16_t offset,
                      uint16_t len, uint8_t *pValue, attsAttr_t *pAttr);

/*************************************************************************************************/
/*!
 *  \brief  Setting characteristic value and send the button value to the peer device.
 *
 *  \param  features       The button value.
 *
 *  \return None.
 */
/*************************************************************************************************/
void McsSetFeatures(uint8_t features);

/*************************************************************************************************/
/*!
 *  \brief  Initialize the mcs server.
 *
 *  \param  handlerId    WSF handler ID of the application using this service.
 *  \param  pCfg         mcs configurable parameters.
 *
 *  \return None.
 */
/*************************************************************************************************/
void McsInit(wsfHandlerId_t handlerId, mcsCfg_t *pCfg);

/*************************************************************************************************/
/*!
 *  \brief  Start periodic mcs button state read.  This function starts a timer to perform
 *          periodic button read.
 *
 *  \param  connId      DM connection identifier.
 *  \param  timerEvt    WSF event designated by the application for the timer.
 *  \param  mcsCccIdx   Index of mcs level CCC descriptor in CCC descriptor handle table.
 *  \param  btnState    State of the push button.
 *
 *  \return None.
 */
/*************************************************************************************************/
void McsButtonCheckStart(dmConnId_t connId, uint8_t timerEvt, uint8_t mcsCccIdx, uint8_t btnState);

/*************************************************************************************************/
/*!
 *  \brief  Stop periodic mcs button read.
 *
 *  \param  connId      DM connection identifier.
 *
 *  \return None.
 */
/*************************************************************************************************/
void McsButtonCheckStop(dmConnId_t connId);

#ifdef __cplusplus
};
#endif

#endif // EXAMPLES_MAX32665_BLUETOOTH_BLE_MCS_PROFILES_MCS_MCS_API_H_
