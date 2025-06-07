/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Heart Rate profile sensor.
 *
 *  Copyright (c) 2011-2018 Arm Ltd. All Rights Reserved.
 *
 *  Copyright (c) 2019 Packetcraft, Inc.
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
#ifndef IMU_API_H
#define IMU_API_H

#include "wsf_timer.h"
#include "att_api.h"
#include "svc_imu.h" // To get DATA_LEN_8 macro

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup IMU_PROFILE
 *  \{ */

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*! \brief Configurable parameters */
typedef struct
{
  wsfTimerTicks_t     period;     /*!< \brief Measurement timer expiration period in ms */
  // TODO: Future use. Can disable certain IMU measurement based on flags below.
  // uint8_t             flags;      /*!< \brief Heart rate measurement flags */
} imuCfg_t;

/*! \brief Single IMU measurement data structure */
typedef struct
{
  int16_t           qX;               /*!< \brief Quanterian X-axis measurement */
#if DATA_LEN_8
  int16_t           qY;               /*!< \brief Quanterian Y-axis measurement */
  int16_t           qZ;               /*!< \brief Quanterian Z-axis measurement */
  int16_t           qW;              /*!< \brief Quanterian X-axis measurement */
#endif // DATA_LEN_8
} singleImuData_t;

/*! \brief All IMU measurement structure */
typedef struct
{
  singleImuData_t         imu1;                 /*!< \brief IMU1 measurement */
  // singleImuData_t         imu2;                 /*!< \brief IMU2 measurement */
  // singleImuData_t         imu3;                 /*!< \brief IMU3 measurement */
  // singleImuData_t         imu4;                 /*!< \brief IMU4 measurement */
  // singleImuData_t         imu5;                 /*!< \brief IMU5 measurement */
  // singleImuData_t         imu6;                 /*!< \brief IMU6 measurement */
} imuData_t;

/*************************************************************************************************/
/*!
 *  \brief  Initialize the IMU profile sensor.
 *
 *  \param  handlerId    WSF handler ID of the application using this service.
 *  \param  pCfg        Configurable parameters.
 *
 *  \return None.
 */
/*************************************************************************************************/
void ImuInit(wsfHandlerId_t handlerId, imuCfg_t *pCfg);

/*************************************************************************************************/
/*!
 *  \brief  Start periodic imu measurement. This function starts a timer to perform
 *          periodic measurements.
 *
 *  \param  connId      DM connection identifier.
 *  \param  timerEvt    WSF event designated by the application for the timer.
 *  \param  hrmCccIdx   Index of IMU CCC descriptor in CCC descriptor handle table.
 *
 *  \return None.
 */
/*************************************************************************************************/
void ImuMeasStart(dmConnId_t connId, uint8_t timerEvt, uint8_t hrmCccIdx);

/*************************************************************************************************/
/*!
 *  \brief  Stop periodic heart rate measurement.
 *
 *  \param  connId      DM connection identifier.
 *
 *  \return None.
 */
/*************************************************************************************************/
void ImuMeasStop(dmConnId_t connId);

/*************************************************************************************************/
/*!
 *  \brief  Process received WSF message.
 *
 *  \param  pMsg     Event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void ImuProcMsg(wsfMsgHdr_t *pMsg);

/*************************************************************************************************/
/*!
 *  \brief  ATTS write callback for Imu service Use this function as a parameter
 *          to SvcHrsCbackRegister().
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
uint8_t ImuWriteCback(dmConnId_t connId, uint16_t handle, uint8_t operation,
                       uint16_t offset, uint16_t len, uint8_t *pValue, attsAttr_t *pAttr);

/*! \} */    /* IMU_PROFILE */

#ifdef __cplusplus
};
#endif

#endif /* IMU_API_H */
