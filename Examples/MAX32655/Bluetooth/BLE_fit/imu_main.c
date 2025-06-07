/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  IMU sensor.
 *
 *  Copyright (c) 2012-2018 Arm Ltd. All Rights Reserved.
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

#include <string.h>
#include "wsf_types.h"
#include "wsf_assert.h"
#include "wsf_buf.h"
#include "wsf_trace.h"
#include "util/bstream.h"
#include "att_api.h"
#include "svc_ch.h"
#include "svc_imu.h"
#include "app_api.h"
#include "app_hw.h"
#include "imu_api.h"

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! \brief Connection control block */
typedef struct
{
  dmConnId_t    connId;               /*! \brief Connection ID */
  bool_t        imuToSend;            /*! \brief IMU data ready to be sent on this channel */
} imuConn_t;

/*! \brief Control block */
static struct
{
  imuConn_t     conn[DM_CONN_MAX];    /* \brief connection control block */
  wsfTimer_t    measTimer;            /* \brief periodic measurement timer */
  imuData_t     data;                  /* \brief IMU measurement */
  imuCfg_t      cfg;                  /* \brief configurable parameters */
  bool_t        txReady;              /* \brief TRUE if ready to send notifications */
  uint8_t       flags;                /* \brief heart rate measurement flags */
} imuCb;

/**************************************************************************************************
  Connections
**************************************************************************************************/
static bool_t ImuNoConnActive(void)
{
  imuConn_t    *pConn = imuCb.conn;
  uint8_t       i;

  for (i = 0; i < DM_CONN_MAX; i++, pConn++)
  {
    if (pConn->connId != DM_CONN_ID_NONE)
    {
      return FALSE;
    }
  }
  return TRUE;
}

static void imuSetupToSend(void)
{
  imuConn_t    *pConn = imuCb.conn;
  uint8_t       i;

  for (i = 0; i < DM_CONN_MAX; i++, pConn++)
  {
    if (pConn->connId != DM_CONN_ID_NONE)
    {
      pConn->imuToSend = TRUE;
    }
  }
}

static imuConn_t *imuFindNextToSend(uint8_t cccIdx)
{
  imuConn_t     *pConn = imuCb.conn;
  uint8_t       i;

  for (i = 0; i < DM_CONN_MAX; i++, pConn++)
  {
    if (pConn->connId != DM_CONN_ID_NONE && pConn->imuToSend)
    {
      if (AttsCccEnabled(pConn->connId, cccIdx))
      {
        return pConn;
      }
    }
  }
  return NULL;
}

static uint8_t imuBuild(dmConnId_t connId, uint8_t **pBuf, imuData_t *pLatestImuData)
{
  uint8_t   *pImuData;
#if DATA_LEN_8
  uint8_t   len = 8; /* TODO: Try to make this dynamic with sizeof and likely Start with 9 => 1 byte for identity and 8 bytes of data */
#else
  uint8_t   len = 2; /* TODO: 2 for demo purpose */
#endif // DATA_LEN_8
  uint8_t   maxLen = AttGetMtu(connId) - ATT_VALUE_NTF_LEN;

  /* Adjust length if necessary */
  if (len > maxLen)
  {
    len = maxLen;
  }

  /* Allocate buffer */
  if ((*pBuf = (uint8_t *)WsfBufAlloc(len)) != NULL)
  {
    /* Add data to buffer */
    pImuData = *pBuf;
    /* qX, qY, qZ, qW */
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu1.qX);
#if DATA_LEN_8
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu1.qY);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu1.qZ);
    UINT16_TO_BSTREAM(pImuData, (uint16_t)pLatestImuData->imu1.qW);
#endif // DATA_LEN_8
    /* return length */
    return (uint8_t)(pImuData - *pBuf);
  }
  return 0;
}

// TODO: remove hrm from name
static void imuSendHrmNtf(dmConnId_t connId)
{
  uint8_t *pBuf;
  uint8_t len;

  /* Build heart rate measurement characteristic */
  if ((len = imuBuild(connId, &pBuf, &imuCb.data)) > 0)
  {
    /* Send notification */
    AttsHandleValueNtf(connId, IMU_GYRO_VAL_HDL, len, pBuf);

    /* Free allocated buffer */
    WsfBufFree(pBuf);
  }
}

static void imuConnOpen(dmEvt_t *pMsg)
{
  imuCb.txReady = TRUE;
}

static void imuHandleValueCnf(attEvt_t *pMsg)
{
  imuConn_t  *pConn;

  if (pMsg->hdr.status == ATT_SUCCESS && pMsg->handle == IMU_GYRO_VAL_HDL)
  {
    imuCb.txReady = TRUE;

    /* find next connection to send (note ccc idx is stored in timer status) */
    if ((pConn = imuFindNextToSend(imuCb.measTimer.msg.status)) != NULL)
    {
      imuSendHrmNtf(pConn->connId);
      imuCb.txReady = FALSE;
      pConn->imuToSend = FALSE;
    }
  }
}

// TODO: Meet's point
void AppHwImuRead(imuData_t *pImu)
{
  ++(pImu->imu1.qX); // Simulated quaternion X value
#if DATA_LEN_8
  ++(pImu->imu1.qY); // Simulated quaternion Y value
  ++(pImu->imu1.qZ); // Simulated quaternion Z value
  ++(pImu->imu1.qW); // Simulated quaternion W value
  APP_TRACE_INFO4("qX = %x qY = %x, qZ = %x, qW = %x\n", pImu->imu1.qX, pImu->imu1.qY, pImu->imu1.qZ, pImu->imu1.qW);
#else
  APP_TRACE_INFO1("qX = %x", pImu->imu1.qX);
#endif // DATA_LEN_8
}

void ImuMeasTimerExp(wsfMsgHdr_t *pMsg)
{
  imuConn_t *pConn;
  /* if there are active connections */
  if (ImuNoConnActive() == FALSE)
  {
    /* set up imu measurement to be sent on all connections */
    imuSetupToSend();

    /* read IMU measurement sensor data */
    AppHwImuRead(&imuCb.data);

    /* if ready to send measurements */
    if (imuCb.txReady)
    {
      /* find next connection to send (note ccc idx is stored in timer status) */
      if ((pConn = imuFindNextToSend(pMsg->status)) != NULL)
      {
        imuSendHrmNtf(pConn->connId);
        imuCb.txReady = FALSE;
        pConn->imuToSend = FALSE;
      }
    }
    else
    {
      APP_TRACE_ERR0("txReady is false\n");
    }
    /* restart timer */
    WsfTimerStartMs(&imuCb.measTimer, imuCb.cfg.period);
  }
  else
  {
    APP_TRACE_INFO0("No connection active\n");
  }
}

void ImuInit(wsfHandlerId_t handlerId, imuCfg_t *pCfg)
{
    imuCb.measTimer.handlerId = handlerId;
    imuCb.cfg = *pCfg;
}

void ImuMeasStart(dmConnId_t connId, uint8_t timerEvt, uint8_t hrmCccIdx)
{
  /* if this is first connection */
  if (ImuNoConnActive())
  {
    APP_TRACE_INFO3("ImuMeasStart: conn %u timer for %u ms with event %d\n", connId, imuCb.cfg.period, timerEvt);
    /* initialize control block */
    imuCb.measTimer.msg.event = timerEvt;
    imuCb.measTimer.msg.status = hrmCccIdx;

    /* start timer */
    WsfTimerStartMs(&imuCb.measTimer, imuCb.cfg.period);
  }

  /* set conn id */
  imuCb.conn[connId - 1].connId = connId;
}

void ImuMeasStop(dmConnId_t connId)
{
  /* clear connection */
  imuCb.conn[connId - 1].connId = DM_CONN_ID_NONE;
  imuCb.conn[connId - 1].imuToSend = FALSE;

  /* if no remaining connections */
  if (ImuNoConnActive())
  {
    /* stop timer */
    WsfTimerStop(&imuCb.measTimer);
  }
}

void ImuProcMsg(wsfMsgHdr_t *pMsg)
{
  // APP_TRACE_INFO1("ImuProcMsg event %u", pMsg->event);
  if (pMsg->event == DM_CONN_OPEN_IND)
  {
    // APP_TRACE_INFO0("imuConnOpen");
    imuConnOpen((dmEvt_t *) pMsg);
  }
  else if (pMsg->event == ATTS_HANDLE_VALUE_CNF)
  {
    // APP_TRACE_INFO0("imuHandleValueCnf");
    imuHandleValueCnf((attEvt_t *) pMsg);
  }
  else if (pMsg->event == imuCb.measTimer.msg.event)
  {
    // APP_TRACE_INFO0("ImuMeasTimerExp");
    ImuMeasTimerExp(pMsg);
  }
}

// TODO: Do we need to implement this?
// uint8_t ImuWriteCback(dmConnId_t connId, uint16_t handle, uint8_t operation,
//     uint16_t offset, uint16_t len, uint8_t *pValue, attsAttr_t *pAttr);