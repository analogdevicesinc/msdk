/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Maxim Custom service server.
 *
 *  Copyright (c) 2012-2018 Arm Ltd. All Rights Reserved.
 *  ARM Ltd. confidential and proprietary.
 *
 *  IMPORTANT.  Your use of this file is governed by a Software License Agreement
 *  ("Agreement") that must be accepted in order to download or otherwise receive a
 *  copy of this file.  You may not use or copy this file for any purpose other than
 *  as described in the Agreement.  If you do not agree to all of the terms of the
 *  Agreement do not use this file and delete all copies in your possession or control;
 *  if you do not have a copy of the Agreement, you must contact ARM Ltd. prior
 *  to any use, copying or further distribution of this software.
 */
/*************************************************************************************************/

#include "app_ui.h"
#include "mcs_api.h"
#include "pal_led.h"
#include <stdbool.h>

/**************************************************************************************************
  Macros
**************************************************************************************************/

#ifndef LED_RED
#define LED_RED 0
#endif

#ifndef LED_GREEN
#define LED_GREEN 1
#endif

#ifndef LED_BLUE
#define LED_BLUE LED_GREEN
#endif

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! \brief Connection control block */
typedef struct {
    dmConnId_t connId; /*! \brief Connection ID */
    bool_t mcsToSend; /*! \brief mcs measurement ready to be sent on this channel */
    uint8_t sentMcsBtnState; /*! \brief value of last sent mcs button state */
} mcsConn_t;

/*! \brief Control block */
static struct {
    mcsConn_t conn[DM_CONN_MAX]; /*! \brief connection control block */
    wsfTimer_t btnStateChkTimer; /*! \brief periodic check timer */
    mcsCfg_t cfg; /*! \brief configurable parameters */
    uint16_t currCount; /*! \brief current measurement period count */
    bool_t txReady; /*! \brief TRUE if ready to send notifications */
    uint8_t btnState; /*! \brief value of last button state */
} mcsCb;

/*************************************************************************************************/
/*!
 *  \brief  Return TRUE if no connections with active measurements.
 *
 *  \return TRUE if no connections active.
 */
/*************************************************************************************************/
static bool_t mcsNoConnActive(void)
{
    mcsConn_t* pConn = mcsCb.conn;
    uint8_t i;

    for (i = 0; i < DM_CONN_MAX; i++, pConn++) {
        if (pConn->connId != DM_CONN_ID_NONE) {
            return FALSE;
        }
    }
    return TRUE;
}

/*************************************************************************************************/
/*!
 *  \brief  ATTS write callback for maxim custom service.  Use this function as a parameter
 *          to SvcMcsCbackRegister().
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
uint8_t McsWriteCback(dmConnId_t connId, uint16_t handle, uint8_t operation, uint16_t offset,
    uint16_t len, uint8_t* pValue, attsAttr_t* pAttr)
{
    AttsSetAttr(handle, sizeof(*pValue), (uint8_t*)pValue);
    /* Turn LED on if non-zero value was written */
    bool on = *pValue != 0;

    /* Get LED ID */
    uint8_t ch = 0;
    switch (handle) {
    case MCS_R_HDL:
        ch = LED_RED;
        break;
    case MCS_B_HDL:
        ch = LED_BLUE;
        break;
    case MCS_G_HDL:
        ch = LED_GREEN;
        break;
    }

    if (on)
        LED_On(ch);
    else
        LED_Off(ch);
    return ATT_SUCCESS;
}

/*************************************************************************************************/

/*!
 *  \brief  Setting characteristic value and send the button value to the peer device
 *
 *  \return None.
 */
/*************************************************************************************************/
void McsSetFeatures(uint8_t features)
{
    AttsSetAttr(MCS_BUTTON_HDL, sizeof(features),
        (uint8_t*)&features); /*Setting mcsButtonVal characteristic value */
    dmConnId_t connId = AppConnIsOpen(); /*Getting connected */
    if (connId != DM_CONN_ID_NONE) {
        AttsHandleValueNtf(
            connId, MCS_BUTTON_HDL, sizeof(features), (uint8_t*)&features); /*Send notify */
    }
}

/*************************************************************************************************/
/*!
 *  \brief  Initialize the mcs server.
 *
 *  \param  handerId    WSF handler ID of the application using this service.
 *  \param  pCfg        mcs configurable parameters.
 *
 *  \return None.
 */
/*************************************************************************************************/
void McsInit(wsfHandlerId_t handlerId, mcsCfg_t* pCfg)
{
    mcsCb.btnStateChkTimer.handlerId = handlerId;
    mcsCb.cfg = *pCfg;

    /* De-init the PAL LEDs so we can control them here */
    PalLedDeInit();
}

/*************************************************************************************************/
/*!
 *  \brief  Start periodic mcs button state check.  This function starts a timer to perform
 *          periodic button checks.
 *
 *  \param  connId      DM connection identifier.
 *  \param  timerEvt    WSF event designated by the application for the timer.
 *  \param  mcsCccIdx   Index of mcs button state CCC descriptor in CCC descriptor handle table.
 *
 *  \return None.
 */
/*************************************************************************************************/
void McsButtonCheckStart(dmConnId_t connId, uint8_t timerEvt, uint8_t mcsCccIdx, uint8_t btnState)
{
    /* if this is first connection */
    if (mcsNoConnActive()) {
        /* initialize control block */
        mcsCb.btnStateChkTimer.msg.event = timerEvt;
        mcsCb.btnStateChkTimer.msg.status = mcsCccIdx;
        mcsCb.btnState = btnState;
        mcsCb.currCount = mcsCb.cfg.count;

        /* start timer */
        WsfTimerStartSec(&mcsCb.btnStateChkTimer, mcsCb.cfg.period);
    }

    /* set conn id and last sent button level */
    mcsCb.conn[connId - 1].connId = connId;
    mcsCb.conn[connId - 1].sentMcsBtnState = btnState;
}

/*************************************************************************************************/
/*!
 *  \brief  Stop periodic button state check.
 *
 *  \param  connId      DM connection identifier.
 *
 *  \return None.
 */
/*************************************************************************************************/
void McsButtonCheckStop(dmConnId_t connId)
{
    /* clear connection */
    mcsCb.conn[connId - 1].connId = DM_CONN_ID_NONE;
    mcsCb.conn[connId - 1].mcsToSend = FALSE;

    /* if no remaining connections */
    if (mcsNoConnActive()) {
        /* stop timer */
        WsfTimerStop(&mcsCb.btnStateChkTimer);
    }
}
