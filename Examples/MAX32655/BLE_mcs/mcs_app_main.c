/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Data transmitter sample application.
 *
 *  Copyright (c) 2012-2019 Arm Ltd. All Rights Reserved.
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

#include <string.h>
#include "wsf_types.h"
#include "util/bstream.h"
#include "wsf_msg.h"
#include "wsf_trace.h"
#include "wsf_buf.h"
#include "wsf_nvm.h"
#include "hci_api.h"
#include "sec_api.h"
#include "dm_api.h"
#include "smp_api.h"
#include "att_api.h"
#include "app_api.h"
#include "app_main.h"
#include "app_db.h"
#include "app_ui.h"
#include "svc_ch.h"
#include "svc_core.h"
#include "svc_mcs.h"
#include "util/calc128.h"
#include "gatt/gatt_api.h"
#include "mcs_app_api.h"
#include "svc_mcs.h"
#include "mcs_api.h"
#include "pal_btn.h"
#include "tmr.h"
#include "pal_btn.h"
#include "tmr.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Button press handling constants */
#define BTN_SHORT_MS                      200
#define BTN_MED_MS                        500
#define BTN_LONG_MS                       1000

#define BTN_1_TMR                         MXC_TMR2
#define BTN_2_TMR                         MXC_TMR3

/*! Enumeration of client characteristic configuration descriptors */
enum {
    MCS_APP_GATT_SC_CCC_IDX,           /*! GATT service, service changed characteristic */
    MCS_APP_BT_CCC_IDX,                /*! MCS characteristic */
    MCS_APP_NUM_CCC_IDX
};

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*! Application message type */
typedef union {
    wsfMsgHdr_t     hdr;
    dmEvt_t         dm;
    attsCccEvt_t    ccc;
    attEvt_t        att;
} mcsMsg_t;

/**************************************************************************************************
  Configurable Parameters
**************************************************************************************************/

/*! configurable parameters for advertising */
static const appAdvCfg_t mcsAppAdvCfg = {
    {30000,     0,     0},                  /*! Advertising durations in ms */
    {   96,  1600,     0}                   /*! Advertising intervals in 0.625 ms units */
};

/*! configurable parameters for slave */
static const appSlaveCfg_t mcsAppSlaveCfg = {
    1,                                      /*! Maximum connections */
};

/*! configurable parameters for security */
static const appSecCfg_t mcsAppSecCfg = {
    DM_AUTH_BOND_FLAG | DM_AUTH_SC_FLAG,    /*! Authentication and bonding flags */
    DM_KEY_DIST_IRK,                        /*! Initiator key distribution flags */
    DM_KEY_DIST_LTK | DM_KEY_DIST_IRK,      /*! Responder key distribution flags */
    FALSE,                                  /*! TRUE if Out-of-band pairing data is present */
    FALSE                                   /*! TRUE to initiate security upon connection */
};

/*! TRUE if Out-of-band pairing data is to be sent */
static const bool_t mcsAppSendOobData = FALSE;

/*! SMP security parameter configuration */
static const smpCfg_t mcsAppSmpCfg = {
    500,                                    /*! 'Repeated attempts' timeout in msec */
    SMP_IO_NO_IN_NO_OUT,                    /*! I/O Capability */
    7,                                      /*! Minimum encryption key length */
    16,                                     /*! Maximum encryption key length */
    1,                                      /*! Attempts to trigger 'repeated attempts' timeout */
    0,                                      /*! Device authentication requirements */
    64000,                                  /*! Maximum repeated attempts timeout in msec */
    64000,                                  /*! Time msec before attemptExp decreases */
    2                                       /*! Repeated attempts multiplier exponent */
};

/*! configurable parameters for connection parameter update */
static const appUpdateCfg_t mcsAppUpdateCfg = {
    0,                                      /*! Connection idle period in ms before attempting
                                              connection parameter update; set to zero to disable */
    640,                                    /*! Minimum connection interval in 1.25ms units */
    800,                                    /*! Maximum connection interval in 1.25ms units */
    3,                                      /*! Connection latency */
    900,                                    /*! Supervision timeout in 10ms units */
    5                                       /*! Number of update attempts before giving up */
};

/*! ATT configurable parameters (increase MTU) */
static const attCfg_t mcsAppAttCfg = {
    15,                               /* ATT server service discovery connection idle timeout in seconds */
    241,                              /* desired ATT MTU */
    ATT_MAX_TRANS_TIMEOUT,            /* transcation timeout in seconds */
    4                                 /* number of queued prepare writes supported by server */
};

/*! local IRK */
static uint8_t localIrk[] = {
    0x95, 0xC8, 0xEE, 0x6F, 0xC5, 0x0D, 0xEF, 0x93, 0x35, 0x4E, 0x7C, 0x57, 0x08, 0xE2, 0xA3, 0x85
};

/**************************************************************************************************
  Advertising Data
**************************************************************************************************/

/*! advertising data, discoverable mode */
static const uint8_t mcsAppAdvDataDisc[] = {
    /*! flags */
    2,                                      /*! length */
    DM_ADV_TYPE_FLAGS,                      /*! AD type */
    DM_FLAG_LE_GENERAL_DISC |               /*! flags */
    DM_FLAG_LE_BREDR_NOT_SUP,

    /*! manufacturer specific data */
    3,                                      /*! length */
    DM_ADV_TYPE_MANUFACTURER,               /*! AD type */
    UINT16_TO_BYTES(HCI_ID_ANALOG)          /*! company ID */
};

/*! scan data, discoverable mode */
static const uint8_t mcsAppScanDataDisc[] = {
    /*! device name */
    4,                                      /*! length */
    DM_ADV_TYPE_LOCAL_NAME,                 /*! AD type */
    'M',
    'C',
    'S',
};

/*! Mcs measurement configuration */
static const mcsCfg_t mcsMcsCfg = {
    2,        /*! Mcs measurement timer expiration period in seconds */
    1,        /*! Perform mcs measurement after this many timer periods */
    3         /*! Send mcs level notification to peer when below this level. */
};

/**************************************************************************************************
  Client Characteristic Configuration Descriptors
**************************************************************************************************/

/*! client characteristic configuration descriptors settings, indexed by above enumeration */
static const attsCccSet_t mcsAppCccSet[MCS_APP_NUM_CCC_IDX] = {
    /* cccd handle          value range               security level */
    {GATT_SC_CH_CCC_HDL,    ATT_CLIENT_CFG_INDICATE,  DM_SEC_LEVEL_NONE},   /* MCS_APP_GATT_SC_CCC_IDX */
    {MCS_BUTTON_CH_CCC_HDL, ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE}    /* MCS_BUTTON_SM_CCC_IDX */
};

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! application control block */
static struct {
    wsfHandlerId_t    handlerId;          /* WSF handler ID */
    appDbHdl_t        resListRestoreHdl;  /*! Resolving List last restored handle */
    bool_t            restoringResList;   /*! Restoring resolving list from NVM */
} mcsAppCb;

/* LESC OOB configuration */
static dmSecLescOobCfg_t *mcsAppOobCfg;

extern void setAdvTxPower(void);
/*************************************************************************************************/
/*!
 *  \brief  Application DM callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void mcsAppDmCback(dmEvt_t *pDmEvt)
{
    dmEvt_t   *pMsg;
    uint16_t  len;

    if (pDmEvt->hdr.event == DM_SEC_ECC_KEY_IND) {
        DmSecSetEccKey(&pDmEvt->eccMsg.data.key);

        /* If the local device sends OOB data. */
        if (mcsAppSendOobData) {
            uint8_t oobLocalRandom[SMP_RAND_LEN];
            SecRand(oobLocalRandom, SMP_RAND_LEN);
            DmSecCalcOobReq(oobLocalRandom, pDmEvt->eccMsg.data.key.pubKey_x);
        }
    } else if (pDmEvt->hdr.event == DM_SEC_CALC_OOB_IND) {
        if (mcsAppOobCfg == NULL) {
            mcsAppOobCfg = WsfBufAlloc(sizeof(dmSecLescOobCfg_t));
        }

        if (mcsAppOobCfg) {
            Calc128Cpy(mcsAppOobCfg->localConfirm, pDmEvt->oobCalcInd.confirm);
            Calc128Cpy(mcsAppOobCfg->localRandom, pDmEvt->oobCalcInd.random);
        }
    } else {
        len = DmSizeOfEvt(pDmEvt);

        if ((pMsg = WsfMsgAlloc(len)) != NULL) {
            memcpy(pMsg, pDmEvt, len);
            WsfMsgSend(mcsAppCb.handlerId, pMsg);
        }
    }
}

/*************************************************************************************************/
/*!
 *  \brief  Application ATT callback.
 *
 *  \param  pEvt    ATT callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void mcsAppAttCback(attEvt_t *pEvt)
{
    attEvt_t  *pMsg;

    if ((pMsg = WsfMsgAlloc(sizeof(attEvt_t) + pEvt->valueLen)) != NULL) {
        memcpy(pMsg, pEvt, sizeof(attEvt_t));
        pMsg->pValue = (uint8_t *)(pMsg + 1);
        memcpy(pMsg->pValue, pEvt->pValue, pEvt->valueLen);
        WsfMsgSend(mcsAppCb.handlerId, pMsg);
    }
}

/*************************************************************************************************/
/*!
 *  \brief  Application ATTS client characteristic configuration callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void mcsAppCccCback(attsCccEvt_t *pEvt)
{
    appDbHdl_t    dbHdl;
    attsCccEvt_t  *pMsg;

    /* If CCC not set from initialization and there's a device record and currently bonded */
    if ((pEvt->handle != ATT_HANDLE_NONE) &&
            ((dbHdl = AppDbGetHdl((dmConnId_t) pEvt->hdr.param)) != APP_DB_HDL_NONE) &&
            AppCheckBonded((dmConnId_t)pEvt->hdr.param)) {
        /* Store value in device database. */
        AppDbSetCccTblValue(dbHdl, pEvt->idx, pEvt->value);
        AppDbNvmStoreCccTbl(dbHdl);
    }

    if ((pMsg = WsfMsgAlloc(sizeof(attsCccEvt_t))) != NULL) {
        memcpy(pMsg, pEvt, sizeof(attsCccEvt_t));
        WsfMsgSend(mcsAppCb.handlerId, pMsg);
    }
}

/*************************************************************************************************/
/*!
 *  \brief  Process CCC state change.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void mcsAppProcCccState(mcsMsg_t *pMsg)
{
    APP_TRACE_INFO3("ccc state ind value:%d handle:%d idx:%d", pMsg->ccc.value, pMsg->ccc.handle, pMsg->ccc.idx);
}

/*************************************************************************************************/
/*!
*
*  \brief  Add device to resolving list.
*
*  \param  dbHdl   Device DB record handle.
*
*  \return None.
*/
/*************************************************************************************************/
static void mcsAppPrivAddDevToResList(appDbHdl_t dbHdl)
{
    dmSecKey_t *pPeerKey;

    /* if peer IRK present */
    if ((pPeerKey = AppDbGetKey(dbHdl, DM_KEY_IRK, NULL)) != NULL) {
        /* set advertising peer address */
        AppSetAdvPeerAddr(pPeerKey->irk.addrType, pPeerKey->irk.bdAddr);
    }
}

/*************************************************************************************************/
/*!
*
*  \brief  Handle remove device from resolving list indication.
*
*  \param  pMsg    Pointer to DM callback event message.
*
*  \return None.
*/
/*************************************************************************************************/
static void mcsAppPrivRemDevFromResListInd(dmEvt_t *pMsg)
{
    if (pMsg->hdr.status == HCI_SUCCESS) {
        if (AppDbGetHdl((dmConnId_t) pMsg->hdr.param) != APP_DB_HDL_NONE) {
            uint8_t addrZeros[BDA_ADDR_LEN] = { 0 };

            /* clear advertising peer address and its type */
            AppSetAdvPeerAddr(HCI_ADDR_TYPE_PUBLIC, addrZeros);
        }
    }
}

/*************************************************************************************************/
/*!
 *  \brief  Perform UI actions on connection close.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void mcsAppClose(dmEvt_t *pMsg)
{
    /* stop mcs measurement */
    McsButtonCheckStop((dmConnId_t) pMsg->hdr.param);
}

/*************************************************************************************************/
/*!
 *
 *  \brief  Display stack version.
 *
 *  \param  version    version number.
 *
 *  \return None.
 */
/*************************************************************************************************/
void mcsAppDisplayStackVersion(const char *pVersion)
{
    APP_TRACE_INFO1("Stack Version: %s", pVersion);
}

/*************************************************************************************************/
/*!
 *  \brief  Set up advertising and other procedures that need to be performed after
 *          device reset.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void mcsAppSetup(dmEvt_t *pMsg)
{
    /* Initialize control information */
    mcsAppCb.restoringResList = FALSE;

    /* set advertising and scan response data for discoverable mode */
    AppAdvSetData(APP_ADV_DATA_DISCOVERABLE, sizeof(mcsAppAdvDataDisc), (uint8_t *) mcsAppAdvDataDisc);
    AppAdvSetData(APP_SCAN_DATA_DISCOVERABLE, sizeof(mcsAppScanDataDisc), (uint8_t *) mcsAppScanDataDisc);

    /* set advertising and scan response data for connectable mode */
    AppAdvSetData(APP_ADV_DATA_CONNECTABLE, sizeof(mcsAppAdvDataDisc), (uint8_t *) mcsAppAdvDataDisc);
    AppAdvSetData(APP_SCAN_DATA_CONNECTABLE, sizeof(mcsAppScanDataDisc), (uint8_t *) mcsAppScanDataDisc);

    /* start advertising; automatically set connectable/discoverable mode and bondable mode */
    AppAdvStart(APP_MODE_AUTO_INIT);
}

/*************************************************************************************************/
/*!
 *  \brief  Begin restoring the resolving list.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void mcsAppRestoreResolvingList(dmEvt_t *pMsg)
{
    /* Restore first device to resolving list in Controller. */
    mcsAppCb.resListRestoreHdl = AppAddNextDevToResList(APP_DB_HDL_NONE);

    if (mcsAppCb.resListRestoreHdl == APP_DB_HDL_NONE) {
        /* No device to restore.  Setup application. */
        mcsAppSetup(pMsg);
    } else {
        mcsAppCb.restoringResList = TRUE;
    }
}

/*************************************************************************************************/
/*!
*  \brief  Handle add device to resolving list indication.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void mcsAppPrivAddDevToResListInd(dmEvt_t *pMsg)
{
    /* Check if in the process of restoring the Device List from NV */
    if (mcsAppCb.restoringResList) {
        /* Set the advertising peer address. */
        mcsAppPrivAddDevToResList(mcsAppCb.resListRestoreHdl);

        /* Retore next device to resolving list in Controller. */
        mcsAppCb.resListRestoreHdl = AppAddNextDevToResList(mcsAppCb.resListRestoreHdl);

        if (mcsAppCb.resListRestoreHdl == APP_DB_HDL_NONE) {
            /* No additional device to restore. Setup application. */
            mcsAppSetup(pMsg);
        }
    } else {
        mcsAppPrivAddDevToResList(AppDbGetHdl((dmConnId_t) pMsg->hdr.param));
    }
}

/*************************************************************************************************/
/*!
 *  \brief  Process messages from the event handler.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void mcsAppProcMsg(dmEvt_t *pMsg)
{
    uint8_t uiEvent = APP_UI_NONE;

    switch(pMsg->hdr.event) {
        case ATTS_CCC_STATE_IND:
            mcsAppProcCccState((mcsMsg_t*)pMsg);
            break;

        case DM_RESET_CMPL_IND:
            AttsCalculateDbHash();
            DmSecGenerateEccKeyReq();
            AppDbNvmReadAll();
            mcsAppRestoreResolvingList(pMsg);
            setAdvTxPower();
            uiEvent = APP_UI_RESET_CMPL;
            break;

        case DM_ADV_START_IND:
            uiEvent = APP_UI_ADV_START;
            break;

        case DM_ADV_STOP_IND:
            uiEvent = APP_UI_ADV_STOP;
            break;

        case DM_CONN_OPEN_IND:
            uiEvent = APP_UI_CONN_OPEN;
            break;

        case DM_CONN_CLOSE_IND:
            APP_TRACE_INFO2("Connection closed status 0x%x, reason 0x%x", pMsg->connClose.status, pMsg->connClose.reason);
            switch (pMsg->connClose.reason) {
                case HCI_ERR_CONN_TIMEOUT:
                    APP_TRACE_INFO0(" TIMEOUT");
                    break;
                case HCI_ERR_LOCAL_TERMINATED:
                    APP_TRACE_INFO0(" LOCAL TERM");
                    break;
                case HCI_ERR_REMOTE_TERMINATED:
                    APP_TRACE_INFO0(" REMOTE TERM");
                    break;
                case HCI_ERR_CONN_FAIL:
                    APP_TRACE_INFO0(" FAIL ESTABLISH");
                    break;
                case HCI_ERR_MIC_FAILURE:
                    APP_TRACE_INFO0(" MIC FAILURE");
                    break;
            }
            mcsAppClose(pMsg);
            uiEvent = APP_UI_CONN_CLOSE;
            break;

        case DM_SEC_PAIR_CMPL_IND:
            DmSecGenerateEccKeyReq();
            AppDbNvmStoreBond(AppDbGetHdl((dmConnId_t) pMsg->hdr.param));
            uiEvent = APP_UI_SEC_PAIR_CMPL;
            break;

        case DM_SEC_PAIR_FAIL_IND:
            DmSecGenerateEccKeyReq();
            uiEvent = APP_UI_SEC_PAIR_FAIL;
            break;

        case DM_SEC_ENCRYPT_IND:
            uiEvent = APP_UI_SEC_ENCRYPT;
            break;

        case DM_SEC_ENCRYPT_FAIL_IND:
            uiEvent = APP_UI_SEC_ENCRYPT_FAIL;
            break;

        case DM_SEC_AUTH_REQ_IND:

            if (pMsg->authReq.oob) {
                dmConnId_t connId = (dmConnId_t) pMsg->hdr.param;

                /* TODO: Perform OOB Exchange with the peer. */


                /* TODO: Fill mcsAppOobCfg peerConfirm and peerRandom with value passed out of band */

                if (mcsAppOobCfg != NULL) {
                    DmSecSetOob(connId, mcsAppOobCfg);
                }

                DmSecAuthRsp(connId, 0, NULL);
            } else {
                AppHandlePasskey(&pMsg->authReq);
            }
            break;

        case DM_SEC_COMPARE_IND:
            AppHandleNumericComparison(&pMsg->cnfInd);
            break;

        case DM_PRIV_ADD_DEV_TO_RES_LIST_IND:
            mcsAppPrivAddDevToResListInd(pMsg);
            break;

        case DM_PRIV_REM_DEV_FROM_RES_LIST_IND:
            mcsAppPrivRemDevFromResListInd(pMsg);
            break;

        case DM_ADV_NEW_ADDR_IND:
            break;

        case DM_PRIV_CLEAR_RES_LIST_IND:
            APP_TRACE_INFO1("Clear resolving list status 0x%02x", pMsg->hdr.status);
            break;

        default:
            break;
    }

    if (uiEvent != APP_UI_NONE) {
        AppUiAction(uiEvent);
    }
}

/*************************************************************************************************/
/*!
 *  \brief  Application handler init function called during system initialization.
 *
 *  \param  handlerID  WSF handler ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
void McsAppHandlerInit(wsfHandlerId_t handlerId)
{
    APP_TRACE_INFO0("McsAppHandlerInit");

    /* store handler ID */
    mcsAppCb.handlerId = handlerId;

    /* Set configuration pointers */
    pAppSlaveCfg = (appSlaveCfg_t *) &mcsAppSlaveCfg;
    pAppAdvCfg = (appAdvCfg_t *) &mcsAppAdvCfg;
    pAppSecCfg = (appSecCfg_t *) &mcsAppSecCfg;
    pAppUpdateCfg = (appUpdateCfg_t *) &mcsAppUpdateCfg;
    pSmpCfg = (smpCfg_t *) &mcsAppSmpCfg;
    pAttCfg = (attCfg_t *) &mcsAppAttCfg;

    /* Initialize application framework */
    AppSlaveInit();
    AppServerInit();

    /* Set IRK for the local device */
    DmSecSetLocalIrk(localIrk);

    /* initialize mcs server */
    McsInit(handlerId, (mcsCfg_t *) &mcsMcsCfg);
}

/*************************************************************************************************/
/*!
 *  \brief  Button press callback.
 *
 *  \param  btn    Button press.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void mcsAppBtnCback(uint8_t btn)
{
    APP_TRACE_INFO1("mcsAppBtnCback; %d", btn);
    McsSetFeatures(btn);
}

/*************************************************************************************************/
/*!
 *  \brief     Platform button press handler.
 *
 *  \param[in] btnId  button ID.
 *  \param[in] state  button state. See ::PalBtnPos_t.
 *
 *  \return    None.
 */
/*************************************************************************************************/
static void btnPressHandler(uint8_t btnId, PalBtnPos_t state)
{
    if(btnId == 1) {
        /* Start/stop button timer */
        if(state == PAL_BTN_POS_UP) {
            /* Button Up, stop the timer, call the action function */
            unsigned btnUs = MXC_TMR_SW_Stop(BTN_1_TMR);
            if((btnUs > 0) && (btnUs < BTN_SHORT_MS*1000)) {
                AppUiBtnTest(APP_UI_BTN_1_SHORT);
            } else if (btnUs < BTN_MED_MS*1000) {
                AppUiBtnTest(APP_UI_BTN_1_MED);
            } else if (btnUs < BTN_LONG_MS*1000) {
                AppUiBtnTest(APP_UI_BTN_1_LONG);
            } else {
                AppUiBtnTest(APP_UI_BTN_1_EX_LONG);
            }
        } else {
            /* Button down, start the timer */
            MXC_TMR_SW_Start(BTN_1_TMR);
        }
    } else if (btnId == 2) {
        /* Start/stop button timer */
        if(state == PAL_BTN_POS_UP) {
            /* Button Up, stop the timer, call the action function */
            unsigned btnUs = MXC_TMR_SW_Stop(BTN_2_TMR);
            if((btnUs > 0) && (btnUs < BTN_SHORT_MS*1000)) {
                AppUiBtnTest(APP_UI_BTN_2_SHORT);
            } else if (btnUs < BTN_MED_MS*1000) {
                AppUiBtnTest(APP_UI_BTN_2_MED);
            } else if (btnUs < BTN_LONG_MS*1000) {
                AppUiBtnTest(APP_UI_BTN_2_LONG);
            } else {
                AppUiBtnTest(APP_UI_BTN_2_EX_LONG);
            }
        } else {
            /* Button down, start the timer */
            MXC_TMR_SW_Start(BTN_2_TMR);
        }
    } else {
        APP_TRACE_ERR0("Undefined button");
    }
}

/*************************************************************************************************/
/*!
 *  \brief  Callback for WSF buffer diagnostic messages.
 *
 *  \param  pInfo     Diagnostics message
 *
 *  \return None.
 */
/*************************************************************************************************/
static void mcsAppWsfBufDiagnostics(WsfBufDiag_t *pInfo)
{
    if (pInfo->type == WSF_BUF_ALLOC_FAILED) {
        APP_TRACE_INFO2("McsApp got WSF Buffer Allocation Failure - Task: %d Len: %d",
                        pInfo->param.alloc.taskId, pInfo->param.alloc.len);
    }
}

/*************************************************************************************************/
/*!
 *  \brief  WSF event handler for application.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void McsAppHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
    if (pMsg != NULL) {
        APP_TRACE_INFO1("McsApp got evt %d", pMsg->event);

        /* process ATT messages */
        if (pMsg->event >= ATT_CBACK_START && pMsg->event <= ATT_CBACK_END) {
            /* process server-related ATT messages */
            AppServerProcAttMsg(pMsg);
        }
        /* process DM messages */
        else if (pMsg->event >= DM_CBACK_START && pMsg->event <= DM_CBACK_END) {
            /* process advertising and connection-related messages */
            AppSlaveProcDmMsg((dmEvt_t *) pMsg);

            /* process security-related messages */
            AppSlaveSecProcDmMsg((dmEvt_t *) pMsg);
        }

        /* perform profile and user interface-related operations */
        mcsAppProcMsg((dmEvt_t *) pMsg);
    }
}

/*************************************************************************************************/
/*!
 *  \brief  Start the application.
 *
 *  \return None.
 */
/*************************************************************************************************/
void McsAppStart(void)
{
    /* Register for stack callbacks */
    DmRegister(mcsAppDmCback);
    DmConnRegister(DM_CLIENT_ID_APP, mcsAppDmCback);
    AttRegister(mcsAppAttCback);
    AttConnRegister(AppServerConnCback);
    AttsCccRegister(MCS_APP_NUM_CCC_IDX, (attsCccSet_t *) mcsAppCccSet, mcsAppCccCback);

    SvcMcsCbackRegister(NULL, McsWriteCback);
    SvcMcsAddGroup();

    /* Initialize attribute server database */
    SvcCoreGattCbackRegister(GattReadCback, GattWriteCback);
    SvcCoreAddGroup();

    /* Set Service Changed CCCD index. */
    GattSetSvcChangedIdx(MCS_APP_GATT_SC_CCC_IDX);

    /* Register for app framework button callbacks */
    AppUiBtnRegister(mcsAppBtnCback);

    WsfNvmInit();

    WsfBufDiagRegister(mcsAppWsfBufDiagnostics);

    PalBtnInit(btnPressHandler);

    /* Reset the device */
    DmDevReset();
}
