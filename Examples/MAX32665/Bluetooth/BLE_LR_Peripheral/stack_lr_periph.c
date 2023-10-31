/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Stack initialization for long range peripheral.
 *
 *  Copyright (c) 2016-2019 Arm Ltd. All Rights Reserved.
 *
 *  Copyright (c) 2019 Packetcraft, Inc.
 *
 *  Paritial Copyright (c) 2023 Analog Devices, Inc.
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

#include "wsf_types.h"
#include "wsf_os.h"
#include "wsf_trace.h"
#include "util/bstream.h"

#include "lr_periph_api.h"

#include "hci_handler.h"
#include "dm_handler.h"
#include "l2c_handler.h"
#include "att_handler.h"
#include "smp_handler.h"
#include "l2c_api.h"
#include "att_api.h"
#include "smp_api.h"
#include "app_api.h"
#include "hci_core.h"
#include "svc_dis.h"
#include "svc_core.h"
#include "sec_api.h"
#include "hci_defs.h"

typedef struct {
    uint8_t advType; /*!< Advertising type. */
    bool_t useLegacyPdu; /*!< Use legacy advertising PDUs. */
    bool_t omitAdvAddr; /*!< Omit advertiser's address from all PDUs. */
    bool_t incTxPwr; /*!< Include TxPower in extended header of advertising PDU. */
    int8_t advTxPwr; /*!< Advertising Tx Power. */
    uint8_t priAdvPhy; /*!< Primary Advertising PHY. */
    uint8_t secAdvMaxSkip; /*!< Secondary Advertising Maximum Skip. */
    uint8_t secAdvPhy; /*!< Secondary Advertising PHY. */
    bool_t scanReqNotifEna; /*!< Scan request notification enable. */
    uint8_t fragPref; /*!< Fragment preference for advertising data. */
    uint8_t advSid; /*!< Advertising Sid. */
    bool_t advDataSet; /*!< TRUE if extended adv data has been set. */
    bool_t scanDataSet; /*!< TRUE if extended scan data has been set. */
    uint8_t connId; /*!< Connection identifier (used by directed advertising). */
} dmExtAdvCb_t;

extern dmExtAdvCb_t dmExtAdvCb[];

/*************************************************************************************************/
/*!
 *  \brief      Initialize stack.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void StackInitFit(void)
{
    wsfHandlerId_t handlerId;

    SecInit();
    SecAesInit();
    SecCmacInit();
    SecEccInit();

    handlerId = WsfOsSetNextHandler(HciHandler);
    HciHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(DmHandler);
    DmDevVsInit(0);
    DmConnInit();
#if BT_VER >= HCI_VER_BT_CORE_SPEC_5_0
    APP_TRACE_INFO0("DmExtAdvInit");
    DmExtAdvInit();

    for (uint8_t i = 0; i < DM_NUM_ADV_SETS; i++) {
        // dmExtAdvCbInit(i);
        dmExtAdvCb[i].useLegacyPdu = FALSE;
        dmExtAdvCb[i].priAdvPhy = HCI_ADV_PHY_LE_CODED;
        dmExtAdvCb[i].secAdvPhy = HCI_ADV_PHY_LE_CODED;
    }

    DmExtConnSlaveInit();
#else
    DmAdvInit();
    DmConnSlaveInit();
#endif
    DmSecInit();
    DmSecLescInit();
    DmPrivInit();
    DmHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(L2cSlaveHandler);
    L2cSlaveHandlerInit(handlerId);
    L2cInit();
    L2cSlaveInit();

    handlerId = WsfOsSetNextHandler(AttHandler);
    AttHandlerInit(handlerId);
    AttsInit();
    AttsIndInit();

    handlerId = WsfOsSetNextHandler(SmpHandler);
    SmpHandlerInit(handlerId);
    SmprInit();
    SmprScInit();
    HciSetMaxRxAclLen(100);

    handlerId = WsfOsSetNextHandler(AppHandler);
    AppHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(FitHandler);
    FitHandlerInit(handlerId);
}
