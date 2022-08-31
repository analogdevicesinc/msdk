/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Main module.
 *
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

#include "bb_ble_sniffer_api.h"
#include "chci_tr.h"
#include "hci_defs.h"
#include "lhci_api.h"
#include "ll_init_api.h"
#include "pal_bb.h"
#include "pal_cfg.h"
#include "wsf_assert.h"
#include "wsf_buf.h"
#include "wsf_bufio.h"
#include "wsf_heap.h"
#include "wsf_timer.h"
#include "wsf_trace.h"

/*! \brief UART TX buffer size */
#define PLATFORM_UART_TERMINAL_BUFFER_SIZE 2048U

#define DEFAULT_TX_POWER 0 /* dBm */

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! \brief  Persistent BB runtime configuration. */
static BbRtCfg_t mainBbRtCfg;

/*! \brief  Persistent LL runtime configuration. */
static LlRtCfg_t mainLlRtCfg;

/**************************************************************************************************
  Functions
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Load runtime configuration.
 */
/*************************************************************************************************/
static void mainLoadConfiguration(void)
{
    PalBbLoadCfg((PalBbCfg_t*)&mainBbRtCfg);
    LlGetDefaultRunTimeCfg(&mainLlRtCfg);
    PalCfgLoadData(PAL_CFG_ID_LL_PARAM, &mainLlRtCfg.maxAdvSets, sizeof(LlRtCfg_t) - 9);
    PalCfgLoadData(PAL_CFG_ID_BLE_PHY, &mainLlRtCfg.phy2mSup, 4);

    /* Set 5.0 requirements. */
    mainLlRtCfg.btVer = BT_VER;

    /* Set the 32k sleep clock accuracy into one of the following bins, default is 20
      HCI_CLOCK_500PPM
      HCI_CLOCK_250PPM
      HCI_CLOCK_150PPM
      HCI_CLOCK_100PPM
      HCI_CLOCK_75PPM
      HCI_CLOCK_50PPM
      HCI_CLOCK_30PPM
      HCI_CLOCK_20PPM
    */
    mainBbRtCfg.clkPpm = 20;

    /* Set the default connection power level */
    mainLlRtCfg.defTxPwrLvl = DEFAULT_TX_POWER;

    /* Adjust the extended advertising and ISO settings */
    mainLlRtCfg.maxAdvSets = 2;
    mainLlRtCfg.maxAdvReports = 4;
    mainLlRtCfg.numIsoTxBuf = 8;
    mainLlRtCfg.maxCis = 2;
    mainLlRtCfg.maxBis = 2;
}

/*************************************************************************************************/
/*!
 *  \brief  Initialize WSF.
 */
/*************************************************************************************************/
static void mainWsfInit(void)
{
    /* +12 for message headroom, + 2 event header, +255 maximum parameter length. */
    const uint16_t maxRptBufSize = 12 + 2 + 255;

    /* +12 for message headroom, +ISO Data Load, +4 for header. */
    const uint16_t dataBufSize
        = 12 + HCI_ISO_DL_MAX_LEN + mainLlRtCfg.maxAclLen + 4 + BB_DATA_PDU_TAILROOM;

    /* Use single pool for data buffers. */
    WSF_ASSERT(mainLlRtCfg.maxAclLen == mainLlRtCfg.maxIsoSduLen);

    /* Ensure pool buffers are ordered correctly. */
    WSF_ASSERT(maxRptBufSize < dataBufSize);

    wsfBufPoolDesc_t poolDesc[] = { { 16, 8 }, { 32, 4 }, { 128, mainLlRtCfg.maxAdvReports },
        { maxRptBufSize, mainLlRtCfg.maxAdvReports }, /* Extended reports. */
        { dataBufSize,
            mainLlRtCfg.numTxBufs + mainLlRtCfg.numRxBufs + mainLlRtCfg.numIsoTxBuf
                + mainLlRtCfg.numIsoRxBuf } };

    const uint8_t numPools = sizeof(poolDesc) / sizeof(poolDesc[0]);

    /* Initial buffer configuration. */
    uint16_t memUsed;
    memUsed = WsfBufInit(numPools, poolDesc);
    WsfHeapAlloc(memUsed);
    WsfOsInit();
    WsfTimerInit();
#if (WSF_TRACE_ENABLED == TRUE)
    memUsed = WsfBufIoUartInit(WsfHeapGetFreeStartAddress(), PLATFORM_UART_TERMINAL_BUFFER_SIZE);
    WsfHeapAlloc(memUsed);
    WsfTraceRegisterHandler(WsfBufIoWrite);
    WsfTraceEnable(TRUE);
#endif
}

/*************************************************************************************************/
/*!
 *  \brief  Check and service tokens (Trace and sniffer).
 *
 *  \return TRUE if there is token pending.
 */
/*************************************************************************************************/
static bool_t mainCheckServiceTokens(void)
{
    bool_t eventPending = FALSE;

#if (WSF_TOKEN_ENABLED == TRUE) || (BB_SNIFFER_ENABLED == TRUE)
    eventPending = LhciIsEventPending();
#endif

#if WSF_TOKEN_ENABLED == TRUE
    /* Allow only a single token to be processed at a time. */
    if (!eventPending) {
        eventPending = WsfTokenService();
    }
#endif

#if (BB_SNIFFER_ENABLED == TRUE)
    /* Service one sniffer packet, if in the buffer. */
    if (!eventPending) {
        eventPending = LhciSnifferHandler();
    }
#endif

    return eventPending;
}

/*************************************************************************************************/
/*!
 *  \brief  Main entry point.
 */
/*************************************************************************************************/
int main(void)
{
    uint32_t memUsed;

    mainLoadConfiguration();
    mainWsfInit();

#if (WSF_TRACE_ENABLED == TRUE)
    memUsed = WsfBufIoUartInit(WsfHeapGetFreeStartAddress(), PLATFORM_UART_TERMINAL_BUFFER_SIZE);
    WsfHeapAlloc(memUsed);
#endif

    LlInitRtCfg_t llCfg = { .pBbRtCfg = &mainBbRtCfg,
        .wlSizeCfg = 4,
        .rlSizeCfg = 4,
        .plSizeCfg = 4,
        .pLlRtCfg = &mainLlRtCfg,
        .pFreeMem = WsfHeapGetFreeStartAddress(),
        .freeMemAvail = WsfHeapCountAvailable() };

    memUsed = LlInitControllerInit(&llCfg);
    WsfHeapAlloc(memUsed);

    bdAddr_t bdAddr;
    PalCfgLoadData(PAL_CFG_ID_BD_ADDR, bdAddr, sizeof(bdAddr_t));
    /* Coverity[uninit_use_in_call] */
    LlSetBdAddr((uint8_t*)&bdAddr);
    LlSetAdvTxPower(DEFAULT_TX_POWER);

    WsfOsRegisterSleepCheckFunc(mainCheckServiceTokens);
    WsfOsRegisterSleepCheckFunc(ChciTrService);

    WsfOsEnterMainLoop();

    /* Does not return. */
    return 0;
}
