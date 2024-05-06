/*************************************************************************************************/
/*!
 * @file    main.c
 * @brief   BLE project with simple serial console for FCC testing
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

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "ll_init_api.h"
#include "chci_tr.h"
#include "lhci_api.h"
#include "hci_defs.h"
#include "wsf_assert.h"
#include "wsf_buf.h"
#include "wsf_cs.h"
#include "wsf_heap.h"
#include "wsf_timer.h"
#include "wsf_trace.h"
#include "wsf_bufio.h"
#include "bb_ble_sniffer_api.h"
#include "pal_bb.h"
#include "pal_cfg.h"
#include "pal_radio.h"
#include "tmr.h"
/**************************************************************************************************
  Definitions
**************************************************************************************************/

/*! \brief UART TX buffer size */
#define PLATFORM_UART_TERMINAL_BUFFER_SIZE 2048U

#define FREQ_HOP_PERIOD_US 20000
typedef enum {
    PAL_BB_CW,
    PAL_BB_PRBS9,
    PAL_BB_PRBS15,
} PalBbDbbPrbsType_t;
/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! \brief  Persistent BB runtime configuration. */
static BbRtCfg_t mainBbRtCfg;

/*! \brief  Persistent LL runtime configuration. */
static LlRtCfg_t mainLlRtCfg;

static uint8_t phy = LL_PHY_LE_1M;
static uint8_t phy_str[16];
static uint8_t txFreqHopCh;

static uint32_t numTxPowers;
static int8_t *txPowersAvailable;
/**************************************************************************************************
  Functions
**************************************************************************************************/

/*! \brief Physical layer functions. */

extern void PalBbAfeSetTxCfg(uint8_t rfChannel, int8_t txPower);
extern void PalBbDbbEnablePatternGen(PalBbDbbPrbsType_t prbsType);
extern void PalBbDbbDisablePatternGen(void);
extern bool_t PalBbAfeTxSetup(void);
extern bool_t PalBbAfeTxDone(void);

/*************************************************************************************************/
/*!
 *  \fn     Get PHY String.
 *
 *  \brief  Convert the PHY definition to a string.
 *
 *  \param  phy   PHY definition.
 *
 *  \return Pointer to string describing the PHY.
 */
/*************************************************************************************************/
static uint8_t *getPhyStr(uint8_t phy)
{
    switch (phy) {
    case LL_TEST_PHY_LE_1M:
    default:
        memcpy(phy_str, "1M PHY", 7);
        break;
    case LL_TEST_PHY_LE_2M:
        memcpy(phy_str, "2M PHY", 7);
        break;
    case LL_TEST_PHY_LE_CODED_S8:
        memcpy(phy_str, "S8 PHY", 7);
        break;
    case LL_TEST_PHY_LE_CODED_S2:
        memcpy(phy_str, "S2 PHY", 7);
        break;
    }
    return phy_str;
}

/*************************************************************************************************/
/*!
 *  \fn     Timer 2 interrupts handler.
 *
 *  \brief  Controls the frequency hopping.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*************************************************************************************************/
void TMR2_IRQHandler(void)
{
    int res;

    MXC_TMR_TO_Clear(MXC_TMR2);

    /* Start the next channel */
    res = LlEnhancedTxTest(txFreqHopCh++, 255, LL_TEST_PKT_TYPE_AA, phy, 0);
    if (res != LL_SUCCESS)
        APP_TRACE_INFO2("res = %u %s", res, res == LL_SUCCESS ? "(SUCCESS)" : "(FAIL)");

    /* Wrap the channels */
    if (txFreqHopCh == 40)
        txFreqHopCh = 0;

    /* Restart the timeout */
    MXC_TMR_TO_Start(MXC_TMR2, FREQ_HOP_PERIOD_US);
    MXC_TMR_EnableInt(MXC_TMR2);
}

/*************************************************************************************************/
/*!

 *  \brief  Print all available TX Powers the radio is capable of.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void printAvailablePowers(void)
{
    uint8_t top = numTxPowers > 9 ? 9 : numTxPowers;

    for (uint32_t i = 0; i < top; i++) {
        APP_TRACE_INFO2("%u: %d", i, txPowersAvailable[i]);
    }
}
/*************************************************************************************************/
/*!
 *  \fn     Usage statement
 *
 *  \brief  Prints the usage statement.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*************************************************************************************************/
void printUsage(void)
{
    APP_TRACE_INFO0("Usage: ");
    APP_TRACE_INFO0(" (0) Transmit Continuous Modulated on RF channel 0 (2402 MHz)");
    APP_TRACE_INFO0(" (1) Transmit Continuous Modulated on RF channel 19 (2440 MHz)");
    APP_TRACE_INFO0(" (2) Transmit Continuous Modulated RF channel 39 (2480 MHz)");
    APP_TRACE_INFO0(" (3) Receive  on RF channel 39 (2480 MHz)");
    APP_TRACE_INFO0(" (4) Set Transmit power");
    APP_TRACE_INFO0(" (5) Enable Constant Unmodulated TX");
    APP_TRACE_INFO0(" (6) Disable constant TX -- MUST be called after (5)");
    /* APP_TRACE_INFO0(" (7) Set PA value"); */
    APP_TRACE_INFO0(" (8) Set PHY");
    APP_TRACE_INFO0(" (9) TX Frequency Hop");
    APP_TRACE_INFO0(" (e) End transmission -- MUST be used after each (0-3, 9)");
    APP_TRACE_INFO0(" (u) Print usage");
}

/*************************************************************************************************/
/*!
 *  \fn     Process the Console RX
 *
 *  \brief  State machine for the console inputs.
 *
 *  \param  rxByte Character received from the console.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void processConsoleRX(uint8_t rxByte)
{
    int res;

    /* Holds the state of the command and the parameter */
    static uint8_t cmd = 0;
    static uint8_t param = 0;
    static int8_t power = INT8_MIN;
    static uint8_t channel = UINT8_MAX;

    /* Determines if the incoming character is a command or a parameter */
    if (cmd == 0)
        cmd = rxByte;
    else
        param = rxByte;

    switch (cmd) {
    case '0':

        APP_TRACE_INFO1("Transmit RF channel 0, 255 bytes/pkt, PRBS15, %s, forever ..",
                        getPhyStr(phy));
        res = LlEnhancedTxTest(0, 255, LL_TEST_PKT_TYPE_PRBS15, phy, 0);
        APP_TRACE_INFO2("res = %u %s", res, res == LL_SUCCESS ? "(SUCCESS)" : "(FAIL)");
        cmd = 0;
        break;

    case '1':

        APP_TRACE_INFO1("Transmit RF channel 19, 255 bytes/pkt, PRBS15, %s, forever ..",
                        getPhyStr(phy));
        res = LlEnhancedTxTest(19, 255, LL_TEST_PKT_TYPE_PRBS15, phy, 0);
        APP_TRACE_INFO2("res = %u %s", res, res == LL_SUCCESS ? "(SUCCESS)" : "(FAIL)");
        cmd = 0;
        break;

    case '2':

        APP_TRACE_INFO1("Transmit RF channel 39, 255 bytes/pkt, PRBS15, %s, forever ..",
                        getPhyStr(phy));
        res = LlEnhancedTxTest(39, 255, LL_TEST_PKT_TYPE_PRBS15, phy, 0);
        APP_TRACE_INFO2("res = %u %s", res, res == LL_SUCCESS ? "(SUCCESS)" : "(FAIL)");
        cmd = 0;
        break;

    case '3':

        APP_TRACE_INFO1("Receive RF channel 39, %s, forever ..", getPhyStr(phy));
        res = LlEnhancedRxTest(39, phy, 0, 0);
        APP_TRACE_INFO2("res = %u %s", res, res == LL_SUCCESS ? "(SUCCESS)" : "(FAIL)");
        cmd = 0;
        break;

    case '4':
        PalBbEnable();

        if (param == 0) {
            printAvailablePowers();
            break;
        } else if (param >= '0' && param <= '0' + numTxPowers) {
            uint8_t set_channel = channel == UINT8_MAX ? 0 : channel;

            power = txPowersAvailable[param - '0'];
            PalBbAfeSetTxCfg(set_channel, power);
            LlSetAdvTxPower(power);
            APP_TRACE_INFO1("Power set to %d dBm", txPowersAvailable[param - '0']);

        } else if (param < '0' || param > '9') {
            APP_TRACE_INFO0("Invalid selection");
        }

        cmd = 0;
        param = 0;
        break;

    case '5':
        PalBbEnable();
        if (param == 0) {
            APP_TRACE_INFO0("Select transmit channel");
            APP_TRACE_INFO0(" 0: 0");
            APP_TRACE_INFO0(" 1: 19");
            APP_TRACE_INFO0(" 2: 39");
            break;
        }

        int8_t set_power = power == INT8_MIN ? 0 : power;

        switch (param) {
        case '0': {
            channel = 0;
            APP_TRACE_INFO0("Channel set to 0");
            break;
        }
        case '1': {
            channel = 19;
            APP_TRACE_INFO0("Channel set to 19");
            break;
        }
        case '2': {
            channel = 39;
            APP_TRACE_INFO0("Channel set to 39");
            break;
        }
        default:
            APP_TRACE_INFO0("Invalid selection");
            break;
        }

        PalBbAfeSetTxCfg(channel, set_power);

        APP_TRACE_INFO0("Starting TX");

        /* Enable constant TX */
        PalBbAfeTxSetup();
        PalBbDbbEnablePatternGen(PAL_BB_CW);

        cmd = 0;
        param = 0;
        break;

    case '6':
        APP_TRACE_INFO0("Disabling TX");

        /* Disable constant TX */

        PalBbAfeTxDone();
        PalBbDbbDisablePatternGen();
        PalBbDisable();

        cmd = 0;
        break;

    case '8':
        if (param == 0) {
            /* Set the PHY */
            APP_TRACE_INFO0("Select PHY");
            APP_TRACE_INFO0("1: 1M");
            APP_TRACE_INFO0("2: 2M");
            APP_TRACE_INFO0("3: S8");
            APP_TRACE_INFO0("4: S2");
            break;
        }

        switch (param) {
        case '1':
            phy = LL_TEST_PHY_LE_1M;
            APP_TRACE_INFO0("PHY set to 1M");
            break;
        case '2':
            phy = LL_TEST_PHY_LE_2M;
            APP_TRACE_INFO0("PHY set to 2M");
            break;
        case '3':
            phy = LL_TEST_PHY_LE_CODED_S8;
            APP_TRACE_INFO0("PHY set to S8");
            break;
        case '4':
            phy = LL_TEST_PHY_LE_CODED_S2;
            APP_TRACE_INFO0("PHY set to S2");
            break;
        default:
            APP_TRACE_INFO0("Invalid selection");
            break;
        }

        cmd = 0;
        param = 0;
        break;
    case '9':
        /* Frequency hopping TX */
        APP_TRACE_INFO0("Starting frequency hopping");
        NVIC_EnableIRQ(TMR2_IRQn);
        MXC_TMR_TO_Start(MXC_TMR2, FREQ_HOP_PERIOD_US);
        MXC_TMR_EnableInt(MXC_TMR2);
        cmd = 0;
        break;
    case 'E':
    case 'e':

        APP_TRACE_INFO0("End test");
        MXC_TMR_Stop(MXC_TMR2);
        LlEndTest(NULL);
        cmd = 0;
        break;

    case 'U':
    case 'u':
        printUsage();
        cmd = 0;
        break;

    default:
        APP_TRACE_INFO0("Invalid selection");
        cmd = 0;
        param = 0;
        break;
    }
}

/*************************************************************************************************/
/*!
 *  \brief  Load runtime configuration.
 */
/*************************************************************************************************/
static void mainLoadConfiguration(void)
{
    PalBbLoadCfg((PalBbCfg_t *)&mainBbRtCfg);
    LlGetDefaultRunTimeCfg(&mainLlRtCfg);
    PalCfgLoadData(PAL_CFG_ID_LL_PARAM, &mainLlRtCfg.maxAdvSets, sizeof(LlRtCfg_t) - 9);
    PalCfgLoadData(PAL_CFG_ID_BLE_PHY, &mainLlRtCfg.phy2mSup, 4);
    /* Set 5.1 requirements. */
    mainLlRtCfg.btVer = LL_VER_BT_CORE_SPEC_5_0;

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
}
/*************************************************************************************************/
/*!
 *  \brief  Initialize Tx Powers Available for use
 */
/*************************************************************************************************/
static void mainInitTxPowers(void)
{
    numTxPowers = PalRadioGetNumAvailableTxPowers();
    txPowersAvailable = malloc(numTxPowers * sizeof(int8_t));

    if (txPowersAvailable == NULL) {
        APP_TRACE_ERR0("Failed to get number of available TX powers.");
        APP_TRACE_ERR0("Malloc returned NULL");
    }

    numTxPowers = PalRadioGetAvailableTxPowers(txPowersAvailable, numTxPowers);
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
    const uint16_t dataBufSize =
        12 + HCI_ISO_DL_MAX_LEN + mainLlRtCfg.maxAclLen + 4 + BB_DATA_PDU_TAILROOM;
#if (BT_VER > 9)
    /* Use single pool for data buffers. */
    WSF_ASSERT(mainLlRtCfg.maxAclLen == mainLlRtCfg.maxIsoSduLen);
#endif
    /* Ensure pool buffers are ordered correctly. */
    WSF_ASSERT(maxRptBufSize < dataBufSize);

    wsfBufPoolDesc_t poolDesc[] = {
        { 16, 8 },
        { 32, 4 },
        { 128, mainLlRtCfg.maxAdvReports },
        { maxRptBufSize, mainLlRtCfg.maxAdvReports }, /* Extended reports. */
        { dataBufSize, mainLlRtCfg.numTxBufs + mainLlRtCfg.numRxBufs + mainLlRtCfg.numIsoTxBuf +
                           mainLlRtCfg.numIsoRxBuf }
    };

    const uint8_t numPools = sizeof(poolDesc) / sizeof(poolDesc[0]);

    /* Initial buffer configuration. */
    uint16_t memUsed;
    WsfCsEnter();
    memUsed = WsfBufInit(numPools, poolDesc);
    WsfHeapAlloc(memUsed);
    WsfCsExit();

    WsfOsInit();
    WsfTimerInit();
#if (WSF_TRACE_ENABLED == TRUE)
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
    if (!eventPending)
        eventPending = WsfTokenService();
#endif

#if (BB_SNIFFER_ENABLED == TRUE)
    /* Service one sniffer packet, if in the buffer. */
    if (!eventPending)
        eventPending = LhciSnifferHandler();
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
    mainInitTxPowers();

#if (WSF_TRACE_ENABLED == TRUE)
    WsfCsEnter();
    memUsed = WsfBufIoUartInit(WsfHeapGetFreeStartAddress(), PLATFORM_UART_TERMINAL_BUFFER_SIZE);
    WsfHeapAlloc(memUsed);
    WsfCsExit();
#endif

    WsfCsEnter();
    LlInitRtCfg_t llCfg = { .pBbRtCfg = &mainBbRtCfg,
                            .wlSizeCfg = 4,
                            .rlSizeCfg = 4,
                            .plSizeCfg = 4,
                            .pLlRtCfg = &mainLlRtCfg,
                            .pFreeMem = WsfHeapGetFreeStartAddress(),
                            .freeMemAvail = WsfHeapCountAvailable() };

    memUsed = LlInitControllerInit(&llCfg);
    WsfHeapAlloc(memUsed);
    WsfCsExit();

    bdAddr_t bdAddr;
    PalCfgLoadData(PAL_CFG_ID_BD_ADDR, bdAddr, sizeof(bdAddr_t));
    /* Coverity[uninit_use_in_call] */
    LlSetBdAddr((uint8_t *)&bdAddr);

    WsfOsRegisterSleepCheckFunc(mainCheckServiceTokens);
    WsfOsRegisterSleepCheckFunc(ChciTrService);

    /* Register the UART RX request */
    WsfBufIoUartRegister(processConsoleRX);

    printUsage();

    WsfOsEnterMainLoop();

    /* Does not return. */
    return 0;
}
