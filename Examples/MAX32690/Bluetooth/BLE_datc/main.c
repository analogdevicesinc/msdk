/*************************************************************************************************/
/*!
 * @file    main.c
 * @brief   Simple BLE Data Client for unformatted data exchange.
 *
 *  Copyright (c) 2013-2019 Arm Ltd. All Rights Reserved.
 *
 *  Copyright (c) 2019 Packetcraft, Inc.
 *
 *  Portions Copyright (c) 2022-2023 Analog Devices, Inc.
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
#include "wsf_trace.h"
#include "wsf_bufio.h"
#include "wsf_msg.h"
#include "wsf_assert.h"
#include "wsf_buf.h"
#include "wsf_heap.h"
#include "wsf_cs.h"
#include "wsf_timer.h"
#include "wsf_os.h"

#include "sec_api.h"
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
#include "app_terminal.h"
#include "wut.h"
#include "rtc.h"
#include "trimsir_regs.h"

#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)
#include "ll_init_api.h"
#endif

#include "pal_bb.h"
#include "pal_cfg.h"

#include "datc_api.h"
#include "app_ui.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! \brief UART TX buffer size */
#define PLATFORM_UART_TERMINAL_BUFFER_SIZE 2048U
#define DEFAULT_TX_POWER 0 /* dBm */

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! \brief  Pool runtime configuration. */
static wsfBufPoolDesc_t mainPoolDesc[] = { { 16, 8 }, { 32, 4 }, { 192, 8 }, { 256, 16 } };

#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)
static LlRtCfg_t mainLlRtCfg;
#endif

volatile int wutTrimComplete;

/**************************************************************************************************
  Functions
**************************************************************************************************/

/*! \brief  Stack initialization for app. */
extern void StackInitDatc(void);

/*************************************************************************************************/
/*!
 *  \brief  Initialize WSF.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void mainWsfInit(void)
{
#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)
    /* +12 for message headroom, + 2 event header, +255 maximum parameter length. */
    const uint16_t maxRptBufSize = 12 + 2 + 255;

    /* +12 for message headroom, +4 for header. */
    const uint16_t aclBufSize = 12 + mainLlRtCfg.maxAclLen + 4 + BB_DATA_PDU_TAILROOM;

    /* Adjust buffer allocation based on platform configuration. */
    mainPoolDesc[2].len = maxRptBufSize;
    mainPoolDesc[2].num = mainLlRtCfg.maxAdvReports;
    mainPoolDesc[3].len = aclBufSize;
    mainPoolDesc[3].num = mainLlRtCfg.numTxBufs + mainLlRtCfg.numRxBufs;
#endif

    const uint8_t numPools = sizeof(mainPoolDesc) / sizeof(mainPoolDesc[0]);

    uint16_t memUsed;
    WsfCsEnter();
    memUsed = WsfBufCalcSize(numPools, mainPoolDesc);
    WsfHeapAlloc(memUsed);
    WsfBufInit(numPools, mainPoolDesc);
    WsfCsExit();

    WsfOsInit();
    WsfTimerInit();
#if (WSF_TOKEN_ENABLED == TRUE) || (WSF_TRACE_ENABLED == TRUE)
    WsfTraceRegisterHandler(WsfBufIoWrite);
    WsfTraceEnable(TRUE);
#endif
}

/*************************************************************************************************/
/*!
*  \fn     wutTrimCb
*
*  \brief  Callback function for the WUT 32 kHz crystal trim.
*
*  \param  err    Error code from the WUT driver.
*
*  \return None.
*/
/*************************************************************************************************/
void wutTrimCb(int err)
{
    if (err != E_NO_ERROR) {
        APP_TRACE_INFO1("32 kHz trim error %d\n", err);
    } else {
        APP_TRACE_INFO1("32kHz trimmed to 0x%x", (MXC_TRIMSIR->rtc & MXC_F_TRIMSIR_RTC_RTCX1) >>
                                                     MXC_F_TRIMSIR_RTC_RTCX1_POS);
    }
    wutTrimComplete = 1;
}

/*************************************************************************************************/
/*!
*  \fn     setAdvTxPower
*
*  \brief  Set the default advertising TX power.
*
*  \return None.
*/
/*************************************************************************************************/
void setAdvTxPower(void)
{
    LlSetAdvTxPower(DEFAULT_TX_POWER);
}

/*************************************************************************************************/
/*!
*  \fn     main
*
*  \brief  Entry point for demo software.
*
*  \param  None.
*
*  \return None.
*/
/*************************************************************************************************/
int main(void)
{
#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)
    /* Configurations must be persistent. */
    static BbRtCfg_t mainBbRtCfg;

    PalBbLoadCfg((PalBbCfg_t *)&mainBbRtCfg);
    LlGetDefaultRunTimeCfg(&mainLlRtCfg);
#if (BT_VER >= LL_VER_BT_CORE_SPEC_5_0)
    /* Set 5.0 requirements. */
    mainLlRtCfg.btVer = LL_VER_BT_CORE_SPEC_5_0;
#endif
    PalCfgLoadData(PAL_CFG_ID_LL_PARAM, &mainLlRtCfg.maxAdvSets, sizeof(LlRtCfg_t) - 9);
#if (BT_VER >= LL_VER_BT_CORE_SPEC_5_0)
    PalCfgLoadData(PAL_CFG_ID_BLE_PHY, (uint8_t *)&mainLlRtCfg.phy2mSup, 4);
#endif

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

    /* Increase the default ACL buffer size and count */
    mainLlRtCfg.numTxBufs = 8;
    mainLlRtCfg.numRxBufs = 8;
    mainLlRtCfg.maxAclLen = 256;

    /* Set the default connection power level */
    mainLlRtCfg.defTxPwrLvl = DEFAULT_TX_POWER;
#endif

    WsfCsEnter();
    WsfHeapAlloc(PLATFORM_UART_TERMINAL_BUFFER_SIZE);
    WsfBufIoUartInit(WsfHeapGetFreeStartAddress(), PLATFORM_UART_TERMINAL_BUFFER_SIZE);
    WsfCsExit();

    mainWsfInit();
    AppTerminalInit();

#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)

    uint32_t llmemUsed;

    /* Calculate how much memory we will need for the LL initialization */

    WsfCsEnter();

    WsfTraceEnable(FALSE);

    LlInitRtCfg_t llCfg = {
        .pBbRtCfg = &mainBbRtCfg,
        .wlSizeCfg = 4,
        .rlSizeCfg = 4,
        .plSizeCfg = 4,
        .pLlRtCfg = &mainLlRtCfg,
        /* Not significant yet, only being used for memory size requirement calculation. */
        .pFreeMem = WsfHeapGetFreeStartAddress(),
        /* Not significant yet, only being used for memory size requirement calculation. */
        .freeMemAvail = WsfHeapCountAvailable()
    };

    llmemUsed = LlInitSetRtCfg(&llCfg);

#if (WSF_TOKEN_ENABLED == TRUE) || (WSF_TRACE_ENABLED == TRUE)
    WsfTraceEnable(TRUE);
#endif

    /* Complete the LL initialization */
    /* Allocate the memory */
    WsfHeapAlloc(llmemUsed);

    /* Set the free memory pointers */
    llCfg.pFreeMem = WsfHeapGetFreeStartAddress();
    llCfg.freeMemAvail = WsfHeapCountAvailable();

    /* Run the initialization with properly set the free memory pointers */
    if (llmemUsed != LlInit(&llCfg)) {
        WSF_ASSERT(0);
    }

    WsfCsExit();

    bdAddr_t bdAddr;
    PalCfgLoadData(PAL_CFG_ID_BD_ADDR, bdAddr, sizeof(bdAddr_t));
    LlSetBdAddr((uint8_t *)&bdAddr);

    /* Start the 32 MHz crystal and the BLE DBB counter to trim the 32 kHz crystal */
    PalBbEnable();

    /* Output buffered square wave of 32 kHz clock to GPIO */
    // MXC_RTC_SquareWaveStart(MXC_RTC_F_32KHZ);

    /* Execute the trim procedure */
    wutTrimComplete = 0;
    MXC_WUT_TrimCrystalAsync(MXC_WUT0, wutTrimCb);
    while (!wutTrimComplete) {}

    /* Shutdown the 32 MHz crystal and the BLE DBB */
    PalBbDisable();
#endif

    StackInitDatc();
    DatcStart();

    WsfOsEnterMainLoop();

    /* Does not return. */
    return 0;
}
