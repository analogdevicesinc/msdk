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
#include "main.h"
/**************************************************************************************************
  Definitions
**************************************************************************************************/

/*! \brief UART TX buffer size */
#define PLATFORM_UART_TERMINAL_BUFFER_SIZE 2048U

#define FREQ_HOP_PERIOD_US 20000

/* FreeRTOS*************************************************************/
/* Array sizes */
#define CMD_LINE_BUF_SIZE 100
#define OUTPUT_BUF_SIZE   512
#define CONSOLE_UART      0 //EvKit/FTHR
/* Task IDs */
TaskHandle_t cmd_task_id;
TaskHandle_t tx_task_id;
TaskHandle_t wfs_task_id;
TaskHandle_t sweep_task_id;
TaskHandle_t help_task_id;
/* FreeRTOS+CLI */
void vRegisterCLICommands(void);
mxc_uart_regs_t* ConsoleUART = MXC_UART_GET_UART(CONSOLE_UART);
xSemaphoreHandle rfTestMutex;
/* Enables/disables tick-less mode */
unsigned int disable_tickless = 1;
/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! \brief  Persistent BB runtime configuration. */
static BbRtCfg_t mainBbRtCfg;

/*! \brief  Persistent LL runtime configuration. */
static LlRtCfg_t mainLlRtCfg;

static uint8_t phy = LL_PHY_LE_1M;
static uint8_t phy_str[16];
static uint8_t packetType_str[16];
static uint8_t txFreqHopCh;
static uint8_t packetLen  = 255;
static uint8_t packetType = LL_TEST_PKT_TYPE_AA;
char receivedChar;
/* helper flags */
test_t activeTest = NO_TEST;
bool clearScreen  = false;
bool pausePrompt  = false;
/**************************************************************************************************
  Functions
**************************************************************************************************/

/*! \brief Physical layer functions. */
extern void llc_api_set_txpower(int8_t power);
extern void dbb_seq_select_rf_channel(uint32_t rf_channel);
extern void llc_api_tx_ldo_setup(void);
extern void dbb_seq_tx_enable(void);
extern void dbb_seq_tx_disable(void);
extern const CLI_Command_Definition_t xCommandList[];
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
static uint8_t* getPhyStr(uint8_t phy)
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
 *  \fn     Get PHY String.
 *
 *  \brief  Convert the PHY definition to a string.
 *
 *  \param  phy   PHY definition.
 *
 *  \return Pointer to string describing the PHY.
 */
/*************************************************************************************************/

static uint8_t* getPacketTypeStr(void)
{
    switch (packetType) {
        case LL_TEST_PKT_TYPE_PRBS9:
            memcpy(packetType_str, "PRBS9", 6);
            break;
        case LL_TEST_PKT_TYPE_0F:
            memcpy(packetType_str, "0x0F", 5);
            break;
        case LL_TEST_PKT_TYPE_55:
            memcpy(packetType_str, "0x55", 5);
            break;
        case LL_TEST_PKT_TYPE_PRBS15:
            memcpy(packetType_str, "PRBS15", 7);
            break;
        case LL_TEST_PKT_TYPE_FF:
            memcpy(packetType_str, "0xFF", 5);
            break;
        case LL_TEST_PKT_TYPE_00:
            memcpy(packetType_str, "0x00", 5);
            break;
        case LL_TEST_PKT_TYPE_F0:
            memcpy(packetType_str, "0xF0", 5);
            break;
        case LL_TEST_PKT_TYPE_AA:
        default:
            memcpy(packetType_str, "0xAA", 5);
            break;
    }
    return packetType_str;
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
    res = LlEnhancedTxTest(txFreqHopCh++, packetLen, packetType, phy, 0);
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
    BaseType_t xHigherPriorityTaskWoken;
    receivedChar = rxByte;
    /* Wake the task */
    xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(cmd_task_id, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

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

    /* Use single pool for data buffers. */
    WSF_ASSERT(mainLlRtCfg.maxAclLen == mainLlRtCfg.maxIsoSduLen);

    /* Ensure pool buffers are ordered correctly. */
    WSF_ASSERT(maxRptBufSize < dataBufSize);

    wsfBufPoolDesc_t poolDesc[] = {
        {16, 8},
        {32, 4},
        {128, mainLlRtCfg.maxAdvReports},
        {maxRptBufSize, mainLlRtCfg.maxAdvReports}, /* Extended reports. */
        {dataBufSize, mainLlRtCfg.numTxBufs + mainLlRtCfg.numRxBufs + mainLlRtCfg.numIsoTxBuf +
                          mainLlRtCfg.numIsoRxBuf}};

    const uint8_t numPools = sizeof(poolDesc) / sizeof(poolDesc[0]);

    /* Initial buffer configuration. */
    uint16_t memUsed;
    memUsed = WsfBufInit(numPools, poolDesc);
    WsfHeapAlloc(memUsed);
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
//*********************|  Freertos |***********************************
void cls(void)
{
    char str[7];
    // TODO check clear on putty
    sprintf(str, "\033[2J");
    WsfBufIoWrite((const uint8_t*)str, 5);
    clearScreen = false;
}
void prompt(void)
{
    if (pausePrompt)
        return;
    char str[25];
    uint8_t len = 0;
    if (activeTest) {
        sprintf(str, "\r\n(active test) cmd:");
        len = 21;
    } else {
        sprintf(str, "\r\ncmd:");
        len = 7;
    }

    fflush(stdout);
    if (clearScreen) {
        cls();
    }
    //using app_trace would add newline after prompt which does not look right
    WsfBufIoWrite((const uint8_t*)str, len);
}
void printHint(uint8_t* buff)
{
    int i           = 0;
    uint8_t bufflen = strlen(buff);
    bool foundMatch = false;
    do {
        if (memcmp(buff, xCommandList[i].pcCommand, bufflen) == 0 && bufflen > 0) {
            printf("\r\n%s %s", xCommandList[i].pcCommand, xCommandList[i].pcHelpString);
            foundMatch = true;
        }
        i++;
    } while (xCommandList[i].pcCommand != NULL);
    if (foundMatch) {
        /* print new prompt with what user had previouslly typed */
        printf("\r\n");
        prompt();
        vTaskDelay(5);
        printf("%s", buff);
        fflush(stdout);
    }
}
void vCmdLineTask(void* pvParameters)
{
    unsigned char tmp;
    unsigned int index; /* Index into buffer */
    unsigned int x;
    int uartReadLen;
    char buffer[CMD_LINE_BUF_SIZE]; /* Buffer for input */
    char output[OUTPUT_BUF_SIZE];   /* Buffer for output */
    BaseType_t xMore;
    mxc_uart_req_t async_read_req;
    uint8_t backspace[] = "\x08 \x08";
    memset(buffer, 0, CMD_LINE_BUF_SIZE);
    index = 0;

    /* Register available CLI commands */
    vRegisterCLICommands();
    vTaskDelay(100); /* give time for ll api to print things */
    fflush(stdout);
    /* clear screen and print help table */
    cls();
    pausePrompt = true;
    xTaskNotify(help_task_id, 0xFF, eSetBits);

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        tmp = receivedChar;
        /* Check that we have a valid character */
        if (async_read_req.rxCnt > 0) {
            /* Process character */
            do {
                /* 0x08 BS linux , 127 Del windows/putty */
                if (tmp == 0x08 || tmp == 127) {
                    /* Backspace */
                    if (index > 0) {
                        index--;
                        memset(&buffer[index], 0x00, 1);
                        WsfBufIoWrite((const uint8_t*)backspace, sizeof(backspace));
                    }
                    fflush(stdout);
                } else if (tmp == 0x09)
                /* tab hint */
                {
                    printHint(buffer);

                }
                /*since freq hop does not allow user to see what they are typing, simply typing
                  'e' without the need to press enter willl stop the frequency hopping test */
                else if ((char)tmp == 'e' && activeTest == BLE_FHOP_TEST) {
                    LlEndTest(NULL);
                    MXC_TMR_Stop(MXC_TMR2);
                    activeTest = NO_TEST;

                    xSemaphoreGive(rfTestMutex);
                    prompt();
                } else if (tmp == 0x03) {
                    /* ^C abort */
                    index = 0;
                    APP_TRACE_INFO0("^C");
                    prompt();
                } else if ((tmp == '\r') || (tmp == '\n')) {
                    if (strlen(buffer) > 0) {
                        APP_TRACE_INFO0("\r\n");
                        /* Null terminate for safety */
                        buffer[index] = 0x00;
                        /* Evaluate */
                        do {
                            xMore = FreeRTOS_CLIProcessCommand(buffer, output, OUTPUT_BUF_SIZE);
                            for (x = 0; x < (xMore == pdTRUE ? OUTPUT_BUF_SIZE : strlen(output));
                                 x++) {
                                putchar(*(output + x));
                            }
                        } while (xMore != pdFALSE);
                    }
                    /* New prompt */
                    index = 0;
                    memset(buffer, 0x00, 100);
                    prompt();
                } else if (index < CMD_LINE_BUF_SIZE) {
                    putchar(tmp);
                    buffer[index++] = tmp;
                    fflush(stdout);

                } else {
                    /* Throw away data and beep terminal */
                    putchar(0x07);
                    fflush(stdout);
                }
                uartReadLen = 1;
                /* If more characters are ready, process them here */
            } while ((MXC_UART_GetRXFIFOAvailable(MXC_UART_GET_UART(CONSOLE_UART)) > 0) &&
                     (MXC_UART_Read(ConsoleUART, (uint8_t*)&tmp, &uartReadLen) == 0));
        }
    }
}
void txTestTask(void* pvParameters)
{
    static int res    = 0xff;
    uint32_t notifVal = 0;
    tx_config_t testConfig;
    char str[80];
    while (1) {
        /* Wait for notification to initiate TX/RX */
        xTaskNotifyWait(0, 0xFFFFFFFF, &notifVal, portMAX_DELAY);
        /* Get test settings from the notification value */
        testConfig.allData = notifVal;

        if (testConfig.testType == BLE_TX_TEST) {
            sprintf(str, "Transmit RF channel %d : %d bytes/pkt : ", testConfig.channel, packetLen);
            strcat(str, (const char*)getPacketTypeStr());
        } else {
            sprintf(str, "Receive RF channel %d : ", testConfig.channel);
        }
        strcat(str, " : ");
        strcat(str, (const char*)getPhyStr(phy));
        APP_TRACE_INFO1("%s", str);

        /* stat test */
        if (testConfig.testType == BLE_TX_TEST) {
            res = LlEnhancedTxTest(testConfig.channel, packetLen, packetType, phy, 0);
        } else {
            res = LlEnhancedRxTest(testConfig.channel, phy, 0, 0);
        }
        APP_TRACE_INFO2("result = %u %s", res, res == LL_SUCCESS ? "(SUCCESS)" : "(FAIL)");
        if (testConfig.duration_ms) {
            vTaskDelay(testConfig.duration_ms);
            LlEndTest(NULL);
            xSemaphoreGive(rfTestMutex);
        }
        pausePrompt = false;

        prompt();
    }
}
void sweepTestTask(void* pvParameters)
{
    int res = 0xff;

    uint32_t notifVal = 0;
    sweep_config_t sweepConfig;
    tx_config_t txCommand;
    /* channles in order of appreance in the spectrum */
    uint8_t ble_channels_spectrum[40] = {37, 0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 38, 11,
                                         12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
                                         26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 39};
    // remap to find channel location in above list
    uint8_t ble_channels_remap[40] = {1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 13, 14, 15,
                                      16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
                                      30, 31, 32, 33, 34, 35, 36, 37, 38, 0,  12, 39};
    while (1) {
        /* Wait for notification to initiate sweep */
        xTaskNotifyWait(0, 0xFFFFFFFF, &notifVal, portMAX_DELAY);
        sweepConfig.allData = notifVal;
        APP_TRACE_INFO3("\r\nStarting TX sweep from Ch %d to Ch %d @ %d ms per channel",
                        sweepConfig.start_channel, sweepConfig.end_channel,
                        sweepConfig.duration_per_ch_ms);

        /* get index  */
        uint8_t start_ch = ble_channels_remap[sweepConfig.start_channel];
        uint8_t end_ch   = ble_channels_remap[sweepConfig.end_channel];
        char str[6]      = "";
        /* config txCommand to RF Task */
        txCommand.duration_ms = sweepConfig.duration_per_ch_ms;
        txCommand.testType    = BLE_TX_TEST;
        strcat(str, (const char*)getPhyStr(phy));
        /* sweep channels */
        for (int i = start_ch; i <= end_ch; i++) {
            APP_TRACE_INFO2("\r\n-----------------| channel %d %s |----------------------\r\n",

                            ble_channels_spectrum[i], str);
            txCommand.channel = ble_channels_spectrum[i];
            res = LlEnhancedTxTest(ble_channels_spectrum[i], packetLen, packetType, phy, 0);
            vTaskDelay(sweepConfig.duration_per_ch_ms);
            LlEndTest(NULL);
            xSemaphoreGive(rfTestMutex);
            vTaskDelay(50); /* give console time to print end of  test reuslts */
        }
        activeTest  = NO_TEST;
        pausePrompt = false;
        prompt();
    }
}
void helpTask(void* pvParameters)
{
    uint32_t notifVal = 0;
    while (1) {
        xTaskNotifyWait(0, 0xFFFFFFFF, &notifVal, portMAX_DELAY);

        // clang-format off
    
        // printf(" Command   parameters [optional] <required>                        description                       \r\n");
        // printf("--------- ---------------------------------- ------------------------------------------------------- \r\n");
        // printf(" cls       N/A                                clears the screen                                      \r\n\r\n");
        // printf(" constTx   <channel> : 1 - 39                 Constant TX on given channel. Channel param            \r\n");
        // printf("                                              is required on first command call                      \r\n");
        // printf("                                              subsequent calls default to last given channel         \r\n\r\n");
        // printf(" e         N/A                                Ends any active RX/TX/Constant/Freq.hop RF test        \r\n\r\n");
        // printf(" plen      <packet_length> : 0-255 bytes      Sets payload packet length                             \r\n\r\n");
        // printf(" ptype     <packet_type>                      Sets payload packet type                               \r\n");
        // printf("                                              PRBS9,PRBS15,00,FF,0F,F0,55,AA                         \r\n\r\n");
        // printf(" phy       <symbol rate> : 1M 2M S2 S8        Sets the PHY symbol rate. Not case sensitive 1M == 1m  \r\n\r\n");
        // printf(" ps        N/A                                Display freeRTOS task stats                            \r\n\r\n");
        // printf(" rx        <channel> : 1 - 39                 RX test on given channel. Channel param                \r\n");
        // printf("                                              is required on first command call.                     \r\n");
        // printf("                                              Subsequent calls default to last given channel         \r\n\r\n");
        // printf(" sweep     <start ch> <end ch> <ms/per ch>    Sweeps TX tests through a range of channels given      \r\n");
        // printf("                                              their order of appearance on the spectrum.             \r\n\r\n");
        // printf(" tx        <channel> : 1 - 39                 TX test on given channel. Channel param                \r\n");
        // printf("                                              is required on first command call.                     \r\n");
        // printf("                                              Subsequent calls default to last given channel         \r\n\r\n");
        // printf(" txdbm     <dbm> : 4.5 2 -10                  Select transmit power                                  \r\n\r\n");
        // printf(" help      N/A                                Displays this help table                               \r\n");





    printf("┌─────────┬──────────────────────────────────┬───────────────────────────────────────────────────────┐\r\n");
    printf("│ Command │ parameters [optional] <required> │                      description                      │\r\n");
    printf("├─────────┼──────────────────────────────────┼───────────────────────────────────────────────────────┤\r\n");
    printf("│ cls     │ N/A                              │ clears the screen                                     │\r\n");
    printf("│         │                                  │                                                       │\r\n");
    printf("│ constTx │ <channel> : 1 - 39               │ Constant TX on given channel. Channel param           │\r\n");
    printf("│         │ ex: constTx 0                    │ is required on first command call                     │\r\n");
    printf("│         │                                  │ subsequent calls default to last given channel        │\r\n");
    printf("│         │                                  │                                                       │\r\n");
    printf("│ e       │ N/A                              │ Ends any active RX/TX/Constant/Freq.hop RF test       │\r\n");
    printf("│         │                                  │                                                       │\r\n");
    printf("│ phy     │ <symbol_rate> : 1M 2M S2 S8      │ Sets the PHY symbol rate. Not case sensitive 1M == 1m │\r\n");
    printf("│         │ ex: phy 2M                       │                                                       │\r\n");
    printf("│         │                                  │                                                       │\r\n");
    printf("│ plen    │ <packet_length>                  │ Sets payload packet length 0-255 bytes                │\r\n");
    printf("│         │                                  │                                                       │\r\n");
    printf("│ ptype   │ <packet_type>                    │ Sets payload packet type.                             |\r\n");
    printf("│         │                                  │ PRBS9,PRBS15,00,FF,F0,0F,55,AA                        │\r\n");
    printf("│         │                                  │                                                       │\r\n");
    printf("│ ps      │ N/A                              │ Display freeRTOS task stats                           │\r\n");
    printf("│         │                                  │                                                       │\r\n");
    printf("│ rx      │ <channel> <duration_ms>          │ RX test on given channel. Channel param               │\r\n");
    printf("│         │ ex: rx 0 500                     │ is required on first command call. Duration param     │\r\n");
    printf("│         │ ex: rx 1 (use prev. duration)    │ defautls to 0 which is infinite until stopped.        │\r\n");
    printf("│         │ ex: rx (use prev. ch & duration) │ Subsequent calls to rx default to last given params   │\r\n");
    printf("│         │                                  │                                                       │\r\n");
    printf("│ sweep   │ <start_ch> <end_ch> <ms/per_ch>  │ Sweeps TX tests through a range of channels given     │\r\n");
    printf("│         │ ex: sweep 0 10 500               │ their order of appearance on the spectrum.            │\r\n");
    printf("│         │ ex: sweep (use prev. params)     │ Subsequent calls to sweep uses last given params      │\r\n");
    printf("│         │                                  │                                                       │\r\n");
    printf("│ tx      │ <channel> [duration_ms]          │ TX test on given channel. Channel param               │\r\n");
    printf("│         │ ex: tx 0 500                     │ is required on first command call. Duration param     │\r\n");
    printf("│         │ ex: tx 1 (use prev. duration)    │ defautls to 0 which is infinite until stopped.        │\r\n");
    printf("│         │ ex: tx (use prev. ch & duration) │ Subsequent calls to tx default to last given params   │\r\n");
    printf("│         │                                  │                                                       │\r\n");
    printf("│ txdbm   │ <dbm> :TODO: what is range here? │ Select transmit power                                 │\r\n");
    printf("│         │ ex: txdbm -10                    │                                                       │\r\n");
    printf("│         │                                  │                                                       │\r\n");
    printf("│ help    │ N/A                              │ Displays this help table                              │\r\n");
    printf("└─────────┴──────────────────────────────────┴───────────────────────────────────────────────────────┘\r\n");

        // clang-format on

        pausePrompt = false;
        prompt();
    }
}
void wfsLoop(void* pvParameters)
{
    while (1) {
        WsfOsEnterMainLoop();
    }
}
void setPhy(uint8_t newPhy)
{
    phy          = newPhy;
    char str[20] = "Phy now set to ";
    strcat(str, (phy == LL_TEST_PHY_LE_1M)       ? "1M PHY" :
                (phy == LL_TEST_PHY_LE_2M)       ? "2M PHY" :
                (phy == LL_TEST_PHY_LE_CODED_S8) ? "S8 PHY" :
                (phy == LL_TEST_PHY_LE_CODED_S2) ? "S2 PHY" :
                                                   "");
    APP_TRACE_INFO1("%s", str);
}
void startFreqHopping(void)
{
    NVIC_EnableIRQ(TMR2_IRQn);
    MXC_TMR_TO_Start(MXC_TMR2, FREQ_HOP_PERIOD_US);
    MXC_TMR_EnableInt(MXC_TMR2);
}
void setPacketLen(uint8_t len)
{
    packetLen = len;
    APP_TRACE_INFO1("Packet length set to %d", len);
}
void setPacketType(uint8_t type)
{
    packetType = type;
    APP_TRACE_INFO1("Packet type set to %s", getPacketTypeStr());
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

    LlInitRtCfg_t llCfg = {.pBbRtCfg     = &mainBbRtCfg,
                           .wlSizeCfg    = 4,
                           .rlSizeCfg    = 4,
                           .plSizeCfg    = 4,
                           .pLlRtCfg     = &mainLlRtCfg,
                           .pFreeMem     = WsfHeapGetFreeStartAddress(),
                           .freeMemAvail = WsfHeapCountAvailable()};

    memUsed = LlInitControllerInit(&llCfg);
    WsfHeapAlloc(memUsed);

    bdAddr_t bdAddr;
    PalCfgLoadData(PAL_CFG_ID_BD_ADDR, bdAddr, sizeof(bdAddr_t));
    /* Coverity[uninit_use_in_call] */
    LlSetBdAddr((uint8_t*)&bdAddr);

    WsfOsRegisterSleepCheckFunc(mainCheckServiceTokens);
    WsfOsRegisterSleepCheckFunc(ChciTrService);
    /* Register the UART RX request */
    WsfBufIoUartRegister(processConsoleRX);

    /* FreeRTOS */
    rfTestMutex = xSemaphoreCreateMutex();
    if (rfTestMutex == NULL) {
        printf("xSemaphoreCreateMutex failed to create a mutex.\n");
        printf("necessary for operation...\r\n");
        while (1) {
            //hang here....
        }
    }
    xTaskCreate(vCmdLineTask, (const char*)"CmdLineTask",
                configMINIMAL_STACK_SIZE + CMD_LINE_BUF_SIZE + OUTPUT_BUF_SIZE, NULL,
                tskIDLE_PRIORITY + 1, &cmd_task_id);
    // TX tranismit test task
    xTaskCreate(txTestTask, (const char*)"Tx Task", 1024, NULL, tskIDLE_PRIORITY + 1, &tx_task_id);

    // Sweep test task
    xTaskCreate(sweepTestTask, (const char*)"Sweep Task", 1024, NULL, tskIDLE_PRIORITY + 1,
                &sweep_task_id);

    // help task
    xTaskCreate(helpTask, (const char*)"Help Task", 1024, NULL, tskIDLE_PRIORITY + 1,
                &help_task_id);

    //wsfLoop task
    xTaskCreate(wfsLoop, (const char*)"WFS Task", 1024, NULL, tskIDLE_PRIORITY + 1, &wfs_task_id);

    /* Start scheduler */
    APP_TRACE_INFO0(">> Starting scheduler.\r\n");

    vTaskStartScheduler();

    return 0;
}
