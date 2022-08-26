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

volatile int longTestActive = 0;
bool pausePrompt            = false;
/* FreeRTOS+CLI */
void vRegisterCLICommands(void);
#define UARTx_IRQHandler UART0_IRQHandler
#define UARTx_IRQn       UART0_IRQn
mxc_gpio_cfg_t uart_cts = {MXC_GPIO0, MXC_GPIO_PIN_2, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_WEAK_PULL_UP,
                           MXC_GPIO_VSSEL_VDDIOH};
mxc_gpio_cfg_t uart_rts = {MXC_GPIO0, MXC_GPIO_PIN_3, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE,
                           MXC_GPIO_VSSEL_VDDIOH};
mxc_uart_regs_t* ConsoleUART = MXC_UART_GET_UART(CONSOLE_UART);

mxc_gpio_cfg_t uart_cts_isr;
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
static uint8_t txFreqHopCh;

/**************************************************************************************************
  Functions
**************************************************************************************************/

/*! \brief Physical layer functions. */
extern void llc_api_set_txpower(int8_t power);
extern void dbb_seq_select_rf_channel(uint32_t rf_channel);
extern void llc_api_tx_ldo_setup(void);
extern void dbb_seq_tx_enable(void);
extern void dbb_seq_tx_disable(void);

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
    printf("Usage: \r\n");
    printf(" (0) Transmit on RF channel 0 (2402 MHz)\r\n");
    printf(" (1) Transmit on RF channel 19 (2440 MHz)\r\n");
    printf(" (2) Transmit on RF channel 39 (2480 MHz)\r\n");
    printf(" (3) Receive  on RF channel 39 (2480 MHz)\r\n");
    printf(" (4) Set Transmit power\r\n");
    printf(" (5) Enable constant TX\r\n");
    printf(" (6) Disable constant TX -- MUST be called after (5)\r\n");
    /* APP_TRACE_INFO0(" (7) Set PA value"); */
    printf(" (8) Set PHY\r\n");
    printf(" (9) TX Frequency Hop\r\n");
    printf(" (e) End transmission -- MUST be used after each (0-3, 9)\r\n");
    printf(" (u) Print usage\r\n");
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
    static uint8_t cmd   = 0;
    static uint8_t param = 0;

    /* Determines if the incoming character is a command or a parameter */
    if (cmd == 0)
        cmd = rxByte;
    else
        param = rxByte;

    switch (cmd) {
        case '0':

            APP_TRACE_INFO1("Transmit RF channel 0, 255 bytes/pkt, 0xAA, %s, forever ..",
                            getPhyStr(phy));
            res = LlEnhancedTxTest(0, 255, LL_TEST_PKT_TYPE_AA, phy, 0);
            APP_TRACE_INFO2("res = %u %s", res, res == LL_SUCCESS ? "(SUCCESS)" : "(FAIL)");
            cmd = 0;
            break;

        case '1':

            APP_TRACE_INFO1("Transmit RF channel 19, 255 bytes/pkt, 0xAA, %s, forever ..",
                            getPhyStr(phy));
            res = LlEnhancedTxTest(19, 255, LL_TEST_PKT_TYPE_AA, phy, 0);
            APP_TRACE_INFO2("res = %u %s", res, res == LL_SUCCESS ? "(SUCCESS)" : "(FAIL)");
            cmd = 0;
            break;

        case '2':

            APP_TRACE_INFO1("Transmit RF channel 39, 255 bytes/pkt, 0xAA, %s, forever ..",
                            getPhyStr(phy));
            res = LlEnhancedTxTest(39, 255, LL_TEST_PKT_TYPE_AA, phy, 0);
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

            if (param == 0) {
                APP_TRACE_INFO0("Select transmit power");
                APP_TRACE_INFO0(" 0: -10 dBm");
                APP_TRACE_INFO0(" 1:   0 dBm");
                APP_TRACE_INFO0(" 2: 4.5 dBm");
                break;
            }

            switch (param) {
                case '0':
                    llc_api_set_txpower(-10);
                    LlSetAdvTxPower(-10);
                    APP_TRACE_INFO0("Power set to -10 dBm");
                    break;
                case '1':
                    llc_api_set_txpower(0);
                    LlSetAdvTxPower(0);
                    APP_TRACE_INFO0("Power set to 0 dBm");
                    break;
                case '2':
                    llc_api_set_txpower(4);
                    LlSetAdvTxPower(4);
                    APP_TRACE_INFO0("Power set to 4.5 dBm");
                    break;
                default:
                    APP_TRACE_INFO0("Invalid selection");
                    break;
            }
            cmd   = 0;
            param = 0;
            break;

        case '5':
            if (param == 0) {
                APP_TRACE_INFO0("Select transmit channel");
                APP_TRACE_INFO0(" 0: 0");
                APP_TRACE_INFO0(" 1: 19");
                APP_TRACE_INFO0(" 2: 39");
                break;
            }

            switch (param) {
                case '0':
                    dbb_seq_select_rf_channel(0);
                    APP_TRACE_INFO0("Channel set to 0");
                    break;
                case '1':
                    dbb_seq_select_rf_channel(19);
                    APP_TRACE_INFO0("Channel set to 19");
                    break;
                case '2':
                    dbb_seq_select_rf_channel(39);
                    APP_TRACE_INFO0("Channel set to 39");
                    break;
                default:
                    APP_TRACE_INFO0("Invalid selection");
                    break;
            }

            APP_TRACE_INFO0("Starting TX");

            PalBbEnable();

            llc_api_tx_ldo_setup();

            /* Enable constant TX */
            dbb_seq_tx_enable();

            cmd   = 0;
            param = 0;
            break;

        case '6':
            APP_TRACE_INFO0("Disabling TX");

            /* Disable constant TX */
            dbb_seq_tx_disable();

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

            cmd   = 0;
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
            cmd   = 0;
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
void prompt(void)
{
    if (pausePrompt)
        return;

    printf((longTestActive) ? "\r\n(active test) cmd> " : "\r\ncmd> ");
    fflush(stdout);
}
void vCmdLineTask_cb(mxc_uart_req_t* req, int error)
{
    BaseType_t xHigherPriorityTaskWoken;

    /* Wake the task */
    xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(cmd_task_id, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

    memset(buffer, 0, CMD_LINE_BUF_SIZE);
    index = 0;

    /* Register available CLI commands */
    vRegisterCLICommands();

    /* Enable UARTx interrupt */
    NVIC_ClearPendingIRQ(UARTx_IRQn);
    NVIC_DisableIRQ(UARTx_IRQn);
    NVIC_SetPriority(UARTx_IRQn, 1);
    NVIC_EnableIRQ(UARTx_IRQn);

    /* Async read will be used to wake process */
    async_read_req.uart     = ConsoleUART;
    async_read_req.rxData   = &tmp;
    async_read_req.rxLen    = 1;
    async_read_req.txData   = NULL;
    async_read_req.txLen    = 0;
    async_read_req.callback = vCmdLineTask_cb;

    printf("\nEnter 'help' to view a list of available commands.\n");
    prompt();
    while (1) {
        /* Register async read request */
        if (MXC_UART_TransactionAsync(&async_read_req) != E_NO_ERROR) {
            printf("Error registering async request. Command line unavailable.\n");
            vTaskDelay(portMAX_DELAY);
        }
        /* Hang here until ISR wakes us for a character */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // while (testActive) {
        //     volatile int x = 0;
        // }

        /* Check that we have a valid character */
        if (async_read_req.rxCnt > 0) {
            /* Process character */
            do {
                if (tmp == 0x08) {
                    /* Backspace */
                    if (index > 0) {
                        index--;
                        printf("\x08 \x08");
                    }
                    fflush(stdout);
                } else if (tmp == 0x03) {
                    /* ^C abort */
                    index = 0;
                    printf("^C");
                    prompt();
                } else if ((tmp == '\r') || (tmp == '\n')) {
                    printf("\r\n");
                    /* Null terminate for safety */
                    buffer[index] = 0x00;
                    /* Evaluate */
                    do {
                        xMore = FreeRTOS_CLIProcessCommand(buffer, output, OUTPUT_BUF_SIZE);
                        /* If xMore == pdTRUE, then output buffer contains no null termination, so
             *  we know it is OUTPUT_BUF_SIZE. If pdFALSE, we can use strlen.
             */
                        for (x = 0; x < (xMore == pdTRUE ? OUTPUT_BUF_SIZE : strlen(output)); x++) {
                            putchar(*(output + x));
                        }
                    } while (xMore != pdFALSE);
                    /* New prompt */
                    index = 0;
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
    tx_task_command_t notifyCommand;
    while (1) {
        xTaskNotifyWait(0, 0xFFFFFFFF, &notifVal, portMAX_DELAY);
        notifyCommand.allData = notifVal;
        uint8_t str[100]      = "Transmit RF channel : 0 :255 bytes/pkt : 0xAA : ";
        strcat(str, (phy == LL_TEST_PHY_LE_1M)       ? "1M PHY\n" :
                    (phy == LL_TEST_PHY_LE_2M)       ? "2M PHY\n" :
                    (phy == LL_TEST_PHY_LE_CODED_S8) ? "S8 PHY\n" :
                    (phy == LL_TEST_PHY_LE_CODED_S2) ? "S2 PHY\n" :
                                                       "");
        printf("%s", str);
        fflush(stdout);
        switch (notifyCommand.duration) {
            case 0:
                // // If max duration is recevied then assume test will be manually stopped
                fflush(stdout);
                res = LlEnhancedTxTest(notifyCommand.channel, 255, LL_TEST_PKT_TYPE_AA, phy, 0);
                printf("result = %u %s", res, res == LL_SUCCESS ? "(SUCCESS)\r\n" : "(FAIL)\r\n");
                fflush(stdout);
                pausePrompt = false;
                prompt();
                break;
            default:
                // perform timed test
                res = LlEnhancedTxTest(notifyCommand.channel, 255, LL_TEST_PKT_TYPE_AA, phy, 0);
                vTaskDelay(notifyCommand.duration);
                LlEndTest(NULL);
                pausePrompt = false;
                printf("result = %u %s", res, res == LL_SUCCESS ? "(SUCCESS)\r\n" : "(FAIL)\r\n");
                fflush(stdout);
                prompt();

                break;
        }
    }
}
void wfsLoop(void* pvParameters)
{
    int res           = 0;
    uint32_t notifVal = 0;

    while (1) {
        WsfOsEnterMainLoop();
    }
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

    // WsfHeapAlloc(memUsed);

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

    /* Enable incoming characters */
    /* Setup manual CTS/RTS to lockout console and wake from deep sleep */
    MXC_GPIO_Config(&uart_cts);
    MXC_GPIO_Config(&uart_rts);

    MXC_GPIO_OutClr(uart_rts.port, uart_rts.mask);
    // command line task
    xTaskCreate(vCmdLineTask, (const char*)"CmdLineTask",
                configMINIMAL_STACK_SIZE + CMD_LINE_BUF_SIZE + OUTPUT_BUF_SIZE, NULL,
                tskIDLE_PRIORITY + 1, &cmd_task_id);
    // TX tranismit test task
    xTaskCreate(txTestTask, (const char*)"Tx Task", 1024, NULL, tskIDLE_PRIORITY + 1, &tx_task_id);
    //wsfLoop task
    xTaskCreate(wfsLoop, (const char*)"Tx Task", 1024, NULL, tskIDLE_PRIORITY + 1, &wfs_task_id);

    /* Start scheduler */
    printf("Starting scheduler.\n");

    vTaskStartScheduler();
}
