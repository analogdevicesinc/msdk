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
/*! UART TX buffer size */
#define PLATFORM_UART_TERMINAL_BUFFER_SIZE 2048U
#define FREQ_HOP_PERIOD_US 20000
#define HISTORY_MEMORY_LENGTH 10
/* FreeRTOS */
#define CMD_LINE_BUF_SIZE 100
#define OUTPUT_BUF_SIZE 512
#define CONSOLE_UART 0 //EvKit/FTHR
/* Task IDs */
TaskHandle_t cmd_task_id;
TaskHandle_t tx_task_id;
TaskHandle_t sweep_task_id;
TaskHandle_t help_task_id;
/* FreeRTOS+CLI */
xSemaphoreHandle rfTestMutex;

mxc_uart_regs_t *ConsoleUART = MXC_UART_GET_UART(CONSOLE_UART);
unsigned int disable_tickless = 1; /* Enables/disables tick-less mode */
/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/* Persistent BB runtime configuration. */
static BbRtCfg_t mainBbRtCfg;

/* Persistent LL runtime configuration. */
static LlRtCfg_t mainLlRtCfg;

static uint8_t phy = LL_PHY_LE_1M;
static uint8_t phy_str[16];
static uint8_t packetType_str[16];
static uint8_t txFreqHopCh;
static uint8_t packetLen = 255;
static uint8_t packetType = LL_TEST_PKT_TYPE_AA;
static int8_t txPower = 10;
/* UART RX */
char inputBuffer[CMD_LINE_BUF_SIZE]; /* Buffer for input */
unsigned int bufferIndex; /* Index into buffer */
char receivedChar;
/* CLI escape sequences*/
uint8_t backspace[] = "\x08 \x08";
enum { UP_ARROW, DOWN_ARROW, RIGHT_ARROW, LEFT_ARROW };
/* CLI History */
cmd_history_t cmd_history[HISTORY_MEMORY_LENGTH];
static uint32_t escCounter = 0;
static uint8_t keyBoardSequenceBuff[3] = { 0 };
queue_t historyQueue;

/* CLI Prompt */
test_t activeTest = NO_TEST;
bool clearScreen = false;
bool pausePrompt = false;

/**************************************************************************************************
  Functions
**************************************************************************************************/
/* Physical layer functions. */
extern void llc_api_set_txpower(int8_t power);
extern void dbb_seq_select_rf_channel(uint32_t rf_channel);
extern void llc_api_tx_ldo_setup(void);
extern void dbb_seq_tx_enable(void);
extern void dbb_seq_tx_disable(void);
extern const CLI_Command_Definition_t xCommandList[];
void vRegisterCLICommands(void);
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
 *  \fn     Get PHY String.
 *
 *  \brief  Convert the PHY definition to a string.
 *
 *  \param  phy   PHY definition.
 *
 *  \return Pointer to string describing the PHY.
 */
/*************************************************************************************************/
static uint8_t *getPacketTypeStr(void)
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
 *  \fn     processEscSequence.
 *
 *  \brief  Compares keyboard sequence to ansi escape codes.
 *
 *  \param  seq keyboard sequence.
 *
 *  \return None.
 */
/*************************************************************************************************/
uint8_t processEscSequence(uint8_t *seq)
{
    uint8_t retVal = 0;

    uint8_t arrows[4][3] = {
        /* Order of arrows here alligns with enum uptop */
        { 27, 91, 65 }, /* up arrow */

        { 27, 91, 66 }, /* down arrow */

        { 27, 91, 67 }, /* right arrow */

        { 27, 91, 68 }, /* left arrow */
    };

    /*arrows*/
    for (int arrowKey = 0; arrowKey < 4; arrowKey++) { //cycle through 4 arrow keys
        for (int i = 0; i < 3; i++) { //cycle thorugh each index of each arrow key sequence
            //compare each index of arrow key to each index of typed sequence buffer
            for (int j = 0; j < 3; j++) {
                if (seq[j] == arrows[arrowKey][i]) {
                    retVal++;
                    break;
                }
            }
        }
        /* If 3 motches found */
        if (retVal == 3)
            return arrowKey;
        else
            retVal = 0;
    }

    return 0xFF;
}
/*************************************************************************************************/
/*!
 *  \fn     cmdHistoryAdd.
 *
 *  \brief  adds latest command to command history buffer
 *
 *  \param  q pointer to the circular buffer holding command history 
 *  \param  cmd pointer to the command string to be added 
 *
 *  \return None.
 */
/*************************************************************************************************/
void cmdHistoryAdd(queue_t *q, const uint8_t *cmd)
{
    /* clear command history slot of any previous data */
    memset(&q->command[q->head].cmd, 0x00, CMD_LINE_BUF_SIZE);
    /* copy new command histroy */
    memcpy(&q->command[q->head], cmd, strlen((const char *)cmd));
    q->command[q->head].length = strlen((const char *)cmd);

    /* update head, and push tail up if we have looped back around */
    q->head = (q->head + 1) % HISTORY_MEMORY_LENGTH;
    if (q->head == q->tail) {
        q->tail = (q->tail + 1) % HISTORY_MEMORY_LENGTH;
    }
    memset(&q->command[q->head].cmd, 0x00, CMD_LINE_BUF_SIZE);
    /* update pointer */
    q->queuePointer = historyQueue.head;
}
/*************************************************************************************************/
/*!
 *  \fn     updateQueuePointer.
 *
 *  \brief  Updates an internal marker pointing to historical command to be printed, based on up/down arrow
 * 
 *  \param  q pointer to the circular buffer holding command history 
 * 
 *  \param  upArrow flag used to upated the queuePoniter delimiting which command to print
 *
 *  \return None.
 */
/*************************************************************************************************/
void updateQueuePointer(queue_t *q, bool upArrow)
{
    if (upArrow) {
        /* empty queue or reached tail already */
        if (q->head == q->tail || q->queuePointer == q->tail)
            return;
        if (q->queuePointer == 0)
            q->queuePointer = HISTORY_MEMORY_LENGTH - 1;
        else
            q->queuePointer = (q->queuePointer - 1) % HISTORY_MEMORY_LENGTH;

    } else {
        /* empty queue or reached head already */
        if (q->head == q->tail || q->queuePointer == q->head)
            return;
        q->queuePointer = (q->queuePointer + 1) % HISTORY_MEMORY_LENGTH;
    }
}
/*************************************************************************************************/
/*!
 *  \fn     printHistory.
 *
 *  \brief  prints previously typed commands
 *
 *  \param  upArrow flag used to upated the queuePoniter delimiting which command to print
 *
 *  \return None.
 */
/*************************************************************************************************/
void printHistory(bool upArrow)
{
    uint8_t numCharsToDelete = strlen(inputBuffer);
    uint8_t moveForwardCount = 0;
    updateQueuePointer(&historyQueue, upArrow);
    uint8_t right[] = "\x1b\x5b\x43";
    /* no history yet */
    if (historyQueue.queuePointer < 0)
        return;
    /* if cursor is not at the end then move it forward */
    if (bufferIndex != strlen(inputBuffer)) {
        moveForwardCount = strlen(inputBuffer) - bufferIndex;
        for (int i = 0; i < moveForwardCount; i++) {
            WsfBufIoWrite((const uint8_t *)right, sizeof(right));
            vTaskDelay(1); //give UART time to print
            bufferIndex++;
        }
    }
    /* send backspace to delete any currently typed text */
    if (numCharsToDelete) {
        for (int i = 0; i < numCharsToDelete; i++) printf("%s", backspace);
    }
    /* copy history into inputBuffer */
    memset(inputBuffer, 0x00, 100);
    memcpy(inputBuffer, historyQueue.command[historyQueue.queuePointer].cmd,
           strlen((const char *)historyQueue.command[historyQueue.queuePointer].cmd));
    printf("%s", inputBuffer);
    bufferIndex = strlen(inputBuffer);

    fflush(stdout);
}

/*************************************************************************************************/
/*!
 *  \fn     cls
 *
 *  \brief  Clears screen
 *
 *  \return None.
 */
/*************************************************************************************************/
void cls(void)
{
    char str[7];
    sprintf(str, "\033[2J");
    WsfBufIoWrite((const uint8_t *)str, 5);
    clearScreen = false;
}
/*************************************************************************************************/
/*!
 *  \fn     prompt
 *
 *  \brief  Prints prompt to screen, indicates if a test is active
 *
 *  \return None.
 */
/*************************************************************************************************/
void prompt(void)
{
    char str[25];
    uint8_t len = 0;

    if (pausePrompt)
        return;

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
    WsfBufIoWrite((const uint8_t *)str, len);
}
/*************************************************************************************************/
/*!
 *  \fn     printHint
 *
 *  \brief  Prints the help string of any command matching the current inputbuffer
 * 
 *  \param  buff pointer to the inputbuffer
 * 
 *  \return None.
 */
/*************************************************************************************************/
void printHint(char *buff)
{
    int i = 0;
    uint8_t bufflen = strlen((const char *)buff);
    bool foundMatch = false;
    do {
        if (memcmp(buff, xCommandList[i].pcCommand, bufflen) == 0 && bufflen > 0) {
            if (foundMatch == false)
                printf("\r\n");
            printf("\r\n> %s : %s", xCommandList[i].pcCommand, xCommandList[i].pcHelpString);
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
    static uint32_t i = 0;

    BaseType_t xHigherPriorityTaskWoken;
    // static uint8_t keyBoardSequenceBuff[3] = {0};
    receivedChar = rxByte;
    keyBoardSequenceBuff[i++ % 3] = rxByte;

    // TODO put all of this in command line task
    /* if received esc character start escape sequence counter */
    if (rxByte == 27)
        escCounter++;

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
#if (BT_VER > 9)
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
void vCmdLineTask(void *pvParameters)
{
    unsigned char tmp;
    unsigned int x;
    int uartReadLen;
    char output[OUTPUT_BUF_SIZE]; /* Buffer for output */
    BaseType_t xMore;
    mxc_uart_req_t async_read_req;
    memset(inputBuffer, 0, CMD_LINE_BUF_SIZE);
    bufferIndex = 0;

    /* Register available CLI commands */
    vRegisterCLICommands();
    vTaskDelay(100); /* give time for ll api to print things */
    fflush(stdout);
    /* clear screen and print help table */
    cls();
    pausePrompt = true;
    xTaskNotify(help_task_id, 0xFF, eSetBits);

    memset(cmd_history, 0, sizeof(cmd_history_t) * HISTORY_MEMORY_LENGTH);
    historyQueue.head = 0;
    historyQueue.tail = 0;
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        tmp = receivedChar;
        /* Check that we have a valid character */
        if (escCounter > 0 && escCounter < 4) {
            switch (processEscSequence(keyBoardSequenceBuff)) {
            case UP_ARROW:
                memset(keyBoardSequenceBuff, 0, 3);
                escCounter = 0;
                printHistory(true);
                break;

            case DOWN_ARROW:
                memset(keyBoardSequenceBuff, 0, 3);
                escCounter = 0;
                printHistory(false);
                break;

            case RIGHT_ARROW:
                memset(keyBoardSequenceBuff, 0, 3);
                escCounter = 0;
                if (bufferIndex < strlen(inputBuffer)) {
                    uint8_t right[] = "\x1b\x5b\x43";
                    WsfBufIoWrite((const uint8_t *)right, sizeof(right));
                    bufferIndex++;
                }
                escCounter = 0;

                break;
            case LEFT_ARROW:
                memset(keyBoardSequenceBuff, 0, 3);
                escCounter = 0;
                if (bufferIndex > 0) {
                    uint8_t left[] = "\x1b\x5b\x44";
                    WsfBufIoWrite((const uint8_t *)left, sizeof(left));
                    bufferIndex--;
                }
                break;

            default:
                escCounter++;
                break;
            }
        } else {
            escCounter = 0;
            if (async_read_req.rxCnt > 0) {
                /* Process character */
                do {
                    /* 0x08 BS linux , 127 Del windows/putty */
                    if (tmp == 0x08 || tmp == 127) {
                        /* Backspace */
                        if (bufferIndex > 0) {
                            bufferIndex--;
                            memset(&inputBuffer[bufferIndex], 0x00, 1);
                            printf("%s", backspace);
                        }
                        fflush(stdout);
                    } else if (tmp == 0x09)
                    /* tab hint */
                    {
                        printHint(inputBuffer);

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
                        bufferIndex = 0;
                        APP_TRACE_INFO0("^C");
                        prompt();
                    } else if ((tmp == '\r') || (tmp == '\n')) {
                        historyQueue.queuePointer = historyQueue.head;
                        if (strlen(inputBuffer) > 0) {
                            APP_TRACE_INFO0("\r\n");
                            /* save to history */
                            cmdHistoryAdd(&historyQueue, (const uint8_t *)inputBuffer);
                            /* Evaluate */
                            do {
                                xMore = FreeRTOS_CLIProcessCommand(inputBuffer, output,
                                                                   OUTPUT_BUF_SIZE);
                                for (x = 0;
                                     x < (xMore == pdTRUE ? OUTPUT_BUF_SIZE : strlen(output));
                                     x++) {
                                    putchar(*(output + x));
                                }
                            } while (xMore != pdFALSE);
                        }
                        /* New prompt */
                        bufferIndex = 0;
                        memset(inputBuffer, 0x00, 100);
                        prompt();
                    } else if (bufferIndex < CMD_LINE_BUF_SIZE) {
                        putchar(tmp);
                        inputBuffer[bufferIndex++] = tmp;
                        fflush(stdout);

                    } else {
                        /* Throw away data and beep terminal */
                        putchar(0x07);
                        fflush(stdout);
                    }
                    uartReadLen = 1;
                    /* If more characters are ready, process them here */
                } while ((MXC_UART_GetRXFIFOAvailable(MXC_UART_GET_UART(CONSOLE_UART)) > 0) &&
                         (MXC_UART_Read(ConsoleUART, (uint8_t *)&tmp, &uartReadLen) == 0));
            }
        }
    }
}
/*************************************************************************************************/
void txTestTask(void *pvParameters)
{
    static int res = 0xff;
    uint32_t notifVal = 0;
    tx_config_t testConfig;
    char str[80];
    while (1) {
        /* Wait for notification to initiate TX/RX */
        xTaskNotifyWait(0, 0xFFFFFFFF, &notifVal, portMAX_DELAY);
        /* Get test settings from the notification value */
        testConfig.allData = notifVal;

        if (testConfig.testType == BLE_TX_TEST) {
            sprintf(str, "Transmit RF channel %d on Freq %dMHz bytes/pkt : ", testConfig.channel,
                    getFreqFromRfChannel(testConfig.channel), packetLen);
            strcat(str, (const char *)getPacketTypeStr());
        } else {
            sprintf(str, "Receive RF channel %d Freq %dMHz: ", testConfig.channel,
                    getFreqFromRfChannel(testConfig.channel));
        }
        strcat(str, " : ");
        strcat(str, (const char *)getPhyStr(phy));
        APP_TRACE_INFO1("%s", str);

        /* stat test */
        if (testConfig.testType == BLE_TX_TEST) {
            res = LlEnhancedTxTest(testConfig.channel, packetLen, packetType, phy, 0);
        } else {
            res = LlEnhancedRxTest(testConfig.channel, phy, 0, 0);
        }
        APP_TRACE_INFO2("result = %u %s", res, res == LL_SUCCESS ? "(SUCCESS)" : "(FAIL)");
        /* if duration value was given then let the test run that amount of time and end */
        if (testConfig.duration_ms) {
            vTaskDelay(testConfig.duration_ms);
            LlEndTest(NULL);
            xSemaphoreGive(rfTestMutex);
        }
        pausePrompt = false;

        prompt();
    }
}
/*************************************************************************************************/
void sweepTestTask(void *pvParameters)
{
    uint32_t notifVal = 0;
    sweep_config_t sweepConfig;

    while (1) {
        /* Wait for notification to initiate sweep */
        xTaskNotifyWait(0, 0xFFFFFFFF, &notifVal, portMAX_DELAY);
        sweepConfig.allData = notifVal;
        APP_TRACE_INFO3("\r\nStarting TX sweep from Ch %d to Ch %d @ %d ms per channel",
                        sweepConfig.start_channel, sweepConfig.end_channel,
                        sweepConfig.duration_per_ch_ms);

        char str[6] = "";

        strcat(str, (const char *)getPhyStr(phy));
        /* sweep channels */
        for (int i = sweepConfig.start_channel; i <= sweepConfig.end_channel; i++) {
            APP_TRACE_INFO3(
                "\r\n-----------------| RF channel %d %s Freq: %dMHz |----------------------\r\n",
                i, str, getFreqFromRfChannel(i));

            LlEnhancedTxTest(i, packetLen, packetType, phy, 0);
            vTaskDelay(sweepConfig.duration_per_ch_ms);
            LlEndTest(NULL);
            xSemaphoreGive(rfTestMutex);
            vTaskDelay(50); /* give console time to print end of  test reuslts */
        }
        activeTest = NO_TEST;
        pausePrompt = false;
        prompt();
    }
}
/*************************************************************************************************/
void helpTask(void *pvParameters)
{
    uint32_t notifVal = 0;
    while (1) {
        xTaskNotifyWait(0, 0xFFFFFFFF, &notifVal, portMAX_DELAY);

        // clang-format off
    printf("┌─────────┬──────────────────────────────────┬───────────────────────────────────────────────────────┐\r\n");
    printf("│ Command │ parameters [optional] <required> │                      description                      │\r\n");
    printf("├─────────┼──────────────────────────────────┼───────────────────────────────────────────────────────┤\r\n");
    printf("│ cls     │ N/A                              │ clears the screen                                     │\r\n");
    printf("│         │                                  │                                                       │\r\n");
    printf("│ constTx │ <channel> <phy>                  │ Constant TX on given channel.                         │\r\n");
    printf("│         │ ex: constTx 0 1M                 │ (channel: 0-39 ) (phy: 1M 2M S2 S8)                   │\r\n");
    printf("│         │                                  │                                                       │\r\n");
    printf("│ e       │ N/A                              │ Ends any active RX/TX/Constant/Freq.hop RF test       │\r\n");
    printf("│         │                                  │                                                       │\r\n");
    printf("│ ps      │ N/A                              │ Display freeRTOS task stats                           │\r\n");
    printf("│         │                                  │                                                       │\r\n");
    printf("│ rx      │ <channel> <phy> <duration_ms>    │ RX test on given channel.                             │\r\n");
    printf("│         │ ex: rx 0 2M 500                  │ Duration of 0 is max duration until stopped           │\r\n");
    printf("│         │                                  │ (channel: 0-39 ) (phy: 1M 2M S2 S8)                   │\r\n");
    printf("│         │                                  │ (duaration in ms: 0 65535 )                           │\r\n");
    printf("│         │                                  │                                                       │\r\n");
    printf("│ sweep   │ <start_ch> <end_ch> <packet len> │ Sweeps TX tests through a range of RF channels given  │\r\n");
    printf("│         │ <packet_type> <phy> <ms/per_ch>  │ their order of appearance on the spectrum.            │\r\n");
    printf("│         │ ex: sweep 0 10 255 FF 2M 500     │ (channel: 0-39 ) (packet len: 0-255)                  │\r\n");
    printf("│         │                                  │ (packet type: PRBS9,PRBS15,00,FF,F0,0F,55,AA)         │\r\n");
    printf("│         │                                  │ (phy: 1M 2M S2 S8) (duaration in ms: 0 65535 )        │\r\n");
    printf("│         │                                  │                                                       │\r\n");
    printf("│ tx      │ <channel> <packet_len>           │ TX test on given RF channel.                          │\r\n");
    printf("│         │ <packet_type> <phy> <duartion>   │ Duration of 0 is max duration until stopped           │\r\n");
    printf("│         │ ex: tx 0 255 FF 2M 1000          │ (channel: 0-39 ) (packet len: 0-255)                  │\r\n");
    printf("│         │                                  │ (packet type: PRBS9,PRBS15,00,FF,F0,0F,55,AA)         │\r\n");
    printf("│         │                                  │ (phy: 1M 2M S2 S8) (duaration in ms: 0 65535 )        │\r\n");
    printf("│         │                                  │                                                       │\r\n");
    printf("│ power   │ <power>                          │ Select transmit power. Supported power levels (dBm)   │\r\n");
    printf("│         │ ex: power -10                    │ -10 , 0 , 4                                           │\r\n");
    printf("│         │                                  │                                                       │\r\n");
    printf("│ help    │ N/A                              │ Displays this help table                              │\r\n");
    printf("└─────────┴──────────────────────────────────┴───────────────────────────────────────────────────────┘\r\n");
        // clang-format on

        pausePrompt = false;
        prompt();
    }
}
/*************************************************************************************************/
void setPhy(uint8_t newPhy)
{
    phy = newPhy;
    char str[20] = "> Phy now set to ";
    strcat(str, (phy == LL_TEST_PHY_LE_1M)       ? "1M PHY" :
                (phy == LL_TEST_PHY_LE_2M)       ? "2M PHY" :
                (phy == LL_TEST_PHY_LE_CODED_S8) ? "S8 PHY" :
                (phy == LL_TEST_PHY_LE_CODED_S2) ? "S2 PHY" :
                                                   "");
    APP_TRACE_INFO1("%s", str);
}
/*************************************************************************************************/
void startFreqHopping(void)
{
    NVIC_EnableIRQ(TMR2_IRQn);
    MXC_TMR_TO_Start(MXC_TMR2, FREQ_HOP_PERIOD_US);
    MXC_TMR_EnableInt(MXC_TMR2);
} /*************************************************************************************************/
void setPacketLen(uint8_t len)
{
    packetLen = len;
    APP_TRACE_INFO1("> Packet length set to %d", len);
}
/*************************************************************************************************/
void setPacketType(uint8_t type)
{
    packetType = type;
    APP_TRACE_INFO1("> Packet type set to %s", getPacketTypeStr());
}
/*************************************************************************************************/
void setTxPower(int8_t power)
{
    // TODO : validate value
    txPower = power;
    llc_api_set_txpower((int8_t)power);
    LlSetAdvTxPower((int8_t)power);
    printf("> Power set to %d dBm\n", power);
}
/*************************************************************************************************/
void printConfigs(void)
{
    printf("-----| Current RF Configrations |-----\r\n");
    printf("Phy : %s \r\nPayload packet : %s\r\nPayload length : %d \r\nTX Power : %d dbm\r\n",
           getPhyStr(phy), getPacketTypeStr(), packetLen, txPower);
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
    LlSetBdAddr((uint8_t *)&bdAddr);

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
    xTaskCreate(vCmdLineTask, (const char *)"CmdLineTask",
                1024 + CMD_LINE_BUF_SIZE + OUTPUT_BUF_SIZE, NULL, tskIDLE_PRIORITY + 1,
                &cmd_task_id);
    // TX tranismit test task
    xTaskCreate(txTestTask, (const char *)"Tx Task", 1024, NULL, tskIDLE_PRIORITY + 1, &tx_task_id);

    // Sweep test task
    xTaskCreate(sweepTestTask, (const char *)"Sweep Task", 1024, NULL, tskIDLE_PRIORITY + 1,
                &sweep_task_id);

    // help task
    xTaskCreate(helpTask, (const char *)"Help Task", 1024, NULL, tskIDLE_PRIORITY + 1,
                &help_task_id);

    /* Start scheduler */
    APP_TRACE_INFO0(">> Starting scheduler.\r\n");

    vTaskStartScheduler();

    return 0;
}
/*************************************************************************************************/
/*!
 *  \brief  Calculates frequency of given RF channel
 *  \return Returns frequency of given RF channel
 */
/*************************************************************************************************/
uint16_t getFreqFromRfChannel(uint8_t ch)
{
    return 2402 + (ch * 2);
}