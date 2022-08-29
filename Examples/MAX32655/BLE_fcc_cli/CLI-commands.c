
#include "main.h"

#define NUM_OF_COMMANDS 12
extern int disable_tickless;
extern TaskHandle_t tx_task_id;
extern TaskHandle_t cmd_task_id;
extern TaskHandle_t sweep_task_id;
extern bool longTestActive;
extern bool pausePrompt;
extern bool clearScreen;
/*! \brief Physical layer functions. */
extern void llc_api_set_txpower(int8_t power);
extern void dbb_seq_select_rf_channel(uint32_t rf_channel);
extern void llc_api_tx_ldo_setup(void);
extern void dbb_seq_tx_enable(void);
extern void dbb_seq_tx_disable(void);

CLI_Command_Definition_t registeredCommandList[10];

/***************************| Command handler protoypes |******************************/
static BaseType_t cmd_clearScreen(char* pcWriteBuffer, size_t xWriteBufferLen,
                                  const char* pcCommandString);
static BaseType_t prvTaskStatsCommand(char* pcWriteBuffer, size_t xWriteBufferLen,
                                      const char* pcCommandString);

static BaseType_t cmd_PrintUsage(char* pcWriteBuffer, size_t xWriteBufferLen,
                                 const char* pcCommandString);

static BaseType_t cmd_StartRFTest(char* pcWriteBuffer, size_t xWriteBufferLen,
                                  const char* pcCommandString);

static BaseType_t cmd_StopRFTest(char* pcWriteBuffer, size_t xWriteBufferLen,
                                 const char* pcCommandString);

static BaseType_t cmd_SetPhy(char* pcWriteBuffer, size_t xWriteBufferLen,
                             const char* pcCommandString);

static BaseType_t cmd_SetTxdBm(char* pcWriteBuffer, size_t xWriteBufferLen,
                               const char* pcCommandString);

static BaseType_t cmd_EnableConstTx(char* pcWriteBuffer, size_t xWriteBufferLen,
                                    const char* pcCommandString);

static BaseType_t cmd_DisableConstTx(char* pcWriteBuffer, size_t xWriteBufferLen,
                                     const char* pcCommandString);

static BaseType_t cmd_EnableFreqHop(char* pcWriteBuffer, size_t xWriteBufferLen,
                                    const char* pcCommandString);

static BaseType_t cmd_Sweep(char* pcWriteBuffer, size_t xWriteBufferLen,
                            const char* pcCommandString);

/***************************| Command structures |******************************/
/* Structure that defines the "ps" command line command. */
static const CLI_Command_Definition_t xCommandList[] = {
    {
        .pcCommand                   = "cls", /* The command string to type. */
        .pcHelpString                = "Clears screen",
        .pxCommandInterpreter        = cmd_clearScreen, /* The function to run. */
        .cExpectedNumberOfParameters = 0                /* No parameters are expected. */
    },
    {
        .pcCommand                   = "ps", /* The command string to type. */
        .pcHelpString                = "Displays a table showing the state of each FreeRTOS task",
        .pxCommandInterpreter        = prvTaskStatsCommand, /* The function to run. */
        .cExpectedNumberOfParameters = 0                    /* No parameters are expected. */
    },
    {

        .pcCommand                   = "u", /* The command string to type. */
        .pcHelpString                = "Displays FCC app usage",
        .pxCommandInterpreter        = cmd_PrintUsage, /* The function to run. */
        .cExpectedNumberOfParameters = 0

    },
    {

        .pcCommand                   = "tx", /* The command string to type. */
        .pcHelpString                = "tx tx <channel> <optional: duration>: Performs TX test",
        .pxCommandInterpreter        = cmd_StartRFTest, /* The function to run. */
        .cExpectedNumberOfParameters = -1

    },
    {

        .pcCommand                   = "rx", /* The command string to type. */
        .pcHelpString                = "<channel> <optional: duration>:Performs RX test",
        .pxCommandInterpreter        = cmd_StartRFTest, /* The function to run. */
        .cExpectedNumberOfParameters = -1

    },
    {

        .pcCommand                   = "e", /* The command string to type. */
        .pcHelpString                = "Stops any active TX test",
        .pxCommandInterpreter        = cmd_StopRFTest, /* The function to run. */
        .cExpectedNumberOfParameters = 0

    },
    {

        .pcCommand                   = "phy", /* The command string to type. */
        .pcHelpString                = "<param> : Sets Phy. Param: 1M 2M S8 S2 ",
        .pxCommandInterpreter        = cmd_SetPhy, /* The function to run. */
        .cExpectedNumberOfParameters = 1

    },
    {

        .pcCommand                   = "txdbm", /* The command string to type. */
        .pcHelpString                = "<param> : Sets transmit power.",
        .pxCommandInterpreter        = cmd_SetTxdBm, /* The function to run. */
        .cExpectedNumberOfParameters = 1

    },
    {

        .pcCommand                   = "constTx on", /* The command string to type. */
        .pcHelpString                = "<param> : Enable constant TX.",
        .pxCommandInterpreter        = cmd_EnableConstTx, /* The function to run. */
        .cExpectedNumberOfParameters = 1

    },
    {

        .pcCommand                   = "constTx off", /* The command string to type. */
        .pcHelpString                = "Disable constant TX.",
        .pxCommandInterpreter        = cmd_DisableConstTx, /* The function to run. */
        .cExpectedNumberOfParameters = 0

    },
    {

        .pcCommand                   = "fhop", /* The command string to type. */
        .pcHelpString                = "Starts frequency hopping",
        .pxCommandInterpreter        = cmd_EnableFreqHop, /* The function to run. */
        .cExpectedNumberOfParameters = 0

    },
    {

        .pcCommand                   = "sweep", /* The command string to type. */
        .pcHelpString                = "<param> <param> <param> sweeps channels at given intervals",
        .pxCommandInterpreter        = cmd_Sweep, /* The function to run. */
        .cExpectedNumberOfParameters = -1

    }

};
/***************************| Command handlers |******************************/
static BaseType_t cmd_clearScreen(char* pcWriteBuffer, size_t xWriteBufferLen,
                                  const char* pcCommandString)
{
    const char* const pcHeader = "Task          State  Priority  Stack	"
                                 "#\r\n************************************************\r\n";

    /* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
    (void)pcCommandString;
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);
    clearScreen = true;
    /* There is no more data to return after this single string, so return
	pdFALSE. */
    return pdFALSE;
}
static BaseType_t prvTaskStatsCommand(char* pcWriteBuffer, size_t xWriteBufferLen,
                                      const char* pcCommandString)
{
    const char* const pcHeader = "Task          State  Priority  Stack	"
                                 "#\r\n************************************************\r\n";

    /* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
    (void)pcCommandString;
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);

    /* Generate a table of task stats. */
    strcpy(pcWriteBuffer, pcHeader);
    vTaskList(pcWriteBuffer + strlen(pcHeader));

    /* There is no more data to return after this single string, so return
	pdFALSE. */
    return pdFALSE;
}
/*-----------------------------------------------------------*/
static BaseType_t cmd_PrintUsage(char* pcWriteBuffer, size_t xWriteBufferLen,
                                 const char* pcCommandString)
{
    sprintf(pcWriteBuffer, "Usage: \r\n");
    strcat(pcWriteBuffer, " (0) Transmit on RF channel 0 (2402 MHz)\r\n");
    strcat(pcWriteBuffer, " (1) Transmit on RF channel 19 (2440 MHz)\r\n");
    strcat(pcWriteBuffer, " (2) Transmit on RF channel 39 (2480 MHz)\r\n");
    strcat(pcWriteBuffer, " (3) Receive  on RF channel 39 (2480 MHz)\r\n");
    strcat(pcWriteBuffer, " (4) Set Transmit power\r\n");
    strcat(pcWriteBuffer, " (5) Enable constant TX\r\n");
    strcat(pcWriteBuffer, " (6) Disable constant TX -- MUST be called after (5)\r\n");
    strcat(pcWriteBuffer, " (8) Set PHY\r\n");
    strcat(pcWriteBuffer, " (9) TX Frequency Hop\r\n");
    strcat(pcWriteBuffer, " (e) End transmission -- MUST be used after each (0-3, 9)\r\n");
    strcat(pcWriteBuffer, " (u) Print usage\r\n");
    return pdFALSE;
}
/*-----------------------------------------------------------*/
static BaseType_t cmd_StartRFTest(char* pcWriteBuffer, size_t xWriteBufferLen,
                                  const char* pcCommandString)
{
    /* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
    (void)pcCommandString;
    (void)xWriteBufferLen;
    uint8_t channel;
    uint16_t testLen;
    tx_config_t txCommand;
    BaseType_t lParameterStringLength;
    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);

    //  TODO : validate  channel ignore letters
    channel = atoi(
        FreeRTOS_CLIGetParameter(pcCommandString,        /* The command string itself. */
                                 1,                      /* Return the next parameter. */
                                 &lParameterStringLength /* Store the parameter string length. */
                                 ));

    testLen = atoi(
        FreeRTOS_CLIGetParameter(pcCommandString,        /* The command string itself. */
                                 2,                      /* Return the next parameter. */
                                 &lParameterStringLength /* Store the parameter string length. */
                                 ));

    if (channel > 39 || channel < 0)
        return pdFALSE;

    txCommand.channel     = channel;
    txCommand.duration_ms = testLen;
    txCommand.testType    = (memcmp(pcCommandString, "tx", 2) == 0) ? TX_TEST : RX_TEST;

    if (txCommand.duration_ms == 0) {
        longTestActive = true;
    }

    xTaskNotify(tx_task_id, txCommand.allData, eSetBits);
    pausePrompt = true;
    return pdFALSE;
}
/*-----------------------------------------------------------*/
static BaseType_t cmd_StopRFTest(char* pcWriteBuffer, size_t xWriteBufferLen,
                                 const char* pcCommandString)
{
    /* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
    (void)pcCommandString;
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);

    sprintf(pcWriteBuffer, "Ending active tests\r\n");
    LlEndTest(NULL);
    MXC_TMR_Stop(MXC_TMR2);
    longTestActive = false;
    pausePrompt    = false;
    return pdFALSE;
}
/*-----------------------------------------------------------*/
static BaseType_t cmd_SetPhy(char* pcWriteBuffer, size_t xWriteBufferLen,
                             const char* pcCommandString)
{
    /* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
    (void)pcCommandString;
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    BaseType_t lParameterStringLength;

    const char* newPhy =
        FreeRTOS_CLIGetParameter(pcCommandString,        /* The command string itself. */
                                 1,                      /* Return the next parameter. */
                                 &lParameterStringLength /* Store the parameter string length. */
        );

    (memcmp(newPhy, "1M", 2) == 0) ? setPhy(LL_TEST_PHY_LE_1M) :
    (memcmp(newPhy, "1m", 2) == 0) ? setPhy(LL_TEST_PHY_LE_1M) :
    (memcmp(newPhy, "2M", 2) == 0) ? setPhy(LL_TEST_PHY_LE_2M) :
    (memcmp(newPhy, "2m", 2) == 0) ? setPhy(LL_TEST_PHY_LE_2M) :
    (memcmp(newPhy, "S2", 2) == 0) ? setPhy(LL_TEST_PHY_LE_CODED_S2) :
    (memcmp(newPhy, "s2", 2) == 0) ? setPhy(LL_TEST_PHY_LE_CODED_S2) :
    (memcmp(newPhy, "S8", 2) == 0) ? setPhy(LL_TEST_PHY_LE_CODED_S8) :
    (memcmp(newPhy, "s8", 2) == 0) ? setPhy(LL_TEST_PHY_LE_CODED_S8) :
                                     sprintf(pcWriteBuffer, "Bad param\r\n");

    return pdFALSE;
}
/*-----------------------------------------------------------*/
static BaseType_t cmd_SetTxdBm(char* pcWriteBuffer, size_t xWriteBufferLen,
                               const char* pcCommandString)
{
    /* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
    (void)pcCommandString;
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    BaseType_t lParameterStringLength;

    int8_t newTxdBm = atoi(
        FreeRTOS_CLIGetParameter(pcCommandString,        /* The command string itself. */
                                 1,                      /* Return the next parameter. */
                                 &lParameterStringLength /* Store the parameter string length. */
                                 ));
    // TODO : validate value
    llc_api_set_txpower((int8_t)newTxdBm);
    LlSetAdvTxPower((int8_t)newTxdBm);
    sprintf(pcWriteBuffer, "Power set to %d dBm\n", newTxdBm);

    return pdFALSE;
}
/*-----------------------------------------------------------*/
static BaseType_t cmd_EnableConstTx(char* pcWriteBuffer, size_t xWriteBufferLen,
                                    const char* pcCommandString)
{
    /* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
    (void)pcCommandString;
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    BaseType_t lParameterStringLength;

    uint32_t channel = (uint32_t)atoi(
        FreeRTOS_CLIGetParameter(pcCommandString,        /* The command string itself. */
                                 1,                      /* Return the next parameter. */
                                 &lParameterStringLength /* Store the parameter string length. */
                                 ));
    // TODO : validate value
    dbb_seq_select_rf_channel(channel);
    sprintf(pcWriteBuffer, "Constant TX channel set to %d\n", channel);
    sprintf(pcWriteBuffer, "Starting TX\n");
    PalBbEnable();
    llc_api_tx_ldo_setup();
    /* Enable constant TX */
    dbb_seq_tx_enable();

    longTestActive = true;
    return pdFALSE;
}
/*-----------------------------------------------------------*/
static BaseType_t cmd_DisableConstTx(char* pcWriteBuffer, size_t xWriteBufferLen,
                                     const char* pcCommandString)
{
    /* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
    (void)pcCommandString;
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);

    // TODO : validate value
    sprintf(pcWriteBuffer, "Disabling TX\n");
    /* Disable constant TX */
    dbb_seq_tx_disable();
    PalBbDisable();
    longTestActive = false;
    return pdFALSE;
}
/*-----------------------------------------------------------*/
static BaseType_t cmd_EnableFreqHop(char* pcWriteBuffer, size_t xWriteBufferLen,
                                    const char* pcCommandString)
{
    /* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
    (void)pcCommandString;
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);

    // TODO : validate value
    sprintf(pcWriteBuffer, "Starting frequency hopping\n");
    startFreqHopping();
    longTestActive = true;
    return pdFALSE;
}
/*-----------------------------------------------------------*/
static BaseType_t cmd_Sweep(char* pcWriteBuffer, size_t xWriteBufferLen,
                            const char* pcCommandString)
{
    /* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
    (void)pcCommandString;
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    sweep_config_t sweepConfig;
    BaseType_t lParameterStringLength;

    uint8_t start, end, duration;
    sweepConfig.start_channel = atoi(
        FreeRTOS_CLIGetParameter(pcCommandString,        /* The command string itself. */
                                 1,                      /* Return the next parameter. */
                                 &lParameterStringLength /* Store the parameter string length. */
                                 ));
    sweepConfig.end_channel = atoi(
        FreeRTOS_CLIGetParameter(pcCommandString,        /* The command string itself. */
                                 2,                      /* Return the next parameter. */
                                 &lParameterStringLength /* Store the parameter string length. */
                                 ));

    sweepConfig.duration_per_ch_ms = atoi(
        FreeRTOS_CLIGetParameter(pcCommandString,        /* The command string itself. */
                                 3,                      /* Return the next parameter. */
                                 &lParameterStringLength /* Store the parameter string length. */
                                 ));

    //start sweep task
    xTaskNotify(sweep_task_id, sweepConfig.allData, eSetBits);
    longTestActive = true;
    pausePrompt    = true;
    return pdFALSE;
}
/*-----------------------------------------------------------*/
void vRegisterCLICommands(void)
{
    /* Register all the command line commands defined immediately above. */
    for (int i = 0; i < NUM_OF_COMMANDS; i++) {
        FreeRTOS_CLIRegisterCommand(&xCommandList[i]);
    }
}
/*-----------------------------------------------------------*/