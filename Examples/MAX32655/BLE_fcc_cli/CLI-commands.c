
#include "main.h"

extern int disable_tickless;
extern TaskHandle_t tx_task_id;
extern TaskHandle_t cmd_task_id;
extern int longTestActive;
extern bool pausePrompt;
/***************************| Command protoypes |******************************/
static BaseType_t prvTaskStatsCommand(char* pcWriteBuffer, size_t xWriteBufferLen,
                                      const char* pcCommandString);

static BaseType_t cmd_PrintUsage(char* pcWriteBuffer, size_t xWriteBufferLen,
                                 const char* pcCommandString);

static BaseType_t cmd_StartTxTest(char* pcWriteBuffer, size_t xWriteBufferLen,
                                  const char* pcCommandString);

static BaseType_t cmd_StopTxTest(char* pcWriteBuffer, size_t xWriteBufferLen,
                                 const char* pcCommandString);

static BaseType_t cmd_SetPhy(char* pcWriteBuffer, size_t xWriteBufferLen,
                             const char* pcCommandString);

static BaseType_t cmd_StartRxTest(char* pcWriteBuffer, size_t xWriteBufferLen,
                                  const char* pcCommandString);

static BaseType_t cmd_StopRxTest(char* pcWriteBuffer, size_t xWriteBufferLen,
                                 const char* pcCommandString);

/***************************| Command structures |******************************/
/* Structure that defines the "ps" command line command. */
static const CLI_Command_Definition_t xTaskStats = {
    "ps", /* The command string to type. */
    "\r\nps:\r\n Displays a table showing the state of each FreeRTOS task\r\n\r\n",
    prvTaskStatsCommand, /* The function to run. */
    0                    /* No parameters are expected. */
};

/* Structure that defines the "usage" command line command. */
static const CLI_Command_Definition_t xPrintUsage = {
    "u",                                                         /* The command string to type. */
    "\r\nu:\r\n Displays FCC app usage\r\n\r\n", cmd_PrintUsage, /* The function to run. */
    0                                                            /* No parameters are expected. */
};
static const CLI_Command_Definition_t xStartTxTest = {
    "tx", "\r\ntx <channel> <optional: duration>:\r\n Performs TX test",
    cmd_StartTxTest, /* The function to run. */
    -1               /* Three parameters are expected, which can take any value. */
};
static const CLI_Command_Definition_t xStopTxTest = {
    "e", "\r\ne :\r\n Stops any active TX test", cmd_StopTxTest, /* The function to run. */
    -1 /* Three parameters are expected, which can take any value. */
};
static const CLI_Command_Definition_t xSetPhy = {
    "phy", "\r\nphy <param> :\r\n Sets Phy. Param: 1M 2M S8 S2 ",
    cmd_SetPhy, /* The function to run. */
    1           /* Three parameters are expected, which can take any value. */
};
/***************************| Command handlers |******************************/
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
static BaseType_t cmd_StartTxTest(char* pcWriteBuffer, size_t xWriteBufferLen,
                                  const char* pcCommandString)
{
    /* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
    (void)pcCommandString;
    (void)xWriteBufferLen;
    uint8_t channel;
    uint16_t testLen;
    tx_task_command_t notifyCommand;
    BaseType_t lParameterStringLength;
    uint8_t str[50];
    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
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

    // bad channel param
    //  TODO : currently if an alpha character is input it is equal to 0
    //because of atoi() behavior

    if (channel > 39 || channel < 0)
        return pdFALSE;

    notifyCommand.channel  = channel;
    notifyCommand.duration = testLen;
    notifyCommand.testType = TX_TEST;
    if (notifyCommand.duration == 0) {
        longTestActive = 1;
    }

    xTaskNotify(tx_task_id, notifyCommand.allData, eSetBits);
    pausePrompt = true;
    return pdFALSE;
}
/*-----------------------------------------------------------*/
static BaseType_t cmd_StopTxTest(char* pcWriteBuffer, size_t xWriteBufferLen,
                                 const char* pcCommandString)
{
    /* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
    (void)pcCommandString;
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);

    LlEndTest(NULL);
    longTestActive = 0;
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
void vRegisterCLICommands(void)
{
    /* Register all the command line commands defined immediately above. */
    FreeRTOS_CLIRegisterCommand(&xTaskStats);
    FreeRTOS_CLIRegisterCommand(&xPrintUsage);
    FreeRTOS_CLIRegisterCommand(&xStartTxTest);
    FreeRTOS_CLIRegisterCommand(&xStopTxTest);
    FreeRTOS_CLIRegisterCommand(&xSetPhy);
}
/*-----------------------------------------------------------*/