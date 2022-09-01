
#include "main.h"

/* Bluetooth DBB registers */
#define MXC_R_CONST_OUPUT *((volatile uint16_t*)(0x40052040))
#define MXC_R_PATTERN_GEN *((volatile uint16_t*)(0x4005203C))
#define MXC_R_TX_CTRL     *((volatile uint16_t*)(0x4005101C))

extern int disable_tickless;
extern TaskHandle_t tx_task_id;
extern TaskHandle_t cmd_task_id;
extern TaskHandle_t sweep_task_id;
extern TaskHandle_t help_task_id;
extern bool longTestActive;
extern bool pausePrompt;
extern bool clearScreen;
extern bool freqHopisActive;
/*! \brief Physical layer functions. */
extern void llc_api_set_txpower(int8_t power);
extern void dbb_seq_select_rf_channel(uint32_t rf_channel);
extern void llc_api_tx_ldo_setup(void);
extern void dbb_seq_tx_enable(void);
extern void dbb_seq_tx_disable(void);
bool constTxisActive = false;
CLI_Command_Definition_t registeredCommandList[10];

/* helpers */
static bool isDigit(const char* symbol, uint8_t len);
static bool paramsValid(const char* commandstring, uint8_t numOfParams);
static bool commandOnly(const char* commandstring);
static bool testIsActive(void);
/***************************| Command handler protoypes |******************************/
static BaseType_t cmd_clearScreen(char* pcWriteBuffer, size_t xWriteBufferLen,
                                  const char* pcCommandString);
static BaseType_t prvTaskStatsCommand(char* pcWriteBuffer, size_t xWriteBufferLen,
                                      const char* pcCommandString);

static BaseType_t cmd_Help(char* pcWriteBuffer, size_t xWriteBufferLen,
                           const char* pcCommandString);

static BaseType_t cmd_StartRFTest(char* pcWriteBuffer, size_t xWriteBufferLen,
                                  const char* pcCommandString);

static BaseType_t cmd_StopRFTest(char* pcWriteBuffer, size_t xWriteBufferLen,
                                 const char* pcCommandString);

static BaseType_t cmd_SetPhy(char* pcWriteBuffer, size_t xWriteBufferLen,
                             const char* pcCommandString);

static BaseType_t cmd_SetTxdBm(char* pcWriteBuffer, size_t xWriteBufferLen,
                               const char* pcCommandString);

static BaseType_t cmd_ConstTx(char* pcWriteBuffer, size_t xWriteBufferLen,
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

        .pcCommand                   = "constTx", /* The command string to type. */
        .pcHelpString                = "Enable constant TX.",
        .pxCommandInterpreter        = cmd_ConstTx, /* The function to run. */
        .cExpectedNumberOfParameters = -1

    },
    {

        .pcCommand                   = "e", /* The command string to type. */
        .pcHelpString                = "Stops any active TX test",
        .pxCommandInterpreter        = cmd_StopRFTest, /* The function to run. */
        .cExpectedNumberOfParameters = 0

    },
    {

        .pcCommand                   = "fhop", /* The command string to type. */
        .pcHelpString                = "Starts frequency hopping",
        .pxCommandInterpreter        = cmd_EnableFreqHop, /* The function to run. */
        .cExpectedNumberOfParameters = 0

    },
    {

        .pcCommand                   = "phy", /* The command string to type. */
        .pcHelpString                = "Sets Phy. Param: 1M 2M S8 S2 ",
        .pxCommandInterpreter        = cmd_SetPhy, /* The function to run. */
        .cExpectedNumberOfParameters = 1

    },
    {
        .pcCommand                   = "ps", /* The command string to type. */
        .pcHelpString                = "Displays a table showing the state of each FreeRTOS task",
        .pxCommandInterpreter        = prvTaskStatsCommand, /* The function to run. */
        .cExpectedNumberOfParameters = 0                    /* No parameters are expected. */
    },
    {

        .pcCommand                   = "rx", /* The command string to type. */
        .pcHelpString                = "Performs RX test on given channel",
        .pxCommandInterpreter        = cmd_StartRFTest, /* The function to run. */
        .cExpectedNumberOfParameters = -1

    },
    {

        .pcCommand                   = "sweep", /* The command string to type. */
        .pcHelpString                = "Sweeps channels at given interval ",
        .pxCommandInterpreter        = cmd_Sweep, /* The function to run. */
        .cExpectedNumberOfParameters = -1

    },
    {

        .pcCommand                   = "tx", /* The command string to type. */
        .pcHelpString                = "Performs TX test",
        .pxCommandInterpreter        = cmd_StartRFTest, /* The function to run. */
        .cExpectedNumberOfParameters = -1

    },
    {

        .pcCommand                   = "txdbm", /* The command string to type. */
        .pcHelpString                = "Sets transmit power",
        .pxCommandInterpreter        = cmd_SetTxdBm, /* The function to run. */
        .cExpectedNumberOfParameters = 1

    },
    {

        .pcCommand                   = "help", /* The command string to type. */
        .pcHelpString                = "Displays FCC app usage",
        .pxCommandInterpreter        = cmd_Help, /* The function to run. */
        .cExpectedNumberOfParameters = 0

    },
    {

        .pcCommand = NULL /* simply used as delimeter for end of array*/
    }};
/***************************| Command handlers |******************************/
static BaseType_t cmd_clearScreen(char* pcWriteBuffer, size_t xWriteBufferLen,
                                  const char* pcCommandString)
{
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
static BaseType_t cmd_Help(char* pcWriteBuffer, size_t xWriteBufferLen, const char* pcCommandString)
{
    /* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
    (void)pcCommandString;
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    pausePrompt = true;
    xTaskNotify(help_task_id, 0xFF, eSetBits);
    return pdFALSE;
}
/*-----------------------------------------------------------*/
static BaseType_t cmd_StartRFTest(char* pcWriteBuffer, size_t xWriteBufferLen,
                                  const char* pcCommandString)
{
    /* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
    (void)xWriteBufferLen;
    const char* temp;
    BaseType_t lParameterStringLength;
    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    static tx_config_t rfCommand = {.testType = 0xFF, .duration_ms = 0};

    if (testIsActive()) {
        sprintf(pcWriteBuffer, "Another test is currently active\r\n");
        return pdFALSE;
    }

    /*if no params given and sweepConfig still has its init value */
    if (commandOnly(pcCommandString) && rfCommand.testType == 0xFF) {
        sprintf(pcWriteBuffer, "You have to call the command at least once with parameters\r\n");
        return pdFALSE;
    }

    if (paramsValid(pcCommandString, 1)) {
        rfCommand.channel = atoi(FreeRTOS_CLIGetParameter(
            pcCommandString,        /* The command string itself. */
            1,                      /* Return the next parameter. */
            &lParameterStringLength /* Store the parameter string length. */
            ));
        if (rfCommand.channel > 39 || rfCommand.channel < 0) {
            sprintf(pcWriteBuffer, "Bad channel parameter\r\n");
            return pdFALSE;
        }
        /* get duration parameter which is optional */
        temp = FreeRTOS_CLIGetParameter(
            pcCommandString,        /* The command string itself. */
            2,                      /* Return the next parameter. */
            &lParameterStringLength /* Store the parameter string length. */
        );
        /* verify duration string */
        if (temp != NULL) {
            if (isDigit(temp, lParameterStringLength))
                rfCommand.duration_ms = atoi(temp);
            else {
                sprintf(pcWriteBuffer, "Bad duration parameter\r\n");
                return pdFALSE;
            }
        }
        /* check which command was called rx or tx */
        rfCommand.testType = (memcmp(pcCommandString, "tx", 2) == 0) ? TX_TEST : RX_TEST;

    }
    /* if parameters were given and are not valid */
    else if (!paramsValid(pcCommandString, 1) && !commandOnly(pcCommandString)) {
        sprintf(pcWriteBuffer, "Bad parameters given\r\n");
        return pdFALSE;
    }

    if (rfCommand.duration_ms == 0) {
        longTestActive = true;
    }

    xTaskNotify(tx_task_id, rfCommand.allData, eSetBits);
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
    configASSERT(pcWriteBuffer);

    sprintf(pcWriteBuffer, "Ending active tests\r\n");
    if (constTxisActive) {
        /* Disable constant TX */
        MXC_R_TX_CTRL     = 0x2;
        MXC_R_PATTERN_GEN = 0x48;
        PalBbDisable();
        constTxisActive = false;
    } else if (longTestActive) {
        LlEndTest(NULL);
        MXC_TMR_Stop(MXC_TMR2);
    } else {
        sprintf(pcWriteBuffer, "No active test to disable\n");
    }
    longTestActive = false;
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
static BaseType_t cmd_ConstTx(char* pcWriteBuffer, size_t xWriteBufferLen,
                              const char* pcCommandString)
{
    /* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
    (void)pcCommandString;
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    static uint32_t channelNum = 0xFF;
    BaseType_t lParameterStringLength;

    if (testIsActive()) {
        sprintf(pcWriteBuffer, "Another test is currently active\r\n");
        return pdFALSE;
    }

    const char* channelStr =
        FreeRTOS_CLIGetParameter(pcCommandString,        /* The command string itself. */
                                 1,                      /* Return the next parameter. */
                                 &lParameterStringLength /* Store the parameter string length. */
        );

    if (channelStr != NULL) {
        if (paramsValid(pcCommandString, 1))
            channelNum = atoi(channelStr);
        else {
            sprintf(pcWriteBuffer, "Bad parameters given\r\n");
            return pdFALSE;
        }
    } else {
        if (channelNum == 0xFF) {
            sprintf(pcWriteBuffer,
                    "You have to call the command at least once with parameters\r\n");
            return pdFALSE;
        }
    }

    dbb_seq_select_rf_channel(channelNum);
    //  sprintf(pcWriteBuffer, "Constant TX channel set to %d\r\n", channelNum);
    strcat(pcWriteBuffer, "Starting TX\r\n");
    PalBbEnable();
    llc_api_tx_ldo_setup();
    /* Enable constant TX */
    /* Enable constant TX */
    MXC_R_TX_CTRL = 0x1;

    /* Enable pattern generator, set PRBS-9 */
    MXC_R_CONST_OUPUT = 0x0;
    MXC_R_PATTERN_GEN = 0x4B;
    // dbb_seq_tx_enable();
    // dbb_seq_tx_enable();
    if (longTestActive | constTxisActive | freqHopisActive) {
        sprintf(pcWriteBuffer, "Another test is currently active\r\n");
        return pdFALSE;
    }
    longTestActive  = true;
    constTxisActive = true;

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

    if (testIsActive()) {
        sprintf(pcWriteBuffer, "Another test is currently active\r\n");
        return pdFALSE;
    }
    // TODO : validate value
    sprintf(pcWriteBuffer, "Starting frequency hopping\n");
    longTestActive  = true;
    freqHopisActive = true;
    startFreqHopping();
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
    BaseType_t lParameterStringLength;
    static sweep_config_t sweepConfig = {.duration_per_ch_ms = 0};

    if (testIsActive()) {
        sprintf(pcWriteBuffer, "Another test is currently active\r\n");
        return pdFALSE;
    }

    /*if no params given and sweepConfig still has its init value */
    if (commandOnly(pcCommandString) && sweepConfig.duration_per_ch_ms == 0) {
        sprintf(pcWriteBuffer, "You have to call the command at least once with parameters\r\n");
        return pdFALSE;
    }

    /* save new parameters if they are valid*/
    if (paramsValid(pcCommandString, 3)) {
        sweepConfig.start_channel = atoi(FreeRTOS_CLIGetParameter(
            pcCommandString,        /* The command string itself. */
            1,                      /* Return the next parameter. */
            &lParameterStringLength /* Store the parameter string length. */
            ));

        sweepConfig.end_channel = atoi(FreeRTOS_CLIGetParameter(
            pcCommandString,        /* The command string itself. */
            2,                      /* Return the next parameter. */
            &lParameterStringLength /* Store the parameter string length. */
            ));

        sweepConfig.duration_per_ch_ms = atoi(FreeRTOS_CLIGetParameter(
            pcCommandString,        /* The command string itself. */
            3,                      /* Return the next parameter. */
            &lParameterStringLength /* Store the parameter string length. */
            ));

    }
    /* if parameters were given and are not valid */
    else if (!paramsValid(pcCommandString, 3) && !commandOnly(pcCommandString)) {
        sprintf(pcWriteBuffer, "Bad parameters given\r\n");
        return pdFALSE;
    }

    //start sweep task
    xTaskNotify(sweep_task_id, sweepConfig.allData, eSetBits);
    longTestActive = true;
    pausePrompt    = true;
    return pdFALSE;
}
/*-----------------------------------------------------------*/
void vRegisterCLICommands(void)
{
    int i = 0;
    /* Register all the command line commands defined immediately above. */
    do {
        FreeRTOS_CLIRegisterCommand(&xCommandList[i++]);
    } while (xCommandList[i].pcCommand != NULL);
}
/*-----------------------------------------------------------*/
static bool isDigit(const char* symbol, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        if (!(symbol[i] >= 48 && symbol[i] <= 57)) {
            return false;
        }
    }

    return true;
}

static bool paramsValid(const char* commandstring, uint8_t numOfParams)
{
    const char* str;
    BaseType_t lParameterStringLength;
    /*params start at index 1*/
    for (int i = 1; i <= numOfParams; i++) {
        str = FreeRTOS_CLIGetParameter(
            commandstring,          /* The command string itself. */
            i,                      /* Return the next parameter. */
            &lParameterStringLength /* Store the parameter string length. */
        );
        /* if param exist verify it is a digit */
        if (str != NULL) {
            if (!isDigit(str, lParameterStringLength)) {
                return false;
            }
        } else {
            return false;
        }
    }

    return true;
}

static bool commandOnly(const char* commandstring)
{
    BaseType_t lParameterStringLength;
    const char* str;
    str = FreeRTOS_CLIGetParameter(commandstring,          /* The command string itself. */
                                   1,                      /* Return the next parameter. */
                                   &lParameterStringLength /* Store the parameter string length. */
    );
    if (str == NULL)
        return true;

    return false;
}
static bool testIsActive(void)
{
    if (longTestActive | constTxisActive | freqHopisActive) {
        return true;
    }
    return false;
}