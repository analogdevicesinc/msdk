
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

extern test_t activeTest;
extern bool pausePrompt;
extern bool clearScreen;
extern xSemaphoreHandle rfTestMutex;

/*! \brief Physical layer functions. */
extern void llc_api_set_txpower(int8_t power);
extern void dbb_seq_select_rf_channel(uint32_t rf_channel);
extern void llc_api_tx_ldo_setup(void);
extern void dbb_seq_tx_enable(void);
extern void dbb_seq_tx_disable(void);

/* helpers */
static bool isDigit(const char* symbol, uint8_t len);
static bool paramsValid(const char* commandstring, uint8_t numOfParams);
static isChannel(const char* commandstring, uint8_t paramIndex);
/***************************| Command handler protoypes |******************************/
static BaseType_t cmd_configs(char* pcWriteBuffer, size_t xWriteBufferLen,
                              const char* pcCommandString);
static BaseType_t cmd_clearScreen(char* pcWriteBuffer, size_t xWriteBufferLen,
                                  const char* pcCommandString);
static BaseType_t prvTaskStatsCommand(char* pcWriteBuffer, size_t xWriteBufferLen,
                                      const char* pcCommandString);

static BaseType_t cmd_Help(char* pcWriteBuffer, size_t xWriteBufferLen,
                           const char* pcCommandString);

static BaseType_t cmd_StartBleRXTest(char* pcWriteBuffer, size_t xWriteBufferLen,
                                     const char* pcCommandString);

static BaseType_t cmd_StartBleTXTest(char* pcWriteBuffer, size_t xWriteBufferLen,
                                     const char* pcCommandString);

static BaseType_t cmd_StopBleRFTest(char* pcWriteBuffer, size_t xWriteBufferLen,
                                    const char* pcCommandString);

static BaseType_t cmd_SetPhy(char* pcWriteBuffer, size_t xWriteBufferLen,
                             const char* pcCommandString);

static BaseType_t cmd_SetPacketLen(char* pcWriteBuffer, size_t xWriteBufferLen,
                                   const char* pcCommandString);

static BaseType_t cmd_SetPacketType(char* pcWriteBuffer, size_t xWriteBufferLen,
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
const CLI_Command_Definition_t xCommandList[] = {
    {
        .pcCommand                   = "configs", /* The command string to type. */
        .pcHelpString                = "Displays RF configuration",
        .pxCommandInterpreter        = cmd_configs, /* The function to run. */
        .cExpectedNumberOfParameters = 0            /* No parameters are expected. */
    },
    {
        .pcCommand                   = "cls", /* The command string to type. */
        .pcHelpString                = "Clears screen",
        .pxCommandInterpreter        = cmd_clearScreen, /* The function to run. */
        .cExpectedNumberOfParameters = 0                /* No parameters are expected. */
    },
    {

        .pcCommand                   = "constTx", /* The command string to type. */
        .pcHelpString                = "<channel>",
        .pxCommandInterpreter        = cmd_ConstTx, /* The function to run. */
        .cExpectedNumberOfParameters = 1

    },
    {

        .pcCommand                   = "e", /* The command string to type. */
        .pcHelpString                = "Stops any active TX test",
        .pxCommandInterpreter        = cmd_StopBleRFTest, /* The function to run. */
        .cExpectedNumberOfParameters = 0

    },
    {

        .pcCommand                   = "fhop", /* The command string to type. */
        .pcHelpString                = "Starts frequency hopping",
        .pxCommandInterpreter        = cmd_EnableFreqHop, /* The function to run. */
        .cExpectedNumberOfParameters = 0

    },
    {
        .pcCommand                   = "ps", /* The command string to type. */
        .pcHelpString                = "Displays a table showing the state of each FreeRTOS task",
        .pxCommandInterpreter        = prvTaskStatsCommand, /* The function to run. */
        .cExpectedNumberOfParameters = 0                    /* No parameters are expected. */
    },
    {

        .pcCommand                   = "rx", /* The command string to type. */
        .pcHelpString                = "<channel> <phy> <duartion>",
        .pxCommandInterpreter        = cmd_StartBleRXTest, /* The function to run. */
        .cExpectedNumberOfParameters = 3

    },
    {

        .pcCommand                   = "sweep", /* The command string to type. */
        .pcHelpString                = "<start_ch> <end_ch> <ms/per_ch>",
        .pxCommandInterpreter        = cmd_Sweep, /* The function to run. */
        .cExpectedNumberOfParameters = 3

    },
    {

        .pcCommand                   = "tx", /* The command string to type. */
        .pcHelpString                = "<channel> <packet_length> <packet_type> <phy> <duration>",
        .pxCommandInterpreter        = cmd_StartBleTXTest, /* The function to run. */
        .cExpectedNumberOfParameters = 5

    },
    {

        .pcCommand                   = "dbm", /* The command string to type. */
        .pcHelpString                = "<dbm>",
        .pxCommandInterpreter        = cmd_SetTxdBm, /* The function to run. */
        .cExpectedNumberOfParameters = 1

    },
    {

        .pcCommand                   = "help", /* The command string to type. */
        .pcHelpString                = "Display help table",
        .pxCommandInterpreter        = cmd_Help, /* The function to run. */
        .cExpectedNumberOfParameters = 0

    },
    {

        .pcCommand = NULL /* simply used as delimeter for end of array*/
    }};
/***************************| Command handlers |******************************/
static BaseType_t cmd_configs(char* pcWriteBuffer, size_t xWriteBufferLen,
                              const char* pcCommandString)
{
    /* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
    (void)pcCommandString;
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    printConfigs();

    return pdFALSE;
}
static BaseType_t cmd_clearScreen(char* pcWriteBuffer, size_t xWriteBufferLen,
                                  const char* pcCommandString)
{
    /* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
    (void)pcCommandString;
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    clearScreen = true;

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
    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
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
    (void)pcCommandString;
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    pausePrompt = true;
    xTaskNotify(help_task_id, 0xFF, eSetBits);
    return pdFALSE;
}
/*-----------------------------------------------------------*/
static BaseType_t cmd_StartBleTXTest(char* pcWriteBuffer, size_t xWriteBufferLen,
                                     const char* pcCommandString)
{
    const char* temp;
    BaseType_t lParameterStringLength;
    uint16_t packetLen    = 0;
    uint8_t packetTypeVal = 0;
    uint8_t phyVal        = 0;
    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    int err = E_NO_ERROR;

    static tx_config_t rfCommand = {.testType = 0xFF, .duration_ms = 0};

    if (xSemaphoreTake(rfTestMutex, 0) == pdFALSE) {
        sprintf(pcWriteBuffer, "Another test is currently active\r\n");
        /* no point in doing anything else */
        return pdFALSE;
    }

    /* if parameters given are valid integers fetch them and verify*/
    if (isChannel(pcCommandString, 1) && err == E_NO_ERROR) {
        /* fetch channel parameter */
        rfCommand.channel = atoi(FreeRTOS_CLIGetParameter(
            pcCommandString,        /* The command string itself. */
            1,                      /* Return the next parameter. */
            &lParameterStringLength /* Store the parameter string length. */
            ));

        /* fetchc packet length parameter */
        const char* temp = FreeRTOS_CLIGetParameter(
            pcCommandString,        /* The command string itself. */
            2,                      /* Return the next parameter. */
            &lParameterStringLength /* Store the parameter string length. */
        );
        packetLen = atoi(temp);
        if (!isDigit(temp, lParameterStringLength) || packetLen > 255 || packetLen < 0)
            err++;

        /* fetch packet type parameter */
        const char* packetType = FreeRTOS_CLIGetParameter(
            pcCommandString,        /* The command string itself. */
            3,                      /* Return the next parameter. */
            &lParameterStringLength /* Store the parameter string length. */
        );
        (memcmp(packetType, "PRBS9", 5) == 0)  ? packetTypeVal  = LL_TEST_PKT_TYPE_PRBS9 :
        (memcmp(packetType, "PRBS15", 6) == 0) ? packetTypeVal = LL_TEST_PKT_TYPE_PRBS15 :
        (memcmp(packetType, "0F", 2) == 0)     ? packetTypeVal     = LL_TEST_PKT_TYPE_0F :
        (memcmp(packetType, "55", 2) == 0)     ? packetTypeVal     = LL_TEST_PKT_TYPE_55 :
        (memcmp(packetType, "FF", 2) == 0)     ? packetTypeVal     = LL_TEST_PKT_TYPE_FF :
        (memcmp(packetType, "00", 2) == 0)     ? packetTypeVal     = LL_TEST_PKT_TYPE_00 :
        (memcmp(packetType, "F0", 2) == 0)     ? packetTypeVal     = LL_TEST_PKT_TYPE_F0 :
        (memcmp(packetType, "AA", 2) == 0)     ? packetTypeVal     = LL_TEST_PKT_TYPE_AA :
                                                 err++;

        const char* newPhy = FreeRTOS_CLIGetParameter(
            pcCommandString,        /* The command string itself. */
            4,                      /* Return the next parameter. */
            &lParameterStringLength /* Store the parameter string length. */
        );

        (memcmp(newPhy, "1M", 2) == 0) ? phyVal = LL_TEST_PHY_LE_1M :
        (memcmp(newPhy, "1m", 2) == 0) ? phyVal = LL_TEST_PHY_LE_1M :
        (memcmp(newPhy, "2M", 2) == 0) ? phyVal = LL_TEST_PHY_LE_2M :
        (memcmp(newPhy, "2m", 2) == 0) ? phyVal = LL_TEST_PHY_LE_2M :
        (memcmp(newPhy, "S2", 2) == 0) ? phyVal = LL_TEST_PHY_LE_CODED_S2 :
        (memcmp(newPhy, "s2", 2) == 0) ? phyVal = LL_TEST_PHY_LE_CODED_S2 :
        (memcmp(newPhy, "S8", 2) == 0) ? phyVal = LL_TEST_PHY_LE_CODED_S8 :
        (memcmp(newPhy, "s8", 2) == 0) ? phyVal = LL_TEST_PHY_LE_CODED_S8 :
                                         err++;

        /* fetch duration parameter which is optional */
        rfCommand.duration_ms = atoi(FreeRTOS_CLIGetParameter(
            pcCommandString,        /* The command string itself. */
            5,                      /* Return the next parameter. */
            &lParameterStringLength /* Store the parameter string length. */
            ));

        rfCommand.testType = BLE_TX_TEST;
    } else {
        err++;
    }
    /* duration of 0 is infinite time , set the active test flag */
    if (rfCommand.duration_ms == 0 && err == E_NO_ERROR) {
        activeTest = rfCommand.testType;
    }
    if (err == E_NO_ERROR) {
        setPacketLen(packetLen);
        setPacketType(packetTypeVal);
        setPhy(phyVal);
        xTaskNotify(tx_task_id, rfCommand.allData, eSetBits);
        pausePrompt = true;
    } else {
        sprintf(pcWriteBuffer, "Bad parameter, see help menu for options\r\n");
        xSemaphoreGive(rfTestMutex);
    }
    return pdFALSE;
}
static BaseType_t cmd_StartBleRXTest(char* pcWriteBuffer, size_t xWriteBufferLen,
                                     const char* pcCommandString)
{
    const char* temp;
    BaseType_t lParameterStringLength;
    uint16_t packetLen    = 0;
    uint8_t packetTypeVal = 0;
    uint8_t phyVal        = 0;
    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    int err = E_NO_ERROR;

    static tx_config_t rfCommand = {.testType = 0xFF, .duration_ms = 0};

    if (xSemaphoreTake(rfTestMutex, 0) == pdFALSE) {
        sprintf(pcWriteBuffer, "Another test is currently active\r\n");
        /* no point in doing anything else */
        return pdFALSE;
    }

    /* if parameters given are valid integers fetch them and verify*/
    if (isChannel(pcCommandString, 1) && err == E_NO_ERROR) {
        /* fetch channel parameter */
        rfCommand.channel  = atoi(FreeRTOS_CLIGetParameter(
             pcCommandString,        /* The command string itself. */
             1,                      /* Return the next parameter. */
             &lParameterStringLength /* Store the parameter string length. */
             ));
        const char* newPhy = FreeRTOS_CLIGetParameter(
            pcCommandString,        /* The command string itself. */
            2,                      /* Return the next parameter. */
            &lParameterStringLength /* Store the parameter string length. */
        );

        (memcmp(newPhy, "1M", 2) == 0) ? phyVal = LL_TEST_PHY_LE_1M :
        (memcmp(newPhy, "1m", 2) == 0) ? phyVal = LL_TEST_PHY_LE_1M :
        (memcmp(newPhy, "2M", 2) == 0) ? phyVal = LL_TEST_PHY_LE_2M :
        (memcmp(newPhy, "2m", 2) == 0) ? phyVal = LL_TEST_PHY_LE_2M :
        (memcmp(newPhy, "S2", 2) == 0) ? phyVal = LL_TEST_PHY_LE_CODED_S2 :
        (memcmp(newPhy, "s2", 2) == 0) ? phyVal = LL_TEST_PHY_LE_CODED_S2 :
        (memcmp(newPhy, "S8", 2) == 0) ? phyVal = LL_TEST_PHY_LE_CODED_S8 :
        (memcmp(newPhy, "s8", 2) == 0) ? phyVal = LL_TEST_PHY_LE_CODED_S8 :
                                         err++;

        /* fetch duration parameter which is optional */
        rfCommand.duration_ms = atoi(FreeRTOS_CLIGetParameter(
            pcCommandString,        /* The command string itself. */
            3,                      /* Return the next parameter. */
            &lParameterStringLength /* Store the parameter string length. */
            ));

        rfCommand.testType = BLE_RX_TEST;
    } else {
        err++;
    }
    /* duration of 0 is infinite time , set the active test flag */
    if (rfCommand.duration_ms == 0 && err == E_NO_ERROR) {
        activeTest = rfCommand.testType;
    }
    if (err == E_NO_ERROR) {
        setPhy(phyVal);
        xTaskNotify(tx_task_id, rfCommand.allData, eSetBits);
        pausePrompt = true;
    } else {
        sprintf(pcWriteBuffer, "Bad parameter, see help menu for options\r\n");
        xSemaphoreGive(rfTestMutex);
    }
    return pdFALSE;
}
/*-----------------------------------------------------------*/
static BaseType_t cmd_StopBleRFTest(char* pcWriteBuffer, size_t xWriteBufferLen,
                                    const char* pcCommandString)
{
    (void)pcCommandString;
    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    sprintf(pcWriteBuffer, "Ending active tests\r\n");
    if (activeTest == BLE_CONST_TX) {
        /* Disable constant TX */
        MXC_R_TX_CTRL     = 0x2;
        MXC_R_PATTERN_GEN = 0x48;
        PalBbDisable();
    } else if (activeTest) {
        LlEndTest(NULL);
        MXC_TMR_Stop(MXC_TMR2);
    } else {
        sprintf(pcWriteBuffer, "No active test to disable\n");
    }
    activeTest  = NO_TEST;
    pausePrompt = false;
    xSemaphoreGive(rfTestMutex);
    return pdFALSE;
}
/*-----------------------------------------------------------*/
static BaseType_t cmd_SetPacketLen(char* pcWriteBuffer, size_t xWriteBufferLen,
                                   const char* pcCommandString)
{
    /* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    BaseType_t lParameterStringLength;
    uint16_t packetLen = 0;
    const char* temp =
        FreeRTOS_CLIGetParameter(pcCommandString,        /* The command string itself. */
                                 1,                      /* Return the next parameter. */
                                 &lParameterStringLength /* Store the parameter string length. */
        );

    packetLen = atoi(temp);
    if (isDigit(temp, lParameterStringLength) && packetLen <= 255 && packetLen > 0)
        setPacketLen(packetLen);
    else {
        sprintf(pcWriteBuffer, "Bad parameter, see help menu for options\r\n");
    }

    return pdFALSE;
}
/*-----------------------------------------------------------*/
static BaseType_t cmd_SetPacketType(char* pcWriteBuffer, size_t xWriteBufferLen,
                                    const char* pcCommandString)
{
    /* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    BaseType_t lParameterStringLength;

    const char* packetType =
        FreeRTOS_CLIGetParameter(pcCommandString,        /* The command string itself. */
                                 1,                      /* Return the next parameter. */
                                 &lParameterStringLength /* Store the parameter string length. */
        );

    (memcmp(packetType, "PRBS9", 5) == 0)  ? setPacketType(LL_TEST_PKT_TYPE_PRBS9) :
    (memcmp(packetType, "PRBS15", 6) == 0) ? setPacketType(LL_TEST_PKT_TYPE_PRBS15) :
    (memcmp(packetType, "0F", 2) == 0)     ? setPacketType(LL_TEST_PKT_TYPE_0F) :
    (memcmp(packetType, "55", 2) == 0)     ? setPacketType(LL_TEST_PKT_TYPE_55) :
    (memcmp(packetType, "FF", 2) == 0)     ? setPacketType(LL_TEST_PKT_TYPE_FF) :
    (memcmp(packetType, "00", 2) == 0)     ? setPacketType(LL_TEST_PKT_TYPE_00) :
    (memcmp(packetType, "F0", 2) == 0)     ? setPacketType(LL_TEST_PKT_TYPE_00) :
    (memcmp(packetType, "AA", 2) == 0)     ? setPacketType(LL_TEST_PKT_TYPE_AA) :
                                             sprintf(pcWriteBuffer,
                                                     "Bad parameter, see help menu for options\r\n");

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
                                     sprintf(pcWriteBuffer,
                                             "Bad parameter, see help menu for options\r\n");

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
    // TODO validate this number, need to know the range
    setTxPower(newTxdBm);

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
    int err                    = E_NO_ERROR;
    static uint32_t channelNum = 0xFF;
    BaseType_t lParameterStringLength;

    if (xSemaphoreTake(rfTestMutex, 0) == pdFALSE) {
        sprintf(pcWriteBuffer, "Another test is currently active\r\n");
        /* no point in doing anything else */
        return pdFALSE;
    }

    if (isChannel(pcCommandString, 1)) {
        channelNum = atoi(FreeRTOS_CLIGetParameter(
            pcCommandString,        /* The command string itself. */
            1,                      /* Return the next parameter. */
            &lParameterStringLength /* Store the parameter string length. */
            ));
    } else {
        err++;
    }
    /* start test */
    if (err == E_NO_ERROR) {
        dbb_seq_select_rf_channel(channelNum);
        strcat(pcWriteBuffer, "Starting TX\r\n");
        PalBbEnable();
        llc_api_tx_ldo_setup();

        /* Enable constant TX */
        MXC_R_TX_CTRL = 0x1;

        /* Enable pattern generator, set PRBS-9 */
        MXC_R_CONST_OUPUT = 0x0;
        MXC_R_PATTERN_GEN = 0x4B;
        activeTest        = BLE_CONST_TX;
    } else {
        sprintf(pcWriteBuffer, "Bad parameter, see help menu for options\r\n");
        xSemaphoreGive(rfTestMutex);
    }

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
    int err = E_NO_ERROR;
    if (xSemaphoreTake(rfTestMutex, 0) == pdFALSE) {
        sprintf(pcWriteBuffer, "Another test is currently active\r\n");
        /* no point in doing anything else */
        return pdFALSE;
    }

    if (err == E_NO_ERROR) {
        sprintf(pcWriteBuffer, "Starting frequency hopping\n");
        activeTest = BLE_FHOP_TEST;
        startFreqHopping();
    } else {
        xSemaphoreGive(rfTestMutex);
    }
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
    int err = E_NO_ERROR;
    BaseType_t lParameterStringLength;
    static sweep_config_t sweepConfig = {.duration_per_ch_ms = 0};

    if (xSemaphoreTake(rfTestMutex, 0) == pdFALSE) {
        sprintf(pcWriteBuffer, "Another test is currently active\r\n");
        /* no point in doing anything else */
        return pdFALSE;
    }

    /* save new parameters if they are valid*/
    if (paramsValid(pcCommandString, 3) && (err == E_NO_ERROR) && isChannel(pcCommandString, 1) &&
        isChannel(pcCommandString, 2)) {
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

        if (sweepConfig.start_channel == sweepConfig.end_channel) {
            sweepConfig.duration_per_ch_ms = 0;
            sprintf(pcWriteBuffer, "Bad parameter, see help menu for options\r\n");
            err++;
        }

    } else {
        err++;
    }

    //start sweep task
    if (err == E_NO_ERROR) {
        xTaskNotify(sweep_task_id, sweepConfig.allData, eSetBits);
        activeTest  = BLE_SWEEP_TEST;
        pausePrompt = true;
    } else {
        sprintf(pcWriteBuffer, "Bad parameter, see help menu for options\r\n");
        xSemaphoreGive(rfTestMutex);
    }

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
/*-----------------------------------------------------------*/
static isChannel(const char* commandstring, uint8_t paramIndex)
{
    const char* str;
    BaseType_t lParameterStringLength;
    uint8_t channel;
    /*params start at index 1*/
    str = FreeRTOS_CLIGetParameter(commandstring,          /* The command string itself. */
                                   paramIndex,             /* Return the next parameter. */
                                   &lParameterStringLength /* Store the parameter string length. */
    );
    /* if param exist verify it is a channel */
    if (str != NULL) {
        channel = atoi(str);
        if (!isDigit(str, lParameterStringLength) || (channel < 0) || (channel > 39)) {
            return false;
        }
    } else {
        return false;
    }
    return true;
}
/*-----------------------------------------------------------*/
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
