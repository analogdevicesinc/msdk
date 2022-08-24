/*
    FreeRTOS V8.2.1 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/* Modified by Maxim Integrated 26-Jun-2015 to quiet compiler warnings */
#include <string.h>
#include <stdio.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

/* FreeRTOS+CLI includes. */
#include "FreeRTOS_CLI.h"

#include "wut_regs.h"

extern int disable_tickless;

static BaseType_t prvTaskStatsCommand(char* pcWriteBuffer, size_t xWriteBufferLen,
                                      const char* pcCommandString);

static BaseType_t cmd_PrintUsage(char* pcWriteBuffer, size_t xWriteBufferLen,
                                 const char* pcCommandString);

static BaseType_t cmd_TxTest(char* pcWriteBuffer, size_t xWriteBufferLen,
                             const char* pcCommandString);

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
static const CLI_Command_Definition_t xTxTest = {
    "tx", "\r\ntx <param1> <param2>:\r\n Expects two parameters [channel] [length of test]",
    cmd_TxTest, /* The function to run. */
    -1          /* Three parameters are expected, which can take any value. */
};

/*-----------------------------------------------------------*/

void vRegisterCLICommands(void)
{
    /* Register all the command line commands defined immediately above. */
    FreeRTOS_CLIRegisterCommand(&xTaskStats);
    FreeRTOS_CLIRegisterCommand(&xPrintUsage);
    FreeRTOS_CLIRegisterCommand(&xTxTest);
}
/*-----------------------------------------------------------*/
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
    BaseType_t xReturn;
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
static BaseType_t cmd_TxTest(char* pcWriteBuffer, size_t xWriteBufferLen,
                             const char* pcCommandString)
{
    uint8_t channel;
    uint16_t testLen;

    BaseType_t lParameterStringLength, xReturn;
    static BaseType_t lParameterNumber = 0;

    /* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
    (void)pcCommandString;
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);
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

    // TODO : validate channel

    sprintf(pcWriteBuffer, "Transmitting on Channel[ %d ] ", channel);
    if (testLen) {
        char str[30];
        sprintf(str, "for [ %d ms ]", (uint16_t)testLen);
        strcat(pcWriteBuffer, str);
    }

    return pdFALSE;
}

/*-----------------------------------------------------------*/
