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

#include "rtc_regs.h"

extern int disable_tickless;

/*
 * Defines a command that returns a table showing the state of each task at the
 * time the command is called.
 */
static BaseType_t prvTaskStatsCommand(char* pcWriteBuffer, size_t xWriteBufferLen, const char* pcCommandString);

/*
 * Define a command which reports how long the scheduler has been operating (uptime)
 *
 */
static BaseType_t prvUptimeCommand(char* pcWriteBuffer, size_t xWriteBufferLen, const char* pcCommandString);

/*
 * Defines a command that expects exactly three parameters.  Each of the three
 * parameter are echoed back one at a time.
 */
static BaseType_t prvThreeParameterEchoCommand(char* pcWriteBuffer, size_t xWriteBufferLen, const char* pcCommandString);

/*
 * Defines a command that can take a variable number of parameters.  Each
 * parameter is echoes back one at a time.
 */
static BaseType_t prvParameterEchoCommand(char* pcWriteBuffer, size_t xWriteBufferLen, const char* pcCommandString);

/* Enable or disable tickless operation */
static BaseType_t prvTickless(char* pcWriteBuffer, size_t xWriteBufferLen, const char* pcCommandString);

/* Structure that defines the "ps" command line command. */
static const CLI_Command_Definition_t xTaskStats = {
    "ps", /* The command string to type. */
    "\r\nps:\r\n Displays a table showing the state of each FreeRTOS task\r\n\r\n",
    prvTaskStatsCommand, /* The function to run. */
    0 /* No parameters are expected. */
};

/* Structure that defines the "uptime" command line command. */
static const CLI_Command_Definition_t xUptime = {
    "uptime", /* The command string to type. */
    "\r\nuptime:\r\n Displays the uptime of the FreeRTOS system\r\n\r\n",
    prvUptimeCommand, /* The function to run. */
    0 /* No parameters are expected. */
};

/* Structure that defines the "tickless" command line command. */
static const CLI_Command_Definition_t xTickless = {
    "tickless", /* The command string to type. */
    "\r\ntickless <0/1>:\r\n Disable (0) or enable (1) tick-less operation\r\n\r\n",
    prvTickless, /* The function to run. */
    1 /* One parameter expected */
};

/* Structure that defines the "echo_3_parameters" command line command.  This
takes exactly three parameters that the command simply echos back one at a
time. */
static const CLI_Command_Definition_t xThreeParameterEcho = {
    "echo_3_parameters",
    "\r\necho_3_parameters <param1> <param2> <param3>:\r\n Expects three parameters, echos each in turn\r\n\r\n",
    prvThreeParameterEchoCommand, /* The function to run. */
    3 /* Three parameters are expected, which can take any value. */
};

/* Structure that defines the "echo_parameters" command line command.  This
takes a variable number of parameters that the command simply echos back one at
a time. */
static const CLI_Command_Definition_t xParameterEcho = {
    "echo_parameters",
    "\r\necho_parameters <...>:\r\n Take variable number of parameters, echos each in turn\r\n\r\n",
    prvParameterEchoCommand, /* The function to run. */
    -1 /* The user can enter any number of commands. */
};

/*-----------------------------------------------------------*/

void vRegisterCLICommands(void)
{
    /* Register all the command line commands defined immediately above. */
    FreeRTOS_CLIRegisterCommand(&xTaskStats);
    FreeRTOS_CLIRegisterCommand(&xUptime);
    FreeRTOS_CLIRegisterCommand(&xTickless);
    FreeRTOS_CLIRegisterCommand(&xThreeParameterEcho);
    FreeRTOS_CLIRegisterCommand(&xParameterEcho);
}
/*-----------------------------------------------------------*/

static BaseType_t prvTaskStatsCommand(char* pcWriteBuffer, size_t xWriteBufferLen, const char* pcCommandString)
{
    const char* const pcHeader = "Task          State  Priority  Stack	#\r\n************************************************\r\n";
    
    /* Remove compile time warnings about unused parameters, and check the
    write buffer is not NULL.  NOTE - for simplicity, this example assumes the
    write buffer length is adequate, so does not check for buffer overflows. */
    (void) pcCommandString;
    (void) xWriteBufferLen;
    configASSERT(pcWriteBuffer);
    
    /* Generate a table of task stats. */
    strcpy(pcWriteBuffer, pcHeader);
    vTaskList(pcWriteBuffer + strlen(pcHeader));
    
    /* There is no more data to return after this single string, so return
    pdFALSE. */
    return pdFALSE;
}
/*-----------------------------------------------------------*/

static BaseType_t prvUptimeCommand(char* pcWriteBuffer, size_t xWriteBufferLen, const char* pcCommandString)
{
    TickType_t ticks;
    
    ticks = xTaskGetTickCount();
    
#if configUSE_TICKLESS_IDLE
    pcWriteBuffer += snprintf(pcWriteBuffer, xWriteBufferLen,
                              "Uptime is 0x%08x (%u ms)\r\nMXC_RTCTMR->timer is %u\r\n",
                              ticks, ticks / portTICK_PERIOD_MS, MXC_RTC->sec);
#else
    pcWriteBuffer += snprintf(pcWriteBuffer, xWriteBufferLen,
                              "Uptime is 0x%08x (%u ms)\r\n",
                              ticks, ticks / portTICK_PERIOD_MS);
#endif
                              
    /* No more data to return */
    return pdFALSE;
}
/*-----------------------------------------------------------*/

static BaseType_t prvThreeParameterEchoCommand(char* pcWriteBuffer, size_t xWriteBufferLen, const char* pcCommandString)
{
    const char* pcParameter;
    BaseType_t lParameterStringLength, xReturn;
    static BaseType_t lParameterNumber = 0;
    
    /* Remove compile time warnings about unused parameters, and check the
    write buffer is not NULL.  NOTE - for simplicity, this example assumes the
    write buffer length is adequate, so does not check for buffer overflows. */
    (void) pcCommandString;
    (void) xWriteBufferLen;
    configASSERT(pcWriteBuffer);
    
    if (lParameterNumber == 0) {
        /* The first time the function is called after the command has been
        entered just a header string is returned. */
        sprintf(pcWriteBuffer, "The three parameters were:\r\n");
        
        /* Next time the function is called the first parameter will be echoed
        back. */
        lParameterNumber = 1L;
        
        /* There is more data to be returned as no parameters have been echoed
        back yet. */
        xReturn = pdPASS;
    }
    else {
        /* Obtain the parameter string. */
        pcParameter = FreeRTOS_CLIGetParameter
                      (
                          pcCommandString,        /* The command string itself. */
                          lParameterNumber,       /* Return the next parameter. */
                          &lParameterStringLength /* Store the parameter string length. */
                      );
                      
        /* Sanity check something was returned. */
        configASSERT(pcParameter);
        
        /* Return the parameter string. */
        memset(pcWriteBuffer, 0x00, xWriteBufferLen);
        sprintf(pcWriteBuffer, "%d: ", (int) lParameterNumber);
        strncat(pcWriteBuffer, pcParameter, lParameterStringLength);
        strcat(pcWriteBuffer, "\r\n");
        
        /* If this is the last of the three parameters then there are no more
        strings to return after this one. */
        if (lParameterNumber == 3L) {
            /* If this is the last of the three parameters then there are no more
            strings to return after this one. */
            xReturn = pdFALSE;
            lParameterNumber = 0L;
        }
        else {
            /* There are more parameters to return after this one. */
            xReturn = pdTRUE;
            lParameterNumber++;
        }
    }
    
    return xReturn;
}
/*-----------------------------------------------------------*/

static BaseType_t prvParameterEchoCommand(char* pcWriteBuffer, size_t xWriteBufferLen, const char* pcCommandString)
{
    const char* pcParameter;
    BaseType_t lParameterStringLength, xReturn;
    static BaseType_t lParameterNumber = 0;
    
    /* Remove compile time warnings about unused parameters, and check the
    write buffer is not NULL.  NOTE - for simplicity, this example assumes the
    write buffer length is adequate, so does not check for buffer overflows. */
    (void) pcCommandString;
    (void) xWriteBufferLen;
    configASSERT(pcWriteBuffer);
    
    if (lParameterNumber == 0) {
        /* The first time the function is called after the command has been
        entered just a header string is returned. */
        sprintf(pcWriteBuffer, "The parameters were:\r\n");
        
        /* Next time the function is called the first parameter will be echoed
        back. */
        lParameterNumber = 1L;
        
        /* There is more data to be returned as no parameters have been echoed
        back yet. */
        xReturn = pdPASS;
    }
    else {
        /* Obtain the parameter string. */
        pcParameter = FreeRTOS_CLIGetParameter
                      (
                          pcCommandString,        /* The command string itself. */
                          lParameterNumber,       /* Return the next parameter. */
                          &lParameterStringLength /* Store the parameter string length. */
                      );
                      
        if (pcParameter != NULL) {
            /* Return the parameter string. */
            memset(pcWriteBuffer, 0x00, xWriteBufferLen);
            sprintf(pcWriteBuffer, "%d: ", (int) lParameterNumber);
            strncat(pcWriteBuffer, pcParameter, lParameterStringLength);
            strcat(pcWriteBuffer, "\r\n");
            
            /* There might be more parameters to return after this one. */
            xReturn = pdTRUE;
            lParameterNumber++;
        }
        else {
            /* No more parameters were found.  Make sure the write buffer does
            not contain a valid string. */
            pcWriteBuffer[ 0 ] = 0x00;
            
            /* No more data to return. */
            xReturn = pdFALSE;
            
            /* Start over the next time this command is executed. */
            lParameterNumber = 0;
        }
    }
    
    return xReturn;
}

static BaseType_t prvTickless(char* pcWriteBuffer, size_t xWriteBufferLen, const char* pcCommandString)
{
    const char* pcParameter;
    BaseType_t lParameterStringLength;
    
    /* Get parameter */
    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParameterStringLength);
    
    if (pcParameter != NULL) {
        if (pcParameter[0] == '0') {
            disable_tickless = 1;
            pcWriteBuffer += snprintf(pcWriteBuffer, xWriteBufferLen, "Tick-less mode disabled\r\n");
        }
        else if (pcParameter[0] == '1') {
            pcWriteBuffer += snprintf(pcWriteBuffer, xWriteBufferLen, "Tick-less mode enabled\r\n");
            disable_tickless = 0;
        }
        else {
            pcWriteBuffer += snprintf(pcWriteBuffer, xWriteBufferLen, "Must supply 0 (Disable) or 1 (Enable)\r\n");
        }
    }
    else {
        pcWriteBuffer += snprintf(pcWriteBuffer, xWriteBufferLen, "Must supply 0 (Disable) or 1 (Enable)\r\n");
    }
    
    return pdFALSE;
}
