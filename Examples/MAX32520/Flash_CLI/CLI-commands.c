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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* FreeRTOS+CLI includes. */
#include "FreeRTOS_CLI.h"
#include "definitions.h"

#include "mxc_errors.h"

static char *getParamString(const char *commandString, size_t paramNo) {
  BaseType_t length;
  const char *param = FreeRTOS_CLIGetParameter(commandString, paramNo, &length);
  if (!param) return NULL;
  char *buff = (char *)pvPortMalloc(length + 1);
  strncpy(buff, param, length);
  buff[length] = 0;
  return buff;
}

static BaseType_t prvEraseCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                  const char *pcCommandString);

/* Structure that defines the "echo_parameters" command line command.  This
takes a variable number of parameters that the command simply echos back one at
a time. */
static const CLI_Command_Definition_t xEraseCommand = {
    "erase", "\r\nerase: \r\n Erases page in flash being operated on\r\n",
    prvEraseCommand, /* The function to run. */
    0                /* The user can enter any number of commands. */
};

/*-----------------------------------------------------------*/

static BaseType_t prvWriteCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                  const char *pcCommandString);

/* Structure that defines the "echo_parameters" command line command.  This
takes a variable number of parameters that the command simply echos back one at
a time. */
static const CLI_Command_Definition_t xWriteCommand = {
    "write",
    "\r\nwrite <word offset> <text>: \r\n Writes text string to flash starting at the 32-bit word in the flash page\n specified by \"word offset\" (e.g. word offset=3 -> address offset=0xC,\n word offset=4 -> address offset=0x10)\r\n",
    prvWriteCommand, /* The function to run. */
    2                /* The user can enter any number of commands. */
};

/*-----------------------------------------------------------*/

static BaseType_t prvReadCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                 const char *pcCommandString);

/* Structure that defines the "echo_parameters" command line command.  This
takes a variable number of parameters that the command simply echos back one at
a time. */
static const CLI_Command_Definition_t xReadCommand = {
    "read",
    "\r\nread <word offset> <number of letters>: \r\n Reads text from flash starting at the 32-bit word in the flash page\n specified by \"word offset\" (e.g. word offset=3 -> address offset=0xC,\n word offset=4 -> address offset=0x10)\r\n",
    prvReadCommand, /* The function to run. */
    2               /* The user can enter any number of commands. */
};

/*-----------------------------------------------------------*/

static BaseType_t prvCRCCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                const char *pcCommandString);

/* Structure that defines the "echo_parameters" command line command.  This
takes a variable number of parameters that the command simply echos back one at
a time. */
static const CLI_Command_Definition_t xCRCCommand = {
    "crc", "\r\ncrc: \r\n Calculates CRC of entire flash page\r\n",
    prvCRCCommand, /* The function to run. */
    0              /* The user can enter any number of commands. */
};
/*-----------------------------------------------------------*/

void vRegisterCLICommands(void) {
  /* Register all the command line commands defined immediately above. */
  FreeRTOS_CLIRegisterCommand(&xEraseCommand);
  FreeRTOS_CLIRegisterCommand(&xWriteCommand);
  FreeRTOS_CLIRegisterCommand(&xReadCommand);
  FreeRTOS_CLIRegisterCommand(&xCRCCommand);
}
/*-----------------------------------------------------------*/

static BaseType_t prvEraseCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                  const char *pcCommandString) {
  /* Remove compile time warnings about unused parameters, and check the
  write buffer is not NULL.  NOTE - for simplicity, this example assumes the
  write buffer length is adequate, so does not check for buffer overflows. */
  (void)pcCommandString;
  (void)xWriteBufferLen;
  configASSERT(pcWriteBuffer);
  if (!check_erased(FLASH_STORAGE_START_ADDR, MXC_FLASH_PAGE_SIZE)) {
    portENTER_CRITICAL();
    int retval = MXC_FLC_PageErase(FLASH_STORAGE_START_ADDR);
    portEXIT_CRITICAL();
    if (retval != E_NO_ERROR) {
      memset(pcWriteBuffer, 0x00, xWriteBufferLen);
      sprintf(pcWriteBuffer, "Erase failed with error %i\r\n", retval);
      return pdFALSE;
    }
  }
  memset(pcWriteBuffer, 0x00, xWriteBufferLen);
  sprintf(pcWriteBuffer, "Success\r\n");
  return pdFALSE;
}
/*-----------------------------------------------------------*/

static BaseType_t prvWriteCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                  const char *pcCommandString) {
  /* Remove compile time warnings about unused parameters, and check the
  write buffer is not NULL.  NOTE - for simplicity, this example assumes the
  write buffer length is adequate, so does not check for buffer overflows. */
  (void)pcCommandString;
  (void)xWriteBufferLen;
  configASSERT(pcWriteBuffer);

  const char *pcParameterOffset, *pcParameterText;

  /* Obtain the parameter string. */
  pcParameterOffset = getParamString(pcCommandString, 1);
  pcParameterText = getParamString(pcCommandString, 2);

  int offset = atoi(pcParameterOffset);
  uint32_t *data = (uint32_t *)pvPortMalloc(strlen(pcParameterText) * 4);
  for (int i = 0; i < strlen(pcParameterText); i++) {
    data[i] = (uint32_t)pcParameterText[i];
  }
  portENTER_CRITICAL();
  int retval = flash_write(FLASH_STORAGE_START_ADDR + offset * 4,
                             strlen(pcParameterText), data);
  portEXIT_CRITICAL();
  vPortFree(data);
  if (retval != E_NO_ERROR) {
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    sprintf(pcWriteBuffer, "Write failed with error %i\r\n", retval);
  } else {
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    sprintf(pcWriteBuffer, "Success\r\n");
  }

  return pdFALSE;
}
/*-----------------------------------------------------------*/

static BaseType_t prvReadCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                 const char *pcCommandString) {
  /* Remove compile time warnings about unused parameters, and check the
  write buffer is not NULL.  NOTE - for simplicity, this example assumes the
  write buffer length is adequate, so does not check for buffer overflows. */
  (void)pcCommandString;
  (void)xWriteBufferLen;
  configASSERT(pcWriteBuffer);

  const char *pcParameterOffset, *pcParameterLength;

  /* Obtain the parameter string. */
  pcParameterOffset = getParamString(pcCommandString, 1);
  pcParameterLength = getParamString(pcCommandString, 2);

  int offset = atoi(pcParameterOffset);
  int length = atoi(pcParameterLength);
  uint8_t *data = (uint8_t *)pvPortMalloc(length);

  int retval = flash_read(FLASH_STORAGE_START_ADDR + offset * 4, length, data);
  if (retval != E_NO_ERROR) {
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    sprintf(pcWriteBuffer, "Read failed with error %i\r\n", retval);
  } else {
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    sprintf(pcWriteBuffer, "Success: \r\n");
    strncat(pcWriteBuffer, (char *)data, length);
    strcat(pcWriteBuffer, "\r\n");
  }
  vPortFree(data);

  return pdFALSE;
}
/*-----------------------------------------------------------*/

static BaseType_t prvCRCCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                const char *pcCommandString) {
  /* Remove compile time warnings about unused parameters, and check the
  write buffer is not NULL.  NOTE - for simplicity, this example assumes the
  write buffer length is adequate, so does not check for buffer overflows. */
  (void)pcCommandString;
  (void)xWriteBufferLen;
  configASSERT(pcWriteBuffer);

  portENTER_CRITICAL();
  uint32_t crc = calculate_crc((uint32_t *)FLASH_STORAGE_START_ADDR,
                               MXC_FLASH_PAGE_SIZE / sizeof(uint32_t));
  portEXIT_CRITICAL();
  memset(pcWriteBuffer, 0x00, xWriteBufferLen);
  sprintf(pcWriteBuffer, "CRC: 0x%08X\r\n", crc);
  return pdFALSE;
}
/*-----------------------------------------------------------*/
