/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/

/* -------------------------------------------------- */
//                 INCLUDE GUARD
/* -------------------------------------------------- */
#ifndef EXAMPLES_MAX78000_SDHC_FTHR_INCLUDE_CLI_H_
#define EXAMPLES_MAX78000_SDHC_FTHR_INCLUDE_CLI_H_

/* -------------------------------------------------- */
//                      INCLUDES
/* -------------------------------------------------- */
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "board.h"
#include "led.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "uart.h"
#include "user-cli.h"

/* -------------------------------------------------- */
//                      MACROS
/* -------------------------------------------------- */
#define ENTER                   0X0D
#define NEW_LINE                0x0A
#define SPACE                   0x20
#define TAB                     0x09
#define BACKSPACE               0X08
#define MAXBUFF                 2000
#define DELETE                  0x7F
#define DOLLAR                  0x24


/* -------------------------------------------------- */
//                 FUNCTION PROTOTYPES
/* -------------------------------------------------- */
void line_accumlator(uint8_t user_char);

void process_command(char *input);

void User_Prompt_Sequence(void);

// Command table hander prototype with parameters
typedef void (*command_handler_t)(int, char *argv[]);

//Command table structure
typedef struct
{
	const char *name;
	command_handler_t handler;
	const char *help_string;
} command_table_t;

/*
 * This table is an array of command_table_t structures that defines a set of supported commands in the program.
 * Each command_table_t structure contains the name of the command, a function pointer to the corresponding command handler function,
 * and a short description of what the command does.
 *
 * The structure of this lookup table makes it trivially easy to add a new command to this command processor.
 */
extern const command_table_t commands[];

//Calculates the number of commands based on commands and the command table
extern const int num_commands;


#endif /* EXAMPLES_MAX78000_SDHC_FTHR_INCLUDE_CLI_H_ */