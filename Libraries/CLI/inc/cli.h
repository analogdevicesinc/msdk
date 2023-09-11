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

/*! \file cli.h
    \brief A CLI Implementation using a command table to store command string, function pointer and a help string is dispatched.

    Details. 

    1.  Line Accumlator 

    Reads the incoming bytes
    Accumulates into a line buffer
    Echos char back to the emulator
    Handles backspace

    2.  Prcoess Command

    Processes input into a series of tokens
    All tokes are seperated by whitespace characters 
    Lookup first token in a table of functions
    Dispatch to handler functions 
    
*/

/* -------------------------------------------------- */
//                 INCLUDE GUARD
/* -------------------------------------------------- */
#ifndef MXC_CLI_H
#define MXC_CLI_H

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
#include "user-cli.h" // Change to user implementation header

/* -------------------------------------------------- */
//                 FUNCTION PROTOTYPES
/* -------------------------------------------------- */

/** Reads incoming bytes, Accumulate into line buffer, Echo's chars back to the other side and handles backspace.
 * 		  It calls the process_command function upon pressing the ENTER key
 *
 * @param   user_char       User input of each character
 */
void line_accumulator(uint8_t user_char);

/** Performs a Lexical analysis and tokenisis the user's commands
 *  Lookup first token in a table of functions, dispatch to handler function
 *
 * @param   input   Character pointer containing the line accumulator input when enter key is pressed 
 *
 * @return  void
 */
void process_command(char *input);

/** Prints the help string of each command from the command table
*
* @param argc The command element number within the command string
* 
* @param argv[] array of arguments storing different tokens of the command string in the same order as they were
*   passed in the command line.
*
* @return void
*/
void handle_help(int argc, char *argv[]);

/** Command table hander prototype with parameters
 * 
 * @param argc Used to determine which token element of the commands string is being used.
 * 
 * @param argv[] Char Array of tokens of the command string entered by the user.
 * */
typedef void (*command_handler_t)(int, char *argv[]);

/** This command table structure contains the name of the command, a function pointer to the corresponding command handler function,
 *  and the help string which provides a short description of what the command does.
 *
 * */
typedef struct {
    const char *name; /**< command string */
    command_handler_t handler; /**< function pointer of the handler function */
    const char *help_string; /**< help string of each command */
} command_table_t;

/** This table is an array of command_table_t structures which should be initialized by the user in user-cli.c to define a set of supported commands in the program.
 *  The structure of this lookup table makes it trivially easy to add a new command to this command processor.
 *
 * */
extern const command_table_t commands[];

/** Calculates the number of commands based on commands and the command table
 *
 * */
extern const int num_commands;

#endif /* MXC_CLI_H */