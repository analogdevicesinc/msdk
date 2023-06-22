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
#include "sdhc.h"
#include "led.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "uart.h"
#include "sdhc.h"

/* -------------------------------------------------- */
//                      MACROS
/* -------------------------------------------------- */
#define ENTER 0X0D
#define SPACE 0x20
#define TAB 0x09
#define BACKSPACE 0X08
#define MAXBUFF 2000
#define DELETE 0x7F

void line_accumlator(uint8_t user_char);

// Command table hander prototype with parameters
typedef void (*command_handler_t)(int, char *argv[]);

//Command table structure
typedef struct
{
	const char *name;
	command_handler_t handler;
	const char *help_string;
} command_table_t;


/* -------------------------------------------------- */
//             FUNCTION PROTOTYPES
/* -------------------------------------------------- */
void handle_size(int argc, char *argv[]);

void handle_format(int argc, char *argv[]);

void hande_mount(int argc, char *argv[]);

void handle_ls(int argc, char *argv[]);

void handle_mkdir(int argc, char *argv[]);

void handle_createfile(int argc, char *argv[]);

void handle_cd(int argc, char *argv[]);

void handle_add_data(int argc, char *argv[]);

void handle_del(int argc, char *argv[]);

void handle_fatfs(int argc, char *argv[]);

void handle_unmount(int argc, char *argv[]);

void handle_help(int argc, char *argv[]);

void process_command(char *input);

/*
 * This table is an array of command_table_t structures that defines a set of supported commands in the program.
 * Each command_table_t structure contains the name of the command, a function pointer to the corresponding command handler function,
 * and a short description of what the command does.
 *
 * The structure of this lookup table makes it trivially easy to add a new command to this command processor.
 */

static const command_table_t commands[] = {{"Size", handle_size, "Find the Size of the SD Card and Free Space\n\r"},
											{"Format", handle_format, "Format the Card\n\r"},
                                            {"Mount", hande_mount, "Manually Mount Card"},
											{"ls", handle_ls, "list the contents of the current directory\n\r"},
											{"mkdir", handle_mkdir, "Create a directory\n\r"},
                                            {"file_create", handle_createfile, "Create a file of random data\n\r"},
                                            {"cd", handle_cd, "Move into a directory\n\r"},
                                            {"add_data", handle_add_data, "Add random Data to an Existing File\n\r"},
                                            {"Del", handle_del, "Delete a file\n\r"},
                                            {"FatFs", handle_fatfs, "Format Card and Run Example of FatFS Operations"},
                                            {"Unmount", handle_unmount, "Unmount card and Quit"},
											{"Help", handle_help, "Prints a help message with info about all of the supported commands.\n\r"}};
//Calculates the number of commands based on commands and the command table
static const int num_commands = sizeof(commands) / sizeof(command_table_t);




#endif /* EXAMPLES_MAX78000_SDHC_FTHR_INCLUDE_CLI_H_ */