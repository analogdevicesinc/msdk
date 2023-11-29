/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc., All Rights Reserved.
 * (now owned by Analog Devices, Inc.)
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
 ******************************************************************************
 *
 * Copyright 2023 Analog Devices, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

/* -------------------------------------------------- */
//                     INCLUDES
/* -------------------------------------------------- */
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "cli.h"
#include "nvic_table.h"

/* -------------------------------------------------- */
//                      MACROS
/* -------------------------------------------------- */
// Characters
#define ENTER 0x0D
#define NEW_LINE 0x0A
#define SPACE 0x20
#define BACKSPACE 0x08
#define DOLLAR 0x24

// Define a buffer length to store commands
#define MAX_COMMAND_LENGTH 256
#define MAX_COMMAND_TOKENS 10

#define UART_BAUD 115200
#define BUFF_SIZE 1

/* -------------------------------------------------- */
//                FUNCTION PROTOTYPES
/* -------------------------------------------------- */
void line_accumulator(uint8_t user_char);
void process_command(char *input);
int handle_help(int argc, char *argv[]);

/* -------------------------------------------------- */
//                 GLOBAL VARIABLES
/* -------------------------------------------------- */
char cmd_buf[MAX_COMMAND_LENGTH]; // Command buffer
uint16_t buf_idx = 0;

uint8_t char_recv; // Variable to store characters received by CLI UART
mxc_uart_req_t cli_req; // CLI UART transaction request structure

// Help Command
const command_t help_command = { "Help", "help",
                                 "Prints details regarding the usage of the supported commands.",
                                 handle_help };

// Command table parameters;
const command_t *command_table = NULL;
unsigned int command_table_sz = 0;

// UART instance used for CLI operations
mxc_uart_regs_t *cli_uart = NULL;

/* -------------------------------------------------- */
//            PRIVATE FUNCTION DEFINITIONS
/* -------------------------------------------------- */
/** 
 * @brief Prints the CLI prompt
 */
void User_Prompt_Sequence(void)
{
    if (cli_uart == NULL) {
        return;
    }

    MXC_UART_WriteCharacter(cli_uart, NEW_LINE);
    MXC_UART_WriteCharacter(cli_uart, DOLLAR);
    MXC_UART_WriteCharacter(cli_uart, SPACE);
}

/** 
 * @brief Clears the buffer storing the command string
 */
void Clear_buffer(void)
{
    memset(cmd_buf, '\0', MAX_COMMAND_LENGTH);
}

/** 
 * @brief Writes the backspace sequence to the UART console
 */
void Console_Backspace_Sequence(void)
{
    if (cli_uart == NULL) {
        return;
    }

    MXC_UART_WriteCharacter(cli_uart, BACKSPACE);
    MXC_UART_WriteCharacter(cli_uart, SPACE);
    MXC_UART_WriteCharacter(cli_uart, BACKSPACE);
}

/** 
 * @brief Clears the line on the UART console
 */
void Console_Cmd_Clear(void)
{
    for (int i = 0; i < buf_idx; i++) {
        Console_Backspace_Sequence();
    }
}

/** 
 * @brief Adds characters to the command string as they are received and echos them to the terminal
 */
void line_accumulator(uint8_t user_char)
{
    if (cli_uart == NULL) {
        return;
    }

    switch (user_char) {
    case BACKSPACE:
        // Handle Backspace and Delete
        if (buf_idx > 0) {
            //Sequence to implement a backspace on the terminal
            Console_Backspace_Sequence();
            buf_idx--;
            cmd_buf[buf_idx] = '\0';
        }
        break;

    case ENTER:
        // Handle Enter or carriage return
        MXC_UART_WriteCharacter(cli_uart, NEW_LINE);
        MXC_UART_WriteCharacter(cli_uart, ENTER);

        // Parse and execute command
        process_command(cmd_buf);

        // Reset command buffer
        buf_idx = 0;
        Clear_buffer();
        break;

    default:
        // Handle all other characters
        if (buf_idx < MAX_COMMAND_LENGTH) {
            cmd_buf[buf_idx++] = user_char; //pushes characters into the buffer
            MXC_UART_WriteCharacter(cli_uart, user_char);
        }
        break;
    }
}

/**
 * @brief Determines whether the command received was valid and calls the appropriate handler
 * 
 * @param input   Command string received by the CLI 
 */
void process_command(char *input)
{
    // Initialize variables
    char *argv[MAX_COMMAND_TOKENS + 1]; // Plus 1 so that argv can always be null terminated
    int argc = 0;
    int success_flag = 1;

    // Initialize command string and token pointers
    char *cmd = input;
    char *token;

    // Parse command string (delimiters: space and tab)
    while (argc < MAX_COMMAND_TOKENS && (token = strtok_r(cmd, " \t", &cmd))) {
        argv[argc++] = token;
    }

    // Set last argv value to NULL
    argv[argc] = NULL;

    // If no arguments, return
    if (argc == 0) {
        return;
    }

    // Check for a valid command
    if (strcasecmp(argv[0], help_command.cmd) == 0) {
        // Help command received
        success_flag = help_command.handler(argc, argv);
    } else {
        // Help command not received, iterate over all user-defined commands
        for (int i = 0; i < command_table_sz; i++) {
            if (strcasecmp(argv[0], command_table[i].cmd) == 0) {
                // Call corresponding command's handler
                success_flag = command_table[i].handler(argc, argv);
                break;
            }
        }
    }

    // Check for errors
    if (success_flag == 1) {
        // Command entered is not supported
        printf("\nCommand isn't valid!\n");
    } else if (success_flag < E_NO_ERROR) {
        // Command entered is supported, but arguments entered incorrectly
        printf("\nEnter 'help' for details on how to use the '%s' command.\n", argv[0]);
    }

    // Print prompt
    User_Prompt_Sequence();
}

/**
 * @brief Prints the help string of each command from the command table
 *
 * @param argc      The command element number within the command string
 * @param argv[]    Array of arguments storing different tokens of the
 *                  command string in the same order as they were passed
 *                  in the command line.
 * 
 * @returns E_NO_ERROR if successful, otherwise an error code.
 */
int handle_help(int argc, char *argv[])
{
    // Print out name, usage, and description of each supported command
    for (int i = 0; i < command_table_sz; i++) {
        printf("\n%s:\n", command_table[i].cmd);
        printf("  Usage: %s\n", command_table[i].usage);
        printf("  Description: %s\n\n", command_table[i].description);
    }

    return E_NO_ERROR;
}

/**
 * @brief Callback function for when a character is received by the CLI
 */
void CLI_Callback(mxc_uart_req_t *req, int error)
{
    if (error == E_ABORT) {
        // Shutdown called, nothing to do in callback
        return;
    }

    // Process received character
    line_accumulator(char_recv);

    // Get ready to receive next character
    MXC_UART_TransactionAsync(req);
}

/* -------------------------------------------------- */
//             PUBLIC FUNCTION DEFINITIONS
/* -------------------------------------------------- */
int MXC_CLI_Init(mxc_uart_regs_t *uart, const command_t *commands, unsigned int num_commands)
{
    int error;
    int uart_idx = MXC_UART_GET_IDX(uart);

    // Check for valid parameters
    if (uart_idx < 0) {
        return E_BAD_PARAM;
    } else if (commands == NULL) {
        return E_NULL_PTR;
    } else if (num_commands <= 0) {
        return E_BAD_PARAM;
    }

    // Return error if CLI is already initialized
    if (cli_uart != NULL) {
        return E_BAD_STATE;
    }

    // Save the command table
    cli_uart = uart;
    command_table = commands;
    command_table_sz = num_commands;

    // Initialize Console UART
    if ((error = MXC_UART_Init(uart, UART_BAUD, MXC_UART_APB_CLK)) != E_NO_ERROR) {
        printf("-->Error initializing CLI UART: %d\n", error);
        return error;
    }

    // Initialize an asynchoronous request for the first character
    cli_req.uart = cli_uart;
    cli_req.rxData = &char_recv;
    cli_req.rxLen = BUFF_SIZE;
    cli_req.txLen = 0;
    cli_req.callback = CLI_Callback;
    if ((error = MXC_UART_TransactionAsync(&cli_req)) != E_NO_ERROR) {
        return error;
    }

    // Print success message and prompt
    printf("CLI Initialized! Enter 'help' to see a list of available commands.\n");
    User_Prompt_Sequence();
    while (MXC_UART_GetActive(uart)) {}

#ifdef USE_CLI_LIB_IRQHANDLER
    // Give users the option to define their own IRQ handler in their application. By default,
    // we point the interrupt vector at MXC_CLI_Handler.
    MXC_NVIC_SetVector(MXC_UART_GET_IRQ(uart_idx), MXC_CLI_Handler);
#endif // USE_CLI_LIB_IRQHANDLER

    // Enable interrupts
    NVIC_EnableIRQ(MXC_UART_GET_IRQ(uart_idx));

    return E_NO_ERROR;
}

int MXC_CLI_Shutdown(void)
{
    // Return if CLI is uninitialized
    if (cli_uart == NULL) {
        return E_BAD_STATE;
    }

    // Abort existing async transaction
    MXC_UART_AbortAsync(cli_uart);

    // Reset state variables
    cli_uart = NULL;
    command_table = NULL;
    command_table_sz = 0;
    buf_idx = 0;

    return E_NO_ERROR;
}

void MXC_CLI_Handler(void)
{
    // Return if CLI is uninitialized
    if (cli_uart == NULL) {
        return;
    }

    MXC_UART_AsyncHandler(cli_uart);
}
