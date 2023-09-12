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
//                     INCLUDES
/* -------------------------------------------------- */
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "cli.h"
#include "nvic_table.h"
#include "uart.h"

/* -------------------------------------------------- */
//                      MACROS
/* -------------------------------------------------- */
#define ENTER 0X0D
#define NEW_LINE 0x0A
#define SPACE 0x20
#define TAB 0x09
#define BACKSPACE 0X08
#define MAXBUFF 2000
#define DELETE 0x7F
#define DOLLAR 0x24

#define ARROW_KEY_CODE_1 0x1B
#define ARROW_KEY_CODE_2 0x5B
#define ARROW_KEY_CODE_LEFT 0x44
#define ARROW_KEY_CODE_RIGHT 0x43
#define ARROW_KEY_CODE_UP 0x41
#define ARROW_KEY_CODE_DOWN 0x42

// Define a buffer length to store commands
#define MAX_COMMAND_LENGTH 256

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
const command_t help_command = { "Help", "help", "Prints details regarding the usage of the supported commands.", handle_help };

// Command table parameters;
const command_t *command_table;
unsigned int command_table_sz;

/* -------------------------------------------------- */
//            PRIVATE FUNCTION DEFINITIONS
/* -------------------------------------------------- */
/** 
 * @brief Checks whether the current character is a white space character
 * 
 * @param p     Character to check
 * 
 * @returns True - chacracter is whitespace, False - character is not whitespace
 */
bool white_space_present(char *p)
{
    return *p == SPACE || *p == TAB;
}

/** 
 * @brief Clears the buffer storing each character of the user command input
 */
void User_Prompt_Sequence(void)
{
    MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART), NEW_LINE);
    MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART), DOLLAR);
    MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART), SPACE);
}

/** 
 * @brief Clears the buffer storing each character of the user command input
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
    MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART), BACKSPACE);
    MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART), SPACE);
    MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART), BACKSPACE);
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
        MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART), NEW_LINE);
        MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART), ENTER);
        cmd_buf[buf_idx++] = '\r';
        cmd_buf[buf_idx] = '\0';
        buf_idx = 0;
        process_command(cmd_buf);
        Clear_buffer();
        break;

    default:
        // Handle all other characters
        if (buf_idx < MAX_COMMAND_LENGTH - 1) {
            cmd_buf[buf_idx++] = user_char; //pushes characters into the buffer
            MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART), user_char);
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
    // Initialize p and end pointers
    char *p = input;
    char *end;

    // Find end of string
    for (end = input; *end != '\0'; end++) {}

    // Initialize variables
    bool in_token = false;
    bool beginning_space;
    char *argv[10];
    int argc = 0;
    memset(argv, 0, sizeof(argv));

    if (*p == SPACE || *p == TAB) {
        beginning_space = true;
    }

    // Iterate over each character in input
    for (p = input; p < end; p++) {
        if (in_token) {
            // If in a token
            if (white_space_present(p)) {
                // If whitespace found, token ends
                in_token = true;
            } else {
                // Else, token continues
                input = p;
                in_token = false;
            }
        } else {
            // If not in a token
            if (*p == ENTER || (white_space_present(p))) {
                // If ENTER or whitespace found
                if (*p == ENTER) {
                    // If ENTER, end of command
                    *p = '\0';
                    argv[argc++] = input;
                } else if (white_space_present(p) && !beginning_space) {
                    // If whitespace found, end of token
                    *p = '\0';
                    argv[argc++] = input;
                    in_token = true;
                }
            } else if (!white_space_present(p) && beginning_space) {
                // If not in token and whitespace not found, new token starts
                input = p;
                in_token = false;
                beginning_space = false;
            } else if (!white_space_present(p) && !beginning_space) {
                // If not in token and whitespace not found and beginning_space is false, token continues
                in_token = false;
            }
        }
    }

    // Set last argv value to NULL
    argv[argc] = NULL;

    // If no arguments, return
    if (argc == 0) {
        return;
    }

    int success_flag = 1;

    // Check for a valid command
    if (strcasecmp(argv[0], help_command.name) == 0) {
        // Help command received
    	success_flag = help_command.handler(argc, argv);
    } else {
        // Help command not received, iterate over all user-defined commands
        for (int i = 0; i < command_table_sz; i++) {
            if (strcasecmp(argv[0], command_table[i].name) == 0) {
                // Call corresponding command's handler
                success_flag = command_table[i].handler(argc, argv);
                break;
            }
        }
    }

    // If no commands match, print error message
    if (success_flag == 1) {
        printf("\n\rCommand isn't valid!\n\r");
    } else if (success_flag < E_NO_ERROR) {
    	printf("\n\rEnter 'help' to see a list of available commands.\n\r");
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
    printf("\n\r");
    for (int i = 0; i < command_table_sz; i++) {
        printf("%s:\n", command_table[i].name);
        printf("  Usage: %s\n", command_table[i].usage);
        printf("  Description: %s\n\n", command_table[i].description);
    }

    return E_NO_ERROR;
}

/**
 * @brief IRQ Handler for the CLI UART
 */ 
void UART_Handler(void)
{
    MXC_UART_AsyncHandler(MXC_UART_GET_UART(CONSOLE_UART));
}

/**
 * @brief Callback function for when a character is received by the CLI
 */ 
void CLI_Callback(mxc_uart_req_t *req, int error)
{
    line_accumulator(char_recv);
    MXC_UART_TransactionAsync(req);
}

/* -------------------------------------------------- */
//             PUBLIC FUNCTION DEFINITIONS
/* -------------------------------------------------- */
int CLI_Init(const command_t *commands, unsigned int num_commands)
{
    int error;

    // Check for valid parameters
    if (commands == NULL) {
        return E_NULL_PTR;
    } else if (num_commands <= 0) {
        return E_BAD_PARAM;
    }

    // Save the command table
    command_table = commands;
    command_table_sz = num_commands;

    // UART interrupt setup
    NVIC_ClearPendingIRQ(MXC_UART_GET_IRQ(CONSOLE_UART));
    NVIC_DisableIRQ(MXC_UART_GET_IRQ(CONSOLE_UART));
    MXC_NVIC_SetVector(MXC_UART_GET_IRQ(CONSOLE_UART), UART_Handler);
    NVIC_EnableIRQ(MXC_UART_GET_IRQ(CONSOLE_UART));

    // Initialize Console UART
    if ((error = MXC_UART_Init(MXC_UART_GET_UART(CONSOLE_UART), UART_BAUD, MXC_UART_APB_CLK)) !=
        E_NO_ERROR) {
        printf("-->Error initializing CLI UART: %d\n", error);
        return error;
    }

    // Print success message and prompt
    printf("CLI Initialized! Enter 'help' to see a list of available commands.\n");
    User_Prompt_Sequence();
    while(MXC_UART_GetActive(MXC_UART_GET_UART(CONSOLE_UART))) {}

    // Initialize an asynchoronous request for the first character
    cli_req.uart = MXC_UART_GET_UART(CONSOLE_UART);
    cli_req.rxData = &char_recv;
    cli_req.rxLen = BUFF_SIZE;
    cli_req.txLen = 0;
    cli_req.callback = CLI_Callback;

    error = MXC_UART_TransactionAsync(&cli_req);

    return error;
}
