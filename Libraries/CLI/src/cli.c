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
//                      INCLUDES
/* -------------------------------------------------- */

#include "cli.h"

// Define a buffer length to store commands
#define MAX_COMMAND_LENGTH 256

/* -------------------------------------------------- */
//             FUNCTION PROTOTYPES
/* -------------------------------------------------- */

bool white_space_present(char *p);

bool white_space_not_present(char *p);

void User_Prompt_Sequence(void);

void Console_Backspace_Sequence(void);

void Console_Cmd_Clear(void);

void Clear_buffer(void);

/* -------------------------------------------------- */
//                      GLOBALS
/* -------------------------------------------------- */

/*! \name buf
    \brief Buffer to store the characters of the command 
*/
char buf[MAX_COMMAND_LENGTH];

/*! \name idx
    \brief Idx to keep track of the buffer which stores the characters of the command 
*/
uint16_t idx = 0;

void line_accumulator(uint8_t user_char)
{
    switch (user_char) {
    case BACKSPACE: {
        // Handle Backspace and Delete
        if (idx > 0) {
            //Sequence to implement a backspace on the terminal
            Console_Backspace_Sequence();
            idx--;
            buf[idx] = '\0';
        }
        break;
    }
    case ENTER: {
        // Handle Enter or carriage return
        MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART), NEW_LINE);
        MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART), ENTER);
        buf[idx++] = '\r';
        buf[idx] = '\0';
        idx = 0;
        process_command(buf);
        Clear_buffer();
        break;
    }

    default: {
        // Handle all other characters
        if (idx < MAX_COMMAND_LENGTH - 1) {
            buf[idx++] = user_char; //pushes characters into the buffer
            MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART), user_char);
        }
        break;
    }
    }
}

void process_command(char *input)
{
    //Initialize p and end pointers
    char *p = input;
    char *end;

    //Find end of string
    for (end = input; *end != '\0'; end++) {}

    //Initialize variables
    bool in_token = false;
    bool beginning_space;
    if (*p == SPACE || *p == TAB)
        beginning_space = true;
    char *argv[10];
    int argc = 0;
    memset(argv, 0, sizeof(argv));

    //Iterate over each character in input
    for (p = input; p < end; p++) {
        //If in a token
        if (in_token) {
            //If whitespace found, token ends
            if (white_space_present(p)) {
                in_token = true;
            }
            //Else, token continues
            else if (white_space_not_present(p)) {
                input = p;
                in_token = false;
            }
        }
        //If not in a token
        else {
            //If ENTER or whitespace found
            if (*p == ENTER || (white_space_present(p))) {
                //If ENTER, end of command
                if (*p == ENTER) {
                    *p = '\0';
                    argv[argc++] = input;
                    break;
                }

                //If whitespace found, end of token
                if (white_space_present(p) && !beginning_space) {
                    *p = '\0';
                    argv[argc++] = input;
                    in_token = true;
                }
            }
            //If not in token and whitespace not found, new token starts
            else if (white_space_not_present(p) && beginning_space) {
                input = p;
                in_token = false;
                beginning_space = false;
            }
            //If not in token and whitespace not found and beginning_space is false, token continues
            else if (white_space_not_present(p) && !beginning_space) {
                in_token = false;
            }
        }
    }

    //Set last argv value to NULL
    argv[argc] = NULL;

    //If no arguments, return
    if (argc == 0)
        return;

    bool success_flag = 0; //True if input command matches

    //Iterate over all commands to check if input command matches
    for (int i = 0; i < num_commands; i++) {
        if (strcasecmp(argv[0], commands[i].name) == 0) {
            //Call corresponding command's handler
            commands[i].handler(argc, argv);
            success_flag = 1;
            break;
        }
    }

    //If no commands match, print error message
    if (success_flag == 0) {
        printf("\n\rCommand isn't valid!\n\r");
    }

    //Print prompt
    User_Prompt_Sequence();
}

bool white_space_present(char *p)
{
    return *p == SPACE || *p == TAB;
}

bool white_space_not_present(char *p)
{
    return *p != SPACE || *p != TAB;
}

void User_Prompt_Sequence(void)
{
    MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART), DOLLAR);
    MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART), SPACE);
}

/** Clears the buffer storing each character of the user command input
*
* @param void
*
* @return void
*/
void Clear_buffer(void)
{
    memset(buf, '\0', MAX_COMMAND_LENGTH);
}

/** Clears the line on the uart console
*
* @param void
*
* @return void
*/
void Console_Cmd_Clear(void)
{
    for (int i = 0; i < idx; i++) {
        Console_Backspace_Sequence();
    }
}

/** Writes the backspace in sequence to the uart console
*
* @param void
*
* @return void
*/
void Console_Backspace_Sequence(void)
{
    MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART), BACKSPACE);
    MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART), SPACE);
    MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART), BACKSPACE);
}

void handle_help(int argc, char *argv[])
{
    printf("\n\r");
    for (int i = 0; i < num_commands; i++)
        printf("%s --> %s", commands[i].name, commands[i].help_string);
}