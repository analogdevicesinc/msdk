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
 ******************************************************************************/\

///* -------------------------------------------------- */
////                      INCLUDES
///* -------------------------------------------------- */

#include "cli.h"


/* -------------------------------------------------- */
//             FUNCTION PROTOTYPES
/* -------------------------------------------------- */
bool white_space_present(char *p);

bool white_space_not_present(char *p);

char *cmd_history_str[100];
// int cmd_history_len[100];
int cmd_idx = 0;
char buf[256];
/*
 * @name line_accumlator
 *
 * @brief Reads incoming bytes, Accumulate into line buffer, Echo's chars back to the other side and handles backspace.
 * 		  It calls the process_command function upon pressing the ENTER key
 *
 * @param:
 * 		uint8_t user_char
 * 		User input of characters
 */
void line_accumlator(uint8_t user_char)
{
    // Declare static variables
    static int idx = 0;
    
    switch (user_char) {
    case BACKSPACE:
    {
        // Handle Backspace and Delete
        if (idx > 0) {
        	//Sequence to actually implement a backspace on the terminal
            MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART),BACKSPACE);
            MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART),SPACE);
            MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART),BACKSPACE);
            idx--;
            buf[idx] = '\0';
        }
        break;
    	}
        case ENTER: {
            // Handle Enter or carriage return
            MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART),NEW_LINE);
            MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART),ENTER);
            buf[idx++] = '\r';
            buf[idx] = '\0';

            // cmd_history_len[cmd_idx] = idx;     //Storing the length of the command into an array
            cmd_history_str[cmd_idx++] = buf; //Storing the excecuted command into a buffer
            idx = 0;
            char *accum = buf; //Assign buf
            process_command(accum);
            break;
        }
        case ARROW_KEY_CODE_LEFT: 
        case ARROW_KEY_CODE_RIGHT: 
        case ARROW_KEY_CODE_UP:
        case ARROW_KEY_CODE_DOWN:{
            // Check the sequence to see if one of the arrow keys were pressed
            if (buf[idx] == ARROW_KEY_CODE_2 && buf[idx - 1] == ARROW_KEY_CODE_1){
                MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART),BACKSPACE);
                MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART),BACKSPACE);

                if(user_char == ARROW_KEY_CODE_UP && cmd_idx > 0){
                    //Erase what's currently on the prompt
                    //MXC_UART_Write(MXC_UART_GET_UART(CONSOLE_UART), )
                    MXC_UART_Write(MXC_UART_GET_UART(CONSOLE_UART),cmd_history_str[--cmd_idx],strlen(cmd_history_str[cmd_idx]));

                    //Empty current value of buf and assign respective cmd_history_str
                    idx = strlen(cmd_history_str[cmd_idx]);

                }
            }
            // This else statement is excecuted when the arrow keys weren't pressed and the user entered W,A,S or D keys
            else {
                if (idx < 255) {
                buf[idx++] = user_char; //pushes characters into the buffer
                MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART),user_char);
            }
            break;
            }
        }
        default: {
            // Handle all other characters
            if (idx < 255) {
                buf[idx++] = user_char; //pushes characters into the buffer
                MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART),user_char);
            }
            break;
        }
    }
}

/*
 * @name handle_size
 *
 * @brief Finds the Size of the SD Card and Free Space
 *
 * @param argc and *argv[]
 *
 *
 * @return
 * 		   void
 */
void handle_size(int argc, char *argv[]){
    getSize();
}

/*
 * @name handle_format
 *
 * @brief han
 *
 * @param argc and *argv[]
 *
 *
 * @return
 * 		   void
 */
void handle_format(int argc, char *argv[]){
    formatSDHC();
}

void hande_mount(int argc, char *argv[]){
    mount();
}

void handle_ls(int argc, char *argv[]){
    ls();
}

void handle_mkdir(int argc, char *argv[]){
    mkdir(argv[1]);
}

void handle_createfile(int argc, char *argv[]){
    unsigned int length = str_to_dec(argv[2]);
    createFile(argv[1],length);
}

void handle_cd(int argc, char *argv[]){
    cd(argv[1]);
}

void handle_add_data(int argc, char *argv[]){
    unsigned int length = str_to_dec(argv[2]);
    appendFile(argv[1],length);
}

void handle_del(int argc, char *argv[]){
    delete(argv[1]);
}

void handle_fatfs(int argc, char *argv[]){
    example();
}

void handle_unmount(int argc, char *argv[]){
    umount();
}
/* @name handle_help
 *
 * @brief: Prints a help message with info about all of the supported commands.
 *
 */
void handle_help(int argc, char *argv[])
{
	printf("\n\r");
	for (int i = 0; i < num_commands;i++)
		printf("%s --> %s", commands[i].name, commands[i].help_string);
}

/*
 * @name process_command
 *
 * @brief Performs a Lexical analysis and tokenisis the user's commands
 * 		  Lookup first token in a table of functions, dispatch to handler function
 *
 * @param char *input
 * 		  character pointer containing the line accumulator input
 *
 * @return
 * 		   void
 */
void process_command(char *input)
{
    //Initialize p and end pointers
    char *p = input;
    char *end;

    //Find end of string
    for (end = input; *end != '\0'; end++);

    //Initialize variables
    bool in_token = false;
    bool beginning_space;
    if(*p == SPACE || *p == TAB)
        beginning_space = true;
    char *argv[10];
    int argc = 0;
    memset(argv, 0, sizeof(argv));

    //Iterate over each character in input
    for (p = input; p < end; p++)
    {
        //If in a token
        if (in_token)
        {
            //If whitespace found, token ends
            if (white_space_present(p))
            {
                in_token = true;
            }
            //Else, token continues
            else if(white_space_not_present(p))
            {
                input = p;
                in_token = false;
            }
        }
        //If not in a token
        else
        {
            //If ENTER or whitespace found
            if(*p == ENTER || (white_space_present(p)))
            {
                //If ENTER, end of command
                if(*p == ENTER)
                {
                    *p = '\0';
                    argv[argc++] = input;
                    break;
                }

                //If whitespace found, end of token
                if(white_space_present(p) && !beginning_space)
                {
                    *p = '\0';
                    argv[argc++] = input;
                    in_token = true;
                }
            }
            //If not in token and whitespace not found, new token starts
            else if (white_space_not_present(p) && beginning_space)
            {
                input = p;
                in_token = false;
                beginning_space = false;
            }
            //If not in token and whitespace not found and beginning_space is false, token continues
            else if (white_space_not_present(p) && !beginning_space)
            {
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
    for (int i=0; i < num_commands; i++)
    {
        if (strcasecmp(argv[0], commands[i].name) == 0)
        {
            //Call corresponding command's handler
            commands[i].handler(argc, argv);
            success_flag = 1;
            break;
        }
    }

    //If no commands match, print error message
    if (success_flag == 0)
    {
        printf("\n\rCommand isn't valid!\n\r");
    }

    //Print prompt
    user_prompt_sequence();
}
/*
 * @name white_space_present
 *
 * @brief Checks if white space is present in the given character pointer
 *
 * @param char *p
 * 		  character pointer containing the line accumulator input
 *
 * @return 1 if white space is present
 * 		   0 if white space isn't present
 */
bool white_space_present(char *p){
	return *p == SPACE || *p == TAB;
}
/*
 * @name white_space_not_present
 *
 * @brief Checks if there is no white space present in the given character pointer
 *
 * @param char *p
 * 		  character pointer containing the line accumulator input
 *
 * @return 1 if no white space is present
 * 		   0 if white space is present
 */
bool white_space_not_present(char *p){
	return *p != SPACE || *p != TAB;
}

/*
*
* @name str_to_dec
*
* @brief Converts a string into an integer
*
* @param const char *str
*
* @return int
*           converted integer 
*/
int str_to_dec(const char *str)
{
    int val = 0;

    // Loop through each character in the string
    for (int i = 0; str[i] != '\0'; i++) {
        char c = str[i];

        // Check if the character is a valid decimal digit
        if (isdigit(c)) {
            val = (val * 10) + (c - '0');
        } else {
            // Invalid character
            return -1;
        }
    }
    return val;
}

void user_prompt_sequence(void)
{
    MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART),DOLLAR);
    MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART),SPACE);
}