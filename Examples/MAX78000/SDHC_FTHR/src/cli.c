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
    static char buf[256];
    
    switch (user_char) {
    case BACKSPACE:
    case DELETE: {
        // Handle Backspace and Delete
        if (idx > 0) {
        	//Sequence to actually implement a backspace on the terminal
            putchar(BACKSPACE);
            putchar(SPACE);
            putchar(BACKSPACE);
            idx--;
            buf[idx] = '\0';
        }
        break;
    	}
        case ENTER: {
            // Handle Enter or carriage return
            printf("\r");
            buf[idx++] = '\r';
            buf[idx] = '\0';
            idx = 0;
            char *accum = buf; //Assign buf
            process_command(accum);
            break;
        }
        default: {
            // Handle all other characters
            if (idx < 255) {
                buf[idx++] = user_char; //pushes characters into the buffer
                putchar(user_char);
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
    mkdir();
}

void handle_createfile(int argc, char *argv[]){
    createFile();
}

void handle_cd(int argc, char *argv[]){
    cd();
}

void handle_add_data(int argc, char *argv[]){
    appendFile();
}

void handle_del(int argc, char *argv[]){
    delete();
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

/* @name handle_sp
 *
 * @brief: Prints the current stack pointer in hex which is fetched from the MSP [Main stack pointer register]
 */
// void handle_sp(int argc, char *argv[])
// {
// 	  uint32_t *msp;
// 	  asm("mrs %0, msp" : "=r" (msp)); // inline assembly to read the MSP (Main Stack Pointer) register of the Cortex-M4 processor and store its value in the msp variable.
// 	  printf("\n\rStack Pointer (MSP): %08x\n\r", (unsigned int)msp); //The value of the msp variable is then printed to the terminal using printf()
// }

/* @name handle_dump
 *
 * @brief: Prints a hexdump of the memory requested, with up to 8 bytes per line of output.
 */
// void handle_dump(int argc, char *argv[])
// {
//     printf("invalid command\n\r");
// 	hexdump(argv[1], argv[2]);
// }

/* @name handle_info
 *
 * @brief: Prints a string with build information that is dynamically generated at build time from your machine.
 * 			Referencing macros defined in the makefile.def file
 *
 */
// void handle_info(int argc, char *argv[])
// {
// 	printf("\n\rVersion");
// 	// printf("built on %s", MACHINE);
// 	// printf(" at %s\n\r", DATE_TIME);
// 	// printf("Commit %s\n\r", COMMIT_HASH);
// }
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
    printf("$$ ");
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