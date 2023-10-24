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

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "board.h"
#include "cli.h"
#include "file.h"
#include "led.h"
#include "main.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "uart.h"

/* ASCII macros */
#define END_OF_TEXT 0x03
#define BACK_SPACE 0x08
#define SPACE_BAR 0x20
#define NULL_TERMINATION 0x00
#define ARROW_KEY_CODE_1 0x1B
#define ARROW_KEY_CODE_2 0x5B
#define ARROW_KEY_CODE_LEFT 0x44
#define ARROW_KEY_CODE_RIGHT 0x43
#define ARROW_KEY_CODE_UP 0x41
#define ARROW_KEY_CODE_DOWN 0x42
#define TAB_SPACE 0x09
#define POWER_OF 0x5E
#define CAPITAL_C 0x43

/********************************************************************************/
/******************************* Private Functions ******************************/
/********************************************************************************/

/***** Handler Functions *****/
static int help_CmdHandler(void)
{
    // Print command descriptions
    printf("\nThe available commands are:\n");
    printf("  * help\n");
    printf("       Description: Prints out list of available commands.\n");
    printf("       Usage: help\n\n");
    printf("  * stop\n");
    printf("       Description: Ends the example.\n");
    printf("       Usage: stop\n\n");
    printf("  * read\n");
    printf("       Description: Reads data from a specific location within a file. If\n");
    printf("                    the read is successful, the data read is printed to the\n");
    printf("                    terminal.\n");
    printf("       Usage: read <filename> <number of bytes to read> <location>\n\n");
    printf("  * write\n");
    printf("       Description: Writes a character string to a specific location within\n");
    printf("                    a file.\n");
    printf("       Usage: write (--create) <filename> <character string> <location>\n");
    printf("       Options:\n");
    printf("          --create: Creates file <filename> if it does not already exist.\n");
    printf("  * swl\n");
    printf("       Description: Stands for \"show wear leveling\". Writes to a file the\n");
    printf("                    specified number of times. Once all writes have completed,\n");
    printf("                    the number of times each flash page (filesystem block)\n");
    printf("                    was written to is printed to the terminal. This command may\n");
    printf("                    take a while to complete. LED0 is used as a heartbeat while\n");
    printf("                    the command is executing.\n");
    printf("       Usage: swl <number of writes>\n\n");

    return E_NO_ERROR;
}

static int stop_CmdHandler(void)
{
    return E_SHUTDOWN;
}

static int read_CmdHandler(lfs_t *lfs, char *args)
{
    // Check for invalid arguments
    if (lfs == NULL || args == NULL) {
        printf("Invalid argument string. Read failed.\n");
        return E_NULL_PTR;
    }

    char *save_ptr = args;

    // Parse arguments sting
    strtok_r(args, " ", &save_ptr);
    char *filename = strtok_r(NULL, " ", &save_ptr);
    char *str_num_bytes = strtok_r(NULL, " ", &save_ptr);
    char *str_pos = strtok_r(NULL, "\r\n", &save_ptr);
    char data[MAX_FILE_READ_SIZE];
    lfs_file_t file;

    memset(data, '\0', sizeof(data));

    // Convert arguments to integers
    int num = atoi(str_num_bytes);
    int pos = atoi(str_pos);

    // Read data from file
    num = file_read(lfs, &file, filename, data, num, pos);
    if (num < LFS_ERR_OK) {
        printf("Read failed with error code %d.\n", num);
        return num;
    } else {
        printf("%d bytes were read from %s in filesystem block %d.\n", num, filename, file.block);
    }

    // Print data read from file to the terminal
    printf("The following string was read from file %s:\n", filename);

    for (int i = 0; i < num; i++) {
        printf("%c", data[i]);
    }
    printf("\n");

    return E_NO_ERROR;
}

static int write_CmdHandler(lfs_t *lfs, char *args)
{
    // Check for invalid arguments
    if (lfs == NULL) {
        printf("Invalid filesystem instance. Write failed.\n");
        return E_NULL_PTR;
    } else if (args == NULL) {
        printf("Invalid argument string. Write failed.\n");
        return E_INVALID;
    }

    lfs_file_t file;
    char *create_fl;
    char *filename;
    char *data;
    char *str_pos;
    int pos, err;
    bool create;
    char *save_ptr = args;

    // Parse arguments
    strtok_r(args, " ", &save_ptr);
    create_fl = strtok_r(NULL, " ", &save_ptr);

    // Parse remainder of the arguments based on whether not create flag was passed
    if (memcmp(create_fl, "--create", sizeof("--create") - 1) == 0) {
        // Create flag passed, next argument is filename
        create = true;
        filename = strtok_r(NULL, " ", &save_ptr);
        data = strtok_r(NULL, " ", &save_ptr);
        str_pos = strtok_r(NULL, "\r\n", &save_ptr);
    } else {
        // create flag not passed, last argument parsed was file name
        create = false;
        filename = create_fl;
        data = strtok_r(NULL, " ", &save_ptr);
        str_pos = strtok_r(NULL, "\r\n", &save_ptr);
    }

    // Convert position to an integer value
    pos = atoi(str_pos);

    // Write data to the file
    err = file_write(lfs, &file, filename, data, strlen(data), pos, create);
    if (err < LFS_ERR_OK) {
        printf("Write failed with error code %d.\n", err);
    } else {
        printf("%d bytes were written to %s in filesystem block %d.\n", err, filename, file.block);
    }

    return err;
}

static int swl_CmdHandler(lfs_t *lfs, char *args)
{
    // Check for invalid arguments
    if (lfs == NULL) {
        printf("Invalid filesystem instance. Write failed.\n");
        return E_NULL_PTR;
    } else if (args == NULL) {
        printf("Invalid argument string. Write failed.\n");
        return E_INVALID;
    }

    char *save_ptr = args;
    char *str_num_writes;
    int num_writes, err;
    int hit_count[LFS_PAGE_CNT] = { 0 };

    // Parse argument string
    strtok_r(args, " ", &save_ptr);
    str_num_writes = strtok_r(NULL, "\r\n", &save_ptr);
    num_writes = atoi(str_num_writes);

    //Set up dummy arguments
    char filename[] = "swl_test_file";
    char data[] = "show_littlefs_wear_leveling"; // Length of this string must exceed lfs.cache_size
    lfs_file_t file;

    // Write to the test file the specified number of writes and
    // track how many times each flash page is written to
    for (int i = 0; i < num_writes; i++) {
        // Do next write
        err = file_write(lfs, &file, filename, data, strlen(data), 0, true);
        if (err < LFS_ERR_OK) {
            printf("Failed to write to test file. Aborting \"swl\" command.\n");
            return err;
        }

        // Increment the hit count
        if (file.block >= 0 && file.block < LFS_PAGE_CNT) {
            hit_count[file.block]++;
        }

        // Heartbeat, this loop can take a while if num_writes is large
        if (i % 50 == 0) {
            LED_Toggle(0);
        }
    }

    // Print results
    printf("All writes have completed. Here are the results:\n");
    for (int i = 0; i < LFS_PAGE_CNT; i++) {
        printf("Block %d was written to %d times.\n", i, hit_count[i]);
    }
    printf("\n");

    return E_NO_ERROR;
}

/* =| checkArrowKeys |======================================
 *
 * Function to check for the presence of arrow keys in the 
 *  input command.
 *
 * NOTE: The cmd should have atleast 3 bytes of data to 
 *  check for the arrow keys. Sanity check should be done 
 *  before calling this function.
 * 
 * =============================================================
 */
bool checkArrowKeys(char *cmd)
{
    if (((cmd[0] == ARROW_KEY_CODE_1) && (cmd[1] == ARROW_KEY_CODE_2)) &&
        ((cmd[2] == ARROW_KEY_CODE_UP) || (cmd[2] == ARROW_KEY_CODE_DOWN) ||
         (cmd[2] == ARROW_KEY_CODE_RIGHT) || (cmd[2] == ARROW_KEY_CODE_LEFT)))
        return true;
    else
        return false;
}

/* =| checkLeadingSpaces |======================================
 *
 * Function to check for the leading spaces in the 
 *  input command.
 *
 * If any Leading spaces are present, the function would remove 
 *  those spaces and returns a valid Null terminated command 
 *  in the same input cmd.
 * 
 * =============================================================
 */
void checkLeadingSpaces(char *cmd, unsigned int *num_recv)
{
    unsigned int leadingZerosCount = 0;
    for (leadingZerosCount = 0; leadingZerosCount < (*num_recv); leadingZerosCount++) {
        if (cmd[leadingZerosCount] != SPACE_BAR)
            break;
    }
    if (leadingZerosCount > 0) {
        memmove(cmd, cmd + leadingZerosCount, (*num_recv) - leadingZerosCount);
        *num_recv = *num_recv - leadingZerosCount;
        cmd[*num_recv] = NULL_TERMINATION;
        ++(*num_recv);
    }
}

uint8_t backSpaceSequence[3] = { BACK_SPACE, SPACE_BAR, BACK_SPACE };
uint8_t abortSequence[2] = { POWER_OF, CAPITAL_C };
/********************************************************************************/
/******************************* Public Functions *******************************/
/********************************************************************************/
int cmd_get(char *cmd, size_t size)
{
    if (cmd == NULL) {
        return E_NULL_PTR;
    } else if (size < 0) {
        return E_BAD_PARAM;
    }

    /* Clear the buffer before storing the command */
    memset(cmd, 0x00, CMD_MAX_SIZE);

    bool eoc = false;
    unsigned int num_recv = 0;
    int next_ch;
    int backSpaceLen = (int)(sizeof(backSpaceSequence) / sizeof(backSpaceSequence[0]));
    int abortLen = (int)(sizeof(abortSequence) / sizeof(abortSequence[0]));

    while (!eoc) {
        // Read character from RX FIFO, wait here until 1 is available
        while ((next_ch = MXC_UART_ReadCharacter(MXC_UART_GET_UART(CONSOLE_UART))) < E_NO_ERROR) {}

        if (next_ch == BACK_SPACE) { // backspace
            if (num_recv > 0) {
                num_recv--;
                MXC_UART_Write(MXC_UART_GET_UART(CONSOLE_UART), backSpaceSequence, &backSpaceLen);
                cmd[num_recv] = NULL_TERMINATION;
            }
        } else if (next_ch == TAB_SPACE) {
            /* Ignore Tab Space input */
            continue;
        } else if (next_ch == END_OF_TEXT) {
            /* ^C abort */
            num_recv = 0;
            MXC_UART_Write(MXC_UART_GET_UART(CONSOLE_UART), abortSequence, &abortLen);
            break;
        } else {
            /* Store character */
            cmd[num_recv++] = (char)next_ch;
            bool arrowKey = false;
            if (num_recv >= 3) {
                arrowKey = checkArrowKeys(cmd + num_recv - 3);
                if (arrowKey) {
                    /* Remove the arrow key from the cmd */
                    cmd[num_recv - 3] = NULL_TERMINATION;
                    num_recv -= 3;
                }
            }
            if ((!arrowKey) && ((char)next_ch != ARROW_KEY_CODE_1) &&
                ((char)next_ch != ARROW_KEY_CODE_2)) {
                /* Echo out if not an arrow key */
                MXC_UART_WriteCharacter(MXC_UART_GET_UART(CONSOLE_UART), next_ch);
            }
        }

        // if buffer full or EOC received, exit loop
        if (num_recv == size || next_ch == '\r' || next_ch == '\n') {
            eoc = true;
        }
    }
    cmd[num_recv] = NULL_TERMINATION;
    checkLeadingSpaces(cmd, &num_recv);
    return num_recv;
}

int cmd_process(lfs_t *lfs, char *cmd, size_t size)
{
    // Check for invalid parameters
    if (lfs == NULL || cmd == NULL) {
        return E_NULL_PTR;
    } else if (size <= 0) {
        return E_BAD_PARAM;
    }

    // Parse out command and argument strings
    int err = E_INVALID;

    // Call appropriate command handler for valid commands
    if (memcmp(cmd, "help", sizeof("help") - 1) == 0) {
        // Process help command
        err = help_CmdHandler();
    } else if (memcmp(cmd, "stop", sizeof("stop") - 1) == 0) {
        // Process stop command
        err = stop_CmdHandler();
    } else if (memcmp(cmd, "read", sizeof("read") - 1) == 0) {
        // Process file read command
        err = read_CmdHandler(lfs, cmd);
    } else if (memcmp(cmd, "write", sizeof("write") - 1) == 0) {
        // Process file write command
        err = write_CmdHandler(lfs, cmd);
    } else if (memcmp(cmd, "swl", sizeof("swl") - 1) == 0) {
        // Process show wear leveling command
        err = swl_CmdHandler(lfs, cmd);
    }

    memset(cmd, '\0', size);

    return err;
}
