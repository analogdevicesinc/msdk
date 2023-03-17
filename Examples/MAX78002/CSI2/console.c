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
/**
* @file console.c
* @brief Serial console implementation file
*****************************************************************************/

#include "console.h"
#include <string.h>
#include <stdlib.h>
#include "mxc_delay.h"
#include "led.h"

char g_serial_buffer[SERIAL_BUFFER_SIZE];
int g_buffer_index = 0;
int g_num_commands = 0;

int g_num_commands; // Calculated in 'console_init' as part of initialization
char *cmd_table[] = { "help", "reset", "capture" };

char *help_table[] = { ": Print this help string", ": Issue a soft reset to the host MCU.",
                       ": Perform a standard blocking capture of a single image" };

int starts_with(char *a, char *b)
{
    // Utility function for checking whether 'a' starts with 'b'
    char *ptr_a = a;
    char *ptr_b = b;

    while (*ptr_a && *ptr_b) {
        if (*ptr_a != *ptr_b) {
            return 0;
        }
        ptr_a++;
        ptr_b++;
    }

    return 1;
}

int MXC_UART_WriteBytes(mxc_uart_regs_t *uart, const uint8_t *bytes, int len)
{
    int err = E_NO_ERROR;
    for (int i = 0; i < len; i++) {
        // Wait until FIFO has space for the character.
        while (MXC_UART_GetTXFIFOAvailable(uart) < 1) {}

        if ((err = MXC_UART_WriteCharacterRaw(uart, bytes[i])) != E_NO_ERROR) {
            return err;
        }
    }

    return E_NO_ERROR;
}

// Initialize the serial console and transmits the "*SYNC*" string out of the UART port.
// This function will block until the host sends the "*SYNC*" string back in response.
int console_init(void)
{
    g_num_commands = sizeof(cmd_table) / sizeof(char *);
    clear_serial_buffer();

    int ret = E_NO_ERROR;
    if ((ret = MXC_UART_Init(Con_Uart, CON_BAUD, MXC_UART_IBRO_CLK)) != E_NO_ERROR) {
        return ret;
    }

    printf("Establishing communication with host...\n");
    char *sync = "*SYNC*";

    // Wait until the string "*SYNC*" is echoed back over the serial port before starting the example
    while (1) {
        // Transmit sync string
        send_msg(sync);
        LED_Toggle(LED1);
        MXC_Delay(MXC_DELAY_MSEC(500));

        int available = MXC_UART_GetRXFIFOAvailable(Con_Uart);
        if (available > 0) {
            clear_serial_buffer();
            MXC_UART_Read(Con_Uart, (uint8_t*)g_serial_buffer, &available);
            if (strcmp(g_serial_buffer, sync) == 0) {
                // Received sync string back, break the loop.
                LED_On(LED1);
                break;
            }
        }
    }

    printf("Established communications with host!\n");
    print_help();
    clear_serial_buffer();

    return ret;
}

// Transmit a message over the console's UART with a newline appended.
int send_msg(const char *msg)
{
    int ret = 0;
    int len = strlen(msg);

    // Transmit message string
    if ((ret = MXC_UART_WriteBytes(Con_Uart, (uint8_t *)msg, len)) != E_NO_ERROR) {
        return ret;
    }
    // Transmit newline to complete the message.
    if ((ret = MXC_UART_WriteBytes(Con_Uart, (uint8_t *)"\n", 1)) != E_NO_ERROR) {
        return ret;
    }

    return E_NO_ERROR;
}

// Recieve a message into the global serial buffer.  Returns 1 if a full message
// has been received, otherwise returns 0.
int recv_msg(char *buffer)
{
    int available = MXC_UART_GetRXFIFOAvailable(Con_Uart);
    while (available > 0) {
        char c = MXC_UART_ReadCharacter(Con_Uart);

        if (c == '\n') {
            // Received newline character, terminate the string and return
            g_serial_buffer[g_buffer_index] = '\0';
            return 1;
        } else {
            // Received a character, add to buffer and continue
            g_serial_buffer[g_buffer_index] = c;
        }

        // Increment buffer index (wrap if necessary)
        if (++g_buffer_index >= SERIAL_BUFFER_SIZE) {
            g_buffer_index = 0;
        }

        available = MXC_UART_GetRXFIFOAvailable(Con_Uart);
    }

    // If we reach here, we've received all available characters but
    // no newline.  Return 0 - a full command hasn't been received
    // yet.
    return 0;
}

// Attempts to receive a full command over the console UART.
// Writes the received command to the 'out_cmd' pointer and returns
// 1 if a valid command has been received.  Otherwise, returns 0 and
// sets 'out_cmd' to CMD_UNKNOWN.
int recv_cmd(cmd_t *out_cmd)
{
    if (recv_msg(g_serial_buffer)) {
        cmd_t cmd = CMD_UNKNOWN;
        for (int i = 0; i < g_num_commands; i++) {
            if (starts_with(g_serial_buffer, cmd_table[i])) {
                // If the received command in g_serial_buffer matches
                // a commmand at index i in the cmd_table, then
                // cast the index i to the 'cmd_t' enum.  The enum
                // will now match the received command and is more
                // convenient to process from here since we don't
                // have to do string comparisons anymore.
                cmd = (cmd_t)i;
                *out_cmd = cmd;
                return 1;
            }
        }

        *out_cmd = CMD_UNKNOWN;
        return 1;
    }

    return 0;
}

void clear_serial_buffer(void)
{
    memset(g_serial_buffer, '\0', SERIAL_BUFFER_SIZE);
    g_buffer_index = 0;
}

// Print out all of the entries in the console's command table.
void print_help(void)
{
    int g_num_commands = sizeof(cmd_table) / sizeof(char *);
    printf("Registered %i total commands:\n", g_num_commands);
    printf("-----\n");
    for (int i = 0; i < g_num_commands; i++) {
        printf("\t'%s' %s\n", cmd_table[i], help_table[i]);
    }
    printf("-----\n");
}

#ifdef SD
// Utility function for streaming data out of a file into the UART TX FIFO
UINT out_stream(const BYTE *p, UINT btf)
{
    // If btf > 0, btf is the number of bytes to send.
    // If btf == 0, sense call querying if the stream is available.

    if (btf == 0) { // Sense call, this function should return the stream status.
        return MXC_UART_GetTXFIFOAvailable(Con_Uart);
    } else {
        return MXC_UART_WriteTXFIFO(Con_Uart, (const unsigned char *)p, (unsigned int)btf);
    }
}
#endif
