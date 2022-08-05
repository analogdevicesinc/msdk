/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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
* @file console.h
* @brief Serial console header file
*****************************************************************************/

#ifndef H_CONSOLE
#define H_CONSOLE
#include "uart.h"
#include "board.h"
#include "example_config.h"

#define SERIAL_BUFFER_SIZE 256
#define CON_BAUD           921600 // UART baudrate used for sending data to PC

typedef enum {
    CMD_UNKNOWN = -1,
    CMD_RESET   = 0,
    CMD_CAPTURE,
    CMD_IMGRES,
    CMD_STREAM,
    CMD_SETREG,
    CMD_GETREG
} cmd_t;

extern char* cmd_table[];

static mxc_uart_regs_t* Con_Uart = MXC_UART_GET_UART(CONSOLE_UART);
extern char g_serial_buffer[SERIAL_BUFFER_SIZE];
extern int g_buffer_index;
extern int g_num_commands;

int console_init();
int send_msg(const char* msg);
int recv_msg(char* buffer);
int recv_cmd(cmd_t* out_cmd);
void clear_serial_buffer(void);
void print_help(void);

#endif
