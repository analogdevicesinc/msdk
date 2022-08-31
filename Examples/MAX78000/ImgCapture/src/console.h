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
#include "board.h"
#include "example_config.h"
#include "uart.h"

#define SERIAL_BUFFER_SIZE 256
#define CON_BAUD 921600 // UART baudrate used for sending data to PC

#ifdef SD
#include "sd.h"
#endif

typedef enum {
    CMD_UNKNOWN = -1,
    CMD_HELP = 0,
    CMD_RESET,
    CMD_CAPTURE,
    CMD_IMGRES,
    CMD_STREAM,
    CMD_SETREG,
    CMD_GETREG,
#ifdef SD
    CMD_SD_MOUNT,
    CMD_SD_UNMOUNT,
    CMD_SD_CWD,
    CMD_SD_CD,
    CMD_SD_LS,
    CMD_SD_MKDIR,
    CMD_SD_RM,
    CMD_SD_TOUCH,
    CMD_SD_WRITE,
    CMD_SD_CAT,
    CMD_SD_SNAP
#endif
} cmd_t;

extern char* cmd_table[];
extern char* help_table[];

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

#ifdef SD
// Supporting function for use with f_forward (http://elm-chan.org/fsw/ff/doc/forward.html)
// Streams fatFS bytes to the UART TX FIFO
UINT out_stream(const BYTE* p, UINT btf);
#endif

#endif
