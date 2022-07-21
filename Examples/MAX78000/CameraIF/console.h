#ifndef H_CONSOLE
#define H_CONSOLE
#include "uart.h"
#include "board.h"

#define SERIAL_BUFFER_SIZE 256
#define CON_BAUD 921600   //UART baudrate used for sending data to PC, use max 921600 for serial stream

typedef enum {
    CMD_UNKNOWN = -1,
    CMD_CAPTURE = 0,
    CMD_IMGRES,
    CMD_ENABLE_DMA,
    CMD_DISABLE_DMA,
    CMD_STREAM
} cmd_t;

static mxc_uart_regs_t* Con_Uart = MXC_UART_GET_UART(CONSOLE_UART);
extern char g_serial_buffer[SERIAL_BUFFER_SIZE];
extern int g_buffer_index;
extern int g_num_commands;

void console_init();
int send_msg(const char* msg);
int recv_msg(char* buffer);
int recv_cmd(cmd_t* out_cmd);
void clear_serial_buffer(void);
void print_help(void);

#endif
