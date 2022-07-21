#include "console.h"
#include <string.h>

char g_serial_buffer[SERIAL_BUFFER_SIZE];
int g_buffer_index = 0;
int g_num_commands = 0;

int g_num_commands; // Calculated in 'main' as part of initialization
char* cmd_table[] = {
    "capture",
    "imgres",
    "enable_dma",
    "disable_dma",
    "stream"
};

int starts_with(char* a, char* b) {
    // Utility function for checking whether 'a' starts with 'b'
    char* ptr_a = a;
    char* ptr_b = b;

    while(*ptr_a && *ptr_b) {
        if (*a != *b) {
            return 0;            
        }
        ptr_a++;
        ptr_b++;
    }

    return 1;
}

void console_init(void) {
    g_num_commands = sizeof(cmd_table) / sizeof(char*);
    clear_serial_buffer();
}

int send_msg(const char* msg) {
    int ret = 0;
    int len = strlen(msg);

    // Transmit message string
    if ((ret = MXC_UART_Write(Con_Uart, (uint8_t*)msg, &len)) != E_NO_ERROR) {
        return ret;
    }
    // Transmit newline to complete the message.
    len = 1;
    if ((ret = MXC_UART_Write(Con_Uart, (uint8_t*)"\n", &len)) != E_NO_ERROR) {
        return ret;
    }

    return E_NO_ERROR;
}

int recv_msg(char* buffer) {
    int available = MXC_UART_GetRXFIFOAvailable(Con_Uart);
    while (available > 0) {
        char c = MXC_UART_ReadCharacter(Con_Uart);

        if (c == '\n') {
            // Received newline character, terminate the string and return
            g_serial_buffer[g_buffer_index] = '\0';
            return 1;
        }
        else {
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

int recv_cmd(cmd_t* out_cmd) {
    if (recv_msg(g_serial_buffer)) {
        cmd_t cmd = CMD_UNKNOWN;
        for (int i = 0;i < g_num_commands;i++) {
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

void clear_serial_buffer(void) {
    memset(g_serial_buffer, '\0', SERIAL_BUFFER_SIZE);
    g_buffer_index = 0;
}

void print_help(void) {
    int g_num_commands = sizeof(cmd_table) / sizeof(char*);
    printf("Registered %i total commands:\n", g_num_commands);
    for (int i = 0; i < g_num_commands; i++) {
        printf("Command %i: '%s'\n", i, cmd_table[i]);
    }
}