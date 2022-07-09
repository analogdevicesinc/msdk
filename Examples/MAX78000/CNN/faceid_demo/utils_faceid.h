#ifndef UTILS_FACEID_H_
#define UTILS_FACEID_H_

#include "uart.h"
#include "sampledata.h"

#define DATA_SIZE_IN (160*120*3)
#define CONSOLE_UART 0
#define MXC_UARTn   MXC_UART_GET_UART(CONSOLE_UART)

// Data input: HWC (little data): 3x160x120
static const uint32_t input_0[] = INPUT_0;
extern uint8_t rxBuffer[DATA_SIZE_IN];

int uart_write(uint8_t* data, unsigned int len);

int uart_read(uint8_t* buffer, unsigned int len);

int wait_for_feedback();

void load_input(int8_t mode);

#endif
