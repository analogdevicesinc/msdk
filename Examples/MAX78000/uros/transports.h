#include <rmw_microros/rmw_microros.h>
#include "uart.h"

/**
 * @brief Function signature callback for opening a custom transport.
 * @param transport     Custom transport structure. Has the args passed through bool uxr_init_custom_transport(uxrCustomTransport* transport, void * args);.
 * @return      true in case of successful opened transport, and false in other case.
 */
bool MXC_Serial_Open(struct uxrCustomTransport* transport) 
{
    mxc_uart_regs_t *uart_instance = (mxc_uart_regs_t *)transport->args;
    if (MXC_UART_Init(uart_instance, 115200, MXC_UART_APB_CLK) != E_NO_ERROR) {
        return false;
    }
    MXC_UART_ClearRXFIFO(uart_instance);
    MXC_UART_ClearTXFIFO(uart_instance);
    return true;
}

/**
 * @brief Function signature callback for closing a custom transport.
 * @param transport     Custom transport structure. Has the args passed through bool uxr_init_custom_transport(uxrCustomTransport* transport, void * args);.
 * @return      true in case of successful closed transport, and false in other case.
 */
bool MXC_Serial_Close(struct uxrCustomTransport* transport) 
{
    mxc_uart_regs_t *uart_instance = (mxc_uart_regs_t *)transport->args;
    return (MXC_UART_Shutdown(uart_instance) == E_NO_ERROR);
}

/**
 * @brief Function signature callback for writing to a custom transport.
 * @param transport     Custom transport structure. Has the args passed through bool uxr_init_custom_transport(uxrCustomTransport* transport, void * args);.
 * @param buffer        Buffer to be sent.
 * @param length        Length of buffer.
 * @param error_code    Error code that should be set in case the write process experiences some error.
 * @return      Number of successfully sent bytes.
 */
size_t MXC_Serial_Write (
        struct uxrCustomTransport* transport,
        const uint8_t* buffer,
        size_t length,
        uint8_t* error_code)
{
    mxc_uart_regs_t *uart_instance = (mxc_uart_regs_t *)transport->args;
    for (int i = 0; i < length; i++) {
        // Wait until FIFO has space for the character.
        while (MXC_UART_GetTXFIFOAvailable(uart_instance) < 1) {}
        MXC_UART_WriteCharacterRaw(uart_instance, buffer[i]);
    }
    return length;
}

/**
 * @brief Function signature callback for reading from a custom transport.
 * @param transport     Custom transport structure. Has the args passed through bool uxr_init_custom_transport(uxrCustomTransport* transport, void * args);.
 * @param buffer        Buffer to write.
 * @param length        Maximum length of buffer.
 * @param timeout       Maximum timeout of the read operation.
 * @param error_code    Error code that should be set in case the write process experiences some error.
 * @return      Number of successfully received bytes.
 */
size_t MXC_Serial_Read (
        struct uxrCustomTransport* transport,
        uint8_t* buffer,
        size_t length,
        int timeout,
        uint8_t* error_code)
{
    int ticker = 0;
    int i = 0;
    mxc_uart_regs_t *uart_instance = (mxc_uart_regs_t *)transport->args;
    while(MXC_UART_GetRXFIFOAvailable(uart_instance) > 0 && i < length) {
        int result = MXC_UART_ReadCharacterRaw(uart_instance);
        if (result < 0) {
            // Error reading from FIFO
            return i;
        } else {
            buffer[i++] = (uint8_t)result;
        }
    }
    return i;
}