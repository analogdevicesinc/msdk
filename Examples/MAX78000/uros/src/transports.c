#include "transports.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "dma.h"
#include "nvic_table.h"

static TaskHandle_t xTaskToNotify = NULL;
const UBaseType_t xArrayIndexWrite = 1, xArrayIndexRead = 2;

volatile bool tx_flag = 0;
volatile bool rx_flag = 0;

void TX_DMA_ISR(void)
{
    MXC_DMA_Handler();
}

void RX_DMA_ISR(void)
{
    MXC_DMA_Handler();
}

void rx_callback(mxc_uart_req_t *req, int error)
{
    rx_flag = 1;
}

void tx_callback(mxc_uart_req_t *req, int error)
{
    tx_flag = 1;
}

/**
 * @brief Function signature callback for opening a custom transport.
 * @param transport     Custom transport structure. Has the args passed through bool uxr_init_custom_transport(uxrCustomTransport* transport, void * args);.
 * @return      true in case of successful opened transport, and false in other case.
 */
bool vMXC_Serial_Open(struct uxrCustomTransport* transport)
{
    transport_config_t *args = (transport_config_t *)transport->args;
    return MXC_Serial_Open(args->uart_instance, args->baudrate);
}

bool MXC_Serial_Open(mxc_uart_regs_t *uart_instance, unsigned int baudrate)
{
    int error = MXC_UART_Init(uart_instance, baudrate, MXC_UART_APB_CLK);
    if (error) {
        return false;
    }

    error = MXC_DMA_Init();
    if (error)
        return false;

    NVIC_EnableIRQ(DMA0_IRQn);
    MXC_NVIC_SetVector(DMA0_IRQn, TX_DMA_ISR);
    
    NVIC_EnableIRQ(DMA1_IRQn);
    MXC_NVIC_SetVector(DMA1_IRQn, RX_DMA_ISR);
}

/**
 * @brief Function signature callback for closing a custom transport.
 * @param transport     Custom transport structure. Has the args passed through bool uxr_init_custom_transport(uxrCustomTransport* transport, void * args);.
 * @return      true in case of successful closed transport, and false in other case.
 */
bool vMXC_Serial_Close(struct uxrCustomTransport* transport) 
{

}

/**
 * @brief Function signature callback for writing to a custom transport.
 * @param transport     Custom transport structure. Has the args passed through bool uxr_init_custom_transport(uxrCustomTransport* transport, void * args);.
 * @param buffer        Buffer to be sent.
 * @param length        Length of buffer.
 * @param error_code    Error code that should be set in case the write process experiences some error.
 * @return      Number of successfully sent bytes.
 */
size_t vMXC_Serial_Write (
        struct uxrCustomTransport* transport,
        const uint8_t* buffer,
        size_t length,
        uint8_t* error_code) 
{
    transport_config_t *args = (transport_config_t *)transport->args;
    return MXC_Serial_Write(args->uart_instance, buffer, length);
}

size_t MXC_Serial_Write (mxc_uart_regs_t *uart, const uint8_t* buffer, size_t length)
{
    mxc_uart_req_t req = {
        .uart = uart,
        .rxLen = 0,
        .rxData = NULL,
        .txLen = length,
        .txData = buffer,
        .callback = tx_callback
    };

    while(uart->status & MXC_F_UART_STATUS_RX_BUSY) {
        // Our UART says it's full duplex, but results shows otherwise...
    }

    tx_flag = 0;
    MXC_UART_TransactionDMA(&req);

    while(!tx_flag) {}

    return true;
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
size_t vMXC_Serial_Read (
        struct uxrCustomTransport* transport,
        uint8_t* buffer,
        size_t length,
        int timeout,
        uint8_t* error_code)
{
    transport_config_t *args = (transport_config_t *)transport->args;
    return MXC_Serial_Read(args->uart_instance, buffer, length);
}

size_t MXC_Serial_Read (mxc_uart_regs_t *uart, uint8_t* buffer, size_t length)
{
    mxc_uart_req_t req = {
        .uart = uart,
        .rxLen = length,
        .rxData = buffer,
        .txLen = 0,
        .txData = NULL,
        .callback = rx_callback
    };

    while(uart->status & MXC_F_UART_STATUS_TX_BUSY) {
        // Our UART says it's full duplex, but results shows otherwise...
    }

    rx_flag = 0;
    MXC_UART_TransactionDMA(&req);

    while(!rx_flag) {}

    return true;
}