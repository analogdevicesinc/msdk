#include <time.h>
#include "transports.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "dma.h"
#include "nvic_table.h"
#include "gpio.h"

static TaskHandle_t xTaskToNotify = NULL;
const UBaseType_t xArrayIndexWrite = 1, xArrayIndexRead = 2;

volatile bool tx_flag = 0;
volatile bool rx_flag = 0;

mxc_gpio_cfg_t indicator = {
    .port = MXC_GPIO2,
    .mask = MXC_GPIO_PIN_4,
    .pad = MXC_GPIO_PAD_NONE,
    .func = MXC_GPIO_FUNC_OUT,
};

static inline int TS_TO_MSEC(struct timespec ts)
{
    return (int)((ts.tv_sec * 1000) + (ts.tv_nsec / 1000000));
}

bool MXC_Serial_Open(mxc_uart_regs_t *uart_instance, unsigned int baudrate);
size_t MXC_Serial_Write (mxc_uart_regs_t *uart, const uint8_t* buffer, size_t length);
size_t MXC_Serial_Read (mxc_uart_regs_t *uart, uint8_t* buffer, size_t length, int timeout);

// void TX_DMA_ISR(void)
// {
//     MXC_DMA_Handler();
// }

// void RX_DMA_ISR(void)
// {
//     MXC_DMA_Handler();
// }

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

    MXC_GPIO_Config(&indicator);
    MXC_GPIO_OutClr(indicator.port, indicator.mask);
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

    MXC_UART_SetAutoDMAHandlers(uart_instance, true);

    // NVIC_EnableIRQ(DMA0_IRQn);
    // MXC_NVIC_SetVector(DMA0_IRQn, TX_DMA_ISR);
    
    // NVIC_EnableIRQ(DMA1_IRQn);
    // MXC_NVIC_SetVector(DMA1_IRQn, RX_DMA_ISR);
    return true;
}

/**
 * @brief Function signature callback for closing a custom transport.
 * @param transport     Custom transport structure. Has the args passed through bool uxr_init_custom_transport(uxrCustomTransport* transport, void * args);.
 * @return      true in case of successful closed transport, and false in other case.
 */
bool vMXC_Serial_Close(struct uxrCustomTransport* transport) 
{
    transport_config_t *args = (transport_config_t *)transport->args;
    int error = MXC_UART_Shutdown(args->uart_instance);
    return (error == E_NO_ERROR);
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

    tx_flag = 0;
    MXC_UART_TransactionDMA(&req);

    while(!tx_flag) {}

    return req.txCnt;
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
    return MXC_Serial_Read(args->uart_instance, buffer, length, timeout);
}

size_t MXC_Serial_Read (mxc_uart_regs_t *uart, uint8_t* buffer, size_t length, int timeout)
{
    MXC_GPIO_OutSet(indicator.port, indicator.mask);

    mxc_uart_req_t req = {
        .uart = uart,
        .rxLen = length,
        .rxData = buffer,
        .txLen = 0,
        .txData = NULL,
        .callback = rx_callback
    };

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    unsigned int start_ms = TS_TO_MSEC(ts);
    unsigned int elapsed = 0;

    rx_flag = 0;
    MXC_UART_TransactionDMA(&req);

    while(!rx_flag) {
        clock_gettime(CLOCK_REALTIME, &ts);
        elapsed = TS_TO_MSEC(ts) - start_ms;
        if (elapsed > timeout) {
            MXC_UART_AbortTransmission(uart);
            break;
        }
    }

    MXC_GPIO_OutClr(indicator.port, indicator.mask);

    return req.rxCnt;
}