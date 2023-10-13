#include <rmw_microros/rmw_microros.h>
#include "rmw/rmw/ret_types.h"
#include "uart.h"
#include "time.h"
#include "nvic_table.h"

typedef struct custom_transport_args {
    mxc_uart_regs_t * uart_instance;
} custom_transport_args_t;

static custom_transport_args_t transport_cfg = {
    .uart_instance = MXC_UART0
};

static volatile bool rx_finished = false, tx_finished = false;

void UART_ISR(void)
{
    MXC_UART_AsyncHandler(transport_cfg.uart_instance);
}

void UART_RX_Callback(mxc_uart_req_t * req, int error_code)
{
    printf("error code: %i\n", error_code);
    rx_finished = true;
}

void UART_TX_Callback(mxc_uart_req_t * req, int error_code)
{
    tx_finished = true;
}

/**
 * @brief Function signature callback for opening a custom transport.
 * @param transport     Custom transport structure. Has the args passed through bool uxr_init_custom_transport(uxrCustomTransport* transport, void * args);.
 * @return      true in case of successful opened transport, and false in other case.
 */
bool MXC_Serial_Open(struct uxrCustomTransport* transport) 
{
    custom_transport_args_t *args = (custom_transport_args_t *)transport->args;
    int error = MXC_UART_Init(args->uart_instance, 115200, MXC_UART_APB_CLK);
    if (error != E_NO_ERROR) {
        return false;
    }
    MXC_UART_ClearRXFIFO(args->uart_instance);
    MXC_UART_ClearTXFIFO(args->uart_instance);

    NVIC_EnableIRQ(MXC_UART_GET_IRQ(MXC_UART_GET_IDX(args->uart_instance)));
    MXC_NVIC_SetVector(MXC_UART_GET_IRQ(MXC_UART_GET_IDX(args->uart_instance)), UART_ISR);
    return true;
}

/**
 * @brief Function signature callback for closing a custom transport.
 * @param transport     Custom transport structure. Has the args passed through bool uxr_init_custom_transport(uxrCustomTransport* transport, void * args);.
 * @return      true in case of successful closed transport, and false in other case.
 */
bool MXC_Serial_Close(struct uxrCustomTransport* transport) 
{
    struct custom_transport_args *args = (struct custom_transport_args *)transport->args;
    return (MXC_UART_Shutdown(args->uart_instance) == E_NO_ERROR);
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
    struct custom_transport_args *args = (struct custom_transport_args *)transport->args;
    // mxc_uart_regs_t *uart_instance = (mxc_uart_regs_t *)args->uart_instance;
    // int transmitted = 0;
    // int fifo_available = 0;
    // while (transmitted < length) {
    //     fifo_available = MXC_UART_GetTXFIFOAvailable(uart_instance);
    //     if (fifo_available > 0) {
    //         transmitted += MXC_UART_WriteTXFIFO(uart_instance, &buffer[transmitted], fifo_available);
    //     }
    // }

    mxc_uart_req_t req = {
        .uart = args->uart_instance,
        .rxLen = 0,
        .rxData = NULL,
        .txLen = length,
        .txData = buffer,
        .callback = UART_TX_Callback
    };

    tx_finished = false;
    MXC_UART_TransactionAsync(&req);

    while(!tx_finished) {

    }

    return req.txCnt;
}

static inline int TS_TO_MSEC(struct timespec ts)
{
    return (int)((ts.tv_sec * 1000) + (ts.tv_nsec / 1000000));
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
    struct custom_transport_args *args = (struct custom_transport_args *)transport->args;
    // int fifo_available = 0;
    // int read = 0;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    int start_ms = TS_TO_MSEC(ts);
    int elapsed = 0;

    mxc_uart_req_t req = {
        .uart = args->uart_instance,
        .rxLen = length,
        .rxData = buffer,
        .txLen = 0,
        .txData = NULL,
        .callback = UART_RX_Callback
    };

    rx_finished = false;
    MXC_UART_TransactionAsync(&req);

    while(!rx_finished) {
        clock_gettime(CLOCK_REALTIME, &ts);
        elapsed = TS_TO_MSEC(ts) - start_ms;
        if (elapsed > timeout) {
            MXC_UART_AbortAsync(args->uart_instance);
            break;
        }
    }

    return req.rxCnt;
}