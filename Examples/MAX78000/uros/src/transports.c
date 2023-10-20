#include <time.h>
#include "transports.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "dma.h"
#include "nvic_table.h"
#include "gpio.h"
#include "tmr.h"

static TaskHandle_t xTaskToNotify = NULL;
const UBaseType_t xArrayIndexWrite = 1, xArrayIndexRead = 2;

volatile bool tx_flag = 0;
volatile bool rx_flag = 0;
mxc_uart_req_t rx_buf_req;

#define FIFO_BUFFER_SIZE 256

typedef enum {
    FIFO_BUFFER_SELECT_A,
    FIFO_BUFFER_SELECT_B
} select_t;

typedef struct {
    uint8_t data[FIFO_BUFFER_SIZE];
    uint8_t *head;
    uint8_t *tail;
} head_tail_buffer_t;

typedef struct {
    head_tail_buffer_t a, b;
    select_t selected;
    mxc_dma_ch_regs_t *dma_channel;
} fifo_buffer_t;

fifo_buffer_t rx_buffer, tx_buffer;

void head_tail_buffer_init(head_tail_buffer_t *ht_buffer)
{
    ht_buffer->head = &(ht_buffer->data[0]);
    ht_buffer->tail = ht_buffer->head;
    memset((void*)ht_buffer->data, 0, FIFO_BUFFER_SIZE);
}

void fb_init(fifo_buffer_t *fifo_buffer, mxc_dma_ch_regs_t *dma_channel)
{
    head_tail_buffer_init(&(fifo_buffer->a));
    head_tail_buffer_init(&(fifo_buffer->b));
    fifo_buffer->selected = FIFO_BUFFER_SELECT_A;
    fifo_buffer->dma_channel = dma_channel;
}

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

void RX_DMA_ISR(void)
{
    MXC_DMA_Handler();
}

void RX_DMA_Callback(mxc_uart_req_t *req, int error)
{
    printf("Hello callback!\n");
}

void rx_callback(mxc_uart_req_t *req, int error)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    configASSERT( xTaskToNotify != NULL );
    vTaskNotifyGiveIndexedFromISR( xTaskToNotify, xArrayIndexRead, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    rx_flag = 1;
}

void tx_callback(mxc_uart_req_t *req, int error)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    configASSERT( xTaskToNotify != NULL );
    vTaskNotifyGiveIndexedFromISR( xTaskToNotify, xArrayIndexWrite, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
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

    // MXC_UART_SetAutoDMAHandlers(uart_instance, true);

    int channel = MXC_DMA_AcquireChannel();
    NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(channel));
    MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(channel), RX_DMA_ISR);

    MXC_UART_SetRXDMAChannel(uart_instance, channel);

    fb_init(&rx_buffer, &MXC_DMA->ch[channel]);

    rx_buf_req.uart = uart_instance;
    rx_buf_req.rxData = &(rx_buffer.a.data[0]);
    rx_buf_req.rxLen = 8;
    rx_buf_req.txData = NULL;
    rx_buf_req.txLen = 0;
    rx_buf_req.callback = RX_DMA_Callback;

    error = MXC_UART_TransactionDMA(&rx_buf_req);
    if (error) {
        return false;
    }

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

    configASSERT( xTaskToNotify == NULL );
    xTaskToNotify = xTaskGetCurrentTaskHandle();

    uint32_t ulNotificationValue;
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(1000);

    mxc_uart_req_t req = {
        .uart = args->uart_instance,
        .rxLen = 0,
        .rxData = NULL,
        .txLen = length,
        .txData = buffer,
        .callback = tx_callback
    };

    tx_flag = 0;
    MXC_UART_TransactionDMA(&req);
#if 0
    ulNotificationValue = ulTaskNotifyTakeIndexed( xArrayIndexWrite, pdTRUE, xMaxBlockTime );

    if (ulNotificationValue != 1) {
        MXC_UART_AbortTransmission(args->uart_instance);
        *error_code = 1;
    }
#else
    while(!tx_flag);
#endif

    return req.txCnt;
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
    uint32_t ulNotificationValue;

    configASSERT( xTaskToNotify == NULL );
    xTaskToNotify = xTaskGetCurrentTaskHandle();

    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(timeout);

    MXC_GPIO_OutSet(indicator.port, indicator.mask); // A

    mxc_uart_req_t req = {
        .uart = args->uart_instance,
        .rxLen = length,
        .rxData = buffer,
        .txLen = 0,
        .txData = NULL,
        .callback = rx_callback
    };

    rx_flag = 0;
    MXC_UART_TransactionDMA(&req);

#if 0
    ulNotificationValue = ulTaskNotifyTakeIndexed(xArrayIndexRead, pdTRUE, xMaxBlockTime);

    MXC_GPIO_OutClr(indicator.port, indicator.mask); // B

    if (ulNotificationValue != 1) {
        MXC_UART_AbortTransmission(args->uart_instance);
        *error_code = 1;
    }
#else
    // while(!rx_flag);
#endif

    return req.rxCnt;
}

size_t MXC_Serial_Read (mxc_uart_regs_t *uart, uint8_t* buffer, size_t length, int timeout)
{
    MXC_GPIO_OutSet(indicator.port, indicator.mask); // A

    mxc_uart_req_t req = {
        .uart = uart,
        .rxLen = length,
        .rxData = buffer,
        .txLen = 0,
        .txData = NULL,
        .callback = rx_callback
    };

    rx_flag = 0;
    MXC_UART_TransactionDMA(&req);

    unsigned int elapsed_ms = 0;
    while(!rx_flag) {
        MXC_TMR_Delay(MXC_TMR0, 1000);
        elapsed_ms += 1;
        if (elapsed_ms > timeout) {
            MXC_UART_AbortTransmission(uart);
            break;
        }
    }

    MXC_GPIO_OutClr(indicator.port, indicator.mask); // B

    return req.rxCnt;
}