/******************************************************************************
 *
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

#include <time.h>
#include "mxc_transports.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "dma.h"
#include "nvic_table.h"
#include "gpio.h"
#include "queue.h"

static TaskHandle_t xTaskToNotify = NULL;

#define FIFO_BUFFER_SIZE 256

/* Our UART FIFOs are pretty shallow (8 bytes), so we'll configure a DMA channel
to constantly unload the UART RX FIFO into a queue that our custom read transport
function can pull from.  This prevents overflow and missed data, especially in 
framed mode.  

The micro-ros library issues two separate read calls in framed mode, one for 
the frame and one for the data.  There can be a significant delay between the 
frame request and the data request.
[read frame] [~1ms delay] [read data]
*/


QueueHandle_t rx_queue; // A FreeRTOS queue will work and has built-in mechanisms for thread safety
uint8_t rx_buf; // Buffer for received byte
mxc_uart_req_t rx_req; // UART request, initialized in MXC_Serial_Open

SemaphoreHandle_t uart_tx_mutex;
SemaphoreHandle_t uart_rx_mutex;

extern int usleep (useconds_t __useconds);

mxc_gpio_cfg_t indicator = {
    .port = MXC_GPIO2,
    .mask = MXC_GPIO_PIN_4,
    .pad = MXC_GPIO_PAD_NONE,
    .func = MXC_GPIO_FUNC_OUT,
};

void RX_DMA_Callback(mxc_uart_req_t *req, int error)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendToBackFromISR(rx_queue, &rx_buf, &xHigherPriorityTaskWoken);
    MXC_UART_TransactionDMA(&rx_req);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

volatile int tx_flag = 0;
void tx_callback(mxc_uart_req_t *req, int error)
{
    tx_flag = 1;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    configASSERT( xTaskToNotify != NULL );
    vTaskNotifyGiveFromISR( xTaskToNotify, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
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

    uart_tx_mutex = xSemaphoreCreateMutex();
    uart_rx_mutex = xSemaphoreCreateMutex();

    int error = MXC_UART_Init(args->uart_instance, args->baudrate, MXC_UART_APB_CLK);
    if (error) {
        return false;
    }

    /* Initialize queue */
    rx_queue = xQueueCreate(FIFO_BUFFER_SIZE, sizeof(uint8_t));
    if (rx_queue == NULL) {
        return false;
    }

    error = MXC_DMA_Init();
    if (error)
        return false;

    /* Acquire DMA channels */
    int rx_channel = MXC_DMA_AcquireChannel();
    NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(rx_channel));
    MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(rx_channel), MXC_DMA_Handler); // Default handler (dma.h)
    MXC_UART_SetRXDMAChannel(args->uart_instance, rx_channel);

    int tx_channel = MXC_DMA_AcquireChannel();
    NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(tx_channel));
    MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(tx_channel), MXC_DMA_Handler); // Default handler (dma.h)
    MXC_UART_SetTXDMAChannel(args->uart_instance, tx_channel);

    /* Set up DMA transaction (will be continuously re-armed in callback) */
    rx_req.uart = args->uart_instance;
    rx_req.rxData = &rx_buf;
    rx_req.rxLen = 1;
    rx_req.txData = NULL;
    rx_req.txLen = 0;
    rx_req.callback = RX_DMA_Callback;

    error = MXC_UART_TransactionDMA(&rx_req);
    if (error) {
        return false;
    }

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
    TickType_t elapsed = 0;
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(timeout);

    MXC_GPIO_OutSet(indicator.port, indicator.mask); // A

    unsigned int num_received = 0;
    if (xSemaphoreTake(uart_tx_mutex, xMaxBlockTime) == pdTRUE) {
        while(num_received < length && elapsed < timeout) {
            if (uxQueueMessagesWaiting(rx_queue) > 0) {
                if(xQueueReceive(rx_queue, &buffer[num_received], 1)) {
                    num_received++;
                }
            }
            usleep(1000); // Sleep 1ms
            elapsed++;
        }

        xSemaphoreGive(uart_tx_mutex);
    }

    /* 
    Reset the queue if the number of bytes received was less than expected.
    This is very important to prevent bogging down the queue, especially
    with best effort streams.  Failing to do this will eventually cause the
    transports to completely break.
    */
    if (num_received < length){
        xQueueReset(rx_queue);
    }

    MXC_GPIO_OutClr(indicator.port, indicator.mask); // B

    return num_received;
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

    uint32_t ulNotificationValue;

    mxc_uart_req_t req = {
        .uart = args->uart_instance,
        .rxLen = 0,
        .rxData = NULL,
        .txLen = length,
        .txData = buffer,
        .callback = tx_callback
    };

    tx_flag = 0;

    xTaskToNotify = xTaskGetCurrentTaskHandle();

    if (xSemaphoreTake(uart_tx_mutex, portMAX_DELAY) == pdTRUE) {
        MXC_UART_TransactionDMA(&req);
        ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (ulNotificationValue != 1) {
            while(1) {}
            // TODO: Value is hitting here, which seems to indicate timeout.  Not sure why
        }
        // while (!tx_flag); // TODO: ulTaskNotifyTake doesn't seem to be working?

        xSemaphoreGive(uart_tx_mutex);
    }

    return req.txCnt;
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
