/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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

/* **** Includes **** */

#include <stdint.h>
#include "mxc_device.h"
#include "uart.h"

/* **** Definitions **** */

/* **** Globals **** */

static mxc_uart_regs_t *Controller_UART;

/* **** Functions **** */

void UART0_IRQHandler(void)
{
    MXC_UART_AsyncHandler(Controller_UART);
}

int Controller_Init(mxc_uart_regs_t *uart, uint32_t baud)
{
    int error;

    error = MXC_UART_Shutdown(uart);
    if (error != E_NO_ERROR) {
        return error;
    }

    // Set up System UART interrupt for Controller.
    //  Clear if Console UART was previously set up in SystemInit.
    NVIC_ClearPendingIRQ(MXC_UART_GET_IRQ((MXC_UART_GET_IDX(uart))));
    NVIC_DisableIRQ(MXC_UART_GET_IRQ((MXC_UART_GET_IDX(uart))));
    NVIC_EnableIRQ(MXC_UART_GET_IRQ((MXC_UART_GET_IDX(uart))));

    // This will re-initialize the console UART since the controller
    //  and console share the same UART port.
    error = MXC_UART_Init(uart, baud, MXC_UART_APB_CLK);
    if (error != E_NO_ERROR) {
        return error;
    }

    Controller_UART = uart;

    return E_NO_ERROR;
}

int Controller_Start(mxc_uart_req_t *req)
{
    int error;

    error = MXC_UART_TransactionAsync(req);
    if (error != E_NO_ERROR) {
        return error;
    }

    return E_NO_ERROR;
}
