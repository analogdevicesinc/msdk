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

#ifndef EXAMPLES_MAX32655_DEMO_2048_RISCV_INC_CONTROLLER_H_
#define EXAMPLES_MAX32655_DEMO_2048_RISCV_INC_CONTROLLER_H_

/* **** Includes **** */

#include <stdint.h>
#include "mxc_device.h"
#include "uart.h"

/* **** Definitions **** */

/* **** Function Prototypes **** */

/**
 * @brief   Initialize the user controller (PC Keyboard).
 *              - Use the system clock for UART which should be the IPO (fastest).
 * 
 * @param   uart    Select which UART port the controller is connected to.
 * @param   baud    Select the highest baud rate that the device can support.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int Controller_Init(mxc_uart_regs_t *uart, uint32_t baud);

/**
 * @brief   Start the Controller and listen for any keypresses.
 * 
 * @param   req    UART request struct needed for transactions.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int Controller_Start(mxc_uart_req_t *req);

#endif // EXAMPLES_MAX32655_DEMO_2048_RISCV_INC_CONTROLLER_H_
