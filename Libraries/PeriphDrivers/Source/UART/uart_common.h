/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_UART_UART_COMMON_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_UART_UART_COMMON_H_

#include "uart_regs.h"

int MXC_UART_Common_ReadCharacter(mxc_uart_regs_t *uart);
int MXC_UART_Common_WriteCharacter(mxc_uart_regs_t *uart, uint8_t character);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_UART_UART_COMMON_H_
