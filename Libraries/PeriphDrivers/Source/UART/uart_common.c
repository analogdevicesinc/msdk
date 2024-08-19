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

#include "uart_common.h"
#include "uart.h"

int MXC_UART_Common_ReadCharacter(mxc_uart_regs_t *uart)
{
    // Wait until FIFO has a character ready.
    while (MXC_UART_GetRXFIFOAvailable(uart) < 1) {}

    // Read the character using the non-blocking function.
    return MXC_UART_ReadCharacterRaw(uart);
}

int MXC_UART_Common_WriteCharacter(mxc_uart_regs_t *uart, uint8_t character)
{
    // Wait until FIFO has space for the character.
    while (MXC_UART_GetTXFIFOAvailable(uart) < 1) {}

    // Write the character using the non-blocking function.
    return MXC_UART_WriteCharacterRaw(uart, character);
}
