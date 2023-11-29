/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc., All Rights Reserved.
 * (now owned by Analog Devices, Inc.)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************
 *
 * Copyright 2023 Analog Devices, Inc.
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

/**
 * @file    main.c
 * @brief   read and write sdhc
 * @details This example uses the sdhc and ffat to read/write the file system on
 *          an SD card. The Fat library used supports long filenames (see ffconf.h)
 *          the max length is 256 characters. It uses the CLI library for taking user
 *          user commands.
 *
 *          You must connect an sd card to the sd card slot.
 */

/***** Includes *****/
#include "board.h"
#include "cli.h"
#include "nvic_table.h"
#include "sdhc.h"
#include "uart.h"
#include "user-cli.h"

#ifdef BOARD_EVKIT_V1
#warning This example is not supported by the MAX78000EVKIT.
#endif

/******************************************************************************/
int main(void)
{
    int err;
    printf("\n\n***** MAX78000 SDHC FAT Filesystem Example *****\n");

    // Wait for SD Card to be inserted
    waitCardInserted();

    printf("Card inserted.\n");
    while (MXC_UART_GetActive(MXC_UART_GET_UART(CONSOLE_UART))) {}

    // Initialize CLI
    if ((err = MXC_CLI_Init(MXC_UART_GET_UART(CONSOLE_UART), user_commands, num_user_commands)) !=
        E_NO_ERROR) {
        return err;
    }

    // Run CLI
    while (1) {}
}
