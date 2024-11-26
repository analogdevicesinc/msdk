/*************************************************************************************************/
/*!
 *  @file    main.c
 *  @brief   PAL UART, echoes input via Serial Port.
 *
 *  Copyright (c) 2013-2019 Arm Ltd. All Rights Reserved.
 *
 *  Copyright (c) 2019-2020 Packetcraft, Inc.
 *
 *  Portions Copyright (c) 2022-2023 Analog Devices, Inc.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
/*************************************************************************************************/
#include <stdio.h>
#include "mxc_delay.h"
#include "uart.h"
#include "board.h"
#include "pal_uart.h"

/**************************************************************************************************
  Global Variables
**************************************************************************************************/
static uint8_t rxData[10];

/**************************************************************************************************
  Functions
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Read callback that immediately echoes.
 */
/*************************************************************************************************/
void rd_callback(void)
{
    PalUartWriteData(PAL_UART_ID_TERMINAL, rxData, 1);
}

/*************************************************************************************************/
/*!
 *  \brief  A write callback that reads after echoing.
 */
/*************************************************************************************************/
void wr_callback(void)
{
    PalUartReadData(PAL_UART_ID_TERMINAL, rxData, 1);
}

/*************************************************************************************************/
/*!
 *  \brief  Main entry point.
 */
/*************************************************************************************************/
int main(void)
{
    printf("Starting\n");
    MXC_Delay(500000);

    const PalUartConfig_t pCfg = { .rdCback = rd_callback, .wrCback = wr_callback, .baud = 115200 };

    PalUartInit(PAL_UART_ID_TERMINAL, &pCfg);
    PalUartReadData(PAL_UART_ID_TERMINAL, rxData, 1);

    while (1) {}

    /* Does not return. */
    return 0;
}
