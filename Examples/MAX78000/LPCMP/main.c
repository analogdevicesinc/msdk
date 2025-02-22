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

/**
 * @file    main.c
 * @brief   This example demonstrates the use of the Analog Comparator to wake up the device from sleep mode.
 */

/***** Includes *****/
#include <stdio.h>
#include "mxc_errors.h"
#include "nvic_table.h"
#include "board.h"
#include "led.h"
#include "lp.h"
#include "lpcmp.h"
#include "pb.h"
#include "uart.h"

/***** Definitions *****/
#if defined(BOARD_EVKIT_V1)
#define LPCMP MXC_LPCMP_CMP0
#define BUTTON_NUM 2
#elif defined(BOARD_FTHR_REVA)
#define LPCMP MXC_LPCMP_CMP3
#define BUTTON_NUM 1
#endif

/***** Globals *****/

/***** Functions *****/

void CMP_Handler(void)
{
    MXC_LPCMP_ClearFlags(LPCMP);
    LED_Toggle(1);
}

int main(void)
{
    printf("\n******************** Comparator Example ********************\n");
#if defined(BOARD_EVKIT_V1)
    printf("\nConnect the analog signal used as the positive comparator input\n");
    printf("to analog pin 1 (AIN1/AIN0P).\n");
    printf("Connect the analog signal used as the negative comparator input\n");
    printf("to analog pin 0 (AIN0/AIN0N).\n");
#elif defined(BOARD_FTHR_REVA)
    printf("\nConnect the analog signal used as the positive comparator input\n");
    printf("to analog pin 7 (TX pin on header J8).\n");
    printf("Connect the analog signal used as the negative comparator input\n");
    printf("to analog pin 6 (RX pin on header J8).\n");
#endif
    printf("\nThe device will be placed in sleep mode and requires a rising\n");
    printf("edge of the comparator output to wakeup.\n\n");

    printf("Press SW%d to begin.\n", BUTTON_NUM);
    while (!PB_Get(0)) {}

    // Enable comparator 0
    MXC_LP_EnableLPCMPWakeup(LPCMP);
    MXC_LPCMP_Init(LPCMP);

    // Enable comparator interrupts
    MXC_LPCMP_EnableInt(LPCMP, MXC_LPCMP_POL_RISE);
    MXC_NVIC_SetVector(LPCMP_IRQn, CMP_Handler);
    NVIC_EnableIRQ(LPCMP_IRQn);

    while (1) {
        printf("\nEntering sleep mode.\n");
        while (MXC_UART_GetActive(MXC_UART_GET_UART(CONSOLE_UART))) {}
        MXC_LP_EnterSleepMode();
        printf("Waking up.\n");
    }
}
