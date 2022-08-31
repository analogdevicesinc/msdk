/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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
 ******************************************************************************/

/**
 * @file    main.c
 * @brief   ADC demo application
 * @details Continuously monitors the ADC channels
 */

/***** Includes *****/
#include "board.h"
#include "led.h"
#include "lp.h"
#include "lpcmp.h"
#include "mxc_errors.h"
#include "nvic_table.h"
#include "pb.h"
#include "uart.h"
#include <stdio.h>

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
    while (!PB_Get(0)) { }

    // Enable comparator 0
    MXC_LP_EnableLPCMPWakeup(LPCMP);
    MXC_LPCMP_Init(LPCMP);

    // Enable comparator interrupts
    MXC_LPCMP_EnableInt(LPCMP, MXC_LPCMP_POL_RISE);
    MXC_NVIC_SetVector(LPCMP_IRQn, CMP_Handler);
    NVIC_EnableIRQ(LPCMP_IRQn);

    while (1) {
        printf("\nEntering sleep mode.\n");
        while (MXC_UART_GetActive(MXC_UART_GET_UART(CONSOLE_UART))) { }
        MXC_LP_EnterSleepMode();
        printf("Waking up.\n");
    }
}
