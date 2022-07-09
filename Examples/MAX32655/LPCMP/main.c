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
#include <stdio.h>
#include <stdint.h>
#include "mxc_delay.h"
#include "mxc_errors.h"
#include "mxc_pins.h"
#include "nvic_table.h"
#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "led.h"
#include "lp.h"
#include "lpcmp.h"
#include "pb.h"
#include "tmr.h"
#include "uart.h"

/***** Definitions *****/

/***** Globals *****/

/***** Functions *****/

void CMP_Handler(void)
{
    MXC_LPCMP_ClearFlags(MXC_LPCMP_CMP0);
    LED_Toggle(1);
}

int main(void)
{
    printf("********** Comparator Example **********\n");
    printf("\nConnect the analog signal used as the positive comparator input to analog pin 1 (AIN1/AIN0P).\n");
    printf("Connect the analog signal used as the negative comparator input to analog pin 0 (AIN0/AIN0N).\n");
    printf("\nThe device will be placed in sleep mode and requires a rising edge of the\n");
    printf("comparator output to wakeup.\n\n");

    printf("Press SW2 to begin.\n");
    while(!PB_Get(0));

    // Enable comparator 0
    MXC_LP_EnableLPCMPWakeup(MXC_LPCMP_CMP0);
    MXC_LPCMP_Init(MXC_LPCMP_CMP0);

    // Enable comparator interrupts
    MXC_LPCMP_EnableInt(MXC_LPCMP_CMP0, MXC_LPCMP_POL_RISE);
	MXC_NVIC_SetVector(LPCMP_IRQn, CMP_Handler);
	NVIC_EnableIRQ(LPCMP_IRQn);

    while(1) {
        printf("\nEntering sleep mode.\n");
        while(MXC_UART_GetActive(MXC_UART_GET_UART(CONSOLE_UART)));
        MXC_LP_EnterSleepMode();
        printf("Waking up.\n");
    }
}
