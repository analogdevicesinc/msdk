/**
 * @file
 * @brief   Demonstrates the SysTick timer and interrupt. Toggles LED0 every SysTick period.
 */

/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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

/* **** Includes **** */
#include <stdio.h>
#include <stdint.h>
#include "mxc_errors.h"
#include "mxc_sys.h"
#include "nvic_table.h"
#include "board.h"
#include "led.h"

/* **** Definitions **** */
#define USE_SYSTEM_CLK 1
#define SYSTICK_PERIOD_SYS_CLK 4800000 //40ms with 120MHz system clock
#define SYSTICK_PERIOD_EXT_CLK 3277 //100ms with 32768Hz external RTC clock

/* **** Globals **** */

/* **** Functions **** */

// *****************************************************************************
void SysTick_Handler(void)
{
    //Toggle LED0 every systick period
    LED_Toggle(0);
}

// *****************************************************************************
int main(void)
{
    printf("\n************ Blinky SysTick ****************\n");
    uint32_t sysTicks;

    if (USE_SYSTEM_CLK) {
        sysTicks = SYSTICK_PERIOD_SYS_CLK;
    } else {
        sysTicks = SYSTICK_PERIOD_EXT_CLK;
    }

    uint32_t error = MXC_SYS_SysTick_Config(sysTicks, USE_SYSTEM_CLK, NULL);

    printf("SysTick Clock = %d Hz\n", MXC_SYS_SysTick_GetFreq());
    printf("SysTick Period = %d ticks\n", sysTicks);

    if (error != E_NO_ERROR) {
        printf("ERROR: Ticks is not valid");
        LED_On(1);
    }

    LED_On(0);

    while (1) {}
}
