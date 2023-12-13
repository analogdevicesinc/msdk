/**
 * @file
 * @brief   Demonstrates the SysTick timer and interrupt. Toggles LED0 every SysTick period.
 */

/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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
