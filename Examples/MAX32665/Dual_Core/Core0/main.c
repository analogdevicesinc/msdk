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
 * @brief   The main application for Core 0.
 * @details This example is similar to the "Hello_World" example but the console
 *          UART and LEDs are split between Core 0 and Core 1.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_delay.h"
#include "led.h"
#include "board.h"
#include "sema.h"
#include "tmr.h"

/***** Definitions *****/
int count0 = 0;
int count1 = 0;

/***** Globals *****/

/***** Functions *****/

// *****************************************************************************
int main(void)
{
    printf("\n\n\n***** MAX32665 Dual Core Example *****\n");
    printf("Similar to the 'Hello World' example but split the\n");
    printf("lights and console uart between core 0 and core 1.\n");
    printf("Halting this example with a debugger will not stop core 1.\n\n");

    MXC_SEMA_Init();

    MXC_SEMA_GetSema(0);

    Start_Core1();

    MXC_SEMA_FreeSema(1);

    mxc_tmr_cfg_t tmr_cfg;
    tmr_cfg.pres = MXC_TMR_PRES_1;
    tmr_cfg.mode = MXC_TMR_MODE_CONTINUOUS;
    tmr_cfg.cmp_cnt = PeripheralClock / 2;
    tmr_cfg.pol = 0;
    MXC_TMR_Init(MXC_TMR2, &tmr_cfg);

    MXC_TMR_Start(MXC_TMR2);

    while (1) {
        // Wait for Core 1 to update count and release the semaphore
        while (MXC_SEMA_CheckSema(0) == E_BUSY) {}
        MXC_SEMA_GetSema(0);

        printf("Core 0: Pong: %d\n", count0);

        LED_On(0);
        LED_Off(1);

        MXC_TMR_Delay(MXC_TMR2, MXC_DELAY_MSEC(500));

        // Update the count for Core 1 and release the semaphore
        count1++;
        MXC_SEMA_FreeSema(1);
    }
}
