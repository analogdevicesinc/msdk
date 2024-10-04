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
 * @file    main_core1.c
 * @brief   The main application for Core 1.
 * @details Core 1 synchronizes with Core 0.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_delay.h"
#include "led.h"
#include "board.h"
#include "sema.h"
#include "tmr.h"

/***** Definitions *****/
extern int count0;
extern int count1;

/***** Globals *****/

/***** Functions *****/

// *****************************************************************************
// main_core1 is Core 1's official main function that is called at program startup.
int main_core1(void)
{
    printf("Core 1: enter while loop.\n");
    while (1) {
        // Wait for Core 0 to release the semaphore
        while (MXC_SEMA_GetSema(1) == E_BUSY) {}

        // Print the updated value from Core 0
        printf("Core 1: Ping: %d\n", count1);

        LED_Off(0);
        LED_On(1);

        MXC_TMR_Delay(MXC_TMR1, MXC_DELAY_MSEC(500));

        // Update the count for Core 0 and release the semaphore
        count0++;
        MXC_SEMA_FreeSema(0);
    }
}
