/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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
 * @brief   Hello World - Non-Secure.
 * @details This TrustZone-enabled exampled splits Hello_World into two partitions.
 *          The Secure world setups the transition to the Non-Secure world, and
 *          increments the counter. The Non-Secure world prints the count,
 *          and toggles the LED.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "led.h"
#include "pb.h"
#include "board.h"
#include "mxc_delay.h"

/***** Definitions *****/

/***** Globals *****/

/***** Functions *****/

// From the Secure code, loaded into Non-Secure project through secure_implib.o object file.
extern int IncrementCount_S(volatile int *count_ns);

// *****************************************************************************
int main(void)
{
    int count = 0;

    printf("Hello from Non-Secure World!\n");

    while (1) {
        LED_On(LED_RED);
        MXC_Delay(500000);
        LED_Off(LED_RED);
        MXC_Delay(500000);

        IncrementCount_S(&count);
        printf("count = %d\n", count);
    }
}
