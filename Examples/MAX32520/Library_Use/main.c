/**
 * @file    main.c
 * @brief   Static library example
 * @details Calls static library functions to toggle an LED.
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
#include "mxc_delay.h"
#include "led.h"

#include "gpiolib.h"

/* ************************************************************************** */
int main(void)
{
    printf("\n\n*********************** Static Library Example **********************\n\n");
    printf("This example calls static library functions to toggle an LED.\n");

    gpio_clear(&led_pin[0]);
    while (1) {
        if (!gpio_get(&led_pin[0])) {
            gpio_set(&led_pin[0]);
        } else {
            gpio_clear(&led_pin[0]);
        }
        MXC_Delay(500000);
    }
}
