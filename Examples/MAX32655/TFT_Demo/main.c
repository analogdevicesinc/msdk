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

/**
 * @file    main.c
 * @brief   TFT Demo Example!
 *
 * @details
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "board.h"
#include "mxc.h"
#include "icc.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "utils.h"
#include "state.h"
#include "tft_ssd2119.h"
#include "bitmap.h"
#include "keypad.h"
#include "led.h"
#include "pb.h"

int main(void)
{
    /* Enable cache */
    MXC_ICC_Enable(MXC_ICC0);

    /* Set system clock to 100 MHz */
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();

    int key;
    unsigned int start_time;
    State *state;

    /* Initialize RTC */
    MXC_RTC_Init(0, 0);
    MXC_RTC_Start();

    /* Initialize TFT display */
    MXC_TFT_Init();

    /* Initialize Touch Screen controller */
    MXC_TS_Init();
    MXC_TS_Start();

    /* Display Home page */
    state_init();

    /* Get current time */
    start_time = utils_get_time_ms();

    while (1) {
        /* Get current screen state */
        state = state_get_current();

        /* Check pressed touch screen key */
        key = MXC_TS_GetKey();

        if (key > 0) {
            state->prcss_key(key);
            start_time = utils_get_time_ms();
        }

        /* Check timer tick */
        if (utils_get_time_ms() >= (start_time + state->timeout)) {
            if (state->tick) {
                state->tick();
                start_time = utils_get_time_ms();
            }
        }
    }
}
