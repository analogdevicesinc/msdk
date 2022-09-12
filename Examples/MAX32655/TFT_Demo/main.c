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
