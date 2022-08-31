/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
 *
 * This software is protected by copyright laws of the United States and
 * of foreign countries. This material may also be protected by patent laws
 * and technology transfer regulations of the United States and of foreign
 * countries. This software is furnished under a license agreement and/or a
 * nondisclosure agreement and may only be used or reproduced in accordance
 * with the terms of those agreements. Dissemination of this information to
 * any party or parties not specified in the license agreement and/or
 * nondisclosure agreement is expressly prohibited.
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
#include <stdint.h>
#include <stdio.h>

#include <MAX32xxx.h>

#include "keypad.h"
#include "state.h"
#include "utils.h"

#include "tft_ssd2119.h"

/***** Definitions *****/

/***** Globals *****/

/***** Functions *****/
static int system_init(void)
{
    /* Check RTC Status */
    MXC_RTC_Init(0, 0);
    MXC_RTC_Start();

    // Initialize KBD
    keypad_init();

    //
    MXC_TFT_Init();

    //
    MXC_TS_Init();
    MXC_TS_Start();

    return 0;
}

/*****************************************************************************/
int main(void)
{
    int key;
    unsigned int start_time;
    State* state;

    system_init();

    //
    state_init();

    /* Infinite loop */
    start_time = utils_get_time_ms();

    while (1) {
        state = state_get_current();

        // check keyboard key
        key = keypad_getkey();

        if (key > 0) {
            state->prcss_key(key);
            start_time = utils_get_time_ms();
        }

        // check touch screen key
        key = MXC_TS_GetKey();

        if (key > 0) {
            state->prcss_key(key);
            start_time = utils_get_time_ms();
        }

        // check tick
        if (utils_get_time_ms() >= (start_time + state->timeout)) {
            if (state->tick) {
                state->tick();
                start_time = utils_get_time_ms();
            }
        }
    }
}
