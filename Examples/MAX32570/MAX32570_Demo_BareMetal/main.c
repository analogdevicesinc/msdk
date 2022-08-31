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
 * @brief   MAX32570 Bare-metal Demo Example!
 *
 * @details
 */

#include <stdint.h>
#include <stdio.h>

#include "MAX32xxx.h"
#include "task_logo_animation.h"
#include "task_msr.h"
#include "task_nfc.h"
#include "task_smartcard.h"

#include "state.h"
#include "tft_ssd2119.h"
#include "utils.h"

/********************************* 		VARIABLES	 *************************/
static volatile unsigned int timeout_status = 0;

/******************************   STATIC FUNCTIONS  **************************/
static void timeout_cb(void)
{
    timeout_status = 1;
}

static int system_init(void)
{
    MXC_RTC_Init(0, 0);
    MXC_RTC_Start();

#if !defined(MN_EvKit_V1) && !defined(M_EvKit_V1)
    keypad_init();

    nfc_init();

    msr_init();
#endif

    sc_init();

    MXC_TFT_Init();

    MXC_TS_Init();
    MXC_TS_Start();

    timer_init(timeout_cb);

    return 0;
}

/*****************************************************************************/
int main(void)
{
    int ret = 0;
    State* state = NULL;
    int key;
    unsigned int timeout = 0;
    // cumulative time in idle condition, no any event.
    unsigned int total_idle_time = 0;
    // In idle after this time state will go to screen saver mode
    unsigned int max_idle_time = 15000;

    printf("\n************************** MAX32570 Demo Example **************************\n\n");
    printf("This example interact with user\n");
    printf(
        "Depend on the user selection on TFT display, some functionality of EvKit can be tested\n");
    printf("Please follow instruction on TFT Display\n");
    printf("Note:\n"
           "\tMSR: VBAT_SEL need to be connected to 3.3V, VDD_MSR need to be connected\n"
           "\tSmartCard can be configured to 5V mode (Class A) or 3V mode (Class B),\n"
           "\t		To configure 5V mode:\n"
           "\t			1- On EvKit connect SC_PWR_SEL jumper to 5V\n"
           "\t			2- In demo_config_h file update SMARTCARD_EXT_AFE_Voltage to 5V\n"
           "\t			3- Rebuild project and load it\n"
           "\t		To configure 3V mode:\n"
           "\t			1- On EvKit connect SC_PWR_SEL jumper to 3V\n"
           "\t			2- In demo_config_h file update SMARTCARD_EXT_AFE_Voltage to 3V\n"
           "\t			3- Rebuild project and load it\n");

    system_init();
    state_init();

    /* Infinite loop */
    while (1) {
        if (state != state_get_current()) {
            state = state_get_current();

            timeout = state->timeout;
            if (timeout == 0) {
                timeout = 1000; // 1 sec default timeout
            }

            timeout_status = 0;
            timer_start(timeout);
        }

#if !defined(MN_EvKit_V1) && !defined(M_EvKit_V1)
        // check keyboard key
        key = keypad_getkey();
        if (key > 0) {
            state->prcss_key(key);
            total_idle_time = 0;
            continue;
        }
#endif

        // check touch screen key
        key = MXC_TS_GetKey();
        if (key > 0) {
            state->prcss_key(key);
            total_idle_time = 0;
            continue;
        }

        /*
         *  check state timeout status
         */
        if (timeout_status) {
            if (state->tick) {
                ret = state->tick();
                if (ret == 0) { // means tick function is used, do not switch idle state
                    total_idle_time = 0;
                }
            }

            // check total idle time
            total_idle_time += timeout;
            if (total_idle_time >= max_idle_time) {
                state_set_current(get_idle_state());
                total_idle_time = 0;
            }

            // restart timeout
            timeout_status = 0;
            timer_start(timeout);
        }

        /*
         *  logo animation
         */
        logo_animation_tick();
    }

    return ret;
}
