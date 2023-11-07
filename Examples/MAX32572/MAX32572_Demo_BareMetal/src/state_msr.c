/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc., All Rights Reserved.
 * (now owned by Analog Devices, Inc.)
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
 ******************************************************************************
 *
 * Copyright 2023 Analog Devices, Inc.
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
#include <string.h>

#include "state.h"
#include "utils.h"
#include "task_msr.h"

/*********************************      DEFINES      *************************/
#define TICK_TIMEOUT 10
#define DISPLAY_MSG_TIMEOUT 1000

/*********************************      VARIABLES    *************************/
static text_t text_msg[] = {
    { (char *)"MAG STRIPE", 10 },
#ifndef MN_EvKit_V1
    { (char *)"Swipe a card", 12 },
    { (char *)"Track:", 6 },
#else
    { (char *)"This EvKit does not support MSR", 31 }
#endif
};

#ifndef MN_EvKit_V1
static area_t area_clean = { 0, 0, 0, 0 };
static area_t area_cleanMSG = { 0, 0, 0, 0 };

static int g_tick_counter = 0;
#endif

/********************************* Static Functions **************************/
static int init(void)
{
    MXC_TFT_SetBackGroundColor(0);
    MXC_TFT_PrintFont(101, 12, urw_gothic_16_bleu_bg_grey, &text_msg[0], NULL); // "MAG STRIPE"
    MXC_TFT_ShowImage(33, 12, integrated_only_small_bmp);
    MXC_TFT_ShowImage(98, 69, magstripe_large_bmp);
    MXC_TFT_ShowImage(135, 191, home_bmp);

#ifndef MN_EvKit_V1
    MXC_TFT_PrintFont(101, 40, urw_gothic_12_white_bg_grey, &text_msg[1],
                      &area_clean); //"Swipe a card", 12
#else
    MXC_TFT_PrintFont(23, 40, urw_gothic_12_white_bg_grey, &text_msg[1], NULL); //
#endif

    MXC_TS_RemoveAllButton();
    MXC_TS_AddButton(135, 191, (135 + 48), (191 + 39), 'C'); //Home

#ifndef MN_EvKit_V1
    g_tick_counter = 0;
    msr_start();
#endif

    return 0;
}

static int key_process(unsigned int key)
{
    switch (key) {
    case KEY_C: // exit
#ifndef MN_EvKit_V1
        msr_stop();
#endif
        state_set_current(get_home_state());
        break;
    default:
        break;
    }

    return 0;
}

#ifndef MN_EvKit_V1
static int time_tick(void)
{
    char msg[64];
    int msg_len = 0;

    msr_tick(msg, &msg_len);
    if (msg_len > 0) {
        text_t msg_var;

        MXC_TFT_ClearArea(&area_clean, 0);
        msg_var.data = (char *)msg;
        msg_var.len = msg_len;
        if (msg_var.len > 18) {
            msg_var.len = 18;
        }
        MXC_TFT_PrintFont(82, 162, urw_gothic_16_white_bg_grey, &msg_var, &area_clean);
        MXC_TFT_PrintFont(17, 162, urw_gothic_16_bleu_bg_grey, &text_msg[2],
                          &area_cleanMSG); // "Track:",

        g_tick_counter = 1; // means enable timeout
    }

    if (g_tick_counter > 0) {
        if (++g_tick_counter > (DISPLAY_MSG_TIMEOUT / TICK_TIMEOUT)) {
            g_tick_counter = 0; // stop counter

            // Swipe again
            MXC_TFT_ClearArea(&area_clean, 0);
            MXC_TFT_ClearArea(&area_cleanMSG, 0);
            MXC_TFT_PrintFont(101, 40, urw_gothic_12_white_bg_grey, &text_msg[1],
                              &area_clean); //"Swipe a card",
        }
    }

    return 0;
}
#endif // for #ifndef MN_EvKit_V1

#ifndef MN_EvKit_V1
static State g_state = { "msr", init, key_process, time_tick, TICK_TIMEOUT };
#else
static State g_state = { "msr", init, key_process, NULL, 0 };
#endif

/********************************* Public Functions **************************/
State *get_msr_state(void)
{
    return &g_state;
}
