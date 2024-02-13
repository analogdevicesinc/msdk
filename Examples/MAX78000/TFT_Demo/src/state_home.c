/*
 * @file state_home.c
 *
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
#include <string.h>

#include "bitmap.h"
#include "keypad.h"
#include "state.h"
#include "example_config.h"

/*********************************      DEFINES      *************************/
#define TICK_TIMEOUT 2000

/*********************************      VARIABLES    *************************/
static int slide_mode = 0;

/********************************* Static Functions **************************/
static int init(void)
{
    MXC_TFT_SetPalette(footer_lightgrey_bottom_bmp);
    MXC_TFT_SetBackGroundColor(3);

    MXC_TFT_ShowImage(0, 240 - 48, footer_lightgrey_bottom_bmp);
    MXC_TFT_ShowImage(0, 240 - 48 - 8, footer_lightgrey_top_bg_lightgrey_bmp);
    MXC_TFT_ShowImage(11, 7, logo_white_bg_lightgrey_bmp);
    MXC_TFT_ShowImage(11, 77, slideshow_bg_lightgrey_bmp);
    MXC_TFT_ShowImage(117, 77, keyboard_bg_lightgrey_bmp);
    MXC_TFT_ShowImage(221, 77, info_bg_lightgrey_bmp);

    slide_mode = 0;

#ifdef ENABLE_TS
    MXC_TS_RemoveAllButton();
    MXC_TS_AddButton(11, 77, 11 + 87, 77 + 62, KEY_A);
    MXC_TS_AddButton(117, 77, 117 + 87, 77 + 62, KEY_0);
    MXC_TS_AddButton(221, 77, 221 + 87, 77 + 62, KEY_B);
#endif

    return 0;
}

static void time_tick(void)
{
    switch (slide_mode) {
    case 1:
        MXC_TFT_ShowImage(0, 0, img_1_bmp);
        break;

    case 2:
        MXC_TFT_ShowImage(0, 0, img_2_bmp);
        break;

    case 3:
        MXC_TFT_ShowImage(0, 0, img_3_bmp);
        break;

    case 4:
        MXC_TFT_ShowImage(0, 0, img_4_bmp);
        break;

    default:
        break;
    }

    if (slide_mode > 0) {
        if (++slide_mode > 5) {
            slide_mode = 1; // go first slide
        }
    }
}

static int key_process(int key)
{
    switch (key) {
    case KEY_0:
        state_set_current(get_keypad_state());
        break;

    case KEY_1:
        break;

    case KEY_2:
        break;

    case KEY_3:
        break;

    case KEY_4:
        break;

    case KEY_5:
        break;

    case KEY_6:
        break;

    case KEY_7:
        break;

    case KEY_8:
        break;

    case KEY_9:
        break;

    case KEY_A:
        if (slide_mode == 0) {
#ifdef ENABLE_TS
            MXC_TS_RemoveAllButton();
            MXC_TS_AddButton(0, 0, 320, 240, KEY_C);
#endif
            slide_mode = 1;
            time_tick();
        }

        break;

    case KEY_B:
        state_set_current(get_info_state());
        break;

    case KEY_C:
        if (slide_mode != 0) {
            init(); // go home
        }

        break;

    case KEY_D:
        break;

    case KEY_E:
        break;

    case KEY_F:
        break;

    default:
        break;
    }

    return 0;
}

static State g_state = { "home", init, key_process, time_tick, TICK_TIMEOUT };

/********************************* Public Functions **************************/
State *get_home_state(void)
{
    return &g_state;
}
