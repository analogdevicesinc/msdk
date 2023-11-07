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
#include "task_logo_animation.h"
#include "utils.h"

/*********************************      DEFINES      *************************/
#define TICK_TIMEOUT 60

/*********************************      VARIABLES    *************************/
static const int screensaver_data[] = {
    medium_logo_000_bmp, medium_logo_001_bmp, medium_logo_002_bmp, medium_logo_001_bmp,
    medium_logo_003_bmp, medium_logo_004_bmp, medium_logo_005_bmp, medium_logo_006_bmp,
    medium_logo_007_bmp, medium_logo_008_bmp, medium_logo_009_bmp, medium_logo_010_bmp,
    medium_logo_011_bmp, medium_logo_012_bmp, medium_logo_011_bmp, medium_logo_013_bmp,
    medium_logo_014_bmp, medium_logo_015_bmp, medium_logo_016_bmp, medium_logo_017_bmp,
    medium_logo_018_bmp, medium_logo_019_bmp, medium_logo_020_bmp, medium_logo_021_bmp,
    medium_logo_022_bmp, medium_logo_021_bmp, medium_logo_023_bmp, medium_logo_024_bmp,
    medium_logo_025_bmp, medium_logo_026_bmp, medium_logo_027_bmp, medium_logo_028_bmp,
    medium_logo_029_bmp, medium_logo_030_bmp, medium_logo_031_bmp, medium_logo_032_bmp,
    medium_logo_031_bmp, medium_logo_033_bmp, medium_logo_034_bmp, medium_logo_035_bmp
};

static unsigned int image_index = 0;
static unsigned int max_loop = ARRAY_SIZE(screensaver_data);

/********************************* Static Functions **************************/
static int init(void)
{
    MXC_TFT_SetPalette(maxim_integrated_large_bmp);
    MXC_TFT_SetBackGroundColor(0);
    MXC_TFT_ShowImage(52, 87, maxim_integrated_large_bmp);

    image_index = 0;

    MXC_TS_RemoveAllButton();
    MXC_TS_AddButton(0, 0, 320, 240, 'C'); //Home

    logo_animation_stop();

    return 0;
}

static int time_tick(void)
{
    MXC_TFT_ShowImage(52, 87, screensaver_data[image_index]);
    image_index = (image_index + 1) % max_loop;

    return 0;
}

static int key_process(unsigned int key)
{
    switch (key) {
    case KEY_C:
        state_set_current(get_home_state());
        break;
    default:
        break;
    }
    return 0;
}

static State g_state = { "idle", init, key_process, time_tick, TICK_TIMEOUT };

/********************************* Public Functions **************************/
State *get_idle_state(void)
{
    return &g_state;
}
