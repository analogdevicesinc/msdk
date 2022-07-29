/*
 * @file state_home.c
 *
 ******************************************************************************
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
#include <string.h>

#include "bitmap.h"
#include "keypad.h"
#include "state.h"
#include "tft_ssd2119.h"

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

    MXC_TS_RemoveAllButton();
    MXC_TS_AddButton(11, 77, 11 + 87, 77 + 62, KEY_A);
    MXC_TS_AddButton(117, 77, 117 + 87, 77 + 62, KEY_0);
    MXC_TS_AddButton(221, 77, 221 + 87, 77 + 62, KEY_B);

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
                MXC_TS_RemoveAllButton();
                MXC_TS_AddButton(0, 0, 320, 240, KEY_C);

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

static State g_state = {"home", init, key_process, time_tick, TICK_TIMEOUT};

/********************************* Public Functions **************************/
State* get_home_state(void)
{
    return &g_state;
}
