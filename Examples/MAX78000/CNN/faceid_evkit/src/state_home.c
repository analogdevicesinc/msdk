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
#include <string.h>
#include "board.h"

#include "keypad.h"
#include "state.h"
#include "utils.h"
#ifdef BOARD_FTHR_REVA
#include "tft_ili9341.h"
#endif
#ifdef BOARD_EVKIT_V1
#include "tft_ssd2119.h"
#include "bitmap.h"
#endif

#define X_START 56
#define Y_START 156
#define THICKNESS 4

/********************************** Type Defines  *****************************/
typedef void (*ScreenFunc)(void);

/************************************ VARIABLES ******************************/
#ifdef TFT_ENABLE
static void screen_faceID_home(void);

static text_t screen_msg[] = {
    // info
    { (char *)"FACEID DEMO", 11 },
    { (char *)"Start DEMO", 10 },
};

#ifdef BOARD_EVKIT_V1
static int bitmap = logo_white_bg_darkgrey_bmp;
static int font = urw_gothic_12_grey_bg_white;
#endif

#ifdef BOARD_FTHR_REVA
static int bitmap = (int)&logo_rgb565[0];
static int font = (int)&SansSerif16x16[0];
#endif

/********************************* Static Functions **************************/
static void screen_faceID_home(void)
{
    MXC_TFT_SetPalette(bitmap);
    MXC_TFT_SetBackGroundColor(4);
    //MXC_TFT_ShowImage(3, 5, bitmap);

    MXC_TFT_PrintFont(98, 5, font, &screen_msg[0], NULL); // FACEID DEMO
    MXC_TFT_PrintFont(X_START + THICKNESS, Y_START + THICKNESS, font, &screen_msg[1],
                      NULL); // START DEMO
    // texts

    area_t left = { X_START, Y_START, 4, 4 + 26 };
    MXC_TFT_ClearArea(&left, 5);

    area_t right = { X_START + 120, Y_START, 4, 4 + 26 };
    MXC_TFT_ClearArea(&right, 5);

    area_t top = { X_START, Y_START, 120, 4 };
    MXC_TFT_ClearArea(&top, 5);

    area_t bottom = { X_START, Y_START + 26, 120, 4 };
    MXC_TFT_ClearArea(&bottom, 5);
#ifdef TS_ENABLE
    MXC_TS_RemoveAllButton();
    MXC_TS_AddButton(X_START, Y_START, X_START + 120, Y_START + 4 + 26 + 4, KEY_1);
#endif
}
#endif

static int init(void)
{
#ifdef TFT_ENABLE
    screen_faceID_home();
#endif
    return 0;
}

static int key_process(int key)
{
    switch (key) {
    case KEY_1:
        state_set_current(get_faceID_state());
        break;
    default:
        break;
    }

    return 0;
}

static State g_state = { "faceID_home", init, key_process, NULL, 0 };

/********************************* Public Functions **************************/
State *get_home_state(void)
{
    return &g_state;
}
