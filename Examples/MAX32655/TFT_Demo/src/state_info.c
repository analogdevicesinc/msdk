/*
 * @file state_info.c
 *
 ******************************************************************************
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

#include "bitmap.h"
#include "keypad.h"
#include "state.h"
#include "utils.h"
#include "tft.h"


//
#define urw_gothic_16_white_bg_grey 0


/********************************** Type Defines  *****************************/
typedef void (*ScreenFunc)(void);

/************************************ VARIABLES ******************************/
static void screen_info(void);
static void screen_info1(void);
static void screen_info2(void);
static void screen_info3(void);

static ScreenFunc info_screens[] = {screen_info, screen_info1, screen_info2, screen_info3 };
static unsigned int screen_index = 0;

static text_t screen_msg[] = {
    // info
    { (char*) "INFORMATION", 11},
    { (char*) "Scroll through pages", 20},
    { (char*) "TEXT 1", 6},
    { (char*) "TEXT 2", 6},
    { (char*) "TEXT 3", 6},
    // pages 1
    { (char*) "Cortex M4 @ 150MHz", 18},
    { (char*) "1MB Flash, 760KB SRAM", 21},
    { (char*) "TFT, ISO7816 PHY, MSR", 21},
    // pages 2
    { (char*) "6 dynamic sensor pairs", 22},
    { (char*) "Hardware crypto blocks", 22},
    { (char*) "ECDSA Bootloader", 16},
    // pages 3
    { (char*) "EMV-L1 stack", 12},
    { (char*) "Cryptographic library", 21},
    { (char*) "FreeRTOS OS", 11}
};

/********************************* Static Functions **************************/
static void screen_info(void)
{
    MXC_TFT_SetPalette(logo_white_bg_darkgrey_bmp);
    MXC_TFT_SetBackGroundColor(4);
    
    MXC_TFT_ShowImage(11,   7,      logo_white_bg_darkgrey_bmp);
    MXC_TFT_ShowImage(16,   80,     check_success_bg_darkgrey_bmp);
    MXC_TFT_ShowImage(16,   110,    check_success_bg_darkgrey_bmp);
    MXC_TFT_ShowImage(16,   140,    check_success_bg_darkgrey_bmp);
    
    MXC_TFT_PrintFont(110, 12, urw_gothic_16_white_bg_grey, &screen_msg[0],  NULL);  // information
    MXC_TFT_PrintFont(50, 50, urw_gothic_12_white_bg_grey, &screen_msg[1],  NULL);  // scroll through pages
    // texts
    MXC_TFT_PrintFont(46, 80,  urw_gothic_16_white_bg_grey, &screen_msg[2],  NULL);  // text 1
    MXC_TFT_PrintFont(46, 110, urw_gothic_16_white_bg_grey, &screen_msg[3],  NULL);  // text 2
    MXC_TFT_PrintFont(46, 140, urw_gothic_16_white_bg_grey, &screen_msg[4],  NULL);  // text 3
    
    MXC_TFT_ShowImage(12,   191,  left_arrow_bmp);
    MXC_TFT_ShowImage(135,  191,  home_bmp);
    MXC_TFT_ShowImage(259,  191,  right_arrow_bmp);
    
    MXC_TS_RemoveAllButton();
    MXC_TS_AddButton(12,  191,   12 + 48,  191 + 39,    KEY_1);
    MXC_TS_AddButton(135, 191,  135 + 48,  191 + 39,    KEY_2);
    MXC_TS_AddButton(259, 191,  259 + 48,  191 + 39,    KEY_3);
}

static void screen_info1(void)
{
    MXC_TFT_SetBackGroundColor(4);
    
    MXC_TFT_ShowImage(11,   7,      logo_white_bg_darkgrey_bmp);
    MXC_TFT_ShowImage(16,   80,     check_success_bg_darkgrey_bmp);
    MXC_TFT_ShowImage(16,   110,    check_success_bg_darkgrey_bmp);
    MXC_TFT_ShowImage(16,   140,    check_success_bg_darkgrey_bmp);
    
    MXC_TFT_PrintFont(110, 12, urw_gothic_16_white_bg_grey, &screen_msg[0],  NULL);  // information
    MXC_TFT_PrintFont(50, 50, urw_gothic_12_white_bg_grey, &screen_msg[1],  NULL);  // scroll through pages
    // texts
    MXC_TFT_PrintFont(46, 80,  urw_gothic_12_white_bg_grey, &screen_msg[5],  NULL);  // text 1
    MXC_TFT_PrintFont(46, 110, urw_gothic_12_white_bg_grey, &screen_msg[6],  NULL);  // text 2
    MXC_TFT_PrintFont(46, 140, urw_gothic_12_white_bg_grey, &screen_msg[7],  NULL);  // text 3
    
    MXC_TFT_ShowImage(12,   191,    left_arrow_bmp);
    MXC_TFT_ShowImage(135,  191,    home_bmp);
    MXC_TFT_ShowImage(259,  191,    right_arrow_bmp);
    
    MXC_TS_RemoveAllButton();
    MXC_TS_AddButton(12,  191,   12 + 48,  191 + 39,    KEY_1);
    MXC_TS_AddButton(135, 191,  135 + 48,  191 + 39,    KEY_2);
    MXC_TS_AddButton(259, 191,  259 + 48,  191 + 39,    KEY_3);
}

static void screen_info2(void)
{
    MXC_TFT_SetBackGroundColor(4);
    
    MXC_TFT_ShowImage(11,   7,      logo_white_bg_darkgrey_bmp);
    MXC_TFT_ShowImage(16,   80,     check_success_bg_darkgrey_bmp);
    MXC_TFT_ShowImage(16,   110,    check_success_bg_darkgrey_bmp);
    MXC_TFT_ShowImage(16,   140,    check_success_bg_darkgrey_bmp);
    
    MXC_TFT_PrintFont(110, 12, urw_gothic_16_white_bg_grey, &screen_msg[0],  NULL);  // information
    MXC_TFT_PrintFont(50, 50, urw_gothic_12_white_bg_grey, &screen_msg[1],  NULL);  // scroll through pages
    // texts
    MXC_TFT_PrintFont(46, 80,  urw_gothic_12_white_bg_grey, &screen_msg[8],  NULL);  // text 1
    MXC_TFT_PrintFont(46, 110, urw_gothic_12_white_bg_grey, &screen_msg[9],  NULL);  // text 2
    MXC_TFT_PrintFont(46, 140, urw_gothic_12_white_bg_grey, &screen_msg[10],  NULL);  // text 3
    
    MXC_TFT_ShowImage(12,   191,    left_arrow_bmp);
    MXC_TFT_ShowImage(135,  191,    home_bmp);
    MXC_TFT_ShowImage(259,  191,    right_arrow_bmp);
    
    MXC_TS_RemoveAllButton();
    MXC_TS_AddButton(12,  191,   12 + 48,  191 + 39,    KEY_1);
    MXC_TS_AddButton(135, 191,  135 + 48,  191 + 39,    KEY_2);
    MXC_TS_AddButton(259, 191,  259 + 48,  191 + 39,    KEY_3);
}

static void screen_info3(void)
{
    MXC_TFT_SetBackGroundColor(4);
    
    MXC_TFT_ShowImage(11,   7,      logo_white_bg_darkgrey_bmp);
    MXC_TFT_ShowImage(16,   80,     check_success_bg_darkgrey_bmp);
    MXC_TFT_ShowImage(16,   110,    check_success_bg_darkgrey_bmp);
    MXC_TFT_ShowImage(16,   140,    check_success_bg_darkgrey_bmp);
    
    MXC_TFT_PrintFont(110, 12, urw_gothic_16_white_bg_grey, &screen_msg[0],  NULL);  // information
    MXC_TFT_PrintFont(50, 50, urw_gothic_12_white_bg_grey, &screen_msg[1],  NULL);  // scroll through pages
    // texts
    MXC_TFT_PrintFont(46, 80,  urw_gothic_12_white_bg_grey, &screen_msg[11],  NULL);  // text 1
    MXC_TFT_PrintFont(46, 110, urw_gothic_12_white_bg_grey, &screen_msg[12],  NULL);  // text 2
    MXC_TFT_PrintFont(46, 140, urw_gothic_12_white_bg_grey, &screen_msg[13],  NULL);  // text 3
    
    MXC_TFT_ShowImage(12,   191,    left_arrow_bmp);
    MXC_TFT_ShowImage(135,  191,    home_bmp);
    MXC_TFT_ShowImage(259,  191,    right_arrow_bmp);
    
    MXC_TS_RemoveAllButton();
    MXC_TS_AddButton(12,  191,   12 + 48,  191 + 39,    KEY_1);
    MXC_TS_AddButton(135, 191,  135 + 48,  191 + 39,    KEY_2);
    MXC_TS_AddButton(259, 191,  259 + 48,  191 + 39,    KEY_3);
}

static int init(void)
{
    screen_info();
    screen_index = 0;
    return 0;
}

static int key_process(int key)
{
    switch (key) {
    case KEY_1:
        if (screen_index > 0) {
            --screen_index;
            info_screens[screen_index]();
        }
        
        break;
        
    case KEY_2:
        state_set_current(get_home_state());
        break;
        
    case KEY_3:
        if (screen_index < (ARRAY_SIZE(info_screens) - 1)) {
            ++screen_index;
            info_screens[screen_index]();
        }
        
        break;
        
    case KEY_A:
        break;
        
    case KEY_B:
        break;
        
    case KEY_C:
        state_set_current(get_home_state());
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

static State g_state = {"info", init, key_process, NULL, 0 };

/********************************* Public Functions **************************/
State* get_info_state(void)
{
    return &g_state;
}

