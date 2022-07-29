/*
 ******************************************************************************
 * Copyright (C) Maxim Integrated Products, Inc., All rights Reserved.
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
 ******************************************************************************
 */
#include <string.h>

#include "state.h"
#include "utils.h"

/********************************** Type Defines  *****************************/
typedef void (*ScreenFunc)(void);

/************************************ VARIABLES ******************************/
static void display_page1(void);
static void display_page2(void);
static void display_page3(void);

static ScreenFunc info_screens[] = {display_page1, display_page2, display_page3};
static int screen_index          = 0;

static text_t text_msg[] = {
    {(char*)"INFORMATION", 11},
    {(char*)"Scroll through pages", 20},
    // pages 1
    {(char*)"Cortex M4 @ 150MHz", 18},
    {(char*)"1MB Flash, 760KB SRAM", 21},
    {(char*)"TFT, ISO7816 PHY, MSR", 21},
    // pages 2
    {(char*)"6 dynamic sensor pairs", 22},
    {(char*)"Hardware crypto blocks", 22},
    {(char*)"ECDSA Bootloader", 16},
    // pages 3
    {(char*)"EMV-L1 stack", 12},
    {(char*)"Cryptographic library", 21},
    {(char*)"FreeRTOS OS", 11},
};

static area_t area_cleanMSG = {0, 70, 320, 115};

/********************************* Static Functions **************************/
static void display_page1(void)
{
    TFT_ClearArea(&area_cleanMSG, 0);

    TFT_ShowImage(13, 74, check_bmp);
    TFT_ShowImage(13, 108, check_bmp);
    TFT_ShowImage(13, 143, check_bmp);
    TFT_PrintFont(47, 75, urw_gothic_16_white_bg_grey, &text_msg[2],
                  NULL); // "Cortex M3 @ 108MHz",18
    TFT_PrintFont(47, 109, urw_gothic_16_white_bg_grey, &text_msg[3],
                  NULL); // "1MB Flash, 256KB SRAM",20
    TFT_PrintFont(47, 145, urw_gothic_16_white_bg_grey, &text_msg[4],
                  NULL); // "TFT, ISO7817 PHY, MSR",19
}

static void display_page2(void)
{
    TFT_ClearArea(&area_cleanMSG, 0);

    TFT_ShowImage(13, 74, check_bmp);
    TFT_ShowImage(13, 108, check_bmp);
    TFT_ShowImage(13, 143, check_bmp);
    TFT_PrintFont(47, 75, urw_gothic_16_white_bg_grey, &text_msg[5],
                  NULL); // "6 dynamic sensor pairs",22
    TFT_PrintFont(47, 109, urw_gothic_16_white_bg_grey, &text_msg[6],
                  NULL); // "Hardware crypto blocks",22
    TFT_PrintFont(47, 145, urw_gothic_16_white_bg_grey, &text_msg[7], NULL); //"ECDSA Bootloader",16
}

static void display_page3(void)
{
    TFT_ClearArea(&area_cleanMSG, 0);

    TFT_ShowImage(13, 74, check_bmp);
    TFT_ShowImage(13, 108, check_bmp);
    TFT_ShowImage(13, 143, check_bmp);
    TFT_PrintFont(47, 75, urw_gothic_16_white_bg_grey, &text_msg[8], NULL); //"EMV-L1 stack",12
    TFT_PrintFont(47, 109, urw_gothic_16_white_bg_grey, &text_msg[9],
                  NULL); //"Cryptographic library",21
    TFT_PrintFont(47, 145, urw_gothic_16_white_bg_grey, &text_msg[10], NULL); // "FreeRTOS OS",11
}

static int init(void)
{
    TFT_SetBackGroundColor(0);

    TFT_PrintFont(96, 12, urw_gothic_16_bleu_bg_grey, &text_msg[0], NULL); // "INFORMATION"
    TFT_PrintFont(79, 40, urw_gothic_12_white_bg_grey, &text_msg[1],
                  NULL); // "Scroll through pages",20
    TFT_ShowImage(33, 12, integrated_only_small_bmp);
    TFT_ShowImage(12, 191, arrow_left_bmp);
    TFT_ShowImage(135, 191, home_bmp);
    TFT_ShowImage(259, 191, arrow_right_bmp);

    MXC_TS_RemoveAllButton();
    MXC_TS_AddButton(135, 191, (135 + 48), (191 + 39), KEY_C); //Home
    MXC_TS_AddButton(12, 191, (12 + 48), (191 + 39), KEY_A);
    MXC_TS_AddButton(259, 191, (259 + 48), (191 + 39), KEY_B);

    //
    screen_index = 0;
    display_page1();

    return 0;
}

static int key_process(unsigned int key)
{
    switch (key) {
        case KEY_A:
            if (--screen_index < 0) {
                screen_index = (ARRAY_SIZE(info_screens) - 1);
            }
            info_screens[screen_index]();
            break;
        case KEY_B:
            if (++screen_index >= (int)ARRAY_SIZE(info_screens)) {
                screen_index = 0;
            }
            info_screens[screen_index]();
            break;
        case KEY_C:
            state_set_current(get_home_state());
            break;
        default:
            break;
    }

    return 0;
}

static State g_state = {"info", init, key_process, NULL, 1000, NULL, NULL};

/********************************* Public Functions **************************/
State* get_info_state(void)
{
    return &g_state;
}
