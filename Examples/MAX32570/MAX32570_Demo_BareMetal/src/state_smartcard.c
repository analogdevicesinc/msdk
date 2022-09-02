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
#include <string.h>

#include "MAX32xxx.h"

#include "state.h"
#include "utils.h"
#include "task_smartcard.h"

/********************************* 		DEFINES		 *************************/
#define MAX_CHAR_ON_SCREEN 24

/********************************* 	 	TYPE DEF	 *************************/

/*******************************	Function Prototypes	   ********************/

/********************************* 		VARIABLES	 *************************/
static unsigned char input_buf[MAX_CHAR_ON_SCREEN + 1] = {0};

static text_t text_msg[] = {
    {(char*)input_buf, 0}, {(char*)"Insert a card", 13}, {(char*)"SMARTCARD", 9},
    {(char*)"Smart", 5},   {(char*)"card", 4},           {(char*)"ATR:", 4},
};

static text_t* text_line = &text_msg[0];

area_t sm_area_clean    = {0, 0, 0, 0};
area_t sm_area_cleanMSG = {0, 0, 0, 0};

//
static uint32_t g_card_last_status = ICC_ERR_REMOVED;

/********************************* Static Functions **************************/
static void read_atr(void)
{
    sc_read_atr(text_msg->data, &text_msg->len);
    if (text_msg->len > 0) {
        MXC_TFT_ClearArea(&sm_area_clean, 0);

        text_line->len = 18;
        MXC_TFT_PrintFont(24, 162, urw_gothic_16_bleu_bg_grey, &text_msg[5],
                          &sm_area_cleanMSG); //"ATR:", 4
        MXC_TFT_PrintFont(77, 162, urw_gothic_16_white_bg_grey, &text_msg[0], &sm_area_clean);
    }
}

/********************************* State Functions **************************/
static int init(void)
{
    MXC_TFT_SetBackGroundColor(0);
    MXC_TFT_PrintFont(105, 40, urw_gothic_12_white_bg_grey, &text_msg[1],
                      &sm_area_clean); //"Insert a card", 13

    // Test ATR
    MXC_TFT_PrintFont(94, 12, urw_gothic_16_bleu_bg_grey, &text_msg[2], NULL); //"SMARTCARD"
    MXC_TFT_ShowImage(33, 12, integrated_only_small_bmp);
    MXC_TFT_ShowImage(98, 69, smartcard_large_bmp);
    MXC_TFT_ShowImage(135, 191, home_bmp);

    // TS keys
    MXC_TS_RemoveAllButton();
    MXC_TS_AddButton(135, 191, (135 + 48), (191 + 39), 'C'); //Home

    g_card_last_status = ICC_ERR_REMOVED;

    return 0;
}

static int key_process(unsigned int key)
{
    switch (key) {
        case KEY_A:
            break;
        case KEY_B:
            break;
        case KEY_C: // exit
            state_set_current(get_home_state());
            break;
        case KEY_CARD_INSERTED:
            break;
        case KEY_CARD_REMOVED:
            break;
        default:
            break;
    }

    return 0;
}

static int time_tick(void)
{
    uint32_t status;

    status = sc_get_card_status();
    if (status != g_card_last_status) {
        g_card_last_status = status;

        if (!((IccReturn_t)g_card_last_status == ICC_ERR_REMOVED)) {
            read_atr();
        } else {
            //
            MXC_TFT_ClearArea(&sm_area_clean, 0);
            MXC_TFT_ClearArea(&sm_area_cleanMSG, 0);
            MXC_TFT_PrintFont(105, 40, urw_gothic_12_white_bg_grey, &text_msg[1],
                              &sm_area_clean); //"Insert a card", 13
        }
    }

    return 0;
}

static State g_state = {"smartcard", init, key_process, time_tick, 10};

/********************************* Public Functions **************************/
State* get_smartcard_state(void)
{
    return &g_state;
}
