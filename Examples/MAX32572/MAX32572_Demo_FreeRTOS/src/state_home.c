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
#include "task_logo_animation.h"

/********************************* 		DEFINES		 *************************/

/********************************* 		VARIABLES	 *************************/
static text_t text_msg[] = {
    { (char*)"MAX32572 - FreeRTOS Demo", 24 },
    { (char*)"Select an activity", 18 },
};

/********************************* Static Functions **************************/
static int init(void)
{
    TFT_SetPalette(integrated_only_small_bmp);
    TFT_SetBackGroundColor(0);

    TFT_ShowImage(33, 12, integrated_only_small_bmp);
    TFT_ShowImage(7, 72, smartcard_bmp);
    TFT_ShowImage(113, 72, magstripe_bmp);
    TFT_ShowImage(219, 72, keypad_bmp);
    TFT_ShowImage(7, 155, nfc_bmp);
    TFT_ShowImage(113, 155, slideshow_bmp);
    TFT_ShowImage(219, 155, information_bmp);

    TFT_PrintFont(50, 12, urw_gothic_12_white_bg_grey, &text_msg[0], NULL); // MAX32572
    TFT_PrintFont(89, 40, urw_gothic_12_white_bg_grey, &text_msg[1], NULL); // Select an activity

    MXC_TS_RemoveAllButton();
    MXC_TS_AddButton(7, 72, (7 + 93), (72 + 70), '1');
    MXC_TS_AddButton(113, 72, (113 + 93), (72 + 70), '2');
    MXC_TS_AddButton(219, 72, (219 + 93), (72 + 70), '3');
    MXC_TS_AddButton(6, 155, (6 + 94), (155 + 70), '4');
    MXC_TS_AddButton(113, 155, (113 + 93), (155 + 70), '5');
    MXC_TS_AddButton(219, 155, (219 + 93), (155 + 70), '6');

    logo_animation_start();

    return 0;
}

static int key_process(unsigned int key)
{
    switch (key) {
    case KEY_0:
        break;
    case KEY_1:
        state_set_current(get_smartcard_state());
        break;
    case KEY_2:
        state_set_current(get_msr_state());
        break;
    case KEY_3:
        state_set_current(get_keypad_state());
        break;
    case KEY_4:
        state_set_current(get_nfc_state());
        break;
    case KEY_5:
        state_set_current(get_slide_state());
        break;
    case KEY_6:
        state_set_current(get_info_state());
        break;
    case KEY_7:
        break;
    case KEY_8:
        break;
    case KEY_9:
        break;
    case KEY_A:
        break;
    case KEY_B:
        break;
    case KEY_C:
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

static State g_state = { "home", init, key_process, NULL, 0, NULL, NULL };

/********************************* Public Functions **************************/
State* get_home_state(void)
{
    return &g_state;
}
