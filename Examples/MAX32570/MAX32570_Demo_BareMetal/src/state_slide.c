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

#include "state.h"
#include "task_logo_animation.h"

/********************************* 		DEFINES		 *************************/
#define TICK_TIMEOUT 2000

/********************************* 		VARIABLES	 *************************/
static int img_number = 0;

static text_t text_msg[] = { { (char *)"SLIDESHOW", 9 }, { (char *)"Press Play to start", 19 } };

/********************************* Static Functions **************************/
static int init(void)
{
    MXC_TFT_SetBackGroundColor(0);
    MXC_TFT_PrintFont(104, 12, urw_gothic_16_bleu_bg_grey, &text_msg[0], NULL); // "SLIDESHOW"
    MXC_TFT_PrintFont(89, 40, urw_gothic_12_white_bg_grey, &text_msg[1],
                      NULL); // "Press Play to start"
    MXC_TFT_ShowImage(33, 12, integrated_only_small_bmp);
    MXC_TFT_ShowImage(119, 82, slideshow_large_bmp);
    MXC_TFT_ShowImage(135, 191, home_bmp);

    img_number = 0;

    MXC_TS_RemoveAllButton();
    MXC_TS_AddButton(119, 82, (119 + 78), (82 + 78), '1');
    MXC_TS_AddButton(135, 191, (135 + 48), (191 + 39), KEY_C); //Home

    return 0;
}

static int time_tick(void)
{
    if (img_number <= 0) {
        return 1;
    }

    switch (img_number) {
    case 1:
        MXC_TFT_ShowImage(0, 0, mpos_bmp);
        break;
    case 2:
        MXC_TFT_ShowImage(0, 0, parrot_bmp);
        break;
    case 3:
        MXC_TFT_SetBackGroundColor(0);
        MXC_TFT_ShowImage(52, 87, maxim_integrated_large_bmp);
        break;
    default:
        break;
    }

    if (++img_number > 4) {
        img_number = 1; // go first slide
    }

    return 0;
}

static int key_process(unsigned int key)
{
    switch (key) {
    case KEY_1:
        logo_animation_stop();

        MXC_TS_RemoveAllButton();
        MXC_TS_AddButton(0, 0, 320, 240, KEY_C); //Home

        // start slide
        img_number = 1;
        time_tick();
        break;
    case KEY_C:
        state_set_current(get_home_state());
        break;
    default:
        break;
    }
    return 0;
}

static State g_state = { "slide", init, key_process, time_tick, TICK_TIMEOUT };

/********************************* Public Functions **************************/
State *get_slide_state(void)
{
    return &g_state;
}
