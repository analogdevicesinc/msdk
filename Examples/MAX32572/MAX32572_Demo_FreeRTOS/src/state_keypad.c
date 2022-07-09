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
#define MAX_CHAR_ON_SCREEN	8

#define BUTTON_SIZE_X 	42 + 4  // 6 for free space
#define BUTTON_SIZE_Y 	41 + 6  // 6 for free space


/********************************* 		VARIABLES	 *************************/
static unsigned char input_buf[MAX_CHAR_ON_SCREEN + 1] = {0};

static text_t text_msg[] = {
	{ (char *)"Pressed", 		7 	},
	{ (char *)input_buf,  		0	},

};

static text_t *input_text = &text_msg[1];

/********************************* Static Functions **************************/
static int init(void)
{
	int x, y;
	int x0 = 120;
	int y0 = 40;

	logo_animation_stop();

	MXC_TS_RemoveAllButton();
	TFT_SetPalette(logo_white_bg_white_bmp);
    TFT_SetBackGroundColor(0);

    TFT_ShowImage(11, 7, logo_white_bg_white_bmp);

    x = x0;
    y = y0;
    TFT_ShowImage(x, y, key_1_bg_white_bmp);
    MXC_TS_AddButton(x, y,  x+42,  y+41, 	KEY_1);

    x += BUTTON_SIZE_X ;
    TFT_ShowImage(x, y, key_2_bg_white_bmp);
    MXC_TS_AddButton(x, y,  x+42,  y+41, 	KEY_2);

    x += BUTTON_SIZE_X ;
    TFT_ShowImage(x, y, key_3_bg_white_bmp);
    MXC_TS_AddButton(x, y,  x+42,  y+41, 	KEY_3);

    x += BUTTON_SIZE_X ;
    TFT_ShowImage(x, y, key_clear_bg_white_bmp);
    MXC_TS_AddButton(x, y,  x+42,  y+41, 	KEY_F);

    //
    x = x0;
    y += BUTTON_SIZE_Y ;
    TFT_ShowImage(x, y, key_4_bg_white_bmp);
    MXC_TS_AddButton(x, y,  x+42,  y+41, 	KEY_4);

    x += BUTTON_SIZE_X ;
    TFT_ShowImage(x, y, key_5_bg_white_bmp);
    MXC_TS_AddButton(x, y,  x+42,  y+41, 	KEY_5);

    x += BUTTON_SIZE_X ;
    TFT_ShowImage(x, y, key_6_bg_white_bmp);
    MXC_TS_AddButton(x, y,  x+42,  y+41, 	KEY_6);

    x += BUTTON_SIZE_X ;
    TFT_ShowImage(x, y, key_cancel_bg_white_bmp);
    MXC_TS_AddButton(x, y,  x+42,  y+41, 	KEY_E);

    //
    x = x0;
    y += BUTTON_SIZE_Y ;
    TFT_ShowImage(x, y, key_7_bg_white_bmp);
    MXC_TS_AddButton(x, y,  x+42,  y+41, 	KEY_7);

    x += BUTTON_SIZE_X ;
    TFT_ShowImage(x, y, key_8_bg_white_bmp);
    MXC_TS_AddButton(x, y,  x+42,  y+41, 	KEY_8);

    x += BUTTON_SIZE_X ;
    TFT_ShowImage(x, y, key_9_bg_white_bmp);
    MXC_TS_AddButton(x, y,  x+42,  y+41, 	KEY_9);

    x += BUTTON_SIZE_X ;
    TFT_ShowImage(x, y, key_enter_bg_white_bmp);
    MXC_TS_AddButton(x, y,  x+42,  y+41, 	KEY_D);

    //
    x = x0;
    x += BUTTON_SIZE_X ;
    y += BUTTON_SIZE_Y ;
    TFT_ShowImage(x, y, key_0_bg_white_bmp);
    MXC_TS_AddButton(x, y,  x+42,  y+41, 	KEY_0);

    TFT_PrintFont(11,	80, urw_gothic_13_grey_bg_white,  &text_msg[0],  NULL);
    input_text->len = 0;

    return 0;
}

static void print_inputs(int key)
{
	static area_t area;

	if (key == 0) { // means clear screen
		TFT_ClearArea(&area, 0);
		input_text->len = 0;
	} else if (key == -1) {		// means clear last key
		if (input_text->len > 0) {
			input_text->len--;
			TFT_ClearArea(&area, 0);
			TFT_PrintFont(11, 120, urw_gothic_13_grey_bg_white, input_text,  &area);
		}
	} else if (input_text->len < MAX_CHAR_ON_SCREEN) {
		input_text->data[input_text->len++] = key;
		TFT_PrintFont(11, 120, urw_gothic_13_grey_bg_white, input_text,  &area);
	}
}

static int key_process(unsigned int key)
{
	switch(key) {
		case KEY_0:
		case KEY_1:
		case KEY_2:
		case KEY_3:
		case KEY_4:
		case KEY_5:
		case KEY_6:
		case KEY_7:
		case KEY_8:
		case KEY_9:
		case KEY_A:
		case KEY_B:
			print_inputs(key);
			break;
		case KEY_C: // exit
			state_set_current( get_home_state() );
			break;
		case KEY_D: // enter
			state_set_current( get_home_state() );
			break;
		case KEY_E: // cancel
			print_inputs(-1);
			break;
		case KEY_F: // clear
			print_inputs(0);
			break;
		default:
			break;
	}

	return 0;
}

static State g_state = {"keypad", init, key_process, NULL, 0, NULL, NULL};

/********************************* Public Functions **************************/
State *get_keypad_state(void)
{
	return &g_state;
}

