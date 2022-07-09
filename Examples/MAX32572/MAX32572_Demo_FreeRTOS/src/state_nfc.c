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
#include "task_nfc.h"


/********************************* 		DEFINES		 *************************/
#define TICK_TIMEOUT			2000
//
#define MAX_CHAR_ON_SCREEN		24

/********************************* 	 	TYPE DEF	 *************************/

/*******************************	Function Prototypes	   ********************/

/********************************* 		VARIABLES	 *************************/
static text_t text_msg[] = {
	{ (char *)"NFC", 					 		  3 },
#ifndef MN_EvKit_V1
	{ (char *)"Place card near target", 		 22 },
#else
	{ (char *)"This EvKit does not support NFC", 31 }
#endif
};

#ifndef MN_EvKit_V1
	static area_t area_clean    = {0, 0, 0, 0};
	static area_t area_clean_1  = {0, 0, 0, 0};
	static int after_timeout_clear_screen = 0;
#endif

/********************************* Static Functions **************************/
static int init(void)
{
    TFT_SetBackGroundColor(0);

    TFT_PrintFont(140,   12,  urw_gothic_16_bleu_bg_grey, &text_msg[0], NULL); //"NFC"
#ifndef MN_EvKit_V1
    MXC_TFT_PrintFont(58,    40,  urw_gothic_12_white_bg_grey,&text_msg[1], NULL); //"Place card near target"
#else
    MXC_TFT_PrintFont(23,    40,  urw_gothic_12_white_bg_grey,&text_msg[1], NULL); //
#endif
    //
    TFT_ShowImage(33,  12,  integrated_only_small_bmp);
    TFT_ShowImage(106, 72,  nfc_large_bmp);
    TFT_ShowImage(135, 191, home_bmp);

	MXC_TS_RemoveAllButton();
	MXC_TS_AddButton(135, 191, 	(135 + 48), (191 + 39), 		'C'); //Home

#ifndef MN_EvKit_V1
	nfc_enable_polling();
#endif

    return 0;
}

#ifndef MN_EvKit_V1
static int nfc_process(unsigned char *msg, unsigned int msg_len)
{
	if (msg_len > 0) {
		text_t 		 msg_var;
		text_t 		 msg_var_1;
		unsigned int k;

		// NFC info Display
		TFT_ClearArea(&area_clean,	0);
        TFT_ClearArea(&area_clean_1,0);

		// Parse message looking for "\n" signaling a new line
		for (k = 0; k < msg_len; k++) {
			if (msg[k] == '\n') {
				break;
			}
		}

		if (k == msg_len) {
			// Single line to show
			msg_var.data = (char *)msg;
			msg_var.len  = msg_len;

			if (msg_var.len > 24) {
				msg_var.len = 24;
			}
			TFT_PrintFont(17, 150, urw_gothic_16_white_bg_grey, &msg_var, &area_clean);

		} else {
			// Show 2 lines, MAX
			msg_var.data = (char *)msg;
			msg_var.len  = k;

			k++; // skip past \n
			msg_var_1.data = (char *)(msg + k);
			msg_var_1.len  = msg_len - k;

			if (msg_var.len > 24) {
				msg_var.len = 24;
			}

			if (msg_var_1.len > 24) {
				msg_var_1.len = 24;
			}
			TFT_PrintFont(17, 150, urw_gothic_16_white_bg_grey, &msg_var, &area_clean);
			TFT_PrintFont(17, 168, urw_gothic_16_white_bg_grey, &msg_var_1, &area_clean_1);
		}

		after_timeout_clear_screen = 1;

		nfc_enable_polling();
	}

	return 0;
}

static int time_tick(void)
{
	if (after_timeout_clear_screen) {
		// clear
		TFT_ClearArea(&area_clean,0);
		TFT_ClearArea(&area_clean_1,0);

		after_timeout_clear_screen = 0;
	}
	return 0;
}

#endif //  for #ifndef MN_EvKit_V1

static int key_process(unsigned int key)
{
	switch(key) {
		case KEY_C: // exit
			#ifndef MN_EvKit_V1
			nfc_disable_polling();
			#endif
			state_set_current( get_home_state() );
			break;
		default:
			break;
	}

	return 0;
}


#ifndef MN_EvKit_V1
	static State g_state = {"nfc", init, key_process, time_tick, TICK_TIMEOUT, NULL, nfc_process };
#else
	static State g_state = {"nfc", init, key_process, NULL, 	 0, 		   NULL,   NULL	 	 };
#endif

/********************************* Public Functions **************************/
State *get_nfc_state(void)
{
	return &g_state;
}
