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
#include "utils.h"
#include "task_smartcard.h"

#include "sc_errors.h"
#include "sc_states.h"
#include "sc_types.h"
#include "smartcard_api.h"

/********************************* 		DEFINES		 *************************/
#define MAX_CHAR_ON_SCREEN		24

/********************************* 	 	TYPE DEF	 *************************/
typedef void (*SmartCard_Test)(void);


/*******************************	Function Prototypes	   ********************/
static void read_atr(void);

/********************************* 		VARIABLES	 *************************/
static unsigned char input_buf[MAX_CHAR_ON_SCREEN + 1] = {0};

static text_t text_msg[] = {
	{ (char *)input_buf,  			0},
	{ (char *)"Insert a card", 		13},
	{ (char *)"SMARTCARD" , 		9},
	{ (char *)"Smart",				5},
	{ (char *)"card",				4},
	{ (char *)"ATR:", 				4},
};

static text_t *text_line = &text_msg[0];

area_t sm_area_clean    = {0, 0, 0, 0};
area_t sm_area_cleanMSG = {0, 0, 0, 0};

extern ActivationParams_t ActivationParams;

static SmartCard_Test test_array[] = {	read_atr,
										//...
									  };
static unsigned int test_max   = ARRAY_SIZE(test_array);
static unsigned int test_index = 0;

/********************************* Static Functions **************************/
static int init(void)
{
	TFT_SetBackGroundColor(0);
	TFT_PrintFont(105, 40, urw_gothic_12_white_bg_grey, &text_msg[1], &sm_area_clean);  //"Insert a card", 13

	if (test_index == 0) {
		// Test ATR
	    TFT_PrintFont(94, 12, urw_gothic_16_bleu_bg_grey,  &text_msg[2], NULL); //"SMARTCARD"
	    TFT_ShowImage(33, 12, integrated_only_small_bmp);
	    TFT_ShowImage(98, 69, smartcard_large_bmp);
	    TFT_ShowImage(135, 191, home_bmp);

        // TS keys
	    MXC_TS_RemoveAllButton();
		MXC_TS_AddButton(135, 191, 	(135 + 48), (191 + 39), 		'C'); //Home
	}

	sc_set_afe_intterrupt(1);

    return 0;
}

static void read_atr(void)
{
	unsigned char atr[128];
	unsigned int  i;
	IccReturn_t   retval;
	uint32_t	  status;

	retval = SCAPI_ioctl(SC_SLOT_NUMBER, IOCTL_GET_CARD_STATE, &status);

	if (!((IccReturn_t) status == ICC_ERR_REMOVED)) {
		/*power up the card */
		status = POWER_UP;
		ActivationParams.IccWarmReset = bFALSE;
		retval = SCAPI_ioctl(SC_SLOT_NUMBER, IOCTL_POWER_CARD, &status);
		if (ICC_OK != retval) {
			goto read_atr_out;
		}

		/*
		 * Read the ATR and save into the atr buffer
		 * as output, status will contains the exact ATR length
		 */
		status = sizeof(atr);
		retval = SCAPI_read(SC_SLOT_NUMBER, atr, &status);
		if (retval) {
			memcpy(text_line->data, "Smartcard Error", 15);
			text_line->len = 15;
		} else {
			memset(text_line->data, ' ', MAX_CHAR_ON_SCREEN);
			switch (status) {
				case 1:
					text_line->len = 2;
					utils_hex2char(atr[0], &text_line->data[0]);
					break;
				case 2:
					text_line->len = 5;
					utils_hex2char(atr[0], &text_line->data[0]);
					utils_hex2char(atr[1], &text_line->data[3]);
					break;
				default:
					utils_hex2char(atr[0], &text_line->data[0]);
					utils_hex2char(atr[1], &text_line->data[3]);
					if (status < 6) {
						for (i = 2; i < status; i++) {
							utils_hex2char(atr[i], &text_line->data[3 * i]);
						}
						text_line->len = 3 * status - 1;
					} else {
						utils_hex2char(atr[2], &text_line->data[6]);
						text_line->data[9] = '.';
						text_line->data[10] = '.';
						text_line->data[11] = '.';
						utils_hex2char(atr[status - 2], &text_line->data[13]);
						utils_hex2char(atr[status - 1], &text_line->data[16]);
						text_line->len = 18;
					}
					break;
			}

			// Smartcard ATR Display
			TFT_ClearArea(&sm_area_clean,0);

			text_line->len = 18;
			TFT_PrintFont(24, 162, urw_gothic_16_bleu_bg_grey,  &text_msg[5], &sm_area_cleanMSG); //"ATR:", 4
			TFT_PrintFont(77, 162, urw_gothic_16_white_bg_grey, &text_msg[0], &sm_area_clean);

			/* Enable AFE interrupt */
			sc_set_afe_intterrupt(1);
		}
	}
read_atr_out:
	/*power off the card */
	status = POWER_DOWN;
	SCAPI_ioctl(SC_SLOT_NUMBER, IOCTL_POWER_CARD, &status);
}

static int key_process(unsigned int key)
{
	switch(key) {
		case KEY_A:
			if (test_index > 0) {
				--test_index;
				init();
			}
			break;
		case KEY_B:
			if ( test_index < (test_max-1) ) {
				test_index++;
				init();
			}
			break;
		case KEY_C: // exit
			state_set_current( get_home_state() );
			sc_set_afe_intterrupt(0);
			break;
		case KEY_CARD_INSERTED:
			test_array[test_index]();
			break;
		case KEY_CARD_REMOVED:
			//
			MXC_TFT_ClearArea(&sm_area_clean,0);
			MXC_TFT_ClearArea(&sm_area_cleanMSG,0);
			MXC_TFT_PrintFont(105, 40, urw_gothic_12_white_bg_grey, &text_msg[1], &sm_area_clean);  //"Insert a card", 13
			break;
		default:
			break;
	}

	return 0;
}

static State g_state = {"smartcard", init, key_process, NULL, 0, NULL, NULL};

/********************************* Public Functions **************************/
State *get_smartcard_state(void)
{
	return &g_state;
}
