/*
 * @file main.c
 *
 */

/*******************************************************************************
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
*******************************************************************************
*/

/** @file main.c MAIN NFC PCD DTE EMV Loop back
 * This example Implements The DTE for EMV NFC including loop back for digital
 * and analog testing.
 * NOTE: Set of maximum frequency is already did in function SystemCoreClockUpdate() in system/src/cortexm/_initialize_hardware.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mxc_device.h"
#include "mxc_delay.h"
#include <emvl1_app.h>
#include <board.h>

#include "tft.h"
#include "bitmap.h"
#include <mml_nfc_pcd_rf_driver.h>


static int setup_display(void)
{
	// init TFT
 	MXC_TFT_Init();

	// show image
    MXC_TFT_ShowImage(11, 7, logo_white_bg_white_bmp);

    // set up font
    MXC_TFT_SetFont(urw_gothic_13_grey_bg_white);

    // Set print area
    area_t print = {20, 50, 300, 80};
    MXC_TFT_ConfigPrintf(&print);

    return 0;
}


int main(void)
{
    int32_t result = MML_NFC_PCD_E_SUCCESS;

    // Disable line buffering for this serial menu driven DTE
    //  avoid may calls to fflush();
    setvbuf(stdout, NULL, _IONBF, 0);

    // initialize display, setup font
    setup_display();

    MXC_TFT_Printf("DTE EMV LOOPBACK\n\n");
    MXC_TFT_Printf("Connect UART0 115200,8,N,1");

    // Welcome message
    printf("Maxim MAX32570 DTE\n\n");
    fflush(0);

	result = mml_nfc_pcd_init();

	if (result != MML_NFC_PCD_E_SUCCESS) {
	    printf("Failed to initialize NFC block.  Error: %d\n", result);
	}
    else {
        emvl1_main_loop();
    }

    return 1;
}
