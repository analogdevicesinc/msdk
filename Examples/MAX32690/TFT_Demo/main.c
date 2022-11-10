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

/**
 * @file    main.c
 * @brief   TFT Demo Example!
 *
 * @details
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "icc.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "led.h"
#include "pb.h"
#include "tft_st7735.h"

void TFT_Print(char *str, int len, int x, int y, int font)
{
    // fonts id
    text_t text;
    text.data = str;
    text.len = len;
    MXC_TFT_PrintFont(x, y, font, &text, NULL);
}

void TFT_test(void)
{
    char buff[] = "Analog";
    char buff1[] = "Devices";
    char buff2[] = "Inc.";

    while (1) {
    	// Display ADI Logo
    	MXC_TFT_ShowImage(2, 3, (int)&logo_rgb565[0]);
		MXC_Delay(5000000);

		// Demonstrate Geometric Functions
		MXC_TFT_SetBackGroundColor(ROYAL_BLUE);	// Clear screen

		MXC_TFT_Line(10, 10, 100, 100, WHITE);
		MXC_Delay(1000000);

		MXC_TFT_Rectangle(10, 10, 100, 100, WHITE);
		MXC_Delay(1000000);

		MXC_TFT_Circle(100, 100, 30, WHITE);
		MXC_Delay(1000000);

		MXC_TFT_FillCircle(36, 74, 26, WHITE);
		MXC_TFT_FillCircle(74, 36, 26, WHITE);
		MXC_Delay(1000000);

		// Demonstrate Text Functions
		MXC_TFT_SetBackGroundColor(ROYAL_BLUE); // Clear screen
		MXC_TFT_SetForeGroundColor(WHITE); // set chars to white
		MXC_TFT_ClearScreen();

		TFT_Print(buff, sizeof(buff), 3, 10, (int)&Arial12x12[0]);	// Use Arial 12x12 font
		TFT_Print(buff1, sizeof(buff1), 3, 22, (int)&Arial12x12[0]);
		TFT_Print(buff2, sizeof(buff2), 3, 34, (int)&Arial12x12[0]);
		MXC_Delay(1000000);
		MXC_TFT_ClearScreen();

		TFT_Print(buff, sizeof(buff), 3, 30, (int)&Arial24x23[0]);	// Use Arial 24x23 font
		TFT_Print(buff1, sizeof(buff1), 3, 54, (int)&Arial24x23[0]);
		TFT_Print(buff2, sizeof(buff2), 3, 78, (int)&Arial24x23[0]);
		MXC_Delay(1000000);
		MXC_TFT_ClearScreen();

		TFT_Print(buff, sizeof(buff), 3, 40, (int)&Arial28x28[0]);	// Use Arial 28x28 font
		TFT_Print(buff1, sizeof(buff1), 3, 68, (int)&Arial28x28[0]);
		TFT_Print(buff2, sizeof(buff2), 3, 96, (int)&Arial28x28[0]);
		MXC_Delay(1000000);
		MXC_TFT_ClearScreen();

		TFT_Print(buff, sizeof(buff), 3, 80, (int)&SansSerif16x16[0]);	// Use Sans Serif 16x16 font
		TFT_Print(buff1, sizeof(buff1), 3, 96, (int)&SansSerif16x16[0]);
		TFT_Print(buff2, sizeof(buff2), 3, 112, (int)&SansSerif16x16[0]);
		MXC_Delay(1000000);
    }
}

int main(void)
{
    /* Enable cache */
    MXC_ICC_Enable(MXC_ICC0);

    /* Set system clock to 100 MHz */
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();

    /* Initialize TFT display */
	MXC_TFT_Init();
    TFT_test();

    return 0;
}
