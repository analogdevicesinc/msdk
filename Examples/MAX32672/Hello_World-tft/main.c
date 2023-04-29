/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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
 * @brief   Hello World!
 * @details This example uses the UART to print to a terminal and flashes an LED.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "mxc_sys.h"
#include "pb.h"
#include "led.h"
#include "board.h"
#include "mxc_delay.h"
#include "bitmap.h"
#include "tft_st7735s.h"

#ifdef BOARD_FTHR_REVA
#error "ERR_NOTSUPPORTED: This example is not supported by the MAX32672 FTHR_RevA."
#endif

/***** Definitions *****/

/***** Globals *****/

/***** Functions *****/

volatile int buttonPressedCount = 0;
void buttonHandler(void *pb)
{
    buttonPressedCount++;
}

// *****************************************************************************
int main(void)
{
    int i;
    int count = 0;
    int pb_state = 0;
    uint8_t usn[MXC_SYS_USN_LEN];
    area_t printf_area;

    printf("Hello World!\n");

    MXC_SYS_GetUSN(usn, NULL);

    PB_RegisterCallback(0, (pb_callback)buttonHandler);

    MXC_TFT_Init();
    MXC_TFT_ShowImage(2, 3, (int)&logo_rgb565[0]);

    MXC_Delay(MXC_DELAY_SEC(2));

    MXC_TFT_SetBackGroundColor(WHITE);

    MXC_TFT_SetFont((int)&SansSerif16x16[0]);

    // Set print area
    printf_area.x = 2;
    printf_area.y = 3;
    printf_area.w = 128;
    printf_area.h = 128;
    MXC_TFT_ConfigPrintf(&printf_area);

    while (1) {
        MXC_TFT_Printf("cnt: %d\n", count);

        if (buttonPressedCount > 4) {
            buttonPressedCount = 0;

            // Printf USN on bottom of screen.
            MXC_TFT_SetBackGroundColor(WHITE);
            MXC_TFT_ConfigPrintf(&printf_area);

            // Print revision and USN.
            MXC_TFT_Printf("Rev: %x\n", MXC_GCR->revision);
            MXC_TFT_Printf("USN:\n  %02x%02x%02x%02x%02x\n   -%02x%02x%02x%02x\n   -%02x%02x%02x%02x\n", usn[0], usn[1], usn[2], usn[3], usn[4], usn[5], usn[6], usn[7], usn[8], usn[9], usn[10], usn[11], usn[12]);

            printf("\nRev: %x\n", MXC_GCR->revision);
            printf("USN: ");
            for (i = 0; i < MXC_SYS_USN_LEN; i++) {
                printf("%02x", usn[i]);
            }
            printf("\n\n");

            MXC_TFT_Printf("\nContinue in\n");
            for (i = 3; i > 0; i--) {
                MXC_TFT_Printf("%ds...", i);
                MXC_Delay(1000000);
            }

            // Clear screen and continue count.
            MXC_TFT_ConfigPrintf(&printf_area);
            MXC_TFT_SetBackGroundColor(WHITE);
        }
        
        pb_state = PB_Get(0);

        LED_On(0);

        // Invert LED1 if PB0 is pressed.
        if (pb_state) {
            LED_Off(1);
        } else {
            LED_On(1);
        }

        MXC_Delay(500000);

        LED_Off(0);

        // Invert LED1 if PB0 is pressed.
        if (pb_state) {
            LED_On(1);
        } else {
            LED_Off(1);
        }

        MXC_Delay(500000);

        // Print count every second.
        printf("count = %d\n", count++);
    }
}
