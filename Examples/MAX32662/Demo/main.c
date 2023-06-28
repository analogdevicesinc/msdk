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
 * @brief   Demo
 * @details This example displays the uptime to the TFT Display and Terminal,
 *          toggles the LED at 1-2Hz depending on pushbutton press, and shows
 *          the chip information if the pushbutton is pressed 5 times.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_sys.h"
#include "pb.h"
#include "led.h"
#include "board.h"
#include "rtc.h"
#include "bitmap.h"
#include "tft_st7735.h"

/***** Definitions *****/

#define SECS_PER_MIN 60
#define SECS_PER_HR (60 * SECS_PER_MIN)

/***** Globals *****/

int buttonPressedCount = 0;

/***** Functions *****/

volatile int buttonPressed = 0;
void buttonHandler(void *pb)
{
    buttonPressed = 1;
}

// This function runs at every 250ms to account for debouncing.
void checkForButtonRelease(void)
{
    if (buttonPressed && !PB_Get(0)) {
        buttonPressedCount++;
        buttonPressed = 0;
    }
}

// *****************************************************************************
int main(void)
{
    int i;
    int pb_state = 0;
    uint8_t usn[MXC_SYS_USN_LEN];
    area_t usn_printf_area;
    area_t units_printf_area;
    area_t uptime_printf_area;
    int hr, min;
    uint32_t sec;
    int error;

    MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_IPO);
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);

    printf("**** MAX32662 EV Kit Demo ****\n");

    // Predefine Printf Area for displaying USN on TFT.
    usn_printf_area.x = 2;
    usn_printf_area.y = 3;
    usn_printf_area.w = 128;
    usn_printf_area.h = 128;

    MXC_SYS_GetUSN(usn, NULL);

    PB_RegisterCallback(0, (pb_callback)buttonHandler);

    MXC_TFT_Init();
    MXC_TFT_ShowImage(2, 3, (int)&logo_rgb565[0]);

    MXC_Delay(MXC_DELAY_SEC(2));

    MXC_TFT_SetBackGroundColor(WHITE);

    MXC_TFT_SetFont((int)&SansSerif16x16[0]);

    error = MXC_RTC_Init(0, 0);
    if (error != E_NO_ERROR) {
        printf("Failed RTC Initialization\n");
        return error;
    }

    error = MXC_RTC_Start();
    if (error != E_NO_ERROR) {
        printf("Failed RTC_Start\n");
        return error;
    }

    // Set print area
    units_printf_area.x = 24;
    units_printf_area.y = 25;
    units_printf_area.w = 128;
    units_printf_area.h = 30;
    MXC_TFT_ConfigPrintf(&units_printf_area);
    MXC_TFT_Printf("hh:mm:ss");

    uptime_printf_area.x = 22;
    uptime_printf_area.y = 58;
    uptime_printf_area.w = 128;
    uptime_printf_area.h = 30;
    MXC_TFT_ConfigPrintf(&uptime_printf_area);

    while (1) {
        MXC_RTC_GetSeconds(&sec);

        hr = sec / SECS_PER_HR;
        sec -= hr * SECS_PER_HR;

        min = sec / SECS_PER_MIN;
        sec -= min * SECS_PER_MIN;

        printf("\n(hhh:mm:ss): %03d:%02d:%02d\n\n", hr, min, sec);
        MXC_TFT_Printf("%03d:%02d:%02d\n", hr, min, sec);

        if (buttonPressedCount > 4) {
            buttonPressedCount = 0;

            // Printf USN on bottom of screen.
            MXC_TFT_SetBackGroundColor(WHITE);

            MXC_TFT_ConfigPrintf(&usn_printf_area);

            // Print revision and USN.
            MXC_TFT_Printf("Rev: %x\n", MXC_GCR->revision);
            MXC_TFT_Printf(
                "USN:\n  %02x%02x%02x%02x%02x\n   -%02x%02x%02x%02x\n   -%02x%02x%02x%02x\n",
                usn[0], usn[1], usn[2], usn[3], usn[4], usn[5], usn[6], usn[7], usn[8], usn[9],
                usn[10], usn[11], usn[12]);

            printf("\nRev: %x\n", MXC_GCR->revision);
            printf("USN: ");
            for (i = 0; i < MXC_SYS_USN_LEN; i++) {
                printf("%02x", usn[i]);
            }
            printf("\n\n");

            MXC_TFT_Printf("\nContinue in\n");
            for (i = 5; i > 0; i--) {
                MXC_TFT_Printf("%d..", i);
                MXC_Delay(MXC_DELAY_SEC(1));
            }

            // Clear screen and continue uptime.
            MXC_TFT_SetBackGroundColor(WHITE);

            MXC_TFT_ConfigPrintf(&units_printf_area);
            MXC_TFT_Printf("hh:mm:ss");

            MXC_TFT_ConfigPrintf(&uptime_printf_area);
            MXC_TFT_Printf("%03d:%02d:%02d\n", hr, min, sec);
        }

        pb_state = PB_Get(0);

        LED_Toggle(0);
        checkForButtonRelease();

        MXC_Delay(250000);

        // Toggle LED0 at 250ms (instead of 500ms) when PB0 is pressed.
        if (pb_state) {
            LED_Toggle(0);
        }

        checkForButtonRelease();

        MXC_Delay(250000);

        LED_Toggle(0);
        checkForButtonRelease();

        MXC_Delay(250000);

        // Toggle LED0 at 250ms (instead of 500ms) when PB0 is pressed.
        if (pb_state) {
            LED_Toggle(0);
        }

        checkForButtonRelease();

        // Set as 50000 (50ms) instead of 250000 (250ms) because logic at beginning of while
        //  loop takes around ~200ms.
        MXC_Delay(50000);
    }
}
