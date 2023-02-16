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
 * @file        main.c
 * @brief       Configures and starts the RTC and demonstrates the use of the
 * alarms.
 * @details     The RTC is enabled and the sub-second alarm set to trigger every
 * 250 ms. P2.25 (LED0) is toggled each time the sub-second alarm triggers.  The
 *              time-of-day alarm is set to 2 seconds.  When the time-of-day
 * alarm triggers, the rate of the sub-second alarm is switched to 500 ms.  The
 *              time-of-day alarm is then rearmed for another 2 sec.  Pressing
 * SW2 will output the current value of the RTC to the console UART.
 */

/***** Includes *****/
#include <stdint.h>
#include <stdio.h>

#include "board.h"
#include "led.h"
#include "lp.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "nvic_table.h"
#include "rtc.h"
#include "uart.h"

/***** Definitions *****/
#define LED_TODA 0
#define TIME_OF_DAY_SEC 7

#define MSEC_TO_RSSA(x) \
    (0 - ((x * 4096) /  \
          1000)) // Converts a time in milleseconds to the equivalent RSSA register value.
#define SECS_PER_MIN 60
#define SECS_PER_HR (60 * SECS_PER_MIN)
#define SECS_PER_DAY (24 * SECS_PER_HR)

/***** Globals *****/

/***** Functions *****/
// *****************************************************************************
void rescheduleAlarm()
{
    int time;
    int flags = MXC_RTC_GetFlags();

    if (flags & MXC_F_RTC_CTRL_ALDF) { // Check for TOD alarm flag
        MXC_RTC_ClearFlags(MXC_F_RTC_CTRL_ALDF);

        time = MXC_RTC_GetSecond(); // Get Current time (s)

        while (MXC_RTC_DisableInt(MXC_F_RTC_CTRL_ADE) == E_BUSY) {}
        // Disable interrupt while re-arming RTC alarm

        if (MXC_RTC_SetTimeofdayAlarm(time + TIME_OF_DAY_SEC) !=
            E_NO_ERROR) { // Reset TOD alarm for TIME_OF_DAY_SEC in the future
            /* Handle Error */
        }

        while (MXC_RTC_EnableInt(MXC_F_RTC_CTRL_ADE) == E_BUSY) {}
        // Re-enable TOD alarm interrupt
    }

    MXC_LP_EnableRTCAlarmWakeup(); // Enable RTC as a wakeup source from low power modes
}

// *****************************************************************************
void printTime()
{
    int day, hr, min, sec;

    sec = MXC_RTC_GetSecond(); // Get current time

    day = sec / SECS_PER_DAY;
    sec -= day * SECS_PER_DAY;

    hr = sec / SECS_PER_HR;
    sec -= hr * SECS_PER_HR;

    min = sec / SECS_PER_MIN;
    sec -= min * SECS_PER_MIN;

    printf("\nCurrent Time (dd:hh:mm:ss): %02d:%02d:%02d:%02d\n\n", day, hr, min,
           sec); // Print current time
}

// *****************************************************************************
int configureRTC()
{
    printf("\n\n***************** RTC Wake from Backup Example *****************\n\n");
    printf("The time-of-day alarm is set to wake the device every %d seconds.\n", TIME_OF_DAY_SEC);
    printf("When the alarm goes off it will print the current time to the console.\n\n");

    if (MXC_RTC_Init(0, 0) != E_NO_ERROR) { // Initialize RTC
        printf("Failed RTC Initialization\n");
        printf("Example Failed\n");
        while (1) {}
    }

    if (MXC_RTC_SetTimeofdayAlarm(TIME_OF_DAY_SEC) != E_NO_ERROR) { // Arm TOD alarm
        printf("Failed RTC_SetTimeofdayAlarm\n");
        printf("Example Failed\n");
        while (1) {}
    }

    MXC_RTC_ClearFlags(MXC_F_RTC_CTRL_ALDF);
    if (MXC_RTC_EnableInt(MXC_F_RTC_CTRL_ADE) == E_BUSY) { // Enable TOD interrupt
        return E_BUSY;
    }

    if (MXC_RTC_Start() != E_NO_ERROR) { // Start RTC
        printf("Failed RTC_Start\n");
        printf("Example Failed\n");
        while (1) {}
    }

    printf("RTC started\n");

    return E_NO_ERROR;
}

// *****************************************************************************
void RTC_IRQHandler(void) {}

// *****************************************************************************
int main(void)
{
    if (!(MXC_RTC_GetFlags() & MXC_F_RTC_CTRL_ALDF)) { // Check whether the wakeup source is RTC
        if (configureRTC() != E_NO_ERROR) { // System start/restart
            printf("Example Failed\n");
            while (1) {}
        }
    } else {
        LED_On(LED_TODA); // RTC alarm fired off. Perform periodic task here
        printTime();
    }

    rescheduleAlarm(); // Re-arm RTC TOD alarm
    MXC_Delay(MXC_DELAY_SEC(1));

    LED_Off(LED_TODA);

    while (MXC_UART_ReadyForSleep(MXC_UART_GET_UART(CONSOLE_UART)) != E_NO_ERROR) {}

    MXC_LP_EnableRTCAlarmWakeup();
    MXC_LP_EnterBackupMode(); // Enter a backup mode

    return 0;
}
