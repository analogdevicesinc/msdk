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
 * @details     THE RTC is configured to wake the device from backup mode every
 * 				TIME_OF_DAY seconds. On wakeup, the device will print the current
 * 				time, rearm the alarm, and
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
void RTC_IRQHandler(void) {}

// *****************************************************************************
void rescheduleAlarm()
{
    int time;
    int flags = MXC_RTC_GetFlags();

    // Check for TOD alarm flag
    if (flags & MXC_F_RTC_CTRL_TOD_ALARM) {
        MXC_RTC_ClearFlags(MXC_F_RTC_CTRL_TOD_ALARM);

        time = MXC_RTC_GetSecond(); // Get Current time (s)

        // Disable interrupt while re-arming RTC alarm
        while (MXC_RTC_DisableInt(MXC_F_RTC_CTRL_TOD_ALARM_IE) == E_BUSY) {}

        // Reset TOD alarm for TIME_OF_DAY_SEC in the future
        if (MXC_RTC_SetTimeofdayAlarm(time + TIME_OF_DAY_SEC) != E_NO_ERROR) {
            /* Handle Error */
        }

        // Re-enable TOD alarm interrupt
        while (MXC_RTC_EnableInt(MXC_F_RTC_CTRL_TOD_ALARM_IE) == E_BUSY) {}
    }

    // Enable RTC as a wakeup source from low power modes
    MXC_LP_EnableRTCAlarmWakeup();
}

// *****************************************************************************
void printTime()
{
    uint32_t day, hr, min, sec;
    uint32_t dummy;

    // Get current time (don't care about Sub-second count here)
    while(MXC_RTC_GetTime(&sec, &dummy) != E_NO_ERROR) {}

    day = sec / SECS_PER_DAY;
    sec -= day * SECS_PER_DAY;

    hr = sec / SECS_PER_HR;
    sec -= hr * SECS_PER_HR;

    min = sec / SECS_PER_MIN;
    sec -= min * SECS_PER_MIN;

    printf("\nCurrent Time (dd:hh:mm:ss): %02d:%02d:%02d:%02d\n", day, hr, min, sec);
}

// *****************************************************************************
int configureRTC()
{
    int rtcTrim;

    printf("\n\n***************** RTC Wake from Backup Example *****************\n\n");
    printf("The time-of-day alarm is set to wake the device every %d seconds.\n", TIME_OF_DAY_SEC);
    printf("When the alarm goes off it will print the current time to the console.\n\n");

    // Initialize RTC
    if (MXC_RTC_Init(0, 0) != E_NO_ERROR) {
        printf("Failed RTC Initialization\n");
        printf("Example Failed\n");
        while (1) {}
    }

    // Start RTC
    if (MXC_RTC_Start() != E_NO_ERROR) {
        printf("Failed RTC_Start\n");
        printf("Example Failed\n");
        while (1) {}
    }

    printf("RTC started\n");

    // Trim RTC
    rtcTrim = MXC_RTC_TrimCrystal(MXC_TMR0);
    if (rtcTrim < 0) {
        printf("Error trimming RTC %d\n", rtcTrim);
    }

    // Set the Time of Day Alarm
    if (MXC_RTC_SetTimeofdayAlarm(TIME_OF_DAY_SEC) != E_NO_ERROR) {
        printf("Failed RTC_SetTimeofdayAlarm\n");
        printf("Example Failed\n");
        while (1) {}
    }

    // Enable Time of Day Alarm interrupt
    if (MXC_RTC_EnableInt(MXC_F_RTC_CTRL_TOD_ALARM_IE) == E_BUSY) {
        return E_BUSY;
    }

    // Re-start RTC
    if (MXC_RTC_Start() != E_NO_ERROR) {
        printf("Failed RTC_Start\n");
        printf("Example Failed\n");
        while (1) {}
    }

    return E_NO_ERROR;
}

// *****************************************************************************
int main(void)
{
    if (!(MXC_PWRSEQ->lppwkst & MXC_F_PWRSEQ_LPPWKST_BACKUP)) {
    	// Did not wake from backup mode --> start RTC
        if (configureRTC() != E_NO_ERROR) {
            printf("Example Failed\n");
            while (1) {}
        }
    } else {
    	// Woke up from backup mode --> Reset backup status and print time
        MXC_PWRSEQ->lppwkst |=MXC_F_PWRSEQ_LPPWKST_BACKUP;

        LED_On(LED_TODA); // RTC alarm fired off. Perform periodic task here
        printTime();
    }

    // (Re)arm RTC TOD alarm
    rescheduleAlarm();
    MXC_Delay(MXC_DELAY_SEC(1)); // Prevent bricks

    // Prepare for entering backup mode
    LED_Off(LED_TODA);
    while (MXC_UART_ReadyForSleep(MXC_UART_GET_UART(CONSOLE_UART)) != E_NO_ERROR) {}

    // Enter backup mode and wait for RTC to send wakeup signal
    MXC_LP_EnterBackupMode();
    while(1);

    return 0;
}
