/******************************************************************************
 *
 * Copyright (C) 2025 Analog Devices, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

/**
 * @file        main.c
 * @brief       Configures and starts the RTC and demonstrates the use of the alarms.
 * @details     In this example the RTC is set up to show the functionality of the 
 *                  RTC second and sub-second alarms. When the sub-second alarm expires,
 *                  the LED is toggled. When the second alarm expires, the rate at which
 *                  the LED is toggled is switched between 250 and 750ms. Additionally,
 *                  when PB0 is pressed the current time will be printed to the console.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc.h"

/***** Definitions *****/
#define TIME_OF_DAY_SEC 5
#define SUBSECOND_MSEC_0 250
#define SUBSECOND_MSEC_1 750

#define MSEC_TO_RSSA(x) \
    (0 - ((x * 4096) /  \
          1000)) /* Converts a time in milleseconds to the equivalent RSSA register value. */

#define SECS_PER_MIN 60
#define SECS_PER_HR (60 * SECS_PER_MIN)
#define SECS_PER_DAY (24 * SECS_PER_HR)

/***** Globals *****/
int ss_interval = SUBSECOND_MSEC_0;

/***** Functions *****/
void RTC_IRQHandler(void)
{
    uint32_t time;
    int flags = MXC_RTC_GetFlags();
    /* Check sub-second alarm flag. */
    if (flags & MXC_RTC_INT_FL_SHORT) {
        LED_Toggle(0);
        MXC_RTC_ClearFlags(MXC_RTC_INT_FL_SHORT);
    }

    /* Check time-of-day alarm flag. */
    if (flags & MXC_RTC_INT_FL_LONG) {
        MXC_RTC_ClearFlags(MXC_RTC_INT_FL_LONG);

        /* Set a new alarm 10 seconds from current time. */
        MXC_RTC_GetSeconds(&time);

        while (MXC_RTC_DisableInt(MXC_RTC_INT_EN_LONG) == E_BUSY) {}
        if (MXC_RTC_SetTimeofdayAlarm(time + TIME_OF_DAY_SEC) != E_NO_ERROR) {
            /* Handle Error */
        }
        while (MXC_RTC_EnableInt(MXC_RTC_INT_EN_LONG) == E_BUSY) {}

        // Toggle the sub-second alarm interval.
        if (ss_interval == SUBSECOND_MSEC_0) {
            ss_interval = SUBSECOND_MSEC_1;
        } else {
            ss_interval = SUBSECOND_MSEC_0;
        }

        while (MXC_RTC_DisableInt(MXC_RTC_INT_EN_SHORT) == E_BUSY) {}
        if (MXC_RTC_SetSubsecondAlarm(MSEC_TO_RSSA(ss_interval)) != E_NO_ERROR) {
            /* Handle Error */
        }
        while (MXC_RTC_EnableInt(MXC_RTC_INT_EN_SHORT) == E_BUSY) {}
    }

    return;
}

volatile int buttonPressed = 0;
void buttonHandler(void)
{
    buttonPressed = 1;
}

void printTime(void)
{
    int day, hr, min, err;
    uint32_t sec, rtc_readout;
    double subsec;

    do {
        err = MXC_RTC_GetSubSeconds(&rtc_readout);
    } while (err != E_NO_ERROR);
    subsec = rtc_readout / 4096.0;

    do {
        err = MXC_RTC_GetSeconds(&rtc_readout);
    } while (err != E_NO_ERROR);
    sec = rtc_readout;

    day = sec / SECS_PER_DAY;
    sec -= day * SECS_PER_DAY;

    hr = sec / SECS_PER_HR;
    sec -= hr * SECS_PER_HR;

    min = sec / SECS_PER_MIN;
    sec -= min * SECS_PER_MIN;

    subsec += sec;

    printf("\nCurrent Time (dd:hh:mm:ss): %02d:%02d:%02d:%05.2f\n\n", day, hr, min, subsec);
}

// *****************************************************************************
int main(void)
{
    printf("\n*************************** RTC Example ****************************\n\n");
    printf("In this example the RTC is set up to demonstrate the functionality of the\n");
    printf("   RTC second and sub-second alarms. When the sub-second alarm expires,\n");
    printf("   the LED is toggled. When the second alarm expires, the rate at which\n");
    printf("   the LED is toggled is switched between %dms and %dms. Additionally,\n",
           SUBSECOND_MSEC_0, SUBSECOND_MSEC_1);
    printf("   when PB0 is pressed the current time will be printed to the console.\n\n");

    NVIC_EnableIRQ(RTC_IRQn);

    /* Setup callback to receive notification of when button is pressed. */
    PB_RegisterCallback(0, (pb_callback)buttonHandler);

    /* Turn LED off initially */
    LED_Off(0);

    if (MXC_RTC_Init(0, 0) != E_NO_ERROR) {
        printf("Failed RTC Initialization\n");
        printf("Example Failed\n");
        while (1) {}
    }

    if (MXC_RTC_DisableInt(MXC_RTC_INT_EN_LONG) == E_BUSY) {
        return E_BUSY;
    }

    if (MXC_RTC_SetTimeofdayAlarm(TIME_OF_DAY_SEC) != E_NO_ERROR) {
        printf("Failed RTC_SetTimeofdayAlarm\n");
        printf("Example Failed\n");
        while (1) {}
    }

    if (MXC_RTC_EnableInt(MXC_RTC_INT_EN_LONG) == E_BUSY) {
        return E_BUSY;
    }

    if (MXC_RTC_DisableInt(MXC_RTC_INT_EN_SHORT) == E_BUSY) {
        return E_BUSY;
    }

    if (MXC_RTC_SetSubsecondAlarm(MSEC_TO_RSSA(SUBSECOND_MSEC_0)) != E_NO_ERROR) {
        printf("Failed RTC_SetSubsecondAlarm\n");
        printf("Example Failed\n");
        while (1) {}
    }

    if (MXC_RTC_EnableInt(MXC_RTC_INT_EN_SHORT) == E_BUSY) {
        return E_BUSY;
    }

    if (MXC_RTC_Start() != E_NO_ERROR) {
        printf("Failed RTC_Start\n");
        printf("Example Failed\n");
        while (1) {}
    }

    printf("RTC started\n");
    printTime();

    while (1) {
        if (buttonPressed) {
            /* Show the time elapsed. */
            printTime();
            /* Delay for switch debouncing. */
            MXC_Delay(MXC_DELAY_MSEC(100));
            /* Re-arm switch detection. */
            buttonPressed = 0;
        }
    }
}
