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
 * @file        main.c
 * @brief       Configures and starts the RTC and demonstrates the use of the alarms.
 * @details     In this example the RTC is set up to show the functionality of the 
 *                  RTC second and sub-second alarms. When the sub-second alarm expires,
 *                  the LED is toggled. When the second alarm expires, the rate at which
 *                  the LED is toggled is switched between 250 and 750ms. Additionally,
 *                  when SW2 is pressed the current time will be printed to the console.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "nvic_table.h"
#include "board.h"
#include "rtc.h"
#include "gpio.h"
#include "led.h"
#include "pb.h"
#include "mxc_delay.h"

/***** Definitions *****/
#define TIME_OF_DAY_SEC     5
#define SUBSECOND_MSEC_0    250
#define SUBSECOND_MSEC_1    750

#define MSEC_TO_RSSA(x) (0 - ((x * 4096) / 1000)) /* Converts a time in milleseconds to the equivalent RSSA register value. */

#define SECS_PER_MIN        60
#define SECS_PER_HR         (60 * SECS_PER_MIN)
#define SECS_PER_DAY        (24 * SECS_PER_HR)

/***** Globals *****/
int ss_interval = SUBSECOND_MSEC_0;

/***** Functions *****/
void RTC_IRQHandler(void)
{
    int time;
    int flags = MXC_RTC_GetFlags();
    /* Check sub-second alarm flag. */
    if (flags & MXC_RTC_INT_FL_SHORT) {
        LED_Toggle(0);
        MXC_RTC_ClearFlags(MXC_RTC_INT_FL_SHORT);
    }
    
    /* Check time-of-day alarm flag. */
    if (flags & MXC_RTC_INT_FL_LONG) {
        MXC_RTC_ClearFlags(MXC_RTC_INT_FL_LONG);
        
        while(MXC_RTC_DisableInt(MXC_RTC_INT_EN_LONG) == E_BUSY);   

        /* Set a new alarm TIME_OF_DAY_SEC seconds from current time. */
        /* Don't need to check busy here as it was checked in MXC_RTC_DisableInt() */		
        time = MXC_RTC_GetSecond();
		
        if (MXC_RTC_SetTimeofdayAlarm( time + TIME_OF_DAY_SEC) != E_NO_ERROR) {
            /* Handle Error */
        }
        while(MXC_RTC_EnableInt(MXC_RTC_INT_EN_LONG) == E_BUSY);    

        // Toggle the sub-second alarm interval.
        if (ss_interval == SUBSECOND_MSEC_0) {
            ss_interval = SUBSECOND_MSEC_1;
        } else {
            ss_interval = SUBSECOND_MSEC_0;
        }

        while(MXC_RTC_DisableInt(MXC_RTC_INT_EN_SHORT) == E_BUSY);
        if (MXC_RTC_SetSubsecondAlarm( MSEC_TO_RSSA(ss_interval)) != E_NO_ERROR) {
            /* Handle Error */
        }
        while(MXC_RTC_EnableInt(MXC_RTC_INT_EN_SHORT) == E_BUSY); 
    }
    
    return;
}

volatile int buttonPressed = 0;
void buttonHandler()
{
    buttonPressed = 1;
}

void printTime()
{
    int day, hr, min, sec, rtc_readout;
    double subsec;

    do {
    	rtc_readout = MXC_RTC_GetSubSecond();
    }
    while (rtc_readout == E_BUSY);
    subsec = rtc_readout / 4096.0;

    do {
    	rtc_readout = MXC_RTC_GetSecond();
    }
    while (rtc_readout == E_BUSY);
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
    printf("   the LED is toggled is switched between %d and %dms. Additionally,\n", SUBSECOND_MSEC_0, SUBSECOND_MSEC_1);
    printf("   when SW2 is pressed the current time will be printed to the console.\n\n");
    
    NVIC_EnableIRQ(RTC_IRQn);

    
    /* Setup callback to receive notification of when button is pressed. */
    PB_RegisterCallback(0, (pb_callback)buttonHandler);

    /* Turn LED off initially */
    LED_Off(0);
    
    if (MXC_RTC_Init(0, 0) != E_NO_ERROR) {
        printf("Failed RTC Initialization\n");
        printf("Example Failed\n");
        while(1);
    }
    
    if(MXC_RTC_DisableInt(MXC_RTC_INT_EN_LONG) == E_BUSY) {
        return E_BUSY;
    }

    if (MXC_RTC_SetTimeofdayAlarm(TIME_OF_DAY_SEC) != E_NO_ERROR) {
        printf("Failed RTC_SetTimeofdayAlarm\n");
        printf("Example Failed\n");
        while(1);
    }

    if(MXC_RTC_EnableInt(MXC_RTC_INT_EN_LONG) == E_BUSY) {
        return E_BUSY;
    }

    if(MXC_RTC_DisableInt(MXC_RTC_INT_EN_SHORT) == E_BUSY) {
        return E_BUSY;
    }

    if (MXC_RTC_SetSubsecondAlarm(MSEC_TO_RSSA(SUBSECOND_MSEC_0)) != E_NO_ERROR) {
        printf("Failed RTC_SetSubsecondAlarm\n");
        printf("Example Failed\n");
        while(1);
    }
    
    if(MXC_RTC_EnableInt(MXC_RTC_INT_EN_SHORT) == E_BUSY) {
        return E_BUSY;
    }

    if (MXC_RTC_SquareWaveStart(MXC_RTC_F_512HZ) == E_BUSY) {
        return E_BUSY;
    }

    if (MXC_RTC_Start() != E_NO_ERROR) {
        printf("Failed RTC_Start\n");
        printf("Example Failed\n");
        while(1);
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
