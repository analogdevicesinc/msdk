/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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
 * @details     The RTC is enabled and the sub-second alarm set to trigger every 250 ms.
 *              P2.25 (LED0) is toggled each time the sub-second alarm triggers.  The
 *              time-of-day alarm is set to 2 seconds.  When the time-of-day alarm
 *              triggers, the rate of the sub-second alarm is switched to 500 ms.  The
 *              time-of-day alarm is then rearmed for another 2 sec.  Pressing SW2 will
 *              output the current value of the RTC to the console UART.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>

#include <MAX32xxx.h>

/***** Definitions *****/
#define LED_ALARM           0

#define TIME_OF_DAY_SEC     2
#define SUBSECOND_MSEC_0    250
#define SUBSECOND_MSEC_1    500

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
        LED_Toggle(LED_ALARM);
        MXC_RTC_ClearFlags(MXC_RTC_INT_FL_SHORT);
    }
    
    /* Check time-of-day alarm flag. */
    if (flags & MXC_RTC_INT_FL_LONG) {
        MXC_RTC_ClearFlags(MXC_RTC_INT_FL_LONG);
        
		if(MXC_RTC_DisableInt(MXC_RTC_INT_EN_LONG | MXC_RTC_INT_EN_SHORT) == E_BUSY) {
			/* Handle Error */
		}
	
        /* Set a new alarm TIME_OF_DAY_SEC seconds from current time. */
        time = MXC_RTC_GetSecond();
        
        if (MXC_RTC_SetTimeofdayAlarm(time + TIME_OF_DAY_SEC) != E_NO_ERROR) {
            /* Handle Error */
        }
        
        // Toggle the sub-second alarm interval.
        if (ss_interval == SUBSECOND_MSEC_0) {
            ss_interval = SUBSECOND_MSEC_1;
        }
        else {
            ss_interval = SUBSECOND_MSEC_0;
        }
        
        if (MXC_RTC_SetSubsecondAlarm(MSEC_TO_RSSA(ss_interval)) != E_NO_ERROR) {
            /* Handle Error */
        }
		
		if(MXC_RTC_EnableInt(MXC_RTC_INT_EN_LONG | MXC_RTC_INT_EN_SHORT) == E_BUSY) {
			/* Handle Error */
		}
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
    int day, hr, min, sec;
    double subsec;
    
    subsec = MXC_RTC_GetSubSecond() / 4096.0;
    sec = MXC_RTC_GetSecond();
    
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
    printf("The RTC is enabled and the sub-second alarm set to trigger every %d ms.\n", SUBSECOND_MSEC_0);
    printf("(LED0) is toggled each time the sub-second alarm triggers.\n\n");
    printf("The time-of-day alarm is set to %d seconds.  When the time-of-day alarm\n", TIME_OF_DAY_SEC);
    printf("triggers, the rate of the sub-second alarm is switched to %d ms.\n\n",		SUBSECOND_MSEC_1);
    printf("The time-of-day alarm is then rearmed for another %d sec.  Pressing PB1\n",TIME_OF_DAY_SEC);
    printf("will output the current value of the RTC to the console UART.\n\n");
    
    NVIC_EnableIRQ(RTC_IRQn);
    
    
    /* Setup callback to receive notification of when button is pressed. */
    PB_RegisterCallback(0, (pb_callback) buttonHandler);
    
    /* Turn LED off initially */
    LED_On(LED_ALARM);
    
    if (MXC_RTC_Init(0, 0) != E_NO_ERROR) {
        printf("Failed RTC Initialization\n");
        printf("Example Failed\n");
        
        while (1);
    }
    
    printf("RTC started\n");
    printTime();
    
    if (MXC_RTC_DisableInt(MXC_RTC_INT_EN_LONG) == E_BUSY) {
        return E_BUSY;
    }
    
    if (MXC_RTC_SetTimeofdayAlarm(TIME_OF_DAY_SEC) != E_NO_ERROR) {
        printf("Failed RTC_SetTimeofdayAlarm\n");
        printf("Example Failed\n");
        
        while (1);
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
        
        while (1);
    }
    
    if (MXC_RTC_EnableInt(MXC_RTC_INT_EN_SHORT) == E_BUSY) {
        return E_BUSY;
    }
    
    if (MXC_RTC_Start() != E_NO_ERROR) {
        printf("Failed RTC_Start\n");
        printf("Example Failed\n");
        
        while (1);
    }
    
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
