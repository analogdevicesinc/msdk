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

/*
 * @file    main.c
 * @brief   Demonstrates a watchdog timer in run mode
 *
 * @details When the program starts LED0 blinks three times and stops.
 *          Then LED0 start blinking continuously.
 *          Open a terminal program to see interrupt messages.
 *
 *          SW2: Push SW2 to trigger a watchdog reset. This will reset the watchdog before
 *               the wait period has expired and trigger an interrupt.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_sys.h"
#include "nvic_table.h"
#include "gpio.h"
#include "wdt.h"
#include "board.h"
#include "led.h"
#include "pb.h"

/***** Definitions *****/
#define OVERFLOW //Test Windowed timer \
                 //OVERFLOW            \
                 //UNDERFLOW

#define RESET_PERIOD MXC_WDT_PERIOD_2_28
#define INT_PERIOD   MXC_WDT_PERIOD_2_27

// refers to array, do not change constants
#define SW2 0
#define LED 0

/***** Globals *****/
volatile uint32_t intCnt = 0;

/***** Functions *****/
void watchdogHandler()
{
    MXC_WDT_ClearIntFlag(MXC_WDT0);
    printf("\nTIMEOUT!\n");
    if (intCnt == 0) {
        MXC_WDT_ResetTimer(MXC_WDT0);
    }
    intCnt++;
}

void WDT0_IRQHandler(void)
{
    watchdogHandler();
}

void WDT_Setup()
{
    MXC_WDT_Disable(MXC_WDT0);
    MXC_WDT_SetResetPeriod(MXC_WDT0, RESET_PERIOD);
    MXC_WDT_SetIntPeriod(MXC_WDT0, INT_PERIOD);
    MXC_WDT_ResetTimer(MXC_WDT0);
    MXC_WDT_EnableReset(MXC_WDT0);
    MXC_WDT_EnableInt(MXC_WDT0);
    MXC_NVIC_SetVector(WDT0_IRQn, WDT0_IRQHandler);
    NVIC_EnableIRQ(WDT0_IRQn);
    MXC_WDT_Enable(MXC_WDT0);

    printf("\nWatchdog configured.\n");
    printf("Press (and hold) SW2 to allow Watchdog to expire.\n\n");
}

int main(void)
{
    MXC_WDT_Init(MXC_WDT0);

    if (MXC_WDT_GetResetFlag(MXC_WDT0)) {
        uint32_t resetFlags = MXC_WDT_GetResetFlag(MXC_WDT0);
        if (resetFlags == MXC_F_WDT_CTRL_RST_FLAG) {
            printf("\nWatchdog reset occurred.\n");
        }
        MXC_WDT_ClearResetFlag(MXC_WDT0);
        MXC_WDT_ClearIntFlag(MXC_WDT0);
    }

    printf("\n************** Watchdog Timer Demo ****************\n");
    printf("Watchdog timer is configured in Windowed mode. Until the\n");
    printf("button is pressed, the watchdog will be continually reset\n");
    printf("in the main loop.\n");
    printf("\nPressing button (SW2) will prevent the watchdog from\n");
    printf("reseting, allowing the timer to expire and the device to reset.\n");

    //Blink LED
    LED_Off(LED);
    int numBlinks = 5;

    while (numBlinks) {
        LED_On(LED);
        MXC_Delay(MXC_DELAY_MSEC(100));
        LED_Off(LED);
        MXC_Delay(MXC_DELAY_MSEC(100));
        numBlinks--;
    }

    //Setup watchdog
    WDT_Setup();

    while (1) {
        //Push SW1 to see interrupt and reset handlers
        if (PB_Get(SW2) != 0) {
            while (intCnt == 0)
                ;
            printf("WDT Timer restored in interrupt handler before reset.\n");
            while (intCnt == 1)
                ;
            printf("WDT Timer not restored in interrupt handler. Device will reset shortly.\n");
            while (1)
                ;
        }

        //blink LED0
        MXC_Delay(MXC_DELAY_MSEC(500));
        LED_On(LED);
        MXC_Delay(MXC_DELAY_MSEC(500));
        LED_Off(LED);

        //Reset watchdog
        MXC_WDT_ResetTimer(MXC_WDT0);
    }
}
