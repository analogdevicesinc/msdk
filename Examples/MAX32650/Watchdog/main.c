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
 * @details When the program starts LED1 blinks three times and stops.
 *          Then LED0 start blinking continuously.
 *          Open a terminal program to see interrupt messages.
 *
 *          SW2: Push SW1 to trigger a watchdog reset. This will reset the watchdog before
 *               the wait period has expired and trigger an interrupt.
 *
 *          SW3: Push SW2 to trigger a delay and see LED0 stop blinking momentarily.
 *               This delay long enough for the timeout period to expire and trigger an interrupt.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "nvic_table.h"
#include "board.h"
#include "wdt.h"
#include "led.h"
#include "pb.h"

/***** Definitions *****/
/***** Globals *****/
//use push buttons defined in board.h
extern const mxc_gpio_cfg_t pb_pin[];

#define SW1 0
#define SW2 1

/***** Functions *****/

// *****************************************************************************
void watchdog_timeout_handler()
{
    //get and clear flag
    MXC_WDT_ClearIntFlag(MXC_WDT0);
    printf("\nTIMEOUT!\n");
    printf("Waiting for watchdog reset...\n");
}

// *****************************************************************************
void WDT0_IRQHandler(void)
{
    watchdog_timeout_handler();
}
// *****************************************************************************
void MXC_WDT_Setup()
{
    MXC_WDT_Disable(MXC_WDT0);
    MXC_WDT_Enable(MXC_WDT0);
}

// *****************************************************************************
int main(void)
{
    MXC_WDT_Init(MXC_WDT0);
    if (MXC_WDT_GetResetFlag(MXC_WDT0)) {
        MXC_WDT_ClearResetFlag(MXC_WDT0);
        MXC_WDT_DisableReset(MXC_WDT0);
        MXC_WDT_Enable(MXC_WDT0);
        printf("\nWatchdog reset\n");
    }

    printf("\n************** Watchdog Timer Example ****************\n");
    printf("Press a button to trigger a watchdog reset.\n");
    printf("SW2 (P2.25) = Reset without interrupt.\n");
    printf("SW3 (P2.26) = Reset with interrupt.\n");

    //blink LED1 three times at startup
    int numBlinks = 10;

    while (numBlinks) {
        LED_On(1);
        MXC_Delay(MXC_DELAY_MSEC(100));
        LED_Off(1);
        MXC_Delay(MXC_DELAY_MSEC(100));
        numBlinks--;
    }

    //setup watchdog
    MXC_WDT_Setup();

    while (1) {
        //Push SW1 to reset watchdog
        if (MXC_GPIO_InGet(pb_pin[SW1].port, pb_pin[SW1].mask) == 0) {
            printf("\nWaiting for watchdog reset...\n");
            MXC_WDT_SetResetPeriod(MXC_WDT0, MXC_WDT_PERIOD_2_28);
            MXC_WDT_EnableReset(MXC_WDT0);
            MXC_WDT_EnableInt(MXC_WDT0);
            while (1)
                ;
        }

        //Push SW2 to start longer delay - shows Interrupt before the reset happens
        if (MXC_GPIO_InGet(pb_pin[SW2].port, pb_pin[SW2].mask) == 0) {
            printf("\nEnabling Timeout Interrupt...\n");
            MXC_WDT_SetResetPeriod(MXC_WDT0, MXC_WDT_PERIOD_2_28);
            MXC_WDT_SetIntPeriod(MXC_WDT0, MXC_WDT_PERIOD_2_27);
            MXC_WDT_EnableReset(MXC_WDT0);
            MXC_WDT_EnableInt(MXC_WDT0);
            NVIC_EnableIRQ(WDT0_IRQn);
            while (1)
                ;
        }

        //blink LED0
        MXC_Delay(MXC_DELAY_MSEC(100));
        LED_On(0);
        MXC_Delay(MXC_DELAY_MSEC(100));
        LED_Off(0);

        //Reset watchdog
        MXC_WDT_ResetTimer(MXC_WDT0);
    }
}
