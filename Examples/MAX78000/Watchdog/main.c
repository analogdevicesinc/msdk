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
 * @details SW2: Push SW2 to trigger a "too-late" watchdog reset. This will stop resetting the
 *               watchdog timer until it generates the "too-late" interrupt.  After that it will
 *               reset the watchdog timer only once, allowing it to pass the reset timeout period.
 *
 *          SW3: Push SW3 to reset the watchdog timer in the "too-early" period.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "nvic_table.h"
#include "board.h"
#include "mxc_sys.h"
#include "wdt.h"
#include "mxc_delay.h"
#include "led.h"
#include "pb.h"

/***** Definitions *****/

#if defined(BOARD_EVKIT_V1)
#define LATE_SW_NAME  "SW2"
#define EARLY_SW_NAME "SW3"
#elif defined(BOARD_FTHR_REVA)
#define LATE_SW_NAME  "SW1"
#define EARLY_SW_NAME "SW2"
#else
#error "This example has been written to work with the MAX78000 EV Kit and FTHR boards."
#endif

/***** Globals *****/
static mxc_wdt_cfg_t cfg;

volatile int sw1_pressed     = 0;
volatile int sw2_pressed     = 0;
volatile int interrupt_count = 0;

// refers to array, do not change constants
#define SW1 0
#define LED LED1
/***** Functions *****/

// *****************************************************************************
void watchdogHandler()
{
    MXC_WDT_ClearIntFlag(MXC_WDT0);

    if (interrupt_count == 0) {
        printf("\nWatchdog has tripped!\n");
        printf("This is the first time, so we'll go ahead and reset it\n");
        printf("once it is within the proper window.\n");
        interrupt_count++;
    } else {
        printf("\nWatchdog has tripped!\n");
        printf("This is the not the first time.  What happens if we\n");
        printf("do not reset it?\n");
    }
}

// *****************************************************************************
void WDT0_IRQHandler(void)
{
    watchdogHandler();
}
// *****************************************************************************
void MXC_WDT_Setup()
{
    MXC_WDT_Disable(MXC_WDT0);
    MXC_WDT_ResetTimer(MXC_WDT0);
    cfg.lowerResetPeriod = MXC_WDT_PERIOD_2_24;
    cfg.upperResetPeriod = MXC_WDT_PERIOD_2_28;
    MXC_WDT_SetResetPeriod(MXC_WDT0, &cfg);
    MXC_WDT_Enable(MXC_WDT0);
}

void SW1_Callback()
{
    printf("\nEnabling Timeout Interrupt...\n");
    MXC_WDT_Disable(MXC_WDT0);
    cfg.upperResetPeriod = MXC_WDT_PERIOD_2_28;
    cfg.upperIntPeriod   = MXC_WDT_PERIOD_2_27;
    cfg.lowerResetPeriod = MXC_WDT_PERIOD_2_24;
    cfg.lowerIntPeriod   = MXC_WDT_PERIOD_2_23;
    MXC_WDT_SetResetPeriod(MXC_WDT0, &cfg);
    MXC_WDT_SetIntPeriod(MXC_WDT0, &cfg);
    MXC_WDT_ResetTimer(MXC_WDT0);
    MXC_WDT_EnableReset(MXC_WDT0);
    MXC_WDT_EnableInt(MXC_WDT0);
    MXC_NVIC_SetVector(WDT0_IRQn, WDT0_IRQHandler);
    NVIC_EnableIRQ(WDT0_IRQn);
    MXC_WDT_Enable(MXC_WDT0);
    sw1_pressed = 1;
    PB_RegisterCallback(0, NULL);
}

void SW2_Callback()
{
    printf("What happens if the watchdog is reset too early?\n");
    sw2_pressed = 1;
    PB_RegisterCallback(1, NULL);
}

// *****************************************************************************
int main(void)
{
    cfg.mode = MXC_WDT_WINDOWED;
    MXC_WDT_Init(MXC_WDT0, &cfg);

    if (MXC_WDT_GetResetFlag(MXC_WDT0)) {
        uint32_t resetFlags = MXC_WDT_GetResetFlag(MXC_WDT0);

        if (resetFlags == MXC_F_WDT_CTRL_RST_LATE) {
            printf("\nWatchdog Reset occured too late (OVERFLOW)\n");
        } else if (resetFlags == MXC_F_WDT_CTRL_RST_EARLY) {
            printf("\nWatchdog Reset occured too soon (UNDERFLOW)\n");
        }

        MXC_WDT_ClearResetFlag(MXC_WDT0);
        MXC_WDT_ClearIntFlag(MXC_WDT0);
        MXC_WDT_EnableReset(MXC_WDT0);
        MXC_WDT_Enable(MXC_WDT0);
    }

    printf("\n************** Watchdog Timer Demo ****************\n");
    printf("%s: Push %s to trigger a \"too-late\" watchdog reset. This will stop resetting\n",
           LATE_SW_NAME, LATE_SW_NAME);
    printf("     the watchdog timer until it generates the \"too-late\" interrupt.  After that\n");
    printf("     it will reset the watchdog timer only once, allowing it to pass the reset\n");
    printf("     timeout period.\n\n");
    printf("%s: Push %s to reset the watchdog timer in the \"too-early\" period.\n", EARLY_SW_NAME,
           EARLY_SW_NAME);

    //Blink LED
    LED_Off(0);

    //Blink LED three times at startup
    int numBlinks = 3;

    while (numBlinks) {
        LED_On(0);
        MXC_Delay(MXC_DELAY_MSEC(100));
        LED_Off(0);
        MXC_Delay(MXC_DELAY_MSEC(100));
        numBlinks--;
    }

    //Setup and start watchdog
    MXC_WDT_Setup();

    // Configure push buttons
    PB_RegisterCallback(0, SW1_Callback);
    PB_IntEnable(0);
    PB_RegisterCallback(1, SW2_Callback);
    PB_IntEnable(1);

    //Push SW1 to start longer delay - shows Interrupt before the reset happens

    while (1) {
        if (sw1_pressed) {
            if (interrupt_count == 0) {
                while (interrupt_count == 0) {};

                MXC_Delay(MXC_DELAY_MSEC(1500));
            } else {
                while (1)
                    ;
            }
        }

        if (sw2_pressed) {
            // Reset the WDT too early.
            MXC_Delay(MXC_DELAY_MSEC(200));
            MXC_WDT_ResetTimer(MXC_WDT0);
            sw2_pressed = 0;
        }

        //blink LED0
        MXC_Delay(MXC_DELAY_MSEC(500));
        LED_On(0);
        MXC_Delay(MXC_DELAY_MSEC(500));
        LED_Off(0);
        //Reset watchdog
        MXC_WDT_ResetTimer(MXC_WDT0);
    }
}
