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

/*
 * @file    main.c
 * @brief   Demonstrates a watchdog timer in run mode
 *
 * @details When the program starts LED3 blinks three times and stops.
 *          Then LED0 start blinking continuously.
 *          Open a terminal program to see interrupt messages.
 *
 *          SW2: Push SW3 to trigger a watchdog reset. This will reset the watchdog before
 *               the wait period has expired and trigger an interrupt.
 *
 *          SW3: Push SW3 to trigger a longer delay and see the program restart by blinking LED3 three times.
 *               This delay is long enough for the reset period to expire and trigger a reset.
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
#include "uart.h"

/***** Definitions *****/
#define OVERFLOW
//#define UNDERFLOW

/***** Globals *****/
static mxc_wdt_cfg_t cfg;

/***** Functions *****/
void WDT0_IRQHandler(void)
{
	MXC_WDT_ClearIntFlag(MXC_WDT0);
	printf("\nTIMEOUT!\n");
}

void wdt_setup()
{
    MXC_WDT_Disable(MXC_WDT0);
    MXC_WDT_ResetTimer(MXC_WDT0);
    MXC_WDT_Enable(MXC_WDT0);
}

void blink_led(int led, int num_of_blink, unsigned int delay_ms)
{
    for (int i = 0; i < num_of_blink; i++) {
        LED_On(led);
        MXC_Delay(MXC_DELAY_MSEC(delay_ms));
        LED_Off(led);
        MXC_Delay(MXC_DELAY_MSEC(delay_ms));
    }
}

// *****************************************************************************
int main(void)
{
    cfg.mode = MXC_WDT_WINDOWED;
    MXC_WDT_Init(MXC_WDT0, &cfg);

    if (MXC_WDT_GetResetFlag(MXC_WDT0)) {
    	// WDT Caused device reset
    	uint32_t resetFlags = MXC_WDT_GetResetFlag(MXC_WDT0);

    	// Determine whether Underflow or Overflow caused reset
        if (resetFlags == MXC_F_WDT_CTRL_RST_LATE) {
            printf("\nWatchdog Reset occurred too late (OVERFLOW)\n");
        } else if (resetFlags == MXC_F_WDT_CTRL_RST_EARLY) {
            printf("\nWatchdog Reset occurred too soon (UNDERFLOW)\n");
        }

        // Clear Flags
        MXC_WDT_ClearResetFlag(MXC_WDT0);
        MXC_WDT_ClearIntFlag(MXC_WDT0);
    }

    printf("\n******************** Watchdog Timer Demo ********************\n");
    printf("This example demonstrates the WDT in windowed mode. With UNDERFLOW\n");
    printf("defined the WDT count reset will occur before the window, causing\n");
    printf("a \"too soon\" WDT system reset. With OVERFLOW defined the device\n");
    printf("will wait in an infinite loop until the window expires, causing a\n");
    printf("\"too late\" WDT system reset\n\n");

    printf("Press push button SW3 (P0.18) to trigger the WDT interrupt and system\n");
    printf("reset described above.\n\n");

    //Blink LED three times at startup
    blink_led(0, 3, 100);

    //Setup Watchdog
    wdt_setup();

    while (1) {
        //Push user push button to reset watchdog
        if (PB_Get(0) == TRUE) {
            MXC_WDT_Disable(MXC_WDT0);

            // Configure reset window
            cfg.mode = MXC_WDT_WINDOWED;
            cfg.upperResetPeriod = MXC_WDT_PERIOD_2_28;
            cfg.upperIntPeriod = MXC_WDT_PERIOD_2_27;
            cfg.lowerResetPeriod = MXC_WDT_PERIOD_2_24;
            cfg.lowerIntPeriod = MXC_WDT_PERIOD_2_25;
            MXC_WDT_SetResetPeriod(MXC_WDT0, &cfg);
            MXC_WDT_SetIntPeriod(MXC_WDT0, &cfg);

            // Enable WDT reset and interrupts
            MXC_WDT_EnableReset(MXC_WDT0);
            MXC_WDT_ClearIntFlag(MXC_WDT0);
            MXC_WDT_EnableInt(MXC_WDT0);
            NVIC_EnableIRQ(WDT0_IRQn);

            MXC_WDT_ResetTimer(MXC_WDT0); //Feed the dog
            MXC_WDT_Enable(MXC_WDT0); //Re-enable WDT

            printf("Watchdog reset window configured.\n");

#ifdef OVERFLOW
            // Wait for reset window to pass (causes reset)
            printf("Starving the dog until reset window expires...\n");
            while (1) {}
#else
            printf("Feeding the dog before entering reset window...\n");
            while(MXC_UART_GetActive(MXC_UART_GET_UART(CONSOLE_UART)));

            // Reset timer before window (causes reset)
            MXC_WDT_ResetTimer(MXC_WDT0);
#endif //OVERFLOW
        }

        // blink LED0
        blink_led(0, 1, 500);

        // Feed the dog
        MXC_WDT_ResetTimer(MXC_WDT0);
    }
}
