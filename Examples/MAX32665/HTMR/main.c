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
 * @brief       Configures and starts the HTMR and demonstrates the use of the alarms.
 * @details     The HTMR is enabled and the sub-second alarm set to trigger every 500 ms.
 *              LED0 is toggled each time the sub-second alarm triggers.
 *              Pressing PB0 will output the current value of the HTMR to the console UART.
 *              Pressing PB1 will set time-of-day alarm to LONG_ALARM_COUNT value.
 *              When the time-of-day alarm triggers, the LED1 will be toggled.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc_errors.h"
#include "nvic_table.h"
#include "board.h"
#include "htmr.h"
#include "led.h"
#include "pb.h"
#include "gpio.h"

/***** Definitions *****/
#define LED_FLASH 0
#define LED_ALARM 1

#define LONG_ALARM_COUNT 2500

/***** Globals *****/
volatile int buttonPressed = 0;

/***** Functions *****/
void HTMR0_IRQHandler(void)
{
    if (MXC_HTMR_GetFlags(MXC_HTMR0) & MXC_F_HTMR_CTRL_ALDF) {
        LED_Toggle(LED_ALARM);
        MXC_HTMR_ClearFlags(MXC_HTMR0, MXC_F_HTMR_CTRL_ALDF);
    }
    if (MXC_HTMR_GetFlags(MXC_HTMR0) & MXC_F_HTMR_CTRL_ALSF) {
        LED_Toggle(LED_FLASH);
        MXC_HTMR_ClearFlags(MXC_HTMR0, MXC_F_HTMR_CTRL_ALSF);
    }

    return;
}

void buttonHandler()
{
    buttonPressed = 1;
}

void alarmSetHandler()
{
    MXC_HTMR_Stop(MXC_HTMR0);

    if (MXC_HTMR_SetLongAlarm(MXC_HTMR0, MXC_HTMR_GetLongCount(MXC_HTMR0) + LONG_ALARM_COUNT) !=
        E_NO_ERROR) {
        printf("Failed to set Long Interval Alarm.\n");
    }

    MXC_HTMR_Start(MXC_HTMR0);
}

static void printTime(void)
{
    double count;

    /* The short count register is 20 bits long, however
     * the long count counter is incremented every 4096 (12 bits) short counts
     * so we need to take short count mod 4096 and add long count to
     * get a float representing the total number of long counts
     */
    count = (MXC_HTMR_GetShortCount(MXC_HTMR0) % 4096) / 4096.0;
    count += MXC_HTMR_GetLongCount(MXC_HTMR0);

    printf("Current Count %f\n", count);
}

// *****************************************************************************
int main(void)
{
    printf("\n***** High Speed Timer Example *****\n\n");
    printf("This example enables the HTMR and sets the short interval\n");
    printf("alarm to trigger every ~2^22 short interval counts (0.5sec)\n");
    printf("Pressing PB0 will print the current count to the console\n");
    printf("Pressing PB1 will set the Long Interval alarm to light LED1 in %d counts\n\n",
           LONG_ALARM_COUNT);
#if !defined(BOARD_FTHR)
    printf("Pressing PB1 will set the Long Interval alarm to light LED1 in %d counts\n\n",
           LONG_ALARM_COUNT);
#else
    printf("The board has only one push button\n");
#endif

    MXC_NVIC_SetVector(HTMR0_IRQn, HTMR0_IRQHandler);
    NVIC_EnableIRQ(HTMR0_IRQn);

    /* Setup callback to receive notification of when button is pressed. */
    PB_RegisterCallback(0, (pb_callback)buttonHandler);
#if !defined(BOARD_FTHR)
    PB_RegisterCallback(1, (pb_callback)alarmSetHandler);
#endif

    /* Turn LED off initially */
    LED_Off(LED_ALARM);
    LED_Off(LED_FLASH);

    if (MXC_HTMR_Init(MXC_HTMR0, 0, 0) != E_NO_ERROR) {
        printf("Failed HTMR Initialization.\n");
        while (1) {}
    }

    if (MXC_HTMR_SetShortAlarm(MXC_HTMR0, 0xFFC7BFFF) != E_NO_ERROR) {
        printf("Failed to set short interval alarm\n");
    }

    MXC_HTMR_Start(MXC_HTMR0);
    printf("\nTimer started.\n\n");
    printTime();

    while (1) {
        if (buttonPressed) {
            /* Show the time elapsed. */
            printTime();
            /* Delay for switch debouncing. */
            MXC_Delay(MXC_DELAY_MSEC(200));
            /* Re-arm switch detection. */
            buttonPressed = 0;
        }
    }
}
