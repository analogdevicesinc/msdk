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
 * @file    main.c
 * @brief   Hello World!
 * @details This example uses the UART to print to a terminal and flashes an LED.
 */

/***** Includes *****/
#include "board.h"
#include "led.h"
#include "lp.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "pb.h"
#include "uart.h"
#include "wut.h"
#include <stdint.h>
#include <stdio.h>

/***** Definitions *****/
#define MILLISECONDS_WUT 5000

/***** Functions *****/
void setTrigger(int waitForTrigger)
{
    if (waitForTrigger) {
        while (!PB_Get(0)) { }
        MXC_Delay(MXC_DELAY_MSEC(250));
    }

    // Debounce the button press.
    for (int tmp = 0; tmp < 0x80000; tmp++) {
        __NOP();
    }

    // Wait for serial transactions to complete.
    while (MXC_UART_ReadyForSleep(MXC_UART_GET_UART(CONSOLE_UART)) != E_NO_ERROR) { }
}

void WUT0_IRQHandler()
{
    MXC_WUT_IntClear();
}

int main(void)
{
    mxc_wut_cfg_t cfg;
    uint32_t ticks;

    printf("\n\n/*********** Wakeup timer example *************/\n");
    printf("This example demonstrates how to use the Wakeup Timer.\n\n");
    printf("Pressing SW2 to will put the chip to sleep and enable the\n");
    printf("wakeup timer to wake the device in %d Miliseconds.\n\n", MILLISECONDS_WUT);

    // Initialize WUT
    MXC_WUT_Init(MXC_WUT_PRES_1);

    // Get ticks based off of milliseconds
    MXC_WUT_GetTicks(MILLISECONDS_WUT, MXC_WUT_UNIT_MILLISEC, &ticks);

    // Config WUT
    cfg.mode = MXC_WUT_MODE_ONESHOT;
    cfg.cmp_cnt = ticks;
    MXC_WUT_Config(&cfg);
    NVIC_EnableIRQ(WUT0_IRQn);

    // Enable WUT wakeup event
    MXC_LP_EnableWUTAlarmWakeup();

    while (1) {
        // Wait for SW2 press
        setTrigger(1);

        // Start WUT
        printf("Entering SLEEP mode.\n");
        MXC_WUT_Enable();

        // Put the chip to sleep
        MXC_LP_EnterSleepMode();

        printf("Waking up from SLEEP mode.\n");
    }
}
