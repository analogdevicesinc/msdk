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
 * @file    main.c
 * @brief   Wake-Up Timer (WUT) example for low power modes.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_delay.h"
#include "mxc_device.h"
#include "board.h"
#include "led.h"
#include "lp.h"
#include "pb.h"
#include "uart.h"
#include "wut.h"

/***** Definitions *****/
#define MILLISECONDS_WUT 5000

/***** Functions *****/
void setTrigger(int waitForTrigger)
{
    if (waitForTrigger) {
        while (!PB_Get(0)) {}
        MXC_Delay(MXC_DELAY_MSEC(250));
    }

    // Debounce the button press.
    for (int tmp = 0; tmp < 0x80000; tmp++) {
        __NOP();
    }

    // Wait for serial transactions to complete.
    while (MXC_UART_ReadyForSleep(MXC_UART_GET_UART(CONSOLE_UART)) != E_NO_ERROR) {}
}

void WUT0_IRQHandler(void)
{
    MXC_WUT_ClearFlags(MXC_WUT0);
}

int main(void)
{
    mxc_wut_cfg_t cfg;
    uint32_t ticks;

    printf("\n\n/*********** Wakeup timer example *************/\n");
    printf("This example demonstrates how to use the Wakeup Timer.\n\n");
    printf("Pressing PB0 to will put the chip to sleep and enable the\n");
    printf("wakeup timer to wake the device in %d Miliseconds.\n\n", MILLISECONDS_WUT);

    // Initialize WUT
    MXC_WUT_Init(MXC_WUT0, MXC_WUT_PRES_1);

    // Get ticks based off of milliseconds
    MXC_WUT_GetTicks(MXC_WUT0, MILLISECONDS_WUT, MXC_WUT_UNIT_MILLISEC, &ticks);

    // Config WUT
    cfg.mode = MXC_WUT_MODE_ONESHOT;
    cfg.cmp_cnt = ticks;
    MXC_WUT_Config(MXC_WUT0, &cfg);
    NVIC_EnableIRQ(WUT0_IRQn);

    // Enable WUT wakeup event
    MXC_LP_EnableWUTAlarmWakeup();

    while (1) {
        // Wait for SW2 press
        setTrigger(1);

        // Start WUT
        printf("Entering SLEEP mode.\n");
        MXC_WUT_Enable(MXC_WUT0);

        // Put the chip to sleep
        MXC_LP_EnterSleepMode();

        printf("Waking up from SLEEP mode.\n");
    }
}
