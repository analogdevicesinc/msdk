/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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
#include "mxc.h"
#include "board.h"
#include "pb.h"

/***** Definitions *****/
//#define SLEEP_MODE           // Select between SLEEP_MODE and LPM_MODE

#define MILLISECONDS_WUT 5000

/***** Globals *****/
volatile int buttonPressed;

/***** Functions *****/
void buttonHandler(void *pb)
{
    buttonPressed = 1;
}

void setTrigger(int waitForTrigger)
{
    int tmp;

    buttonPressed = 0;

    if (waitForTrigger) {
        while (!buttonPressed) {}
    }

    // Debounce the button press.
    for (tmp = 0; tmp < 0x80000; tmp++) {
        __NOP();
    }

    // Wait for serial transactions to complete.
    while (MXC_UART_ReadyForSleep(MXC_UART_GET_UART(CONSOLE_UART)) != E_NO_ERROR) {}
}

void WUT_IRQHandler()
{
    MXC_WUT_IntClear();
}

// *****************************************************************************
int main(void)
{
    mxc_wut_cfg_t cfg;
    uint32_t ticks;

    printf("\n\n************** Wakeup timer example ********************\n");
    printf("This example is to show how the Wakeup timer is used and configured.\n");

#ifdef BOARD_EVKIT_V1
    printf("Press SW2 to continue.\n");
#else
    printf("Press SW1 to continue.\n");
#endif

    PB_RegisterCallback(0, buttonHandler);

    // Get ticks based off of milliseconds
    MXC_WUT_GetTicks(MILLISECONDS_WUT, MXC_WUT_UNIT_MILLISEC, &ticks);

    // config structure for one shot timer to trigger in a number of ticks
    cfg.mode = MXC_WUT_MODE_ONESHOT;
    cfg.cmp_cnt = ticks;

    // Init WUT
    MXC_WUT_Init(MXC_WUT_PRES_1);

    //Config WUT
    MXC_WUT_Config(&cfg);
    MXC_LP_EnableWUTAlarmWakeup();

    while (1) {
        setTrigger(1);

        NVIC_EnableIRQ(WUT_IRQn);
        MXC_WUT_Enable();

#ifdef SLEEP_MODE
        printf("Entering SLEEP mode.\n");
        MXC_LP_EnterSleepMode();
        printf("Waking up from SLEEP mode.\n");

#else
        printf("Entering LPM mode.\n");
        MXC_LP_EnterLowPowerMode();
        printf("Waking up from LPM mode.\n");
#endif
        printf("\nPress the button again to trigger another sleep-wake cycle.\n");
    }
}
