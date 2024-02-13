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
 * @brief   ADC comparator example
 * @details This example demonstrates the use of the Analog Comparator
 * 			to wake up the device from deep sleep.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>

#include "mxc_delay.h"
#include "mxc_errors.h"
#include "mxc_pins.h"
#include "nvic_table.h"
#include "adc.h"
#include "lp.h"
#include "board.h"
#include "pb.h"
#include "uart.h"

/***** Definitions *****/

/***** Globals *****/

extern mxc_uart_regs_t *ConsoleUart;

/***** Functions *****/

void my_isr(void)
{
    MXC_PWRSEQ->lppwkst |= MXC_F_PWRSEQ_LPPWKST_AINCOMP1;
}

int main(void)
{
    printf("\n********** Comparator Example **********\n");

    printf("\nConnect the positive comparator input to analog pin 7 (P0.15).\n");
    printf("Connect the negative comparator input to analog pin 3 (P0.11).\n");

    printf("\nThe device will repeatedly be placed in Deep Sleep and requires an edge\n");
    printf("transition of the comparator output to wakeup.\n\n");

    printf("Press any user push button to begin.\n");
    while (!PB_IsPressedAny()) {}

    // Configure Comparator as a low-power wakeup source
    MXC_LP_EnableComparatorWakeup(MXC_ADC_COMP_1);
    MXC_ADC_EnableComparator(MXC_ADC_COMP_1, MXC_ADC_CH_3, MXC_ADC_CH_7);

    // Enable Comparator interrupt
    MXC_NVIC_SetVector(GPIOWAKE_IRQn, my_isr);
    NVIC_EnableIRQ(GPIOWAKE_IRQn);
    __enable_irq();

    while (1) {
        printf("\nEntering sleep mode.\n");
        while (MXC_UART_GetActive(ConsoleUart)) {}

        // Go into Deep sleep and wait for comparator wakeup signal
        MXC_LP_EnterDeepSleepMode();
        printf("Waking up.\n");
    }

    return 0;
}
