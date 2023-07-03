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
