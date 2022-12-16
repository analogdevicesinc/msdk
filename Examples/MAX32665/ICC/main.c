/**
 * @file    	main.c
 * @brief   	Instruction cache example
 * @details     Show the time difference when executing operations when instruction
 *              cache is enabled and disabled
 */

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

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "icc.h"
#include "rtc.h"
#include "tmr.h"

/***** Functions *****/

//Test function to do simple calculations
void example_func1(void)
{
    volatile int i, j, k;

    for (i = 0; i < 5000; i++) {
        for (j = 0; j < 500; j++) {
            k = i * j;
            if (((i % 500) == 0) && (j == 1)) {
                printf("%d%%,\t k=%d\n", i / 50, k);
            }
        }
    }
    printf("\n");

    return;
}

void example_func2(void)
{
    int i, j, k;

    for (i = 1; i <= 5000; i++) {
        for (j = 1; j <= 500; j++) {
            k = i * j;
            if (((i % 500) == 0) && (j == 1)) {
                printf("%d%%,\t k=%d\n", i / 50, k);
            }
        }
    }
    printf("\n");

    return;
}

//Start timer before test function
void start_timer(void)
{
    MXC_TMR_SW_Start(MXC_TMR0);
    return;
}

//Stop current timer and print elapsed time
int stop_timer(void)
{
    int time_elapsed = MXC_TMR_SW_Stop(MXC_TMR0);
    unsigned int sec = time_elapsed / 1000000;
    unsigned int mili = (time_elapsed - (sec * 1000000)) / 1000;
    unsigned int micro = time_elapsed - (sec * 1000000) - (mili * 1000);
    printf("Time Elapsed: %d.%d%d Seconds\n", sec, mili, micro);
    return time_elapsed;
}

// *****************************************************************************
int main(void)
{
    int fail = 0;
    int time_elapsed1 = 0;
    int time_elapsed2 = 0;

    printf("\n******** ICC Example ********\n");

    printf("\n***** Volatile  Example *****\n");

    printf("\nWith instruction cache enabled:\n");
    MXC_ICC_Enable();
    start_timer();
    example_func1(); //waste time
    time_elapsed1 = stop_timer();

    printf("\n\nWith instruction cache disabled:\n");
    MXC_ICC_Disable();
    start_timer();
    example_func1(); //waste time
    time_elapsed2 = stop_timer();

    if (time_elapsed2 <= time_elapsed1) {
        fail += 1;
    }

    MXC_ICC_Flush();
    printf("\n\n***** Non-volatile Example *****\n");

    printf("\nWith instruction cache enabled:\n");
    MXC_ICC_Enable();
    start_timer();
    example_func2(); //waste time
    time_elapsed1 = stop_timer();

    printf("\n\nWith instruction cache disabled:\n");
    MXC_ICC_Disable();
    start_timer();
    example_func2(); //waste time
    time_elapsed2 = stop_timer();

    if (time_elapsed2 <= time_elapsed1) {
        fail += 1;
    }

    printf("\n\n");
    if (fail == 0) {
        printf("EXAMPLE SUCCEEDED\n");
    } else {
        printf("EXAMPLE FAILED\n");
        return E_FAIL;
    }

    return E_NO_ERROR;
}
