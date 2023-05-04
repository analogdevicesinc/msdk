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
 * @brief   External Clock 
 * @details This example demonstrates how to switch the system clock to the external
 *          clock input using the @ref mxc_sys drivers.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "led.h"
#include "board.h"
#include "mxc_delay.h"
#include "mxc_sys.h"

/***** Definitions *****/

/***** Globals *****/

/***** Functions *****/

// *****************************************************************************
int main(void)
{
    int count = 0;
    int err = E_NO_ERROR;

    MXC_Delay(MXC_DELAY_SEC(2)); // Provide a window for the debugger to connect

    // Print a 3,2,1... countdown before switching.  This validates that
    // the system is working normally off of the default system clock.
    printf("External Clock (EXT_CLK) example\n");
    printf("Switching to %i Hz external clock input in...\n", EXTCLK_FREQ);
    for (int i = 3; i > 0; i--) {
        printf("%i...\n", i);
        MXC_Delay(MXC_DELAY_SEC(1));
    }

    err = MXC_SYS_Clock_Select(MXC_SYS_CLOCK_EXTCLK);
    if (err) {
        printf("Failed to switch to external clock with error %i\n", err);
        LED_On(1);
        return err;
    }

    // Reinitialize the BSP.  This is necessary to recalculate clock divisors for
    // UART, etc. after switching to the external clock source.
    Board_Init();

    printf("Successfully switched to external clock (%i Hz)\n", EXTCLK_FREQ);
    // ^ Note:  EXTCLK_FREQ default value comes from system header file.  It is
    // overridden by defining it at compile-time in the build system.  See project.mk

    printf("Hello World!\n");

    while (count <= 10) {
        LED_On(0);
        MXC_Delay(500000);
        LED_Off(0);
        MXC_Delay(500000);
        printf("count = %d\n", count++);
    }

    // Switch back to the IPO.  This is done because leaving the
    // EVKIT running of the EXT_CLK could make SWD unreliable to reconnect/reflash.
    // RSTN is not driven by hardware, and the EXT_CLK signal could also be disconnected at any time
    printf("Success!  Example complete, switching back to IPO...\n");
    err = MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    Board_Init(); // Reinit BSP again for IPO
    printf("Back on IPO.  Done!\n");
    return E_SUCCESS;
}
