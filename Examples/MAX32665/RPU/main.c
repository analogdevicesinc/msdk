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
 * @brief   Resource Protection Unit Example
 * @details This example uses the resource protection unit to block DMA from reading the
 *          key out of the crypto module during an operation. The attempt to read will result
 *          in a hardfault which ends the example
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc_errors.h"
#include "rpu.h"
#include "tmr.h"
#include "led.h"
#include "core1.h"

/***** Definitions *****/

/***** Globals *****/
//#if defined ( __GNUC__)
//    extern uint8_t __load_start_cpu1;
//#endif

/***** Functions *****/
void HardFault_Handler(void)
{
    printf("\n\nHard Fault reached\n");
    printf("Press reset to run the example again\n");
    printf("Example Complete\n");
    while (1) {}
}

int Core1_Main(void)
{
    int err;
    // The RPU defaults to all access enabled
    // We need to disallow everything we don't want to access
    // the RPU_Allow function can only be used to selectively
    // allow masters that have already been disallowed
    err = MXC_RPU_Disallow(MXC_RPU_TMR3,
                           (~MXC_RPU_SYS1_ALLOW) & 0x1FF); // Acquire exclusive access to TMR3

    if (err == E_BAD_STATE) {
        LED_On(0);
        while (1) {}
    } else if (err == E_BAD_PARAM) {
        while (1) {}
    }

    LED_On(1);

    while (1) {}
}

// *****************************************************************************
int main(void)
{
    printf("********* Resource Protection Unit Example **********\n");
    printf("This example uses the resource protection unit to prevent\n");
    printf("code running in Core 0 from accessing the timer in use by Core 1\n\n");

    LED_On(1);
    MXC_Delay(500000);
    LED_Off(1);

    Core1_Start();
    MXC_Delay(1000);

    // Try to read TMR3's config register
    uint32_t invalidaddr = MXC_TMR3->cn;

    printf("TMR3 Control Register: 0x%08x\n", invalidaddr);

    printf("Did not fault\n");
    while (1) {}
}
