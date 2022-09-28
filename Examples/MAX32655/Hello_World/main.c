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
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "led.h"
#include "pb.h"
#include "board.h"
#include "mxc_delay.h"

#include "sema_reva.h"

/***** Definitions *****/
#define NDX_ARM         (0)
#define NDX_RISCV       (1)

/***** Globals *****/

/***** Functions *****/

// *****************************************************************************
int main(void)
{
    int cnt = 0;
    int ret;

    printf("\nRISC-V: start\n");
    
    // Signal SEMA(NDX_ARM)
    MXC_SEMA_FreeSema(NDX_ARM);
    printf("Singal SEMA(NDX_ARM) to start the output.\n");

    while (1) {
        // Wait
        ret = MXC_SEMA_CheckSema(NDX_RISCV);
        //printf("RISCV: CheckSema(1) returned %s.\n", ret == E_BUSY ? "BUSY" : "NOT BUSY");
        if (E_BUSY != ret) {  // Register is not busy.
            ret = MXC_SEMA_GetSema(NDX_RISCV);  // Reading the register does an atomic test and set, returns previous value.
            //printf("RISC-V: GetSema(1) returned %s with previous semaphors[1] value %d\n", ret == E_BUSY ? "BUSY" : "NOT BUSY", MXC_SEMA->semaphores[NDX_RISCV]);
            
            // Do the job
            printf("RISC-V: cnt=%d\n", cnt++);

            LED_On(LED_RED);
            MXC_Delay(500000);
            LED_Off(LED_RED);
            MXC_Delay(500000);
            
            // Signal
            MXC_SEMA_FreeSema(NDX_ARM);
        }

        MXC_Delay(500000);  // Void displaying CheckSema results too soon
    }    
}
