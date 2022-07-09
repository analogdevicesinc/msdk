
/**
 * @file    	main.c
 * @brief   	Semaphore example
 * @details 	Press button to overwrite a global variable. If someone is already writing to it, deny the right
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
#include "mxc_device.h"
#include "mxc_errors.h"
#include "pb.h"
#include "sema.h"
#include "mxc_sys.h"

/***** Definitions *****/
#define SW1 0
#define SW2 1

/***** Globals *****/
volatile int A_active = 0;
volatile int B_active = 0;

int shared_variable = 0;

/***** Functions *****/
static void PB_AWrites(void* idx)
{
    int retval;
    //First check if A is already writing
    if (!A_active) {
        //Check if B is writing
        retval = MXC_SEMA_CheckSema(0);
        if (retval == E_NO_ERROR) {
            if ((MXC_SEMA_GetSema(0)) == E_NO_ERROR) {
                printf("A acquired semaphore!\n");
                printf("A started writing...\n");
                shared_variable = 100;
                printf("var = %d\n", shared_variable);
                A_active = 1;
            }
        }
        else if (retval == E_BUSY) {
            printf("A can't write right now.\n\n");
            printf("var = %d\n", shared_variable);
            return;
        }
    }
    else {
        A_active = !A_active;
        //Semaphore should be busy...
        retval = MXC_SEMA_CheckSema(0);
        if (retval == E_BUSY) {
            printf("A stopped writing.\n");
            MXC_SEMA_FreeSema(0);
            printf("A dropped the semaphore...\n");
        }else {
            printf("Something went wrong.\n");
            return;
        }
    }

    printf("\n");

    return;
}

static void PB_BWrites(void* idx) {
    int retval;
    //First check if B is already writing
    if (!B_active) {
        //Check if A is writing
        retval = MXC_SEMA_CheckSema(0);
        if (retval == E_NO_ERROR) {
            if ((MXC_SEMA_GetSema(0)) == E_NO_ERROR) {
                printf("B acquired semaphore!\n");
                printf("B started writing...\n");
                shared_variable = 200;
                printf("var = %d\n", shared_variable);
                B_active = 1;
            }
        }
        else if (retval == E_BUSY) {
            printf("B can't write right now.\n\n");
            printf("var = %d\n", shared_variable);
            return;
        }
    }
    else {
        B_active = !B_active;
        //Semaphore should be busy...
        retval = MXC_SEMA_CheckSema(0);
        if (retval == E_BUSY) {
            printf("B stopped writing.\n");
            MXC_SEMA_FreeSema(0);
            printf("B dropped the semaphore...\n");
        }
        else {
            printf("Something went wrong.\n");
            return;
        }
    }

    printf("\n");

    return;

}
// *****************************************************************************
int main(void)
{
    printf("***** Semaphore Example *****\n");
    printf("SW1 = A tries to write\n");
    printf("SW2 = B tries to write\n");

    printf("\n");

    MXC_SEMA_Init();

    if ((MXC_SEMA_GetSema(0)) == E_NO_ERROR) {
        printf("Semaphore acquired.\n");
    }

    if ((MXC_SEMA_CheckSema(0)) == E_NO_ERROR) {
        printf("Semaphore free.\n");
    }
    else {
        printf("Semaphore locked.\n");
    }

    MXC_SEMA_FreeSema(0);

    if ((MXC_SEMA_CheckSema(0)) == E_NO_ERROR) {
        printf("Semaphore free.\n");
    }
    else {
        printf("Semaphore locked.\n");
    }

    PB_RegisterCallback(SW1, PB_AWrites);
    PB_RegisterCallback(SW2, PB_BWrites);


    printf("\nExample running.\n");
    while(1);
}
