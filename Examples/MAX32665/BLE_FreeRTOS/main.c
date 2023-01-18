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
 * @brief   BLE_FreeRTOS
 * @details This example demonstrates FreeRTOS with BLE capabilities.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "mxc_device.h"
#include "wut.h"
#include "lp.h"
#include "led.h"
#include "board.h"

/* Shadow register definitions */
#define MXC_R_SIR_SHR13 *((uint32_t *)(0x40005434))
#define MXC_R_SIR_SHR17 *((uint32_t *)(0x40005444))

/* Stringification macros */
#define STRING(x) STRING_(x)
#define STRING_(x) #x

extern void bleStartup(void);

/***** Functions *****/

/* =| vAssertCalled |==============================
 *
 *  Called when an assertion is detected. Use debugger to backtrace and 
 *  continue.
 *
 * =======================================================
 */
void vAssertCalled(const char *const pcFileName, uint32_t ulLine)
{
    volatile uint32_t ulSetToNonZeroInDebuggerToContinue = 0;

    /* Parameters are not used. */
    (void)ulLine;
    (void)pcFileName;

    __asm volatile("cpsid i");
    {
        /* You can step out of this function to debug the assertion by using
        the debugger to set ulSetToNonZeroInDebuggerToContinue to a non-zero
        value. */
        while (ulSetToNonZeroInDebuggerToContinue == 0) {}
    }
    __asm volatile("cpsie i");
}

/* =| vApplicationIdleHook |==============================
 *
 *  Call the user defined function from within the idle task.  This
 *  allows the application designer to add background functionality
 *  without the overhead of a separate task.
 *  NOTE: vApplicationIdleHook() MUST NOT, UNDER ANY CIRCUMSTANCES,
 *  CALL A FUNCTION THAT MIGHT BLOCK.
 *
 * =======================================================
 */
void vApplicationIdleHook(void)
{
    /* Sleep while idle */
    LED_Off(SLEEP_LED);

    MXC_LP_EnterSleepMode();

    LED_On(SLEEP_LED);
}

/* =| turnOffUnused |==========================
 *
 * Disable unused hardware to conserve power.
 *
 * =======================================================
 */
void turnOffUnused(void)
{
    /* Prevent SIMO leakage in DS by reducing the SIMO buck clock */
    if (MXC_GCR->revision == 0xA2) {
        MXC_R_SIR_SHR13 = 0x0;
        MXC_R_SIR_SHR17 &= ~(0xC0);
    } else if (MXC_GCR->revision == 0xA4) {
        MXC_R_SIR_SHR17 &= ~(0xC0);
    }

    MXC_LP_USBSWLPDisable();
}

/* =| main |==============================================
 *
 * This program demonstrates FreeRTOS tasks, mutexes.
 *
 * =======================================================
 */
int main(void)
{
    /* Print banner (RTOS scheduler not running) */
    printf("\n-=- %s BLE FreeRTOS (%s) Demo -=-\n", STRING(TARGET), tskKERNEL_VERSION_NUMBER);
#if configUSE_TICKLESS_IDLE
    printf("Tickless idle is enabled\n");
    /* Initialize CPU Active LED */
    LED_On(SLEEP_LED);
    LED_On(DEEPSLEEP_LED);
#endif
    printf("SystemCoreClock = %d\n", SystemCoreClock);

    /* Delay to prevent bricks */
    volatile int i;
    for (i = 0; i < 0x3FFFFF; i++) {}

    /* Turn off unused hardware to conserve power */
    turnOffUnused();

    /* Start the BLE application */
    bleStartup();

    /* Start scheduler */
    vTaskStartScheduler();

    /* This code is only reached if the scheduler failed to start */
    printf("ERROR: FreeRTOS did not start due to above error!\n");
    while (1) {
        __NOP();
    }

    /* Quiet GCC warnings */
    return -1;
}

typedef struct __attribute__((packed)) ContextStateFrame {
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t return_address;
    uint32_t xpsr;
} sContextStateFrame;

/*****************************************************************/
void HardFault_Handler(void)
{
    __asm(" TST LR, #4\n"
          " ITE EQ \n"
          " MRSEQ R0, MSP \n"
          " MRSNE R0, PSP \n"
          " B HardFault_Decoder \n");
}

/*****************************************************************/
/* Disable optimizations for this function so "frame" argument */
/* does not get optimized away */
__attribute__((optimize("O0"))) void HardFault_Decoder(sContextStateFrame *frame)
{
    /* Hang here */
    while (1) {}
}
