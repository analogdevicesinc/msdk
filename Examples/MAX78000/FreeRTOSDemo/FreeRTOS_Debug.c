/********************************************************************************
 * Copyright (C) 2023 Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "tmr.h"
#include "nvic_table.h"

// Only include the contents of this file if DEBUG == 1
#if defined(DEBUG) && (DEBUG == 1)

// Scheduler tick is 1ms
// RTOS Stats Clock should be 10-100x faster
#define RTOS_STATS_TMR MXC_TMR0
#define RTOS_STATS_IRQn TMR0_IRQn
#define RTOS_STATS_TMR_SRC MXC_TMR_32K_CLK
#define RTOS_STATS_TMR_CNT 0xFFFFFFFF

/** NOTE:   
 * Based on this configuration, this timer will roll over after 2^32 / (TMR_SRC / prescaler) seconds. 
 * Because the RTOS tracks the total time, rollover events cannot be protected by the application. 
 * Thus, rollover events will affect statistics over long periods of measurement.  
 *  
 * EXAMPLE: 
 * With APB Clock as the source & 2048 prescale, this will be 2^32 / 32000 = 134,217 s. or ~93 days.
 * 
 * **/
void ConfigTimerForStats()
{
    mxc_tmr_cfg_t tmr;

    MXC_TMR_Shutdown(RTOS_STATS_TMR);

    tmr.pres = MXC_TMR_PRES_1;
    tmr.mode = TMR_MODE_CONTINUOUS;
    tmr.bitMode = MXC_TMR_BIT_MODE_32;
    tmr.clock = RTOS_STATS_TMR_SRC;
    tmr.cmp_cnt = RTOS_STATS_TMR_CNT; //SystemCoreClock*(1/interval_time);
    tmr.pol = 0;
    MXC_TMR_Init(RTOS_STATS_TMR, &tmr, true);

    MXC_TMR_Start(RTOS_STATS_TMR);
}

uint32_t GetTimerForStats()
{
    return MXC_TMR_GetCount(RTOS_STATS_TMR);
}

void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress)
{
    /* These are volatile to try and prevent the compiler/linker optimising them
    away as the variables never actually get used.  If the debugger won't show the
    values of the variables, make them global my moving their declaration outside
    of this function. */
    volatile uint32_t r0;
    volatile uint32_t r1;
    volatile uint32_t r2;
    volatile uint32_t r3;
    volatile uint32_t r12;
    volatile uint32_t lr; /* Link register. */
    volatile uint32_t pc; /* Program counter. */
    volatile uint32_t psr; /* Program status register. */

    r0 = pulFaultStackAddress[0];
    r1 = pulFaultStackAddress[1];
    r2 = pulFaultStackAddress[2];
    r3 = pulFaultStackAddress[3];

    r12 = pulFaultStackAddress[4];
    lr = pulFaultStackAddress[5];
    pc = pulFaultStackAddress[6];
    psr = pulFaultStackAddress[7];

    /* When the following line is hit, the variables contain the register values. */
    for (;;) {}
    for (;;) {}
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
    for (;;) {}
    for (;;) {}
}

/* The prototype shows it is a naked function - in effect this is just an
assembly function. */\
void HardFault_Handler(void) __attribute__((naked, aligned(8)));

/* The fault handler implementation calls a function called
prvGetRegistersFromStack(). */
void HardFault_Handler(void)
{
    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler2_address_const                            \n"
        " bx r2                                                     \n"
        " handler2_address_const: .word prvGetRegistersFromStack    \n");
}
#endif
