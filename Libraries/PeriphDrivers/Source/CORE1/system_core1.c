/*******************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
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
 * $Date: 2018-08-10 17:11:51 -0500 (Fri, 10 Aug 2018) $
 * $Revision: 36872 $
 *
 ******************************************************************************/

/**
 * @file    core1startup.c
 * @brief   Startup Code for MAX32665 Family CPU1
 * @details These functions are called at the startup of the second ARM core (CPU1/Core1)
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "max32665.h"
#include "mxc_sys.h"
#include "gcr_regs.h"
#include "icc_regs.h"
#include "pwrseq_regs.h"

extern uint32_t __isr_vector_core1;

void Core1_Start(void)
{
    MXC_GCR->gp0 = (uint32_t)(&__isr_vector_core1);
    MXC_GCR->perckcn1 &= ~MXC_F_GCR_PERCKCN1_CPU1D;
}

void Core1_Stop(void)
{
    MXC_GCR->perckcn1 |= MXC_F_GCR_PERCKCN1_CPU1D;
}

__weak int Core1_Main(void)
{
    // The user should declare this in application code, so we'll just spin
    while (1) {}
}
__weak void PreInit_Core1(void)
{
    return;
}

__weak void SystemInit_Core1(void)
{
    /* Configure the interrupt controller to use the application vector table in
     * the application space */
    SCB->VTOR = (uint32_t)&__isr_vector_core1;

    /* Enable FPU on Cortex-M4, which occupies coprocessor slots 10 & 11
     * Grant full access, per "Table B3-24 CPACR bit assignments".
     * DDI0403D "ARMv7-M Architecture Reference Manual" */
    SCB->CPACR |= SCB_CPACR_CP10_Msk | SCB_CPACR_CP11_Msk;
    __DSB();
    __ISB();

    // Enable ICache1 Clock
    MXC_GCR->perckcn1 &= ~(1 << 22);

    // Invalidate cache and wait until ready
    MXC_ICC1->invalidate = 1;
    while (!(MXC_ICC1->cache_ctrl & MXC_F_ICC_CACHE_CTRL_RDY)) {}

    // Enable Cache
    MXC_ICC1->cache_ctrl |= MXC_F_ICC_CACHE_CTRL_EN;
    while (!(MXC_ICC1->cache_ctrl & MXC_F_ICC_CACHE_CTRL_RDY)) {}
}
