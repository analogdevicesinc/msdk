/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
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
