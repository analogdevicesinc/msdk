/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2026 Analog Devices, Inc.
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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "mxc_device.h"
#include "mxc_sys.h"
#include "gcr_regs.h"

extern void (*const __isr_vector_core1[])(void);

void Start_Core1(void)
{
    // Save Core 1 vector table location in GCR.
    MXC_GCR->gp0 = (uint32_t)&__isr_vector_core1;
    MXC_GCR->perckcn1 &= ~MXC_F_GCR_PERCKCN1_CPU1D;
}

void Stop_Core1(void)
{
    MXC_GCR->perckcn1 |= MXC_F_GCR_PERCKCN1_CPU1D;
}

/**
 * The user declares this in application code.
 */
__weak int main_core1(void)
{
    while (1) {}
}

/**
 * You may override this function in your program by defining a custom 
 *  PreInit_Core1().
 */
__weak void PreInit_Core1(void)
{
    return;
}

__weak int PeripheralInit_Core1(void)
{
    /* Do nothing */
    return E_NO_ERROR;
}

/**
 * This function is called just before control is transferred to main()
 *  on Core 1.
 *
 * You may override this function in your program by defining a custom 
 *  SystemInit(), but care should be taken to reproduce the initialization
 *  steps or a non-functional system may result.
 */
__weak void SystemInit_Core1(void)
{
    /**
     * Configure the interrupt controller to use the application vector 
     *  table in flash. Initially, VTOR points to the ROM's table.
     */
    SCB->VTOR = (uint32_t)&__isr_vector_core1;

    /**
     * Enable FPU on Cortex-M4, which occupies coprocessor slots 10 & 11
     *  Grant full access, per "Table B3-24 CPACR bit assignments".
     *  DDI0403D "ARMv7-M Architecture Reference Manual"
     */
    SCB->CPACR |= SCB_CPACR_CP10_Msk | SCB_CPACR_CP11_Msk;
    __DSB();
    __ISB();

    PeripheralInit_Core1();
}
