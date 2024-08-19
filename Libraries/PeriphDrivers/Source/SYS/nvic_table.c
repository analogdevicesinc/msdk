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

#ifndef __riscv
#include <string.h>
#include "mxc_device.h"
#include "nvic_table.h"

#if !defined(NVIC_USER_IRQ_OFFSET)
#define NVIC_USER_IRQ_OFFSET 16 /**! Offset for device specific IRQs */
#endif

/* RAM vector_table needs to be aligned with the size of the vector table */
#if defined(__ICCARM__)
#pragma data_alignment = 512
#else
__attribute__((aligned(512)))
#endif
static void (*ramVectorTable[MXC_IRQ_COUNT])(void);

void NVIC_SetRAM(void)
{
#if defined(__ICCARM__)
    extern void (*const __isr_vector[])(void);
#else
    /* should be defined in starup_<device>.S */
    extern uint32_t __isr_vector[MXC_IRQ_COUNT];
#endif

    memcpy(&ramVectorTable, &__isr_vector, sizeof(ramVectorTable));
    SCB->VTOR = (uint32_t)&ramVectorTable;
}

void MXC_NVIC_SetVector(IRQn_Type irqn, void (*irq_handler)(void))
{
    int index = irqn + 16; /* offset for externals */

    /* If not copied, do copy */
    if (SCB->VTOR != (uint32_t)&ramVectorTable) {
        NVIC_SetRAM();
    }

    ramVectorTable[index] = irq_handler;
    NVIC_EnableIRQ(irqn);
}

uint32_t MXC_NVIC_GetVector(IRQn_Type irqn)
{
    uint32_t *vectors = (uint32_t *)SCB->VTOR;
    return vectors[(int32_t)irqn + NVIC_USER_IRQ_OFFSET];
}
#endif // !__riscv
