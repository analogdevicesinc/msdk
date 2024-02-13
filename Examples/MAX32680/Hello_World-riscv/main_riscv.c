/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "mxc.h"
#include "fcr_regs.h"
#include "sema_regs.h"
#include "board.h"

int main(void)
{
    int cnt = 0;

    Debug_Init(); // Set up RISCV JTAG
    MXC_ICC_Enable(MXC_ICC1); // Enable cache

    // Signal the Cortex-M4
    MXC_SEMA->irq0 = MXC_F_SEMA_IRQ0_EN | MXC_F_SEMA_IRQ0_CM4_IRQ;

    printf("Hello World!\n");
    while (1) {
        LED_On(0);
        MXC_Delay(500000);
        LED_Off(0);
        MXC_Delay(500000);
        printf("count = %d\n", cnt++);
    }

    return 0;
}
