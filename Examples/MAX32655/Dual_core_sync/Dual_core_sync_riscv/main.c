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
 * @file    main.c
 * @brief   Dual-core synchronization demo code for RISC-V core.
 * @details This example uses the UART to print to a terminal and flashes an
 * LED.
 */

/***** Includes *****/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "mxc_delay.h"
#include "mxc_device.h"
#include "board.h"
#include "led.h"
#include "pb.h"
#include "sema.h"

/***** Definitions *****/
#define DUAL_CORE_SYNC (1)

#define NDX_ARM (0)
#define NDX_RISCV (1)

#define MAILBOX_OVERHEAD (2 * sizeof(uint16_t))
#define MAILBOX_PAYLOAD_LEN (MAILBOX_SIZE - MAILBOX_OVERHEAD)
typedef struct {
    uint16_t readLocation;
    uint16_t writeLocation;
#if (MAILBOX_SIZE == 0)
    uint8_t payload[1];
#else
    uint8_t payload[MAILBOX_PAYLOAD_LEN];
#endif
} mxcSemaBox_t;

/***** Globals *****/
extern mxcSemaBox_t *mxcSemaBox0; // ARM writes, RISCV reads,
extern mxcSemaBox_t *mxcSemaBox1; // ARM reads,  RISCV writes

/***** Functions *****/

// *****************************************************************************
int main(void)
{
#if DUAL_CORE_SYNC
    printf("\nRISC-V: Start.\n");
#else
    printf("Start!\n");
#endif

#if DUAL_CORE_SYNC
    MXC_SEMA_Init();

    int ret = MXC_SEMA_CheckSema(NDX_RISCV);
    printf("RISC-V: After init, CheckSema(%d) returned %s.\n", NDX_RISCV,
           ret == E_BUSY ? "BUSY" : "NOT BUSY");

    if ((MXC_SEMA_GetSema(NDX_RISCV)) == E_NO_ERROR) {
        printf("RISC-V: GetSema returned NOT BUSY with previous semaphore value %d.\n",
               MXC_SEMA->semaphores[NDX_RISCV]);
    } else {
        printf("RISC-V: GetSema returned - BUSY - with previous semaphore value %d.\n",
               MXC_SEMA->semaphores[NDX_RISCV]);
    }

    /* Init code here. */
    printf("RISC-V: Do initialization works here.\n");
    MXC_SEMA_InitBoxes();

    /* Signal ARM core to run. */
    printf("RISC-V: Signal ARM.\n");
    MXC_SEMA_FreeSema(NDX_ARM);
#endif

    uint32_t cnt = 0;

    /* Enter LPM */
    while (1) {
#if DUAL_CORE_SYNC
        /* Wait */
        int ret = MXC_SEMA_CheckSema(NDX_RISCV);
        if (E_BUSY != ret) {
            MXC_SEMA_GetSema(NDX_RISCV);

            /* Do the job */
            // Retrieve the data from the mailbox0
            cnt = mxcSemaBox0->payload[0] << (8 * 0);
            cnt += mxcSemaBox0->payload[1] << (8 * 1);
            cnt += mxcSemaBox0->payload[2] << (8 * 2);
            cnt += mxcSemaBox0->payload[3] << (8 * 3);

            printf("RISC-V: cnt=%d\n", cnt++);
#else
        printf("count = %d\n", cnt++);
#endif

#if DUAL_CORE_SYNC
            mxcSemaBox1->payload[0] = (cnt >> 8 * 0) & 0xFF;
            mxcSemaBox1->payload[1] = (cnt >> 8 * 1) & 0xFF;
            mxcSemaBox1->payload[2] = (cnt >> 8 * 2) & 0xFF;
            mxcSemaBox1->payload[3] = (cnt >> 8 * 3) & 0xFF;

            /* Do other jobs here */
#endif
            LED_On(LED_RED);
            MXC_Delay(500000);
            LED_Off(LED_RED);
            MXC_Delay(500000);
#if DUAL_CORE_SYNC
            /* Signal */
            MXC_SEMA_FreeSema(NDX_ARM);
        }
#endif
    }
}
