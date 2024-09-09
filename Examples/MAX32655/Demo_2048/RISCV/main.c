/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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
 * @brief   RISCV Portion of the 2048 Game.
 * @details 
 */

/***** Includes *****/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// MSDK-provided Drivers
#include "mxc_delay.h"
#include "mxc_device.h"
#include "board.h"
#include "led.h"
#include "pb.h"
#include "sema.h"
#include "rtc.h"
#include "tft_ssd2119.h"
#include "tsc2046.h"

// // Application Libraries
// #include "utils.h"
// #include "state.h"
// #include "bitmap.h"
// #include "keypad.h"

/***** Definitions *****/

#if DEV_MODE_TRACE
#define PRINT(...) printf(__VA_ARGS__)
#else
// Don't print anything
#define PRINT(...)
#endif


/// Semaphores
// Should never reach here
#if (MAILBOX_SIZE == 0)
#error "Mailbox size is 0."
#endif

// Keep track for Semaphore peripheral.
#define SEMA_IDX_ARM (0)
#define SEMA_IDX_RISCV (1)

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

/* **** Globals **** */
// Defined in sema_reva.c
extern mxcSemaBox_t *mxcSemaBox0; // ARM writes, RISCV reads
extern mxcSemaBox_t *mxcSemaBox1; // ARM reads,  RISCV writes

// Rename boxes for readability.
#define SEMA_ARM_MAILBOX mxcSemaBox0
#define SEMA_RISCV_MAILBOX mxcSemaBox1

/***** Functions *****/

// *****************************************************************************
int main(void)
{
    int error;

    // NOTE: Printing to terminal is done on UART0 which both the ARM and RISC-V core must share.
    //       Must be mindful when to use PRINT (printf) for RISC-V side.
    PRINT("RISC-V: Starting RISC-V Initialization.\n\n");

    MXC_SEMA_Init();

    error = MXC_SEMA_CheckSema(SEMA_IDX_RISCV);
    if (error != E_NO_ERROR) {
        PRINT("RISC-V: Semaphore for RISC-V core is busy: %d\n", error);
        LED_On(LED_GREEN);
        while(1);
    }

    error = MXC_SEMA_GetSema(SEMA_IDX_RISCV);
    if (error != E_NO_ERROR) {
        PRINT("RISC-V: Semaphore is busy - with previous value: %d\n\n", MXC_SEMA->semaphores[SEMA_IDX_RISCV]);
        LED_On(LED_RED);
        while(1);
    } else {
        PRINT("RISC-V: Semaphore is not busy - with previous value: %d\n\n", MXC_SEMA->semaphores[SEMA_IDX_RISCV]);
    }

    // Initialize mailboxes between ARM and RISCV cores.
    MXC_SEMA_InitBoxes();

    // RISC-V startup finish startup and initializing mailboxes. Signal ARM to continue.
    PRINT("RISC-V: Finished startup. Handing off major UART0 control to ARM.\n\n");
    MXC_SEMA_FreeSema(SEMA_IDX_ARM);

    // Initialize RTC
    MXC_RTC_Init(0, 0);
    MXC_RTC_Start();

    // // TFT Pre-Init done during ARM startup in SystemInit().
    // MXC_TFT_Init();

    // MXC_TFT_SetBackGroundColor(0);
    // LED_On(1);
    while(1) {}
    // /* Initialize Touch Screen controller */
    // MXC_TS_Init();
    // MXC_TS_Start();

    // /* Display Home page */
    // state_init();

    // /* Get current time */
    // start_time = utils_get_time_ms();

    // while (1) {
    //     /* Get current screen state */
    //     state = state_get_current();

    //     /* Check pressed touch screen key */
    //     key = MXC_TS_GetKey();

    //     if (key > 0) {
    //         state->prcss_key(key);
    //         start_time = utils_get_time_ms();
    //     }

    //     /* Check timer tick */
    //     if (utils_get_time_ms() >= (start_time + state->timeout)) {
    //         if (state->tick) {
    //             state->tick();
    //             start_time = utils_get_time_ms();
    //         }
    //     }
    // }


//     printf("\nRISC-V: Start.\n");

// #if DUAL_CORE_SYNC
//     MXC_SEMA_Init();

//     int ret = MXC_SEMA_CheckSema(NDX_RISCV);
//     printf("RISC-V: After init, CheckSema(%d) returned %s.\n", NDX_RISCV,
//            ret == E_BUSY ? "BUSY" : "NOT BUSY");

//     if ((MXC_SEMA_GetSema(NDX_RISCV)) == E_NO_ERROR) {
//         printf("RISC-V: GetSema returned NOT BUSY with previous semaphore value %d.\n",
//                MXC_SEMA->semaphores[NDX_RISCV]);
//     } else {
//         printf("RISC-V: GetSema returned - BUSY - with previous semaphore value %d.\n",
//                MXC_SEMA->semaphores[NDX_RISCV]);
//     }

//     /* Init code here. */
//     printf("RISC-V: Do initialization works here.\n");
//     MXC_SEMA_InitBoxes();

//     /* Signal ARM core to run. */
//     printf("RISC-V: Signal ARM.\n");
//     MXC_SEMA_FreeSema(NDX_ARM);
// #endif

//     uint32_t cnt = 0;

//     /* Enter LPM */
//     while (1) {
// #if DUAL_CORE_SYNC
//         /* Wait */
//         int ret = MXC_SEMA_CheckSema(NDX_RISCV);
//         if (E_BUSY != ret) {
//             MXC_SEMA_GetSema(NDX_RISCV);

//             /* Do the job */
//             // Retrieve the data from the mailbox0
//             cnt = mxcSemaBox0->payload[0] << (8 * 0);
//             cnt += mxcSemaBox0->payload[1] << (8 * 1);
//             cnt += mxcSemaBox0->payload[2] << (8 * 2);
//             cnt += mxcSemaBox0->payload[3] << (8 * 3);

//             printf("RISC-V: cnt=%d\n", cnt++);
// #else
//         printf("count = %d\n", cnt++);
// #endif

// #if DUAL_CORE_SYNC
//             mxcSemaBox1->payload[0] = (cnt >> 8 * 0) & 0xFF;
//             mxcSemaBox1->payload[1] = (cnt >> 8 * 1) & 0xFF;
//             mxcSemaBox1->payload[2] = (cnt >> 8 * 2) & 0xFF;
//             mxcSemaBox1->payload[3] = (cnt >> 8 * 3) & 0xFF;

//             /* Do other jobs here */
// #endif
//             LED_On(LED_RED);
//             MXC_Delay(500000);
//             LED_Off(LED_RED);
//             MXC_Delay(500000);
// #if DUAL_CORE_SYNC
//             /* Signal */
//             MXC_SEMA_FreeSema(NDX_ARM);
//         }
// #endif
//     }
}
