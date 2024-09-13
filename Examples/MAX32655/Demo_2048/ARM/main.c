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
 * @brief   ARM Core portion of the 2048 Game.
 * @details 
 */

/* **** Includes **** */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// MSDK-provided Drivers
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_sys.h"
#include "sema.h"
#include "led.h"
#include "uart.h"
#include "rtc.h"

// Application Libraries
#include "graphics.h"

/* **** Definitions **** */

#if DEV_MODE_TRACE
#define PRINT(...) printf(__VA_ARGS__)
#else
// Don't print anything
#define PRINT(...)
#endif

// Match the Console's baud rate to what the controller will be set to
//  for the RISC-V as they share the same port.
#define RISCV_CONTROLLER_BAUD (2000000)

/// Semaphores
// Should never reach here
#if (MAILBOX_SIZE == 0)
#error "Mailbox sirrrze is 0."
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
//  Imagine like real mailboxes, owner (core) sends mail (keypress) in their mailbox. 
#define SEMA_RISCV_MAILBOX mxcSemaBox0
#define SEMA_ARM_MAILBOX mxcSemaBox1

static uint32_t ARM_GRID_COPY[4][4] = {0};

/* **** Functions **** */

// void SEMA_GetGrid(void)
// {
//     uint32_t *grid = (uint32_t *)(SEMA_RISCV_MAILBOX->payload);

//     for (int i = 0; i < 16; i++) {
//         ARM_MAIN_GRID[i/4][i%4] = grid[i];
//     }
// }

// *****************************************************************************
int main(void)
{
    int error;
    // The mailbox names are pre-defined, Creating new pointers and pointing them to
    //  memory locations for easier readability.
    // SEMA_RISCV_MAILBOX = mxcSemaBox0;
    // SEMA_ARM_MAILBOX = mxcSemaBox1;

    // System Initialization:
    //     - Use IPO for System Clock for fastest speed. (Done in SystemInit)
    //     - Enable Internal Cache. (Done in SystemInit)

    // Speed up console UART to match player controller baud rate which have shared ports 
    //  (PC Keyboard via Console UART).
    error = MXC_UART_Init(MXC_UART_GET_UART(CONSOLE_UART), RISCV_CONTROLLER_BAUD, MXC_UART_APB_CLK);
    if (error != E_NO_ERROR) {
        PRINT("ARM: Error speeding up baud rate: %d\n", error);
        LED_On(LED_RED);
        while(1);
    }

    PRINT("\n\n*******************************************************************************\n");
   
    // ARM Initialization.
    PRINT("ARM: Starting ARM Initialization.\n\n");

    // Enable ISO clock for RISC-V.
    MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_ISO);

    // Enable RISCV JTAG debugger.
    MXC_GPIO_Config(&gpio_cfg_rv_jtag);

    // Prepare ARM semaphore.
    MXC_SEMA_Init();
    
    // Check status of ARM semaphore.
    error = MXC_SEMA_CheckSema(SEMA_IDX_ARM);
    if (error != E_NO_ERROR) {
        PRINT("ARM: Semaphore for ARM core is busy: %d\n", error);
        LED_On(LED_RED);
        while(1);
    }

    error = MXC_SEMA_GetSema(SEMA_IDX_ARM);
    if (error != E_NO_ERROR) {
        PRINT("ARM: Semaphore is busy - with previous value: %d\n\n", MXC_SEMA->semaphores[SEMA_IDX_ARM]);
        LED_On(LED_RED);
        while(1);
    } else {
        PRINT("ARM: Semaphore is not busy - with previous value: %d\n\n", MXC_SEMA->semaphores[SEMA_IDX_ARM]);
    }

    // Backup Delay for 1 second before starting RISCV core.
    MXC_Delay(MXC_DELAY_SEC(1));

    PRINT("ARM: Starting RISC-V core and handing off major UART0 Control to RISC-V.\n\n");

    // Start the RISCV core.
    MXC_SYS_RISCVRun();

    // Wait the RISC-V core to finish startup (when it frees ARM semaphore).
    while (MXC_SEMA_CheckSema(SEMA_IDX_ARM) != E_NO_ERROR) {}
    MXC_SEMA_GetSema(SEMA_IDX_ARM);

    // Initialize mailboxes between ARM and RISCV cores.
    MXC_SEMA_InitBoxes();

    // Signal RISCV core to run.
    MXC_SEMA_FreeSema(SEMA_IDX_RISCV);

    // Initialize RTC.
    MXC_RTC_Init(0, 0);

    error = Graphics_Init();
    if (error != E_NO_ERROR) {
        PRINT("ARM: Error initializing graphics: %d\n", error);
        LED_On(LED_RED);
        while(1);
    }


    // for (int r = 1; r < 4; r++) {
    //     for (int c = 0; c < 4; c++) {
    //         Graphics_AddBlock(r, c, 2);
    //     }
    // }

    // Graphics_AddNewBlock(0, 1, 2);

    while (1) {
        // Ready to receive game input.

        // Wait for game to finish game logic.
        while (MXC_SEMA_CheckSema(SEMA_IDX_ARM) != E_NO_ERROR) {}

        MXC_SEMA_GetSema(SEMA_IDX_ARM);

        // for (int i = 0; i < 16*4; i++) {
        //     PRINT("%d - ", SEMA_RISCV_MAILBOX->payload[i]);
        // }

        PRINT("\n======\n");

        // for (int x = 0; x < MAILBOX_PAYLOAD_LEN; x++) {
        //     PRINT("ARM: %02x\n", SEMA_ARM_MAILBOX->payload[x]);
        // }
        int i = 0;
        for (int row = 0; row < 4; row++) {
            for (int col = 0; col < 4; col++) {
                ARM_GRID_COPY[row][col] = SEMA_ARM_MAILBOX->payload[i] << (8 * 0);
                ARM_GRID_COPY[row][col] += SEMA_ARM_MAILBOX->payload[i+1] << (8 * 1);
                ARM_GRID_COPY[row][col] += SEMA_ARM_MAILBOX->payload[i+2] << (8 * 2);
                ARM_GRID_COPY[row][col] += SEMA_ARM_MAILBOX->payload[i+3] << (8 * 3);

                PRINT("ARM: r:%d c:%d i:%d := %d - %02x %02x %02x %02x\n", row, col, i, ARM_GRID_COPY[row][col], SEMA_ARM_MAILBOX->payload[i], SEMA_ARM_MAILBOX->payload[i+1], SEMA_ARM_MAILBOX->payload[i+2], SEMA_ARM_MAILBOX->payload[i+3]);

                // ARM_GRID_COPY[row][col] = SEMA_RISCV_MAILBOX->payload[i] << (8 * 0);
                // ARM_GRID_COPY[row][col] += SEMA_RISCV_MAILBOX->payload[i+1] << (8 * 1);
                // ARM_GRID_COPY[row][col] += SEMA_RISCV_MAILBOX->payload[i+2] << (8 * 2);
                // ARM_GRID_COPY[row][col] += SEMA_RISCV_MAILBOX->payload[i+3] << (8 * 3);

                // PRINT("ARM: r:%d c:%d i:%d := %d - %02x %02x %02x %02x\n", row, col, i, ARM_GRID_COPY[row][col], SEMA_ARM_MAILBOX->payload[i], SEMA_ARM_MAILBOX->payload[i+1], SEMA_ARM_MAILBOX->payload[i+2], SEMA_ARM_MAILBOX->payload[i+3]);
                
                i+=4;
            }
        }

        PRINT("ARM: Direction Keypress: %c - 0x%02x\n", SEMA_ARM_MAILBOX->payload[4 * 16], SEMA_ARM_MAILBOX->payload[4 * 16]);

        MXC_SEMA_FreeSema(SEMA_IDX_RISCV);
        // Graphics_EraseSingleBlock(0, 1, GRAPHICS_SLIDE_DIR_LEFT);

        // Graphics_AddBlock(0, 0, 2);
    }
}
