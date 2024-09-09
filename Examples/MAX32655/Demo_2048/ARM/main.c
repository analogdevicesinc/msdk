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

#include "trng.h"

// Application Libraries
#include "controller.h"
#include "game_2048.h"

/* **** Definitions **** */

#if DEV_MODE_TRACE
#define PRINT(...) printf(__VA_ARGS__)
#else
// Don't print anything
#define PRINT(...)
#endif

/// Controller Settings.
// Set to its fastest supported speed (3Mbps when tested).
#define CONTROLLER_UART_BAUD    (2000000)

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
#define SEMA_ARM_MAILBOX mxcSemaBox0
#define SEMA_RISCV_MAILBOX mxcSemaBox1

mxc_uart_req_t CONTROLLER_REQ;
uint8_t CONTROLLER_KEYPRESS;
volatile bool KEYPRESS_READY = false;
uint8_t KEYPRESS_INPUT_DIR;

uint32_t ARM_GRID_COPY[4][4] = {0};

// Select Console UART instance.
mxc_uart_regs_t *CONTROLLER_UART = MXC_UART0;

/* **** Functions **** */

void CONTROLLER_KEYPRESS_Callback(mxc_uart_req_t *req, int cb_error)
{
    int error;

    // Assume no keypress if error detected.
    if (cb_error != E_NO_ERROR) {
        CONTROLLER_KEYPRESS = 0; // NULL character
    }

    KEYPRESS_READY = true;

    // User can add additional directional key switches here.
    switch (CONTROLLER_KEYPRESS) {
        case 'a':
        case 0x44: // Tera term sends Character 'D' for LEFT arrow key. 
            KEYPRESS_INPUT_DIR = INPUT_LEFT;
            break;
        
        case 'd':
        case 0x43: // Tera term sends Character 'C' for RIGHT arrow key. 
            KEYPRESS_INPUT_DIR = INPUT_RIGHT;
            break;
        
        case 'w':
        case 0x41: // Tera term sends Character 'A' for UP arrow key. 
            KEYPRESS_INPUT_DIR = INPUT_UP;
            break;
        
        case 's':
        case 0x42: // Tera term sends Character 'B' for DOWN arrow key. 
            KEYPRESS_INPUT_DIR = INPUT_DOWN;
            break;
        
        default:
            KEYPRESS_READY = false;
    }
    
    // Due to request struct, CONTROLLER_KEYPRESS already contains the keypress character.
    // Send keypress to RISCV through mailbox 1.
    //  The mailbox is 32 bits wide, but the keypress is an ASCII character (8 bits).
    SEMA_ARM_MAILBOX->payload[0] = (CONTROLLER_KEYPRESS >> 8 * 0) & 0xFF;
    SEMA_ARM_MAILBOX->payload[1] = 0;
    SEMA_ARM_MAILBOX->payload[2] = 0;
    SEMA_ARM_MAILBOX->payload[3] = 0;

    PRINT("Keypress: %c - 0x%02x\n", CONTROLLER_KEYPRESS, CONTROLLER_KEYPRESS);

    // Listen for next keypress.
    error = Controller_Start(&CONTROLLER_REQ);
    if (error != E_NO_ERROR) {
        PRINT("Error listening for next controller keypress: %d\n", error);
        LED_On(LED_RED);
    }
}

void PRINT_GRID(void)
{
    Game_2048_GetGrid(ARM_GRID_COPY);

    // Imitate the grid is refreshing on terminal.
    PRINT("\n\n\n\n\n\n\n\n\n\n");

    for (int row = 0; row < 4; row++) {
        PRINT("        |        |        |        \n");

        for (int col = 0; col < 4; col++) {
            if (ARM_GRID_COPY[row][col] != 0) {
                PRINT("  %04d  ", ARM_GRID_COPY[row][col]);
            } else {
                PRINT("        ");
            }

            // Only print border 3 times.
            if (col < 3) {
                PRINT("|");
            }
        }

        PRINT("\n        |        |        |        \n");

        // Only print the row border 3 times.
        if (row < 3) {
            PRINT("-----------------------------------\n");
        }
    }
}

// *****************************************************************************
int main(void)
{
    int error;
    // The mailbox names are pre-defined, Creating new pointers and pointing them to
    //  memory locations for easier readability.
    // sema_arm_mailbox = mxcSemaBox0;
    // sema_riscv_mailbox = mxcSemaBox1;

    // System Initialization:
    //     - Use IPO for System Clock for fastest speed. (Done in SystemInit)
    //     - Enable Internal Cache. (Done in SystemInit)

    // Set up Controller Request Struct.
    CONTROLLER_REQ.uart = CONTROLLER_UART;
    CONTROLLER_REQ.txData = NULL;
    CONTROLLER_REQ.txLen = 0;
    CONTROLLER_REQ.rxData = &CONTROLLER_KEYPRESS;
    CONTROLLER_REQ.rxLen = 1; // Handle 1 keypress at a time
    CONTROLLER_REQ.callback = CONTROLLER_KEYPRESS_Callback;

    // Set up player controller (PC Keyboard via Console UART).
    error = Controller_Init(CONTROLLER_UART, CONTROLLER_UART_BAUD);
    if (error != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return error;
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

    // Start the RISCV core.
    MXC_SYS_RISCVRun();

    // Wait the RISC-V core to finish startup (when it frees ARM semaphore).
    PRINT("ARM: Waiting for RISC-V to finish startup process.\n\n");
    while (MXC_SEMA_CheckSema(SEMA_IDX_ARM) != E_NO_ERROR) {}
    MXC_SEMA_GetSema(SEMA_IDX_ARM);

    // Initialize mailboxes between ARM and RISCV cores.
    MXC_SEMA_InitBoxes();

    PRINT("ARM: Starting Controller and Game\n");

    // Start Controller.
    error = Controller_Start(&CONTROLLER_REQ);
    if (error != E_NO_ERROR) {
        PRINT("ARM: Error starting the controller: %d\n", error);
        LED_On(LED_RED);
        while(1);
    }

    error = Game_2048_Init();
    if (error != E_NO_ERROR) {
        PRINT("ARM: Error starting game: %d\n", error);
        LED_On(LED_RED);
        while(1);
    }

    // Game_2048_PrintGrid();
    PRINT_GRID();

    while (1) {
        // Wait for keypress.
    
        while (KEYPRESS_READY == false) {}

        input_direction_t dir = KEYPRESS_INPUT_DIR;
        
        error = Game_2048_UpdateGrid(dir);
        if (error == E_NONE_AVAIL) {
            PRINT("Game over!\n");
            LED_On(LED_GREEN);
            while(1);
        } else if (error != E_NO_ERROR) {
            PRINT("ARM: Error updating next move: %d\n", error);
            LED_On(LED_RED);
            while(1);
        }

        // Game_2048_PrintGrid();
        PRINT_GRID();

        // MXC_Delay(MXC_DELAY_SEC(1));
        KEYPRESS_READY = false;


    }


//     /* Signal RISC-V core to run */
//     printf("ARM   : Signal RISC-V.\n");
//     MXC_SEMA_FreeSema(NDX_RISCV);

//     uint32_t cnt;

//     /* Enter LPM */
//     while (1) {
// #if DUAL_CORE_SYNC
//         /* Wait */
//         ret = MXC_SEMA_CheckSema(NDX_ARM);
//         if (E_BUSY != ret) {
//             MXC_SEMA_GetSema(NDX_ARM);

//             /* Do the job. */
//             // Retrieve the data from the mailbox1
//             cnt = mxcSemaBox1->payload[0] << (8 * 0);
//             cnt += mxcSemaBox1->payload[1] << (8 * 1);
//             cnt += mxcSemaBox1->payload[2] << (8 * 2);
//             cnt += mxcSemaBox1->payload[3] << (8 * 3);

//             printf("ARM   : cnt=%d\n", cnt++);

//             mxcSemaBox0->payload[0] = (cnt >> 8 * 0) & 0xFF;
//             mxcSemaBox0->payload[1] = (cnt >> 8 * 1) & 0xFF;
//             mxcSemaBox0->payload[2] = (cnt >> 8 * 2) & 0xFF;
//             mxcSemaBox0->payload[3] = (cnt >> 8 * 3) & 0xFF;

//             /* Do other jobs here. */
//             MXC_Delay(MXC_DELAY_SEC(1));

//             /* Signal */
//             MXC_SEMA_FreeSema(NDX_RISCV);
//         }
// #endif
//    }
}
