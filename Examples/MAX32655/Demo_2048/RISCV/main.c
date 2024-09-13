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

// Application Libraries
#include "controller.h"
#include "game_2048.h"

/***** Definitions *****/

#if DEV_MODE_TRACE
#define PRINT(...) printf(__VA_ARGS__)
#else
// Don't print anything
#define PRINT(...)
#endif

/// Controller Settings.
// Set to its fastest supported speed (3Mbps when tested).
// UART speed up is set at the beginning of BOTH ARM and RISC-V main code
//  because SystemInit for both cores default the UART baud rate to
//  115200. 
#define CONTROLLER_UART_BAUD    (2000000)

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
#define SEMA_RISCV_MAILBOX mxcSemaBox0
#define SEMA_ARM_MAILBOX mxcSemaBox1

mxc_uart_req_t CONTROLLER_REQ;
uint8_t CONTROLLER_KEYPRESS;
volatile bool KEYPRESS_READY = false;
uint8_t KEYPRESS_INPUT_DIR;

uint32_t RISCV_GRID_COPY[4][4] = {0};

// Select Console UART instance.
mxc_uart_regs_t *CONTROLLER_UART = MXC_UART0;

/***** Functions *****/

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
            KEYPRESS_INPUT_DIR = INPUT_LEFT;
            break;
        
        case 'd':
            KEYPRESS_INPUT_DIR = INPUT_RIGHT;
            break;
        
        case 'w':
            KEYPRESS_INPUT_DIR = INPUT_UP;
            break;
        
        case 's':
            KEYPRESS_INPUT_DIR = INPUT_DOWN;
            break;
        
        default:
            KEYPRESS_READY = false;
    }
    
    PRINT("RISC-V: Keypress: %c - 0x%02x Error: %d\n", CONTROLLER_KEYPRESS, CONTROLLER_KEYPRESS, cb_error);

    MXC_UART_ClearRXFIFO(MXC_UART0);

    // Listen for next keypress.
    error = Controller_Start(&CONTROLLER_REQ);
    if (error != E_NO_ERROR) {
        PRINT("RISC-V: Error listening for next controller keypress: %d\n", error);
        LED_On(LED_RED);
    }
}

void PRINT_GRID(void)
{
    Game_2048_GetGrid(RISCV_GRID_COPY);

    // Imitate the grid is refreshing on terminal.
    PRINT("\n\n\n\n\n\n\n\n\n\n");

    for (int row = 0; row < 4; row++) {
        PRINT("        |        |        |        \n");

        for (int col = 0; col < 4; col++) {
            if (RISCV_GRID_COPY[row][col] != 0) {
                PRINT("  %04d  ", RISCV_GRID_COPY[row][col]);
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

    // Speed up UART0 (Console) baud rate as the controller and console share the same port.
    //  Plus, Console UART gets reverted to default speed (115200) during SystemInit() during both
    //  ARM and RISC-V SystemInit().
    error = Controller_Init(CONTROLLER_UART, CONTROLLER_UART_BAUD);
    if (error != E_NO_ERROR) {
        PRINT("RISC-V: Error speeding up baud rate: %d\n", error);
        LED_On(LED_RED);
        while(1);
    }

    // Set up Controller Request Struct.
    CONTROLLER_REQ.uart = CONTROLLER_UART;
    CONTROLLER_REQ.txData = NULL;
    CONTROLLER_REQ.txLen = 0;
    CONTROLLER_REQ.rxData = &CONTROLLER_KEYPRESS;
    CONTROLLER_REQ.rxLen = 1; // Handle 1 keypress at a time
    CONTROLLER_REQ.callback = CONTROLLER_KEYPRESS_Callback;

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
    PRINT("RISC-V: Finished startup. Main UART0 control is handled by RISC-V now.\n\n");
    MXC_SEMA_FreeSema(SEMA_IDX_ARM);

    PRINT("RISC-V: Starting Controller and Game\n");

    // Start Controller.
    error = Controller_Start(&CONTROLLER_REQ);
    if (error != E_NO_ERROR) {
        PRINT("RISC-V: Error starting the controller: %d\n", error);
        LED_On(LED_RED);
        while(1);
    }

    error = Game_2048_Init();
    if (error != E_NO_ERROR) {
        PRINT("RISC-V: Error starting game: %d\n", error);
        LED_On(LED_RED);
        while(1);
    }

    // Game_2048_PrintGrid();
    PRINT_GRID();

    while (1) {
        // Wait for keypress.
    
        while (KEYPRESS_READY == false) {}

        MXC_SEMA_GetSema(SEMA_IDX_RISCV);

        input_direction_t dir = KEYPRESS_INPUT_DIR;
        
        error = Game_2048_UpdateGrid(dir);
        if (error == E_NONE_AVAIL) {
            PRINT("Game over!\n");
            LED_On(LED_GREEN);
            while(1);
        } else if (error != E_NO_ERROR) {
            PRINT("RISC-V: Error updating next move: %d\n", error);
            LED_On(LED_RED);
            while(1);
        }

        // Game_2048_PrintGrid();
        PRINT_GRID();

        // Game_2048_GetGrid(RISCV_GRID_COPY);

        // Send updated grid and keypress to RISCV through mailbox 1.
        // Game_2048_GetGridMailBox(SEMA_ARM_WR_MAILBOX->payload);

        // for (int x = 0; x < MAILBOX_PAYLOAD_LEN; x++) {
        //     PRINT("RISCV: %02x\n", SEMA_ARM_MAILBOX->payload[x]);
        // }

        int i = 0;
        for (int row = 0; row < 4; row++) {
            for (int col = 0; col < 4; col++) {
                SEMA_ARM_MAILBOX->payload[i] = (RISCV_GRID_COPY[row][col] >> (8 * 0)) & 0xFF;
                SEMA_ARM_MAILBOX->payload[i+1] = (RISCV_GRID_COPY[row][col] >> (8 * 1)) & 0xFF;
                SEMA_ARM_MAILBOX->payload[i+2] = (RISCV_GRID_COPY[row][col] >> (8 * 2)) & 0xFF;
                SEMA_ARM_MAILBOX->payload[i+3] = (RISCV_GRID_COPY[row][col] >> (8 * 3)) & 0xFF;

                PRINT("RISCV: r:%d c:%d i:%d := %d - %02x %02x %02x %02x\n", row, col, i, RISCV_GRID_COPY[row][col], SEMA_ARM_MAILBOX->payload[i], SEMA_ARM_MAILBOX->payload[i+1], SEMA_ARM_MAILBOX->payload[i+2], SEMA_ARM_MAILBOX->payload[i+3]);
                i+=4;
            }
        }

        SEMA_ARM_MAILBOX->payload[4 * 16] = (CONTROLLER_KEYPRESS >> (8 * 0)) & 0xFF;

        MXC_Delay(500000);

        // MXC_Delay(MXC_DELAY_SEC(1));
        KEYPRESS_READY = false;

        MXC_SEMA_FreeSema(SEMA_IDX_ARM);
    }
}
