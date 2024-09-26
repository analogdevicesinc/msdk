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
#include "ipc_defines.h"

/***** Definitions *****/

#if DEV_MODE_TRACE
#define PRINT(...) printf(__VA_ARGS__)
#else
// Don't print anything
#define PRINT(...)
#endif

/// Controller Settings.
// Change UART speeds here, if needed. 115200 was more than enough, but users
//  have the option to increase the speed here.
// UART speed up is set at the beginning of BOTH ARM and RISC-V main code
//  because SystemInit for both cores default the UART baud rate to
//  115200.
#define CONTROLLER_UART_BAUD (115200)

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

uint32_t RISCV_GRID_COPY[4][4] = { 0 };
uint8_t RISCV_GRID_COPY_STATE[4][4] = { 0 };

uint32_t MOVES_COUNT = 0;

// Select Console UART instance.
mxc_uart_regs_t *CONTROLLER_UART = MXC_UART0;

/***** Functions *****/

void CONTROLLER_KEYPRESS_Callback(mxc_uart_req_t *req, int cb_error)
{
    int error;

    // No keypress if error detected.
    if (cb_error != E_NO_ERROR) {
        PRINT("RISCV: Error listening to keypress: %d\n", cb_error);
        PRINT("RISCV: Try again.\n");

        CONTROLLER_KEYPRESS = 0; // NULL character
        KEYPRESS_READY = false;

        MXC_UART_ClearRXFIFO(MXC_UART0);

        // Listen for next keypress.
        error = Controller_Start(&CONTROLLER_REQ);
        if (error != E_NO_ERROR) {
            PRINT("RISC-V: Error listening for next controller keypress: %d\n", error);
            LED_On(LED_RED);
        }

        return;
    }

    // User can add additional directional key switches here.
    switch (CONTROLLER_KEYPRESS) {
    case 'a':
        KEYPRESS_INPUT_DIR = INPUT_LEFT;
        KEYPRESS_READY = true;
        break;

    case 'd':
        KEYPRESS_INPUT_DIR = INPUT_RIGHT;
        KEYPRESS_READY = true;
        break;

    case 'w':
        KEYPRESS_INPUT_DIR = INPUT_UP;
        KEYPRESS_READY = true;
        break;

    case 's':
        KEYPRESS_INPUT_DIR = INPUT_DOWN;
        KEYPRESS_READY = true;
        break;

    default:
        KEYPRESS_READY = false;
        error = Controller_Start(&CONTROLLER_REQ);
        if (error != E_NO_ERROR) {
            PRINT("RISC-V: Invalid Keypress: %c\n", CONTROLLER_KEYPRESS);
        }
    }

    PRINT("RISC-V: Keypress: %c - 0x%02x Error: %d\n", CONTROLLER_KEYPRESS, CONTROLLER_KEYPRESS,
          cb_error);
}

// Must grab grid space before calling this function to have the latest
//  grid state.
void PRINT_GRID(void)
{
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

void PRINT_GRID_STATE(void)
{
    Game_2048_GetGridState(RISCV_GRID_COPY_STATE);

    // Imitate the grid is refreshing on terminal.
    PRINT("\n\n\n\n\n\n\n\n\n\n");

    for (int row = 0; row < 4; row++) {
        PRINT("        |        |        |        \n");

        for (int col = 0; col < 4; col++) {
            if (RISCV_GRID_COPY_STATE[row][col] != 0) {
                PRINT("   %02d   ", RISCV_GRID_COPY_STATE[row][col]);
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

// Must grab grid space before calling this function to have the latest
//  grid state.
void SendGridToARMCore(void)
{
    int i = MAILBOX_MAIN_GRID_IDX;
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            SEMA_ARM_MAILBOX->payload[i] = (RISCV_GRID_COPY[row][col] >> (8 * 0)) & 0xFF;
            SEMA_ARM_MAILBOX->payload[i + 1] = (RISCV_GRID_COPY[row][col] >> (8 * 1)) & 0xFF;
            SEMA_ARM_MAILBOX->payload[i + 2] = (RISCV_GRID_COPY[row][col] >> (8 * 2)) & 0xFF;
            SEMA_ARM_MAILBOX->payload[i + 3] = (RISCV_GRID_COPY[row][col] >> (8 * 3)) & 0xFF;

            i += 4;
        }
    }

    i = MAILBOX_MAIN_GRID_STATE_IDX;
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            SEMA_ARM_MAILBOX->payload[i] = (RISCV_GRID_COPY_STATE[row][col] >> (8 * 0)) & 0xFF;
            i++;
        }
    }
}

inline void SendKeypressToARMCore(void)
{
    SEMA_ARM_MAILBOX->payload[MAILBOX_KEYPRESS_IDX] = (CONTROLLER_KEYPRESS >> (8 * 0)) & 0xFF;
}

void SendNewBlockIndexToARMCore(uint8_t *new_block_idx, uint8_t did_block_move_or_is_init)
{
    SEMA_ARM_MAILBOX->payload[MAILBOX_IF_BLOCK_MOVED_IDX] = (did_block_move_or_is_init >> (8 * 0)) &
                                                            0xFF;
    SEMA_ARM_MAILBOX->payload[MAILBOX_NEW_BLOCK_LOCATION_IDX] = (*new_block_idx >> (8 * 0)) & 0xFF;
}

void SendGameStateToARMCore(game_state_t state)
{
    SEMA_ARM_MAILBOX->payload[MAILBOX_GAME_STATE_IDX] = (state >> (8 * 0)) & 0xFF;
}

void SendMovesCountToARMCore(uint32_t moves_count)
{
    SEMA_ARM_MAILBOX->payload[MAILBOX_MOVES_COUNT_IDX] = (moves_count >> (8 * 0)) & 0xFF;
    SEMA_ARM_MAILBOX->payload[MAILBOX_MOVES_COUNT_IDX + 1] = (moves_count >> (8 * 1)) & 0xFF;
    SEMA_ARM_MAILBOX->payload[MAILBOX_MOVES_COUNT_IDX + 2] = (moves_count >> (8 * 2)) & 0xFF;
    SEMA_ARM_MAILBOX->payload[MAILBOX_MOVES_COUNT_IDX + 3] = (moves_count >> (8 * 3)) & 0xFF;
}

// *****************************************************************************
int main(void)
{
    int error;
    int state;

    // Location of new block represented as an index for a 1-D array.
    //  (0-15) instead of represented as a coordinate (row, col) for
    //  easier and quicker transfer into mailbox.
    uint8_t new_block_idx_location = 0;

    // Speed up UART0 (Console) baud rate as the controller and console share the same port.
    //  Plus, Console UART gets reverted to default speed (115200) during SystemInit() during both
    //  ARM and RISC-V SystemInit().
    error = Controller_Init(CONTROLLER_UART, CONTROLLER_UART_BAUD);
    if (error != E_NO_ERROR) {
        PRINT("RISC-V: Error speeding up baud rate: %d\n", error);
        LED_On(LED_RED);
        while (1) {}
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
        while (1) {}
    }

    error = MXC_SEMA_GetSema(SEMA_IDX_RISCV);
    if (error != E_NO_ERROR) {
        PRINT("RISC-V: Semaphore is busy - with previous value: %d\n\n",
              MXC_SEMA->semaphores[SEMA_IDX_RISCV]);
        LED_On(LED_RED);
        while (1) {}
    } else {
        PRINT("RISC-V: Semaphore is not busy - with previous value: %d\n\n",
              MXC_SEMA->semaphores[SEMA_IDX_RISCV]);
    }

    // Initialize mailboxes between ARM and RISCV cores.
    MXC_SEMA_InitBoxes();

    // RISCV startup and mailbox initialization is finished. Signal ARM core to continue.
    PRINT("RISC-V: Finished startup. Main UART0 control is handled by RISC-V now.\n\n");
    MXC_SEMA_FreeSema(SEMA_IDX_ARM);

    PRINT("RISC-V: Starting Controller and Game\n");

    // Start Controller.
    error = Controller_Start(&CONTROLLER_REQ);
    if (error != E_NO_ERROR) {
        PRINT("RISC-V: Error starting the controller: %d\n", error);
        LED_On(LED_RED);
        while (1) {}
    }

    error = Game_2048_Init(&new_block_idx_location);
    if (error != E_NO_ERROR) {
        PRINT("RISC-V: Error starting game: %d\n", error);
        LED_On(LED_RED);
        while (1) {}
    }

    // Send starting grid to ARM core.
    // This function must be called before PRINT_GRID() and SendGridToARMCore()
    //  functions to grab the latest grid state.
    Game_2048_GetGrid(RISCV_GRID_COPY);

    PRINT_GRID();

    SendGridToARMCore();

    SendNewBlockIndexToARMCore(&new_block_idx_location, true);

    SendGameStateToARMCore(Game_2048_CheckState());

    // Signal ARM core to display initial grid.
    MXC_SEMA_FreeSema(SEMA_IDX_ARM);

    // Wait for ARM core to finish displaying the starting grid.
    while (MXC_SEMA_CheckSema(SEMA_IDX_RISCV) != E_NO_ERROR) {}

    while (1) {
        // Wait for keypress.
        while (KEYPRESS_READY == false) {}

        // Make sure the ARM core is finished displaying.
        while (MXC_SEMA_CheckSema(SEMA_IDX_RISCV) != E_NO_ERROR) {}
        MXC_SEMA_GetSema(SEMA_IDX_RISCV);

        input_direction_t dir = KEYPRESS_INPUT_DIR;

        state = Game_2048_UpdateGrid(dir, &new_block_idx_location);
        if (state == true) {
            PRINT("RISC-V: Blocks moved.\n");
            SendMovesCountToARMCore(++MOVES_COUNT);
        } else {
            PRINT("RISC-V: Blocks did not move.\n");
        }

        // Get the state of the game after finishing latest move.
        game_state_t game_state = Game_2048_CheckState();

        // These functions must be called before PRINT_GRID() and SendGridToARMCore()
        //  functions to grab the latest grid state.
        Game_2048_GetGrid(RISCV_GRID_COPY);
        Game_2048_GetGridState(RISCV_GRID_COPY_STATE);

        PRINT_GRID();

        SendGridToARMCore();

        SendKeypressToARMCore();

        SendGameStateToARMCore(game_state);

        SendNewBlockIndexToARMCore(&new_block_idx_location, state);

        KEYPRESS_READY = false;

        // Check if game is finished.
        if (game_state == WINNER) {
            // Signal ARM to finish final display update.
            MXC_SEMA_FreeSema(SEMA_IDX_ARM);

            PRINT("RISCV: Congratulations, you win!\n");
            PRINT("RISCV: Ending game.\n");

            while (1) {}
        } else if (game_state == GAME_OVER) {
            // Signal ARM to finish final display update.
            MXC_SEMA_FreeSema(SEMA_IDX_ARM);

            PRINT("RISCV: Game Over. Nice try! Better luck next time.\n");
            PRINT("RISCV: Ending game.\n");
            while (1) {}
        }

        // Listen for next keypress.
        MXC_UART_ClearRXFIFO(MXC_UART0);

        error = Controller_Start(&CONTROLLER_REQ);
        if (error != E_NO_ERROR) {
            PRINT("RISC-V: Error listening for next controller keypress: %d\n", error);
            LED_On(LED_RED);
            while (1) {}
        }

        // Signal ARM to update display.
        MXC_SEMA_FreeSema(SEMA_IDX_ARM);
    }
}
