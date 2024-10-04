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
#include "ipc_defines.h"

/* **** Definitions **** */

#if DEV_MODE_TRACE
#define PRINT(...) printf(__VA_ARGS__)
#else
// Don't print anything
#define PRINT(...)
#endif

// Match the Console's baud rate to what the controller will be set to
//  for the RISC-V as they share the same port.
#define RISCV_CONTROLLER_BAUD (115200)

/* **** Globals **** */
// Defined in sema_reva.c
extern mxcSemaBox_t *mxcSemaBox0; // ARM writes, RISCV reads
extern mxcSemaBox_t *mxcSemaBox1; // ARM reads,  RISCV writes

// Rename boxes for readability.
//  Imagine like real mailboxes, owner (core) sends mail (keypress) in their mailbox.
#define SEMA_RISCV_MAILBOX mxcSemaBox0
#define SEMA_ARM_MAILBOX mxcSemaBox1

uint32_t ARM_GRID_COPY[4][4] = { 0 };
block_state_t ARM_GRID_COPY_STATE[4][4] = { 0 };

uint32_t MOVES_COUNT = 0;

/* **** Functions **** */

void ReceiveGridFromRISCVCore(void)
{
    int i = MAILBOX_MAIN_GRID_IDX;

    // Grid state.
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            ARM_GRID_COPY[row][col] = SEMA_ARM_MAILBOX->payload[i] << (8 * 0);
            ARM_GRID_COPY[row][col] += SEMA_ARM_MAILBOX->payload[i + 1] << (8 * 1);
            ARM_GRID_COPY[row][col] += SEMA_ARM_MAILBOX->payload[i + 2] << (8 * 2);
            ARM_GRID_COPY[row][col] += SEMA_ARM_MAILBOX->payload[i + 3] << (8 * 3);

            i += 4;
        }
    }

    // Grid status (unmoved/delete/combine).
    i = MAILBOX_MAIN_GRID_STATE_IDX;
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            ARM_GRID_COPY_STATE[row][col] = SEMA_ARM_MAILBOX->payload[i] << (8 * 0);

            i++;
        }
    }
}

graphics_slide_direction_t ReceiveDirectionFromRISCVCore(void)
{
    // Add more direction keys here.
    switch (SEMA_ARM_MAILBOX->payload[MAILBOX_KEYPRESS_IDX]) {
    // UP
    case 'w':
        return GRAPHICS_SLIDE_DIR_UP;

    // DOWN
    case 's':
        return GRAPHICS_SLIDE_DIR_DOWN;

    // LEFT
    case 'a':
        return GRAPHICS_SLIDE_DIR_LEFT;

    // RIGHT
    case 'd':
        return GRAPHICS_SLIDE_DIR_RIGHT;

    default:
        return -1;
    }
}

bool ReceiveNewBlockLocationFromRISCVCore(uint16_t *row, uint16_t *col)
{
    uint8_t new_block_added = SEMA_ARM_MAILBOX->payload[MAILBOX_IF_BLOCK_MOVED_IDX];
    if (new_block_added) { // true
        *row = (uint16_t)(SEMA_ARM_MAILBOX->payload[MAILBOX_NEW_BLOCK_LOCATION_IDX] / 4);
        *col = (uint16_t)(SEMA_ARM_MAILBOX->payload[MAILBOX_NEW_BLOCK_LOCATION_IDX] % 4);
    } else {
        *row = 0xFFFF;
        *col = 0xFFFF;
    }

    return new_block_added;
}

game_state_t ReceiveGameStateFromRISCVCore(void)
{
    return SEMA_ARM_MAILBOX->payload[MAILBOX_GAME_STATE_IDX];
}

uint32_t ReceiveMovesCountFromRISCVCore(void)
{
    uint32_t moves_count = 0;
    moves_count = SEMA_ARM_MAILBOX->payload[MAILBOX_MOVES_COUNT_IDX] << (8 * 0);
    moves_count += SEMA_ARM_MAILBOX->payload[MAILBOX_MOVES_COUNT_IDX + 1] << (8 * 1);
    moves_count += SEMA_ARM_MAILBOX->payload[MAILBOX_MOVES_COUNT_IDX + 2] << (8 * 2);
    moves_count += SEMA_ARM_MAILBOX->payload[MAILBOX_MOVES_COUNT_IDX + 3] << (8 * 3);
    return moves_count;
}

// *****************************************************************************
int main(void)
{
    int error;
    uint16_t new_block_row = 0xFFFF, new_block_col = 0xFFFF;
    graphics_slide_direction_t direction;
    game_state_t game_state;

    // System Initialization:
    //     - Use IPO for System Clock for fastest speed. (Done in SystemInit)
    //     - Enable Internal Cache. (Done in SystemInit)

    // Speed up console UART to match player controller baud rate which have shared ports
    //  (PC Keyboard via Console UART).
    error = MXC_UART_Init(MXC_UART_GET_UART(CONSOLE_UART), RISCV_CONTROLLER_BAUD, MXC_UART_APB_CLK);
    if (error != E_NO_ERROR) {
        PRINT("ARM: Error speeding up baud rate: %d\n", error);
        LED_On(LED_RED);
        while (1) {}
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
        while (1) {}
    }

    error = MXC_SEMA_GetSema(SEMA_IDX_ARM);
    if (error != E_NO_ERROR) {
        PRINT("ARM: Semaphore is busy - with previous value: %d\n\n",
              MXC_SEMA->semaphores[SEMA_IDX_ARM]);
        LED_On(LED_RED);
        while (1) {}
    } else {
        PRINT("ARM: Semaphore is not busy - with previous value: %d\n\n",
              MXC_SEMA->semaphores[SEMA_IDX_ARM]);
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
    if (MXC_RTC_Init(0, 0) != E_NO_ERROR) {
        PRINT("ARM: Error initializing RTC: %d\n", error);
        LED_On(LED_RED);
        while (1) {}
    }

    error = Graphics_Init();
    if (error != E_NO_ERROR) {
        PRINT("ARM: Error initializing graphics: %d\n", error);
        LED_On(LED_RED);
        while (1) {}
    }

    // Wait for Game logic to finish initializing on RISCV.
    while (MXC_SEMA_CheckSema(SEMA_IDX_ARM) != E_NO_ERROR) {}
    MXC_SEMA_GetSema(SEMA_IDX_ARM);

    // Get starting grid to keep ARM and RISCV grid copies in sync.
    ReceiveGridFromRISCVCore();

    ReceiveNewBlockLocationFromRISCVCore(&new_block_row, &new_block_col);

    game_state = ReceiveGameStateFromRISCVCore();
    if (game_state != IN_PROGRESS) {
        PRINT("ARM: Error starting game.\n");
        LED_On(LED_RED);
        while (1) {}
    }

    Graphics_AddNewBlock(new_block_row, new_block_col, ARM_GRID_COPY[new_block_row][new_block_col]);

    // Signal RISC-V to start waiting for keypresses.
    MXC_SEMA_FreeSema(SEMA_IDX_RISCV);

    // Start timer.
    if (MXC_RTC_Start() != E_NO_ERROR) {
        PRINT("ARM: Error starting timer: %d\n", error);
        LED_On(LED_RED);
        while (1) {}
    }

    uint32_t prev_seconds = 0;
    uint32_t seconds = 0;
    int prev_moves_count = MOVES_COUNT; // Should start as 0.
    while (1) {
        // Update timer.
        MXC_RTC_GetSeconds(&seconds);
        if (prev_seconds != seconds) {
            Graphics_SetTime(seconds);
            prev_seconds = seconds;
        }

        // Update grid when RISCV signals ARM it's ready.
        if (MXC_SEMA_CheckSema(SEMA_IDX_ARM) == E_NO_ERROR) {
            MXC_SEMA_GetSema(SEMA_IDX_ARM);

            // Get the updated grid then display.
            ReceiveGridFromRISCVCore();

            direction = ReceiveDirectionFromRISCVCore();

            // Erase blocks that are moving before drawing theem at their final location.
            Graphics_EraseBlocks(ARM_GRID_COPY_STATE, direction);

            // Pre-set these values as invalid locations.
            new_block_row = 0xFFFF, new_block_col = 0xFFFF;
            bool new_block_added;
            // If blocks moved, Add new block after all the grid updated.
            new_block_added = ReceiveNewBlockLocationFromRISCVCore(&new_block_row, &new_block_col);

            // Increment moves counter if blocks moved.
            MOVES_COUNT = ReceiveMovesCountFromRISCVCore();
            if (prev_moves_count != MOVES_COUNT) {
                Graphics_UpdateMovesCount(MOVES_COUNT);
                prev_moves_count = MOVES_COUNT;
            }

            // Add new blocks.
            for (int row = 0; row < 4; row++) {
                for (int col = 0; col < 4; col++) {
                    if ((ARM_GRID_COPY[row][col]) != 0 &&
                        (ARM_GRID_COPY_STATE[row][col] != UNMOVED) &&
                        (ARM_GRID_COPY_STATE[row][col] != COMBINE)) {
                        // Don't draw newly spawned block.
                        //  new_block_row and new_block_col will be set to 0xFFFF for invalid
                        //  location if new block was not added.
                        if ((row != new_block_row) || (col != new_block_col)) {
                            Graphics_AddBlock(row, col, ARM_GRID_COPY[row][col]);
                        }
                    }
                }
            }

            // Add combined blocks.
            Graphics_CombineBlocks(ARM_GRID_COPY, ARM_GRID_COPY_STATE);

            // Add new block with spawn animation.
            if (new_block_added == true) {
                Graphics_AddNewBlock(new_block_row, new_block_col,
                                     ARM_GRID_COPY[new_block_row][new_block_col]);
            }

            game_state = ReceiveGameStateFromRISCVCore();
            // Check if game is finished.
            if (game_state == WINNER) {
                PRINT("ARM: Congratulations, you win!\n");
                PRINT("ARM: Ending game.\n");

                // Give some time for user to look at grid before drawing the popup.
                MXC_Delay(MXC_DELAY_MSEC(750));
                Graphics_DisplayYouWin();

                while (1) {}
            } else if (game_state == GAME_OVER) {
                PRINT("ARM: Game Over. Nice try! Better luck next time.\n");
                PRINT("ARM: Ending game.\n");

                // Give some time for user to look at grid before drawing the popup.
                MXC_Delay(MXC_DELAY_MSEC(750));
                Graphics_DisplayGameOver();

                while (1) {}
            }

            // Signal RISC-V Core that it's ready for the next grid state.
            MXC_SEMA_FreeSema(SEMA_IDX_RISCV);
        }
    }
}
