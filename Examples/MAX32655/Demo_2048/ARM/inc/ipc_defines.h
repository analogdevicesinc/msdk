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

#ifndef EXAMPLES_MAX32655_DEMO_2048_ARM_INC_IPC_DEFINES_H_
#define EXAMPLES_MAX32655_DEMO_2048_ARM_INC_IPC_DEFINES_H_

/* **** Includes **** */

#include <stdint.h>

/* **** Definitions **** */

// These should match with the RISC-V core's copy of ipc_defines.h

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

#define MAILBOX_MAIN_GRID_IDX (0) // Main grid indices are from 0 to (16 blocks * 4 bytes) - 1.
#define MAILBOX_MAIN_GRID_STATE_IDX \
    (4 * 16) // Indices are from (4 bytes * 16) to ((4 bytes * 16) + (1 byte * 16)))
#define MAILBOX_KEYPRESS_IDX ((4 * 16) + (1 * 16)) // All indices before are for the main grids.
#define MAILBOX_IF_BLOCK_MOVED_IDX (MAILBOX_KEYPRESS_IDX + 1)
#define MAILBOX_NEW_BLOCK_LOCATION_IDX (MAILBOX_IF_BLOCK_MOVED_IDX + 1)
#define MAILBOX_GAME_STATE_IDX (MAILBOX_NEW_BLOCK_LOCATION_IDX + 1)
#define MAILBOX_MOVES_COUNT_IDX (MAILBOX_GAME_STATE_IDX + 1)

#endif // EXAMPLES_MAX32655_DEMO_2048_ARM_INC_IPC_DEFINES_H_
