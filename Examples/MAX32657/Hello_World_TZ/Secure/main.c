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
 * @brief   Hello World!
 * @details This example uses the UART to print to a terminal and flashes an LED.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_errors.h"
#include "led.h"
#include "pb.h"
#include "board.h"
#include "mxc_delay.h"

/***** Definitions *****/

// Starting address of the Non-Secure Vector Table.
//  Confirm by checking the Non-Secure {project}.map file and finding the
//  Reset_Handler address then "- 4" (Top of Stack).
#define VECTOR_TABLE_START_ADDR_NS (0x010803A8)
#define Reset_Handler_ADDR_NS  (*((uint32_t *)(VECTOR_TABLE_START_ADDR_NS + 4)))

/***** Globals *****/

/***** Functions *****/

void NonSecure_Init(void)
{
    mxc_ns_call_t Reset_Handler_NS;

    // Setup Non-Secure vector table.
    SCB_NS->VTOR = VECTOR_TABLE_START_ADDR_NS;

    // Setup Non-Secure Main Stack Pointer (MSP_NS).
    //  Start of vector table contains top of stack value.
    __TZ_set_MSP_NS((*(uint32_t *)VECTOR_TABLE_START_ADDR_NS));

    // Get Non-Secure Reset_Handler.
    Reset_Handler_NS = (mxc_ns_call_t)Reset_Handler_ADDR_NS;

    // Start Non-Secure code.
    Reset_Handler_NS();

    // The code should never reach this state.
    printf("Error: The core never reached the non-secure world.\n");
}

/**
 * Note: Increment counter in Secure context.
 * 
 * Warning: Prevent leaks by checking pointers passed into
 *  Secure functions before dereferencing them, or the Non-Secure
 *  world can access all of Secure memory. Secure code must also
 *  handle non-secure memory as volatile.
 * 
 * Param:   *count    Pointer of counter value to increment.
 * Return:  Error code.
 */
__ns_entry int IncrementCount_S(volatile int *count_ns)
{
    // CMSE intrinsic function. Check permissions before derefencing pointer.
    count_ns = cmse_check_pointed_object((int *)count_ns, CMSE_NONSECURE);

    // count_ns will be NULL on a failed check.
    if (count_ns == NULL) {
        return E_NULL_PTR;
    }

    (*count_ns)++;

    return E_NO_ERROR;
}

// *****************************************************************************
int main(void)
{
    printf("**** Hello_World example with TrustZone ****\n");
    printf("Currently in the secure world.\n");

    // Add any Secure World software initialization and routines here
    //  before NonSecure_Init();

    printf("Now transitioning to the non-secure world.\n");

    // Transition to Non-Secure world.
    NonSecure_Init();

    // Should never reach here.
    printf("Error: Code should not reach here.\n");

    // Enable Red LED.
    LED_On(1);

    while (1) {}
}
