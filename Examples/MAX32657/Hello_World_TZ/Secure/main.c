/******************************************************************************
 *
 * Copyright (C) 2024-2025 Analog Devices, Inc.
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
 * @brief   Hello World - Secure.
 * @details This TrustZone-enabled exampled splits Hello_World into two partitions.
 *          The Secure world setups the transition to the Non-Secure world, and
 *          increments the counter. The Non-Secure world prints the count,
 *          and toggles the LED.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "mxc.h"

/***** Definitions *****/

/***** Globals *****/

/***** Functions *****/

/**
 * Brief: Increment counter in Secure context from Non-Secure world.
 * 
 * Warning: Prevent leaks by checking pointers passed into
 *  Secure functions before dereferencing them, or the Non-Secure
 *  world can access all of Secure memory. Secure code must also
 *  handle non-secure memory as volatile.
 * 
 * By design, illegal accesses would trigger a hard/secure fault unless
 *  handled by the MPC/PPC.
 * 
 * __ns_entry => __attribute((cmse_nonsecure_entry))
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
    int error;

    printf("**** Hello_World example with TrustZone ****\n");
    printf("Currently in the secure world.\n");

    // Add any Secure World software initialization and routines here
    //  before NonSecure_Init();

    printf("Beginning transition to the non-secure world.\n");

    // Wait for Console UART to finish printing before setting the peripheral to Non-Secure.
    while (MXC_UART_GetActive(MXC_UART_GET_UART(CONSOLE_UART)) == E_BUSY) {}

    // Set Flash (Code region) as Non-Secure Callable for the IncrementCount_S function.
    MXC_SPC_SetCode_NSC(true);

    // Set UART (serial console), SYS, and GPIO (LED) peripheral to Non-Secure.
    //  Note: These peripherals will no longer be accessible in Secure world.
    MXC_SPC_SetNonSecure(MXC_SPC_PERIPH_GPIO0);
    MXC_SPC_SetNonSecure(MXC_SPC_PERIPH_GCR);
    MXC_SPC_SetNonSecure(MXC_SPC_PERIPH_UART);

    // Transition to Non-Secure world.
    //  Defined in system_max32657.c as a weak function.
    error = NonSecure_Init();

    // Should never reach here.
    //  Set Console UART accessible to Secure world to print out error message.
    MXC_SPC_SetSecure(MXC_SPC_PERIPH_UART);
    printf("[Error] Code should not reach here. Transition not successful. Error Code: %d\n",
           error);

    while (1) {}
}
