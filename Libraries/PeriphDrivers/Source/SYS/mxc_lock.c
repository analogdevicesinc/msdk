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

/* **** Includes **** */
#include "mxc_device.h"
#include "mxc_lock.h"

#ifndef __riscv
/* ************************************************************************** */
int MXC_GetLock(uint32_t *lock, uint32_t value)
{
    do {
        // Return if the lock is taken by a different thread
        if (__LDREXW((volatile uint32_t *)lock) != 0) {
            return E_BUSY;
        }

        // Attempt to take the lock
    } while (__STREXW(value, (volatile uint32_t *)lock) != 0);

    // Do not start any other memory access until memory barrier is complete
    __DMB();

    return E_NO_ERROR;
}

/* ************************************************************************** */
void MXC_FreeLock(uint32_t *lock)
{
    // Ensure memory operations complete before releasing lock
    __DMB();
    *lock = 0;
}
#else // __riscv
/* ************************************************************************** */
int MXC_GetLock(uint32_t *lock, uint32_t value)
{
#warning "Unimplemented for RISCV"
    return E_NO_ERROR;
}

/* ************************************************************************** */
void MXC_FreeLock(uint32_t *lock)
{
#warning "Unimplemented for RISCV"
}
#endif
