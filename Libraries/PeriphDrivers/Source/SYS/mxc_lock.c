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


/* ************************************************************************** */
int MXC_GetLock(uint32_t *lock, uint32_t value)
{
    while(*lock != 0) {}

    __disable_irq();
    *lock = 1;
    __enable_irq();

    return E_NO_ERROR;
}

/* ************************************************************************** */
void MXC_FreeLock(uint32_t *lock)
{
    // Ensure memory operations complete before releasing lock
    *lock = 0;
}
