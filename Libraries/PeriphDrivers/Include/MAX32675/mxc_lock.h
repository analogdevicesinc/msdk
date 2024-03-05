/**
 * @file    mxc_lock.h
 * @brief   Exclusive access lock utility functions.
*/

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

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32675_MXC_LOCK_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32675_MXC_LOCK_H_

/* **** Includes **** */
#include "mxc_device.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @ingroup    syscfg
 * @defgroup   mxc_lock_utilities Exclusive Access Locks
 * @brief      Lock functions to obtain and release a variable for exclusive
 *             access. These functions are marked interrupt safe if they are
 *             interrupt safe.
 * @{
 */

/* **** Definitions **** */

/* **** Globals **** */

/* **** Function Prototypes **** */

/**
 * @brief      Attempts to acquire the lock.
 * @details    This in an interrupt safe function that can be used as a mutex.
 *             The lock variable must remain in scope until the lock is
 *             released. Will not block if another thread has already acquired
 *             the lock.
 * @param      lock   Pointer to variable that is used for the lock.
 * @param      value  Value to be place in the lock. Can not be 0.
 *
 * @return     #E_NO_ERROR if everything successful, #E_BUSY if lock is taken.
 */
int MXC_GetLock(uint32_t *lock, uint32_t value);

/**
 * @brief         Free the given lock.
 * @param[in,out] lock  Pointer to the variable used for the lock. When the lock
 *                      is free, the value pointed to by @p lock is set to zero.
 */
void MXC_FreeLock(uint32_t *lock);

/**@} end of group mxc_lock_utilities */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32675_MXC_LOCK_H_
