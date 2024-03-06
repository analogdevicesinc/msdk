/**
 * @file    sema.h
 * @brief   Semaphore (SEMA) function prototypes and data types.
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
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX78002_SEMA_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX78002_SEMA_H_

/* **** Includes **** */
#include "mxc_device.h"
#include "mxc_sys.h"
#include "sema_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup sema Semaphore (SEMA)
 * @ingroup periphlibs
 * @{
 */

/* **** Definitions **** */

/* **** Function Prototypes **** */

/**
 * @brief     Initialize the semaphore peripheral
 * @return    #E_NO_ERROR if semaphore acquired.
 */
int MXC_SEMA_Init(void);

/**
 * @brief     Attempt to get a semaphore.
 * @param     sema   Number of semaphore you are trying to get.
 * @return    #E_NO_ERROR if semaphore acquired. #E_BUSY if semaphore is already locked.
 */
int MXC_SEMA_GetSema(unsigned sema);

/**
 * @brief     Check a semaphore.
 * @param     sema   Number of semaphore you want to check.
 * @return    #E_NO_ERROR if semaphore is free. #E_BUSY if semaphore is already locked.
 * @note      Will not be atomic if you call this function and then attempt to MXC_SEMA_GetSema().
 */
int MXC_SEMA_CheckSema(unsigned sema);

/**
 * @brief     Check all semaphores.
 * @return    Status of all semaphores. Each semaphore will be represented by 1 bit.
 * @note      Will not be atomic if you call this function and then attempt to MXC_SEMA_GetSema().
 */
uint32_t MXC_SEMA_Status(void);

/**
 * @brief     Frees the semaphore.
 * @param     sema   Number of semaphore want to free.
 */
void MXC_SEMA_FreeSema(unsigned sema);

/**
 * @brief     Shutdown the semaphore peripheral
 * @return    #E_NO_ERROR if semaphore released.
 */
int MXC_SEMA_Shutdown(void);

/**@} end of group sema */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX78002_SEMA_H_
