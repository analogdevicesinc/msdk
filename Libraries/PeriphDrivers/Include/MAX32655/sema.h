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
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32655_SEMA_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32655_SEMA_H_

/* **** Includes **** */
#include "mxc_device.h"
#include "mxc_sys.h"
#include "sema_regs.h"
#include "sema_reva.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup sema Semaphore (SEMA)
 * @ingroup periphlibs
 * @{
 */

/* **** Function Prototypes **** */

/**
 * @brief   The callback routine used by the MXC_SEMA_ReadBoxAsync() and 
 *          MXC_SEMA_WriteBoxAsync functions to indicate the operation has completed.
 *
 * @param   result      The error code (if any) of the read/write operation.
 *                      See \ref MXC_Error_Codes for the list of error codes.
 */
typedef void (*mxc_sema_complete_cb_t)(int result);

/**
 * @brief     Initialize the semaphore peripheral
 * @return    #E_NO_ERROR if semaphore initialized.
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

/**
 * @brief     Initialize the mailboxes
 * @return    #E_NO_ERROR if mailboxes initialized.
 */
int MXC_SEMA_InitBoxes(void);

/**
 * @brief     Read from the mailbox
 * @details   Will only read data currently available.
 * @param     data  Buffer to store the data from the mailbox.  
 * @param     len   Number of bytes to read from the mailbox.
 * @return    #E_NO_ERROR if data read properly.
 */
int MXC_SEMA_ReadBox(uint8_t *data, unsigned len);

/**
 * @brief     Write to the mailbox
 * @details   Will only write in the space currently available.
 * @param     data  Data to write to the mailbox.  
 * @param     len   Number of bytes to write to the mailbox.
 * @return    #E_NO_ERROR if data written properly.
 */
int MXC_SEMA_WriteBox(const uint8_t *data, unsigned len);

/**
 * @brief     Semaphore interrupt handler
 * @return    #E_NO_ERROR if interrupt handled properly.
 */
int MXC_SEMA_Handler(void);

/**
 * @brief     Read asynchronously from the mailbox
 * @details   Non-blocking read. Will only read data currently available.
 * @param     cb    Callback function, called once the read is complete.
 * @param     data  Buffer to store the data from the mailbox.  
 * @param     len   Number of bytes to read from the mailbox.
 * @return    #E_NO_ERROR if data read properly.
 */
int MXC_SEMA_ReadBoxAsync(mxc_sema_complete_cb_t cb, uint8_t *data, unsigned len);

/**
 * @brief     Write asynchronously to the mailbox
 * @param     cb    Callback function, called once the write is complete.
 * @param     data  Data to write to the mailbox.  
 * @param     len   Number of bytes to write to the mailbox.
 * @return    #E_NO_ERROR if data written properly.
 */
int MXC_SEMA_WriteBoxAsync(mxc_sema_complete_cb_t cb, const uint8_t *data, unsigned len);

/**@} end of group sema */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32655_SEMA_H_
