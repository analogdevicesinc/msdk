/**
 * @file    sema.h
 * @brief   Semaphore (SEMA) function prototypes and data types.
 */

/* ****************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 *************************************************************************** */

/* Define to prevent redundant inclusion */
#ifndef _SEMA_H_
#define _SEMA_H_

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
 * @brief     Initialize the semaphore peripheral
 * @return    #E_NO_ERROR if semaphore initialized.
 */
int MXC_SEMA_Init (void);

/**
 * @brief     Attempt to get a semaphore.
 * @param     sema   Number of semaphore you are trying to get.
 * @return    #E_NO_ERROR if semaphore acquired. #E_BUSY if semaphore is already locked.
 */
int MXC_SEMA_GetSema (unsigned sema);

/**
 * @brief     Check a semaphore.
 * @param     sema   Number of semaphore you want to check.
 * @return    #E_NO_ERROR if semaphore is free. #E_BUSY if semaphore is already locked.
 * @note      Will not be atomic if you call this function and then attempt to MXC_SEMA_GetSema().
 */
int MXC_SEMA_CheckSema (unsigned sema);

/**
 * @brief     Check all semaphores.
 * @return    Status of all semaphores. Each semaphore will be represented by 1 bit.
 * @note      Will not be atomic if you call this function and then attempt to MXC_SEMA_GetSema().
 */
uint32_t MXC_SEMA_Status (void);

/**
 * @brief     Frees the semaphore.
 * @param     sema   Number of semaphore want to free.
 */
void MXC_SEMA_FreeSema (unsigned sema);

/**
 * @brief     Shutdown the semaphore peripheral
 * @return    #E_NO_ERROR if semaphore released.
 */
int MXC_SEMA_Shutdown (void);

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
int MXC_SEMA_ReadBox(uint8_t* data, unsigned len);

/**
 * @brief     Write to the mailbox
 * @details   Will only write in the space currently available.
 * @param     data  Data to write to the mailbox.  
 * @param     len   Number of bytes to write to the mailbox.
 * @return    #E_NO_ERROR if data written properly.
 */
int MXC_SEMA_WriteBox(const uint8_t* data, unsigned len);

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
int MXC_SEMA_ReadBoxAsync(mxc_sema_complete_cb_t cb,
    uint8_t* data, unsigned len);

/**
 * @brief     Write asynchronously to the mailbox
 * @param     cb    Callback function, called once the write is complete.
 * @param     data  Data to write to the mailbox.  
 * @param     len   Number of bytes to write to the mailbox.
 * @return    #E_NO_ERROR if data written properly.
 */
int MXC_SEMA_WriteBoxAsync(mxc_sema_complete_cb_t cb,
    const uint8_t* data, unsigned len);

/**@} end of group sema */

#ifdef __cplusplus
}
#endif

#endif /* _SEMA_H_ */
