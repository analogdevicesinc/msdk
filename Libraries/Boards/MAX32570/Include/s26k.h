/**
 * @file    s26k.h
 * @brief   Cypress s26k driver header file
 */

/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
 * 
 * This software is protected by copyright laws of the United States and
 * of foreign countries. This material may also be protected by patent laws
 * and technology transfer regulations of the United States and of foreign
 * countries. This software is furnished under a license agreement and/or a
 * nondisclosure agreement and may only be used or reproduced in accordance
 * with the terms of those agreements. Dissemination of this information to
 * any party or parties not specified in the license agreement and/or
 * nondisclosure agreement is expressly prohibited.
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
 ******************************************************************************/

/* Define to prevent redundant inclusion */
#ifndef _S26K_H_
#define _S26K_H_

/* **** Includes **** */
#include "mxc_device.h"
#include "hpb.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup S26K
 * @{
 */

/* **** Definitions **** */

/* **** Function Prototypes **** */

/**
 * @brief   Configure the S26K.
 * @param   cs      1 for cs 1, 0 for cs 0.
 * @param   base    Base address for the given CS.
 */
void S26K_Init(unsigned cs, uint32_t base);

/**
 * @brief   Get the status register of the S26K.
 * @returns Data in the status register.
 */
uint16_t S26K_GetStatus(void);

/**
 * @brief   Clear the flags in the status register register.
 */
void S26K_ClearStatus(void);

/**
 * @brief   Erase the entire chip.
 * @note    Will take a long time to complete.
 * @returns #E_NO_ERROR if successful, #E_UNKNOWN if not.
 */
int S26K_ChipErase(void);

/**
 * @brief   Check to see if the give sector is blank.
 * @param   addr    Address for the sector we want to check.
 * @returns #E_NO_ERROR if blank, #E_UNINITIALIZED if not.
 */
int S26K_BlankCheck(uint32_t addr);

/**
 * @brief   Check to see if the give sector is blank.
 * @param   addr    Address for the sector we want to erase.
 * @returns #E_NO_ERROR if erase succeeded. #E_UNKNOWN otherwise.
 */
int S26K_SectorErase(uint32_t addr);

/**
 * @brief   Write a 16 bit value to the give address.
 * @param   addr    Address we want to write to.
 * @param   data    Data to write.
 * @returns #E_NO_ERROR if erase succeeded. #E_UNKNOWN otherwise.
 */
int S26K_Write16(uint32_t addr, uint16_t data);

/**
 * @brief   Write data to the s26k buffer and program.
 * @param   addr    Address we want to write to.
 * @param   data    Data to write.
 * @param   len     Number of bytes to write.
 * @returns #E_NO_ERROR if erase succeeded. #E_UNKNOWN otherwise.
 */
int S26K_Write(uint32_t addr, uint16_t *data, unsigned len);

/**
 * @brief   Read the s26k ID.
 * @param   offset  Offset to start reading from.
 * @param   data    Pointer to store the ID.
 * @param   len     Number of ID bytes to read.
 */
void S26K_GetID(uint32_t offset, uint16_t *data, unsigned len);

/**
 * @brief   Read the s26k CFI.
 * @param   offset  Offset to start reading from.
 * @param   data    Pointer to store the CFI.
 * @param   len     Number of CFI bytes to read.
 */
void S26K_GetCFI(uint32_t offset, uint16_t *data, unsigned len);

uint16_t S26K_GetSectorProtection(uint32_t addr);
uint16_t S26K_GetSectorPPB(uint32_t addr);
void S26K_PPBErase(void);
uint16_t S26K_GetPPBLockStatus(void);
void S26K_ClearPBLock(void);
uint16_t S26K_GetDYBStatus(uint32_t addr);
void S26K_SetDYB(uint32_t addr);
void S26K_ClearDYB(uint32_t addr);
void S26K_SetSectorPPB(uint32_t addr);
uint16_t S26K_GetASPStatus(void);
void S26K_WriteBufferAbortReset(void);

/**@} end of group S26K */

#ifdef __cplusplus
}
#endif

#endif /* _S26K_H_ */
