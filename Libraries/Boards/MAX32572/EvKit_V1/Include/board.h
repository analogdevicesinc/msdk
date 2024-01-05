/*******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc. (now owned by Analog
 * Devices, Inc.), Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary and confidential to Analog Devices, Inc. and its licensors.
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
 * $Date: 2017-07-17 18:10:38 -0500 (Mon, 17 Jul 2017) $
 * $Revision: 28913 $
 *
 ******************************************************************************/

/**
 * @file    board.h
 * @brief   Board support package API.
 */

#ifndef LIBRARIES_BOARDS_MAX32572_EVKIT_V1_INCLUDE_BOARD_H_
#define LIBRARIES_BOARDS_MAX32572_EVKIT_V1_INCLUDE_BOARD_H_

#include <stdio.h>
#include "spixf.h"
#include "led.h"
#include "pb.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef LED_OFF
#undef LED_OFF
#endif
#define LED_OFF 0 /// Override inactive state of LEDs

#ifdef LED_ON
#undef LED_ON
#endif
#define LED_ON 1 /// Override active state of LEDs

#ifndef CONSOLE_UART
#define CONSOLE_UART 0 /// UART instance to use for console
#endif

#ifndef CONSOLE_BAUD
#define CONSOLE_BAUD 115200 /// Console baud rate
#endif

#define EXT_FLASH_SPIXFC_BAUD 3000000
#define EXT_FLASH_SPIXFM_BAUD 24000000

#define SPIXFC_CMD_VAL 0x0B
#define SPIXFM_BUS_IDLE_VAL 0x1000

/**
 * \brief   Initialize the BSP and board interfaces.
 * \returns #E_NO_ERROR if everything is successful
 */
int Board_Init(void);

/**
 * \brief   Initialize or reinitialize the console. This may be necessary if the
 *          system clock rate is changed.
 * \returns #E_NO_ERROR if everything is successful
 */
int Console_Init(void);

/**
 * \brief   Shutdown the console.
 * \returns #E_NO_ERROR if everything is successful
 */
int Console_Shutdown(void);

/**
 * \brief   Attempt to prepare the console for sleep.
 * \returns #E_NO_ERROR if ready to sleep, #E_BUSY if not ready for sleep.
 */
int Console_PrepForSleep(void);

/**
 * @brief      Writes data to external flash.
 * @note       This function must be executed from RAM.
 * @param      address  Address in external flash to start writing from.
 * @param      length   Number of bytes to be written.
 * @param      buffer   Pointer to data to be written to external flash.
 * @return     #EF_E_SUCCESS If function is successful.
 * @note       make sure to disable SFCC and interrupts; before running this function
 */
int MXC_Ext_Write(uint32_t address, uint32_t length, uint8_t *buffer);

/**
 * @brief      Reads data from external flash
 * @note       This function must be executed from RAM.
 * @param[in]  address  The address to read from
 * @param      buffer   The buffer to read the data into
 * @param[in]  len      The length of the buffer
 * @return     #EF_E_SUCCESS If function is successful.
 * @note       make sure to disable SFCC and interrupts; before running this function
 */
int MXC_Ext_Read(int address, uint8_t* buffer, int len);

/**
 * @brief      Erases the sector of external flash at the specified address.
 * @note       This function must be executed from RAM.
 * @param      address  Any address within the sector to erase.
 * @return     #EF_E_SUCCESS If function is successful.
 * @note       make sure to disable SFCC and interrupts; before running this function
 */
int MXC_Ext_SectorErase(int address);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_BOARDS_MAX32572_EVKIT_V1_INCLUDE_BOARD_H_
