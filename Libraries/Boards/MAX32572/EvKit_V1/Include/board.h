/*******************************************************************************
 * Copyright (C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
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

#include <stdio.h>
#include "spixf.h"

#ifndef _BOARD_H
#define _BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CONSOLE_UART
#define CONSOLE_UART 0 /// UART instance to use for console
#endif

#ifndef CONSOLE_BAUD
#define CONSOLE_BAUD 115200 /// Console baud rate
#endif

#define LED_OFF 1 /// Inactive state of LEDs
#define LED_ON  0 /// Active state of LEDs

#ifndef MX25_BAUD
#define MX25_BAUD 3000000
#endif
// #define MX25_SPI                    MXC_SPIXC
// #define MX25_SSEL                   0
// // #define SPI_CHAR_BITS               8

// const spixc_cfg_t mx25_spixc_cfg;

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
 * \brief   Initialize the SPI peripheral to use for MX25
  * \returns #E_NO_ERROR if everything is successful
 */
int MX25_Board_Init(void);

/**
 * \brief   Translation function to implement SPI Read transaction
 * @param   read        Pointer to where master will store data.
 * @param   len         Number of characters to send.
 * @param   deassert    Deassert slave select at the end of the transaction.
 * @param   width       spi_width_t for how many data lines to use
 * \returns #E_NO_ERROR if successful, !=0 otherwise
 */

int MX25_Board_Read(uint8_t* read, unsigned len, unsigned deassert, mxc_spixf_width_t width);
/**
 * \brief   Translation function to implement SPI Write transaction
 * @param   write       Pointer to data master will write.
 * @param   len         Number of characters to send.
 * @param   deassert    Deassert slave select at the end of the transaction.
 * @param   width       spi_width_t for how many data lines to use
 * \returns #E_NO_ERROR if successful, !=0 otherwise
 */

int MX25_Board_Write(const uint8_t* write, unsigned len, unsigned deassert,
                     mxc_spixf_width_t width);

/**
 * \brief   Send clocks on SCLK.
 * @param   len         Number of characters to send.
 * @param   deassert    Deassert slave select at the end of the transaction.
 * \returns #E_NO_ERROR if successful, !=0 otherwise
 */
int MX25_Clock(unsigned len, unsigned deassert);
;

#ifdef __cplusplus
}
#endif

#endif /* _BOARD_H */
