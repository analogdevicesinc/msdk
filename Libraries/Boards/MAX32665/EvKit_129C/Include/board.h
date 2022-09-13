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
 * $Date: 2019-10-29 09:21:01 -0500 (Tue, 29 Oct 2019) $
 * $Revision: 48140 $
 *
 ******************************************************************************/

/**
 * @file    board.h
 * @brief   Board support package API.
 */

#include <stdio.h>

#ifndef _BOARD_H
#define _BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CONSOLE_UART
#define CONSOLE_UART 1 /// UART instance to use for console
#endif

#ifndef CONSOLE_BAUD
#define CONSOLE_BAUD 115200 /// Console baud rate
#endif

#define LED_OFF 1 /// Inactive state of LEDs
#define LED_ON 0 /// Active state of LEDs

#ifndef EXT_FLASH_BAUD
#define EXT_FLASH_BAUD 4000000
#endif

#ifndef HCI_UART
#define HCI_UART 0
#endif

#ifndef TERMINAL_UART
#define TERMINAL_UART CONSOLE_UART
#endif

#ifndef USER_UART
#define USER_UART 2
#endif

#ifndef HCI_UART_MAP
#define HCI_UART_MAP MAP_A
#endif

#ifndef TERMINAL_UART_MAP
#define TERMINAL_UART_MAP MAP_A
#endif

#ifndef USER_UART_MAP
#define USER_UART_MAP MAP_A
#endif

/**
 *  A reference to LED1 (RED LED in the RGB LED) of the board.
 *  Can be used with the LED_On, LED_Off, and LED_Toggle functions.
 */
#define LED1 0
#define LED_RED LED1

/**
 *  A reference to LED2 (GREEN LED in the RGB LED) of the board.
 *  Can be used with the LED_On, LED_Off, and LED_Toggle functions.
 */
#define LED2 1
#define LED_GREEN LED2

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
 * @brief   Initialize or reinitialize the console. This may be necessary if the
 *          system clock rate is changed.
 * @returns #E_NO_ERROR if everything is successful
 */
int Console_Shutdown(void);

#ifdef __cplusplus
}
#endif

#endif /* _BOARD_H */
