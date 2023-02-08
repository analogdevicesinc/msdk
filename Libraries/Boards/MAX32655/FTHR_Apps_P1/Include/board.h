/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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
 ******************************************************************************/

/**
 * @file    board.h
 * @brief   Board support package API.
 */

#include <stdio.h>

#ifndef LIBRARIES_BOARDS_MAX32655_FTHR_APPS_P1_INCLUDE_BOARD_H_
#define LIBRARIES_BOARDS_MAX32655_FTHR_APPS_P1_INCLUDE_BOARD_H_

#ifdef __cplusplus
extern "C" {
#endif

/* 
    Define board name:
    Use as #if defined(BOARD_FTHR_APPS_P1)
    Not as #if BOARD_FTHR_APPS_P1
*/
#define BOARD_FTHR_APPS_P1 1 /// Used in examples to control program flow.

#ifndef CONSOLE_UART
#define CONSOLE_UART 0 /// UART instance to use for console
#endif

#ifndef CONSOLE_BAUD
#define CONSOLE_BAUD 115200 /// Console baud rate
#endif

#ifndef HCI_UART
#define HCI_UART 3 /// LP UART
#endif

#ifndef TERMINAL_UART
#define TERMINAL_UART CONSOLE_UART
#endif

#ifndef USER_UART
#define USER_UART 3
#endif

#ifndef EXT_FLASH_BAUD
#define EXT_FLASH_BAUD 5000000
#endif

#define LED_OFF 1 /// Inactive state of LEDs
#define LED_ON 0 /// Active state of LEDs

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
 *  A reference to LED2 (BLUE LED in the RGB LED) of the board.
 *  Can be used with the LED_On, LED_Off, and LED_Toggle functions.
 */
#define LED3 2
#define LED_BLUE LED3

/* References to the pins connected to the two user buttons */
#define BUTTON1 0
#define BUTTON2 1

#define BUTTON_PRESSED 1
#define BUTTON_RELEASED 0

/* Reference to the I2C instantiation used to communicate with 
 * the power management IC.
 */
#define MAX20303_I2C MXC_I2C1

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
 * \brief   Initializes GPIO to conserve power in low power mode.
 */
void GPIO_PrepForSleep(void);

#ifdef __riscv
/**
 * \brief   Set up RISCV JTAG
 * \returns #E_NO_ERROR if successful
 */
int Debug_Init(void);
#endif // __riscv

/**
 * \brief   Microphone power control.
 *
 * \param   on          1 for ON, 0 for OFF
 *
 * \return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
#define POWER_OFF 0
#define POWER_ON 1
int Microphone_Power(int on);

/**
 * \brief   Camera power control.
 *
 * \param   on          1 for ON, 0 for OFF
 *
 * \return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int Camera_Power(int on);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_BOARDS_MAX32655_FTHR_APPS_P1_INCLUDE_BOARD_H_
