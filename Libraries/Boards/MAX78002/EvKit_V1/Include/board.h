/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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

#ifndef LIBRARIES_BOARDS_MAX78002_EVKIT_V1_INCLUDE_BOARD_H_
#define LIBRARIES_BOARDS_MAX78002_EVKIT_V1_INCLUDE_BOARD_H_

/**
 * @file    board.h
 * @brief   Board support package API.
 */

#include <stdio.h>
#include "spi.h"

#ifdef TFT_ADAFRUIT
#include "adafruit_3315_tft.h"
#include "adafruit_3315_touch.h"
#endif
#ifdef TFT_NEWHAVEN
#include "tft_st7789v.h"
#include "tsc2046.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_EVKIT_V1

#ifndef CONSOLE_UART
#define CONSOLE_UART 0 /// UART instance to use for console
#endif

#ifndef CONSOLE_BAUD
#define CONSOLE_BAUD 115200 /// Console baud rate
#endif

#define SCCB_SCL_PORT MXC_GPIO0 /// SCCB clock port
#define SCCB_SCL_PIN MXC_GPIO_PIN_16 /// SCCB clock pin
#define SCCB_SDA_PORT MXC_GPIO0 /// SCCB data port
#define SCCB_SDA_PIN MXC_GPIO_PIN_17 /// SCCB data pin

#define LED_OFF 0 /// Inactive state of LEDs
#define LED_ON 1 /// Active state of LEDs

#define TFT_SPI_PORT MXC_GPIO0 /**< GPIO port for SPI peripheral pins. */
#define TFT_SPI_PINS \
    MXC_GPIO_PIN_5 | MXC_GPIO_PIN_6 | MXC_GPIO_PIN_7 /**< GPIO pins for SPI peripheral. */

/**
 *  A reference to LED1 of the board.
 *  Can be used with the LED_On, LED_Off, and LED_Toggle functions.
 */
#define LED1 0
/**
 *  A reference to LED2 of the board.
 *  Can be used with the LED_On, LED_Off, and LED_Toggle functions.
 */
#define LED2 1

#ifdef TFT_NEWHAVEN
#define TFT_SPI MXC_SPI0 // SPI port to use for TFT display
#define TFT_SPI_FREQ 20000000 /**< SPI clock frequency in Hertz. */
#define TFT_DC_PORT MXC_GPIO2 /**< GPIO port for Data/Command signal. */
#define TFT_DC_PIN MXC_GPIO_PIN_2 /**< GPIO pin for Data/Command signal. */
#define TFT_SS_PORT MXC_GPIO0 /**< GPIO port for select signal. */
#define TFT_SS_PIN MXC_GPIO_PIN_3 /**< GPIO pin for select signal. */

#define TS_SPI MXC_SPI0
#define TS_SPI_FREQ 1000000
#define TS_IRQ_PORT MXC_GPIO0
#define TS_IRQ_PIN MXC_GPIO_PIN_19
#define TS_SS_PORT MXC_GPIO1
#define TS_SS_PIN MXC_GPIO_PIN_1

void TFT_SPI_Init(void);
void TFT_SPI_Write(uint8_t data, bool cmd);
void TFT_SPI_Transmit(void *src, int count);
#endif

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

#ifdef __riscv
/**
 * \brief   Set up RISCV JTAG
 * \returns #E_NO_ERROR if successful
 */
int Debug_Init(void);
#endif // __riscv
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

#endif // LIBRARIES_BOARDS_MAX78002_EVKIT_V1_INCLUDE_BOARD_H_
