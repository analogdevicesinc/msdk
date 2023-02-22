/**
 * @file
 * @brief   LED driver API.
 */
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

#ifndef LIBRARIES_BOARDS_MAX32680_INCLUDE_LED_H_
#define LIBRARIES_BOARDS_MAX32680_INCLUDE_LED_H_

#include "mxc_assert.h"
#include "board.h"
#include "gpio.h"

#ifdef __cplusplus
extern "C" {
#endif
/**
 * @ingroup bsp
 * @defgroup led_bsp LED Board Support API.
 * @{
 */
/* **** Definitions **** */
#ifndef LED_OFF
#define LED_OFF 1 /**< Define to turn off the LED. */
#endif

#ifndef LED_ON
#define LED_ON 0 /**< Define to turn on the LED. */
#endif

/* **** Global Variables **** */
extern const mxc_gpio_cfg_t led_pin[];
extern const unsigned int num_leds;

/* **** Function Prototypes **** */

/**
 * @brief      Initialize all LED pins.
 * @retval     #E_NO_ERROR   Push buttons intialized successfully.
 * @retval     "Error Code"  @ref MXC_Error_Codes "Error Code" if unsuccessful.
 */
int LED_Init(void);

/**
 * @brief      Turn the specified LED on.
 * @param      idx   LED index
 */
void LED_On(unsigned int idx);

/**
 * @brief      Turn the specified LED off.
 * @param      idx   LED index
 */
void LED_Off(unsigned int idx);

/**
 * @brief      Toggle the state of the specified LED.
 * @param      idx   LED index
 */
void LED_Toggle(unsigned int idx);

#ifdef __cplusplus
}
#endif
/**@}*/

#endif // LIBRARIES_BOARDS_MAX32680_INCLUDE_LED_H_
