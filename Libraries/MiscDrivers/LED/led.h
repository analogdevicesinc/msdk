/**
 * @file
 * @brief   LED driver API.
 */

/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

#ifndef LIBRARIES_MISCDRIVERS_LED_LED_H_
#define LIBRARIES_MISCDRIVERS_LED_LED_H_

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
#define LED_OFF 0 /**< Define to turn off the LED. */
#endif

#ifndef LED_ON
#define LED_ON 1 /**< Define to turn on the LED. */
#endif

#ifndef LED1
#define LED1 0
#endif
#ifndef LED2
#define LED2 1
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
 * @brief      Turn the specified LED on
 * @param      idx   LED index
 *
 * @note Some board files have defines for 'idx' that match the silk screen on the board (e.g., LED1, LED2, etc.).
 */
void LED_On(unsigned int idx);

/**
 * @brief      Turn the specified LED off.
 * @param      idx   LED index
 *
 * @note Some board files have defines for 'idx' that match the silk screen on the board (e.g., LED1, LED2, etc.).
 */
void LED_Off(unsigned int idx);

/**
 * @brief      Toggle the state of the specified LED.
 * @param      idx   LED index
 *
 * @note Some board files have defines for 'idx' that match the silk screen on the board (e.g., LED1, LED2, etc.).
 */
void LED_Toggle(unsigned int idx);

#ifdef __cplusplus
}
#endif
/**@}*/
#endif // LIBRARIES_MISCDRIVERS_LED_LED_H_
