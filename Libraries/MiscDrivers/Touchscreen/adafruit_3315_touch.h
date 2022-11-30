/**
 * @file    adafruit_3315_touch.h
 * @brief   Touchscreen driver for adafruit 3315
 */
/* ****************************************************************************
 * Copyright (C) 2021 Maxim Integrated Products, Inc., All Rights Reserved.
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

#ifndef LIBRARIES_MISCDRIVERS_TOUCHSCREEN_ADAFRUIT_3315_TOUCH_H_
#define LIBRARIES_MISCDRIVERS_TOUCHSCREEN_ADAFRUIT_3315_TOUCH_H_

#include <stdint.h>
#include <stdbool.h>
#include "spi.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TS_MAX_BUTTONS 16
#define TS_INVALID_KEY_CODE -1

extern volatile bool ts_event;

/************************************************************************************/
/**
 * @brief      Initialize the touchscreen and display
 *
 * @param      ts_spi           The SPI instance the touchscreen controller is connected to
 * @param      ss_idx           The SSEL index to use when communicating with the touchscreen controller
 * @param      reset_ctrl       The GPIO pin configuration for the touchscreen controller's interrupt pin
 * @param      bl_ctrl          The GPIO pin configuration for the touchscreen controller's busy pin
 *
 * @return     See \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TS_Init(mxc_spi_regs_t *ts_spi, int ss_idx, mxc_gpio_cfg_t *int_pin,
                mxc_gpio_cfg_t *busy_pin);

/**
 * @brief      Enables touch interrupts
 *
 */
void MXC_TS_Start(void);

/**
 * @brief      Disables touch interrupts
 *
 */
void MXC_TS_Stop(void);

/**
 * @brief      Register a button
 *
* @param      x0, y0, x1, y1 location of button
* @param      on_press_expected_code  expected keycode when touch related point area
 */
int MXC_TS_AddButton(int x0, int y0, int x1, int y1, int on_press_expected_code);

/**
 * @brief      Remove a button
 *
 * @param      x0, y0, x1, y1 location of button
 */
void MXC_TS_RemoveButton(int x0, int y0, int x1, int y1);

/**
 * @brief      Remove all registered keys
 *
 */
void MXC_TS_RemoveAllButton(void);

/**
 * @brief      Read pressed key
 *
 */
int MXC_TS_GetKey(void);

/**
 * @brief      Read touch point
 *
 */
void MXC_TS_GetTouch(uint16_t *x, uint16_t *y);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_MISCDRIVERS_TOUCHSCREEN_ADAFRUIT_3315_TOUCH_H_
