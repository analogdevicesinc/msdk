/**
 * @file
 * @brief   Touchscreen driver API header file
 */
/* ****************************************************************************
 * Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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

#ifndef LIBRARIES_MISCDRIVERS_TOUCHSCREEN_TSC2046_H_
#define LIBRARIES_MISCDRIVERS_TOUCHSCREEN_TSC2046_H_

#include <spi.h>
#include <gpio.h>

/************************************************************************************/
#define TS_MAX_BUTTONS 16
#define TS_INVALID_KEY_CODE -1

typedef enum {
    TSC_TEMP0 = (0x83 | 0x00 | 0x00),
    TSC_Y = (0x83 | 0x10 | 0x04),
    TSC_VBAT = (0x83 | 0x20 | 0x00),
    TSC_Z1 = (0x83 | 0x30 | 0x04),
    TSC_Z2 = (0x83 | 0x40 | 0x04),
    TSC_X = (0x83 | 0x50 | 0x04),
    TSC_AUX = (0x83 | 0x60 | 0x00),
    TSC_TEMP1 = (0x83 | 0x70 | 0x00),
    TSC_DIFFX = (0x81 | 0x50 | 0x00),
    TSC_DIFFY = (0x81 | 0x10 | 0x00),
    TSC_DIFFZ1 = (0x81 | 0x30 | 0x00),
    TSC_DIFFZ2 = (0x81 | 0x40 | 0x00),
    TSC_START = (0x82 | 0x00 | 0x00),
    TSC_STOP = (0x81 | 0x00 | 0x00)
} mxc_ts_touch_cmd_t;

typedef struct {
    mxc_spi_regs_t *regs; // The SPI instance the touchscreen controller is connected to
    mxc_gpio_cfg_t gpio; // The SPI pins: MOSI, MISO, SCK, SS, must define all them
    unsigned int freq; // The SPI frequency
    int ss_idx; // The SSEL index to use when communicating with the touchscreen controller
} mxc_ts_spi_config;

/************************************************************************************/

// SPI Transport layer functions

/**
 * @brief       Initialize the SPI instance connected to the TouchScreen driver.
 *              Board files must implement this.
 */
extern void TS_SPI_Init(void);

/**
 * @brief       Send a byte of data to the TouchScreen driver over SPI.
 *              Board files must implement this.
 * 
 * @param       datain      Input value to write
 * @param[out]  dataout     Output pointer.  This function will decode and save
 *                          the SPI response to this pointer.
 */
extern void TS_SPI_Transmit(uint8_t datain, uint16_t *dataout);

/************************************************************************************/

int MXC_TS_AssignInterruptPin(mxc_gpio_cfg_t pin);

/**
 * @brief      Used to register hw related configuration, need to be called before MXC_TS_Init()
 *
 * @param      spi_config   Touch screen SPI configuration, SPI instance, freq, pins slave slect id
 * @param      int_pin      The GPIO pin configuration for the touchscreen controller's interrupt pin
 * @param      busy_pin     The GPIO pin configuration for the touchscreen controller's busy pin
 *
 * @return     See \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TS_PreInit(mxc_ts_spi_config *spi_config, mxc_gpio_cfg_t *int_pin,
                   mxc_gpio_cfg_t *busy_pin);

/**
 * @brief      Initialize the touchscreen controller
 *
 * @return     See \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TS_Init(void);

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
 * @brief      Get the x,y coordinates of the last touchscreen press
 *
 * @param[out] x    (Output) Where to save the x coordinate
 * @param[out] y    (Output) Where to save the y coordinate
 */
void MXC_TS_GetXY(unsigned int *x, unsigned int *y);

/**
 * @brief       Returns true if there is a touchscreen event pending,
 *              otherwise returns false.
 */
int MXC_TS_GetTSEvent();

/**
 * @brief      Clears the pending touchscreen event flag.
 */
void MXC_TS_ClearTSEvent();

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

#endif // LIBRARIES_MISCDRIVERS_TOUCHSCREEN_TSC2046_H_
