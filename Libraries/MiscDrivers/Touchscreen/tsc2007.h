/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc. All Rights Reserved. This software
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

#ifndef LIBRARIES_MISCDRIVERS_TOUCHSCREEN_TSC2007_H_
#define LIBRARIES_MISCDRIVERS_TOUCHSCREEN_TSC2007_H_

#include <i2c.h>
#include <gpio.h>

#ifndef TS_MAX_BUTTONS
#define TS_MAX_BUTTONS 16
#endif
#define TS_INVALID_KEY_CODE -1

#define TSC2007_I2C_TARGET_ADDRESS 0x48

typedef enum {
    TSC_MEASURE_TEMP0 = 0b0000,
    TSC_MEASURE_AUX = 0b0010,
    TSC_MEASURE_TEMP1 = 0b0100,
    TSC_ACTIVATE_X_NEGATIVE = 0b1000,
    TSC_ACTIVATE_Y_NEGATIVE = 0b1001,
    TSC_ACTIVATE_Y_POSITIVE_X_NEGATIVE = 0b1010,
    TSC_SETUP_COMMAND = 0b1011,
    TSC_MEASURE_X = 0b1100,
    TSC_MEASURE_Y = 0b1101,
    TSC_MEASURE_Z1 = 0b1110,
    TSC_MEASURE_Z2 = 0b1111
} mxc_ts_cmd_func_t;

typedef enum {
    TSC_POWER_DOWN_IRQ_EN = 0b00,
    TSC_ADC_ON_IRQ_DIS_0 = 0b01,
    TSC_ADC_OFF_IRQ_EN = 0b10,
    TSC_ADC_ON_IRQ_DIS_1 = 0b11
} mxc_ts_cmd_pdown_t;

typedef enum { TSC_12_BIT = 0b0, TSC_8_BIT = 0b1 } mxc_ts_cmd_mode_t;

typedef struct {
    mxc_i2c_regs_t *regs; // The I2C instance the touchscreen controller is connected to
    mxc_gpio_cfg_t gpio; // The I2C pins: SDA and SCL, must define all them
    unsigned int freq; // The I2C frequency
} mxc_ts_i2c_config;

/************************************************************************************/

// I2C Transport layer functions

/**
 * @brief       Initialize the I2C instance connected to the TouchScreen driver.
 *              Board files must implement this.
 */
extern void TS_I2C_Init(void);

/**
 * @brief       Send a byte of data to the TouchScreen driver over I2C.
 *              Board files must implement this.
 * 
 * @param       datain      Input value to write
 * @param[out]  dataout     Output pointer. This function will decode and save
 *                          the I2C response to this pointer.
 */
extern void TS_I2C_Transmit(uint8_t datain, uint16_t *dataout);

/************************************************************************************/

int MXC_TS_AssignInterruptPin(mxc_gpio_cfg_t pin);

/**
 * @brief      Used to register hw related configuration, need to be called before MXC_TS_Init()
 *
 * @param      i2c_config   Touch screen I2C configuration
 * @param      int_pin      The GPIO pin configuration for the touchscreen controller's interrupt pin
 *
 * @return     See \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TS_PreInit(mxc_ts_i2c_config *i2c_config, mxc_gpio_cfg_t *int_pin);

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

#endif // LIBRARIES_MISCDRIVERS_TOUCHSCREEN_TSC2007_H_
