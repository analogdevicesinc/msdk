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

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "mxc_device.h"
#include "mxc_delay.h"
#include "tsc2007.h"

/************************************ DEFINES ********************************/
#ifndef FLIP_SCREEN
#define FLIP_SCREEN 0
#endif
#ifndef SWAP_XY
#define SWAP_XY 0
#endif
#ifndef ROTATE_SCREEN
#define ROTATE_SCREEN 0
#endif
#define X_RES_T 320
#define Y_RES_T 240

#define ADC_Z_THRESHOLD 0x7F0
#define ADC_X_MIN 232
#define ADC_X_MAX 3888
#define ADC_Y_MIN 375
#define ADC_Y_MAX 3799

#define NOT_IN_BOX 0
#define IN_BOX 1

/******************************* TYPE DEFINITIONS ****************************/
typedef struct _TS_Buttons_t {
    int x0;
    int y0;
    int x1;
    int y1;
    int key_code;
} TS_Buttons_t;

static TS_Buttons_t ts_buttons[TS_MAX_BUTTONS];
static int pressed_key = 0;
static mxc_i2c_regs_t *t_i2c;
static unsigned int t_i2c_freq;
static mxc_gpio_cfg_t int_gpio;
static mxc_gpio_cfg_t t_i2c_gpio;
unsigned int g_x, g_y = 0;
// Global touchscreen event flag that can be polled by applications if needed.
// The application should clear it to 0 if used.
// It is set to 1 eve
int ts_event = false;

uint16_t tsX, tsY, tsZ1;

static uint8_t tsConstructCommand(mxc_ts_cmd_func_t function, mxc_ts_cmd_pdown_t pdown,
                                  mxc_ts_cmd_mode_t mode)
{
    // Bits D7-D4: C3-C0     => converter function select bits
    // Bits D3-D2: PD1-PD0    => power-down bits
    // Bits D1: M            => mode bit
    // Bits D0: X            => don't care
    uint8_t command = (uint8_t)function << 4;
    command |= (uint8_t)pdown << 2;
    command |= (uint8_t)mode << 1;
    return command;
}

static int isInBox(int x, int y, int x0, int y0, int x1, int y1)
{
    if ((x >= x0) && (x <= x1) && (y >= y0) && (y <= y1)) {
        return IN_BOX;
    }

    return NOT_IN_BOX;
}

static int tsGetXY(uint16_t *x, uint16_t *y)
{
    int ret;

    TS_I2C_Transmit(tsConstructCommand(TSC_MEASURE_Z1, TSC_ADC_ON_IRQ_DIS_0, TSC_12_BIT), &tsZ1);
    if (tsZ1 & ADC_Z_THRESHOLD) {
#if (SWAP_XY == 1)
        TS_I2C_Transmit(tsConstructCommand(TSC_MEASURE_Y, TSC_ADC_ON_IRQ_DIS_0, TSC_12_BIT), &tsX);
        TS_I2C_Transmit(tsConstructCommand(TSC_MEASURE_X, TSC_ADC_ON_IRQ_DIS_0, TSC_12_BIT), &tsY);
#else
        TS_I2C_Transmit(tsConstructCommand(TSC_MEASURE_Y, TSC_ADC_ON_IRQ_DIS_0, TSC_12_BIT), &tsY);
        TS_I2C_Transmit(tsConstructCommand(TSC_MEASURE_X, TSC_ADC_ON_IRQ_DIS_0, TSC_12_BIT), &tsX);
#endif

        // Wait Release
        do {
            TS_I2C_Transmit(tsConstructCommand(TSC_MEASURE_Z1, TSC_ADC_ON_IRQ_DIS_0, TSC_12_BIT),
                            &tsZ1);
        } while (tsZ1 & ADC_Z_THRESHOLD);

        *x = (((tsX - ADC_X_MIN) * X_RES_T) / (ADC_X_MAX - ADC_X_MIN));
        *y = (((tsY - ADC_Y_MIN) * Y_RES_T) / (ADC_Y_MAX - ADC_Y_MIN));

#if (FLIP_SCREEN == 1)
        *x = X_RES_T - *x;
        *y = Y_RES_T - *y;
#elif (ROTATE_SCREEN == 1)
        uint16_t swap = *x;
        *x = Y_RES_T - *y - 1;
        *y = swap;
#endif
        ret = 1;

    } else {
#if (FLIP_SCREEN == 1)
        *x = X_RES_T;
        *y = Y_RES_T;
#elif (ROTATE_SCREEN == 1)
        *x = Y_RES_T;
        *y = X_RES_T;
#else
        *x = 0;
        *y = 0;
#endif

        ret = 0;
    }

    // power down ADC and enable IRQ for next touch
    TS_I2C_Transmit(tsConstructCommand(TSC_MEASURE_TEMP0, TSC_POWER_DOWN_IRQ_EN, TSC_12_BIT), NULL);

    return ret;
}

static void tsHandler(void)
{
    uint32_t i;

    MXC_TS_Stop();

    if (tsGetXY((uint16_t *)&g_x, (uint16_t *)&g_y)) {
        ts_event = true;
        if (pressed_key == 0) { // wait until prev key process
            for (i = 0; i < TS_MAX_BUTTONS; i++) {
                if (ts_buttons[i].key_code != TS_INVALID_KEY_CODE) {
                    if (isInBox(g_x, g_y, ts_buttons[i].x0, ts_buttons[i].y0, ts_buttons[i].x1,
                                ts_buttons[i].y1)) {
                        // pressed key
                        pressed_key = ts_buttons[i].key_code;
                        break;
                    }
                }
            }
        }
    }
    MXC_GPIO_ClearFlags(int_gpio.port, int_gpio.mask);
    MXC_TS_Start();
}

int MXC_TS_AssignInterruptPin(mxc_gpio_cfg_t pin)
{
    if (pin.port) {
        int_gpio = pin;
        return E_NO_ERROR;
    } else {
        return E_BAD_PARAM;
    }
}

int MXC_TS_PreInit(mxc_ts_i2c_config *i2c_config, mxc_gpio_cfg_t *int_pin)
{
    int result = E_NO_ERROR;

    if ((int_pin == NULL) || (i2c_config == NULL)) {
        return E_NULL_PTR;
    }

    t_i2c = i2c_config->regs;
    t_i2c_freq = i2c_config->freq;
    int_gpio = *int_pin;
    t_i2c_gpio = i2c_config->gpio;

    return result;
}

int MXC_TS_Init(void)
{
    int result = E_NO_ERROR;

    // Touchscreen interrupt pin
    MXC_GPIO_Config(&int_gpio);
    MXC_GPIO_RegisterCallback(&int_gpio, (mxc_gpio_callback_fn)tsHandler, NULL);
    MXC_GPIO_IntConfig(&int_gpio, MXC_GPIO_INT_FALLING);

    // Configure I2C Pins
    TS_I2C_Init();

    MXC_TS_RemoveAllButton();

    // Configure touchscreen interrupt
#ifndef __riscv
    NVIC_SetPriority(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(int_gpio.port)), 5);
#endif
    NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(int_gpio.port)));

    MXC_TS_Stop();

    return result;
}

void MXC_TS_Start(void)
{
    MXC_GPIO_EnableInt(int_gpio.port, int_gpio.mask);
}

void MXC_TS_Stop(void)
{
    MXC_GPIO_DisableInt(int_gpio.port, int_gpio.mask);
}

void MXC_TS_GetXY(unsigned int *x, unsigned int *y)
{
    *x = g_x;
    *y = g_y;
}

int MXC_TS_GetTSEvent()
{
    return ts_event;
}

void MXC_TS_ClearTSEvent()
{
    ts_event = false;
}

int MXC_TS_AddButton(int x0, int y0, int x1, int y1, int on_press_expected_code)
{
    int index;

    for (index = TS_MAX_BUTTONS - 1; index >= 0; index--) {
        if (ts_buttons[index].key_code == TS_INVALID_KEY_CODE) {
            ts_buttons[index].x0 = x0;
            ts_buttons[index].y0 = y0;
            ts_buttons[index].x1 = x1;
            ts_buttons[index].y1 = y1;
            ts_buttons[index].key_code = on_press_expected_code;
            break;
        }
    }

    return index;
}

void MXC_TS_RemoveButton(int x0, int y0, int x1, int y1)
{
    int i;

    for (i = 0; i < TS_MAX_BUTTONS; i++) {
        if (ts_buttons[i].key_code != TS_INVALID_KEY_CODE) {
            if (isInBox(x0, y0, ts_buttons[i].x0, ts_buttons[i].y0, ts_buttons[i].x1,
                        ts_buttons[i].y1)) {
                // clear flag
                ts_buttons[i].key_code = TS_INVALID_KEY_CODE;
            }
        }
    }
}

void MXC_TS_RemoveAllButton(void)
{
    int i;

    for (i = 0; i < TS_MAX_BUTTONS; i++) {
        ts_buttons[i].key_code = TS_INVALID_KEY_CODE;
    }
}

int MXC_TS_GetKey(void)
{
    int key;

    key = pressed_key;
    pressed_key = 0;

    return key;
}
