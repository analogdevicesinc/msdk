/*******************************************************************************
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
 ******************************************************************************/

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "mxc_device.h"
#include "tsc2046.h"
#include "spi.h"
#include "gpio.h"

/************************************ DEFINES ********************************/
#ifndef FLIP_SCREEN
#define FLIP_SCREEN 0
#endif
#ifndef ROTATE_SCREEN
#define ROTATE_SCREEN 0
#endif
#define X_RES_T 320
#define Y_RES_T 240

/******************************* TYPE DEFINITIONS ****************************/
typedef struct _TS_Buttons_t {
    int x0;
    int y0;
    int x1;
    int y1;
    int key_code;
} TS_Buttons_t;

/*********************************** VARIABLES *******************************/
static TS_Buttons_t ts_buttons[TS_MAX_BUTTONS];
static int pressed_key = 0;
static mxc_spi_regs_t *t_spi;
static int t_ssel;
static unsigned int t_spi_freq;
static mxc_gpio_cfg_t int_gpio;
static mxc_gpio_cfg_t busy_gpio;
static mxc_gpio_cfg_t t_spi_gpio;
unsigned int g_x, g_y = 0;
// Global touchscreen event flag that can be polled by applications if needed.
// The application should clear it to 0 if used.
// It is set to 1 eve
int ts_event = false;

/********************************* Static Functions **************************/
static int is_inBox(int x, int y, int x0, int y0, int x1, int y1)
{
    if ((x >= x0) && (x <= x1) && (y >= y0) && (y <= y1)) {
        return 1;
    }

    return 0;
}

static int tsGetXY(uint16_t *x, uint16_t *y)
{
    uint16_t tsX, tsY, tsZ1;
    int ret;

    TS_SPI_Transmit(TSC_DIFFZ1, &tsZ1);

    if (tsZ1 & 0x7F0) {
        TS_SPI_Transmit(TSC_DIFFX, &tsX);
        *x = tsX * 320 / 0x7FF;
        TS_SPI_Transmit(TSC_DIFFY, &tsY);
        *y = tsY * 240 / 0x7FF;

        // Wait Release
        do {
            TS_SPI_Transmit(TSC_DIFFZ1, &tsZ1);
        } while (tsZ1 & 0x7F0);

#if (FLIP_SCREEN == 1)
        *x = X_RES_T - *x;
        *y = Y_RES_T - *y;
#elif (ROTATE_SCREEN == 1)
        uint16_t swap = *x;
        *x = 240 - *y - 1;
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

    return ret;
}

static void tsHandler(void)
{
    int i;

    MXC_TS_Stop();

    if (tsGetXY((uint16_t *)&g_x, (uint16_t *)&g_y)) {
        ts_event = true;
        if (pressed_key == 0) { // wait until prev key process
            for (i = 0; i < TS_MAX_BUTTONS; i++) {
                if (ts_buttons[i].key_code != TS_INVALID_KEY_CODE) {
                    if (is_inBox(g_x, g_y, ts_buttons[i].x0, ts_buttons[i].y0, ts_buttons[i].x1,
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

/********************************* Public Functions **************************/
int MXC_TS_PreInit(mxc_ts_spi_config *spi_config, mxc_gpio_cfg_t *int_pin, mxc_gpio_cfg_t *busy_pin)
{
    int result = E_NO_ERROR;

    if ((int_pin == NULL) || (spi_config == NULL)) {
        return -1;
    }

    t_spi = spi_config->regs;
    t_ssel = spi_config->ss_idx;
    t_spi_freq = spi_config->freq;
    int_gpio = *int_pin;
    t_spi_gpio = spi_config->gpio;

    if (busy_pin) {
        busy_gpio = *busy_pin;
    } else {
        busy_gpio.port = NULL; // means not initialized
    }

    return result;
}

int MXC_TS_Init(void)
{
    int result = E_NO_ERROR;

    // Configure GPIO Pins

    if (busy_gpio.port) {
        // Touchscreen busy pin
        MXC_GPIO_Config(&busy_gpio);
    }
    // Touchscreen interrupt pin
    MXC_GPIO_Config(&int_gpio);
    MXC_GPIO_RegisterCallback(&int_gpio, (mxc_gpio_callback_fn)tsHandler, NULL);

    // Configure SPI Pins
    TS_SPI_Init();

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
    TS_SPI_Transmit(TSC_START, NULL);
    MXC_GPIO_EnableInt(int_gpio.port, int_gpio.mask);
}

void MXC_TS_Stop(void)
{
    MXC_GPIO_DisableInt(int_gpio.port, int_gpio.mask);
    TS_SPI_Transmit(TSC_STOP, NULL);
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
            if (is_inBox(x0, y0, ts_buttons[i].x0, ts_buttons[i].y0, ts_buttons[i].x1,
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
