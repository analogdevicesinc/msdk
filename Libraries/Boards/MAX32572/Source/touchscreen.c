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
 *
 ******************************************************************************/

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "mxc_device.h"
#include "touchscreen.h"
#include "spi.h"
#include "gpio.h"

/************************************ DEFINES ********************************/
#define TS_SPI (MXC_SPI0)
#define TS_INT_GPIO_PIN (MXC_GPIO_PIN_0)
#define TS_INT_GPIO_PORT (MXC_GPIO0)
#define TS_BUSY_GPIO_PIN (MXC_GPIO_PIN_1)
#define TS_BUSY_GPIO_PORT (MXC_GPIO0)

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

/********************************* Static Functions **************************/
static int is_inBox(int x, int y, int x0, int y0, int x1, int y1)
{
    if ((x >= x0) && (x <= x1) && (y >= y0) && (y <= y1)) {
        return 1;
    }

    return 0;
}

static void spi_transmit_tsc2046(mxc_ts_touch_cmd_t datain, uint16_t *dataout)
{
    int i;
    uint8_t rx[2] = { 0, 0 };

    mxc_spi_req_t request = {
        TS_SPI, // spi
        0, // ssIdx
        0, // ssDeassert
        &datain, // txData
        0, // rxData
        1, // txLen
        0 // rxLen
    };

    MXC_SPI_MasterTransaction(&request);

    for (i = 0; i < 100; i++) {
        __asm volatile("nop\n");
    }

    request.ssDeassert = 1;
    request.txData = 0;
    request.rxData = (uint8_t *)(rx);
    request.txLen = 0;
    request.rxLen = 2;

    MXC_SPI_MasterTransaction(&request);

    if (dataout) {
        *dataout = (rx[1] | (rx[0] << 8)) >> 4;
    }
}

static int tsGetXY(uint16_t *x, uint16_t *y)
{
    uint16_t tsX, tsY, tsZ1;
    int ret;

    spi_transmit_tsc2046(TSC_DIFFZ1, &tsZ1);

    if (tsZ1 & 0x7F0) {
        spi_transmit_tsc2046(TSC_DIFFX, &tsX);
        *x = tsX * 320 / 0x7FF;
        spi_transmit_tsc2046(TSC_DIFFY, &tsY);
        *y = tsY * 240 / 0x7FF;

        // Wait Release
        do {
            spi_transmit_tsc2046(TSC_DIFFZ1, &tsZ1);
        } while (tsZ1 & 0x7F0);

        ret = 1;
    } else {
        *x = 0;
        *y = 0;
        ret = 0;
    }

    return ret;
}

static void tsHandler(void)
{
    uint16_t touch_x, touch_y;
    int i;

    MXC_TS_Stop();

    if (tsGetXY(&touch_x, &touch_y)) {
        if (pressed_key == 0) { // wait until prev key process
            for (i = 0; i < TS_MAX_BUTTONS; i++) {
                if (ts_buttons[i].key_code != TS_INVALID_KEY_CODE) {
                    if (is_inBox(touch_x, touch_y, ts_buttons[i].x0, ts_buttons[i].y0,
                                 ts_buttons[i].x1, ts_buttons[i].y1)) {
                        // pressed key
                        pressed_key = ts_buttons[i].key_code;
                        break;
                    }
                }
            }
        }
    }

    MXC_GPIO_ClearFlags(TS_INT_GPIO_PORT, TS_INT_GPIO_PIN);

    MXC_TS_Start();
}

static void ts_gpio_init(void)
{
    mxc_gpio_cfg_t config;

    // Each pin needs to be configured for VDDIOH (3.3V) and no pad
    config.pad = MXC_GPIO_PAD_NONE;
    config.func = MXC_GPIO_FUNC_IN;
    config.vssel = MXC_GPIO_VSSEL_VDDIOH;

    // Touchscreen busy pin
    config.port = TS_BUSY_GPIO_PORT;
    config.mask = TS_BUSY_GPIO_PIN;
    MXC_GPIO_Config(&config);

    // Touchscreen interrupt pin
    config.port = TS_INT_GPIO_PORT;
    config.mask = TS_INT_GPIO_PIN;
    MXC_GPIO_Config(&config);

    MXC_GPIO_RegisterCallback(&config, (mxc_gpio_callback_fn)tsHandler, NULL);
}

static void ts_spi_Init(void)
{
    int master = 1;
    int quadMode = 0;
    int numSlaves = 1;
    int ssPol = 0;
    unsigned int ts_hz = 200000;

    MXC_SPI_Init(TS_SPI, master, quadMode, numSlaves, ssPol, ts_hz);

    // Set each spi pin to select VDDIOH (3.3V)
    gpio_cfg_spi0.port->vssel |= gpio_cfg_spi0.mask;

    MXC_SPI_SetDataSize(TS_SPI, 8);
    MXC_SPI_SetWidth(TS_SPI, SPI_WIDTH_STANDARD);
}

/********************************* Public Functions **************************/
int MXC_TS_Init(void)
{
    int result = E_NO_ERROR;

    // Configure GPIO Pins
    ts_gpio_init();

    // Configure SPI Pins
    ts_spi_Init();

    MXC_TS_RemoveAllButton();

    // Configure touchscreen interrupt
    NVIC_SetPriority(GPIO0_IRQn, 5);
    NVIC_EnableIRQ(GPIO0_IRQn);

    MXC_TS_Stop();

    return result;
}

void MXC_TS_Start(void)
{
    spi_transmit_tsc2046(TSC_START, NULL);
    MXC_GPIO_EnableInt(TS_INT_GPIO_PORT, TS_INT_GPIO_PIN);
}

void MXC_TS_Stop(void)
{
    MXC_GPIO_DisableInt(TS_INT_GPIO_PORT, TS_INT_GPIO_PIN);
    spi_transmit_tsc2046(TSC_STOP, NULL);
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
