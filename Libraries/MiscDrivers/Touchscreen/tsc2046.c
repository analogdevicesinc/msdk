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
    mxc_spi_req_t request;

    request.spi = t_spi;
    request.ssIdx = t_ssel;
    request.ssDeassert = 0;
    request.txData = (uint8_t *)(&datain);
    request.rxData = NULL;
    request.txLen = 1;
    request.rxLen = 0;

    MXC_SPI_SetFrequency(t_spi, t_spi_freq);
    MXC_SPI_SetDataSize(t_spi, 8);

    MXC_SPI_MasterTransaction(&request);

    // Wait to clear TS busy signal
    for (i = 0; i < 100; i++) {
        __asm volatile("nop\n");
    }

    request.ssDeassert = 1;
    request.txData = NULL;
    request.rxData = (uint8_t *)(rx);
    request.txLen = 0;
    request.rxLen = 2;

    MXC_SPI_MasterTransaction(&request);

    if (dataout != NULL) {
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

    MXC_GPIO_ClearFlags(int_gpio.port, int_gpio.mask);

    MXC_TS_Start();
}

static void ts_spi_Init(void)
{
    int master = 1;
    int quadMode = 0;
    int numSlaves = 2;
    int ssPol = 0;

#if defined(OLD_SPI_API) // Defined in spi.h file if the driver if first version
    MXC_SPI_Init(t_spi, master, quadMode, numSlaves, ssPol, t_spi_freq);
    // Todo:
    // Missing SS selection.
    // There is not API in driver,
    // Default SS is used for MAX32570 so it works.
#else
    mxc_spi_pins_t ts_pins;

    ts_pins.clock = true;
    ts_pins.ss0 = (t_ssel == 0); ///< Slave select pin 0
    ts_pins.ss1 = (t_ssel == 1); ///< Slave select pin 1
    ts_pins.ss2 = (t_ssel == 2); ///< Slave select pin 2
    ts_pins.miso = true; ///< miso pin
    ts_pins.mosi = true; ///< mosi pin
    ts_pins.sdio2 = false; ///< SDIO2 pin
    ts_pins.sdio3 = false; ///< SDIO3 pin

    MXC_SPI_Init(t_spi, master, quadMode, numSlaves, ssPol, t_spi_freq, ts_pins);
#endif

    // Set VSSEL
    MXC_GPIO_SetVSSEL(t_spi_gpio.port, t_spi_gpio.vssel, t_spi_gpio.mask);
    MXC_SPI_SetDataSize(t_spi, 8);
    MXC_SPI_SetWidth(t_spi, SPI_WIDTH_STANDARD);
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
    //
    MXC_GPIO_RegisterCallback(&int_gpio, (mxc_gpio_callback_fn)tsHandler, NULL);

    // Configure SPI Pins
    ts_spi_Init();

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
    spi_transmit_tsc2046(TSC_START, NULL);
    MXC_GPIO_EnableInt(int_gpio.port, int_gpio.mask);
}

void MXC_TS_Stop(void)
{
    MXC_GPIO_DisableInt(int_gpio.port, int_gpio.mask);
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
