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
#include "touchscreen.h"
#include "spi.h"
#include "gpio.h"

/************************************ DEFINES ********************************/
#define TS_SPI1_PINS MXC_GPIO_PIN_21 | MXC_GPIO_PIN_22 | MXC_GPIO_PIN_23 | MXC_GPIO_PIN_26
#define TS_SPI_FREQ  200000 // Hz
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
static mxc_spi_regs_t* t_spi;
static int t_ssel;
static mxc_gpio_cfg_t* int_gpio;
static mxc_gpio_cfg_t* busy_gpio;

/********************************* Static Functions **************************/
static int is_inBox(int x, int y, int x0, int y0, int x1, int y1)
{
    if ((x >= x0) && (x <= x1) && (y >= y0) && (y <= y1)) {
        return 1;
    }

    return 0;
}

static void spi_transmit_tsc2046(mxc_ts_touch_cmd_t datain, unsigned short* dataout)
{
    int i         = 0;
    uint8_t rx[3] = {0, 0, 0};
    mxc_spi_req_t request;

    request.spi        = t_spi;
    request.ssIdx      = t_ssel;
    request.ssDeassert = 0;
    request.txData     = (uint8_t*)(&datain);
    request.rxData     = (uint8_t*)(rx);
    request.txLen      = 1;
    request.rxLen      = 0;
    request.ssDeassert = 1;

    MXC_SPI_SetFrequency(t_spi, TS_SPI_FREQ);
    MXC_SPI_SetDataSize(t_spi, 8);

    MXC_SPI_MasterTransaction(&request);

    // Wait to clear TS busy signal
    for (i = 0; i < 100; i++) { __asm volatile("nop\n"); }

    request.txLen = 3;
    request.rxLen = 3;

    MXC_SPI_MasterTransaction(&request);

    if (dataout != NULL) {
        *dataout = (rx[2] | (rx[1] << 8)) >> 4;
    }
}

static int tsGetXY(unsigned short* x, unsigned short* y)
{
    unsigned short tsX, tsY, tsZ1;
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
        unsigned short swap = *x;
        *x                  = 240 - *y - 1;
        *y                  = swap;
#endif
        ret = 1;

    } else {
#if (FLIP_SCREEN == 1)
        *x = X_RES_T;
        *y = Y_RES_T;
#elif (ROTATE_SCREEN == 1)
        *x                  = Y_RES_T;
        *y                  = X_RES_T;
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
    unsigned short touch_x, touch_y;
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

    MXC_GPIO_ClearFlags(int_gpio->port, int_gpio->mask);

    MXC_TS_Start();
}

static void ts_gpio_init(void)
{
    // Touchscreen busy pin
    MXC_GPIO_Config(busy_gpio);

    // Touchscreen interrupt pin
    MXC_GPIO_Config(int_gpio);

    MXC_GPIO_RegisterCallback(int_gpio, (mxc_gpio_callback_fn)tsHandler, NULL);
}

static void ts_spi_Init(void)
{
    int master         = 1;
    int quadMode       = 0;
    int numSlaves      = 2;
    int ssPol          = 0;
    unsigned int ts_hz = TS_SPI_FREQ;
    mxc_spi_pins_t ts_pins;

    ts_pins.clock = true;
    ts_pins.ss0   = (t_ssel == 0); ///< Slave select pin 0
    ts_pins.ss1   = (t_ssel == 1); ///< Slave select pin 1
    ts_pins.ss2   = (t_ssel == 2); ///< Slave select pin 2
    ts_pins.miso  = true;          ///< miso pin
    ts_pins.mosi  = true;          ///< mosi pin
    ts_pins.sdio2 = false;         ///< SDIO2 pin
    ts_pins.sdio3 = false;         ///< SDIO3 pin

    MXC_SPI_Init(t_spi, master, quadMode, numSlaves, ssPol, ts_hz, ts_pins);

    // Set  SPI1 pins to VDDIOH (3.3V) to be compatible with touch screen
    MXC_GPIO_SetVSSEL(MXC_GPIO0, MXC_GPIO_VSSEL_VDDIOH, TS_SPI1_PINS);
    MXC_SPI_SetDataSize(t_spi, 8);
    MXC_SPI_SetWidth(t_spi, SPI_WIDTH_STANDARD);
}

/********************************* Public Functions **************************/
int MXC_TS_Init(mxc_spi_regs_t* ts_spi, int ss_idx, mxc_gpio_cfg_t* int_pin,
                mxc_gpio_cfg_t* busy_pin)
{
    int result = E_NO_ERROR;

    t_spi     = ts_spi;
    t_ssel    = ss_idx;
    int_gpio  = int_pin;
    busy_gpio = busy_pin;

    // Configure GPIO Pins
    ts_gpio_init();

    // Configure SPI Pins
    ts_spi_Init();

    MXC_TS_RemoveAllButton();

    // Configure touchscreen interrupt
#ifndef __riscv
    NVIC_SetPriority(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(int_gpio->port)), 5);
#endif
    NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(int_gpio->port)));

    MXC_TS_Stop();

    return result;
}

void MXC_TS_Start(void)
{
    spi_transmit_tsc2046(TSC_START, NULL);
    MXC_GPIO_EnableInt(int_gpio->port, int_gpio->mask);
}

void MXC_TS_Stop(void)
{
    MXC_GPIO_DisableInt(int_gpio->port, int_gpio->mask);
    spi_transmit_tsc2046(TSC_STOP, NULL);
}

int MXC_TS_AddButton(int x0, int y0, int x1, int y1, int on_press_expected_code)
{
    int index;

    for (index = TS_MAX_BUTTONS - 1; index >= 0; index--) {
        if (ts_buttons[index].key_code == TS_INVALID_KEY_CODE) {
            ts_buttons[index].x0       = x0;
            ts_buttons[index].y0       = y0;
            ts_buttons[index].x1       = x1;
            ts_buttons[index].y1       = y1;
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

    for (i = 0; i < TS_MAX_BUTTONS; i++) { ts_buttons[i].key_code = TS_INVALID_KEY_CODE; }
}

int MXC_TS_GetKey(void)
{
    int key;

    key         = pressed_key;
    pressed_key = 0;

    return key;
}
