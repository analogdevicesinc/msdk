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

#include <stdio.h>
#include <stdbool.h>
#include "mxc_device.h"
#include "mxc_sys.h"
#include "mxc_assert.h"
#include "mxc_pins.h"
#include "board.h"
#include "uart.h"
#include "gpio.h"
#include "spi.h"
#include "led.h"
#include "pb.h"
#include "tft_st7735s.h"

/***** Global Variables *****/
mxc_uart_regs_t *ConsoleUart = MXC_UART_GET_UART(CONSOLE_UART);
extern uint32_t SystemCoreClock;

const mxc_gpio_cfg_t pb_pin[] = {
    { MXC_GPIO0, MXC_GPIO_PIN_18, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIOH },
};
const unsigned int num_pbs = (sizeof(pb_pin) / sizeof(mxc_gpio_cfg_t));

const mxc_gpio_cfg_t led_pin[] = {
    { MXC_GPIO0, MXC_GPIO_PIN_22, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO },
    { MXC_GPIO0, MXC_GPIO_PIN_23, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO }
};
const unsigned int num_leds = (sizeof(led_pin) / sizeof(mxc_gpio_cfg_t));

/***** File Scope Variables *****/

/******************************************************************************/
void mxc_assert(const char *expr, const char *file, int line)
{
    printf("MXC_ASSERT %s #%d: (%s)\n", file, line, expr);

    while (1) {}
}

/******************************************************************************/
int Board_Init(void)
{
    int err;

    if ((err = Console_Init()) < E_NO_ERROR) {
        return err;
    }

    if ((err = PB_Init()) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }

    if ((err = LED_Init()) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }

    return E_NO_ERROR;
}

/******************************************************************************/
int Console_Init(void)
{
    int err;

    if ((err = MXC_UART_Init(ConsoleUart, CONSOLE_BAUD, MXC_UART_APB_CLK)) != E_NO_ERROR) {
        return err;
    }

    return E_NO_ERROR;
}

/******************************************************************************/
int Console_Shutdown(void)
{
    int err;

    if ((err = MXC_UART_Shutdown(ConsoleUart)) != E_NO_ERROR) {
        return err;
    }

    return E_NO_ERROR;
}

/******************************************************************************/
void TFT_SPI_Init(void)
{
    int spi_speed = 4 * 1000 * 1000; // x*MHz
    int error;

    error = MXC_SPI_Init(MXC_SPI0, 1, 0, 1, 0, spi_speed);
    MXC_ASSERT(error == E_NO_ERROR);

    error = MXC_SPI_SetDataSize(MXC_SPI0, 9);
    MXC_ASSERT(error == E_NO_ERROR);

    error = MXC_SPI_SetWidth(MXC_SPI0, SPI_WIDTH_STANDARD);
    MXC_ASSERT(error == E_NO_ERROR);
}

/******************************************************************************/
void TFT_SPI_Write(uint8_t *datain, uint32_t count, bool data)
{
    mxc_spi_req_t req;
    uint8_t spibuf[(1 + LINEBUF_SIZE) * 2], *bptr;
    unsigned int txlen;
    int error;

    MXC_ASSERT(!((count > 0) && (datain == NULL)) || (count > (1 + LINEBUF_SIZE)));

    bptr = spibuf;
    txlen = 0;

    /* The txlen++ is _not_ an error. Since the data is 9 bits, it is held in two bytes */
    while (count--) {
        *bptr++ = *datain++;
        if (data) {
            *bptr++ = 1; /* DATA */
        } else {
            *bptr++ = 0; /* CMD */
        }
        txlen++;
    }

    req.spi = MXC_SPI0;
    req.txData = (uint8_t *)spibuf;
    req.rxData = NULL;
    req.txLen = txlen;
    req.rxLen = 0;
    req.ssIdx = 0;
    req.ssDeassert = 1;
    req.txCnt = 0;
    req.rxCnt = 0;
    req.completeCB = NULL;

    error = MXC_SPI_MasterTransaction(&req);
    MXC_ASSERT(error == E_NO_ERROR);
}

/******************************************************************************/
void NMI_Handler(void)
{
    __NOP();
}
