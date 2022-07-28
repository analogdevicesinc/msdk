/*******************************************************************************
 * Copyright (C) 2015 Maxim Integrated Products, Inc., All Rights Reserved.
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
 * $Date: 2017-08-10 11:01:15 -0500 (Thu, 10 Aug 2017) $
 * $Revision: 29282 $
 *
 ******************************************************************************/

#include <stdio.h>
#include "mxc_device.h"
#include "mxc_sys.h"
#include "mxc_assert.h"
#include "board.h"
#include "uart.h"
#include "gpio.h"
#include "mxc_pins.h"
#include "led.h"
#include "pb.h"
#include "spixf.h"
#include "i2c.h"
#include "mxc_sys.h"
#include "Ext_Flash.h"
#include "tft_ssd2119.h"
#include "tsc2046.h"

/***** Global Variables *****/
mxc_uart_regs_t* ConsoleUart = MXC_UART_GET_UART(CONSOLE_UART);
extern uint32_t SystemCoreClock;

const mxc_gpio_cfg_t pb_pin[] = {
    {MXC_GPIO3, MXC_GPIO_PIN_7, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIO},
    {MXC_GPIO3, MXC_GPIO_PIN_6, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIO}};
const unsigned int num_pbs = (sizeof(pb_pin) / sizeof(mxc_gpio_cfg_t));

const mxc_gpio_cfg_t led_pin[] = {
    {MXC_GPIO3, MXC_GPIO_PIN_5, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO},
    {MXC_GPIO3, MXC_GPIO_PIN_4, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO}};
const unsigned int num_leds = (sizeof(led_pin) / sizeof(mxc_gpio_cfg_t));

/******************************************************************************/
static int ext_flash_board_init(void)
{
    int err;
    err = MXC_SPIXF_Init(0x0B, EXT_FLASH_BAUD);
    if (err == E_NO_ERROR) {
        MXC_GPIO_SetVSSEL(gpio_cfg_spixf.port, MXC_GPIO_VSSEL_VDDIOH, gpio_cfg_spixf.mask);
        MXC_SPIXF_Enable();
    }
    return err;
}

/******************************************************************************/
static int ext_flash_board_read(uint8_t* read, unsigned len, unsigned deassert,
                                Ext_Flash_DataLine_t width)
{
    mxc_spixf_req_t req = {deassert, 0, NULL, read, (mxc_spixf_width_t)width, len, 0, 0, NULL};
    if (MXC_SPIXF_Transaction(&req) != len) {
        return E_COMM_ERR;
    }
    return E_NO_ERROR;
}

/******************************************************************************/
static int ext_flash_board_write(const uint8_t* write, unsigned len, unsigned deassert,
                                 Ext_Flash_DataLine_t width)
{
    mxc_spixf_req_t req = {deassert, 0, write, NULL, (mxc_spixf_width_t)width, len, 0, 0, NULL};
    if (MXC_SPIXF_Transaction(&req) != len) {
        return E_COMM_ERR;
    }
    return E_NO_ERROR;
}

/******************************************************************************/
static int ext_flash_clock(unsigned len, unsigned deassert)
{
    return MXC_SPIXF_Clocks(len, deassert);
}

/******************************************************************************/
void mxc_assert(const char* expr, const char* file, int line)
{
    printf("MXC_ASSERT %s #%d: (%s)\n", file, line, expr);

    while (1)
        ;
}

/******************************************************************************/
int Board_Init(void)
{
    int err;
    Ext_Flash_Config_t exf_cfg = {.init  = ext_flash_board_init,
                                  .read  = ext_flash_board_read,
                                  .write = ext_flash_board_write,
                                  .clock = ext_flash_clock};

    if ((err = Ext_Flash_Configure(&exf_cfg)) != E_NO_ERROR) {
        return err;
    }

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

    /* Set HW related configureation to TFT display */
    mxc_tft_spi_config tft_spi_config = {
        .regs   = MXC_SPI1,
        .gpio   = {MXC_GPIO0, MXC_GPIO_PIN_3 | MXC_GPIO_PIN_4 | MXC_GPIO_PIN_5 | MXC_GPIO_PIN_31,
                 MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH},
        .freq   = 12000000,
        .ss_idx = 0,
    };

    mxc_gpio_cfg_t tft_reset_pin = {MXC_GPIO1, MXC_GPIO_PIN_17, MXC_GPIO_FUNC_OUT,
                                    MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH};
    mxc_gpio_cfg_t tft_bl_pin = {MXC_GPIO1, MXC_GPIO_PIN_16, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE,
                                 MXC_GPIO_VSSEL_VDDIOH};

    MXC_TFT_PreInit(&tft_spi_config, &tft_reset_pin, &tft_bl_pin);

    /* Set HW related configureation to Touchscreen controller */
    mxc_ts_spi_config ts_spi_config = {
        .regs   = MXC_SPI0,
        .gpio   = {MXC_GPIO0, MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3 | MXC_GPIO_PIN_4 | MXC_GPIO_PIN_5,
                 MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH},
        .freq   = 200000,
        .ss_idx = 0,
    };

    /* Touch screen controller interrupt signal */
    mxc_gpio_cfg_t int_pin = {MXC_GPIO0, MXC_GPIO_PIN_0, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_NONE,
                              MXC_GPIO_VSSEL_VDDIOH};
    /* Touch screen controller busy signal */
    mxc_gpio_cfg_t busy_pin = {MXC_GPIO0, MXC_GPIO_PIN_1, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_NONE,
                               MXC_GPIO_VSSEL_VDDIOH};
    /* Initialize Touch Screen controller */
    MXC_TS_PreInit(&ts_spi_config, &int_pin, &busy_pin);

    return E_NO_ERROR;
}

/******************************************************************************/
int Console_Init(void)
{
    int err;

    if ((err = MXC_UART_Init(ConsoleUart, CONSOLE_BAUD)) != E_NO_ERROR) {
        return err;
    }

    return E_NO_ERROR;
}

int Console_Shutdown(void)
{
    int err;

    if ((err = MXC_UART_Shutdown(ConsoleUart)) != E_NO_ERROR) {
        return err;
    }

    return E_NO_ERROR;
}
/******************************************************************************/
__weak void NMI_Handler(void)
{
    __NOP();
}
