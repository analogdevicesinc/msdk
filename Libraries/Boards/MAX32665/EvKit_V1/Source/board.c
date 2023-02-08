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
#include "mxc_device.h"
#include "mxc_sys.h"
#include "mxc_assert.h"
#include "board.h"
#include "uart.h"
#include "gpio.h"
#include "mxc_pins.h"
#include "led.h"
#include "pb.h"
#include "i2c.h"
#include "spi.h"
#include "spixf.h"
#include "Ext_Flash.h"
#include "sharp_mip.h"

/***** Defines *****/
#define DISPLAY_SPI MXC_SPI0
#define DISPLAY_SPI_SPEED 1000000
#define DISPLAY_CS_PORT MXC_GPIO1
#define DISPLAY_CS_PIN MXC_GPIO_PIN_8

#define DISPLAY_ON_OFF_PORT MXC_GPIO1
#define DISPLAY_ON_OFF_PIN MXC_GPIO_PIN_10

#define DISPLAY_HOR_RES (128)
#define DISPLAY_VER_RES (128)

/***** Global Variables *****/
mxc_uart_regs_t *ConsoleUart = MXC_UART_GET_UART(CONSOLE_UART);
extern uint32_t SystemCoreClock;

const mxc_gpio_cfg_t pb_pin[] = {
    { MXC_GPIO1, MXC_GPIO_PIN_6, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIOH },
    { MXC_GPIO1, MXC_GPIO_PIN_7, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIOH },
};
const unsigned int num_pbs = (sizeof(pb_pin) / sizeof(mxc_gpio_cfg_t));

const mxc_gpio_cfg_t led_pin[] = {
    { MXC_GPIO1, MXC_GPIO_PIN_14, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH },
    { MXC_GPIO1, MXC_GPIO_PIN_15, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH },
    /* Logical LEDs for Bluetooth debugging */
    { MXC_GPIO0, MXC_GPIO_PIN_14, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH },
    { MXC_GPIO0, MXC_GPIO_PIN_15, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH },
    { MXC_GPIO1, MXC_GPIO_PIN_10, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH },
    { MXC_GPIO1, MXC_GPIO_PIN_12, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH },
    { MXC_GPIO1, MXC_GPIO_PIN_13, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH },
};
const unsigned int num_leds = (sizeof(led_pin) / sizeof(mxc_gpio_cfg_t));

#ifdef ENABLE_DISPLAY

sharp_mip_dev ls013b7dh03_controller;

/******************************************************************************/
static int display_comm_init(void)
{
    int err = 0;

    err = MXC_SPI_Init(DISPLAY_SPI, 1, 0, 1, 0, DISPLAY_SPI_SPEED, MAP_A);
    if (err != E_NO_ERROR) {
        printf("-->Failed master\n");
        return err;
    } else {
        printf("\n-->SPI Master Initialization Complete");
    }

    MXC_SPI_SetDataSize(DISPLAY_SPI, 8);

    MXC_GPIO_SetVSSEL(gpio_cfg_spi0a.port, MXC_GPIO_VSSEL_VDDIOH, gpio_cfg_spi0a.mask);

    mxc_gpio_cfg_t gpio_cfg;
    gpio_cfg.port = DISPLAY_CS_PORT;
    gpio_cfg.mask = DISPLAY_CS_PIN;
    gpio_cfg.pad = MXC_GPIO_PAD_NONE;
    gpio_cfg.func = MXC_GPIO_FUNC_OUT;
    gpio_cfg.vssel = MXC_GPIO_VSSEL_VDDIOH;
    MXC_GPIO_Config(&gpio_cfg);

    return err;
}

/******************************************************************************/
static void SPI_CS(int val)
{
    if (val == 0)
        MXC_GPIO_OutClr(DISPLAY_CS_PORT, DISPLAY_CS_PIN);
    else
        MXC_GPIO_OutSet(DISPLAY_CS_PORT, DISPLAY_CS_PIN);
}

/******************************************************************************/
static int display_comm_write(uint8_t *data, uint32_t data_len)
{
    int error = 0;

    SPI_CS(1);

    mxc_spi_req_t req = { .spi = DISPLAY_SPI,
                          .txData = data,
                          .txLen = data_len,
                          .rxData = NULL,
                          .rxLen = 0,
                          .ssIdx = 0,
                          .ssDeassert = 1,
                          .txCnt = 0,
                          .rxCnt = 0,
                          .completeCB = NULL };
    if ((error = MXC_SPI_MasterTransaction(&req)) != 0) {
        printf("Error writing: %d\n", error);
        return error;
    }
    SPI_CS(0);
    return error;
}
#endif // ENABLE_DISPLAY

/******************************************************************************/
static int ext_flash_board_init(void)
{
    int err;

    err = MXC_SPIXF_Init(0x0B, EXT_FLASH_BAUD);

    if (err == E_NO_ERROR) {
        MXC_SPIXF_Enable();
    }
    return err;
}

/******************************************************************************/
static int ext_flash_board_read(uint8_t *read, unsigned len, unsigned deassert,
                                Ext_Flash_DataLine_t width)
{
    mxc_spixf_req_t req = { deassert, 0, NULL, read, (mxc_spixf_width_t)width, len, 0, 0, NULL };

    if (MXC_SPIXF_Transaction(&req) != len) {
        return E_COMM_ERR;
    }
    return E_NO_ERROR;
}

/******************************************************************************/
static int ext_flash_board_write(const uint8_t *write, unsigned len, unsigned deassert,
                                 Ext_Flash_DataLine_t width)
{
    mxc_spixf_req_t req = { deassert, 0, write, NULL, (mxc_spixf_width_t)width, len, 0, 0, NULL };

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
void mxc_assert(const char *expr, const char *file, int line)
{
#ifdef DEBUG
    printf("MXC_ASSERT %s #%d: (%s)\n", file, line, expr);
#endif
    while (1) {}
}

/******************************************************************************/
int Board_Init(void)
{
    int err;
    Ext_Flash_Config_t exf_cfg = { .init = ext_flash_board_init,
                                   .read = ext_flash_board_read,
                                   .write = ext_flash_board_write,
                                   .clock = ext_flash_clock };

    if ((err = Ext_Flash_Configure(&exf_cfg)) != E_NO_ERROR) {
        return err;
    }

#ifdef ENABLE_DISPLAY
    display_comm_api comm_api = { .init = display_comm_init,
                                  .write = display_comm_write,
                                  .comm_buffer = NULL,
                                  .comm_buffer_len = 0 };
    sharp_mip_init_param_t init_param = { .row = DISPLAY_HOR_RES,
                                          .col = DISPLAY_VER_RES,
                                          .on_off_port = DISPLAY_ON_OFF_PORT,
                                          .on_off_pin = DISPLAY_ON_OFF_PIN };
    if ((err = sharp_mip_configure(&ls013b7dh03_controller, &init_param, &comm_api))) {
        MXC_ASSERT_FAIL();
        return err;
    }
#endif // ENABLE_DISPLAY

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

    if ((err = MXC_UART_Init(ConsoleUart, CONSOLE_BAUD, MAP_A)) != E_NO_ERROR) {
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
void NMI_Handler(void)
{
    __NOP();
}
