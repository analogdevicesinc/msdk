/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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
#include "ssd1306.h"

/***** Defines *****/
#define DISPLAY_I2C_MASTER (MXC_I2C2)
#define DISPLAY_I2C_FREQ (400000)
#define DISPLAY_I2C_SLAVE_ADDR (0x3D)
#define DISPLAY_HOR_RES (128)
#define DISPLAY_VER_RES (32)

/***** Global Variables *****/
mxc_uart_regs_t *ConsoleUart = MXC_UART_GET_UART(CONSOLE_UART);
extern uint32_t SystemCoreClock;

const mxc_gpio_cfg_t pb_pin[] = {
    { MXC_GPIO0, MXC_GPIO_PIN_10, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIOH },
};
const unsigned int num_pbs = (sizeof(pb_pin) / sizeof(mxc_gpio_cfg_t));

const mxc_gpio_cfg_t led_pin[] = {
    { MXC_GPIO0, MXC_GPIO_PIN_2, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO },
    { MXC_GPIO0, MXC_GPIO_PIN_3, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO },
    { MXC_GPIO0, MXC_GPIO_PIN_4, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO }
};
const unsigned int num_leds = (sizeof(led_pin) / sizeof(mxc_gpio_cfg_t));

#ifdef ENABLE_DISPLAY

ssd1306_dev cfal12832c_controller;

/***** File Scope Variables *****/
static uint8_t txdata[DISPLAY_HOR_RES];
static uint8_t rxdata[1];

/******************************************************************************/
static int display_comm_init(void)
{
    int err;
    err = MXC_I2C_Init(DISPLAY_I2C_MASTER, 1, 0);
    if (err != E_NO_ERROR) {
        printf("-->Failed master\n");
        return err;
    } else {
        printf("\n-->I2C Master Initialization Complete");
    }
    MXC_I2C_SetFrequency(DISPLAY_I2C_MASTER, DISPLAY_I2C_FREQ);
    MXC_GPIO_SetVSSEL(gpio_cfg_i2c2.port, MXC_GPIO_VSSEL_VDDIOH, gpio_cfg_i2c2.mask);
    return 0;
}

/******************************************************************************/
static int display_comm_write(uint8_t *data, uint32_t data_len)
{
    int error = 0;

    mxc_i2c_req_t reqMaster = { .i2c = DISPLAY_I2C_MASTER,
                                .addr = DISPLAY_I2C_SLAVE_ADDR,
                                .tx_buf = data,
                                .tx_len = data_len,
                                .rx_buf = rxdata,
                                .rx_len = 0,
                                .restart = 0,
                                .callback = NULL };

    if ((error = MXC_I2C_MasterTransaction(&reqMaster)) != 0) {
        printf("Error writing: %d\n", error);
        return error;
    }
    return error;
}
#endif // ENABLE_DISPLAY

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

#ifdef ENABLE_DISPLAY
    display_comm_api comm_api = { .init = display_comm_init,
                                  .write = display_comm_write,
                                  .comm_buffer = txdata,
                                  .comm_buffer_len = DISPLAY_HOR_RES };
    ssd1306_init_param_t init_param = { .row = DISPLAY_HOR_RES, .col = DISPLAY_VER_RES };
    if ((err = ssd1306_configure(&cfal12832c_controller, &init_param, &comm_api))) {
        MXC_ASSERT_FAIL();
        return err;
    }
#endif // ENABLE_DISPLAY

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
