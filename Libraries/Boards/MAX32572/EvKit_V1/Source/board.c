/*******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc. (now owned by Analog
 * Devices, Inc.), Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary and confidential to Analog Devices, Inc. and its licensors.
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
#include "spi.h"
#include "tft_ssd2119.h"
#include "tsc2007.h"

/***** Global Variables *****/
mxc_uart_regs_t *ConsoleUart = MXC_UART_GET_UART(CONSOLE_UART);
extern uint32_t SystemCoreClock;

const mxc_gpio_cfg_t pb_pin[] = { { MXC_GPIO0, MXC_GPIO_PIN_31, MXC_GPIO_FUNC_IN,
                                    MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIO },
                                  { MXC_GPIO1, MXC_GPIO_PIN_29, MXC_GPIO_FUNC_IN,
                                    MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIO } };
const unsigned int num_pbs = (sizeof(pb_pin) / sizeof(mxc_gpio_cfg_t));

const mxc_gpio_cfg_t led_pin[] = { { MXC_GPIO1, MXC_GPIO_PIN_30, MXC_GPIO_FUNC_OUT,
                                     MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO },
                                   { MXC_GPIO1, MXC_GPIO_PIN_31, MXC_GPIO_FUNC_OUT,
                                     MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO } };
const unsigned int num_leds = (sizeof(led_pin) / sizeof(mxc_gpio_cfg_t));
// clang-format on

/******************************************************************************/
void mxc_assert(const char *expr, const char *file, int line)
{
    printf("MXC_ASSERT %s #%d: (%s)\n", file, line, expr);

    while (1) {}
}

/******************************************************************************/
/** 
 * NOTE: This weak definition is included to support Push Button interrupts in
 *       case the user does not define this interrupt handler in their application.
 **/
__weak void GPIO0_IRQHandler(void)
{
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO0));
}

/******************************************************************************/
/** 
 * NOTE: This weak definition is included to support Push Button/Touchscreen interrupts
 *       in case the user does not define this interrupt handler in their application.
 **/
__weak void GPIO1_IRQHandler(void)
{
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO1));
}

void TS_I2C_Init(void)
{
    MXC_I2C_Init(TS_I2C, 1, 0);
    MXC_I2C_SetFrequency(TS_I2C, TS_I2C_FREQ);
}

void TS_I2C_Transmit(uint8_t datain, uint16_t *dataout)
{
    uint8_t rx[2] = { 0, 0 };
    mxc_i2c_req_t request;

    request.i2c = TS_I2C;
    request.addr = TS_I2C_TARGET_ADDR;
    request.tx_buf = (uint8_t *)(&datain);
    request.rx_buf = NULL;
    request.tx_len = 1;
    request.rx_len = 0;
    request.restart = 0;
    request.callback = NULL;

    // send command
    MXC_I2C_MasterTransaction(&request);

    request.tx_buf = NULL;
    request.rx_buf = (uint8_t *)(rx);
    request.tx_len = 0;
    request.rx_len = 2;

    // receive value
    MXC_I2C_MasterTransaction(&request);

    // convert 16 bits to 12 bits
    if (dataout != NULL) {
        *dataout = (rx[1] | (rx[0] << 8)) >> 4;
    }
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

    /* TFT SPI */
    mxc_tft_spi_config tft_spi_config = {
        .regs = MXC_SPI0,
        .gpio = { MXC_GPIO0, MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3 | MXC_GPIO_PIN_4 | MXC_GPIO_PIN_5,
                  MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0 },
        .freq = 12000000,
        .ss_idx = 0,
    };

    /* TFT reset and backlight signal */
    mxc_gpio_cfg_t tft_reset_pin = { MXC_GPIO0,         MXC_GPIO_PIN_7,        MXC_GPIO_FUNC_OUT,
                                     MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0 };
    mxc_gpio_cfg_t tft_bl_pin = { MXC_GPIO0,         MXC_GPIO_PIN_6,       MXC_GPIO_FUNC_OUT,
                                  MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0 };

    /* Initialize TFT display */
    MXC_TFT_PreInit(&tft_spi_config, &tft_reset_pin, &tft_bl_pin);

    /* Touch screen controller I2C */
    mxc_ts_i2c_config ts_i2c_config = {
        .regs = MXC_I2C0,
        .gpio = { MXC_GPIO0, (MXC_GPIO_PIN_0 | MXC_GPIO_PIN_1), MXC_GPIO_FUNC_ALT1,
                  MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO, MXC_GPIO_DRVSTR_0 },
        .freq = MXC_I2C_STD_MODE,
    };

    /* Touch screen controller interrupt signal */
    mxc_gpio_cfg_t ts_int_pin = { MXC_GPIO1,         MXC_GPIO_PIN_1,       MXC_GPIO_FUNC_IN,
                               MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0 };

    /* Pre-Initialize Touch Screen controller */
    MXC_TS_PreInit(&ts_i2c_config, &ts_int_pin);

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
__weak void NMI_Handler(void)
{
    __NOP();
}
