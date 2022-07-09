/* *****************************************************************************
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
 **************************************************************************** */

#include <stdio.h>
#include "mxc_sys.h"
#include "mxc_errors.h"
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

/* **** Global Variables **** */
mxc_uart_regs_t * ConsoleUart = MXC_UART_GET_UART(CONSOLE_UART);
extern uint32_t SystemCoreClock;
extern uint8_t  ChipRevision;

const mxc_gpio_cfg_t pb_pin[] = {
    {MXC_GPIO2, MXC_GPIO_PIN_28, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_WEAK_PULL_UP},
    {MXC_GPIO2, MXC_GPIO_PIN_30, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_WEAK_PULL_UP},
};
const unsigned int num_pbs = (sizeof(pb_pin) / sizeof(mxc_gpio_cfg_t));

const mxc_gpio_cfg_t led_pin[] = {
    {MXC_GPIO2, MXC_GPIO_PIN_25, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE},
    {MXC_GPIO2, MXC_GPIO_PIN_26, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE},
};
const unsigned int num_leds = (sizeof(led_pin) / sizeof(mxc_gpio_cfg_t));

const mxc_spixf_cfg_t mx25_spixc_cfg = {
    0, //mode
    0, //ssel_pol
    1000000 //baud
};

/* **** Functions **** */

/******************************************************************************/
static int ext_flash_board_init(void)
{
    return MXC_SPIXF_Init(0, EXT_FLASH_BAUD);
}

/******************************************************************************/
static int ext_flash_board_read(uint8_t* read, unsigned len, unsigned deassert, Ext_Flash_DataLine_t width)
{
    mxc_spixf_req_t req = {deassert, 0, NULL, read, (mxc_spixf_width_t)width, len, 0, 0, NULL};
    
    if (MXC_SPIXF_Transaction(&req) != len) {
        return E_COMM_ERR;
    }
    return E_NO_ERROR;
}

/******************************************************************************/
static int ext_flash_board_write(const uint8_t* write, unsigned len, unsigned deassert, Ext_Flash_DataLine_t width)
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

/* ************************************************************************** */
void mxc_assert(const char *expr, const char *file, int line)
{
    printf("MXC_ASSERT %s #%d: (%s)\n", file, line, expr);
    while (1);
}

/* ************************************************************************** */
int Board_Init(void)
{
    int err;
    Ext_Flash_Config_t exf_cfg = {
                            .init = ext_flash_board_init,
                            .read = ext_flash_board_read,
                            .write = ext_flash_board_write,
                            .clock = ext_flash_clock
                         };

    if ((err = Ext_Flash_Configure(&exf_cfg)) != E_NO_ERROR) {
        return err;
    }

    // Commented out on purpose.  Uncomment for Rev 1,2,3 boards since pmic is installed.
    // if ((err = MAX77650_Init()) != E_NO_ERROR) {
    //     MXC_ASSERT_FAIL();
    //     return err;
    // }
    
    if ((err = Console_Init()) != E_NO_ERROR) {
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

/* ************************************************************************** */
int Console_Init(void)
{
    int err;
    
    if ((err = MXC_UART_Init(ConsoleUart, CONSOLE_BAUD)) != E_NO_ERROR) {
        return err;
    }
    
    return E_NO_ERROR;
}

#if defined ( __GNUC__ )
/* ************************************************************************** */
void NMI_Handler(void)
{
    __NOP();
}
#endif /* __GNUC__ */


/* ************************************************************************** */
int MAX77650_Init(void)
{
    uint8_t data[2];
    int err;
    mxc_i2c_req_t i2c_req;
    
    if ((err = MXC_I2C_Init(MXC_I2C0, 1, 0)) != E_NO_ERROR) {
        return err;
    }

    MXC_I2C_SetFrequency(MXC_I2C0, MXC_I2C_FAST_SPEED);
    
    i2c_req.i2c = MXC_I2C0;
    i2c_req.addr = 0x90;
    i2c_req.tx_buf = data;
    i2c_req.tx_len = 2;
    i2c_req.rx_buf = NULL;
    i2c_req.rx_len = 0;
    i2c_req.restart = 0;
    /* Command PMIC to set VRTC voltage to 1.8V. */
    data[0] = 0x29;
    data[1] = 0x28;
    if ((err = MXC_I2C_MasterTransaction(&i2c_req)) != E_NO_ERROR) {
        return E_COMM_ERR;
    }
    
    /* Command PMIC to set VCORE voltage to 1.1V. */
    data[0] = 0x2B;
    data[1] = 0xD8;
    if ((err = MXC_I2C_MasterTransaction(&i2c_req)) != E_NO_ERROR) {
        return E_COMM_ERR;
    }
    
    /* Command PMIC to set VDDIOH voltage to 3.3V. */
    data[0] = 0x2D;
    data[1] = 0x32;
    if ((err = MXC_I2C_MasterTransaction(&i2c_req)) != E_NO_ERROR) {
        return E_COMM_ERR;
    }
    
    /* Command PMIC to set LDO voltage to 1.8V. */
    data[0] = 0x38;
    data[1] = 0x24;
    if ((err = MXC_I2C_MasterTransaction(&i2c_req)) != E_NO_ERROR) {
        return E_COMM_ERR;
    }
    
    if ((err = MXC_I2C_Shutdown(MXC_I2C0)) != E_NO_ERROR) {
        return err;
    }
    
    return E_NO_ERROR;
    
}


