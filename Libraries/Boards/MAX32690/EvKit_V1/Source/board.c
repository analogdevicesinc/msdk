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
#include "Ext_Flash.h"

/***** Global Variables *****/
mxc_uart_regs_t* ConsoleUart = MXC_UART_GET_UART(CONSOLE_UART);
extern uint32_t SystemCoreClock;

const mxc_gpio_cfg_t pb_pin[] = {
    {MXC_GPIO4, MXC_GPIO_PIN_0, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIO},
};
const unsigned int num_pbs = (sizeof(pb_pin) / sizeof(mxc_gpio_cfg_t));

const mxc_gpio_cfg_t led_pin[] = {
    {MXC_GPIO0, MXC_GPIO_PIN_14, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO},
    {MXC_GPIO2, MXC_GPIO_PIN_12, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO},
    /* Logical LEDs for Bluetooth debug signals */
    {MXC_GPIO0, MXC_GPIO_PIN_7, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO},
    {MXC_GPIO0, MXC_GPIO_PIN_8, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO},
    {MXC_GPIO0, MXC_GPIO_PIN_9, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO},
};
const unsigned int num_leds = (sizeof(led_pin) / sizeof(mxc_gpio_cfg_t));

/* ************************************************************************** */
static int ext_flash_board_init(void)
{
    return MXC_SPIXF_Init(0, EXT_FLASH_BAUD);
}

/* ************************************************************************** */
static int ext_flash_board_read(uint8_t* read, unsigned len, unsigned deassert, Ext_Flash_DataLine_t width)
{
    mxc_spixf_req_t req = {deassert, 0, NULL, read, width, len, 0, 0, NULL};
    
    return MXC_SPIXF_Transaction(&req);
}

/* ************************************************************************** */
static int ext_flash_board_write(const uint8_t* write, unsigned len, unsigned deassert, Ext_Flash_DataLine_t width)
{
    mxc_spixf_req_t req = {deassert, 0, write, NULL, width, len, 0, 0, NULL};
    
    return MXC_SPIXF_Transaction(&req);
}

/* ************************************************************************** */
static int ext_flash_clock(unsigned len, unsigned deassert)
{
    return MXC_SPIXF_Clocks(len, deassert);
}

/******************************************************************************/
void mxc_assert(const char* expr, const char* file, int line)
{
    #ifdef DEBUG
    printf("MXC_ASSERT %s #%d: (%s)\n", file, line, expr);
    #endif

    while (1);
}

/******************************************************************************/
int Board_Init(void)
{
    // int err;
    // Ext_Flash_Config_t exf_cfg = {
    //                         .init = ext_flash_board_init,
    //                         .read = ext_flash_board_read,
    //                         .write = ext_flash_board_write,
    //                         .clock = ext_flash_clock
    //                      };

    // // Enable GPIO
    // MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO0);
    // MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO1);
    // MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO2);
    // MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO3);

    // if ((err = Ext_Flash_Configure(&exf_cfg)) != E_NO_ERROR) {
    //     return err;
    // }

    // if ((err = Console_Init()) < E_NO_ERROR) {
    //     return err;
    // }

    // if ((err = PB_Init()) != E_NO_ERROR) {
    //     MXC_ASSERT_FAIL();
    //     return err;
    // }

    // if ((err = LED_Init()) != E_NO_ERROR) {
    //     MXC_ASSERT_FAIL();
    //     return err;
    // }

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
int Console_PrepForSleep(void)
{
    return MXC_UART_ReadyForSleep(ConsoleUart);
}

/******************************************************************************/
__weak void NMI_Handler(void)
{
    #ifdef DEBUG
    printf("NMI Handler\n");
    #endif
    while(1) {}
}

/******************************************************************************/
__weak void HardFault_Handler(void)
{
    #ifdef DEBUG
    printf("HardFault_Handler\n");
    #endif
    while(1) {}
}


