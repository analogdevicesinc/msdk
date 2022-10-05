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
#include "lpgcr_regs.h"
#include "simo_regs.h"
#include "mxc_delay.h"

/***** Global Variables *****/
mxc_uart_regs_t *ConsoleUart = MXC_UART_GET_UART(CONSOLE_UART);
extern uint32_t SystemCoreClock;

/* GPIO Information
 *
 * 	mic_power_ctrl_pin	:	Controls on-board I2S microphone power
 * 		0 : Microphone is turned off
 * 		1 : Microphone is turned on
 *
 *	codec_clk_en_pin	:	Enables or disables 12.288MHz I2S clock generator
 *		0 : Clock generator is stopped
 *		1 : Clock generator is running
 *
 *	i2s_int_ext_sel_pin	:	Selects on-board (internal) or external (I2S header) operation
 *		0 : I2S bus is connected to on-obard audio codec and microphone
 *		1 : I2S bus is connected to I2S header pins for external devices
 *
 *	cnn_boost_en_pin	:	Enables or disables external regulator for CNN power
 *		0 : External CNN regulator is disabled
 *		1 : External CNN regulator is enabled
 *
 *	mic_ws_sel_pin    	:	On-board (internal) microphone channel select
 *		0 : Data is available when I2S WS signal is low (left channel)
 *		1 : Data is available when I2S WS signal is high (right channel)
 */

const mxc_gpio_cfg_t mic_power_ctrl_pin = { MXC_GPIO0, MXC_GPIO_PIN_12, MXC_GPIO_FUNC_OUT,
                                            MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO };
const mxc_gpio_cfg_t codec_clk_en_pin = { MXC_GPIO1, MXC_GPIO_PIN_0, MXC_GPIO_FUNC_OUT,
                                          MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO };
const mxc_gpio_cfg_t i2s_int_ext_sel_pin = { MXC_GPIO1, MXC_GPIO_PIN_21, MXC_GPIO_FUNC_OUT,
                                             MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO };
const mxc_gpio_cfg_t cnn_boost_en_pin = { MXC_GPIO2, MXC_GPIO_PIN_5, MXC_GPIO_FUNC_OUT,
                                          MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH };
const mxc_gpio_cfg_t mic_ws_sel_pin = { MXC_GPIO0, MXC_GPIO_PIN_18, MXC_GPIO_FUNC_OUT,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO };

const mxc_gpio_cfg_t led_pin[] = {
    { MXC_GPIO2, MXC_GPIO_PIN_0, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH },
    { MXC_GPIO2, MXC_GPIO_PIN_1, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH },
    { MXC_GPIO2, MXC_GPIO_PIN_2, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH },
};
const unsigned int num_leds = (sizeof(led_pin) / sizeof(mxc_gpio_cfg_t));

/******************************************************************************/
void mxc_assert(const char *expr, const char *file, int line)
{
    printf("MXC_ASSERT %s #%d: (%s)\n", file, line, expr);

    while (1) {}
}

/******************************************************************************/
int Board_Init(void)
{
#ifndef __riscv
    int err;

    // Enable GPIO
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO0);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO1);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO2);

    if ((err = Console_Init()) < E_NO_ERROR) {
        return err;
    }

    if ((err = LED_Init()) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }

    // Initialize on-board microphone power control GPIO
    if (MXC_GPIO_Config(&mic_power_ctrl_pin) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }

    // Initialize audio codec clock enable GPIO
    if (MXC_GPIO_Config(&codec_clk_en_pin) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }

    // Initialize I2S internal or external mode selection GPIO
    if (MXC_GPIO_Config(&i2s_int_ext_sel_pin) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }

    // Initialize on-board microphone channel select pin
    if (MXC_GPIO_Config(&mic_ws_sel_pin) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }

    MXC_SIMO->vrego_c = 0x43; // Set CNN voltage

    // Wait for external 1.8V buck-converter to become available
    MXC_Delay(200000);
#endif // __riscv

    return E_NO_ERROR;
}

/******************************************************************************/
int Console_Init(void)
{
    int err;

    if ((err = MXC_UART_Init(ConsoleUart, CONSOLE_BAUD, MXC_UART_IBRO_CLK)) != E_NO_ERROR) {
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
void NMI_Handler(void)
{
    __NOP();
}

#ifdef __riscv
/******************************************************************************/
int Debug_Init(void)
{
    // Set up RISCV JTAG pins (P1[0..3] AF2)
    MXC_GPIO1->en0_clr = 0x0f;
    MXC_GPIO1->en1_set = 0x0f;
    MXC_GPIO1->en2_clr = 0x0f;

    return E_NO_ERROR;
}
#endif // __riscv

/******************************************************************************/
int Camera_Power(int on)
{
    return E_NOT_SUPPORTED;
}

/******************************************************************************/
int Microphone_Power(int on)
{
    if (on)
        MXC_GPIO_OutSet(mic_power_ctrl_pin.port, mic_power_ctrl_pin.mask);
    else
        MXC_GPIO_OutClr(mic_power_ctrl_pin.port, mic_power_ctrl_pin.mask);

    return E_NO_ERROR;
}

/******************************************************************************/
int Audio_Codec_Clock_Enable(int enable)
{
    if (enable)
        MXC_GPIO_OutSet(codec_clk_en_pin.port, codec_clk_en_pin.mask);
    else
        MXC_GPIO_OutClr(codec_clk_en_pin.port, codec_clk_en_pin.mask);

    return E_NO_ERROR;
}

/******************************************************************************/
int Internal_External_I2S_Select(int sel)
{
    if (sel)
        MXC_GPIO_OutSet(i2s_int_ext_sel_pin.port, i2s_int_ext_sel_pin.mask);
    else
        MXC_GPIO_OutClr(i2s_int_ext_sel_pin.port, i2s_int_ext_sel_pin.mask);

    return E_NO_ERROR;
}

/******************************************************************************/
int CNN_Boost_Enable(int enable)
{
    if (enable)
        MXC_GPIO_OutSet(cnn_boost_en_pin.port, cnn_boost_en_pin.mask);
    else
        MXC_GPIO_OutClr(cnn_boost_en_pin.port, cnn_boost_en_pin.mask);

    return E_NO_ERROR;
}

/******************************************************************************/
void SD_Get_Connections(mxc_spi_regs_t **spi, mxc_gpio_regs_t **ssPort, int *ssPin)
{
    return;
}
