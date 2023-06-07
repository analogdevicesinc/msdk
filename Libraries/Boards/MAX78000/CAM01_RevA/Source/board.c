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
#include "camera.h"

/***** Global Variables *****/
mxc_uart_regs_t *ConsoleUart = MXC_UART_GET_UART(CONSOLE_UART);
extern uint32_t SystemCoreClock;

/* GPIO Information
 *
 * 	cam_2v8_en_pin		:	Enables or disables 2.8V supply for the image sensor module
 * 		0 : 2.8V Power supply is turned off
 * 		1 : 2.8V Power supply is turned on
 *
 *	cam_1v2_en_pin		:	Enables or disables 1.2V supply for the image sensor module
 * 		0 : 1.2V Power supply is turned off
 * 		1 : 1.2V Power supply is turned on
 *
 *	cam_strobe_pin		:	Image sensor module strobe input
 *
 *	cam_xsleep_pin		:	Image sensor module low power sleep mode control pin.
 * 		0 : Image sensor module is in sleep mode
 * 		1 : Image sensor module is not in sleep mode
 *
 *	cam_xshutdown_pin	:	Image sensor module reset and power down control pin.
 * 		0 : Image sensor module is powered down.
 * 		1 : Image sensor module is powered on.
 *
 *	cnn_boost_en_pin	:	Enables or disables external regulator for CNN power
 *		0 : External CNN regulator is disabled
 *		1 : External CNN regulator is enabled
 *
 *
 * 	cnn_boost_sw_en_pin	:	Enables or disables external regulator switch for CNN power
 *		0 : External CNN regulator switch is off
 *		1 : External CNN regulator switch is on
 */

const mxc_gpio_cfg_t cnn_boost_en_pin = { MXC_GPIO2, MXC_GPIO_PIN_5, MXC_GPIO_FUNC_OUT,
                                          MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH };
const mxc_gpio_cfg_t cnn_boost_sw_en_pin = { MXC_GPIO2, MXC_GPIO_PIN_4, MXC_GPIO_FUNC_OUT,
                                             MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH };

const mxc_gpio_cfg_t cam_2v8_en_pin = { MXC_GPIO0, MXC_GPIO_PIN_2, MXC_GPIO_FUNC_OUT,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH };
const mxc_gpio_cfg_t cam_1v8_en_pin = { MXC_GPIO0, MXC_GPIO_PIN_12, MXC_GPIO_FUNC_OUT,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH };
const mxc_gpio_cfg_t cam_1v2_en_pin = { MXC_GPIO0, MXC_GPIO_PIN_4, MXC_GPIO_FUNC_OUT,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH };

const mxc_gpio_cfg_t cam_strobe_pin = { MXC_GPIO0, MXC_GPIO_PIN_7, MXC_GPIO_FUNC_IN,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO };

const mxc_gpio_cfg_t cam_xsleep_pin = { MXC_GPIO0, MXC_GPIO_PIN_3, MXC_GPIO_FUNC_OUT,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO };
const mxc_gpio_cfg_t cam_xshutdown_pin = { MXC_GPIO0, MXC_GPIO_PIN_6, MXC_GPIO_FUNC_OUT,
                                           MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO };
const mxc_gpio_cfg_t cam_meter_pin = { MXC_GPIO0, MXC_GPIO_PIN_10, MXC_GPIO_FUNC_OUT,
                                       MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO };
const mxc_gpio_cfg_t cam_clksel_pin = { MXC_GPIO0, MXC_GPIO_PIN_11, MXC_GPIO_FUNC_OUT,
                                        MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO };

const mxc_gpio_cfg_t cam_trig_pin = { MXC_GPIO0, MXC_GPIO_PIN_11, MXC_GPIO_FUNC_OUT,
                                      MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO };

const mxc_gpio_cfg_t pb_pin[] = {
    { MXC_GPIO2, MXC_GPIO_PIN_6, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIOH },
    { MXC_GPIO2, MXC_GPIO_PIN_7, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIOH },
};
const unsigned int num_pbs = (sizeof(pb_pin) / sizeof(mxc_gpio_cfg_t));

const mxc_gpio_cfg_t led_pin[] = {
    { MXC_GPIO2, MXC_GPIO_PIN_0, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH },
    { MXC_GPIO2, MXC_GPIO_PIN_1, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH },
};

const unsigned int num_leds = (sizeof(led_pin) / sizeof(mxc_gpio_cfg_t));

/***** File Scope Variables *****/
// const uart_cfg_t uart_cfg = {
//     UART_PARITY_DISABLE,
//     UART_DATA_SIZE_8_BITS,
//     UART_STOP_1,
//     UART_FLOW_CTRL_DIS,
//     UART_FLOW_POL_DIS,
//     CONSOLE_BAUD
// };

// const sys_cfg_uart_t uart_sys_cfg = {MAP_A,Enable};    // There is no special system configuration parameters for UART on MAX32650
// const sys_cfg_i2c_t i2c_sys_cfg = NULL;     // There is no special system configuration parameters for I2C on MAX32650
// const sys_cfg_spixc_t spixc_sys_cfg = NULL;   // There is no special system configuration parameters for SPIXC on MAX32650

// const spixc_cfg_t mx25_spixc_cfg = {
//     0, //mode
//     0, //ssel_pol
//     1000000 //baud
// };

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

    // Set SWDCLK and SWDIO pads to 3.3V
    // MXC_GPIO0->vssel |= (3 << 28);

    MXC_SIMO->vrego_c = 0x43; // Set CNN voltage

    // Enable GPIO
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO0);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO1);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_GPIO2);

    // Initialize console
    if ((err = Console_Init()) < E_NO_ERROR) {
        return err;
    }

    // Initialize LEDs
    if ((err = LED_Init()) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }

    LED_On(0);
    LED_On(1);

    // Image sensor power supply control signals
    // 2V8
    if (MXC_GPIO_Config(&cam_2v8_en_pin) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }
    MXC_GPIO_OutClr(cam_2v8_en_pin.port, cam_2v8_en_pin.mask);

    // 1V8
    if (MXC_GPIO_Config(&cam_1v8_en_pin) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }
    MXC_GPIO_OutClr(cam_1v8_en_pin.port, cam_1v8_en_pin.mask);

    // 1V2
    if (MXC_GPIO_Config(&cam_1v2_en_pin) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }
    MXC_GPIO_OutClr(cam_1v2_en_pin.port, cam_1v2_en_pin.mask);

    // External CNN power supply control
    if (MXC_GPIO_Config(&cnn_boost_en_pin) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }
    MXC_GPIO_OutClr(cnn_boost_en_pin.port, cnn_boost_en_pin.mask);

    if (MXC_GPIO_Config(&cnn_boost_sw_en_pin) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }
    MXC_GPIO_OutClr(cnn_boost_sw_en_pin.port, cnn_boost_sw_en_pin.mask);

    // XSHUTDOWN signal
    if (MXC_GPIO_Config(&cam_xshutdown_pin) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }
    MXC_GPIO_OutSet(cam_xshutdown_pin.port, cam_xshutdown_pin.mask);

    // 400uS delay, XSHUTDOWN to XSLEEP
    MXC_Delay(USEC(400));

    // XSLEEP signal
    if (MXC_GPIO_Config(&cam_xsleep_pin) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }
    MXC_GPIO_OutSet(cam_xsleep_pin.port, cam_xsleep_pin.mask);

    Camera_Power(1);
    Camera_Sleep(0);

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
    if (on) {
        MXC_GPIO_OutSet(cam_2v8_en_pin.port, cam_2v8_en_pin.mask);
        MXC_GPIO_OutSet(cam_1v8_en_pin.port, cam_1v8_en_pin.mask);
        MXC_GPIO_OutSet(cam_1v2_en_pin.port, cam_1v2_en_pin.mask);

    } else {
        MXC_GPIO_OutClr(cam_2v8_en_pin.port, cam_2v8_en_pin.mask);
        MXC_GPIO_OutClr(cam_1v8_en_pin.port, cam_1v8_en_pin.mask);
        MXC_GPIO_OutClr(cam_1v2_en_pin.port, cam_1v2_en_pin.mask);
    }

    return E_NO_ERROR;
}

int Camera_Sleep(int sleep)
{
    if (sleep) {
        MXC_GPIO_OutClr(cam_xsleep_pin.port, cam_xsleep_pin.mask);

    } else {
        MXC_GPIO_OutSet(cam_xsleep_pin.port, cam_xsleep_pin.mask);
    }

    return E_NO_ERROR;
}
