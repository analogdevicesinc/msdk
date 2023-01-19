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

/*******************************      INCLUDES    ****************************/
#include <stdio.h>
#include <stdint.h>

#include "terminal.h"
#include "platform.h"

#include "uart.h"
#include "gpio.h"
#include "mxc_delay.h"

/*******************************      DEFINES     ****************************/
#define TARGET_COM_PORT MXC_UART1

/******************************* Type Definitions ****************************/

/*******************************    Variables   ****************************/

/******************************* Static Functions ****************************/

/******************************* Public Functions ****************************/
/*
 * 	UART
 */
int plt_uart_init(void)
{
    int ret = 0;

    ret = MXC_UART_Init(TARGET_COM_PORT, 115200, MXC_UART_APB_CLK);

    return ret;
}

int plt_uart_write(const unsigned char *src, unsigned int len, unsigned int to)
{
    (void)to;

    int ret = 0;
    mxc_uart_req_t req;

    req.uart = TARGET_COM_PORT;
    req.txData = (unsigned char *)src;
    req.txLen = len;
    req.rxData = NULL;
    req.rxLen = 0;
    req.txCnt = 0;
    req.rxCnt = 0;
    req.callback = NULL;

    /*
	 * MXC_UART_Write(..) function does not check fifo status
	 * so that MXC_UART_Transaction is used instead of MXC_UART_Write
	 */
    ret = MXC_UART_Transaction(&req);

    return ret;
}

int plt_uart_read(unsigned char *dst, unsigned int len, unsigned int to)
{
    int ret = 0;
    int val;

    to *= 1000; //
    while (len) {
        val = MXC_UART_ReadCharacterRaw(TARGET_COM_PORT);
        if (val >= 0) {
            *dst++ = (unsigned char)val;
            --len;
        } else {
            if (to == 0) {
                ret = -1; // Indicates FAIL
                break;
            }
            --to;
        }
    }

    return ret;
}

/*
 *  GPIO
 */
static const mxc_gpio_cfg_t bl_pins[] = {
    { MXC_GPIO0, MXC_GPIO_PIN_30, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH },
    { MXC_GPIO0, MXC_GPIO_PIN_27, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH },
};

int plt_gpio_init(void)
{
    int i;
    for (i = 0; i < sizeof(bl_pins) / sizeof(bl_pins[0]); i++) {
        MXC_GPIO_Config(&bl_pins[i]);
    }

    plt_gpio_set(0, 1); // set RST pin state to 1
    return 0;
}

void plt_gpio_set(unsigned int idx, int state)
{
    if (state) {
        MXC_GPIO_OutSet(bl_pins[idx].port, bl_pins[idx].mask);
    } else {
        MXC_GPIO_OutClr(bl_pins[idx].port, bl_pins[idx].mask);
    }
}

int plt_gpio_get(unsigned int idx)
{
    return !MXC_GPIO_InGet(bl_pins[idx].port, bl_pins[idx].mask);
}

/*
 *	Delay
 */
void plt_delay_ms(unsigned int ms)
{
    MXC_Delay(ms * 1000UL);
}
