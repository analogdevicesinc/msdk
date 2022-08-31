/**
 * @file        main.c
 * @brief       GPIO Example
 * @details
 */

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

/***** Includes *****/
#include "board.h"
#include "gpio.h"
#include "mxc_device.h"
#include "nvic_table.h"
#include "pb.h"
#include <stdio.h>
#include <string.h>

/***** Definitions *****/
#define MXC_GPIO_PORT_IN MXC_GPIO2
#define MXC_GPIO_PIN_IN MXC_GPIO_PIN_6

#define MXC_GPIO_PORT_OUT MXC_GPIO2
#define MXC_GPIO_PIN_OUT MXC_GPIO_PIN_4

#define MXC_GPIO_PORT_INTERRUPT_IN MXC_GPIO2
#define MXC_GPIO_PIN_INTERRUPT_IN MXC_GPIO_PIN_7

#define MXC_GPIO_PORT_INTERRUPT_STATUS MXC_GPIO2
#define MXC_GPIO_PIN_INTERRUPT_STATUS MXC_GPIO_PIN_5

/***** Globals *****/

/***** Functions *****/
void gpio_isr(void* cbdata)
{
    mxc_gpio_cfg_t* cfg = cbdata;
    MXC_GPIO_OutToggle(cfg->port, cfg->mask);
}

int main(void)
{
    mxc_gpio_cfg_t gpio_in;
    mxc_gpio_cfg_t gpio_out;
    mxc_gpio_cfg_t gpio_interrupt;
    mxc_gpio_cfg_t gpio_interrupt_status;

    printf("\n\n************************* GPIO Example ***********************\n\n");
    printf("1. This example reads P2.6 and outputs the same state onto P2.4.\n");
    printf("2. An interrupt is set up on P2.7. P2.5 toggles when that\n");
    printf("   interrupt occurs.\n\n");

    /* Setup interrupt status pin as an output so we can toggle it on each interrupt. */
    gpio_interrupt_status.port = MXC_GPIO_PORT_INTERRUPT_STATUS;
    gpio_interrupt_status.mask = MXC_GPIO_PIN_INTERRUPT_STATUS;
    gpio_interrupt_status.pad = MXC_GPIO_PAD_NONE;
    gpio_interrupt_status.func = MXC_GPIO_FUNC_OUT;
    gpio_interrupt_status.vssel = MXC_GPIO_VSSEL_VDDIO;
    MXC_GPIO_Config(&gpio_interrupt_status);

    /*
     *   Set up interrupt on P2.7.
     *   Switch on EV kit is open when non-pressed, and grounded when pressed.  Use an internal
     * pull-up so pin reads high when button is not pressed.
     */
    gpio_interrupt.port = MXC_GPIO_PORT_INTERRUPT_IN;
    gpio_interrupt.mask = MXC_GPIO_PIN_INTERRUPT_IN;
    gpio_interrupt.pad = MXC_GPIO_PAD_PULL_UP;
    gpio_interrupt.func = MXC_GPIO_FUNC_IN;
    gpio_interrupt.vssel = MXC_GPIO_VSSEL_VDDIOH;
    MXC_GPIO_Config(&gpio_interrupt);
    MXC_GPIO_RegisterCallback(&gpio_interrupt, gpio_isr, &gpio_interrupt_status);
    MXC_GPIO_IntConfig(&gpio_interrupt, MXC_GPIO_INT_FALLING);
    MXC_GPIO_EnableInt(gpio_interrupt.port, gpio_interrupt.mask);
    NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(MXC_GPIO_PORT_INTERRUPT_IN)));

    /*
     *   Setup input pin.
     *   Switch on EV kit is open when non-pressed, and grounded when pressed.  Use an internal
     * pull-up so pin reads high when button is not pressed.
     */
    gpio_in.port = MXC_GPIO_PORT_IN;
    gpio_in.mask = MXC_GPIO_PIN_IN;
    gpio_in.pad = MXC_GPIO_PAD_PULL_UP;
    gpio_in.func = MXC_GPIO_FUNC_IN;
    MXC_GPIO_Config(&gpio_in);

    /* Setup output pin. */
    gpio_out.port = MXC_GPIO_PORT_OUT;
    gpio_out.mask = MXC_GPIO_PIN_OUT;
    gpio_out.pad = MXC_GPIO_PAD_NONE;
    gpio_out.func = MXC_GPIO_FUNC_OUT;
    MXC_GPIO_Config(&gpio_out);

    while (1) {
        /* Read state of the input pin. */
        if (MXC_GPIO_InGet(gpio_in.port, gpio_in.mask)) {
            /* Input pin was high, set the output pin. */
            MXC_GPIO_OutSet(gpio_out.port, gpio_out.mask);
        } else {
            /* Input pin was low, clear the output pin. */
            MXC_GPIO_OutClr(gpio_out.port, gpio_out.mask);
        }
    }

    return 0;
}
