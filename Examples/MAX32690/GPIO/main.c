/**
 * @file        main.c
 * @brief       GPIO Example
 * @details
 */

/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc., All Rights Reserved.
 * (now owned by Analog Devices, Inc.)
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
 ******************************************************************************
 *
 * Copyright 2023 Analog Devices, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

/***** Includes *****/
#include <stdio.h>
#include <string.h>
#include "mxc_device.h"
#include "nvic_table.h"
#include "pb.h"
#include "board.h"
#include "gpio.h"
#include "mxc_delay.h"

/***** Definitions *****/
#if defined(EvKit_V1)
#define MXC_GPIO_PORT_IN MXC_GPIO2
#else
#define MXC_GPIO_PORT_IN MXC_GPIO1
#endif
#define MXC_GPIO_PIN_IN MXC_GPIO_PIN_11

#define MXC_GPIO_PORT_OUT MXC_GPIO0
#define MXC_GPIO_PIN_OUT MXC_GPIO_PIN_14

#if defined(EvKit_V1)
#define MXC_GPIO_PORT_INTERRUPT_STATUS MXC_GPIO2
#define MXC_GPIO_PIN_INTERRUPT_STATUS MXC_GPIO_PIN_12
#endif

/***** Globals *****/

/***** Functions *****/

void gpio_callback(void *cbdata)
{
    mxc_gpio_cfg_t *cfg = cbdata;
    MXC_GPIO_OutToggle(cfg->port, cfg->mask);
}

void gpio_isr(void)
{
    MXC_Delay(MXC_DELAY_MSEC(100)); // Debounce
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO_PORT_IN));
}

int main(void)
{
    mxc_gpio_cfg_t gpio_out;
    mxc_gpio_cfg_t gpio_interrupt;
    mxc_gpio_cfg_t gpio_interrupt_status;

    printf("\n\n************************* GPIO Example ***********************\n\n");
    printf("1. This example reads P%d.11 and outputs the same state onto P0.14.\n",
           MXC_GPIO_GET_IDX(MXC_GPIO_PORT_IN));
#if defined(EvKit_V1)
    printf("2. A falling edge interrupt is set up on P2.11. P2.12 toggles when\n");
    printf("   that interrupt occurs.\n\n");
    printf("Connect P4.0->P2.11 to use SW2 to trigger a falling edge interrupt\n");
    printf("on each press.\n");

    /* Setup interrupt status pin as an output so we can toggle it on each interrupt. */
    gpio_interrupt_status.port = MXC_GPIO_PORT_INTERRUPT_STATUS;
    gpio_interrupt_status.mask = MXC_GPIO_PIN_INTERRUPT_STATUS;
    gpio_interrupt_status.pad = MXC_GPIO_PAD_NONE;
    gpio_interrupt_status.func = MXC_GPIO_FUNC_OUT;
    gpio_interrupt_status.vssel = MXC_GPIO_VSSEL_VDDIO;
    gpio_interrupt_status.drvstr = MXC_GPIO_DRVSTR_0;
    MXC_GPIO_Config(&gpio_interrupt_status);
#endif

    /*
     *   Set up interrupt input pin.
     *   Switch on EV kit is open when non-pressed, and grounded when pressed.  Use an internal pull-up so pin
     *     reads high when button is not pressed.
     */
    gpio_interrupt.port = MXC_GPIO_PORT_IN;
    gpio_interrupt.mask = MXC_GPIO_PIN_IN;
    gpio_interrupt.pad = MXC_GPIO_PAD_NONE;
    gpio_interrupt.func = MXC_GPIO_FUNC_IN;
    gpio_interrupt.vssel = MXC_GPIO_VSSEL_VDDIOH;
    gpio_interrupt.drvstr = MXC_GPIO_DRVSTR_0;
    MXC_GPIO_Config(&gpio_interrupt);
    MXC_GPIO_RegisterCallback(&gpio_interrupt, gpio_callback, &gpio_interrupt_status);
    MXC_GPIO_IntConfig(&gpio_interrupt, MXC_GPIO_INT_FALLING);
    MXC_GPIO_EnableInt(gpio_interrupt.port, gpio_interrupt.mask);
    NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(MXC_GPIO_PORT_IN)));
    MXC_NVIC_SetVector(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(MXC_GPIO_PORT_IN)), gpio_isr);

    /* Setup output pin. */
    gpio_out.port = MXC_GPIO_PORT_OUT;
    gpio_out.mask = MXC_GPIO_PIN_OUT;
    gpio_out.pad = MXC_GPIO_PAD_NONE;
    gpio_out.func = MXC_GPIO_FUNC_OUT;
    gpio_out.vssel = MXC_GPIO_VSSEL_VDDIO;
    gpio_out.drvstr = MXC_GPIO_DRVSTR_0;
    MXC_GPIO_Config(&gpio_out);

    while (1) {
        /* Read state of the input pin. */
        if (MXC_GPIO_InGet(gpio_interrupt.port, gpio_interrupt.mask)) {
            /* Input pin was high, set the output pin. */
            MXC_GPIO_OutClr(gpio_out.port, gpio_out.mask);
        } else {
            /* Input pin was low, clear the output pin. */
            MXC_GPIO_OutSet(gpio_out.port, gpio_out.mask);
        }
    }

    return 0;
}
