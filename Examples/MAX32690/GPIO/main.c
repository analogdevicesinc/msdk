/**
 * @file        main.c
 * @brief       GPIO Example
 * @details
 */

/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
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
#define MXC_GPIO_PORT_IN 2
#define MXC_GPIO_PIN_IN 11
#define MXC_GPIO_PORT_OUT 0
#define MXC_GPIO_PIN_OUT 14
#define MXC_GPIO_PORT_INTERRUPT_STATUS 2
#define MXC_GPIO_PIN_INTERRUPT_STATUS 12
#define INTERRUPT_SWITCH_NAME "SW2"
#elif defined(BOARD_APARD)
#define MXC_GPIO_PORT_IN 1
#define MXC_GPIO_PIN_IN 27
#define MXC_GPIO_PORT_OUT 2
#define MXC_GPIO_PIN_OUT 1
#define MXC_GPIO_PORT_INTERRUPT_STATUS 0
#define MXC_GPIO_PIN_INTERRUPT_STATUS 11
#define INTERRUPT_SWITCH_NAME "S2"
#elif defined(CTBGA_EvKit_V1)
#define MXC_GPIO_PORT_IN 2
#define MXC_GPIO_PIN_IN 31
#define MXC_GPIO_PORT_OUT 0
#define MXC_GPIO_PIN_OUT 14
#define MXC_GPIO_PORT_INTERRUPT_STATUS 0
#define MXC_GPIO_PIN_INTERRUPT_STATUS 15
#define INTERRUPT_SWITCH_NAME "SW2"
#elif defined(FTHR_RevA)
#define MXC_GPIO_PORT_IN 1
#define MXC_GPIO_PIN_IN 14
#define MXC_GPIO_PORT_OUT 0
#define MXC_GPIO_PIN_OUT 14
#define MXC_GPIO_PORT_INTERRUPT_STATUS 2
#define MXC_GPIO_PIN_INTERRUPT_STATUS 24
#define INTERRUPT_SWITCH_NAME "SW3"
#else
#error "EvKit, BOARD_APARD, CTBGA_EvKit_V1, or FTHR_RevA must be defined to run this example."
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
    MXC_GPIO_Handler(MXC_GPIO_PORT_IN);
}

int main(void)
{
    mxc_gpio_cfg_t gpio_out;
    mxc_gpio_cfg_t gpio_interrupt;
    mxc_gpio_cfg_t gpio_interrupt_status;

    printf("\n\n************************* GPIO Example ***********************\n\n");
    printf("1. This example reads P%d.%d and outputs the same state onto P%d.%d.\n",
           MXC_GPIO_PORT_IN, MXC_GPIO_PIN_IN, MXC_GPIO_PORT_OUT, MXC_GPIO_PIN_OUT);
    printf("2. A falling edge interrupt is set up on P%d.%d. P%d.%d toggles when\n",
           MXC_GPIO_PORT_IN, MXC_GPIO_PIN_IN, MXC_GPIO_PORT_INTERRUPT_STATUS,
           MXC_GPIO_PIN_INTERRUPT_STATUS);
    printf("   that interrupt occurs.\n\n");
#if defined(EvKit_V1)
    printf("Connect P4.0->P%d.%d to use %s to trigger a falling edge interrupt\n", MXC_GPIO_PORT_IN,
           MXC_GPIO_PIN_IN, INTERRUPT_SWITCH_NAME);
    printf("on each press.\n");
#else
    printf("Use %s to trigger a falling edge interrupt on each press.\n", INTERRUPT_SWITCH_NAME);
#endif

    /* Setup interrupt status pin as an output so we can toggle it on each interrupt. */
    gpio_interrupt_status.port = MXC_GPIO_GET_GPIO(MXC_GPIO_PORT_INTERRUPT_STATUS);
    gpio_interrupt_status.mask = 1 << MXC_GPIO_PIN_INTERRUPT_STATUS;
    gpio_interrupt_status.pad = MXC_GPIO_PAD_NONE;
    gpio_interrupt_status.func = MXC_GPIO_FUNC_OUT;
    gpio_interrupt_status.vssel = MXC_GPIO_VSSEL_VDDIO;
    gpio_interrupt_status.drvstr = MXC_GPIO_DRVSTR_0;
    MXC_GPIO_Config(&gpio_interrupt_status);

    /*
     *   Set up interrupt input pin.
     *   Switch on EV kit is open when non-pressed, and grounded when pressed.  Use an internal pull-up so pin
     *     reads high when button is not pressed.
     */
    gpio_interrupt.port = MXC_GPIO_GET_GPIO(MXC_GPIO_PORT_IN);
    gpio_interrupt.mask = 1 << MXC_GPIO_PIN_IN;
    gpio_interrupt.pad = MXC_GPIO_PAD_NONE;
    gpio_interrupt.func = MXC_GPIO_FUNC_IN;
    gpio_interrupt.vssel = MXC_GPIO_VSSEL_VDDIOH;
    gpio_interrupt.drvstr = MXC_GPIO_DRVSTR_0;
    MXC_GPIO_Config(&gpio_interrupt);
    MXC_GPIO_RegisterCallback(&gpio_interrupt, gpio_callback, &gpio_interrupt_status);
    MXC_GPIO_IntConfig(&gpio_interrupt, MXC_GPIO_INT_FALLING);
    MXC_GPIO_EnableInt(gpio_interrupt.port, gpio_interrupt.mask);
    NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(MXC_GPIO_PORT_IN));
    MXC_NVIC_SetVector(MXC_GPIO_GET_IRQ(MXC_GPIO_PORT_IN), gpio_isr);

    /* Setup output pin. */
    gpio_out.port = MXC_GPIO_GET_GPIO(MXC_GPIO_PORT_OUT);
    gpio_out.mask = 1 << MXC_GPIO_PIN_OUT;
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
