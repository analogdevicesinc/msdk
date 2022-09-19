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

#include "mxc_device.h"
#include "led.h"

/******************************************************************************/
int LED_Init(void)
{
    int retval = E_NO_ERROR;
    unsigned int i;

    /* setup GPIO for the LED */
    for (i = 0; i < num_leds; i++) {
        LED_Off(i); // Set the output value
        if (MXC_GPIO_Config(&led_pin[i]) != E_NO_ERROR) {
            retval = E_UNKNOWN;
        }
    }

    return retval;
}

//******************************************************************************
void LED_On(unsigned int idx)
{
    MXC_ASSERT(idx < num_leds);
    if (LED_ON == 0)
        MXC_GPIO_OutClr(led_pin[idx].port, led_pin[idx].mask);
    else
        MXC_GPIO_OutSet(led_pin[idx].port, led_pin[idx].mask);
}

//******************************************************************************
void LED_Off(unsigned int idx)
{
    MXC_ASSERT(idx < num_leds);
    if (LED_ON == 0)
        MXC_GPIO_OutSet(led_pin[idx].port, led_pin[idx].mask);
    else
        MXC_GPIO_OutClr(led_pin[idx].port, led_pin[idx].mask);
}

//******************************************************************************
void LED_Toggle(unsigned int idx)
{
    MXC_ASSERT(idx < num_leds);
    MXC_GPIO_OutToggle(led_pin[idx].port, led_pin[idx].mask);
}