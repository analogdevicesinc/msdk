/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
 * 
 * This software is protected by copyright laws of the United States and
 * of foreign countries. This material may also be protected by patent laws
 * and technology transfer regulations of the United States and of foreign
 * countries. This software is furnished under a license agreement and/or a
 * nondisclosure agreement and may only be used or reproduced in accordance
 * with the terms of those agreements. Dissemination of this information to
 * any party or parties not specified in the license agreement and/or
 * nondisclosure agreement is expressly prohibited.
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

    if (LED_ON == 0) {
        MXC_GPIO_OutClr(led_pin[idx].port, led_pin[idx].mask);
    } else {
        MXC_GPIO_OutSet(led_pin[idx].port, led_pin[idx].mask);
    }
}

//******************************************************************************
void LED_Off(unsigned int idx)
{
    MXC_ASSERT(idx < num_leds);

    if (LED_ON == 0) {
        MXC_GPIO_OutSet(led_pin[idx].port, led_pin[idx].mask);
    } else {
        MXC_GPIO_OutClr(led_pin[idx].port, led_pin[idx].mask);
    }
}

//******************************************************************************
void LED_Toggle(unsigned int idx)
{
    MXC_ASSERT(idx < num_leds);
    MXC_GPIO_OutToggle(led_pin[idx].port, led_pin[idx].mask);
}