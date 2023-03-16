/**
 * @file    main.c
 * @brief   Static library example
 * @details Calls static library functions to read and toggle GPIOs.
 */

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

/* **** Includes **** */
#include <stdio.h>
#include <stdint.h>
#include "mxc_delay.h"
#include "led.h"
#include "pb.h"

#include "../TestLib/gpiolib.h"

/* ************************************************************************** */
int main(void)
{
    int i;

    printf("\n\n*********************** Static Library Example **********************\n\n");
    printf("This example uses static library functions to read the inputs SW1 and\n");
    printf("SW2 then toggles the red and green LEDs according to the read values.\n");

    while (1) {
        for (i = 0; i < num_pbs && i < num_leds; i++) {
            if (gpio_get(&pb_pin[i])) {
                gpio_set(&led_pin[i]);
            } else {
                gpio_clear(&led_pin[i]);
            }
        }
        MXC_Delay(1000);
    }
}
