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

/**
 * @file    main.c
 * @brief   Hello World (C++ variant)
 * @details This example uses the UART to print to a terminal and flashes an LED.  It also demonstrates
 * basic class creation and instancing available in C++.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "led.h"
#include "board.h"
#include "mxc_delay.h"

/***** Definitions *****/

/***** Globals *****/

/***** Functions *****/

class HelloClass {
public:
    HelloClass()
    {
        printf("Hello class!\n");
        count = 0;
    }
    void hello_method()
    {
        printf("Hello method!\n");
    }
    void increment_count()
    {
        this->count++;
    }
    int get_count()
    {
        return this->count;
    }

protected:
private:
    int count;
};

// *****************************************************************************
int main()
{
    HelloClass my_class = HelloClass();
    my_class.hello_method();

    printf("Hello World!\n");

    while (1) {
        LED_On(LED1);
        MXC_Delay(500000);
        LED_Off(LED1);
        MXC_Delay(500000);
        printf("count : %d\n", my_class.get_count());
        my_class.increment_count();
    }
}
