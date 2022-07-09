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



#include "led.h"

/* ************************************************************************** */
__attribute__ ((section(".hpb_cs0_section"))) void hbmc_cs0_function(void)
{
    volatile int i;
    int j;

    for (j = 0; j < 25; j++) {
        MXC_GPIO2->out_set |= led_pin[0].mask;
        for (i = 0; i < 0xFFFFF; i++);
        MXC_GPIO2->out_clr |= led_pin[0].mask;
        for (i = 0; i < 0xFFFFF; i++);
    }
}

/* ************************************************************************** */
__attribute__ ((section(".hpb_cs1_section"))) void hbmc_cs1_function(void)
{
    volatile int i;
    int j;

    for (j = 0; j < 25; j++) {
	    MXC_GPIO2->out_set |= led_pin[0].mask;
	    for (i = 0; i < 0xFFFFF; i++);
        MXC_GPIO2->out_clr |= led_pin[0].mask;
        for(i = 0; i < 0xFFFFF; i++);
    }
}
