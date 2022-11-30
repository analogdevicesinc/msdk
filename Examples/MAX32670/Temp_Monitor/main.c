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

/**
 * @file    main.c
 * @brief   Low-Power Temp Sensor example.
 * @details This is a demo example for reading and 
 */

/***** Included Files *****/
#include <stdio.h>
#include "board.h"
#include "lp.h"
#include "temp_monitor.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "pb.h"
#include "rtc.h"
#include "uart.h"

/***** Functions *****/
void RTC_IRQHandler(void)
{
	int flags = MXC_RTC_GetFlags();
	MXC_RTC_ClearFlags(flags);

	// RTC TOD alarm --> check temperature
	if(flags & MXC_RTC_INT_FL_LONG) {
		temp_monitor_check_temp();
	}

	// RTC SSEC alarm --> toggle warning light
	if(flags & MXC_RTC_INT_FL_SHORT) {
		temp_monitor_flash_warning_light();
	}
}

void print_temperatures(void* pb)
{
	temp_monitor_print_temps();
}

int main(void)
{
	int err;

	MXC_Delay(MXC_DELAY_SEC(2)); //Prevent bricks

	printf("\n********** Temperature Monitor Demo **********\n");

	// Initialize Temperature Monitor
    if((err = temp_monitor_init()) != E_NO_ERROR) {
        return err;
    }

    // Configure pushbutton as a wakeup source and set interrupt callback
    MXC_LP_EnableGPIOWakeup((mxc_gpio_cfg_t*) &pb_pin[0]);
    PB_RegisterCallback(0, print_temperatures);
    PB_IntEnable(0);

    while (1) {
    	while(MXC_UART_GetActive(MXC_UART_GET_UART(CONSOLE_UART))); //Make sure print statements have finished before sleeping
        MXC_LP_EnterDeepSleepMode(); //Wait for next RTC interrupt
    }

    return E_NO_ERROR;
}
