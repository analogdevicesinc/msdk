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
 * @brief   Hello World!
 * @details This example uses the UART to print to a terminal and flashes an LED.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "led.h"
#include "board.h"
#include "mxc_delay.h"
#include "mxc.h"
#include "wut.h"

/***** Definitions *****/
#define MILLISECONDS_WUT 5000

void WUT_IRQHandler()
{
    MXC_WUT_IntClear();
}

// *****************************************************************************
int main(void)
{
    mxc_wut_cfg_t cfg;
    uint32_t ticks;
    
    printf("/************** Wakeup timer example ********************/\n");
    printf("This example is to show how the Wakeup timer is used and configured\n");
    printf("Press PB1 to put the chip into sleep and then the wakeup timer will wake up in %d Miliseconds \n", MILLISECONDS_WUT);

    // Get ticks based off of milliseconds
    MXC_WUT_GetTicks(MILLISECONDS_WUT, MXC_WUT_UNIT_MILLISEC, &ticks);
    
    // config structure for one shot timer to trigger in a number of ticks
    cfg.mode = MXC_WUT_MODE_ONESHOT;
    cfg.cmp_cnt = ticks;
    
    // Init WUT
    MXC_WUT_Init(MXC_WUT_PRES_1);
    
    //Config WUT
    MXC_WUT_Config(&cfg);
    MXC_LP_EnableWUTAlarmWakeup();

    NVIC_EnableIRQ(WUT_IRQn);

    while (1) {
    	if(PB_Get(0) == TRUE) {
			printf("Entering SLEEP mode.\n");

			MXC_WUT_Enable();

            // wait until UART transmit
            while (MXC_UART_ReadyForSleep(MXC_UART_GET_UART(CONSOLE_UART)) != E_NO_ERROR);

			MXC_LP_EnterSleepMode();

			printf("Waking up from SLEEP mode.\n");
    	}
        
    }
}
