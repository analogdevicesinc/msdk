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
 * @brief   FreeRTOSDemo
 * @details This example demonstrates FreeRTOS.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "task.h"
#include "semphr.h"
#include "FreeRTOS_CLI.h"
#include "mxc_device.h"
#include "wut.h"
#include "uart.h"
#include "lp.h"
#include "led.h"
#include "board.h"
#include "mxc_delay.h"
#include "rtc.h"
#include "timer.h"

#include <rmw_microros/rmw_microros.h>

#include "transports.h"

#define STRING(x) STRING_(x)
#define STRING_(x) #x

/***** Functions *****/

// app.c calls usleep, whose prototype is defined in unistd.h
int usleep (useconds_t __useconds) 
{
    MXC_TMR_Delay(MXC_TMR0, __useconds);
    return 0;
}

int clock_gettime (clockid_t clock_id, struct timespec *tp)
{
    uint32_t sec = 0, subsec = 0;
    MXC_RTC_GetSubSeconds(&subsec);
    MXC_RTC_GetSeconds(&sec);
    tp->tv_sec = sec;
    long nsec = (long)((subsec / 4096.0f) * 1000000000);
    tp->tv_nsec = nsec;
    return 0;
}

void appMain(void *argument);

int main(void)
{
    /* Delay to prevent bricks */
    MXC_Delay(MXC_DELAY_SEC(2));

    /* Print banner (RTOS scheduler not running) */
    printf("\n-=- %s micro-ROS + FreeRTOS (%s) Demo -=-\n", STRING(TARGET), tskKERNEL_VERSION_NUMBER);
    printf("SystemCoreClock = %d\n", SystemCoreClock);

    printf("Initializing RTC\n");
    if (MXC_RTC_Init(0, 0) != E_NO_ERROR) printf("Failed RTC init\n");
    if (MXC_RTC_Start() != E_NO_ERROR) printf("Failed RTC start\n");

    printf("Assigning custom transports\n");
    rmw_uros_set_custom_transport(
        MICROROS_TRANSPORTS_FRAMING_MODE,
        // MICROROS_TRANSPORTS_PACKET_MODE,
        (void *)&transport_config,
        vMXC_Serial_Open,
        vMXC_Serial_Close,
        vMXC_Serial_Write,
        vMXC_Serial_Read
    );

    if ((xTaskCreate(appMain, "uros_task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS)) {
        printf("xTaskCreate() failed to create a task.\n");
    } else {
        /* Start scheduler */
        printf("Starting scheduler in 1s...\n");
        MXC_Delay(MXC_DELAY_SEC(1));
        vTaskStartScheduler();
    }

    /* This code is only reached if the scheduler failed to start */
    printf("ERROR: FreeRTOS did not start due to above error!\n");
    while (1) {
        __NOP();
    }

    /* Quiet GCC warnings */
    return -1;
}
