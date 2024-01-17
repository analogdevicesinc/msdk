/******************************************************************************
 *
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

/**
 * @file    main.c
 * @brief   FreeRTOSDemo
 * @details This example demonstrates FreeRTOS.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "task.h"
#include "semphr.h"
#include "led.h"
#include "mxc_delay.h"
#include "rtc.h"
#include "mxc_sys.h"
#include "tmr.h"

#include "rmw_microros/rmw_microros.h"

#include "mxc_microros.h"

#define STRING(x) STRING_(x)
#define STRING_(x) #x

/***** Functions *****/

// app.c calls usleep, whose prototype is defined in unistd.h
int usleep (useconds_t __useconds) 
{
    MXC_TMR_Delay(MXC_TMR0, __useconds);
    return 0;
}

/* glibc complains at the link step about a missing "_gettimeofday" implementation.
As far as I can tell, gettimeofday isn't used...  at least not by micro-ROS.  The structs
are almost identical, so to get rid of the warning we'll just past through to clock_gettime
as well.
*/
int _gettimeofday (struct timeval *__p, void *__tz)
{
    clock_gettime(0, (struct timespec *) __p);
    return 0;
}

/* The micro-ROS library needs an implementation of clock_gettime, defined in <time.h>.
Here, we'll use the RTC. */
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

int main(void)
{
    /* Delay to give debugger a window to connect */
    MXC_Delay(MXC_DELAY_SEC(2));

    LED_On(0);

    /* Print banner (RTOS scheduler not running) */
    printf("\n-=- %s micro-ROS + FreeRTOS (%s) Demo -=-\n", STRING(TARGET), tskKERNEL_VERSION_NUMBER);
    printf("SystemCoreClock = %d\n", SystemCoreClock);

    printf("Initializing RTC\n");
    if (MXC_RTC_Init(0, 0) != E_NO_ERROR) printf("Failed RTC init\n");
    if (MXC_RTC_Start() != E_NO_ERROR) printf("Failed RTC start\n");

    transport_config.uart_instance = configMXC_SERIAL_UART;
    transport_config.baudrate = configMXC_SERIAL_BAUDRATE;

    printf("Assigning custom transports\n");
    rmw_uros_set_custom_transport(
        MICROROS_TRANSPORTS_FRAMING_MODE,
        &transport_config,
        vMXC_Serial_Open,
        vMXC_Serial_Close,
        vMXC_Serial_Write,
        vMXC_Serial_Read
    );

    if ((xTaskCreate(appMain, "appMain", configAPPMAIN_STACK_DEPTH, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS)) {
        printf("xTaskCreate() failed to create a task.\n");
    } else {
        /* Start scheduler */        
        printf("Starting scheduler in 1s...\n");
        MXC_Delay(MXC_DELAY_SEC(1));
        LED_Off(0);
        vTaskStartScheduler();
    }

    /* This code is only reached if the scheduler failed to start */
    printf("ERROR: FreeRTOS did not start due to above error!\n");
    while (1) {
        LED_Toggle(0);
        MXC_Delay(MXC_DELAY_SEC(2));
    }

    /* Quiet GCC warnings */
    return -1;
}
