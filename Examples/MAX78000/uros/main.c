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

#include <rmw_microros/rmw_microros.h>

#include "transports.h"

/* FreeRTOS+CLI */
void vRegisterCLICommands(void);

/* Mutual exclusion (mutex) semaphores */
SemaphoreHandle_t xGPIOmutex;

/* Task IDs */
TaskHandle_t cmd_task_id;

/* Enables/disables tick-less mode */
unsigned int disable_tickless = 1;

/* Stringification macros */
#define STRING(x) STRING_(x)
#define STRING_(x) #x

/* Array sizes */
#define CMD_LINE_BUF_SIZE 80
#define OUTPUT_BUF_SIZE 512

/* Defined in freertos_tickless.c */
extern void wutHitSnooze(void);

/* =| vTask0 |============================================
 *
 * This task blinks LED0 at a 0.5Hz rate, and does not
 *  drift due to the use of vTaskDelayUntil(). It may have
 *  jitter, however, due to any higher-priority task or
 *  interrupt causing delays in scheduling.
 *
 * =======================================================
 */
void vTask0(void *pvParameters)
{
    TickType_t xLastWakeTime;
    unsigned int x = LED_OFF;

    /* Get task start time */
    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        /* Protect hardware access with mutex
     *
     * Note: This is not strictly necessary, since MXC_GPIO_SetOutVal() is implemented with bit-band
     * access, which is inherently task-safe. However, for other drivers, this would be required.
     *
     */
        if (xSemaphoreTake(xGPIOmutex, portMAX_DELAY) == pdTRUE) {
            if (x == LED_OFF) {
                x = LED_ON;
            } else {
                x = LED_OFF;
            }
            /* Return the mutex after we have modified the hardware state */
            xSemaphoreGive(xGPIOmutex);
        }
        /* Wait 1 second until next run */
        vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ);
    }
}

/* =| vTask1 |============================================
 *
 * This task blinks LED1 at a 0.5Hz rate, and does not
 *  drift due to the use of vTaskDelayUntil(). It may have
 *  jitter, however, due to any higher-priority task or
 *  interrupt causing delays in scheduling.
 *
 * NOTE: The MAX32660 EV Kit has only 1 LED, so this task
 *  does not blink an LED.
 *
 * =======================================================
 */
void vTask1(void *pvParameters)
{
    TickType_t xLastWakeTime;
    unsigned int x = LED_ON;

    /* Get task start time */
    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        /* Protect hardware access with mutex
     *
     * Note: This is not strictly necessary, since MXC_GPIO_SetOutVal() is implemented with bit-band
     * access, which is inherently task-safe. However, for other drivers, this would be required.
     *
     */
        if (xSemaphoreTake(xGPIOmutex, portMAX_DELAY) == pdTRUE) {
            if (x == LED_OFF) {
                LED_On(0);
                x = LED_ON;
            } else {
                LED_Off(0);
                x = LED_OFF;
            }
            /* Return the mutex after we have modified the hardware state */
            xSemaphoreGive(xGPIOmutex);
        }
        /* Wait 1 second until next run */
        vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ);
    }
}

/* =| vTickTockTask |============================================
 *
 * This task writes the current RTOS tick time to the console
 *
 * =======================================================
 */
void vTickTockTask(void *pvParameters)
{
    TickType_t ticks = 0;
    TickType_t xLastWakeTime;

    /* Get task start time */
    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        ticks = xTaskGetTickCount();
        printf("Uptime is 0x%08x (%u seconds), tickless-idle is %s\n", ticks,
               ticks / configTICK_RATE_HZ, disable_tickless ? "disabled" : "ENABLED");
        vTaskDelayUntil(&xLastWakeTime, (configTICK_RATE_HZ * 10));
    }
}

/***** Functions *****/

/* =| WUT_IRQHandler |==========================
 *
 * Interrupt handler for the wake up timer.
 *
 * =======================================================
 */
void WUT_IRQHandler(void)
{
    MXC_WUT_IntClear();
    NVIC_ClearPendingIRQ(WUT_IRQn);
}

/* =| main |==============================================
 *
 * This program demonstrates FreeRTOS tasks, mutexes,
 *  and the FreeRTOS+CLI extension.
 *
 * =======================================================
 */

// app.c calls usleep, whose prototype is defined in unistd.h
int 	usleep (useconds_t __useconds) 
{
    MXC_Delay(MXC_DELAY_USEC(__useconds));
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
    printf("\n-=- %s FreeRTOS (%s) Demo -=-\n", STRING(TARGET), tskKERNEL_VERSION_NUMBER);
    printf("SystemCoreClock = %d\n", SystemCoreClock);

    printf("Initializing RTC\n");
    if (MXC_RTC_Init(0, 0) != E_NO_ERROR) printf("Failed RTC init\n");
    if (MXC_RTC_Start() != E_NO_ERROR) printf("Failed RTC start\n");

    MXC_Serial_Open(MXC_UART0, 115200);

    uint8_t buffer[16] = { 'D', 'E', 'A', 'D', '\n', '\n' };
    int error = 0;
    uxrCustomTransport transport = {
        .args = &transport_config
    };
    vMXC_Serial_Write((void*)&transport, buffer, 5, &error);
    vMXC_Serial_Read((void*)&transport, buffer, 5, 1000, &error);
    printf("\nplease Jesus\n");

#if 0
    printf("Assigning custom transports\n");
    rmw_uros_set_custom_transport(
        MICROROS_TRANSPORTS_FRAMING_MODE,
        (void *)&transport_cfg,
        MXC_Serial_Open,
        MXC_Serial_Close,
        MXC_Serial_Write,
        MXC_Serial_Read
    );

    /* Create mutexes */
    xGPIOmutex = xSemaphoreCreateMutex();
    if (xGPIOmutex == NULL) {
        printf("xSemaphoreCreateMutex failed to create a mutex.\n");
    } else {
        /* Configure task */
        if ((xTaskCreate(vTask0, (const char *)"Task0", configMINIMAL_STACK_SIZE, NULL,
                         tskIDLE_PRIORITY + 1, NULL) != pdPASS) ||
            (xTaskCreate(vTask1, (const char *)"Task1", configMINIMAL_STACK_SIZE, NULL,
                         tskIDLE_PRIORITY + 1, NULL) != pdPASS) ||
            (xTaskCreate(appMain, "uros_task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS)) {
                            // start microROS task
            printf("xTaskCreate() failed to create a task.\n");
        } else {
            /* Start scheduler */
            printf("Starting scheduler in 1s...\n");
            MXC_Delay(MXC_DELAY_SEC(1));
            vTaskStartScheduler();
        }
    }

    /* This code is only reached if the scheduler failed to start */
    printf("ERROR: FreeRTOS did not start due to above error!\n");
    while (1) {
        __NOP();
    }
#endif

    /* Quiet GCC warnings */
    return -1;
}
