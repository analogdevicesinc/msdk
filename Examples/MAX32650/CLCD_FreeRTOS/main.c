/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portable.h"
#include "task.h"

#include "mxc_device.h"
#include "mxc_sys.h"
#include "tmr.h"

#include "LCD_Task.h"
#include "UsageTrace.h"

/* Frequency to print the task information to the console */
#define TASK_PRINT_PERIOD_MS 10000

/* Buffer for holding task information.
 * FreeRTOS recommends 40 chars per task. Arbitrarily assume 20 tasks or less
 */
#define MAX_TASK_STATUS 20
static char taskStatusBuf[MAX_TASK_STATUS * 40];

/* Background task handle */
static TaskHandle_t backgroundTask;

/* Local Prototypes */
static void BackgroundTaskBody(void *pvParameters);

/**
 * Define a custom PreInit function to set the system clock to the full 120MHz
 * overriding the default 60MHz.  This is done here prior to the BSP getting
 * initialized so the console baud rate and other features are correct
 *
 * See Libraries\CMSIS\Device\Maxim\MAX32650\Source\system_max32650.c
 */
int PreInit(void)
{
    /* Workaround: Write to SCON register on power up to fix trim issue for SRAM */
    MXC_GCR->scon = (MXC_GCR->scon & ~(MXC_F_GCR_SCON_OVR)) | (MXC_S_GCR_SCON_OVR_1V1);

    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_HIRC96);
    MXC_SYS_Clock_Div(MXC_SYS_SYSTEM_DIV_1);
    return 0;
}

/**
 * Main entry point into our application.
 */
int main(void)
{
    /* Print banner (RTOS scheduler not running) */
    printf("\n-=- FreeRTOS (%s) Demo -=-\n", tskKERNEL_VERSION_NUMBER);
    printf("SystemCoreClock = %d\n", SystemCoreClock);

    /* Initialize the trace helper */
    UsageTraceInit();

    //Create the background task, which also initializes any other system tasks
    xTaskCreate(BackgroundTaskBody, (const char *)"Background", 1024, NULL, tskIDLE_PRIORITY + 1,
                &backgroundTask);
    vTaskStartScheduler();

    //Should never get here!
    while (1) {
        __NOP();
    }
}

/**
 * Hook for handling the timer trace calls. This application just prints to the
 * terminal.
 * @param count - Number of timer ticks
 * @param src - Source of the timer data
 */
void UsageTraceUserHook(uint32_t count, usage_src_t src)
{
    printf("Usage: %d %d\n", count, src);
}

/**
 * Background task body.  Initialize the other tasks at startup, then
 * periodically print the FreeRTOS task statistics
 */
void BackgroundTaskBody(void *pvParameters)
{
    LCD_TaskInitialize();

    while (1) {
        //Periodically print the Task information
        vTaskGetRunTimeStats(taskStatusBuf);
        printf("====================\n");
        printf("Tick Count: %u\n", xTaskGetTickCount());
        printf("%s", taskStatusBuf);
        printf("====================\n");
        vTaskDelay(TASK_PRINT_PERIOD_MS / portTICK_PERIOD_MS);
    }
}
