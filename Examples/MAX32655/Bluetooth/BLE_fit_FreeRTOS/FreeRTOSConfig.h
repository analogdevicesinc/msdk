/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
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

#ifndef EXAMPLES_MAX32655_BLUETOOTH_BLE_FIT_FREERTOS_FREERTOSCONFIG_H_
#define EXAMPLES_MAX32655_BLUETOOTH_BLE_FIT_FREERTOS_FREERTOSCONFIG_H_

#include <stdint.h>
#include "max32655.h"

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

/* CMSIS keeps a global updated with current system clock in Hz */
#define configCPU_CLOCK_HZ ((uint32_t)IPO_FREQ)

#define SLEEP_LED 1
#define DEEPSLEEP_LED 0

// #define configUSE_TICKLESS_IDLE     1

#define configTICK_RATE_HZ ((portTickType)1000)
#define configRTC_TICK_RATE_HZ (32768)

#define configTOTAL_HEAP_SIZE ((size_t)(100 * 1024))

#define configMINIMAL_STACK_SIZE ((uint16_t)128)

#define configMAX_PRIORITIES 5
#define configUSE_PREEMPTION 1
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configUSE_CO_ROUTINES 0
#define configUSE_16_BIT_TICKS 0
#define configUSE_MUTEXES 1

#define configUSE_TIMERS 1
#define configTIMER_TASK_PRIORITY (configMAX_PRIORITIES - 3)
#define configTIMER_QUEUE_LENGTH 8
#define configTIMER_TASK_STACK_DEPTH configMINIMAL_STACK_SIZE

/* Run time and task stats gathering related definitions. */
#define configGENERATE_RUN_TIME_STATS 1

#if configGENERATE_RUN_TIME_STATS == 1
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()
#define portGET_RUN_TIME_COUNTER_VALUE() xTaskGetTickCount()
#endif

#define configUSE_TRACE_FACILITY 1
#define configUSE_STATS_FORMATTING_FUNCTIONS 1

/* Set the following definitions to 1 to include the API function,or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet 0
#define INCLUDE_vTaskDelete 0
#define INCLUDE_vTaskSuspend 1
#define INCLUDE_vTaskDelayUntil 1
#define INCLUDE_uxTaskPriorityGet 0
#define INCLUDE_vTaskDelay 1

/* # of priority bits (configured in hardware) is provided by CMSIS */
#define configPRIO_BITS __NVIC_PRIO_BITS

/* Priority 7,or 255 as only the top three bits are implemented.  This is the lowest priority. */
#define configKERNEL_INTERRUPT_PRIORITY ((unsigned char)7 << (8 - configPRIO_BITS))

/* Priority 5,or 160 as only the top three bits are implemented. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY ((unsigned char)5 << (8 - configPRIO_BITS))

/* Alias the default handler names to match CMSIS weak symbols */
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#ifdef configUSE_TICKLESS_IDLE
/* Provide routines for tickless idle pre- and post- processing */
void vPreSleepProcessing(uint32_t *);
void vPostSleepProcessing(uint32_t);
#define configPRE_SLEEP_PROCESSING(idletime) vPreSleepProcessing(&idletime);
#define configPOST_SLEEP_PROCESSING(idletime) vPostSleepProcessing(idletime);
#endif

/* FreeRTOS+CLI requires this size to be defined,but we do not use it */
#define configCOMMAND_INT_MAX_OUTPUT_SIZE 1

#endif // EXAMPLES_MAX32655_BLUETOOTH_BLE_FIT_FREERTOS_FREERTOSCONFIG_H_
