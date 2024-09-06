/**
 * @file    FreeRTOSConfig.h
 * @brief   FreeRTOSCOnfig function prototypes and data types.
 */

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

/* Define to prevent redundant inclusion */
#ifndef EXAMPLES_MAX32650_USB_TINYUSB_CDC_MSC_FREERTOS_FREERTOSCONFIG_H_
#define EXAMPLES_MAX32650_USB_TINYUSB_CDC_MSC_FREERTOS_FREERTOSCONFIG_H_

/* **** Includes **** */
#include <stdint.h>
#include "max32650.h"

/**
 * @brief   Application specific definitions.
 * @details These definitions should be adjusted for your particular hardware and
 *          application requirements.
 * @note    THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 *          FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * @details See http://www.freertos.org/a00110.html.
 */

/* CMSIS keeps a global updated with current system clock in Hz */
#define configCPU_CLOCK_HZ ((uint32_t)120000000)

/* Tick-less idle forces a 32768 Hz RTC-derived SysTick source, and a 256 Hz task tick */
//#define configUSE_TICKLESS_IDLE 1
#ifdef configUSE_TICKLESS_IDLE
#define configSYSTICK_CLK_HZ ((uint32_t)32768)
#define configTICK_RATE_HZ ((portTickType)256)
#else
#define configTICK_RATE_HZ ((portTickType)1000)
#endif

#define configTOTAL_HEAP_SIZE ((size_t)(26 * 1024))

#define configMINIMAL_STACK_SIZE ((uint16_t)128)

#define configMAX_PRIORITIES 5
#define configUSE_PREEMPTION 0
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configUSE_CO_ROUTINES 0
#define configUSE_16_BIT_TICKS 0
#define configUSE_MUTEXES 1
#define configSUPPORT_STATIC_ALLOCATION 1

/* Run time and task stats gathering related definitions. */
#define configUSE_TRACE_FACILITY 1
#define configUSE_STATS_FORMATTING_FUNCTIONS 1

/* Software timer related definitions. */
#define configUSE_TIMERS 1
#define configTIMER_TASK_PRIORITY (configMAX_PRIORITIES - 2)
#define configTIMER_QUEUE_LENGTH 32
#define configTIMER_TASK_STACK_DEPTH configMINIMAL_STACK_SIZE

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet 0
#define INCLUDE_vTaskDelete 0
#define INCLUDE_vTaskSuspend 1
#define INCLUDE_vTaskDelayUntil 1
#define INCLUDE_uxTaskPriorityGet 0
#define INCLUDE_vTaskDelay 1

/* # of priority bits (configured in hardware) is provided by CMSIS */
#define configPRIO_BITS __NVIC_PRIO_BITS

/* Priority 7, or 255 as only the top three bits are implemented.  This is the lowest priority. */
#define configKERNEL_INTERRUPT_PRIORITY ((unsigned char)7 << (8 - configPRIO_BITS))

/* Priority 5, or 160 as only the top three bits are implemented. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY ((unsigned char)5 << (8 - configPRIO_BITS))

/* The lowest interrupt priority that can be used in a call to a "set priority" function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY ((1 << configPRIO_BITS) - 1)

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 2

/* Alias the default handler names to match CMSIS weak symbols */
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#ifdef configUSE_TICKLESS_IDLE
#define configRTC_TICK_RATE_HZ ((portTickType)256)
/* Provide routines for tickless idle pre- and post- processing */
void vPreSleepProcessing(uint32_t *);
void vPostSleepProcessing(uint32_t);
#define configPRE_SLEEP_PROCESSING(idletime) vPreSleepProcessing(&idletime);
#define configPOST_SLEEP_PROCESSING(idletime) vPostSleepProcessing(idletime);
#endif

/* FreeRTOS+CLI requires this size to be defined, but we do not use it */
#define configCOMMAND_INT_MAX_OUTPUT_SIZE 1

#endif // EXAMPLES_MAX32650_USB_TINYUSB_CDC_MSC_FREERTOS_FREERTOSCONFIG_H_
