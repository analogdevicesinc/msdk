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

#ifndef EXAMPLES_MAX32650_CLCD_FREERTOS_FREERTOSCONFIG_H_
#define EXAMPLES_MAX32650_CLCD_FREERTOS_FREERTOSCONFIG_H_

#include <stdint.h>
#include "max32650.h"

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

/* Ensure definitions are only used by the compiler, and not by the assembler. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
extern uint32_t SystemCoreClock;
#endif

#define configCPU_CLOCK_HZ (SystemCoreClock)

#define configTICK_RATE_HZ ((portTickType)1000)
#define configRTC_TICK_RATE_HZ (32768)

#define configTOTAL_HEAP_SIZE ((size_t)(16 * 1024))

#define configMINIMAL_STACK_SIZE ((uint16_t)128)

#define configMAX_PRIORITIES 5
#define configUSE_PREEMPTION 1
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configUSE_CO_ROUTINES 0
#define configUSE_16_BIT_TICKS 0
#define configUSE_MUTEXES 1

/* Run time and task stats gathering related definitions. */
#define configUSE_TRACE_FACILITY 1
#define configUSE_STATS_FORMATTING_FUNCTIONS 1

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

/* Alias the default handler names to match CMSIS weak symbols */
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

/* FreeRTOS+CLI requires this size to be defined, but we do not use it */
#define configCOMMAND_INT_MAX_OUTPUT_SIZE 1

// MSDK Debug flags
#if defined(DEBUG) && DEBUG == 1
// RTOS Stats Timer Functions
extern void ConfigTimerForStats(void);
extern uint32_t GetTimerForStats(void);
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() (ConfigTimerForStats())
#define portGET_RUN_TIME_COUNTER_VALUE() (GetTimerForStats())
#define configGENERATE_RUN_TIME_STATS 1

// RTOS Assert & other debug configuration
#define configASSERT(x)           \
    if ((x) == 0) {               \
        taskDISABLE_INTERRUPTS(); \
        for (;;) {}               \
    }
#define configENABLE_BACKWARD_COMPATIBILITY 1
#define configQUEUE_REGISTRY_SIZE 10
#define configRECORD_STACK_HIGH_ADDRESS 1

#endif

#endif //EXAMPLES_MAX32650_CLCD_FREERTOS_FREERTOSCONFIG_H_
