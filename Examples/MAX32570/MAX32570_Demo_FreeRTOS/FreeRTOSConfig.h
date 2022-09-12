/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
 * 
 * This software is protected by copyright laws of the United States and
 * of foreign countries. This material may also be protected by patent laws
 * and technology transfer regulations of the United States and of foreign
 * countries. This software is furnished under a license agreement and/or a
 * nondisclosure agreement and may only be used or reproduced in accordance
 * with the terms of those agreements. Dissemination of this information to
 * any party or parties not specified in the license agreement and/or
 * nondisclosure agreement is expressly prohibited.
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

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "max32570.h"

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
#define configCPU_CLOCK_HZ ((unsigned long)150000000)

/* Tick-less idle forces a 32768 Hz RTC-derived SysTick source, and a 256 Hz task tick */
//#define configUSE_TICKLESS_IDLE     1
#if defined(configUSE_TICKLESS_IDLE) && (configUSE_TICKLESS_IDLE > 0)
#define configSYSTICK_CLK_HZ ((unsigned long)32768)
#define configTICK_RATE_HZ ((portTickType)256)
#else
#define configTICK_RATE_HZ ((portTickType)1000)
#define configTICK_RATE_10ms ((portTickType)10)
#endif

#define configTOTAL_HEAP_SIZE ((size_t)(26 * 1024))

#define configMINIMAL_STACK_SIZE ((unsigned short)128)

#define configMAX_PRIORITIES 5
#define configUSE_PREEMPTION 0
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configUSE_CO_ROUTINES 0
#define configUSE_16_BIT_TICKS 0
#define configUSE_MUTEXES 0

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

#ifdef configUSE_TICKLESS_IDLE
#define configRTC_TICK_RATE_HZ ((portTickType)4096)
/* Provide routines for tickless idle pre- and post- processing */
void vPreSleepProcessing(unsigned long *);
void vPostSleepProcessing(unsigned long);
#define configPRE_SLEEP_PROCESSING(idletime) vPreSleepProcessing(&idletime);
#define configPOST_SLEEP_PROCESSING(idletime) vPostSleepProcessing(idletime);
#endif

/* FreeRTOS+CLI requires this size to be defined, but we do not use it */
#define configCOMMAND_INT_MAX_OUTPUT_SIZE 1

#endif /* FREERTOS_CONFIG_H */
