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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/stat.h>

/* FreeRTOS */
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* FreeRTOS+ */
#include "FreeRTOS_CLI.h"

/* Maxim CMSIS SDK */
#include "mxc_device.h"
#include "board.h"
#include "uart.h"
#include "led.h"

/* FreeRTOS+CLI */
void vRegisterCLICommands(void);

/* Mutual exclusion (mutex) semaphores */
SemaphoreHandle_t xGPIOmutex;

/* Task IDs */
TaskHandle_t cmd_task_id;

/* Enables/disables LP1 tick-less mode */
unsigned int disable_lp1 = 1;

/* Stringification macros */
#define STRING(x) STRING_(x)
#define STRING_(x) #x

/* Console ISR selection */
#if (CONSOLE_UART == 0)
#define UARTx_IRQHandler UART0_IRQHandler
#define UARTx_IRQn UART0_IRQn
#elif (CONSOLE_UART == 1)
#define UARTx_IRQHandler UART1_IRQHandler
#define UARTx_IRQn UART1_IRQn
#else
#error "Please update ISR macro for UART CONSOLE_UART"
#endif
mxc_uart_regs_t *ConsoleUART = MXC_UART_GET_UART(CONSOLE_UART);

/* Array sizes */
#define CMD_LINE_BUF_SIZE 80
#define OUTPUT_BUF_SIZE 512

/* UART access */
#define USE_ASYNC_UART

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

/* =| vTask1 |============================================
 *
 * This task blinks LED1 at a 0.5Hz rate, and does not
 *  drift due to the use of vTaskDelayUntil(). It may have
 *  jitter, however, due to any higher-priority task or
 *  interrupt causing delays in scheduling.
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
        if (xSemaphoreTake(xGPIOmutex, portMAX_DELAY) == pdTRUE) {
            if (x == LED_OFF) {
                LED_On(1);
                x = LED_ON;
            } else {
                LED_Off(1);
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
        printf("Uptime is 0x%08x (%u seconds)\n", ticks, ticks / configTICK_RATE_HZ);
        vTaskDelayUntil(&xLastWakeTime, (configTICK_RATE_HZ * 60));
    }
}

/* =| UART0_IRQHandler |======================================
 *
 * This function overrides the weakly-declared interrupt handler
 *  in system_max326xx.c and is needed for asynchronous UART
 *  calls to work properly
 *
 * ===========================================================
 */
void UART0_IRQHandler(void)
{
    MXC_UART_AsyncHandler(MXC_UART0);
}

/* =| vCmdLineTask_cb |======================================
 *
 * Callback on asynchronous reads to wake the waiting command
 *  processor task
 *
 * ===========================================================
 */
void vCmdLineTask_cb(mxc_uart_req_t *req, int error)
{
    BaseType_t xHigherPriorityTaskWoken;

    /* Wake the task */
    xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(cmd_task_id, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void prvProcessInput(char *pBufCmdLine, unsigned int *pIdxCmdLine, unsigned char cReadChar)
{
    unsigned int uTransLen;
    BaseType_t xMore;
    char output[OUTPUT_BUF_SIZE]; /* Buffer for output */

    /* Process character */
    if (cReadChar == 0x08) {
        /* Backspace */
        if (*pIdxCmdLine > 0) {
            (*pIdxCmdLine)--;
            printf("\x08 \x08");
        }

        fflush(stdout);
    } else if (cReadChar == 0x03) {
        /* ^C abort */
        *pIdxCmdLine = 0;
        printf("^C");
        printf("\ncmd> ");
        fflush(stdout);
    } else if ((cReadChar == '\r') || (cReadChar == '\n')) {
        printf("\r\n");
        /* Null terminate for safety */
        pBufCmdLine[*pIdxCmdLine] = 0x00;

        /* Evaluate */
        do {
            xMore = FreeRTOS_CLIProcessCommand(pBufCmdLine, output, OUTPUT_BUF_SIZE);

            /* If xMore == pdTRUE, then output buffer contains no null termination, so
              *  we know it is OUTPUT_BUF_SIZE. If pdFALSE, we can use strlen.
              */
            for (uTransLen = 0; uTransLen < (xMore == pdTRUE ? OUTPUT_BUF_SIZE : strlen(output));
                 uTransLen++) {
                putchar(*(output + uTransLen));
            }
        } while (xMore != pdFALSE);

        /* New prompt */
        *pIdxCmdLine = 0;
        printf("\ncmd> ");
        fflush(stdout);
    } else if (*pIdxCmdLine < CMD_LINE_BUF_SIZE) {
        putchar(cReadChar);
        pBufCmdLine[(*pIdxCmdLine)++] = cReadChar;
        fflush(stdout);
    } else {
        /* Throw away data and beep terminal */
        putchar(0x07);
        fflush(stdout);
    }
}

static inline void prvFlushUART(mxc_uart_regs_t *uart)
{
    while ((uart->stat & MXC_F_UART_STAT_TXEMPTY) != MXC_F_UART_STAT_TXEMPTY) {}
}

/* =| vCmdLineTask |======================================
 *
 * The command line task provides a prompt on the serial
 *  interface and takes input from the user to evaluate
 *  via the FreeRTOS+CLI parser.
 *
 * NOTE: FreeRTOS+CLI is part of FreeRTOS+ and has
 *  different licensing requirements. Please see
 *  http://www.freertos.org/FreeRTOS-Plus for more information
 *
 * =======================================================
 */
void vCmdLineTask(void *pvParameters)
{
    unsigned char cReadChar;
    unsigned int idxCmdLine; /* Index into buffer */
    unsigned int uTransLen;
    char bufCmdLine[CMD_LINE_BUF_SIZE]; /* Buffer for input */
#ifdef USE_ASYNC_UART
    mxc_uart_req_t async_read_req;
#endif /* USE_ASYNC_UART */

    memset(bufCmdLine, 0, CMD_LINE_BUF_SIZE);
    idxCmdLine = 0;

    /* Register available CLI commands */
    vRegisterCLICommands();

    /* Configure wake-up for GPIO pin corresponding to the UART RX line */
    // LP_ConfigGPIOWakeUpDetect(&console_uart_rx, 0, LP_WEAK_PULL_UP);

    /* Enable UART0 interrupt */
    NVIC_ClearPendingIRQ(UART0_IRQn);
    NVIC_DisableIRQ(UART0_IRQn);
    NVIC_SetPriority(UART0_IRQn, 1);
    NVIC_EnableIRQ(UART0_IRQn);

#ifdef USE_ASYNC_UART
    /* Async read will be used to wake process */
    async_read_req.uart = ConsoleUART;
    async_read_req.txData = NULL;
    async_read_req.rxData = &cReadChar;
    async_read_req.txLen = 0;
    async_read_req.rxLen = 1;
    async_read_req.callback = vCmdLineTask_cb;
#endif /* USE_ASYNC_UART */

    printf("\nEnter 'help' to view a list of available commands.\n");
    printf("cmd> ");
    fflush(stdout);

    while (1) {
#ifdef USE_ASYNC_UART
        /* Wait for previous printf/fflush command. */
        prvFlushUART(ConsoleUART);

        /* Register async read request */
        if (MXC_UART_TransactionAsync(&async_read_req) != E_NO_ERROR) {
            printf("Error registering async request. Command line unavailable.\n");
            vTaskDelay(portMAX_DELAY);
        }

        /* Hang here until ISR wakes us for a character */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Check that we have a valid character */
        if (async_read_req.rxCnt > 0) {
            do {
                /* Process character */
                prvProcessInput(&bufCmdLine[0], &idxCmdLine, cReadChar);

                /* If more characters are ready, process them here */
                uTransLen = 1;
            } while ((MXC_UART_GetRXFIFOAvailable(MXC_UART_GET_UART(CONSOLE_UART)) > 0) &&
                     MXC_UART_Read(MXC_UART_GET_UART(CONSOLE_UART), (uint8_t *)&cReadChar,
                                   (int *)&uTransLen));
        }

#else /* USE_ASYNC_UART */

        while (MXC_UART_GetRXFIFOAvailable(MXC_UART_GET_UART(CONSOLE_UART)) == 0) {}

        uTransLen = 1;
        uTransLen = MXC_UART_Read(MXC_UART_GET_UART(CONSOLE_UART), (uint8_t *)&cReadChar,
                                  (int *)&uTransLen);

        /* Process character */
        prvProcessInput(&bufCmdLine[0], &idxCmdLine, cReadChar);
#endif /* USE_ASYNC_UART */
    }
}

#ifdef configUSE_TICKLESS_IDLE
/* =| freertos_permit_lp1 |===============================
 *
 * Determine if any hardware activity should prevent
 *  low-power tickless operation.
 *
 * =======================================================
 */
int freertos_permit_lp1(void)
{
    static TickType_t last_uart_tick = 0;

    if (disable_lp1 == 1) {
        return E_BUSY;
    }

    /* Did the RX pin wake the processor? */
    // if (LP_IsGPIOWakeUpSource(&console_uart_rx)) {
    //   last_uart_tick = xTaskGetTickCount();
    //   LP_ClearWakeUpFlags();
    // }

    /* Delay entry into LP1 until the tick that RX woke us is not current tick */
    if (last_uart_tick == xTaskGetTickCount()) {
        return E_BUSY;
    }

    return Console_PrepForSleep();
}
#endif

/* =| main |==============================================
 *
 * This program demonstrates FreeRTOS tasks, mutexes,
 *  and the FreeRTOS+CLI extension.
 *
 * =======================================================
 */
int main(void)
{
    /* Print banner (RTOS scheduler not running) */
    printf("\n-=- %s FreeRTOS (%s) Demo -=-\n", STRING(TARGET), tskKERNEL_VERSION_NUMBER);

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
            (xTaskCreate(vTickTockTask, (const char *)"TickTock", 2 * configMINIMAL_STACK_SIZE,
                         NULL, tskIDLE_PRIORITY + 2, NULL) != pdPASS) ||
            (xTaskCreate(vCmdLineTask, (const char *)"CmdLineTask",
                         configMINIMAL_STACK_SIZE + CMD_LINE_BUF_SIZE + OUTPUT_BUF_SIZE, NULL,
                         tskIDLE_PRIORITY + 1, &cmd_task_id) != pdPASS)) {
            printf("xTaskCreate() failed to create a task.\n");
        } else {
            /* Start scheduler */
            printf("Starting scheduler.\n");
            vTaskStartScheduler();
        }
    }

    /* This code is only reached if the scheduler failed to start */
    printf("ERROR: FreeRTOS did not start due to above error!\n");

    while (1) {
        __NOP();
    }

    /* Quiet GCC warnings */
    return -1;
}
