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

/* Console ISR selection */
#if (CONSOLE_UART==0)
#define UARTx_IRQHandler UART0_IRQHandler
#define UARTx_IRQn UART0_IRQn
mxc_gpio_cfg_t uart_cts = {MXC_GPIO0, MXC_GPIO_PIN_2, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_WEAK_PULL_UP, MXC_GPIO_VSSEL_VDDIOH};
mxc_gpio_cfg_t uart_rts = {MXC_GPIO0, MXC_GPIO_PIN_3, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH};
#elif (CONSOLE_UART==1)
#define UARTx_IRQHandler UART1_IRQHandler
#define UARTx_IRQn UART1_IRQn
mxc_gpio_cfg_t uart_cts = {MXC_GPIO0, MXC_GPIO_PIN_14, MXC_GPIO_FUNC_IN, MXC_GPIO_PAD_WEAK_PULL_UP, MXC_GPIO_VSSEL_VDDIOH};
mxc_gpio_cfg_t uart_rts = {MXC_GPIO0, MXC_GPIO_PIN_15, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH};
#else
#error "Please update ISR macro for UART CONSOLE_UART"
#endif
mxc_uart_regs_t *ConsoleUART = MXC_UART_GET_UART(CONSOLE_UART);

mxc_gpio_cfg_t uart_cts_isr;

/* Array sizes */
#define CMD_LINE_BUF_SIZE       80
#define OUTPUT_BUF_SIZE         512

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
    printf("Uptime is 0x%08x (%u seconds), tickless-idle is %s\n",
           ticks, ticks / configTICK_RATE_HZ,
           disable_tickless ? "disabled" : "ENABLED");
    vTaskDelayUntil(&xLastWakeTime, (configTICK_RATE_HZ * 10));
  }
}


/***** Functions *****/

/* =| UART_IRQHandler |======================================
 *
 * This function overrides the weakly-declared interrupt handler
 *  in system_max326xx.c and is needed for asynchronous UART
 *  calls to work properly
 *
 * ===========================================================
 */
void UARTx_IRQHandler(void)
{
  MXC_UART_AsyncHandler(ConsoleUART);
  wutHitSnooze();
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
  unsigned char tmp;
  unsigned int index;     /* Index into buffer */
  unsigned int x;
  int uartReadLen;
  char buffer[CMD_LINE_BUF_SIZE];        /* Buffer for input */
  char output[OUTPUT_BUF_SIZE];        /* Buffer for output */
  BaseType_t xMore;
  mxc_uart_req_t async_read_req;

  memset(buffer, 0, CMD_LINE_BUF_SIZE);
  index = 0;

  /* Register available CLI commands */
  vRegisterCLICommands();

  /* Enable UARTx interrupt */
  NVIC_ClearPendingIRQ(UARTx_IRQn);
  NVIC_DisableIRQ(UARTx_IRQn);
  NVIC_SetPriority(UARTx_IRQn, 1);
  NVIC_EnableIRQ(UARTx_IRQn);

  /* Async read will be used to wake process */
  async_read_req.uart = ConsoleUART;
  async_read_req.rxData = &tmp;
  async_read_req.rxLen = 1;
  async_read_req.txData = NULL;
  async_read_req.txLen = 0;
  async_read_req.callback = vCmdLineTask_cb;

  printf("\nEnter 'help' to view a list of available commands.\n");
  printf("cmd> ");
  fflush(stdout);
  while (1) {
    /* Register async read request */
    if (MXC_UART_TransactionAsync(&async_read_req) != E_NO_ERROR) {
      printf("Error registering async request. Command line unavailable.\n");
      vTaskDelay(portMAX_DELAY);
    }
    /* Hang here until ISR wakes us for a character */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    /* Check that we have a valid character */
    if (async_read_req.rxCnt > 0) {
      /* Process character */
      do {
        if (tmp == 0x08) {
          /* Backspace */
          if (index > 0) {
            index--;
            printf("\x08 \x08");
          }
          fflush(stdout);
        } else if (tmp == 0x03) {
          /* ^C abort */
          index = 0;
          printf("^C");
          printf("\ncmd> ");
          fflush(stdout);
        } else if ((tmp == '\r') ||
                   (tmp == '\n')) {
          printf("\r\n");
          /* Null terminate for safety */
          buffer[index] = 0x00;
          /* Evaluate */
          do {
            xMore = FreeRTOS_CLIProcessCommand(buffer, output, OUTPUT_BUF_SIZE);
            /* If xMore == pdTRUE, then output buffer contains no null termination, so
             *  we know it is OUTPUT_BUF_SIZE. If pdFALSE, we can use strlen.
             */
            for (x = 0; x < (xMore == pdTRUE ? OUTPUT_BUF_SIZE : strlen(output)) ; x++) {
              putchar(*(output+x));
            }
          } while (xMore != pdFALSE);
          /* New prompt */
          index = 0;
          printf("\ncmd> ");
          fflush(stdout);
        } else if (index < CMD_LINE_BUF_SIZE) {
          putchar(tmp);
          buffer[index++] = tmp;
          fflush(stdout);
        } else {
          /* Throw away data and beep terminal */
          putchar(0x07);
          fflush(stdout);
        }
        uartReadLen = 1;
        /* If more characters are ready, process them here */
      } while ((MXC_UART_GetRXFIFOAvailable(MXC_UART_GET_UART(CONSOLE_UART)) > 0) &&
               (MXC_UART_Read(ConsoleUART, (uint8_t *)&tmp, &uartReadLen) == 0));
    }
  }
}

#if configUSE_TICKLESS_IDLE
/* =| freertos_permit_tickless |==========================
 *
 * Determine if any hardware activity should prevent
 *  low-power tickless operation.
 *
 * =======================================================
 */
int freertos_permit_tickless(void)
{
  if (disable_tickless == 1) {
    return E_BUSY;
  }

  return MXC_UART_GetActive(ConsoleUART);
}
#endif

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
int main(void)
{
  /* Delay to prevent bricks */
  volatile int i;
  for(i = 0; i < 0xFFFFFF; i++) {}
    
  /* Setup manual CTS/RTS to lockout console and wake from deep sleep */
  MXC_GPIO_Config(&uart_cts);
  MXC_GPIO_Config(&uart_rts);

  /* Enable incoming characters */
  MXC_GPIO_OutClr(uart_rts.port, uart_rts.mask);

#if configUSE_TICKLESS_IDLE

  /* Initialize Wakeup timer */
  MXC_WUT_Init(MXC_WUT_PRES_1);
  mxc_wut_cfg_t wut_cfg;
  wut_cfg.mode = MXC_WUT_MODE_COMPARE;
  wut_cfg.cmp_cnt = 0xFFFFFFFF;

  /* Enable WUT as a wakup source */
  MXC_LP_EnableWUTAlarmWakeup();

  NVIC_ClearPendingIRQ(WUT_IRQn);
  NVIC_EnableIRQ(WUT_IRQn);

  /* Configure and start the WUT */
  MXC_WUT_Config(&wut_cfg);
  MXC_WUT_Enable();

  /* Setup CTS interrupt */
  MXC_GPIO_IntConfig(&uart_cts, MXC_GPIO_INT_FALLING);
  MXC_GPIO_EnableInt(uart_cts.port, uart_cts.mask);

  NVIC_ClearPendingIRQ(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(uart_cts.port)));
  NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(uart_cts.port)));
  MXC_LP_EnableGPIOWakeup(&uart_cts);

  /* Initialize CPU Active LED */
  LED_On(1);

#endif

  /* Print banner (RTOS scheduler not running) */
  printf("\n-=- %s FreeRTOS (%s) Demo -=-\n", STRING(TARGET), tskKERNEL_VERSION_NUMBER);
#if configUSE_TICKLESS_IDLE
  printf("Tickless idle is configured. Type 'tickless 1' to enable.\n");
#endif
  printf("SystemCoreClock = %d\n", SystemCoreClock);

  /* Create mutexes */
  xGPIOmutex = xSemaphoreCreateMutex();
  if (xGPIOmutex == NULL) {
    printf("xSemaphoreCreateMutex failed to create a mutex.\n");
  } else {
    /* Configure task */
    if ((xTaskCreate(vTask0, (const char *)"Task0",
                     configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL) != pdPASS) ||
        (xTaskCreate(vTask1, (const char *)"Task1",
                     configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL) != pdPASS) ||
        (xTaskCreate(vTickTockTask, (const char *)"TickTock",
                     2*configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+2, NULL) != pdPASS) ||
        (xTaskCreate(vCmdLineTask, (const char *)"CmdLineTask",
                     configMINIMAL_STACK_SIZE+CMD_LINE_BUF_SIZE+OUTPUT_BUF_SIZE, NULL, tskIDLE_PRIORITY+1, &cmd_task_id) != pdPASS)) {
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

