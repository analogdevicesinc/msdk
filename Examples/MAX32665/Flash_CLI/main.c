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
 * @brief   Flash Control Mass Erase & Write 32-bit enabled mode Example
 * @details This example shows how to mass erase the flash using the library
 *          and also how to Write and Verify 4 Words to the flash.
 */

/***** Includes *****/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "board.h"
#include "definitions.h"
#include "dma.h"
#include "flc.h"
#include "gcr_regs.h"
#include "icc.h"
#include "mxc_assert.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "nvic_table.h"
#include "semphr.h"
#include "task.h"
#include "tpu.h"
#include "uart.h"

/* FreeRTOS+CLI */
void vRegisterCLICommands(void);

/* Task IDs */
TaskHandle_t cmd_task_id;

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
#define POLY 0xEDB88320

/***** Functions *****/

/* =| UART0_IRQHandler |======================================
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
    unsigned int index; /* Index into buffer */
    unsigned int x;
    int uartReadLen;
    char buffer[CMD_LINE_BUF_SIZE]; /* Buffer for input */
    char output[OUTPUT_BUF_SIZE]; /* Buffer for output */
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
        while (MXC_UART_ReadyForSleep(ConsoleUART)) {}

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
                } else if ((tmp == '\r') || (tmp == '\n')) {
                    printf("\r\n");
                    /* Null terminate for safety */
                    buffer[index] = 0x00;
                    /* Evaluate */
                    do {
                        xMore = FreeRTOS_CLIProcessCommand(buffer, output, OUTPUT_BUF_SIZE);
                        /* If xMore == pdTRUE, then output buffer contains no null
						 * termination, so we know it is OUTPUT_BUF_SIZE. If pdFALSE, we can
						 * use strlen.
						 */
                        for (x = 0; x < (xMore == pdTRUE ? OUTPUT_BUF_SIZE : strlen(output)); x++) {
                            putchar(*(output + x));
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

                if (ConsoleUART->status &
                    MXC_F_UART_STATUS_RX_EMPTY) { // Prevent dropping characters
                    MXC_Delay(500);
                }
            } while ((MXC_UART_GetRXFIFOAvailable(MXC_UART_GET_UART(CONSOLE_UART)) > 0) &&
                     (MXC_UART_Read(ConsoleUART, (uint8_t *)&tmp, &uartReadLen) == 0));
        }
    }
}

//******************************************************************************
int flash_verify(uint32_t address, uint32_t length, uint32_t *data)
{
    volatile uint32_t *ptr;

    for (ptr = (uint32_t *)address; ptr < (uint32_t *)(address + length); ptr++, data++) {
        if (*ptr != *data) {
            printf("Verify failed at 0x%x (0x%x != 0x%x)\n", (unsigned int)ptr, (unsigned int)*ptr,
                   (unsigned int)*data);
            return E_UNKNOWN;
        }
    }

    return E_NO_ERROR;
}

//******************************************************************************
int flash_write(uint32_t startaddr, uint32_t length, uint32_t *data)
{
    int i = 0;

    // Check if flash controller is busy
    if (MXC_FLC0->cn & MXC_F_FLC_CN_PEND) {
        return E_BUSY;
    }

    if (!check_erased(startaddr, length)) {
        return E_INVALID;
    }

    MXC_ICC_Disable();

    for (uint32_t testaddr = startaddr; i < length; testaddr += 4) {
        // Write a word
        int error_status = MXC_FLC_Write(testaddr, 4, &data[i]);
        LOGV("Write addr 0x%08X: %c\r\n", testaddr, data[i]);
        if (error_status != E_NO_ERROR) {
            printf("Failure in writing a word : error %i addr: 0x%08x\n", error_status, testaddr);
            return error_status;
        }
        i++;
    }

    MXC_ICC_Enable();

    return flash_verify(startaddr, length, data);
}

// *****************************************************************************
int flash_read(uint32_t startaddr, uint32_t length, uint8_t *data)
{
    for (int i = 0; i < length; i++) {
        uint32_t addr = startaddr + i * 4;
        data[i] = *(uint32_t *)addr;
        if (data[i] == 0xFF) {
            LOGV("Read addr 0x%08X: %s\r\n", addr, "empty");
        } else {
            LOGV("Read addr 0x%08X: %c\r\n", addr, data[i]);
        }
    }
    return E_NO_ERROR;
}

// *****************************************************************************
int check_mem(uint32_t startaddr, uint32_t length, uint32_t data)
{
    uint32_t *ptr;

    for (ptr = (uint32_t *)startaddr; ptr < (uint32_t *)(startaddr + length); ptr++) {
        if (*ptr != data) {
            return 0;
        }
    }
    return 1;
}

//******************************************************************************
int check_erased(uint32_t startaddr, uint32_t length)
{
    return check_mem(startaddr, length, 0xFFFFFFFF);
}

//******************************************************************************
void flash_init(void)
{
    MXC_FLC_ClearFlags(0x3);
}

//******************************************************************************
uint32_t calculate_crc(uint32_t *array, uint32_t length)
{
    int err;
    uint32_t crc;

    uint8_t *flash_cpy = (uint8_t *)malloc(MXC_FLASH_PAGE_SIZE);
    if (flash_cpy == NULL) {
        return E_INVALID;
    }
    memcpy(flash_cpy, array, MXC_FLASH_PAGE_SIZE); // TPU can't perform CRC from flash

    MXC_TPU_Init(MXC_SYS_PERIPH_CLOCK_TPU);

    if ((err = MXC_TPU_CRC((uint8_t *)flash_cpy, MXC_FLASH_PAGE_SIZE, POLY, &crc)) != E_NO_ERROR) {
        return err;
    }

    free(flash_cpy);

    return crc;
}

//******************************************************************************
int main(void)
{
    printf("\n\n*************** Flash Control CLI Example ***************\n");
    printf("\nThis example demonstrates the CLI commands feature of FreeRTOS, various features");
    printf("\nof the Flash Controller (page erase and write), and how to use the TPU to");
    printf("\ncompute the CRC value of an array. Enter commands in the terminal window.\n\n");

    NVIC_SetRAM();
    // Initialize the Flash
    flash_init();

    /* Configure task */
    if ((xTaskCreate(vCmdLineTask, (const char *)"CmdLineTask",
                     configMINIMAL_STACK_SIZE + CMD_LINE_BUF_SIZE + OUTPUT_BUF_SIZE, NULL,
                     tskIDLE_PRIORITY + 1, &cmd_task_id) != pdPASS)) {
        printf("xTaskCreate() failed to create a task.\n");
    } else {
        /* Start scheduler */
        printf("Starting FreeRTOS scheduler.\n");
        vTaskStartScheduler();
    }

    while (1) {}

    return 0;
}
