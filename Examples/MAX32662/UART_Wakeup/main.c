/**
 * @file        main.c
 * @brief       LP Serial Character Wake Up Example
 * @details     
 */

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

/***** Includes *****/
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc_pins.h"
#include "nvic_table.h"
#include "led.h"
#include "pb.h"
#include "board.h"
#include "gpio.h"
#include "lp.h"
#include "uart.h"

/***** Definitions *****/
#define RXBUF_SIZE 100

/***** Globals *****/
mxc_uart_regs_t *ConsoleUART = MXC_UART_GET_UART(CONSOLE_UART);
char rxBuf[RXBUF_SIZE];
volatile int cnt = 0;
volatile bool crRecv = false;

/***** Functions *****/
void UART0_Handler(void)
{
    // To measure wake up time, probe P0.13 (LED Pin).
    LED_On(0);

    if (ConsoleUART->intfl & MXC_F_UART_INTFL_RX_THD) {
        ConsoleUART->intfl |= MXC_F_UART_INTFL_RX_THD;

        while (
            (ConsoleUART->status &
             MXC_F_UART_STATUS_RX_LVL)) { //Continue to read characters until receive buffer empty
            if (cnt >= RXBUF_SIZE) { //Prevent buffer overflow
                cnt = 0;
            } else {
                rxBuf[cnt] = (char)MXC_UART_ReadCharacter(ConsoleUART); //Read character
                if (rxBuf[cnt] == '\r') { //Last character received?
                    crRecv = true;
                }
                cnt++;
            }
        }
    }
}

int main(void)
{
    memset(rxBuf, 0x0, RXBUF_SIZE * sizeof(char));

    printf("\n\n******************** LP Serial Character Wake Up Example *******************\n\n");
    printf("This example demonstrates how to send a serial character to wake up the device.\n");
    printf("Each string sent will be echoed to the terminal. Strings sent to wake up the\n");
    printf("device will appear as garbage due to wake-up latency, each of the following\n");
    printf("strings will be processed correctly. Sending \"sleep\" will put the device\n");
    printf("back in deep sleep and sending \"quit\" will end the example.\n");
    printf("\nTo measure wake-up latency, probe pins P0.14 (LED) and P0.11 (UART RX Pin).\n");
    printf("\n**NOTE**: Each string sent to the device must end in a \"\\r\" character for the\n");
    printf("strings to be processed correctly.\n");

    printf("\nPress PB1 to begin the demo.\n");
    while (!PB_Get(0)) {}

    /* Configure serial character interrupts */
    NVIC_ClearPendingIRQ(UART0_IRQn);
    NVIC_DisableIRQ(UART0_IRQn);
    MXC_NVIC_SetVector(UART0_IRQn, UART0_Handler);
    NVIC_EnableIRQ(UART0_IRQn);
    MXC_UART_SetRXThreshold(ConsoleUART, 1);
    MXC_UART_EnableInt(ConsoleUART, MXC_F_UART_INTEN_RX_THD);

    /* Configure UART RX (P0.11) as wake up pin */
    MXC_LP_EnableGPIOWakeup(&gpio_cfg_uart0);

    /* Put device in Sleep operating mode */
    printf("Now entering sleep mode. Send any character string to wake up the device.\n\n");
    while (MXC_UART_GetActive(ConsoleUART)) {}
    MXC_LP_ClearWakeStatus();
    MXC_LP_EnterSleepMode();

    while (1) {
        if (crRecv) {
            printf("String Received: %s\n",
                   rxBuf); //Print character string received from the console

            if (!strcmp(rxBuf, "sleep\r")) { //If "sleep\r" received, go back to sleep
                printf("Going back to deep sleep.\n");
                while (MXC_UART_GetActive(ConsoleUART)) {}
                LED_Off(0);
                MXC_LP_ClearWakeStatus();
                MXC_LP_EnterSleepMode();
            } else if (!strcmp(rxBuf, "quit\r")) { //If "quit\r" received, end example.
                printf("Stopping example...\n");
                break;
            }

            crRecv = false;
            cnt = 0;
            memset(rxBuf, 0x0, RXBUF_SIZE * sizeof(char));
        }
    }

    LED_On(0);
    printf("Example complete!");
    while (MXC_UART_GetActive(ConsoleUART)) {}

    return E_NO_ERROR;
}
