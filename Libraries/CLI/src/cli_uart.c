
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

#include "cli_uart.h"

/***** Definitions *****/
#define UART_BAUD 115200
#define BUFF_SIZE 1

/****** Globals *********/
volatile int READ_FLAG;
uint8_t RxData;
mxc_uart_req_t read_req;

/******* Functions ********/
void UART_Handler(void)
{
    MXC_UART_AsyncHandler(MXC_UART_GET_UART(CONSOLE_UART));
}

void readCallback(mxc_uart_req_t *req, int error){
    
    line_accumulator(RxData); 
    READ_FLAG = error;
    MXC_UART_TransactionAsync(req);
}

/** Initializes an asychronous uart transaction for CLI operations. This enables the console read
 * 
 * @param none
 * 
 * @return void
 */
int MXC_CLI_Uart_Init(void){

    // UART interrupt setup
    NVIC_ClearPendingIRQ(MXC_UART_GET_IRQ(CONSOLE_UART));
    NVIC_DisableIRQ(MXC_UART_GET_IRQ(CONSOLE_UART));
    MXC_NVIC_SetVector(MXC_UART_GET_IRQ(CONSOLE_UART), UART_Handler);
    NVIC_EnableIRQ(MXC_UART_GET_IRQ(CONSOLE_UART));

    /* Initialize Console UART*/
    int error;
    if ((error = MXC_UART_Init(MXC_UART_GET_UART(CONSOLE_UART), UART_BAUD, MXC_UART_APB_CLK)) !=
        E_NO_ERROR) {
        printf("-->Error initializing UART: %d\n", error);
        printf("-->Example Failed\n");
        return error;
    }
    
    User_Prompt_Sequence();

    read_req.uart = MXC_UART_GET_UART(CONSOLE_UART);
    read_req.rxData = &RxData;
    read_req.rxLen = BUFF_SIZE;
    read_req.txLen = 0;
    read_req.callback = readCallback;

    error = MXC_UART_TransactionAsync(&read_req);

    return error;
}