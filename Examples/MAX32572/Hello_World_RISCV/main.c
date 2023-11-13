/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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
 * @brief   Hello World RISC-V!
 *
 * @details This example uses the UART to print to a terminal and flashes LED0.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <MAX32xxx.h>
#include "fcr_regs.h"

/***** Definitions *****/

/***** Globals *****/

/* Symbols defined when loading RISCV image */
extern uint32_t _binary_riscv_bin_start;
extern uint32_t _binary_riscv_bin_end;
uint32_t riscv_load_addr = { 
#include "riscv_load.addr"
};
uint32_t riscv_text_addr = { 
#include "riscv_text.addr"
};

/***** Functions *****/

// *****************************************************************************
int main(void)
{
    int count = 0;

    printf("\nARM: ***********Hello World!***********\n");
    printf(  "ARM: LED0 on P1.30 toggles every 500 ms\n");

    /* Load RISCV image from FLASH to SRAM */
    {
        volatile uint32_t * psrc = &_binary_riscv_bin_start;
        volatile uint32_t * pdst = (uint32_t *)riscv_load_addr;
        while (psrc < &_binary_riscv_bin_end) {
            *pdst++ = *psrc++;
        }
    }

    MXC_FCR->urvbootaddr = riscv_text_addr; /* Set RISC-V starting address */
    printf("\nARM: Starting RISC-V at 0x%08x, %s %s \n", MXC_FCR->urvbootaddr,__DATE__,__TIME__);

    /* delay starting RISCV until printf() complete */
    while (MXC_UART_GetActive(MXC_UART_GET_UART(CONSOLE_UART)) == E_BUSY) {}

    MXC_GCR->pclkdis1 &= ~MXC_F_GCR_PCLKDIS1_CPU1; /* enable RISCV clock */

    while (1) {
        LED_On(0);
        MXC_Delay(500000);
        LED_Off(0);
        MXC_Delay(500000);

        // print on console
        printf("\nARM: Counter = %d", count++);
    }
}
