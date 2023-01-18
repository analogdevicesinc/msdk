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
 * @brief   Flash Control Mass Erase & Write 32-bit enabled mode Example
 * @details This example demonstrates how to properly mass erase the entire flash bank
 * from application code.  Additionally, it shows how to read, write, and verify data
 * from flash.
 */

/***** Includes *****/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "nvic_table.h"
#include "flc.h"
#include "icc.h"
#include "gcr_regs.h"
#include "mxc_delay.h"
#include "pb.h"
#include "board.h"
#include "max78000.h"
#include "uart.h"
#include "led.h"
#include "pb.h"

/***** Definitions *****/
#define WORDS_PER_PG (MXC_FLASH_PAGE_SIZE / 4) // 4 bytes make up a 32-bit word
#define FLASH_SIZE \
    (MXC_FLASH_MEM_SIZE / MXC_FLASH_PAGE_SIZE) // Total size of the flash in number of pages
#define NUM_TEST_PAGES \
    FLASH_SIZE // Number of flash pages to test.  Defaults to the entire flash bank
#define TESTSIZE \
    (WORDS_PER_PG * NUM_TEST_PAGES) // Calculate the number of words to write for the test.

#define TEST_ADDRESS 0x1007E000
#define MAGIC 0xDEADBEEF
#define TEST_VALUE 0xFEEDBEEF

/***** Globals *****/
volatile uint32_t isr_cnt;
volatile uint32_t isr_flags;

/***** Functions *****/

//******************************************************************************

int button_pressed = 0;
void button_handler()
{
    button_pressed = 1;
}

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
    // Flash defaults back to all 1's when it's erased.
    return check_mem(startaddr, length, 0xFFFFFFFF);
}

//******************************************************************************

void FLC0_IRQHandler(void)
{
    uint32_t temp;
    isr_cnt++;
    temp = MXC_FLC0->intr;

    if (temp & MXC_F_FLC_INTR_DONE) {
        MXC_FLC0->intr &= ~MXC_F_FLC_INTR_DONE;
    }

    if (temp & MXC_F_FLC_INTR_AF) {
        MXC_FLC0->intr &= ~MXC_F_FLC_INTR_AF;
    }

    isr_flags = temp;
}

void flash_init(void)
{
    /*
    The NVIC_SetRAM function below sets ISRs to execute out of RAM.
    All functions modifying flash contents are set to execute out of RAM
    with the (section(".flashprog")) attribute.  Therefore,
    
    If:
    - An FLC function is in the middle of execution (from RAM)
    ... and...
    - An interrupt triggers an ISR which executes from Flash

    ... Then a hard fault will be triggered.  
    
    FLC functions should be:
    1) Executed from a critical code block (interrupts disabled)
    or
    2) ISRs should be set to execute out of RAM with NVIC_SetRAM()

    It's preferable to use #1 but the Flash Controller can also 
    generate interrupts of its own (as shown in this example), 
    in which case use #2 for the FLC ISRs.
    */
    // NVIC_SetRAM(); // Execute ISRs out of SRAM
    // MXC_NVIC_SetVector(FLC0_IRQn, FLC0_IRQHandler); // Assign ISR
    // NVIC_EnableIRQ(FLC0_IRQn); // Enable interrupt
    // __enable_irq();

    // // Clear and enable flash programming interrupts
    // MXC_FLC_EnableInt(MXC_F_FLC_INTR_DONEIE | MXC_F_FLC_INTR_AFIE);
    // isr_flags = 0;
    // isr_cnt = 0;
}

//******************************************************************************
int flash_erase(uint32_t start, uint32_t end, uint32_t *buffer, unsigned length)
{
    int retval;
    uint32_t start_align, start_len, end_align, end_len, i;

    MXC_ASSERT(buffer);

    // Align start and end on page boundaries, calculate length of data to buffer
    start_align = start - (start % MXC_FLASH_PAGE_SIZE);
    start_len = (start % MXC_FLASH_PAGE_SIZE);
    end_align = end - (end % MXC_FLASH_PAGE_SIZE);
    end_len = MXC_FLASH_PAGE_SIZE - (end % MXC_FLASH_PAGE_SIZE);

    // Make sure the length of buffer is sufficient
    if ((length < start_len) || (length < end_len)) {
        return E_BAD_PARAM;
    }

    // Start and end address are in the same page
    if (start_align == end_align) {
        if (length < (start_len + end_len)) {
            return E_BAD_PARAM;
        }

        // Buffer first page data and last page data, erase and write
        memcpy(buffer, (void *)start_align, start_len);
        memcpy(&buffer[start_len], (void *)end, end_len);
        retval = MXC_FLC_PageErase(start_align);

        if (retval != E_NO_ERROR) {
            return retval;
        }

        retval = MXC_FLC_Write(start_align, start_len, buffer);

        if (retval != E_NO_ERROR) {
            return retval;
        }

        retval = MXC_FLC_Write(end, end_len, &buffer[start_len]);

        if (retval != E_NO_ERROR) {
            return retval;
        }

        return E_NO_ERROR;
    }

    // Buffer, erase, and write the data in the first page
    memcpy(buffer, (void *)start_align, start_len);
    retval = MXC_FLC_PageErase(start_align);

    if (retval != E_NO_ERROR) {
        return retval;
    }

    retval = MXC_FLC_Write(start_align, start_len, buffer);

    if (retval != E_NO_ERROR) {
        return retval;
    }

    // Buffer, erase, and write the data in the last page
    memcpy(buffer, (void *)end, end_len);
    retval = MXC_FLC_PageErase(end_align);

    if (retval != E_NO_ERROR) {
        return retval;
    }

    retval = MXC_FLC_Write(end, end_len, buffer);

    if (retval != E_NO_ERROR) {
        return retval;
    }

    // Erase the remaining pages. MultiPageErase will not erase if start is greater than end.
    for (i = (start_align + MXC_FLASH_PAGE_SIZE); i < (end_align - MXC_FLASH_PAGE_SIZE);
         i += MXC_FLASH_PAGE_SIZE) {
        retval = MXC_FLC_PageErase(i);

        if (retval != E_NO_ERROR) {
            break;
        }
    }

    return retval;
}

int main(void)
{
    int fail = 0;
    int err, i;
    uint32_t start, end;
    uint32_t buffer[0x2000];

    printf("\n\n***** Flash Control Example *****\n");
    printf("Press Push Button 1 (PB1/SW1) to continue...\n\n");

    PB_RegisterCallback(0, (pb_callback)button_handler);

    while (!button_pressed) {
        LED_On(LED_RED);
        MXC_Delay(MXC_DELAY_MSEC(500));
        LED_Off(LED_RED);
        MXC_Delay(MXC_DELAY_MSEC(500));
    }
    LED_Off(LED_RED);

    // Initialize the Flash (see notes in 'flash_init' definition)
    flash_init();

    /*
    Disable the instruction cache controller (ICC).

    Any code that modifies flash contents should disable the ICC,
    since modifying flash contents may invalidate cached instructions.
    */
    MXC_ICC_Disable(MXC_ICC0);

    uint32_t magic = 0;
    MXC_FLC_Read(TEST_ADDRESS, &magic, 4);
    if (magic != MAGIC) {  // Starting example for the first time.

        // Flash must be in the erased state before writing to it.  See
        // the microcontroller's User Guide for more details.
        if ((err = MXC_FLC_PageErase(TEST_ADDRESS)) != E_NO_ERROR) {
            printf("Failed to erase page 64 of flash (addr 0x%x)\n", TEST_ADDRESS);
        } else {
            printf("Sucessfully erased page 64 of flash (addr 0x%x)\n", TEST_ADDRESS);
        }

        printf("Writing magic value 0x%x to address 0x%x...\n", MAGIC, TEST_ADDRESS);
        if ((err = MXC_FLC_Write32(TEST_ADDRESS, MAGIC)) != E_NO_ERROR) {
            printf("Failed to write magic value to first byte of flash!\n");
            return -1;
        }

        for (uint32_t addr = TEST_ADDRESS + 4; addr < TEST_ADDRESS + 0x2000; addr += 4) {
            //                                          ^ Address must be word-aligned
            if ((err = MXC_FLC_Write32(addr, TEST_VALUE)) != E_NO_ERROR) {
                printf("Failed write on address 0x%x with error code %i\n", addr, err);
                return -1;
            }
        }
        printf("Done!\n");
        printf("\nNow reset or power cycle the board...\n");

    } else { // Starting example after reset or power cycle
        printf("** Magic value 0x%x found at address 0x%x! **\n\n", MAGIC, TEST_ADDRESS);
        printf("Flash modifications have survived a reset and/or power cycle.\n");
        printf("Verifying test pattern...\n");
        uint32_t readval = 0;
        int verified = 1;
        for (uint32_t addr = TEST_ADDRESS + 4; addr < TEST_ADDRESS + 0x2000; addr += 4) {
            MXC_FLC_Read(addr, &readval, 4);
            if (readval != TEST_VALUE) {
                printf("Failed verification at address 0x%x!  Expected: 0x%x\tRead: 0x%x\n", addr, TEST_VALUE, readval);
                verified = 0;
                break;
            }
        }

        if (verified) {
            printf("Sucessfully verified test pattern!\n");
        }

        printf("Erasing page again to reset...\n");
        if ((err = MXC_FLC_PageErase(TEST_ADDRESS)) != E_NO_ERROR) {
            printf("Failed to erase page!\n");
            return -1;
        }
        printf("Done!\n");
    }

    return 0;
}
