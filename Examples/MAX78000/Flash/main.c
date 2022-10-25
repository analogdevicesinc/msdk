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

/***** Definitions *****/
#define WORDS_PER_PG (MXC_FLASH_PAGE_SIZE / 4) // 4 bytes make up a 32-bit word
#define FLASH_SIZE \
    (MXC_FLASH_MEM_SIZE / MXC_FLASH_PAGE_SIZE) // Total size of the flash in number of pages
#define NUM_TEST_PAGES \
    FLASH_SIZE // Number of flash pages to test.  Defaults to the entire flash bank
#define TESTSIZE \
    (WORDS_PER_PG * NUM_TEST_PAGES) // Calculate the number of words to write for the test.

/***** Globals *****/
volatile uint32_t isr_cnt;
volatile uint32_t isr_flags;

/***** Functions *****/

//******************************************************************************

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
    NVIC_SetRAM(); // Execute ISRs out of SRAM
    MXC_NVIC_SetVector(FLC0_IRQn, FLC0_IRQHandler); // Assign ISR
    NVIC_EnableIRQ(FLC0_IRQn); // Enable interrupt
    __enable_irq();

    // Clear and enable flash programming interrupts
    MXC_FLC_EnableInt(MXC_F_FLC_INTR_DONEIE | MXC_F_FLC_INTR_AFIE);
    isr_flags = 0;
    isr_cnt = 0;
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
    int error, i;
    uint32_t start, end;
    uint32_t buffer[0x2000];

    printf("\n\n***** Flash Control Example *****\n");
    printf("This example executes entirely from RAM, and\n");
    printf("will mass erase the entire flash contents before\n");
    printf("writing and verifying a test pattern from\n");
    printf("ADDR: 0x%x to ADDR: 0x%x\n", MXC_FLASH_MEM_BASE, MXC_FLASH_MEM_BASE + TESTSIZE);
    printf("Press Push Button 1 (PB1/SW1) to begin\n");

    while (!PB_Get(0)) {}

    // Initialize the Flash (see notes in 'flash_init' definition)
    flash_init();

    /*
    Mass erase the flash.

    Since this example mass erases the entire flash contents (including application code),
    then this _entire_ example must be executed out of SRAM.  This is done by
    assigning a special linkerfile via the project's Makefile (see LINKER=$(TARGET_LC)_ram.ld).

    The '_ram' linkerfile places the .text section (program code) into SRAM instead of Flash.
    The notes about ISRs executing out of flash still must be considered.  (see flash_init)
    */
    printf("Wiping flash...\n");
    error = MXC_FLC_MassErase();

    if (error == E_NO_ERROR) {
        printf("Flash has been successfully wiped.");
    } else if (error == E_BAD_STATE) {
        printf("Flash erase operation is not allowed in this state.\n");
    } else {
        printf("Fail to erase flash's content.\n");
        fail += 1;
    }

    // Check flash's content.  Should be all 1's after erasure.
    if (check_erased(MXC_FLASH_MEM_BASE, MXC_FLASH_MEM_SIZE)) {
        printf("Flash mass erase is verified.\n");
    } else {
        printf("Failed!  Flash mass erase failed.\n");
        return 1;
    }

    /*
    Disable the instruction cache controller (ICC).

    Any code that modifies flash contents should disable the ICC,
    since modifying flash contents may invalidate cached instructions.
    */
    MXC_ICC_Disable(MXC_ICC0);

    i = 0;
    uint32_t testaddr;
    for (testaddr = (MXC_FLASH_MEM_BASE); i < TESTSIZE; testaddr += 4) {
        /* 
        Write test data to flash one word at a time.
        Default test value for each word is 0, 1, 2, ..., n, n + 1, etc.

        The test pattern is created "on the fly" with reference to the 
        running index variable i so that the memory footprint of the 
        test pattern is minimized.  Remember - this example is executing
        out of SRAM.  Strange things can happen when the stack and program
        code start to bleed into each other...
        */
        uint32_t testval = i;
        error = MXC_FLC_Write32(testaddr, testval);
        if (error != E_NO_ERROR) {
            printf("Failure writing 0x%x to ADDR: 0x%x with error code %i\n", i, testaddr, error);
            return 1;
        }

        // Read test data
        uint32_t read_val;
        MXC_FLC_Read(testaddr, &read_val, 4);
        /* ^ Read 4 bytes into 'read_val'.  

        Since 'read_val' is of type uint32_t the 4 bytes -> 1 word 
        conversion happens automatically on pointer de-reference inside the MXC_FLC_Read function.
        MXC_FLC_Read is a wrapper around memcpy.
        */

        // Verify test data
        if (read_val != testval) {
            printf("Verification failed at ADDR: %x\t(val: %x != testval: %x)\n", testaddr,
                   read_val, testval);
        }

        i++;

        if (i < 16) {
            // Only printf for the first 16 words.  Otherwise, the program would take forever.
            printf("Word %d written properly and has been verified.\n", i);
        } else if (i == 16) {
            printf("Continuing for %d more words...\n", TESTSIZE - i);
        } else if (i % (5096) == 0) {
            // Print a progress indicator every 5096 words
            printf("%.2f%%\n", ((float)i / TESTSIZE) * 100);
        }
    }

    printf("100%%\n");
    printf("Done!\n");

    // Erase all of page 2.
    int page = 2;
    uint32_t erase_addr = MXC_FLASH_MEM_BASE + (page * MXC_FLASH_PAGE_SIZE);
    error = MXC_FLC_PageErase(erase_addr);

    if (error) {
        printf("Failed to erase page %i (ADDR: 0x%x) with error code %i\n", page, erase_addr,
               error);
        return 1;
    }

    if (check_erased(erase_addr, MXC_FLASH_PAGE_SIZE)) {
        printf("Successfully verified erasure of page %i (ADDR: 0x%x)\n", page, erase_addr);
    } else {
        printf("Erasure of page %i (ADDR: 0x%x) failed...  Function call completed, but contents "
               "were not fully erased.\n",
               page, erase_addr);
        return 1;
    }

    // Erase partial pages or wide range of pages and keep the data on the page not inbetween start and end.
    // In this case erase part of pages 1 and 2
    start = (MXC_FLASH_MEM_BASE + 0x80);
    end = (MXC_FLASH_MEM_BASE + (2 * MXC_FLASH_PAGE_SIZE) - 0x500);
    printf("Testing partial erasure between pages 1 (ADDR: 0x%x) and 2 (ADDR: 0x%x)...\n", start,
           end);

    flash_erase(start, end, buffer, 0x2000);

    if (check_erased(start, (end - start))) {
        printf("Successfully verified partial erasure.\n");
    } else {
        printf("Failed to verify partial erasure!\n");
        return 1;
    }

    // Flash modifications are complete, so the ICC can be re-enabled.
    MXC_ICC_Enable(MXC_ICC0);

    printf("Example succeeded!\n");

    return 0;
}
