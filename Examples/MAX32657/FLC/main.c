/******************************************************************************
 *
 * Copyright (C) 2025 Analog Devices, Inc.
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

/**
 * @file    main.c
 * @brief   Flash Controller Example
 * @details This example demonstrates how to use the flash controller for general purpose
 * storage.  See the "README.md" file for more details.
 */

/***** Includes *****/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "mxc.h"

/***** Definitions *****/
#define WORDS_PER_FLASH_PG (MXC_FLASH_PAGE_SIZE / 4)
#define TEST_ADDRESS (MXC_FLASH_MEM_BASE + MXC_FLASH_MEM_SIZE) - (1 * MXC_FLASH_PAGE_SIZE)
/*
    ^ Points to last page in flash, which is guaranteed to be unused by this small example.
    For larger applications it's recommended to reserve a dedicated flash region by creating
    a modified linkerfile.
*/
#define MAGIC 0xFEEDBEEF
#define TEST_VALUE 0xDEADBEEF

/***** Globals *****/
volatile uint32_t isr_cnt;
volatile uint32_t isr_flags;

/***** Functions *****/

//******************************************************************************

int button_pressed = 0;
void button_handler(void)
{
    button_pressed = 1;
}

//******************************************************************************

void FLC_IRQHandler(void)
{
    uint32_t temp;
    isr_cnt++;
    temp = MXC_FLC->intr;

    if (temp & MXC_F_FLC_INTR_DONE_IF) {
        MXC_FLC->intr &= ~MXC_F_FLC_INTR_DONE_IF;
        printf(" -> Interrupt! (Flash operation done)\n\n");
    }

    if (temp & MXC_F_FLC_INTR_AF_IF) {
        MXC_FLC->intr &= ~MXC_F_FLC_INTR_AF_IF;
        printf(" -> Interrupt! (Flash access failure)\n\n");
    }

    isr_flags = temp;
}

void setup_irqs(void)
{
    /*
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

    This example demonstrates method #1.  Any code modifying
    flash is executed from a critical block, and the FLC
    interrupts will trigger afterwards.
    */

    // NVIC_SetRAM(); // Execute ISRs out of SRAM (for use with #2 above)
    MXC_NVIC_SetVector(FLC_IRQn, FLC_IRQHandler); // Assign ISR
    NVIC_EnableIRQ(FLC_IRQn); // Enable interrupt

    __enable_irq();

    // Clear and enable flash programming interrupts
    MXC_FLC_EnableInt(MXC_F_FLC_INTR_DONE_IE | MXC_F_FLC_INTR_AF_IE);
    isr_flags = 0;
    isr_cnt = 0;
}

int write_test_pattern(void)
{
    int err;
    // A flash address must be in the erased state before writing to it, because the
    // flash controller can only write a 1 -> 0.
    // See the microcontroller's User Guide for more details.
    printf("Erasing page 64 of flash (addr 0x%x)...\n", TEST_ADDRESS);
    err = MXC_FLC_PageErase(TEST_ADDRESS);
    if (err) {
        printf("Failed with error code %i\n", TEST_ADDRESS, err);
        return err;
    }

    printf("Writing magic value 0x%x to address 0x%x...\n", MAGIC, TEST_ADDRESS);
    err = MXC_FLC_Write32(TEST_ADDRESS, MAGIC);
    if (err) {
        printf("Failed to write magic value to 0x%x with error code %i!\n", TEST_ADDRESS, err);
        return err;
    }

    printf("Writing test pattern...\n");
    for (uint32_t addr = TEST_ADDRESS + 4; addr < TEST_ADDRESS + MXC_FLASH_PAGE_SIZE; addr += 4) {
        /*  
            A single flash page is organized into 4096 128-bit "words", but is still 
            byte-addressible.  Increment the address by 4 bytes to write in 32-bit 
            chunks.  
            
            The Flash Controller also only supports 128-bit writes. The driver
            function below handles the technicalities of inserting the 32-bit value 
            into a 128-bit word without modifying the rest of the word.
        */
        err = MXC_FLC_Write32(addr, TEST_VALUE);
        if (err) {
            printf("Failed write on address 0x%x with error code %i\n", addr, err);
            return err;
        }
    }
    printf("Done!\n");

    return err;
}

int validate_test_pattern(void)
{
    int err = 0;

    printf("Verifying test pattern...\n");
    uint32_t readval = 0;
    for (uint32_t addr = TEST_ADDRESS + 4; addr < TEST_ADDRESS + MXC_FLASH_PAGE_SIZE; addr += 4) {
        MXC_FLC_ReadECC(addr, &readval, 4);
        if (readval != TEST_VALUE) {
            printf(
                "Failed verification at address 0x%x with error code %i!  Expected: 0x%x\tRead: 0x%x\n",
                addr, err, TEST_VALUE, readval);
            return E_ABORT;
        }
    }

    printf("Successfully verified test pattern!\n\n");
    return err;
}

int erase_magic(void)
{
    /*
        To modify a location in flash that has already been written to,
        that location must first be restored to its erased state.
        However, the flash controller only supports erasing a full page
        at a time.
        Therefore, the entire page must be buffered, erased, then modified.
    */
    int err;
    uint32_t buffer[WORDS_PER_FLASH_PG] = {
        0xFFFFFFFF
    }; // 8192 bytes per page / 4 bytes = 2048 uint32_t

    printf("Buffering page...\n");
    MXC_FLC_ReadECC(TEST_ADDRESS, buffer, MXC_FLASH_PAGE_SIZE);

    printf("Erasing page...\n");
    err = MXC_FLC_PageErase(TEST_ADDRESS);
    if (err) {
        printf("Failed to erase page 0x%x with error code %i!\n", TEST_ADDRESS, err);
        return err;
    }

    printf("Erasing magic in buffer...\n");
    buffer[0] = 0xABCD1234; // Erase magic value

    printf("Re-writing from buffer...\n");
    for (int i = 0; i < (MXC_FLASH_PAGE_SIZE >> 2); i++) {
        err = MXC_FLC_Write32(TEST_ADDRESS + 4 * i, buffer[i]);
        if (err) {
            printf("Failed at address 0x%x with error code %i\n", (TEST_ADDRESS + 4) * i, err);
            return err;
        }
    }
    uint32_t magic = 0;
    MXC_FLC_ReadECC(TEST_ADDRESS, &magic, 4);
    printf("New magic value: 0x%x\n", magic);
    return err;
}

int main(void)
{
    int err = 0;

    printf("\n\n***** Flash Control Example *****\n");
    printf("Press Push Button 1 (PB1/SW1) to continue...\n\n");

    PB_RegisterCallback(0, (pb_callback)button_handler);

    // err = MXC_FLC_PageErase(TEST_ADDRESS);

    while (!button_pressed) {
        LED_On(LED1);
        MXC_Delay(MXC_DELAY_MSEC(500));
        LED_Off(LED1);
        MXC_Delay(MXC_DELAY_MSEC(500));
    }
    LED_Off(LED1);

    setup_irqs(); // See notes in function definition

    /*
    Disable the instruction cache controller (ICC).

    Any code that modifies flash contents should disable the ICC,
    since modifying flash contents may invalidate cached instructions.
    */
    MXC_ICC_Disable();
    MXC_ICC_Enable();
    MXC_ICC_Disable();

    uint32_t magic = 0;
    MXC_FLC_ReadECC(TEST_ADDRESS, &magic, 4);

    if (magic != MAGIC) { // Starting example for the first time.
        // clang-format off
        MXC_CRITICAL(
            printf("---(Critical)---\n");
            err = write_test_pattern();
            printf("----------------\n");
        )
        // clang-format on
        if (err)
            return err;

        err = validate_test_pattern();
        if (err)
            return err;

        printf("\nNow reset or power cycle the board...\n");
    } else { // Starting example after reset or power cycle
        printf("** Magic value 0x%x found at address 0x%x! **\n\n", MAGIC, TEST_ADDRESS);
        printf("(Flash modifications have survived a reset and/or power cycle.)\n\n");

        err = validate_test_pattern();
        if (err)
            return err;

        // clang-format off
        MXC_CRITICAL(
            printf("---(Critical)---\n");
            err = erase_magic();
            printf("----------------\n");
        )
        // clang-format on
        if (err)
            return err;

        err = validate_test_pattern();
        if (err)
            return err;

        printf("Flash example successfully completed.\n");
    }

    return E_SUCCESS;
}
