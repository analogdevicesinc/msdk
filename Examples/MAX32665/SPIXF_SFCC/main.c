/**
 * @file    main.c
 * @brief   Hello World!
 * @details This example uses the UART to print to a terminal and flashes an LED.
 */

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

/* **** Includes **** */
#include <stdio.h>

#include "board.h"
#include "Ext_Flash.h"
#include "icc.h"
#include "led.h"
#include "mxc_errors.h"
#include "pb.h"
#include "rtc.h"
#include "spixf.h"

/* **** Definitions **** */
#define EXT_FLASH_ADDR 0
#define EXT_FLASH_SPIXFC_WIDTH Ext_Flash_DataLine_Quad
#define SSEC_TO_MSEC(ssec) ((ssec * 1000) / 4096)

/***** Globals *****/
int fail = 0;
uint32_t start_sec, start_ssec;
uint32_t stop_sec, stop_ssec;

/***** Functions *****/
// These are set in the linkerfile and give the starting and ending address of xip_section
#if defined(__GNUC__)
extern uint8_t __load_start_xip, __load_length_xip;
#endif

#if defined(__CC_ARM)
// Note: This demo has not been tested under IAR and should be considered non-functional
extern int Image$$RW_IRAM2$$Length;
extern char Image$$RW_IRAM2$$Base[];
uint8_t *__xip_addr;
#endif

/******************************************************************************/
/*
 * NOTE: The only purpose of this function is for it to perform lots of operations.
 * 		 It is meant to show the performance gains in execution time when running
 * 		 it with the instruction cache enabled. It is loaded into the external
 * 		 flash chip during the setup phase of this example.
 */
__attribute__((section(".xip_section"))) void icc_test_func(void)
{
    volatile int i, j;
    volatile int k = 0;
    (void)k; // Suppress unused variable warning

    // Perform operations
    for (i = 0; i < 500; i++) {
        for (j = 0; j < 5000; j++) {
            k = i * j;
        }
    }
}

/******************************************************************************/
void spixf_cfg_setup()
{
    MXC_SPIXF_Disable(); // Disable the SPIXFC before setting the SPIXF

    MXC_SPIXF_SetSPIFrequency(EXT_FLASH_BAUD);
    MXC_SPIXF_SetMode(MXC_SPIXF_MODE_0);
    MXC_SPIXF_SetSSPolActiveLow();
    MXC_SPIXF_SetSSActiveTime(MXC_SPIXF_SYS_CLOCKS_2);
    MXC_SPIXF_SetSSInactiveTime(MXC_SPIXF_SYS_CLOCKS_3);

    if (EXT_FLASH_SPIXFC_WIDTH == Ext_Flash_DataLine_Single) {
        MXC_SPIXF_SetCmdValue(EXT_FLASH_CMD_READ);
        MXC_SPIXF_SetCmdWidth(MXC_SPIXF_SINGLE_SDIO);
        MXC_SPIXF_SetAddrWidth(MXC_SPIXF_SINGLE_SDIO);
        MXC_SPIXF_SetDataWidth(MXC_SPIXF_WIDTH_1);
        MXC_SPIXF_SetModeClk(EXT_FLASH_Read_DUMMY);
    } else {
        MXC_SPIXF_SetCmdValue(EXT_FLASH_CMD_QREAD);
        MXC_SPIXF_SetCmdWidth(MXC_SPIXF_SINGLE_SDIO);
        MXC_SPIXF_SetAddrWidth(MXC_SPIXF_QUAD_SDIO);
        MXC_SPIXF_SetDataWidth(MXC_SPIXF_WIDTH_4);
        MXC_SPIXF_SetModeClk(EXT_FLASH_QREAD_DUMMY);
    }

    MXC_SPIXF_Set3ByteAddr();
    MXC_SPIXF_SCKFeedbackEnable();
    MXC_SPIXF_SetSCKNonInverted();
}

void start_timer(void)
{
    while (MXC_RTC_GetTime(&start_sec, &start_ssec) == E_BUSY) {}
}

void stop_timer(int test_num, uint32_t *sec_elapsed, uint32_t *ssec_elapsed)
{
    // Get time at the end of execution
    while (MXC_RTC_GetTime(&stop_sec, &stop_ssec) == E_BUSY) {}

    // Calculate the elapsed seconds and sub-seconds
    *sec_elapsed = stop_sec - start_sec;
    *ssec_elapsed = (MXC_F_RTC_SSEC_SSEC - start_ssec) + stop_ssec;

    if (*ssec_elapsed > MXC_F_RTC_SSEC_SSEC) {
        *sec_elapsed += 1;
        *ssec_elapsed -= MXC_F_RTC_SSEC_SSEC;
    }

    // Print Results
    printf("Test %d Complete!\n", test_num);
    printf("Execution Time: %d.%ds\n\n", *sec_elapsed, SSEC_TO_MSEC(*ssec_elapsed));
}

/* ************************************************************************** */
int main(void)
{
    int err;
    uint32_t id;
    void (*func)(void);
    uint32_t test1_sec, test1_ssec;
    uint32_t test2_sec, test2_ssec;

    printf("\n\n********************* SPIXF/SFCC Example *********************\n");
    printf("This example demonstrates the performance benefits of enabling the\n");
    printf("SFCC when executing from the %s external flash chip.\n\n", EXT_FLASH_NAME);

    // Initialize the RTC (used as an execution timer)
    if (MXC_RTC_Init(0, 0) != E_NO_ERROR) {
        printf("Failed to initialize RTC.\n");
        printf("Examples failed.\n");
        return E_BAD_STATE;
    }
    MXC_RTC_Start();

    // Initialize interface (SPIXF) with external flash chip
    if (Ext_Flash_Init() != E_NO_ERROR) {
        printf("External flash initialization Failed\n");
        printf("Example Failed\n");
        return E_UNINITIALIZED;
    }
    printf("%s Initialized.\n", EXT_FLASH_NAME);
    Ext_Flash_Reset();

    // Check ID of the external flash
    if ((id = Ext_Flash_ID()) != EXT_FLASH_EXP_ID) {
        printf("Error verifying external flash ID: 0x%x\n", id);
        printf("Example Failed\n");
        return E_NONE_AVAIL;
    }

    // Erase sector of external flash to store test function
    if ((err = Ext_Flash_Erase(0x00000, Ext_Flash_Erase_64K)) != E_NO_ERROR) {
        printf("Flash erase failed with error code: %d\n", err);
        printf("Example Failed\n");
        return err;
    }

    // Enable Quad mode if necessary
    if (EXT_FLASH_SPIXFC_WIDTH == Ext_Flash_DataLine_Quad) {
        if ((err = Ext_Flash_Quad(1)) != E_NO_ERROR) {
            printf("Error enabling quad mode\n\n");
            return err;
        }
    } else {
        if ((err = Ext_Flash_Quad(0)) != E_NO_ERROR) {
            printf("Error disabling quad mode\n\n");
            return err;
        }
    }

    // Load test function into external flash
    printf("Loading test function into external flash.\n\n", (uint32_t)(&__load_length_xip),
           &__load_start_xip);
    if ((err = Ext_Flash_Program_Page(EXT_FLASH_ADDR, &__load_start_xip,
                                      (uint32_t)(&__load_length_xip), EXT_FLASH_SPIXFC_WIDTH)) !=
        E_NO_ERROR) {
        printf("Error Programming: %d\n", err);
        fail++;
    }

    // Set function pointer to the base address of the external
    // flash chip (where the sample function is stored)
    func = (void (*)(void))(MXC_XIP_MEM_BASE | 0x1);

    // Setup SPIX
    spixf_cfg_setup();

    printf("Setup complete. Press SW2 to run SFCC test.\n\n");
    while (!PB_Get(0)) {}

    /***** Test #1 - ICC Enabled Test *****/
    MXC_ICC_EnableInst(MXC_SFCC);
    printf("Running test function with SFCC enabled.\n");
    start_timer();
    func();
    stop_timer(1, &test1_sec, &test1_ssec);

    /***** TEST #2 - ICC Disabled Test *****/
    MXC_ICC_DisableInst(MXC_SFCC);
    printf("Running test function with SFCC disabled.\n");
    printf("This will take a few minutes...\n");
    start_timer();
    func();
    stop_timer(2, &test2_sec, &test2_ssec);

    // Compare execution times
    if (test1_sec < test2_sec) {
        printf("Example Succeeded\n\n");
    } else if (test1_sec == test2_sec && test1_ssec < test2_ssec) {
        printf("Example Succeeded\n\n");
    } else {
        printf("Example Failed\n\n");
        return E_FAIL;
    }

    return E_NO_ERROR;
}
