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
#include "spixf.h"
#include "tmr.h"

/* **** Definitions **** */
#define EXT_FLASH_ADDR         0
#define EXT_FLASH_SPIXFC_WIDTH Ext_Flash_DataLine_Quad
#define SSEC_TO_MSEC(ssec)     ((ssec * 1000) / 4096)
#define SEC_TO_MIN(sec)        (sec / 60)
#define SEC_REMAIN(sec)        (sec % 60)

/***** Globals *****/
int fail = 0;

/***** Functions *****/
// These are set in the linkerfile and give the starting and ending address of xip_section
#if defined(__GNUC__)
extern uint8_t __load_start_xip, __load_length_xip;
#endif

#if defined(__CC_ARM)
// Note: This demo has not been tested under IAR and should be considered non-functional
extern int Image$$RW_IRAM2$$Length;
extern char Image$$RW_IRAM2$$Base[];
uint8_t* __xip_addr;
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
    (void) k; // Suppress unused variable warning

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
    // Disable the SPIXFC before setting the SPIXF
    MXC_SPIXF_Disable();
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

/* ************************************************************************** */
void start_timer(void)
{
    MXC_TMR_SW_Start(MXC_TMR0);
    return;
}

/* ************************************************************************** */
int stop_timer(int test_num)
{
    int time_elapsed   = MXC_TMR_SW_Stop(MXC_TMR0);
    unsigned int sec   = time_elapsed / 1000000;
    unsigned int mili  = (time_elapsed - (sec * 1000000)) / 1000;
    unsigned int micro = time_elapsed - (sec * 1000000) - (mili * 1000);

    printf("Test %d complete.\n", test_num);
    printf("Time Elapsed: %d.%d%d Seconds\n\n", sec, mili, micro);

    return time_elapsed;
}

/* ************************************************************************** */
int main(void)
{
    int err;
    uint32_t id;
    void (*func)(void);
    int test1_et, test2_et;

    printf("\n\n********************* SPIXF/ICC1 Example *********************\n");
    printf("This example demonstrates the performance benefits of using the\n");
    printf("instruction cache controller when executing from the %s external\n", EXT_FLASH_NAME);
    printf("flash chip.\n\n");

    // Initialize interface (SPIXF) with external flash chip
    if (Ext_Flash_Init() != E_NO_ERROR) {
        printf("External flash initialization Failed\n");
        printf("Example Failed\n");
        while (1)
            ;
    }
    printf("%s Initialized.\n", EXT_FLASH_NAME);
    Ext_Flash_Reset();

    // Check ID of the external flash
    if ((id = Ext_Flash_ID()) != EXT_FLASH_EXP_ID) {
        printf("Error verifying external flash ID: 0x%x\n", id);
        printf("Example Failed\n");
        while (1)
            ;
    }

    // Erase sector of external flash to store test function
    if ((err = Ext_Flash_Erase(0x00000, Ext_Flash_Erase_64K)) != E_NO_ERROR) {
        printf("Flash erase failed with error code: %d\n", err);
        printf("Example Failed\n");
        while (1)
            ;
    }

    // Enable Quad mode if necessary
    if (EXT_FLASH_SPIXFC_WIDTH == Ext_Flash_DataLine_Quad) {
        if (Ext_Flash_Quad(1) != E_NO_ERROR) {
            printf("Error enabling quad mode\n\n");
            while (1)
                ;
        }
    } else {
        if (Ext_Flash_Quad(0) != E_NO_ERROR) {
            printf("Error disabling quad mode\n\n");
            while (1)
                ;
        }
    }

    // Load test function into external flash and set function pointer to it's location
    printf("Loading test function into external flash.\n\n", (uint32_t)(&__load_length_xip),
           &__load_start_xip);
    if ((err = Ext_Flash_Program_Page(EXT_FLASH_ADDR, &__load_start_xip,
                                      (uint32_t)(&__load_length_xip), EXT_FLASH_SPIXFC_WIDTH)) !=
        E_NO_ERROR) {
        printf("Error Programming: %d\n", err);
        fail++;
    }
    func = (void (*)(void))(MXC_XIP_MEM_BASE | 0x1);

    // Setup SPIX
    spixf_cfg_setup();

    printf("Setup complete. Press SW2 to run ICC1 test.\n\n");
    while (!PB_Get(0))
        ;

    /***** ICC Enabled Test *****/
    MXC_ICC_EnableInst(MXC_ICC1);
    printf("Running test function with ICC1 enabled.\n");
    start_timer();
    func();
    test1_et = stop_timer(0);

    /***** ICC Disabled Test *****/
    MXC_ICC_DisableInst(MXC_ICC1);
    printf("Running test function with ICC1 disabled.\n");
    printf("This will take a few minutes...\n");
    start_timer();
    func();
    test2_et = stop_timer(1);

    // Compare execution times
    if (test1_et < test2_et) {
        printf("Example Succeeded\n\n");
    } else {
        printf("Example Failed\n\n");
    }

    return 0;
}
