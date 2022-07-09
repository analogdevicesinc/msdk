
/**
 * @file    main.c
 * @brief   Demonstration of SRAM ECC
 * @details This program demonstrates single and double-bit error detection and
 *          single-bit correction for SRAM memories.
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

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "led.h"
#include "board.h"
#include "gcr_regs.h"
#include "mcr_regs.h"


/***** Definitions *****/
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

/***** Globals *****/
volatile uint32_t badData;
volatile uint32_t eccFlag;
volatile uint32_t eccErr;
volatile uint32_t eccDErr;
volatile uint32_t eccAddr;

// RAM0 is the only one that supports ECC.  It is 32KB.
uint32_t ramTop = (MXC_SRAM_MEM_BASE + 32768);

/***** Functions *****/


void ECC_IRQHandler(void)
{
    eccErr = MXC_GCR->eccerr;
    eccDErr = MXC_GCR->eccced;
    eccAddr = MXC_GCR->eccaddr;
    eccFlag = 1;
    
    MXC_GCR->eccerr = eccErr;
    MXC_GCR->eccced = eccDErr;
}


// *****************************************************************************
int main(void)
{
    unsigned int i, test_fail, test_pass;
    volatile uint32_t* cursor;
    
    test_fail = test_pass = 0;
    
    printf("\n\n***** " TOSTRING(TARGET) " SRAM ECC Example *****\n\n");
    printf("This example will corrupt a word of data\n");
    printf("and ensure that the ECC interrupts on an error\n");
    printf("when the corrupted address is read\n\n");
    
    // Clear all ECC Errors -- write-1-to-clear
    MXC_GCR->eccerr = (volatile uint32_t)MXC_GCR->eccerr;
    MXC_GCR->eccced = (volatile uint32_t)MXC_GCR->eccced;
    
    // Enable interrupts for ECC errors
    MXC_GCR->eccie |=  MXC_F_GCR_ECCIE_RAM;
    NVIC_EnableIRQ(ECC_IRQn);
    
    // Scan all of memory, which should not cause any errors to be detected
    printf("Preliminary scan to ensure no pre-existing ECC errors\n");
    eccFlag = 0;
    
    for (i = MXC_SRAM_MEM_BASE; i < ramTop - sizeof(uint32_t); i += sizeof(uint32_t)) {
        cursor = (uint32_t*) i;
        (volatile uint32_t)*cursor; // Force a read of the memory
        
        if (eccFlag) {
            // Try to fix the error.
            *cursor = (volatile uint32_t)*cursor;
            eccFlag = 0;
        }
    }

    // Re-scan to see if there were any errors that couldn't be fixed.
    eccFlag = 0;
    
    for (i = MXC_SRAM_MEM_BASE; i < ramTop - sizeof(uint32_t); i += sizeof(uint32_t)) {
        cursor = (uint32_t*) i;
        (volatile uint32_t)*cursor; // Force a read of the memory
        
        if (eccFlag) {
            break;
        }
    }
    
    if (eccFlag) {
        printf("ECC Error:              0x%08x\n", eccErr);
        printf("ECC Not Double Error:   0x%08x\n", eccDErr);
        printf("ECC Error Address:      0x%08x\n", eccAddr);
        printf("FAIL: Error found in preliminary memory scan\n");
        test_fail++;
    }
    else {
        printf("PASS: No errors\n");
        test_pass++;
    }
    
    // Initialize data
    badData = 0xDEADBEEF;
    
    printf("\nData before Corruption: 0x%08x\n", badData);
    printf("Address of data: 0x%08x\n", &badData);
    
    // Disable ECC so data can be corrupted
    MXC_MCR->eccen &= ~MXC_F_MCR_ECCEN_RAM0;
    badData = 0xDEADBEEE;
    MXC_MCR->eccen |= MXC_F_MCR_ECCEN_RAM0;
    
    printf("\nData after single-bit error: 0x%08x\n", badData);
    printf("ECC Error:              0x%08x\n", eccErr);
    printf("ECC Not Double Error:   0x%08x\n", eccDErr);
    printf("ECC Error Address:      0x%08x\n", eccAddr);
    
    if (eccFlag) {
        printf("PASS: An ECC Error was found\n");
        eccFlag = 0;
        test_pass++;
    }
    else {
        printf("FAIL: Error not detected!\n");
        eccFlag = 0;
        test_fail++;
    }
    
    // Disable ECC so data can be corrupted
    MXC_MCR->eccen &= ~MXC_F_MCR_ECCEN_RAM0;
    badData = 0xDEADBEEC;
    MXC_MCR->eccen |= MXC_F_MCR_ECCEN_RAM0;
    
    printf("\nData after double-bit error: 0x%08x\n", badData);
    printf("ECC Error:              0x%08x\n", eccErr);
    printf("ECC Not Double Error:   0x%08x\n", eccDErr);
    printf("ECC Error Address:      0x%08x\n", eccAddr);
    
    if (eccFlag) {
        printf("PASS: An ECC Error was found\n");
        eccFlag = 0;
        test_pass++;
    }
    else {
        printf("FAIL: Error not detected!\n");
        eccFlag = 0;
        test_fail++;
    }
    
    printf("\n# Passed: %u, # Failed: %u, Test %s\n", test_pass, test_fail, test_fail ? "FAIL!" : "Ok");
    printf("Example Complete\n");
    
    while (1);
}
