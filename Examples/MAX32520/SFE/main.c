/**
 * @file    main.c
 * @brief   SPI Master Demo
 * @details Shows Master loopback demo for QSPI0
 *          Read the printf() for instructions
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
#include <string.h>
#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc_pins.h"
#include "nvic_table.h"
#include "sfe.h"
#include "flc.h"
#include "icc.h"

/***** FLASH Commands ******/

//Write Command
#define FLASH_WRITE 0x02
//Erase Commands
#define FLASH_PAGE_ERASE 0x55

#define RAM_SBA 0x20020000
#define FLASH_SBA 0x10000000
#define RAM_STA 0x20048000
#define FLASH_STA 0x10008000

#define RAM_HOST_SBA 0x00330000
#define FLASH_HOST_SBA 0x00990000

#define FLASH_WRITE_SBA 0x20020008
#define FLASH_ERASE_SBA 0x20020000

// To check interrupts
int isr_cnt;
int isr_flags;
uint32_t flashAddr;
uint32_t *data;
uint32_t length;
uint32_t *eraseCmd = (uint32_t *)FLASH_ERASE_SBA;
uint32_t *writeCmd = (uint32_t *)FLASH_WRITE_SBA;

/* ***** Functions ***** */

//******************************************************************************
int Flash_Verify(uint32_t address, uint32_t length, uint8_t *data)
{
    volatile uint8_t *ptr;

    for (ptr = (uint8_t *)address; ptr < (uint8_t *)(address + length); ptr++, data++) {
        if (*ptr != *data) {
            printf("Verify failed at 0x%x (0x%x != 0x%x)\n", (unsigned int)ptr, (unsigned int)*ptr,
                   (unsigned int)*data);
            return E_UNKNOWN;
        }
    }

    return E_NO_ERROR;
}

//******************************************************************************
int Flash_CheckErased(uint32_t startaddr)
{
    uint32_t *ptr;

    for (ptr = (uint32_t *)startaddr; ptr < (uint32_t *)(startaddr + 4096); ptr++) {
        if (*ptr != 0xFFFFFFFF) {
            return 0;
        }
    }

    return 1;
}

//******************************************************************************
void Flash_Write()
{
    int fail = 0;
    int i = 0;

    MXC_ICC_Disable();

    while (i < length) {
        // Clear and enable flash programming interrupts

        isr_flags = 0;
        isr_cnt = 0;

        // Write a word
        if (MXC_FLC_Write32(flashAddr, *data) != E_NO_ERROR) {
            printf("\nFailure in writing a word.\n");
            fail += 1;
            break;
        }

        // Verify that word is written properly
        if (Flash_Verify(flashAddr, 4, (uint8_t *)data) != E_NO_ERROR) {
            printf("\nWord is not written properly.\n");
            fail += 1;
            // break;
        }

        i++;
        data++;
        flashAddr += 4;
    }

    printf("\nData Written to the Flash\n");
    MXC_ICC_Enable();
}

//******************************************************************************
void Flash_Init(void)
{
    MXC_FLC0->flsh_clkdiv = 54;

    // Setup and enable interrupt
    NVIC_EnableIRQ(FLC0_IRQn);
    __enable_irq();
}

//******************************************************************************
void FLC0_IRQHandler(void)
{
    uint32_t temp;
    isr_cnt++;
    temp = MXC_FLC0->flsh_int;

    if (temp & MXC_F_FLC_FLSH_INT_DONE) {
        MXC_FLC0->flsh_int &= ~MXC_F_FLC_FLSH_INT_DONE;
    }

    if (temp & MXC_F_FLC_FLSH_INT_AF) {
        MXC_FLC0->flsh_int &= ~MXC_F_FLC_FLSH_INT_AF;
    }

    isr_flags = temp;
}

//******************************************************************************
void Flash_InterruptEN(mxc_flc_regs_t *regs)
{
    regs->flsh_int = (MXC_F_FLC_FLSH_INT_DONEIE | MXC_F_FLC_FLSH_INT_AFIE);
}

//******************************************************************************
void Flash_CommandCheck()
{
    static uint8_t isErased = 0;

    if (*eraseCmd == FLASH_PAGE_ERASE) {
        eraseCmd++;
        flashAddr = *eraseCmd++;
        flashAddr = (flashAddr - FLASH_HOST_SBA) + (FLASH_SBA);
        printf("\n\nFlash Address: %x", flashAddr);
        printf("\nErasing the Page\n");
        eraseCmd = (uint32_t *)FLASH_ERASE_SBA;
        *eraseCmd = 0x00;

        int error_status = MXC_FLC_PageErase(flashAddr);

        // Mass Erase Flash's Content
        if (error_status == E_NO_ERROR) {
            printf("Flash erased.\n");
        } else if (error_status == E_BAD_STATE) {
            printf("Flash erase operation is not allowed in this state.\n");
        } else {
            printf("Fail to erase flash's content.\n");
        }

        if (Flash_CheckErased(flashAddr)) {
            printf("Flash erase is verified.\n");
            isErased = 1;
        } else {
            printf("Flash erase failed.\n");
        }
    }

    if ((*writeCmd == FLASH_WRITE) & (isErased == 1)) {
        writeCmd++;
        flashAddr = *writeCmd++;
        flashAddr = (flashAddr - FLASH_HOST_SBA) + (FLASH_SBA);
        printf("\n\nFlash Address: %x", flashAddr);
        length = (*writeCmd++) / 4;
        printf("\nWriting %d 32-bit words to flash\n", length);
        data = writeCmd;
        writeCmd = (uint32_t *)FLASH_WRITE_SBA;
        *writeCmd = 0x00;
        isErased = 0;
        Flash_Write();
    }
}

//******************************************************************************
int main(void)
{
    printf("\n*********** Serial Flash Emulator Example************\n");
    printf("\nThis example initializes SFE slave. You can use any SPI\n");
    printf("Master or another MAX32520 running the SFE_Host Example\n");
    printf("\nAlso connect the SSEL1 pin to VCC while using SFE\n");

    MXC_SFE_Init();

    MXC_SFE_SetRAMAddress(RAM_SBA, RAM_STA);

    MXC_SFE_SetFlashAddress(FLASH_SBA, FLASH_STA);

    MXC_SFE_SetHostAddress(RAM_HOST_SBA, FLASH_HOST_SBA);

    MXC_SFE_WriteEnable();
    MXC_SFE_ReadEnable();

    *eraseCmd = 0x00;
    *writeCmd = 0x00;

    // Initialize the Flash
    Flash_Init();

    // Clear and enable flash programming interrupts
    Flash_InterruptEN(MXC_FLC0);

    isr_flags = 0;
    isr_cnt = 0;

    while (1) { Flash_CommandCheck(); }
}
