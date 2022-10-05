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
 * @brief   SPIX example using the external flash.
 * @details Uses the external flash on the EvKit to show the SPIX. Erases, writes, and then
 *          verifies the data. EXT_FLASH_BAUD, EXT_FLASH_ADDR, and EXT_FLASH_SPIXFC_WIDTH
 *          can be changed to alter the communication between the devices. Refer
 *          to the schematic for the pinout and ensure that there are no switches
 *          blocking the communication to the external flash.
 */

/***** Includes *****/
#include <stdio.h>
#include <string.h>

#include <MAX32xxx.h>
#include "Ext_Flash.h"

/***** Definitions *****/

#define EXT_FLASH_ADDR 0
#define EXT_FLASH_SPIXFC_WIDTH Ext_Flash_DataLine_Quad

#define BUFF_SIZE   64

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
uint8_t *__xip_addr;
#endif

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

/******************************************************************************/
int main(void)
{
    uint32_t id;
    void (*func)(void);
    uint8_t rx_buf[BUFF_SIZE];
    int rx_len = (uint32_t)(&__load_length_xip);
    int remain = rx_len;

    printf("\n\n********************* SPIX Example *********************\n");
    printf("This example communicates with an %s flash on the EvKit\n", EXT_FLASH_NAME);
    printf("loads code onto it and then executes that code using the \n");
    printf("SPIX execute-in-place peripheral\n\n");

    printf("SPI Clock: %d Hz\n\n", EXT_FLASH_BAUD);

    // Initialize the SPIXFC registers and set the appropriate output pins
    if (Ext_Flash_Init() != E_NO_ERROR) {
        printf("Board Init Failed\n");
        printf("Example Failed\n");
        while (1) {}
    }
    printf("External flash Initialized.\n\n");

    Ext_Flash_Reset();

    // Get the ID of the external flash
    if ((id = Ext_Flash_ID()) == EXT_FLASH_EXP_ID) {
        printf("External flash ID verified\n\n");
    } else {
        printf("Error verifying external flash ID: 0x%x\n", id);
        printf("Example Failed\n");
        while (1) {}
    }

    int err;

    // Erase Test Sector
    printf("Erasing first 64k sector\n");
    Ext_Flash_Erase(0x00000, Ext_Flash_Erase_64K);
    printf("Erased\n\n");

    // Enable Quad mode if we are using quad
    if (EXT_FLASH_SPIXFC_WIDTH == Ext_Flash_DataLine_Quad) {
        if (Ext_Flash_Quad(1) != E_NO_ERROR) {
            printf("Error enabling quad mode\n\n");
            fail++;
        } else {
            printf("Quad mode enabled\n\n");
        }
    } else {
        if (Ext_Flash_Quad(0) != E_NO_ERROR) {
            printf("Error disabling quad mode\n\n");
            fail++;
        } else {
            printf("Quad mode disabled\n\n");
        }
    }

    // Program the external flash
    printf("Programming function (%d bytes @ 0x%08x) into external flash\n",
           (uint32_t)(&__load_length_xip), &__load_start_xip);
    if ((err = Ext_Flash_Program_Page(EXT_FLASH_ADDR, &__load_start_xip,
                                      (uint32_t)(&__load_length_xip), EXT_FLASH_SPIXFC_WIDTH)) !=
        E_NO_ERROR) {
        printf("Error Programming: %d\n", err);
        fail++;
    } else {
        printf("Programmed\n\n");
    }

    printf("Verifying external flash\n");
    while (remain) {
        int chunk = ((remain > BUFF_SIZE) ? BUFF_SIZE : remain);
        if ((err = Ext_Flash_Read(EXT_FLASH_ADDR + rx_len - remain, rx_buf, chunk,
                                  EXT_FLASH_SPIXFC_WIDTH)) !=
            E_NO_ERROR) {
            printf("Error verifying data %d\n", err);
            fail++;
            break;
        } else if (memcmp(rx_buf, &__load_start_xip + rx_len - remain, chunk) != E_NO_ERROR) {
            printf("Error invalid data\n");
            fail++;
            break;
        } else if (remain == chunk) {
            printf("Verified\n\n");
        }
        remain -= chunk;
    }

    // Setup SPIX
    spixf_cfg_setup();

    printf("Jumping to external flash (@ 0x%08x), watch for blinking LED.\n\n",
           (MXC_XIP_MEM_BASE | 0x1));
    func = (void (*)(void))(MXC_XIP_MEM_BASE | 0x1);
    func();
    printf("Returned from external flash\n\n");

    if (fail == 0) {
        printf("Example Succeeded\n\n");
    } else {
        printf("Example Failed\n\n");
    }
    return 0;
}
