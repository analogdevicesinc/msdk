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
 * @brief   External flash example.
 * @details Demonstrate how external flash can be driven over SPI interface. 
 *          To see low level SPI function call, see board.c file
 *          Erases, writes, and then verifies the data. 
 *          EXT_FLASH_BAUD, EXT_FLASH_ADDR, and EXT_FLASH_SPIXFC_WIDTH
 *          can be changed to alter the communication between the devices. Refer
 *          to the schematic for the pinout and ensure that there are no switches
 *          blocking the communication to the external flash.
 */

/***** Includes *****/
#include <stdio.h>
#include <string.h>
#include "mxc_sys.h"
#include "mxc_errors.h"
#include "max32655.h"
#include "Ext_Flash.h"
#include "board.h"
#include "led.h"

/***** Definitions *****/

#define EXT_FLASH_ADDR 0
#define EXT_FLASH_SPIXFC_WIDTH Ext_Flash_DataLine_Quad

#define BUFF_SIZE 64

/***** Functions *****/

static char data_to_write[] = { "Analog Devices" };
static char data_to_read[32];

/******************************************************************************/
int main(void)
{
    int fail = 0;
    uint32_t id;
    int rx_len = sizeof(data_to_write) / sizeof(char);
    int remain = rx_len;

    printf("\n\n********************* External Flash Example *********************\n");
    printf("This example communicates with an %s flash over SPI (Quad mode)\n\n", EXT_FLASH_NAME);

    printf("SPI Clock: %d Hz\n\n", EXT_FLASH_BAUD);

    // Initialize external flash
    if (Ext_Flash_Init() != E_NO_ERROR) {
        printf("Board Init Failed\n");
        printf("Example Failed\n");
        while (1) {}
    }
    printf("External flash Initialized.\n\n");

    Ext_Flash_Reset();

    // Get the ID of the external flash
    id = Ext_Flash_ID();
    printf("External flash ID: 0x%x\n", id);

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
           (uint32_t)(sizeof(data_to_write)), data_to_write);

    if ((err = Ext_Flash_Program_Page(EXT_FLASH_ADDR, (uint8_t *)data_to_write,
                                      (uint32_t)(sizeof(data_to_write) / sizeof(char)),
                                      EXT_FLASH_SPIXFC_WIDTH)) != E_NO_ERROR) {
        printf("Error Programming: %d\n", err);
        fail++;
    } else {
        printf("Programmed\n");
        printf("Written Data:%s\n\n", data_to_write);
    }

    printf("Verifying external flash\n");
    while (remain) {
        int chunk = ((remain > BUFF_SIZE) ? BUFF_SIZE : remain);
        if ((err = Ext_Flash_Read(EXT_FLASH_ADDR + rx_len - remain, (uint8_t *)data_to_read, chunk,
                                  EXT_FLASH_SPIXFC_WIDTH)) != E_NO_ERROR) {
            printf("Error verifying data %d\n", err);
            fail++;
            break;
        } else if (memcmp(data_to_read, data_to_write + rx_len - remain, chunk) != E_NO_ERROR) {
            printf("Error invalid data\n");
            fail++;
            break;
        } else if (remain == chunk) {
            printf("Verified\n");
            printf("Read Data:%s\n\n", data_to_read);
        }
        remain -= chunk;
    }

    if (fail != 0) {
        printf("\nExample Failed\n");
        return E_FAIL;
    }

    printf("\nExample Succeeded\n");
    return E_NO_ERROR;
}
