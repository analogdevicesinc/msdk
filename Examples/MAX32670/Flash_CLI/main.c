/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
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
 * @brief   Flash Control Mass Erase & Write 32-bit enabled mode Example
 * @details This example shows how to mass erase the flash using the library
 *          and also how to Write and Verify 4 Words to the flash.
 */

/***** Includes *****/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "cli.h"
#include "crc.h"
#include "definitions.h"
#include "ecc_regs.h"
#include "flc.h"
#include "icc.h"
#include "uart.h"

/***** Functions *****/
int main(void)
{
    printf("\n\n*************** Flash Control CLI Example ***************\n");
    printf("\nThis example demonstrates various features of the Flash Controller");
    printf("\n(page erase and write), and how to use the CRC to compute the");
    printf("\nCRC value of an array. Enter commands in the terminal window.\n\n");

    while (MXC_UART_GetActive(MXC_UART_GET_UART(CONSOLE_UART))) {}

    MXC_ECC->en = 0; // Disable ECC on Flash, ICC, and SRAM

    // Set up CLI command table
    const command_t cmd_table[] = CMD_TABLE;
    const unsigned int cmd_table_sz = sizeof(cmd_table) / sizeof(command_t);

    // Initialize CLI
    if (MXC_CLI_Init(MXC_UART_GET_UART(CONSOLE_UART), cmd_table, cmd_table_sz) != E_NO_ERROR) {
        printf("Failed to initialize command-line interface.\n");
        return E_BAD_STATE;
    }

    // Run CLI
    while (1) {}
}

// *****************************************************************************
// ********************* Command Handler Functions *****************************
// *****************************************************************************
int handle_write(int argc, char *argv[])
{
    int err, i = 0;
    uint32_t data[MXC_FLASH_PAGE_SIZE / 4];

    // Check for an invalid command
    if (argc != 3 || argv == NULL) {
        printf("Invalid command format. Aborting flash write.\n");
        return E_BAD_PARAM;
    }

    // Get command-line arguments
    int startaddr = FLASH_STORAGE_START_ADDR + atoi(argv[WORD_OFFSET_POS]) * 4;
    char *text = argv[DATA_POS];

    // Convert character string to uint32_t since we must write flash in 32-bit words
    for (int i = 0; i < strlen(text); i++) {
        data[i] = (uint32_t)text[i];
    }

    // Check if flash controller is busy
    if (MXC_FLC0->ctrl & MXC_F_FLC_CTRL_PEND) {
        return E_BUSY;
    }

    // Check whether the flash we are attempting to write has already been written to
    if (!check_erased(startaddr, strlen(text))) {
        return E_INVALID;
    }

    MXC_ICC_Disable();

    // Write each character to flash
    for (uint32_t testaddr = startaddr; i < strlen(text); testaddr += 4, i++) {
        // Write a word
        err = MXC_FLC_Write(testaddr, 4, &data[i]);
        if (err != E_NO_ERROR) {
            printf("Failure in writing a word : error %i addr: 0x%08x\n", err, testaddr);
            return err;
        }

        printf("Write addr 0x%08X: %c\r\n", testaddr, data[i]);
    }

    MXC_ICC_Enable();

    // Verify the flash write was successful
    err = flash_verify(startaddr, strlen(text), data);
    if (err != E_NO_ERROR) {
        printf("Write failed with error %i\n", err);
    } else {
        printf("Success\n");
    }

    return E_NO_ERROR;
}

// *****************************************************************************
int handle_read(int argc, char *argv[])
{
    uint32_t addr;
    uint8_t data[MXC_FLASH_PAGE_SIZE / 4];

    // Check for an invalid command
    if (argc != 3 || argv == NULL) {
        printf("Invalid command format. Aborting flash read.\n");
        return E_BAD_PARAM;
    }

    // Get command-line arguments
    int startaddr = FLASH_STORAGE_START_ADDR + atoi(argv[WORD_OFFSET_POS]) * 4;
    int length = atoi(argv[LENGTH_POS]);

    // Initialize data buffer
    memset(data, 0x0, sizeof(data));

    // Read requested characters from flash
    for (int i = 0; i < length; i++) {
        addr = startaddr + i * 4;
        data[i] = *(uint32_t *)addr;

        if (data[i] == 0xFF) {
            printf("Read addr 0x%08X: %s\n", addr, "empty");
        } else {
            printf("Read addr 0x%08X: %c\n", addr, data[i]);
        }
    }

    printf("Success:\n");
    printf("%s\n", (char *)data);

    return E_NO_ERROR;
}

// *****************************************************************************
int handle_erase(int argc, char *argv[])
{
    int err = E_NO_ERROR;

    // Check for an invalid command
    if (argc != 1 || argv == NULL) {
        printf("Invalid command format. Aborting flash erase.\n");
        return E_BAD_PARAM;
    }

    // Check whether the flash page is already erased
    if (!check_erased(FLASH_STORAGE_START_ADDR, MXC_FLASH_PAGE_SIZE)) {
        // Erase flash page if it's not already erased
        if ((err = MXC_FLC_PageErase(FLASH_STORAGE_START_ADDR)) != E_NO_ERROR) {
            printf("Failed to erase flash page.\n");
            return err;
        }
    }

    printf("Success\n");

    return err;
}

// *****************************************************************************
int handle_crc(int argc, char *argv[])
{
    int err;
    mxc_crc_req_t req;

    // Check for an invalid command
    if (argc != 1 || argv == NULL) {
        printf("Invalid command format. Aborting CRC compute.\n");
        return E_BAD_PARAM;
    }

    // Setup CRC request to calculate CRC value for the entire flash page
    req.dataBuffer = (uint32_t *)FLASH_STORAGE_START_ADDR;
    req.dataLen = MXC_FLASH_PAGE_SIZE / sizeof(uint32_t);

    // Initialize CRC engine and compute CRC value
    MXC_CRC_Init();
    MXC_CRC_SetPoly(POLY);
    if ((err = MXC_CRC_Compute(&req)) != E_NO_ERROR) {
        return err;
    }

    // Print result
    printf("CRC: 0x%08X\r\n", req.resultCRC);

    return E_NO_ERROR;
}

// *****************************************************************************
// ************************* Helper Functions **********************************
// *****************************************************************************
int flash_verify(uint32_t address, uint32_t length, uint32_t *data)
{
    volatile uint32_t *ptr;

    // Loop through memory and check whether it matches the data array
    for (ptr = (uint32_t *)address; ptr < (uint32_t *)(address + length); ptr++, data++) {
        if (*ptr != *data) {
            printf("Verify failed at 0x%x (0x%x != 0x%x)\n", (unsigned int)ptr, (unsigned int)*ptr,
                   (unsigned int)*data);
            return E_UNKNOWN;
        }
    }

    return E_NO_ERROR;
}

// *****************************************************************************
int check_mem(uint32_t startaddr, uint32_t length, uint32_t data)
{
    uint32_t *ptr;

    // Loop through memory and check whether it matches the expected data value
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
    // Check whether flash memory is set to all 1's (erased state)
    return check_mem(startaddr, length, 0xFFFFFFFF);
}
