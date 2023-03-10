/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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
 * @file    flash.c
 * @brief   Flash read/write/erase functions implementation
 */

#include "flash.h"
#include <stdio.h>
#include "icc.h"
#include "flc.h"
#include "flc_regs.h"
#include "gcr_regs.h"

/***** Functions *****/

int flash_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer,
               lfs_size_t size)
{
    uint32_t first_block = *(uint32_t *)c->context; // Starting page of LittleFS flash space
    uint32_t startaddr = MXC_FLASH_PAGE_ADDR((first_block + block)) + off; // Start address of read
    uint8_t *data = (uint8_t *)buffer; // Pointer to data buffer

    // Copy Flash contents to data buffer
    for (uint8_t *ptr = (uint8_t *)startaddr; ptr < (uint8_t *)(startaddr + size); ptr++, data++) {
        *data = *ptr;
    }
    return LFS_ERR_OK;
}
//******************************************************************************

int flash_write(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer,
                lfs_size_t size)
{
    uint32_t first_block = *(uint32_t *)c->context; // Starting page of LittleFS flash space
    uint32_t startaddr = MXC_FLASH_PAGE_ADDR((first_block + block)) + off; //Start address of write
    uint32_t *data = (uint32_t *)buffer; // Pointer to data buffer

    // Write 4 words to Flash
    return flash_write4(startaddr, size / c->prog_size, data, FALSE);
}

//******************************************************************************

int flash_erase(const struct lfs_config *c, lfs_block_t block)
{
    uint32_t first_block = *(uint32_t *)c->context; // Starting page of LittleFS flash space
    int addr = MXC_FLASH_PAGE_ADDR((first_block + block)); // Address in flash page to erase
    LOGF("Erasing page at address %08x\n", addr);

    // Erase flash page
    int error_status = MXC_FLC_PageErase(addr);
    if (error_status != E_NO_ERROR) {
        return error_status;
    }
    return LFS_ERR_OK;
}

//******************************************************************************

int flash_sync(const struct lfs_config *c)
{
    // Not provided by the SDK
    return LFS_ERR_OK;
}

//******************************************************************************

int flash_verify(uint32_t address, uint32_t length, uint8_t *data)
{
    volatile uint8_t *ptr;

    // Loop through Flash checking whether it matches data buffer
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

int check_mem(uint32_t startaddr, uint32_t length, uint32_t data)
{
    uint32_t *ptr;

    // Loop through Flash comparing what's currently stored to expected value
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
    return check_mem(startaddr, length, 0xFFFFFFFF);
}

//******************************************************************************
int flash_write4(uint32_t startaddr, uint32_t length, uint32_t *data, bool verify)
{
    int i = 0;

    MXC_ICC_Disable();

    // Write data buffer to flash in 16-byte increments
    for (uint32_t testaddr = startaddr; i < length; testaddr += 16) {
        // Write 16-bytes to flash
        int error_status = MXC_FLC_Write(testaddr, 16, &data[i]);
        if (error_status != E_NO_ERROR) {
            printf("Failure in writing a word : error %i addr: 0x%08x\n", error_status, testaddr);
            return error_status;
        } else {
            LOGF("Word %u is written to the flash at addr 0x%08x\n", data[i], testaddr);
        }

        if (verify) {
            // Verify that word is written properly
            if (flash_verify(testaddr, 16, (uint8_t *)&data[i]) != E_NO_ERROR) {
                printf("Word is not written properly.\n");
                return E_UNKNOWN;
            }
        }
        i += 4;
    }

    MXC_ICC_Enable();

    return E_NO_ERROR;
}
