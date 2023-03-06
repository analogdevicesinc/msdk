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
 * @file    main.c
 * @brief   Flash Control Mass Erase & Write 32-bit enabled mode Example
 * @details This example shows how to mass erase the flash using the library
 *          and also how to Write and Verify 4 Words to the flash.
 */

/***** Includes *****/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "flc.h"
#include "mxc_assert.h"
#include "mxc_device.h"
#include "trimsir_regs.h"

#include "flash.h"
#include "lfs.h"

/***** Definitions *****/
// Modifiable LittleFS macros
#define LFS_PGE_CNT 8 ///< Number of flash pages reserved for LittleFS (Valid values 1-112)

// Non-modifiable LittleFS macros
#define LFS_START_PGE 16 ///< Page number of the first LittleFS block
#define LFS_START_ADDR MXC_FLASH_PAGE_ADDR(LFS_START_PGE) ///< Start address of LittleFS
#define LFS_SIZE (LFS_PGE_CNT * MXC_FLASH_PAGE_SIZE) ///< Size of LittleFS

// Set to 1 to perform erase, test data write, and test read back/verify over LittleFS Flash space
#define FLASH_TEST 0

// Size of FLASH_TEST data array
#define TESTSIZE (MXC_FLASH_PAGE_SIZE * LFS_PGE_CNT / 4) ///< LFS_PGE_CNT pages of 32 bit samples

/***** Globals *****/
uint32_t testdata[TESTSIZE]; ///< Test data buffer

// variables used by the filesystem
lfs_t lfs; ///< File system instance
uint32_t start_block = LFS_START_PGE; ///< Internal memory start block

// configuration of the filesystem
const struct lfs_config cfg = {
    .context = &start_block,

    // Function pointers to flash operations (in flash.h/.c)
    .read = flash_read,
    .prog = flash_write,
    .erase = flash_erase,
    .sync = flash_sync,

    // block device configuration
    .read_size = 1, //< 1-byte Flash reads
    .prog_size = 16, //< 16-byte Flash writes
    .block_size = MXC_FLASH_PAGE_SIZE, //< LFS block = 1 Flash page
    .block_count = LFS_PGE_CNT, //< Num flash pages reserved for LFS
    .cache_size = 16,
    .lookahead_size = 16,
    .block_cycles = 500,
};

//******************************************************************************
int main(void)
{
    int error_status = E_NO_ERROR;

    // Disable ECC on Flash 0 if necessary
    uint32_t ecc_status = (MXC_TRIMSIR->bb_sir2 & MXC_F_TRIMSIR_BB_SIR2_FL0ECCEN);
    if (ecc_status) {
        MXC_TRIMSIR->bb_sir2 &= ~MXC_F_TRIMSIR_BB_SIR2_FL0ECCEN;
    }

    printf("\n\n***** MAX32672 Wear Leveling *****\n");

#if FLASH_TEST == 1
    // Initializing Test Data
    for (int i = 0; i < TESTSIZE; i++) {
        testdata[i] = i;
    }

    //Erase page-by-page
    for (int i = 0; i < LFS_PGE_CNT; i++) {
        error_status = flash_erase(&cfg, i);
        if (error_status != E_NO_ERROR) {
            printf("Flash erase failed with error %i\n", error_status);
            return 1;
        }
    }

    // Check flash's content
    if (check_erased(LFS_START_ADDR, LFS_SIZE)) {
        printf("Flash erase is verified.\n");
    } else {
        printf("Flash erase failed.\n");
    }

    printf("Writing %d 32-bit words to flash\n", TESTSIZE);
    printf("Size of testdata : %d\n", sizeof(testdata));

    // Write test data to flash
    error_status = flash_write4(LFS_START_ADDR, TESTSIZE, testdata, TRUE);

    printf("Verifying %d 32-bit words in flash\n", TESTSIZE);
    printf("Size of testdata : %d\n", sizeof(testdata));

    // Read back flash and check whether it matches the test data stored during FULL_WRITE_TEST
    error_status = flash_verify(LFS_START_ADDR, TESTSIZE, (uint8_t *)testdata);
#endif

#if FLASH_TEST == 0
    lfs_file_t file;

    // Mount the filesystem
    error_status = lfs_mount(&lfs, &cfg);

    if (error_status) {
        // Mount failed --> format filesytem (this should only be necessary on the first boot)
        printf("Filesystem is invalid, formatting...\n");
        lfs_format(&lfs, &cfg);
        error_status = lfs_mount(&lfs, &cfg);
    } else {
        // Initial mount attempt successful
        printf("Filesystem is mounted\n");
    }

    // Read current boot count
    uint32_t boot_count = 0;
    lfs_file_open(&lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&lfs, &file, &boot_count, sizeof(boot_count));

    // Update boot count and write back to file
    boot_count += 1;
    lfs_file_rewind(&lfs, &file);
    lfs_file_write(&lfs, &file, &boot_count, sizeof(boot_count));

    // Closed successfully (storage is not updated until file is closed)
    lfs_file_close(&lfs, &file);

    // Release filesystem resources
    lfs_unmount(&lfs);

    // Print updated boot count
    printf("boot_count: %d\n", boot_count);
#endif

    if (error_status != E_NO_ERROR) {
        printf("\nExample Failed\n");
        return E_FAIL;
    }

    printf("\nExample Succeeded\n");

    // Re-enable ECC if necessary
    if (ecc_status) {
        MXC_TRIMSIR->bb_sir2 |= MXC_F_TRIMSIR_BB_SIR2_FL0ECCEN;
    }

    return E_NO_ERROR;
}
