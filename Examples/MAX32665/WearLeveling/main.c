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
 * @brief   Flash Control Mass Erase & Write 32-bit enabled mode Example
 * @details This example shows how to mass erase the flash using the library
 *          and also how to Write and Verify 4 Words to the flash.
 */

/***** Includes *****/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "mxc_assert.h"
#include "mxc_device.h"
#include "flc.h"

#include "flash.h"
#include "lfs.h"

/***** Definitions *****/
#define APP_PAGE_CNT	8///< Flash memory blocks reserved for the app code
#define APP_SIZE		(MXC_FLASH_PAGE_SIZE*APP_PAGE_CNT)///< The app code flash memory area size
#define TESTSIZE        (MXC_FLASH_PAGE_SIZE*8/4)       ///< 8 pages of 32 bit samples
#define TOTAL_FLASH_PAGES		(MXC_FLASH_MEM_SIZE / MXC_FLASH_PAGE_SIZE)///< Flash memory blocks reserved for internal storage
#define FLASH_STORAGE_START_PAGE	8///< Internal storage first flash memory block
#define FLASH_STORAGE_PAGE_CNT	8///< Flash memory blocks reserved for the internal storage
#define FLASH_STORAGE_START_ADDR	MXC_FLASH_PAGE_ADDR(FLASH_STORAGE_START_PAGE)///< Internal storage start address
#define FLASH_STORAGE_SIZE		FLASH_STORAGE_PAGE_CNT * MXC_FLASH_PAGE_SIZE///< Internal storage size

// When set to 1 performs full storage erase and test data write
#ifndef FULL_WRITE_TEST
#define FULL_WRITE_TEST	0
#endif

// When set to 1 performs full storage test data read. Passes only if FULL_WRITE_TEST was performed before
#ifndef FULL_READ_TEST
#define FULL_READ_TEST	0
#endif

/***** Globals *****/
uint32_t testdata[TESTSIZE];///< Test data buffer

// variables used by the filesystem
lfs_t lfs;///< File system instance
uint32_t start_block = FLASH_STORAGE_START_PAGE;///< Internal memory start block to be passed to flash functions by littlefs
// configuration of the filesystem is provided by this struct
const struct lfs_config cfg = {
		.context = &start_block,
		// block device operations
		.read = flash_read,
		.prog = flash_write,
		.erase = flash_erase,
		.sync = flash_sync,

		// block device configuration
		.read_size = 1,
		.prog_size = 4,
		.block_size = MXC_FLASH_PAGE_SIZE,
		.block_count = FLASH_STORAGE_PAGE_CNT,
		.cache_size = 16,
		.lookahead_size = 16,
		.block_cycles = 500,
};

//******************************************************************************
/**
 * @brief Application entry point
 * @return Exit code
 */
int main(void) {
	int error_status = E_NO_ERROR;

	printf("\n\n***** MAX32665 Wear Leveling *****\n");

#if (FULL_WRITE_TEST == 1) || (FULL_READ_TEST == 1)
    // Initializing Test Data
     for (int i = 0; i < TESTSIZE; i++) {
         testdata[i] = i;
     }
#endif
#if FULL_WRITE_TEST == 1
    //Erase page-by-page
    for(int i = 0; i < FLASH_STORAGE_PAGE_CNT; i++) {
    	error_status = flash_erase(&cfg, i);
    	if(error_status != E_NO_ERROR) {
    		printf("Flash erase failed with error %i\n", error_status);
    		return 1;
    	}
    }

    // Check flash's content
    if (check_erased(FLASH_STORAGE_START_ADDR, FLASH_STORAGE_SIZE)) {
        printf("Flash erase is verified.\n");
    }
    else {
        printf("Flash erase failed.\n");

    }

    printf("Writing %d 32-bit words to flash\n", TESTSIZE);
    printf("Size of testdata : %d\n", sizeof(testdata));
    
    error_status = flash_write4(FLASH_STORAGE_START_ADDR, TESTSIZE, testdata, TRUE);

#endif
#if FULL_READ_TEST == 1
    printf("Verifying %d 32-bit words in flash\n", TESTSIZE);
    printf("Size of testdata : %d\n", sizeof(testdata));
    error_status = flash_verify(FLASH_STORAGE_START_ADDR, TESTSIZE, (uint8_t*)testdata);
#endif

#if (FULL_WRITE_TEST == 0) && (FULL_READ_TEST == 0)
    lfs_file_t file;
	// mount the filesystem
    error_status = lfs_mount(&lfs, &cfg);

	// reformat if we can't mount the filesystem
	// this should only happen on the first boot
	if (error_status) {
		printf("Filesystem is invalid, formatting...\n");
		lfs_format(&lfs, &cfg);
		error_status = lfs_mount(&lfs, &cfg);
	}

	if (!error_status) {
		printf("Filesystem is mounted\n");
	}

	// read current count
	uint32_t boot_count = 0;
	lfs_file_open(&lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
	lfs_file_read(&lfs, &file, &boot_count, sizeof(boot_count));

	// update boot count
	boot_count += 1;
	lfs_file_rewind(&lfs, &file);
	lfs_file_write(&lfs, &file, &boot_count, sizeof(boot_count));

	// remember the storage is not updated until the file is closed successfully
	lfs_file_close(&lfs, &file);

	// release any resources we were using
	lfs_unmount(&lfs);

	// print the boot count
	printf("boot_count: %d\n", boot_count);
#endif

	if (error_status != E_NO_ERROR) {
		printf("\nExample Failed\n");
	} else {
		printf("\nExample Succeeded\n");
	}

	while (1);

	return 0;
}
