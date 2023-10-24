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
 * @brief   LittleFS and wear leveling example.
 * @details This example shows the basic functionality of the LittleFS
 *          file system, including it's ability to distibute wear across
 *          the flash memory.
 */

/***** Includes *****/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "cli.h"
#include "flash.h"
#include "file.h"
#include "lfs.h"
#include "main.h"
#include "mxc_device.h"

/***** Globals *****/
uint32_t start_block = LFS_START_PAGE;

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
    .block_count = LFS_PAGE_CNT,
    .cache_size = 16,
    .lookahead_size = 16,
    .block_cycles = 500,
};

//******************************************************************************
/**
 * @brief Application entry point
 * @return Exit code
 */
int main(void)
{
    lfs_t lfs; // File system instance
    char cmd_buf[CMD_MAX_SIZE];
    int cmd_len, err;

    printf("\n\n********** Wear Leveling Example **********\n");

    // mount the filesystem
    printf("Mounting the filesystem...\n");
    err = lfs_mount(&lfs, &cfg);

    // reformat if we can't mount the filesystem
    // this should only happen on the first boot
    if (err) {
        printf("Filesystem is invalid, formatting...\n");
        lfs_format(&lfs, &cfg);
        err = lfs_mount(&lfs, &cfg);
    }

    if (!err) {
        printf("Filesystem is mounted! Ready for commands.\n");
    } else {
        printf("Unable to initialize file system!\n");
        return E_BAD_STATE;
    }

    // Continue to receive and process commands until 'stop' command received
    while (err != E_SHUTDOWN) {
        printf("\ncmd> ");
        fflush(stdout);

        cmd_len = cmd_get(cmd_buf, CMD_MAX_SIZE);
        err = cmd_process(&lfs, cmd_buf, cmd_len);
    }

    // release any resources we were using
    lfs_unmount(&lfs);
    printf("\nFilesystem resources released.\n");

    printf("Example complete!\n");
    return E_NO_ERROR;
}
