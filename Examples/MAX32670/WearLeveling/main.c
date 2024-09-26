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
 * @brief   LittleFS and wear leveling example.
 * @details This example shows the basic functionality of the LittleFS
 *          file system, including it's ability to distibute wear across
 *          the flash memory.
 */

/***** Includes *****/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "board.h"
#include "cli.h"
#include "cmd_table.h"
#include "flash.h"
#include "file.h"
#include "led.h"
#include "lfs.h"
#include "mxc_device.h"
#include "uart.h"

/***** Definitions *****/
#define LFS_START_PAGE 32 // First flash memory block used by LFS
#define LFS_PAGE_CNT 16 // Number of flash memory blocks reserved for LFS

/***** Function Prototypes *****/
int handle_stop(int argc, char *argv[]);
int handle_read(int argc, char *argv[]);
int handle_write(int argc, char *argv[]);
int handle_swl(int argc, char *argv[]);
int mount_filesystem(lfs_t *filesystem, const struct lfs_config *cfg);

/***** Global Variables ******/
lfs_t lfs; // LFS filesystem instance
volatile bool stop_recv = false; // Used to signal whether the example has completed

/***** Function Definitions *****/
int main(void)
{
    int err;

    printf("\n\n*************** Wear Leveling Example ***************\n");

    // Create command table (defined cmd_table.h)
    const command_t cmd_table[] = CMD_TABLE;
    const unsigned int cmd_table_sz = sizeof(cmd_table) / sizeof(command_t);

    // Initialize LFS configuration variables
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

    printf("Mounting the file system...\n");

    if ((err = mount_filesystem(&lfs, &cfg)) != E_NO_ERROR) {
        printf("Unable to mount file system!\n");
        return err;
    }

    printf("File system is mounted!\n\n");
    while (MXC_UART_GetActive(MXC_UART_GET_UART(CONSOLE_UART))) {}

    // Initialize command-line interface
    if (MXC_CLI_Init(MXC_UART_GET_UART(CONSOLE_UART), cmd_table, cmd_table_sz) != E_NO_ERROR) {
        printf("Unable to initialize the command-line interface.\n");
        return E_BAD_STATE;
    }

    // CLI running in the background. Continue to receive and process commands until 'stop' command received
    while (!stop_recv) {}

    // release any resources we were using
    lfs_unmount(&lfs);
    printf("\nFilesystem resources released.\n");

    printf("Example complete!\n");
    return E_NO_ERROR;
}

//******************************************************************************
int handle_stop(int argc, char *argv[])
{
    stop_recv = true;
    return E_NO_ERROR;
}

//******************************************************************************
int handle_read(int argc, char *argv[])
{
    // Check for invalid arguments
    if (argc != 4 || argv == NULL) {
        printf("Invalid command. Aborting file read.\n");
        return E_NULL_PTR;
    }

    lfs_file_t file;
    char data[MAX_FILE_READ_SIZE];

    // Assign CLI arguments to appropriate variables
    char *filename = argv[FILENAME_POS];
    int num = atoi(argv[NUM_BYTES_POS]);
    int pos = atoi(argv[LOCATION_POS]);
    memset(data, '\0', sizeof(data));

    // Read data from file
    num = file_read(&lfs, &file, filename, data, num, pos);
    if (num < LFS_ERR_OK) {
        printf("Read failed with error code %d.\n", num);
        return num;
    } else {
        printf("%d bytes were read from %s in filesystem block %d.\n", num, filename, file.block);
    }

    // Print data read from file to the terminal
    printf("The following string was read from file %s:\n", filename);

    for (int i = 0; i < num; i++) {
        printf("%c", data[i]);
    }
    printf("\n");

    return E_NO_ERROR;
}

//******************************************************************************
int handle_write(int argc, char *argv[])
{
    // Check for invalid arguments
    if (argc != 4 || argv == NULL) {
        printf("Invalid command. Aborting file write.\n");
        return E_INVALID;
    }

    // Assign CLI arguments to appropriate variables
    lfs_file_t file;
    char *filename = argv[FILENAME_POS];
    {
    }
    char *data = argv[DATA_POS];
    int pos = atoi(argv[LOCATION_POS]);
    int err;

    // Write data to the file
    err = file_write(&lfs, &file, filename, data, strlen(data), pos, true);
    if (err < LFS_ERR_OK) {
        printf("Write failed with error code %d.\n", err);
    } else {
        printf("%d bytes were written to %s in filesystem block %d.\n", err, filename, file.block);
    }

    return err;
}

//******************************************************************************
int handle_swl(int argc, char *argv[])
{
    // Check for invalid arguments
    if (argc != 2 || argv == NULL) {
        printf("Invalid command format. Aborting swl.\n");
        return E_INVALID;
    }

    int num_writes, err;
    int hit_count[LFS_PAGE_CNT] = { 0 };

    // Assign CLI arguments to appropriate variables
    num_writes = atoi(argv[NUM_WRITES_POS]);

    //Set up dummy arguments
    char filename[] = "swl_test_file";
    char data[] = "show_littlefs_wear_leveling"; // Length of this string must exceed lfs.cache_size
    lfs_file_t file;

    // Write to the test file the specified number of writes and
    // track how many times each flash page is written to
    for (int i = 0; i < num_writes; i++) {
        // Do next write
        err = file_write(&lfs, &file, filename, data, strlen(data), 0, true);
        if (err < LFS_ERR_OK) {
            printf("Failed to write to test file. Aborting \"swl\" command.\n");
            return err;
        }

        // Increment the hit count
        if (file.block >= 0 && file.block < LFS_PAGE_CNT) {
            hit_count[file.block]++;
        }

        // Heartbeat, this loop can take a while if num_writes is large
        if (i % 50 == 0) {
            LED_Toggle(0);
        }
    }

    // Print results
    printf("All writes have completed. Here are the results:\n");
    for (int i = 0; i < LFS_PAGE_CNT; i++) {
        printf("Block %d was written to %d times.\n", i, hit_count[i]);
    }
    printf("\n");

    return E_NO_ERROR;
}

//******************************************************************************
int mount_filesystem(lfs_t *filesystem, const struct lfs_config *cfg)
{
    int err;

    // mount the file system
    err = lfs_mount(filesystem, cfg);

    // reformat if we can't mount the filesystem
    // this should only happen on the first boot
    if (err) {
        printf("Filesystem is invalid, formatting...\n");
        lfs_format(filesystem, cfg);
        err = lfs_mount(filesystem, cfg);
    }

    if (err) {
        return E_BAD_STATE;
    }

    return err;
}
