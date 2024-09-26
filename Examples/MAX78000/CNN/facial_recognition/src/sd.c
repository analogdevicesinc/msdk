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

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "gpio.h"
#include "uart.h"
#include "ff.h"
#include "cnn_1.h"
#include "MAXCAM_Debug.h"
#define S_MODULE_NAME "sd"

#ifdef BOARD_EVKIT_V1
#warning This example is not supported by the MAX78000EVKIT.
#endif

/***** Definitions *****/

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#define MAXLEN 256

/***** Globals *****/
FATFS *fs; //FFat Filesystem Object
FATFS fs_obj;
FIL file; //FFat File Object
FRESULT err; //FFat Result (Struct)
FILINFO fno; //FFat File Information Object
DIR dir; //FFat Directory Object
TCHAR message[MAXLEN], directory[MAXLEN], cwd[MAXLEN], filename[MAXLEN], volume_label[24],
    volume = '0';
TCHAR *FF_ERRORS[20];
DWORD clusters_free = 0, sectors_free = 0, sectors_total = 0, volume_sn = 0;
UINT bytes_written = 0, bytes_read = 0, mounted = 0;
uint32_t total_bytes;
uint32_t kernel_buffer[2048]; //5K seems to be limit for this app.
BYTE work[2048];
static char charset[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789,.-#'?!";
mxc_gpio_cfg_t SDPowerEnablePin = { MXC_GPIO1, MXC_GPIO_PIN_12, MXC_GPIO_FUNC_OUT,
                                    MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO };

/***** FUNCTIONS *****/

void generateMessage(unsigned length)
{
    for (int i = 0; i < length; i++) {
        /*Generate some random data to put in file*/
        message[i] = charset[rand() % (sizeof(charset) - 1)];
    }
}

int mount(void)
{
    fs = &fs_obj;

    if ((err = f_mount(fs, "", 1)) != FR_OK) { //Mount the default drive to fs now
        PR_INFO("Error opening SD card: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
    } else {
        PR_INFO("SD card mounted.\n");
        mounted = 1;
    }

    f_getcwd(cwd, sizeof(cwd)); //Set the Current working directory

    return err;
}

int umount(void)
{
    if ((err = f_mount(NULL, "", 0)) != FR_OK) { //Unmount the default drive from its mount point
        PR_INFO("Error unmounting volume: %s\n", FF_ERRORS[err]);
    } else {
        PR_INFO("SD card unmounted.\n");
        mounted = 0;
    }

    return err;
}

void waitCardInserted(void)
{
    // On the MAX78000FTHR board, P0.12 will be pulled low when a card is inserted.
    mxc_gpio_cfg_t cardDetect;
    cardDetect.port = MXC_GPIO0;
    cardDetect.mask = MXC_GPIO_PIN_12;
    cardDetect.func = MXC_GPIO_FUNC_IN;
    cardDetect.pad = MXC_GPIO_PAD_NONE;
    cardDetect.vssel = MXC_GPIO_VSSEL_VDDIOH;

    MXC_GPIO_Config(&cardDetect);

    // Exit function if card is already inserted
    if (MXC_GPIO_InGet(MXC_GPIO0, MXC_GPIO_PIN_12) == 0) {
        return;
    }

    PR_INFO("Insert SD card to continue.\n");

    while (MXC_GPIO_InGet(MXC_GPIO0, MXC_GPIO_PIN_12) != 0) {
        // Spin waiting for card to be inserted.
    }

    // Card has been detected, exit the function.
}

/******************************************************************************/
void SD_Init(void)
{
    FF_ERRORS[0] = "FR_OK";
    FF_ERRORS[1] = "FR_DISK_ERR";
    FF_ERRORS[2] = "FR_INT_ERR";
    FF_ERRORS[3] = "FR_NOT_READY";
    FF_ERRORS[4] = "FR_NO_FILE";
    FF_ERRORS[5] = "FR_NO_PATH";
    FF_ERRORS[6] = "FR_INVLAID_NAME";
    FF_ERRORS[7] = "FR_DENIED";
    FF_ERRORS[8] = "FR_EXIST";
    FF_ERRORS[9] = "FR_INVALID_OBJECT";
    FF_ERRORS[10] = "FR_WRITE_PROTECTED";
    FF_ERRORS[11] = "FR_INVALID_DRIVE";
    FF_ERRORS[12] = "FR_NOT_ENABLED";
    FF_ERRORS[13] = "FR_NO_FILESYSTEM";
    FF_ERRORS[14] = "FR_MKFS_ABORTED";
    FF_ERRORS[15] = "FR_TIMEOUT";
    FF_ERRORS[16] = "FR_LOCKED";
    FF_ERRORS[17] = "FR_NOT_ENOUGH_CORE";
    FF_ERRORS[18] = "FR_TOO_MANY_OPEN_FILES";
    FF_ERRORS[19] = "FR_INVALID_PARAMETER";

    //PR_INFO("\n\n***** " TOSTRING(TARGET) " SDHC FAT Filesystem *****\n");

    waitCardInserted();

    PR_INFO("Card inserted\n");

    f_getcwd(cwd, sizeof(cwd));

    err = 0;

    if ((err = mount()) != FR_OK) {
        PR_INFO("Error opening SD Card: %s\n", FF_ERRORS[err]);
        while (1)
            ;
    }

    PR_INFO("SD Card Opened\n");
}

uint32_t *read_weights_from_SD(void)
{
    // Read from SD file
    if ((err = f_read(&file, (uint8_t *)kernel_buffer, sizeof(kernel_buffer), &bytes_read)) !=
        FR_OK) {
        PR_INFO("ERROR reading file: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        while (1)
            ;
    }

    total_bytes += bytes_read;
    // Adjust position in file
    f_lseek(&file, total_bytes);
    //PR_INFO("%d bytes read\n", bytes_read);

    return &kernel_buffer[0];
}

int cnn_2_load_weights_from_SD(void)
{
    int i;
    int buffer_size;
    uint32_t len;
    volatile uint32_t *addr;
    const uint32_t *ptr;

    if ((err = f_open(&file, "weights_2.bin", FA_READ)) != FR_OK) {
        PR_INFO("ERROR opening file: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        while (1)
            ;
    }

    //PR_INFO("Opened file 'weights_2.bin'\n");

    total_bytes = 0;
    buffer_size = sizeof(kernel_buffer);
    //PR_INFO("Size of kernel buffer: %d\n", buffer_size);
    buffer_size >>= 2; // size in words
    // Set beginning of the file
    f_lseek(&file, 0);
    // Copy weights from SD file to intermediate kernel buffer in SRAM
    ptr = read_weights_from_SD();
    i = buffer_size;

    while ((addr = (volatile uint32_t *)*ptr++) != 0) {
        *((volatile uint8_t *)((uint32_t)addr | 1)) = 0x01; // Set CNN address
        i--;

        // Check if end of the kernel buffer is reached
        if (i == 0) {
            // Copy more weights from SD file to buffer
            ptr = read_weights_from_SD();
            i = buffer_size;
        }

        // Get number of weights
        len = *ptr++;
        i--;

        // Check if end of the kernel buffer is reached
        if (i == 0) {
            // Copy more weights from SD file to buffer
            ptr = read_weights_from_SD();
            i = buffer_size;
        }

        while (len-- > 0) {
            // Load weights to CNN
            *addr++ = *ptr++;
            i--;

            // Check if end of the kernel buffer is reached
            if (i == 0) {
                // Copy more weights from SD file to buffer
                ptr = read_weights_from_SD();
                i = buffer_size;
            }
        }
    }

    //PR_INFO("%d total bytes read\n", total_bytes);

    if ((err = f_close(&file)) != FR_OK) {
        PR_INFO("ERROR closing file: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        while (1)
            ;
    }

    //PR_INFO("File Closed\n");

    return CNN_OK;
}
