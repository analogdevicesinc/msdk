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
#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "gpio.h"
#include "uart.h"
#include "ff.h"
#include "cnn_1.h"

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
uint32_t kernel_buffer[1000];
BYTE work[4096];
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

int mount()
{
    fs = &fs_obj;

    if ((err = f_mount(fs, "", 1)) != FR_OK) { //Mount the default drive to fs now
        printf("Error opening SD card: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
    } else {
        printf("SD card mounted.\n");
        mounted = 1;
    }

    f_getcwd(cwd, sizeof(cwd)); //Set the Current working directory

    return err;
}

int umount()
{
    if ((err = f_mount(NULL, "", 0)) != FR_OK) { //Unmount the default drive from its mount point
        printf("Error unmounting volume: %s\n", FF_ERRORS[err]);
    } else {
        printf("SD card unmounted.\n");
        mounted = 0;
    }

    return err;
}

int formatSDHC()
{
    printf("\n\n*****THE DRIVE WILL BE FORMATTED IN 5 SECONDS*****\n");
    printf("**************PRESS ANY KEY TO ABORT**************\n\n");
    MXC_UART_ClearRXFIFO(MXC_UART0);
    MXC_Delay(MSEC(5000));

    if (MXC_UART_GetRXFIFOAvailable(MXC_UART0) > 0) {
        return E_ABORT;
    }

    printf("FORMATTING DRIVE\n");

    if ((err = f_mkfs("", FM_ANY, 0, work, sizeof(work))) !=
        FR_OK) { //Format the default drive to FAT32
        printf("Error formatting SD card: %s\n", FF_ERRORS[err]);
    } else {
        printf("Drive formatted.\n");
    }

    mount();

    if ((err = f_setlabel("MAXIM")) != FR_OK) {
        printf("Error setting drive label: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
    }

    umount();

    return err;
}

int getSize()
{
    if (!mounted) {
        mount();
    }

    if ((err = f_getfree(&volume, &clusters_free, &fs)) != FR_OK) {
        printf("Error finding free size of card: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
    }

    sectors_total = (fs->n_fatent - 2) * fs->csize;
    sectors_free = clusters_free * fs->csize;

    printf("Disk Size: %u bytes\n", sectors_total / 2);
    printf("Available: %u bytes\n", sectors_free / 2);

    return err;
}

int ls()
{
    if (!mounted) {
        mount();
    }

    printf("Listing Contents of %s - \n", cwd);

    if ((err = f_opendir(&dir, cwd)) == FR_OK) {
        while (1) {
            err = f_readdir(&dir, &fno);

            if (err != FR_OK || fno.fname[0] == 0) {
                break;
            }

            printf("%s/%s", cwd, fno.fname);

            if (fno.fattrib & AM_DIR) {
                printf("/");
            }

            printf("\n");
        }

        f_closedir(&dir);
    } else {
        printf("Error opening directory!\n");
        return err;
    }

    printf("\nFinished listing contents\n");

    return err;
}

int createFile()
{
    unsigned int length = 128;

    if (!mounted) {
        mount();
    }

    printf("Enter the name of the text file: \n");
    scanf("%255s", filename);
    printf("Enter the length of the file: (%d max)\n", MAXLEN);
    scanf("%d", &length);

    if (length > MAXLEN) {
        printf("Error. File size limit for this example is %d bytes.\n", MAXLEN);
        return FR_INVALID_PARAMETER;
    }

    printf("Creating file %s with length %d\n", filename, length);

    if ((err = f_open(&file, (const TCHAR *)filename, FA_CREATE_ALWAYS | FA_WRITE)) != FR_OK) {
        printf("Error opening file: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        return err;
    }

    printf("File opened!\n");

    generateMessage(length);

    if ((err = f_write(&file, &message, length, &bytes_written)) != FR_OK) {
        printf("Error writing file: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        return err;
    }

    printf("%d bytes read\n", bytes_read);

    if ((err = f_close(&file)) != FR_OK) {
        printf("Error closing file: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        return err;
    }

    printf("File Closed!\n");
    return err;
}

int appendFile()
{
    unsigned int length = 0;

    if (!mounted) {
        mount();
    }

    printf("Enter name of file to append: \n");
    scanf("%255s", filename);
    printf("Enter length of random data to append: (%d max)\n", MAXLEN);
    scanf("%d", &length);

    if ((err = f_stat((const TCHAR *)filename, &fno)) == FR_NO_FILE) {
        printf("File %s doesn't exist!\n", (const TCHAR *)filename);
        return err;
    }

    if (length > MAXLEN) {
        printf("Error. Size limit for this example is %d bytes.\n", MAXLEN);
        return FR_INVALID_PARAMETER;
    }

    if ((err = f_open(&file, (const TCHAR *)filename, FA_OPEN_APPEND | FA_WRITE)) != FR_OK) {
        printf("Error opening file %s\n", FF_ERRORS[err]);
        return err;
    }

    printf("File opened!\n");

    generateMessage(length);

    if ((err = f_write(&file, &message, length, &bytes_written)) != FR_OK) {
        printf("Error writing file: %s\n", FF_ERRORS[err]);
        return err;
    }

    printf("%d bytes written to file\n", bytes_written);

    if ((err = f_close(&file)) != FR_OK) {
        printf("Error closing file: %s\n", FF_ERRORS[err]);
        return err;
    }

    printf("File closed.\n");
    return err;
}

int mkdir()
{
    if (!mounted) {
        mount();
    }

    printf("Enter directory name: \n");
    scanf("%255s", directory);

    err = f_stat((const TCHAR *)directory, &fno);

    if (err == FR_NO_FILE) {
        printf("Creating directory...\n");

        if ((err = f_mkdir((const TCHAR *)directory)) != FR_OK) {
            printf("Error creating directory: %s\n", FF_ERRORS[err]);
            f_mount(NULL, "", 0);
            return err;
        } else {
            printf("Directory %s created.\n", directory);
        }

    } else {
        printf("Directory already exists.\n");
    }

    return err;
}

int cd()
{
    if (!mounted) {
        mount();
    }

    printf("Directory to change into: \n");
    scanf("%255s", directory);

    if ((err = f_stat((const TCHAR *)directory, &fno)) == FR_NO_FILE) {
        printf("Directory doesn't exist (Did you mean mkdir?)\n");
        return err;
    }

    if ((err = f_chdir((const TCHAR *)directory)) != FR_OK) {
        printf("Error in chdir: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        return err;
    }

    printf("Changed to %s\n", directory);
    f_getcwd(cwd, sizeof(cwd));

    return err;
}

int delete ()
{
    if (!mounted) {
        mount();
    }

    printf("File or directory to delete (always recursive!)\n");
    scanf("%255s", filename);

    if ((err = f_stat((const TCHAR *)filename, &fno)) == FR_NO_FILE) {
        printf("File or directory doesn't exist\n");
        return err;
    }

    if ((err = f_unlink(filename)) != FR_OK) {
        printf("Error deleting file\n");
        return err;
    }

    printf("Deleted file %s\n", filename);
    return err;
}

int example()
{
    unsigned int length = 256;

    if ((err = formatSDHC()) != FR_OK) {
        printf("Error Formatting SD Card: %s\n", FF_ERRORS[err]);
        return err;
    }

    //open SD Card
    if ((err = mount()) != FR_OK) {
        printf("Error opening SD Card: %s\n", FF_ERRORS[err]);
        return err;
    }

    printf("SD Card Opened!\n");

    if ((err = f_setlabel("MAXIM")) != FR_OK) {
        printf("Error setting drive label: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        return err;
    }

    if ((err = f_getfree(&volume, &clusters_free, &fs)) != FR_OK) {
        printf("Error finding free size of card: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        return err;
    }

    if ((err = f_getlabel(&volume, volume_label, &volume_sn)) != FR_OK) {
        printf("Error reading drive label: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        return err;
    }

    if ((err = f_open(&file, "0:HelloWorld.txt", FA_CREATE_ALWAYS | FA_WRITE)) != FR_OK) {
        printf("Error opening file: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        return err;
    }

    printf("File opened!\n");

    generateMessage(length);

    if ((err = f_write(&file, &message, length, &bytes_written)) != FR_OK) {
        printf("Error writing file: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        return err;
    }

    printf("%d bytes written to file!\n", bytes_written);

    if ((err = f_close(&file)) != FR_OK) {
        printf("Error closing file: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        return err;
    }

    printf("File Closed!\n");

    if ((err = f_chmod("HelloWorld.txt", 0, AM_RDO | AM_ARC | AM_SYS | AM_HID)) != FR_OK) {
        printf("Error in chmod: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        return err;
    }

    err = f_stat("MaximSDHC", &fno);

    if (err == FR_NO_FILE) {
        printf("Creating Directory...\n");

        if ((err = f_mkdir("MaximSDHC")) != FR_OK) {
            printf("Error creating directory: %s\n", FF_ERRORS[err]);
            f_mount(NULL, "", 0);
            return err;
        }
    }

    printf("Renaming File...\n");

    if ((err = f_rename("0:HelloWorld.txt", "0:MaximSDHC/HelloMaxim.txt")) !=
        FR_OK) { //cr: clearify 0:file notation
        printf("Error moving file: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        return err;
    }

    if ((err = f_chdir("/MaximSDHC")) != FR_OK) {
        printf("Error in chdir: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        return err;
    }

    printf("Attempting to read back file...\n");

    if ((err = f_open(&file, "HelloMaxim.txt", FA_READ)) != FR_OK) {
        printf("Error opening file: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        return err;
    }

    if ((err = f_read(&file, &message, bytes_written, &bytes_read)) != FR_OK) {
        printf("Error reading file: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        return err;
    }

    printf("Read Back %d bytes\n", bytes_read);
    printf("Message: ");
    printf("%s", message);
    printf("\n");

    if ((err = f_close(&file)) != FR_OK) {
        printf("Error closing file: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        return err;
    }

    printf("File Closed!\n");

    //unmount SD Card
    //f_mount(fs, "", 0);
    if ((err = f_mount(NULL, "", 0)) != FR_OK) {
        printf("Error unmounting volume: %s\n", FF_ERRORS[err]);
        return err;
    }

    return 0;
}

void waitCardInserted()
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

    printf("Insert SD card to continue.\n");

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

    //printf("\n\n***** " TOSTRING(TARGET) " SDHC FAT Filesystem *****\n");

    waitCardInserted();

    printf("Card inserted\n");

    f_getcwd(cwd, sizeof(cwd));

    err = 0;

    if ((err = mount()) != FR_OK) {
        printf("Error opening SD Card: %s\n", FF_ERRORS[err]);
        while (1)
            ;
    }

    printf("SD Card Opened\n");
}

uint32_t *read_weights_from_SD(void)
{
    // Read from SD file
    if ((err = f_read(&file, (uint8_t *)kernel_buffer, sizeof(kernel_buffer), &bytes_read)) !=
        FR_OK) {
        printf("ERROR reading file: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        while (1)
            ;
    }

    total_bytes += bytes_read;
    // Adjust position in file
    f_lseek(&file, total_bytes);
    //printf("%d bytes read\n", bytes_read);

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
        printf("ERROR opening file: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        while (1)
            ;
    }

    //printf("Opened file 'weights_2.bin'\n");

    total_bytes = 0;
    buffer_size = sizeof(kernel_buffer);
    //printf("Size of kernel buffer: %d\n", buffer_size);
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

    //printf("%d total bytes read\n", total_bytes);

    if ((err = f_close(&file)) != FR_OK) {
        printf("ERROR closing file: %s\n", FF_ERRORS[err]);
        f_mount(NULL, "", 0);
        while (1)
            ;
    }

    //printf("File Closed\n");

    return CNN_OK;
}
