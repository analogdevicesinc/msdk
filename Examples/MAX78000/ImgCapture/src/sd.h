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
* @file sd.h
* @brief This file exposes some higher-level SD card interface functions that
    wrap around the low-level operations provided by the fatFS file system 
    (http://elm-chan.org/fsw/ff/00index_e.html).
*****************************************************************************/

#ifndef EXAMPLES_MAX78000_IMGCAPTURE_SRC_SD_H_
#define EXAMPLES_MAX78000_IMGCAPTURE_SRC_SD_H_

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "ff.h"

// Some global working variables and buffers are exposed below.  These hook up to
// the sd.c internals, and present convenient access points and buffers for more
// complex fatFS operations.  Most fatFS operations expect pointers to a global
// file system struct, file struct, directory struct, etc.
#define MAXLEN 256

extern FATFS *sd_fs; //FFat Filesystem Object
extern FIL sd_file; //FFat File Object
extern FRESULT sd_err; //FFat Result (Struct)
extern FILINFO sd_fno; //FFat File Information Object
extern DIR sd_dir; //FFat Directory Object
extern TCHAR sd_message[MAXLEN], sd_directory[MAXLEN], sd_cwd[MAXLEN], sd_filename[MAXLEN],
    sd_volume_label[24], sd_volume;
extern DWORD sd_clusters_free, sd_sectors_free, sd_sectors_total, sd_volume_sn;
extern UINT sd_bytes_written, sd_bytes_read, sd_mounted;
extern BYTE sd_work[4096];
extern TCHAR *FR_ERRORS[20];

/**
* @brief Mount the SD card.  If the SD card is blank (no volume name), format the card
* with FAT32 and give it the name "MAXIM-SD"
* @return FR_OK if successful, FR_xxx error code if unsucessful.
* @details
****************************************************************************/
FRESULT sd_mount();

/**
* @brief Unmount the SD card.
* @return FR_OK if successful, FR_xxx error code if unsucessful.
****************************************************************************/
FRESULT sd_unmount();

/**
* @brief Get the size and free space available on the SD card.  Sets them to the 
global "sd_sectors_total" and "sd_sectors_free" variables, respectively.
* @return FR_OK if successful, FR_xxx error code if unsucessful.
****************************************************************************/
FRESULT sd_get_size();

/**
* @brief Get the current working directory and saves it to the "sd_cwd" global variable.
* @return FR_OK if successful, FR_xxx error code if unsucessful.
****************************************************************************/
FRESULT sd_get_cwd();

/**
* @brief Change directory.
* @param[in] dir Target directory.
* @return FR_OK if successful, FR_xxx error code if unsucessful.
****************************************************************************/
FRESULT sd_cd(const char *dir);

/**
* @brief List the contents of the current directory with printf.
* @return FR_OK if successful, FR_xxx error code if unsucessful.
****************************************************************************/
FRESULT sd_ls();

/**
* @brief Make a directory.  Similar to "mkdir" on linux.
* @param[in] dir Directory path.
* @return FR_OK if successful, FR_xxx error code if unsucessful.
****************************************************************************/
FRESULT sd_mkdir(const char *dir);

/**
* @brief Remove a file or empty directory.  Similar to "rm" on linux.
* @param[in] item Item to remove.
* @return FR_OK if successful, FR_xxx error code if unsucessful.
****************************************************************************/
FRESULT sd_rm(const char *item);

/**
* @brief Create an empty file.  Similar to the "touch" command on linux.
* @param[in] filepath Target file path (must not already exist).
* @return FR_OK if successful, FR_xxx error code if unsucessful.
****************************************************************************/
FRESULT sd_touch(const char *filepath);

/**
* @brief Write a string to a file.
* @param[in] filepath Target file path (must already exist).
* @param[in] strng String to write to the file.  Must be null terminated '\0'
* @return FR_OK if successful, FR_xxx error code if unsucessful.
****************************************************************************/
FRESULT sd_write_string(const char *filepath, const char *strng);

/**
* @brief Write bytes to a file.
* @param[in] filepath Target file path (must already exist).
* @param[in] data Bytes to write to the file.
* @param[in] len Number of bytes to write.
* @return FR_OK if successful, FR_xxx error code if unsucessful.
****************************************************************************/
FRESULT sd_write(const char *filepath, const uint8_t *data, int len);
/**
* @brief Print the contents of a file.  Similar to the "cat" command on linux.
* @param[in] filename Directory path.
* @return FR_OK if successful, FR_xxx error code if unsucessful.
****************************************************************************/
FRESULT sd_cat(const char *filename);

// Supporting function for use with f_forward (http://elm-chan.org/fsw/ff/doc/forward.html)
// The actual implementation of this function is in console.c, which
// streams to the serial console.
UINT out_stream(const BYTE *p, UINT btf);

#endif // EXAMPLES_MAX78000_IMGCAPTURE_SRC_SD_H_
