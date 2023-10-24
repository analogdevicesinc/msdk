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

#include "file.h"
#include "lfs.h"
#include "mxc_errors.h"

int file_write(lfs_t *filesys, lfs_file_t *file, const char *filename, char *write_buf,
               uint32_t len, uint32_t pos, bool create)
{
    int err;
    int lfs_open_flags = LFS_O_WRONLY;

    // Check for bad params
    if (filesys == NULL || write_buf == NULL || filename == NULL) {
        return E_NULL_PTR;
    }

    // Set appropriate LFS flags
    if (create) {
        lfs_open_flags |= LFS_O_CREAT;
    }

    // Open up the file to write to
    err = lfs_file_open(filesys, file, filename, lfs_open_flags);
    if (err != LFS_ERR_OK) {
        printf("Unable to open file.\n");
        return err;
    }

    // Set write position within the file
    err = lfs_file_seek(filesys, file, pos, LFS_SEEK_SET);
    if (err < LFS_ERR_OK || err != pos) {
        printf("Unable to set write pointer to specified position.\n");
        lfs_file_close(filesys, file);
        return err;
    }

    // Write data to file
    err = lfs_file_write(filesys, file, write_buf, len);

    // Close the file. File not written to storage unless the file is explicitly closed.
    lfs_file_close(filesys, file);

    return err;
}

int file_read(lfs_t *filesys, lfs_file_t *file, const char *filename, char *read_buf, uint32_t len,
              uint32_t pos)
{
    int err;

    // Check for bad params
    if (filesys == NULL || read_buf == NULL || filename == NULL) {
        return E_NULL_PTR;
    }

    // Open up the file to read from
    err = lfs_file_open(filesys, file, filename, LFS_O_RDONLY);
    if (err != LFS_ERR_OK) {
        printf("Unable to open file.\n");
        return err;
    }

    // Set read position within the file
    err = lfs_file_seek(filesys, file, pos, LFS_SEEK_SET);
    if (err < LFS_ERR_OK || err != pos) {
        printf("Unable to set read pointer to specified position.\n");
        lfs_file_close(filesys, file);
        return err;
    }

    // Read from file
    err = lfs_file_read(filesys, file, (void *)read_buf, len);

    // Close the file.
    lfs_file_close(filesys, file);

    return err;
}
