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

#ifndef EXAMPLES_MAX32650_WEARLEVELING_INCLUDE_FILE_H_
#define EXAMPLES_MAX32650_WEARLEVELING_INCLUDE_FILE_H_

#include <stdbool.h>
#include <stdint.h>
#include "lfs.h"

/***** Macros *****/
#define MAX_FILE_READ_SIZE 1024

/*
 * @brief Write data to a file.
 *
 * @param filesys 		Pointer to the LittleFS file system instance.
 * @param file 			Pointer to a LittleFS file instance.
 * @param filename 	 	Name of the file to write to.
 * @param write_buf 	Buffer containing the data to write to the file
 * @param len			Number of bytes to write to the file.
 * @param pos			Position within the file to start writing data at.
 * @param create 		Determines behavior if file doesn't already exist, "true" will create the
 * 						file and complete the write, and "false" will return an error.
 *
 * @return The number of bytes written to flash if successful, otherwise an error code.
 */
int file_write(lfs_t *filesys, lfs_file_t *file, const char *filename, char *write_buf,
               uint32_t len, uint32_t pos, bool create);

/*
 * @brief Read data from a file.
 *
 * @param filesys 		Pointer to the LittleFS file system instance.
 * @param file 			Pointer to a LittleFS file instance.
 * @param filename 		Name of the file to read from.
 * @param read_buf		Buffer to store data from the file.
 * @param len 			Number of bytes to read from the file.
 * @param pos			Position within the file to start reading data from.
 *
 * @return The number of bytes read if successful, otherwise an error code.
 */
int file_read(lfs_t *filesys, lfs_file_t *file, const char *filename, char *read_buf, uint32_t len,
              uint32_t pos);

#endif // EXAMPLES_MAX32650_WEARLEVELING_INCLUDE_FILE_H_
