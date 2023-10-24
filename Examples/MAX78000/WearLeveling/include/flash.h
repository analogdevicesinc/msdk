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
 * @file    flash.h
 * @brief   Flash read/write/erase functions declaration
 */

#ifndef EXAMPLES_MAX78000_WEARLEVELING_INCLUDE_FLASH_H_
#define EXAMPLES_MAX78000_WEARLEVELING_INCLUDE_FLASH_H_

// Flash operations log
//#define FLASH_DEBUG
#ifdef FLASH_DEBUG
#define LOGF(...) printf(__VA_ARGS__)
#else
#define LOGF(...)
#endif

#include <stdint.h>
#include <stdbool.h>

#include "lfs.h"

/**
 * @brief Copies contents of flash into a data buffer.
 *
 * @note LittleFS callback method
 *
 * @param c 		LittleFS config
 * @param block 	Flash memory block number
 * @param off 		Data offset in the block
 * @param buffer 	Buffer to copy flash data into
 * @param size 		Number of bytes to read
 *
 * @return LFS_ERR_OK if successful otherwise and error code.
 */
int flash_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer,
               lfs_size_t size);

/**
 * @brief Writes data to flash memory.
 *
 * @note LittleFS callback method
 *
 * @param c 		LittleFS config
 * @param block 	Flash memory block number
 * @param off		Data offset in the block
 * @param buffer 	Buffer containing data to write to flash.
 * @param size 		Number of bytes to write to flash.
 *
 * @return LFS_ERR_OK if successful otherwise and error code.
 */
int flash_write(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer,
                lfs_size_t size);

/**
 * @brief Erases a flash memory block.
 *
 * @note LittleFS callback method
 *
 * @param c 		LittleFS config
 * @param block 	Number of the flash memory block to erase
 *
 * @return LFS_ERR_OK if successful otherwise and error code.
 */
int flash_erase(const struct lfs_config *c, lfs_block_t block);

/**
 * @brief Performs pending flash operations
 * @note LittleFS callback method. Not supported by Maxim SDK.
 *
 * @param c 	LittleFS config
 *
 * @return LFS_ERR_OK if successful otherwise and error code.
 */
int flash_sync(const struct lfs_config *c);

#endif // EXAMPLES_MAX78000_WEARLEVELING_INCLUDE_FLASH_H_
