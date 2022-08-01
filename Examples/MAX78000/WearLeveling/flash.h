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
 * @file    flash.h
 * @brief   Flash read/write/erase functions declaration
 */

#ifndef FLASH_H_
#define FLASH_H_

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
 * @brief Reads flash memory
 * @note LittleFS callback method
 * @param c LittleFS config
 * @param block Flash memory block number
 * @param off Data offset in the block
 * @param buffer Data buffer
 * @param size Data size
 * @return Error code
 */
int flash_read(const struct lfs_config* c, lfs_block_t block, lfs_off_t off, void* buffer,
               lfs_size_t size);

/**
 * @brief Writes flash memory
 * @note LittleFS callback method
 * @param c LittleFS config
 * @param block Flash memory block number
 * @param off Data offset in the block
 * @param buffer Data buffer
 * @param size Data size
 * @return Error code
 */
int flash_write(const struct lfs_config* c, lfs_block_t block, lfs_off_t off, const void* buffer,
                lfs_size_t size);

/**
 * @brief Erases flash memory block
 * @note LittleFS callback method
 * @param c LittleFS config
 * @param block Flash memory block number
 * @return Error code
 */
int flash_erase(const struct lfs_config* c, lfs_block_t block);

/**
 * @brief Performs pending flash operations
 * @note LittleFS callback method. Not supported by Maxim SDK
 * @param c LittleFS config
 * @return Error code
 */
int flash_sync(const struct lfs_config* c);

/**
 * @brief Verifies data in flash
 * @param address Flash memory address
 * @param length Data size
 * @param data Data buffer
 * @return Error code
 */
int flash_verify(uint32_t address, uint32_t length, uint8_t* data);

/**
 * @brief Compares data in flash with value specified
 * @param startaddr Flash memory address
 * @param length Data size
 * @param data The value to compare to
 * @return Error code
 */
int check_mem(uint32_t startaddr, uint32_t length, uint32_t data);

/**
 * @brief Checks whether flash memory is erased
 * @param startaddr Flash memory address
 * @param length Memory block size
 * @return Error code
 */
int check_erased(uint32_t startaddr, uint32_t length);

/**
 * @brief Writes 32bit data words to flash
 * @param startaddr Flash memory address
 * @param length Data size
 * @param data Data buffer
 * @param verify Whether to verify written data
 * @return Error code
 */
int flash_write4(uint32_t startaddr, uint32_t length, uint32_t* data, bool verify);

#endif /* FLASH_H_ */
