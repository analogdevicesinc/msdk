/*******************************************************************************
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
*******************************************************************************
*/

#ifndef EXAMPLES_MAX78000_EEPROM_EMULATOR_CACHE_H_
#define EXAMPLES_MAX78000_EEPROM_EMULATOR_CACHE_H_

/***** Included Files *****/
#include "mxc_device.h"

/***** Definitions *****/
#define CACHE_IDX(flash_addr) (flash_addr % MXC_FLASH_PAGE_SIZE)

/***** Type Definitions *****/
typedef struct {
    uint8_t cache[MXC_FLASH_PAGE_SIZE];
    uint32_t start_addr;
    uint32_t end_addr;
} cache_t;

/***** Functions *****/
/*
 * @brief Initialize the cache
 *
 * @param cache 	Pointer to cache structure.
 * @param init_addr Address in the flash page to initialize the cache with.
 *
 * @return Success/fail. See \ref MXC_Error_Codes for list of error codes.
 */
int cache_init(cache_t *cache, uint32_t init_addr);

/*
 * @brief Store data currently in cache to flash and load the next flash page into the cache.
 *
 * @param cache 	Pointer to cache structure.
 * @param next_addr Address in the next flash page to load into cache.
 *
 * @return Success/fail. See \ref MXC_Error_Codes for list of error codes.
 */
int cache_refresh(cache_t *cache, uint32_t next_addr);

#endif // EXAMPLES_MAX78000_EEPROM_EMULATOR_CACHE_H_
