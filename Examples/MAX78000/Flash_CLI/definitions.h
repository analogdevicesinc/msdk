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

#ifndef EXAMPLES_MAX78000_FLASH_CLI_DEFINITIONS_H_
#define EXAMPLES_MAX78000_FLASH_CLI_DEFINITIONS_H_

#include "flc.h"

#define VERBOSE_LOGGING

#ifdef VERBOSE_LOGGING
#define LOGV(...) printf(__VA_ARGS__)
#else
#define LOGV(...)
#endif

#define FLASH_STORAGE_PAGE_NO                   \
    (MXC_FLASH_MEM_SIZE / MXC_FLASH_PAGE_SIZE - \
     1) ///< Internal storage flash memory page (the last page)
#define FLASH_STORAGE_START_ADDR \
    MXC_FLASH_PAGE_ADDR(FLASH_STORAGE_PAGE_NO) ///< Internal storage start address

int flash_write(uint32_t startaddr, uint32_t length, uint32_t *data);
int flash_read(uint32_t startaddr, uint32_t length, uint8_t *data);
int check_erased(uint32_t startaddr, uint32_t length);
uint32_t calculate_crc(uint32_t *array, uint32_t length);
#endif // EXAMPLES_MAX78000_FLASH_CLI_DEFINITIONS_H_
