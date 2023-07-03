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
 * @file    mscmem.h
 * @brief   Memory routines used by the USB Mass Storage Class example.
 *          See the msc_mem_t structure in msc.h for function details.
 */

#ifndef EXAMPLES_MAX32665_USB_COMPOSITEDEVICE_MSC_CDC_MSCMEM_H_
#define EXAMPLES_MAX32665_USB_COMPOSITEDEVICE_MSC_CDC_MSCMEM_H_

#include <stdint.h>

/* **** Definitions **** */
#define ERASE_MEMORY_ON_INIT \
    1 /* Configuration option to clear the memory (to 0s) on initialization. */
/* Use 1 to clear or 0 to leave untouched. */
/**
 * @brief   Perform any initialization necessary to prepare the memory for reading/writing data.
 * @returns 0 if initialization is successful, non-zero if an error occurred.
 */
int mscmem_Init(void);

/**
 * @brief   Activates the memory.
 * @returns 0 if activation is successful, non-zero if an error occurred.
 */
int mscmem_Start(void);

/**
 * @brief   Deactivates the memory.
 * @returns 0 if activation is successful, non-zero if an error occurred.
 */
int mscmem_Stop(void);

/**
 * @brief   Reports the total size of the mass-storage memory.
 * @returns The number of 512 byte blocks contained in the memory.
 */
uint32_t mscmem_Size(void);

/**
 * @brief   Reads 512 bytes of data from the memory.
 * @param   lba     The index of the 512 byte block to read.
 * @param   buffer  A byte array of at least 512 bytes to hold the values read.
 * @returns 0 if reading is successful, non-zero if an error occurred.
 * @returns
 */
int mscmem_Read(uint32_t lba, uint8_t *buffer);

/**
 * @brief   Writes 512 bytes of data to the memory.
 * @param   lba     The index of the 512 byte block to write.
 * @param   buffer  A byte array of at least 512 bytes holding the values to write.
 * @returns 0 if writing is successful, non-zero if an error occurred.
 * @returns
 */
int mscmem_Write(uint32_t lba, uint8_t *buffer);

/**
 * @brief   Checks if the memory is ready to be read/written.
 * @returns non-zero if the memory is ready, 0 otherwise.
 */
int mscmem_Ready(void);

#endif // EXAMPLES_MAX32665_USB_COMPOSITEDEVICE_MSC_CDC_MSCMEM_H_
