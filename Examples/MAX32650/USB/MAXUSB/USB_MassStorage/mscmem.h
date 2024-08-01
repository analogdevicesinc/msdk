/**
 * @file
 * @brief   Memory routines used by the USB Mass Storage Class example.
 *          See the @ref msc_mem_t structure in msc.h for function details.
 */

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
#ifndef EXAMPLES_MAX32650_USB_USB_MASSSTORAGE_MSCMEM_H_
#define EXAMPLES_MAX32650_USB_USB_MASSSTORAGE_MSCMEM_H_

/* **** Include Files **** */
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

#endif // EXAMPLES_MAX32650_USB_USB_MASSSTORAGE_MSCMEM_H_
