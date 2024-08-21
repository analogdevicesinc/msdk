/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

/**
 * @file    mscmem.h
 * @brief   Memory routines used by the USB Mass Storage Class example.
 *          See the msc_mem_t structure in msc.h for function details.
 */

#ifndef EXAMPLES_MAX32665_USB_USB_COMPOSITEDEVICE_MSC_HID_MSCMEM_H_
#define EXAMPLES_MAX32665_USB_USB_COMPOSITEDEVICE_MSC_HID_MSCMEM_H_

#include <stdint.h>

#define ERASE_MEMORY_ON_INIT \
    1 /* Configuration option to clear the memory (to 0s) on initialization. */
/* Use 1 to clear or 0 to leave untouched. */

int mscmem_Init(void);
int mscmem_Start(void);
int mscmem_Stop(void);
int mscmem_Ready();
uint32_t mscmem_Size(void);
int mscmem_Read(uint32_t lba, uint8_t *buffer);
int mscmem_Write(uint32_t lba, uint8_t *buffer);

#endif // EXAMPLES_MAX32665_USB_USB_COMPOSITEDEVICE_MSC_HID_MSCMEM_H_
