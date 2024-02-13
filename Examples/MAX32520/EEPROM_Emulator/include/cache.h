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

#ifndef EXAMPLES_MAX32520_EEPROM_EMULATOR_INCLUDE_CACHE_H_
#define EXAMPLES_MAX32520_EEPROM_EMULATOR_INCLUDE_CACHE_H_

/***** Included Files *****/
#include <stdbool.h>
#include "mxc_device.h"

/***** Definitions *****/
#define CACHE_IDX(flash_addr) (flash_addr % MXC_FLASH_PAGE_SIZE)

/***** Type Definitions *****/
typedef struct {
    uint8_t cache[MXC_FLASH_PAGE_SIZE];
    uint32_t start_addr;
    uint32_t end_addr;
    bool dirty;
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

/*
 * @brief Store data currently in cache to flash.
 *
 * @param cache 	Pointer to cache structure.
 *
 * @return Success/fail. See \ref MXC_Error_Codes for list of error codes.
 */
int cache_write_back(cache_t *cache);

#endif // EXAMPLES_MAX32520_EEPROM_EMULATOR_INCLUDE_CACHE_H_
