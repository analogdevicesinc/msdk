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

#ifndef EXAMPLES_MAX32670_FLASH_CLI_DEFINITIONS_H_
#define EXAMPLES_MAX32670_FLASH_CLI_DEFINITIONS_H_

/***** Definitions *****/
// Internal storage flash memory page (the last page)
#define FLASH_STORAGE_PAGE_NO (MXC_FLASH_MEM_SIZE / MXC_FLASH_PAGE_SIZE - 1)

// Internal storage start address
#define FLASH_STORAGE_START_ADDR MXC_FLASH_PAGE_ADDR(FLASH_STORAGE_PAGE_NO)

#define POLY 0xEDB88320 // CRC Polynomial

// CLI command table
// clang-format off
#define CMD_TABLE { { "write",                                                                             \
                      "write <word offset> <text>",                                                        \
                      "Writes text string to flash starting at the 32-bit word in the\n"                   \
                      "    flash page specified by \"word offset\" (e.g. word offset=3 -> address\n"       \
                      "    offset=0xC, word offset=4 -> address offset=0x10)",                             \
                      handle_write },                                                                      \
                    { "erase",                                                                             \
                      "erase",                                                                             \
                      "Erases page in flash being operated on",                                            \
                      handle_erase },                                                                      \
                    { "read",                                                                              \
                      "read <word offset> <number of letters>",                                            \
                      "Reads text from flash starting at the 32-bit word in the flash\n"                   \
                      "    page specified by \"word offset\" (e.g. word offset=3 -> address offset=0xC,\n" \
                      "    word offset=4 -> address offset=0x10)",                                         \
                      handle_read },                                                                       \
                    { "crc",                                                                               \
                      "crc",                                                                               \
                      "Calculates CRC of entire flash page",                                               \
                      handle_crc } }
// clang-format on

#define WORD_OFFSET_POS 1
#define DATA_POS 2
#define LENGTH_POS 2

/***** Function Prototypes *****/
// Command handler functions
int handle_write(int argc, char *argv[]);
int handle_read(int argc, char *argv[]);
int handle_erase(int argc, char *argv[]);
int handle_crc(int argc, char *argv[]);

// Command helper functions
int flash_verify(uint32_t address, uint32_t length, uint32_t *data);
int check_mem(uint32_t startaddr, uint32_t length, uint32_t data);
int check_erased(uint32_t startaddr, uint32_t length);

#endif // EXAMPLES_MAX32670_FLASH_CLI_DEFINITIONS_H_
