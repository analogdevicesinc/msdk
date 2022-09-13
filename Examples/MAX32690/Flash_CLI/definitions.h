#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include "flc.h"

#define VERBOSE_LOGGING

#ifdef VERBOSE_LOGGING
#define LOGV(...) printf(__VA_ARGS__)
#else
#define LOGV(...)
#endif

#define FLASH_STORAGE_PAGE_NO                    \
    (MXC_FLASH0_MEM_SIZE / MXC_FLASH_PAGE_SIZE - \
     1) ///< Internal storage flash memory page (the last page)
#define FLASH_STORAGE_START_ADDR \
    MXC_FLASH_PAGE_ADDR(FLASH_STORAGE_PAGE_NO) ///< Internal storage start address

int flash_write(uint32_t startaddr, uint32_t length, uint32_t *data);
int flash_read(uint32_t startaddr, uint32_t length, uint8_t *data);
int check_erased(uint32_t startaddr, uint32_t length);
uint32_t calculate_crc(uint32_t *array, uint32_t length);
#endif // DEFINITIONS_H
