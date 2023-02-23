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

#include "cache.h"

#include <stdio.h>
#include <string.h>
#include "flc.h"

int cache_init(cache_t *cache, uint32_t init_addr)
{
    int err;

    if (cache == NULL) {
        return E_NULL_PTR;
    } else if (init_addr < MXC_FLASH0_MEM_BASE ||
               init_addr > (MXC_FLASH0_MEM_BASE + MXC_FLASH0_MEM_SIZE)) {
        return E_BAD_PARAM;
    }

    // Configure FLC
    err = MXC_FLC_Init();
    if (err != E_NO_ERROR) {
        printf("Failed to initialize flash controller.\n");
        return err;
    }

    // Get starting address of flash page
    init_addr -= init_addr % MXC_FLASH0_PAGE_SIZE;

    // Initialize cache values and starting address
    memcpy(cache->cache, (void *)init_addr, MXC_FLASH0_PAGE_SIZE);
    cache->start_addr = init_addr;
    cache->end_addr = init_addr + MXC_FLASH0_PAGE_SIZE;
    cache->dirty = false;

    return E_NO_ERROR;
}

int cache_refresh(cache_t *cache, uint32_t next_addr)
{
    int err;

    if (cache == NULL) {
        return E_NULL_PTR;
    } else if (next_addr < MXC_FLASH0_MEM_BASE ||
               next_addr >= (MXC_FLASH0_MEM_BASE + MXC_FLASH0_MEM_SIZE)) {
        return E_BAD_PARAM;
    }

    // If cache contents modified, store it back to flash
    if (cache->dirty) {
        // Erase flash page before copying cache contents to it
        err = MXC_FLC_PageErase(cache->start_addr);
        if (err != E_NO_ERROR) {
            return err;
        }

        // Copy contents of cache to erase flash page
        err = MXC_FLC_Write(cache->start_addr, MXC_FLASH0_PAGE_SIZE, (uint32_t *)cache->cache);
        if (err != E_NO_ERROR) {
            return err;
        }
    }

    // Get starting address of flash page
    next_addr &= ~(MXC_FLASH0_PAGE_SIZE - 1);

    // Initialize cache values and starting address
    memcpy(cache->cache, (void *)next_addr, MXC_FLASH0_PAGE_SIZE);
    cache->start_addr = next_addr;
    cache->end_addr = next_addr + MXC_FLASH0_PAGE_SIZE;
    cache->dirty = false;

    return E_NO_ERROR;
}

int cache_write_back(cache_t *cache)
{
    int err;

    // Erase flash page before copying cache contents to it
    err = MXC_FLC_PageErase(cache->start_addr);
    if (err != E_NO_ERROR) {
        return err;
    }

    // Copy contents of cache to erase flash page
    err = MXC_FLC_Write(cache->start_addr, MXC_FLASH0_PAGE_SIZE, (uint32_t *)cache->cache);
    if (err != E_NO_ERROR) {
        return err;
    }

    cache->dirty = false;

    return err;
}
