/*******************************************************************************
 * Copyright (C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
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
 * Description: Communications Device Class ACM (Serial Port) over USB
 * $Id: descriptors.h 31172 2017-10-05 19:05:57Z zach.metzinger $
 *
 *******************************************************************************
 */

/**
 * @file    mscmem.h
 * @brief   Memory routines used by the USB Mass Storage Class example.
 *          See the msc_mem_t structure in msc.h for function details.
 * @details Functions are provided for using the internal RAM of the
 *          device or the external SPI flash memory.  Use the SPIXF_DISK
 *          and RAM_DISK defines to select the desired memory at compile
 *          time.
 */

#include "mscmem.h"
#include <string.h>
#include <stdio.h>
//#include "mx25.h"
//#include "spixf.h"

/***** Definitions *****/

#define SPIXF_DISK      0
#define RAM_DISK        1

#define LBA_SIZE                    512         /* Size of "logical blocks" in bytes */
#define LBA_SIZE_SHIFT              9           /* The shift value used to convert between addresses and block numbers */

/***** Global Data *****/

/***** File Scope Variables *****/

static int initialized = 0;
static int running = 0;

#if SPIXF_DISK

#define MX25_BAUD                   5000000     /* SPI clock rate to communicate with the MX25 */

#define MX25_SECTOR_SIZE            4096        /* Number of bytes in one sector of the MX25 */
#define MX25_SECTOR_SIZE_SHIFT      12          /* The shift value used to convert between addresses and block numbers */
#define MX25_NUM_SECTORS            2048        /* Total number of sectors in the MX25 */

#define MXC_SPIXF_WIDTH             MXC_SPIXF_WIDTH_1      /*Number of data lines*/

#define LBA_PER_SECTOR              (MX25_SECTOR_SIZE >> LBA_SIZE_SHIFT)
#define INVALID_SECTOR              MX25_NUM_SECTORS    /* Use a sector number past the end of memory to indicate invalid */

/***** File Scope Variables *****/
static uint32_t sectorNum = INVALID_SECTOR;
static uint8_t sector[MX25_SECTOR_SIZE];
static int sectorDirty = 0;

/***** Function Prototypes *****/
static uint32_t getSectorNum(uint32_t lba);
static uint32_t getSectorAddr(uint32_t lba);
static uint32_t getSector(uint32_t num);

/******************************************************************************/
static uint32_t getSectorNum(uint32_t lba)
{
    /* Absolute_address = lba * LBA_SIZE                    */
    /* Sector_num = Absolute_address / MX25_SECTOR_SIZE     */
    /* Sector_num = lba * 512 / 4096                        */
    return lba >> (MX25_SECTOR_SIZE_SHIFT - LBA_SIZE_SHIFT);
}

/******************************************************************************/
static uint32_t getSectorAddr(uint32_t lba)
{
    /* eight 512 byte blocks in each sector */
    return (lba & (LBA_PER_SECTOR - 1)) << LBA_SIZE_SHIFT;
}

/******************************************************************************/
static uint32_t getSector(uint32_t num)
{
    /* New sector requested? */
    if (sectorNum != num) {
        /* Is the current sector real? */
        if (sectorNum != INVALID_SECTOR) {
            /* Was it written to after it was read from memory? */
            if (sectorDirty) {
                /* Erase the old data. */
                MX25_Erase(sectorNum << MX25_SECTOR_SIZE_SHIFT, MX25_Erase_4K);
                /* Write the new */
                MX25_Program_Page(sectorNum << MX25_SECTOR_SIZE_SHIFT, sector, MX25_SECTOR_SIZE, MXC_SPIXF_WIDTH);
                /* Mark data as clean */
                sectorDirty = 0;
            }
        }
        
        /* Requesting a new valid sector? */
        if (num != INVALID_SECTOR) {
            MX25_Read(num << MX25_SECTOR_SIZE_SHIFT, sector, MX25_SECTOR_SIZE, MXC_SPIXF_WIDTH);
            sectorDirty = 0;
            sectorNum = num;
        }
    }
    
    return 0;
}

/******************************************************************************/
int mscmem_Init()
{
    if (!initialized) {
        MXC_SPIXF_SetSPIFrequency(MX25_BAUD);
        MX25_Init();
        MX25_Reset();
        
        if (MXC_SPIXF_WIDTH == MXC_SPIXF_WIDTH_4) {
            MX25_Quad(1);
        }
        else {
            MX25_Quad(0);
        }
        
        initialized = 1;
    }
    
    return 0;
}

/******************************************************************************/
uint32_t mscmem_Size(void)
{
    /* Get number of 512 byte chunks the MX25 contains. */
    return (MX25_SECTOR_SIZE >> LBA_SIZE_SHIFT) * MX25_NUM_SECTORS;
}

/******************************************************************************/
int mscmem_Read(uint32_t lba, uint8_t* buffer)
{
    uint32_t addr;
    
    /* Convert to MX25 sector number. */
    uint32_t sNum = getSectorNum(lba);
    
    if (getSector(sNum)) {
        /* Failed to write/read from MX25 */
        return 1;
    }
    
    /* Get the offset into the current sector */
    addr = getSectorAddr(lba);
    
    memcpy(buffer, sector + addr, LBA_SIZE);
    
    return 0;
}

/******************************************************************************/
int mscmem_Write(uint32_t lba, uint8_t* buffer)
{
    uint32_t addr;
    
    /* Convert to MX25 sector number. */
    uint32_t sNum = getSectorNum(lba);
    
    if (getSector(sNum)) {
        /* Failed to write/read from MX25 */
        return 1;
    }
    
    /* Get the offset into the current sector */
    addr = getSectorAddr(lba);
    
    memcpy(sector + addr, buffer, LBA_SIZE);
    sectorDirty = 1;
    
    return 0;
}

/******************************************************************************/
int mscmem_Start()
{
    /* Turn on the MX25 if it is not already. */
    if (!initialized) {
        mscmem_Init();
    }
    
    /* Check if the initialization succeeded. If it has, start running. */
    if (initialized) {
        running = 1;
    }
    
    /* Start should return fail (non-zero) if the memory cannot be initialized. */
    return !initialized;
}

/******************************************************************************/
int mscmem_Stop()
{
    /* TODO - could shut down XIPF interface here. */
    
    /* Flush the currently cached sector if necessary. */
    if (getSector(INVALID_SECTOR)) {
        return 1;
    }
    
    running = 0;
    return 0;
}

/******************************************************************************/
int mscmem_Ready()
{
    return running;
}

#elif RAM_DISK

#define NUM_PAGES               0x100
static uint8_t mem[NUM_PAGES][LBA_SIZE];

/******************************************************************************/
int mscmem_Init()
{
    if (!initialized) {
        initialized = 1;
#if (ERASE_MEMORY_ON_INIT)
        memset(mem, 0, sizeof(mem));
#endif
    }
    
    return 0;
}

/******************************************************************************/
uint32_t mscmem_Size(void)
{
    return NUM_PAGES;
}

/******************************************************************************/
int mscmem_Read(uint32_t lba, uint8_t* buffer)
{
    if (lba >= NUM_PAGES) {
        return 1;
    }
    
    memcpy(buffer, mem[lba], LBA_SIZE);
    return 0;
}

/******************************************************************************/
int mscmem_Write(uint32_t lba, uint8_t* buffer)
{
    if (lba >= NUM_PAGES) {
        return 1;
    }
    
    memcpy(mem[lba], buffer, LBA_SIZE);
    return 0;
}

/******************************************************************************/
int mscmem_Start()
{
    /* Not much to do for this implementation.  The RAM is always ready. */
    if (!initialized) {
        mscmem_Init();
    }
    
    /* Check if the RAM has been initialized. If it has, start running. */
    if (initialized) {
        running = 1;
    }
    
    /* Start should return fail (non-zero) if the memory cannot be initialized. */
    return !initialized;
}

/******************************************************************************/
int mscmem_Stop()
{
    /* Nothing to do for this implementation.  All data is written as it is */
    /*   received so there are no pending writes that need to be flushed.   */
    running = 0;
    return 0;
}

/******************************************************************************/
int mscmem_Ready()
{
    return running;
}

#else
#error "You must assign either RAM_DISK or SPIXF_DISK to 1."
#endif
