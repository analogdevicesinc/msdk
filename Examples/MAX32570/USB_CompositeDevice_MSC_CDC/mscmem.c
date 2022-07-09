/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
 * 
 * This software is protected by copyright laws of the United States and
 * of foreign countries. This material may also be protected by patent laws
 * and technology transfer regulations of the United States and of foreign
 * countries. This software is furnished under a license agreement and/or a
 * nondisclosure agreement and may only be used or reproduced in accordance
 * with the terms of those agreements. Dissemination of this information to
 * any party or parties not specified in the license agreement and/or
 * nondisclosure agreement is expressly prohibited.
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
 * @details Functions are provided for using the internal RAM of the
 *          device or the external SPI flash memory.  Use the SPIXF_DISK
 *          and RAM_DISK defines to select the desired memory at compile
 *          time.
 */

#include "mscmem.h"
#include <string.h>
#include <stdio.h>
#include "Ext_Flash.h"
#include "spixf.h"

/***** Definitions *****/

#define SPIXF_DISK      1
#define RAM_DISK        0

#define LBA_SIZE                    512         /* Size of "logical blocks" in bytes */
#define LBA_SIZE_SHIFT              9           /* The shift value used to convert between addresses and block numbers */

/***** Global Data *****/

/***** File Scope Variables *****/

static int initialized = 0;
static int running = 0;

#if SPIXF_DISK

#undef EXT_FLASH_BAUD
#define EXT_FLASH_BAUD              5000000     /* SPI clock rate to communicate with the external flash */

#define EXT_FLASH_SECTOR_SIZE            4096        /* Number of bytes in one sector of the external flash */
#define EXT_FLASH_SECTOR_SIZE_SHIFT      12          /* The shift value used to convert between addresses and block numbers */
#define EXT_FLASH_NUM_SECTORS            2048        /* Total number of sectors in the external flash */

#define MXC_SPIXF_WIDTH             Ext_Flash_DataLine_Single      /*Number of data lines*/

#define LBA_PER_SECTOR              (EXT_FLASH_SECTOR_SIZE >> LBA_SIZE_SHIFT)
#define INVALID_SECTOR              EXT_FLASH_NUM_SECTORS    /* Use a sector number past the end of memory to indicate invalid */

/***** File Scope Variables *****/
static uint32_t sectorNum = INVALID_SECTOR;
static uint8_t sector[EXT_FLASH_SECTOR_SIZE];
static int sectorDirty = 0;

/***** Function Prototypes *****/
static uint32_t getSectorNum(uint32_t lba);
static uint32_t getSectorAddr(uint32_t lba);
static uint32_t getSector(uint32_t num);

/******************************************************************************/
static uint32_t getSectorNum(uint32_t lba)
{
    /* Absolute_address = lba * LBA_SIZE                    */
    /* Sector_num = Absolute_address / EXT_FLASH_SECTOR_SIZE     */
    /* Sector_num = lba * 512 / 4096                        */
    return lba >> (EXT_FLASH_SECTOR_SIZE_SHIFT - LBA_SIZE_SHIFT);
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
                Ext_Flash_Erase(sectorNum << EXT_FLASH_SECTOR_SIZE_SHIFT, Ext_Flash_Erase_4K);
                /* Write the new */
                Ext_Flash_Program_Page(sectorNum << EXT_FLASH_SECTOR_SIZE_SHIFT, sector, EXT_FLASH_SECTOR_SIZE, MXC_SPIXF_WIDTH);
                /* Mark data as clean */
                sectorDirty = 0;
            }
        }
        
        /* Requesting a new valid sector? */
        if (num != INVALID_SECTOR) {
            Ext_Flash_Read(num << EXT_FLASH_SECTOR_SIZE_SHIFT, sector, EXT_FLASH_SECTOR_SIZE, MXC_SPIXF_WIDTH);
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
        MXC_SPIXF_SetSPIFrequency(EXT_FLASH_BAUD);
        Ext_Flash_Init();
        Ext_Flash_Reset();
        
        if (MXC_SPIXF_WIDTH == Ext_Flash_DataLine_Quad) {
            Ext_Flash_Quad(1);
        }
        else {
            Ext_Flash_Quad(0);
        }
        
        initialized = 1;
    }
    
    return 0;
}

/******************************************************************************/
uint32_t mscmem_Size(void)
{
    /* Get number of 512 byte chunks the external flash contains. */
    return (EXT_FLASH_SECTOR_SIZE >> LBA_SIZE_SHIFT) * EXT_FLASH_NUM_SECTORS;
}

/******************************************************************************/
int mscmem_Read(uint32_t lba, uint8_t* buffer)
{
    uint32_t addr;
    
    /* Convert to external flash sector number. */
    uint32_t sNum = getSectorNum(lba);
    
    if (getSector(sNum)) {
        /* Failed to write/read from external flash */
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
    
    /* Convert to external flash sector number. */
    uint32_t sNum = getSectorNum(lba);
    
    if (getSector(sNum)) {
        /* Failed to write/read from external flash */
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
    /* Turn on the external flash if it is not already. */
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
