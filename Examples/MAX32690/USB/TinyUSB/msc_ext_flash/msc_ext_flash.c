/******************************************************************************
 *
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

#include "bsp/board_api.h"
#include "tusb.h"

#include "Ext_Flash.h"
#include "spixf.h"

#define LBA_SIZE 512 /* Size of "logical blocks" in bytes */
#define LBA_SIZE_SHIFT 9 /* The shift value used to convert between addresses and block numbers */

#if defined(EXT_FLASH_MX25)
#undef EXT_FLASH_BAUD
#define EXT_FLASH_BAUD 5000000 /* SPI clock rate to communicate with the external flash */

#define EXT_FLASH_SECTOR_SIZE 4096 /* Number of bytes in one sector of the external flash */
#define EXT_FLASH_SECTOR_SIZE_SHIFT \
    12 /* The shift value used to convert between addresses and block numbers */
#define EXT_FLASH_NUM_SECTORS 2048 /* Total number of sectors in the external flash */

#define MXC_SPIXF_WIDTH Ext_Flash_DataLine_Quad /*Number of data lines*/
#else
#error "Defined flash device not yet implemented!"
#endif

#define LBA_PER_SECTOR (EXT_FLASH_SECTOR_SIZE >> LBA_SIZE_SHIFT)
#define INVALID_SECTOR \
    EXT_FLASH_NUM_SECTORS /* Use a sector number past the end of memory to indicate invalid */

#define NUM_LBA (EXT_FLASH_NUM_SECTORS * LBA_PER_SECTOR)

static bool is_running = false;
static bool is_initialized = false;
static uint32_t active_sector_num = INVALID_SECTOR;
static uint8_t active_sector[EXT_FLASH_SECTOR_SIZE];
static bool active_sector_dirty = false;

void spixf_disk_init(void);
static uint32_t spixf_disk_get_sect_num(uint32_t lba);
static uint32_t spixf_disk_get_sect_addr(uint32_t lba);
static uint32_t spixf_disk_get_sect(uint32_t num);

// Invoked to determine max LUN
uint8_t tud_msc_get_maxlun_cb(void)
{
    return 1; // single LUN
}

// Invoked when received SCSI_CMD_INQUIRY
// Application fill vendor id, product id and revision with string up to 8, 16, 4 characters respectively
void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16],
                        uint8_t product_rev[4])
{
    (void)lun; // use same ID for both LUNs

    const char vid[] = "TinyUSB";
    const char pid[] = "Mass Storage";
    const char rev[] = "1.0";

    memcpy(vendor_id, vid, strlen(vid));
    memcpy(product_id, pid, strlen(pid));
    memcpy(product_rev, rev, strlen(rev));
}

// Invoked when received Test Unit Ready command.
// return true allowing host to read/write this LUN e.g SD card inserted
bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
    return is_initialized;
}

// Invoked when received SCSI_CMD_READ_CAPACITY_10 and SCSI_CMD_READ_FORMAT_CAPACITY to determine the disk size
// Application update block count and block size
void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size)
{
    (void)lun;

    *block_count = NUM_LBA;
    *block_size = LBA_SIZE;
}

// Invoked when received Start Stop Unit command
// - Start = 0 : stopped power mode, if load_eject = 1 : unload disk storage
// - Start = 1 : active mode, if load_eject = 1 : load disk storage
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject)
{
    (void)lun;
    (void)power_condition;

    if (load_eject) {
        if (start) {
            // load disk storage
            is_running = true;
        } else {
            // unload disk storage
            //Sync dirty sector if any
            spixf_disk_get_sect(INVALID_SECTOR);
            is_running = false;
        }
    }

    return is_initialized;
}

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and return number of copied bytes.
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer,
                          uint32_t bufsize)
{
    uint32_t addr;
    uint32_t s_num;

    if (lba >= NUM_LBA) {
        return -1;
    }

    /* Convert to external flash sector number. */
    s_num = spixf_disk_get_sect_num(lba);

    if (spixf_disk_get_sect(s_num)) {
        /* Failed to write/read from external flash */
        return -1;
    }

    /* Get the offset into the current sector */
    addr = spixf_disk_get_sect_addr(lba);

    memcpy(buffer, active_sector + addr + offset, bufsize);

    return (int32_t)bufsize;
}

bool tud_msc_is_writable_cb(uint8_t lun)
{
    (void)lun;

    return true;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and return number of written bytes
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer,
                           uint32_t bufsize)
{
    uint32_t addr;
    uint32_t s_num;

    // out of ramdisk
    if (lba >= NUM_LBA) {
        return -1;
    }

    /* Convert to external flash sector number. */
    s_num = spixf_disk_get_sect_num(lba);

    if (spixf_disk_get_sect(s_num)) {
        /* Failed to write/read from external flash */
        return -1;
    }

    /* Get the offset into the current sector */
    addr = spixf_disk_get_sect_addr(lba);

    memcpy(active_sector + addr + offset, buffer, bufsize);
    active_sector_dirty = 1;

    return (int32_t)bufsize;
}

// Callback invoked when received an SCSI command not in built-in list below
// - READ_CAPACITY10, READ_FORMAT_CAPACITY, INQUIRY, MODE_SENSE6, REQUEST_SENSE
// - READ10 and WRITE10 has their own callbacks (MUST not be handled here)
int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void *buffer, uint16_t bufsize)
{
    void const *response = NULL;
    int32_t resplen = 0;

    // most scsi handled is input
    bool in_xfer = true;

    switch (scsi_cmd[0]) {
    default:
        // Set Sense = Invalid Command Operation
        tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);

        // negative means error -> tinyusb could stall and/or response with failed status
        return -1;
    }

    // return resplen must not larger than bufsize
    if (resplen > bufsize)
        resplen = bufsize;

    if (response && (resplen > 0)) {
        if (in_xfer) {
            memcpy(buffer, response, (size_t)resplen);
        } else {
            // SCSI output
        }
    }

    return resplen;
}

void spixf_disk_init()
{
    
    if (!is_initialized) {
        Ext_Flash_Init();
        Ext_Flash_Reset();

        if (MXC_SPIXF_WIDTH == Ext_Flash_DataLine_Quad) {
            Ext_Flash_Quad(1);
        } else {
            Ext_Flash_Quad(0);
        }

        is_initialized = true;
    }
}

static uint32_t spixf_disk_get_sect_num(uint32_t lba)
{
    /* Absolute_address = lba * LBA_SIZE                    */
    /* Sector_num = Absolute_address / EXT_FLASH_SECTOR_SIZE     */
    /* Sector_num = lba * 512 / 4096                        */
    return lba >> (EXT_FLASH_SECTOR_SIZE_SHIFT - LBA_SIZE_SHIFT);
}

static uint32_t spixf_disk_get_sect_addr(uint32_t lba)
{
    /* eight 512 byte blocks in each sector */
    return (lba & (LBA_PER_SECTOR - 1)) << LBA_SIZE_SHIFT;
}

static uint32_t spixf_disk_get_sect(uint32_t num)
{
    /* New sector requested? */
    if (active_sector_num != num) {
        /* Is the current sector real? */
        if (active_sector_num != INVALID_SECTOR) {
            /* Was it written to after it was read from memory? */
            if (active_sector_dirty) {
                /* Erase the old data. */
                Ext_Flash_Erase(active_sector_num << EXT_FLASH_SECTOR_SIZE_SHIFT,
                                Ext_Flash_Erase_4K);
                /* Write the new */
                Ext_Flash_Program_Page(active_sector_num << EXT_FLASH_SECTOR_SIZE_SHIFT,
                                       active_sector, EXT_FLASH_SECTOR_SIZE, MXC_SPIXF_WIDTH);
                /* Mark data as clean */
                active_sector_dirty = false;
            }
        }

        /* Requesting a new valid sector? */
        if (num != INVALID_SECTOR) {
            Ext_Flash_Read(num << EXT_FLASH_SECTOR_SIZE_SHIFT, active_sector, EXT_FLASH_SECTOR_SIZE,
                           MXC_SPIXF_WIDTH);
            active_sector_dirty = false;
            active_sector_num = num;
        }
    }

    return 0;
}
