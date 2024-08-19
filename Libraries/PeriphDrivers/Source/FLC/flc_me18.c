/**
 * @file flc_me18.c
 * @brief      Flash Controler driver.
 * @details    This driver can be used to operate on the embedded flash memory.
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

/* **** Includes **** */
#include <string.h>
#include "mxc_errors.h"
#include "mxc_sys.h"
#include "flc.h"
#include "flc_reva.h"
#include "flc_common.h"
#include "icc.h"

/* **** Definitions **** */

/* **** Globals **** */

/* **** Functions **** */

//******************************************************************************
void MXC_FLC_ME18_Flash_Operation(void)
{
    /*
    This function should be called after modifying the contents of flash memory.
    It flushes the instruction caches and line fill buffer.
    
    It should be called _afterwards_ because after flash is modified the cache
    may contain instructions that may no longer be valid.  _Before_ the
    flash modifications the ICC may contain relevant cached instructions related to 
    the incoming flash instructions (especially relevant in the case of external memory),
    and these instructions will be valid up until the point that the modifications are made.
    
    The line fill buffer is a FLC-related buffer that also may no longer be valid.
    It's flushed by reading 2 pages of flash.
    */

    /* Flush all instruction caches */
    // ME18 has bug where top-level sysctrl flush bit only works one.
    // Have to use low-level flush bits for each ICC instance.
    MXC_ICC_Flush(MXC_ICC0);
    MXC_ICC_Flush(MXC_ICC1);

    // Clear the line fill buffer by reading 2 pages from flash
    volatile uint32_t *line_addr;
    volatile uint32_t line;
    line_addr = (uint32_t *)(MXC_FLASH_MEM_BASE);
    line = *line_addr;
    line_addr = (uint32_t *)(MXC_FLASH_MEM_BASE + MXC_FLASH_PAGE_SIZE);
    line = *line_addr;
    (void)line; // Silence build warnings that this variable is not used.
}

//******************************************************************************
int MXC_FLC_ME18_GetByAddress(mxc_flc_regs_t **flc, uint32_t addr)
{
    if ((addr >= MXC_FLASH0_MEM_BASE) && (addr < (MXC_FLASH0_MEM_BASE + MXC_FLASH0_MEM_SIZE))) {
        *flc = MXC_FLC0;
    } else if ((addr >= MXC_FLASH1_MEM_BASE) &&
               (addr < (MXC_FLASH1_MEM_BASE + MXC_FLASH1_MEM_SIZE))) {
        *flc = MXC_FLC1;
    } else if ((addr >= MXC_INFO0_MEM_BASE) && (addr < (MXC_INFO0_MEM_BASE + MXC_INFO_MEM_SIZE))) {
        *flc = MXC_FLC0;
    } else if ((addr >= MXC_INFO1_MEM_BASE) && (addr < (MXC_INFO1_MEM_BASE + MXC_INFO_MEM_SIZE))) {
        *flc = MXC_FLC1;
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

//******************************************************************************
int MXC_FLC_ME18_GetPhysicalAddress(uint32_t addr, uint32_t *result)
{
    if ((addr >= MXC_FLASH0_MEM_BASE) && (addr < (MXC_FLASH0_MEM_BASE + MXC_FLASH0_MEM_SIZE))) {
        *result = addr - MXC_FLASH_MEM_BASE;
    } else if ((addr >= MXC_FLASH1_MEM_BASE) &&
               (addr < (MXC_FLASH1_MEM_BASE + MXC_FLASH1_MEM_SIZE))) {
        *result = addr - MXC_FLASH_MEM_BASE;
    } else if ((addr >= MXC_INFO0_MEM_BASE) && (addr < (MXC_INFO0_MEM_BASE + MXC_INFO_MEM_SIZE))) {
        *result = (addr & (MXC_INFO_MEM_SIZE - 1)) + MXC_FLASH0_MEM_BASE + MXC_FLASH1_MEM_BASE;
    } else if ((addr >= MXC_INFO1_MEM_BASE) && (addr < (MXC_INFO1_MEM_BASE + MXC_INFO_MEM_SIZE))) {
        *result = ((addr - MXC_INFO_MEM_SIZE) & (MXC_INFO_MEM_SIZE - 1)) + MXC_FLASH0_MEM_BASE +
                  MXC_FLASH1_MEM_SIZE;
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

// *****************************************************************************
int MXC_FLC_Init(void)
{
    return E_NO_ERROR;
}

// *****************************************************************************
#if IAR_PRAGMAS
#pragma section = ".flashprog"
#else
__attribute__((section(".flashprog")))
#endif
int MXC_FLC_Busy(void)
{
    return MXC_FLC_RevA_Busy();
}

// *****************************************************************************
int MXC_FLC_MassErase(void)
{
    int err;

    if ((err = MXC_FLC_RevA_MassErase((mxc_flc_reva_regs_t *)MXC_FLC0)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_FLC_RevA_MassErase((mxc_flc_reva_regs_t *)MXC_FLC1)) != E_NO_ERROR) {
        return err;
    }

    // Flush the cache
    MXC_FLC_ME18_Flash_Operation();

    return E_NO_ERROR;
}

// *****************************************************************************
#if IAR_PRAGMAS
#pragma section = ".flashprog"
#else
__attribute__((section(".flashprog")))
#endif
int MXC_FLC_PageErase(uint32_t address)
{
    int err;
    uint32_t physicalAddress;
    mxc_flc_regs_t *flc;

    if ((err = MXC_FLC_ME18_GetPhysicalAddress(address, &physicalAddress)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_FLC_ME18_GetByAddress(&flc, address)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_FLC_RevA_PageErase((mxc_flc_reva_regs_t *)flc, physicalAddress)) != E_NO_ERROR) {
        return err;
    }

    // Flush the cache
    MXC_FLC_ME18_Flash_Operation();

    return E_NO_ERROR;
}

// *****************************************************************************
void MXC_FLC_Read(int address, void *buffer, int len)
{
    if (address < MXC_FLASH_MEM_BASE ||
        address >= (MXC_FLASH_MEM_BASE + MXC_FLASH0_MEM_SIZE + MXC_FLASH1_MEM_SIZE)) {
        return;
    }
    MXC_FLC_Com_Read(address, buffer, len);
}

// *****************************************************************************
int MXC_FLC_Write(uint32_t address, uint32_t length, uint32_t *buffer)
{
    return MXC_FLC_Com_Write(address, length, buffer);
}

// *****************************************************************************
int MXC_FLC_Write32(uint32_t address, uint32_t data)
{
    int err;
    uint32_t physicalAddress, aligned;
    mxc_flc_regs_t *flc;

    aligned = address & ~0xf;
    if ((err = MXC_FLC_ME18_GetPhysicalAddress(aligned, &physicalAddress)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_FLC_ME18_GetByAddress(&flc, address)) != E_NO_ERROR) {
        return err;
    }

    err = MXC_FLC_RevA_Write32Using128((mxc_flc_reva_regs_t *)flc, address, data, physicalAddress);

    // Flush the cache
    MXC_FLC_ME18_Flash_Operation();

    return err;
}

// *****************************************************************************
#if IAR_PRAGMAS
#pragma section = ".flashprog"
#else
__attribute__((section(".flashprog")))
#endif
int MXC_FLC_Write128(uint32_t address, uint32_t *data)
{
    int err;
    uint32_t physicalAddress, aligned;
    mxc_flc_regs_t *flc;

    aligned = address & ~0xf;
    if ((err = MXC_FLC_ME18_GetPhysicalAddress(aligned, &physicalAddress)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_FLC_ME18_GetByAddress(&flc, address)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_FLC_RevA_Write128((mxc_flc_reva_regs_t *)flc, physicalAddress, data)) !=
        E_NO_ERROR) {
        return err;
    }

    // Flush the cache
    MXC_FLC_ME18_Flash_Operation();

    return MXC_FLC_Com_VerifyData(address, 4, data);
}

// *****************************************************************************
int MXC_FLC_EnableInt(uint32_t mask)
{
    return MXC_FLC_RevA_EnableInt(mask);
}

// *****************************************************************************
int MXC_FLC_DisableInt(uint32_t mask)
{
    return MXC_FLC_RevA_DisableInt(mask);
}

// *****************************************************************************
int MXC_FLC_GetFlags(void)
{
    return MXC_FLC_RevA_GetFlags();
}

// *****************************************************************************
int MXC_FLC_ClearFlags(uint32_t mask)
{
    return MXC_FLC_RevA_ClearFlags(mask);
}

// *****************************************************************************
int MXC_FLC_UnlockInfoBlock(uint32_t address)
{
    return MXC_FLC_RevA_UnlockInfoBlock((mxc_flc_reva_regs_t *)MXC_FLC, address);
}

// *****************************************************************************
int MXC_FLC_LockInfoBlock(uint32_t address)
{
    return MXC_FLC_RevA_LockInfoBlock((mxc_flc_reva_regs_t *)MXC_FLC, address);
}

//******************************************************************************
int MXC_FLC_BlockPageWrite(uint32_t address)
{
    int err;

    if (address < MXC_FLASH0_MEM_BASE ||
        address > (MXC_FLASH1_MEM_BASE + MXC_FLASH1_MEM_SIZE)) { // Check address valid
        return E_INVALID;
    }

    mxc_flc_regs_t *flc;
    if ((err = MXC_FLC_ME18_GetByAddress(&flc, address)) !=
        E_NO_ERROR) { // Get FLC to be able to pass correct base address
        return err;
    }

    switch (MXC_FLC_GET_IDX(flc)) {
    case 0:
        err = MXC_FLC_RevA_BlockPageWrite(address, MXC_FLASH0_MEM_BASE);
        break;
    case 1:
        err = MXC_FLC_RevA_BlockPageWrite(address, MXC_FLASH1_MEM_BASE);
        break;
    default:
        return E_INVALID;
    }

    return err;
}

//******************************************************************************
int MXC_FLC_BlockPageRead(uint32_t address)
{
    int err;
    mxc_flc_regs_t *flc;

    if (address < MXC_FLASH0_MEM_BASE ||
        address > (MXC_FLASH1_MEM_BASE + MXC_FLASH1_MEM_SIZE)) { // Check address valid
        return E_INVALID;
    }

    if ((err = MXC_FLC_ME18_GetByAddress(&flc, address)) !=
        E_NO_ERROR) { // Get FLC to be able to pass correct base address
        return err;
    }

    switch (MXC_FLC_GET_IDX(flc)) {
    case 0:
        err = MXC_FLC_RevA_BlockPageRead(address, MXC_FLASH0_MEM_BASE);
        break;
    case 1:
        err = MXC_FLC_RevA_BlockPageRead(address, MXC_FLASH1_MEM_BASE);
        break;
    default:
        return E_INVALID;
    }

    return err;
}

//******************************************************************************
volatile uint32_t *MXC_FLC_GetWELR(uint32_t address, uint32_t page_num)
{
    uint32_t reg_num;
    reg_num = page_num >>
              5; // Divide by 32 to get WELR register number containing the page lock bit

    mxc_flc_regs_t *flc;
    if (MXC_FLC_ME18_GetByAddress(&flc, address) != E_NO_ERROR) { // Check address valid
        return NULL;
    }

    switch (reg_num) {
    case 0:
        return &(flc->welr0);
    case 1:
        return &(flc->welr1);
    case 2:
        return &(flc->welr2);
    case 3:
        return &(flc->welr3);
    case 4:
        return &(flc->welr4);
    case 5:
        return &(flc->welr5);
    }

    return NULL;
}

//******************************************************************************
volatile uint32_t *MXC_FLC_GetRLR(uint32_t address, uint32_t page_num)
{
    uint32_t reg_num;
    reg_num = page_num >> 5; // Divide by 32 to get RLR register number containing the page lock bit

    mxc_flc_regs_t *flc;
    if (MXC_FLC_ME18_GetByAddress(&flc, address) != E_NO_ERROR) { // Check address valid
        return NULL;
    }

    switch (reg_num) {
    case 0:
        return &(flc->rlr0);
    case 1:
        return &(flc->rlr1);
    case 2:
        return &(flc->rlr2);
    case 3:
        return &(flc->rlr3);
    case 4:
        return &(flc->rlr4);
    case 5:
        return &(flc->rlr5);
    }

    return NULL;
}
