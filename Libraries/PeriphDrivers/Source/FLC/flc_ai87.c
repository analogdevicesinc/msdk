/**
 * @file flc.h
 * @brief      Flash Controler driver.
 * @details    This driver can be used to operate on the embedded flash memory.
 */
/* ****************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
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
 * $Date: 2018-10-15 16:49:29 -0500 (Mon, 15 Oct 2018) $
 * $Revision: 38520 $
 *
 *************************************************************************** */

/* **** Includes **** */
#include <string.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "flc.h"
#include "flc_reva.h"
#include "flc_common.h"
#include "mcr_regs.h" // For ECCEN registers.

//******************************************************************************
void MXC_FLC_AI87_Flash_Operation(void)
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
    MXC_GCR->sysctrl |= MXC_F_GCR_SYSCTRL_ICC0_FLUSH;

    /* Wait for flush to complete */
    while (MXC_GCR->sysctrl & MXC_F_GCR_SYSCTRL_ICC0_FLUSH) {}

    // Clear the line fill buffer by reading 2 pages from flash
    volatile uint32_t *line_addr;
    volatile uint32_t __unused line; // __unused attribute removes warning
    line_addr = (uint32_t *)(MXC_FLASH_MEM_BASE);
    line = *line_addr;
    line_addr = (uint32_t *)(MXC_FLASH_MEM_BASE + MXC_FLASH_PAGE_SIZE);
    line = *line_addr;
}

//******************************************************************************
int MXC_FLC_AI87_GetByAddress(mxc_flc_regs_t **flc, uint32_t addr)
{
    if ((addr >= MXC_FLASH_MEM_BASE) && (addr < (MXC_FLASH_MEM_BASE + MXC_FLASH_MEM_SIZE))) {
        *flc = MXC_FLC0;
    } else if ((addr >= MXC_INFO_MEM_BASE) && (addr < (MXC_INFO_MEM_BASE + MXC_INFO_MEM_SIZE))) {
        *flc = MXC_FLC0;
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

//******************************************************************************
int MXC_FLC_AI87_GetPhysicalAddress(uint32_t addr, uint32_t *result)
{
    if ((addr >= MXC_FLASH_MEM_BASE) && (addr < (MXC_FLASH_MEM_BASE + MXC_FLASH_MEM_SIZE))) {
        *result = addr - MXC_FLASH_MEM_BASE;
    } else if ((addr >= MXC_INFO_MEM_BASE) && (addr < (MXC_INFO_MEM_BASE + MXC_INFO_MEM_SIZE))) {
        *result = (addr & (MXC_INFO_MEM_SIZE - 1)) + MXC_FLASH_MEM_SIZE;
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

//******************************************************************************
int MXC_FLC_Init()
{
    return E_NO_ERROR;
}

//******************************************************************************
#if IAR_PRAGMAS
#pragma section = ".flashprog"
#else
__attribute__((section(".flashprog")))
#endif
int MXC_FLC_Busy(void)
{
    return MXC_FLC_RevA_Busy();
}

//******************************************************************************
#if IAR_PRAGMAS
#pragma section = ".flashprog"
#else
__attribute__((section(".flashprog")))
#endif
int MXC_FLC_PageErase(uint32_t address)
{
    int err;
    uint32_t addr;
    mxc_flc_regs_t *flc = NULL;

    // Get FLC Instance
    if ((err = MXC_FLC_AI87_GetByAddress(&flc, address)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_FLC_AI87_GetPhysicalAddress(address, &addr)) < E_NO_ERROR) {
        return err;
    }

    err = MXC_FLC_RevA_PageErase((mxc_flc_reva_regs_t *)flc, addr);
    // Flush the cache
    MXC_FLC_AI87_Flash_Operation();

    return err;
}

//******************************************************************************
#if IAR_PRAGMAS
#pragma section = ".flashprog"
#else
__attribute__((section(".flashprog")))
#endif
// make sure to disable ICC with ICC_Disable(); before Running this function
int MXC_FLC_Write128(uint32_t address, uint32_t *data)
{
    int err;
    mxc_flc_regs_t *flc = NULL;
    uint32_t addr;

    // Address checked if it is 128-bit aligned
    if (address & 0xF) {
        return E_BAD_PARAM;
    }

    // Get FLC Instance
    if ((err = MXC_FLC_AI87_GetByAddress(&flc, address)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_FLC_AI87_GetPhysicalAddress(address, &addr)) < E_NO_ERROR) {
        return err;
    }

    err = MXC_FLC_RevA_Write128((mxc_flc_reva_regs_t *)flc, addr, data);

    // Flush the cache
    MXC_FLC_AI87_Flash_Operation();

    if ((err = MXC_FLC_Com_VerifyData(address, 4, data)) != E_NO_ERROR) {
        return err;
    }

    return E_NO_ERROR;
}

//******************************************************************************
int MXC_FLC_Write32(uint32_t address, uint32_t data)
{
    uint32_t addr, aligned;
    int err;
    mxc_flc_regs_t *flc = NULL;

    // Address checked if it is byte addressable
    if (address & 0x3) {
        return E_BAD_PARAM;
    }

    // Align address to 128-bit word
    aligned = address & 0xfffffff0;

    // Get FLC Instance
    if ((err = MXC_FLC_AI87_GetByAddress(&flc, address)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_FLC_AI87_GetPhysicalAddress(aligned, &addr)) < E_NO_ERROR) {
        return err;
    }

    err = MXC_FLC_RevA_Write32Using128((mxc_flc_reva_regs_t *)flc, address, data, addr);

    // Flush the cache
    MXC_FLC_AI87_Flash_Operation();

    return err;
}

//******************************************************************************
int MXC_FLC_MassErase(void)
{
    int err, i;
    mxc_flc_regs_t *flc;

    for (i = 0; i < MXC_FLC_INSTANCES; i++) {
        flc = MXC_FLC_GET_FLC(i);
        err = MXC_FLC_RevA_MassErase((mxc_flc_reva_regs_t *)flc);

        if (err != E_NO_ERROR) {
            return err;
        }

        MXC_FLC_AI87_Flash_Operation();
    }

    return E_NO_ERROR;
}

//******************************************************************************
int MXC_FLC_UnlockInfoBlock(uint32_t address)
{
    int err;
    mxc_flc_regs_t *flc;

    if ((err = MXC_FLC_AI87_GetByAddress(&flc, address)) != E_NO_ERROR) {
        return err;
    }

    return MXC_FLC_RevA_UnlockInfoBlock((mxc_flc_reva_regs_t *)flc, address);
}

//******************************************************************************
int MXC_FLC_LockInfoBlock(uint32_t address)
{
    int err;
    mxc_flc_regs_t *flc;

    if ((err = MXC_FLC_AI87_GetByAddress(&flc, address)) != E_NO_ERROR) {
        return err;
    }

    return MXC_FLC_RevA_LockInfoBlock((mxc_flc_reva_regs_t *)flc, address);
}

//******************************************************************************
int MXC_FLC_Write(uint32_t address, uint32_t length, uint32_t *buffer)
{
    return MXC_FLC_Com_Write(address, length, buffer);
}

//******************************************************************************
void MXC_FLC_Read(int address, void *buffer, int len)
{
    MXC_FLC_Com_Read(address, buffer, len);
}

//******************************************************************************
int MXC_FLC_EnableInt(uint32_t flags)
{
    return MXC_FLC_RevA_EnableInt(flags);
}

//******************************************************************************
int MXC_FLC_DisableInt(uint32_t flags)
{
    return MXC_FLC_RevA_DisableInt(flags);
}

//******************************************************************************
int MXC_FLC_GetFlags(void)
{
    return MXC_FLC_RevA_GetFlags();
}

//******************************************************************************
int MXC_FLC_ClearFlags(uint32_t flags)
{
    return MXC_FLC_RevA_ClearFlags(flags);
}

//******************************************************************************
int MXC_FLC_BlockPageWrite(uint32_t address)
{
    if (address < MXC_FLASH_MEM_BASE || address > (MXC_FLASH_MEM_BASE + MXC_FLASH_MEM_SIZE)) {
        return E_INVALID;
    }

    return MXC_FLC_RevA_BlockPageWrite(address, MXC_FLASH_MEM_BASE);
}

//******************************************************************************
int MXC_FLC_BlockPageRead(uint32_t address)
{
    if (address < MXC_FLASH_MEM_BASE || address > (MXC_FLASH_MEM_BASE + MXC_FLASH_MEM_SIZE)) {
        return E_INVALID;
    }

    return MXC_FLC_RevA_BlockPageRead(address, MXC_FLASH_MEM_BASE);
}

//******************************************************************************
volatile uint32_t *MXC_FLC_GetWELR(uint32_t address, uint32_t page_num)
{
    uint32_t reg_num;
    reg_num = page_num >>
              5; // Divide by 32 to get WELR register number containing the page lock bit

    if (address < MXC_FLASH_MEM_BASE || address > (MXC_FLASH_MEM_BASE + MXC_FLASH_MEM_SIZE)) {
        return NULL;
    }

    switch (reg_num) {
    case 0:
        return &(MXC_FLC0->welr0);
    case 1:
        return &(MXC_FLC0->welr1);
    case 2:
        return &(MXC_FLC0->welr2);
    case 3:
        return &(MXC_FLC0->welr3);
    case 4:
        return &(MXC_FLC0->welr4);
    }

    return NULL;
}

//******************************************************************************
volatile uint32_t *MXC_FLC_GetRLR(uint32_t address, uint32_t page_num)
{
    uint32_t reg_num;
    reg_num = page_num >> 5; // Divide by 32 to get RLR register number containing the page lock bit

    if (address < MXC_FLASH_MEM_BASE || address > (MXC_FLASH_MEM_BASE + MXC_FLASH_MEM_SIZE)) {
        return NULL;
    }

    switch (reg_num) {
    case 0:
        return &(MXC_FLC0->rlr0);
    case 1:
        return &(MXC_FLC0->rlr1);
    case 2:
        return &(MXC_FLC0->rlr2);
    case 3:
        return &(MXC_FLC0->rlr3);
    case 4:
        return &(MXC_FLC0->rlr4);
    }

    return NULL;
}