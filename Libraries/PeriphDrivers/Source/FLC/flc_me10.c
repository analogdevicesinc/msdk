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
 * $Date: 2019-06-05 16:53:29 -0500 (Wed, 05 Jun 2019) $
 * $Revision: 43696 $
 *
 *************************************************************************** */

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
void MXC_FLC_ME10_Flash_Operation(void)
{
    /* Flush all instruction caches */
    MXC_ICC_Flush();
}

//******************************************************************************
int MXC_FLC_ME10_GetByAddress(mxc_flc_regs_t** flc, uint32_t addr)
{
    if ((addr >= MXC_FLASH_MEM_BASE) && (addr < (MXC_FLASH_MEM_BASE + MXC_FLASH_MEM_SIZE))) {
        *flc = MXC_FLC;
    } else if ((addr >= MXC_INFO_MEM_BASE) && (addr < (MXC_INFO_MEM_BASE + MXC_INFO_MEM_SIZE))) {
        *flc = MXC_FLC;
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

//******************************************************************************
int MXC_FLC_ME10_GetPhysicalAddress(uint32_t addr, uint32_t* result)
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

// *****************************************************************************
int MXC_FLC_Init(void)
{
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_FLC);
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

    if ((err = MXC_FLC_RevA_MassErase((mxc_flc_reva_regs_t*)MXC_FLC)) != E_NO_ERROR) {
        return err;
    }

    MXC_FLC_ME10_Flash_Operation();

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

    if ((err = MXC_FLC_ME10_GetPhysicalAddress(address, &physicalAddress)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_FLC_RevA_PageErase((mxc_flc_reva_regs_t*)MXC_FLC, physicalAddress)) !=
        E_NO_ERROR) {
        return err;
    }

    MXC_FLC_ME10_Flash_Operation();

    return E_NO_ERROR;
}

// *****************************************************************************
void MXC_FLC_Read(int address, void* buffer, int len)
{
    if (address < MXC_FLASH_MEM_BASE || address >= (MXC_FLASH_MEM_BASE + MXC_FLASH_MEM_SIZE)) {
        return;
    }
    MXC_FLC_Com_Read(address, buffer, len);
}

// *****************************************************************************
int MXC_FLC_Write(uint32_t address, uint32_t length, uint32_t* buffer)
{
    return MXC_FLC_Com_Write(address, length, buffer);
}

// *****************************************************************************
int MXC_FLC_Write32(uint32_t address, uint32_t data)
{
    int err;
    uint32_t physicalAddress, aligned;

    aligned = address & ~0xf;
    if ((err = MXC_FLC_ME10_GetPhysicalAddress(aligned, &physicalAddress)) != E_NO_ERROR) {
        return err;
    }

    err = MXC_FLC_RevA_Write32((mxc_flc_reva_regs_t*)MXC_FLC, address, data, physicalAddress);

    MXC_FLC_ME10_Flash_Operation();

    return err;
}

// *****************************************************************************
#if IAR_PRAGMAS
#pragma section = ".flashprog"
#else
__attribute__((section(".flashprog")))
#endif
int MXC_FLC_Write128(uint32_t address, uint32_t* data)
{
    int err;
    uint32_t physicalAddress, aligned;

    aligned = address & ~0xf;
    if ((err = MXC_FLC_ME10_GetPhysicalAddress(aligned, &physicalAddress)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_FLC_RevA_Write128((mxc_flc_reva_regs_t*)MXC_FLC, physicalAddress, data)) !=
        E_NO_ERROR) {
        return err;
    }

    MXC_FLC_ME10_Flash_Operation();

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
    return MXC_FLC_RevA_UnlockInfoBlock((mxc_flc_reva_regs_t*)MXC_FLC, address);
}

// *****************************************************************************
int MXC_FLC_LockInfoBlock(uint32_t address)
{
    return MXC_FLC_RevA_LockInfoBlock((mxc_flc_reva_regs_t*)MXC_FLC, address);
}

//******************************************************************************
int MXC_FLC_BlockPageWrite(uint32_t address)
{
    /* MAX32650 does not support flash page read and write locks */
    return E_NOT_SUPPORTED;
}

//******************************************************************************
int MXC_FLC_BlockPageRead(uint32_t address)
{
    /* MAX32650 does not support flash page read and write locks */
    return E_NOT_SUPPORTED;
}

//******************************************************************************
volatile uint32_t* MXC_FLC_GetWELR(uint32_t address, uint32_t page_num)
{
    /* MAX32650 does not support flash page read and write locks */
    return NULL;
}

//******************************************************************************
volatile uint32_t* MXC_FLC_GetRLR(uint32_t address, uint32_t page_num)
{
    /* MAX32650 does not support flash page read and write locks */
    return NULL;
}