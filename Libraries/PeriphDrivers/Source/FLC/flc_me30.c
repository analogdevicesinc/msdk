/******************************************************************************
 *
 * Copyright (C) 2024-2025 Analog Devices, Inc.
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
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "flc.h"
#include "flc_reva.h"
#include "flc_common.h"
#include "mcr_regs.h" // For ECCEN registers.

/* Line buffer size in bytes */
#define MXC_FLC_LINE_BUFFER_SIZE 32
#define MXC_FLC_LINE_BUFFER_MASK (MXC_FLC_LINE_BUFFER_SIZE - 1)

//******************************************************************************
void MXC_FLC_ME30_Flash_Operation(void)
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
    MXC_GCR->sysctrl |= MXC_F_GCR_SYSCTRL_ICC_FLUSH;

    /* Wait for flush to complete */
    while (MXC_GCR->sysctrl & MXC_F_GCR_SYSCTRL_ICC_FLUSH) {}

    // Clear the line fill buffer by reading 2 pages from flash
    volatile uint32_t *line_addr;
    volatile uint32_t __unused line; // __unused attribute removes warning
    line_addr = (uint32_t *)(MXC_FLASH_MEM_BASE);
    line = *line_addr;
    line_addr = (uint32_t *)(MXC_FLASH_MEM_BASE + MXC_FLASH_PAGE_SIZE);
    line = *line_addr;
}

//******************************************************************************
int MXC_FLC_ME30_GetPhysicalAddress(uint32_t addr, uint32_t *result)
{
    if ((addr >= MXC_FLASH_MEM_BASE) && (addr < (MXC_FLASH_MEM_BASE + MXC_FLASH_MEM_SIZE))) {
        *result = addr & (MXC_FLASH_MEM_SIZE - 1);
#if CONFIG_TRUSTED_EXECUTION_SECURE
    } else if ((addr >= MXC_INFO_MEM_BASE) && (addr < (MXC_INFO_MEM_BASE + MXC_INFO_MEM_SIZE))) {
        *result = (addr & (MXC_INFO_MEM_SIZE - 1)) + MXC_FLASH_MEM_SIZE;
#endif
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

//******************************************************************************
int MXC_FLC_Init(void)
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

    if ((err = MXC_FLC_ME30_GetPhysicalAddress(address, &addr)) < E_NO_ERROR) {
        return err;
    }

    err = MXC_FLC_RevA_PageErase((mxc_flc_reva_regs_t *)MXC_FLC, addr);

    // Flush the cache
    MXC_FLC_ME30_Flash_Operation();

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
    uint32_t addr;

    // Address checked if it is 128-bit aligned
    if (address & 0xF) {
        return E_BAD_PARAM;
    }

    if ((err = MXC_FLC_ME30_GetPhysicalAddress(address, &addr)) < E_NO_ERROR) {
        return err;
    }

    err = MXC_FLC_RevA_Write128((mxc_flc_reva_regs_t *)MXC_FLC, addr, data);

    // Flush the cache
    MXC_FLC_ME30_Flash_Operation();

    return err;
}

//******************************************************************************
int MXC_FLC_Write32(uint32_t address, uint32_t data)
{
    uint32_t addr, aligned;
    int err;

    // Address checked if it is byte addressable
    if (address & 0x3) {
        return E_BAD_PARAM;
    }

    // Align address to 128-bit word
    aligned = address & 0xfffffff0;

    if ((err = MXC_FLC_ME30_GetPhysicalAddress(aligned, &addr)) < E_NO_ERROR) {
        return err;
    }

    err = MXC_FLC_RevA_Write32Using128((mxc_flc_reva_regs_t *)MXC_FLC, address, data, addr);

    // Flush the cache
    MXC_FLC_ME30_Flash_Operation();

    return err;
}

//******************************************************************************
int MXC_FLC_MassErase(void)
{
    int err;

    err = MXC_FLC_RevA_MassErase((mxc_flc_reva_regs_t *)MXC_FLC);

    // Flush the cache
    MXC_FLC_ME30_Flash_Operation();

    return err;
}

//******************************************************************************
int MXC_FLC_UnlockInfoBlock(uint32_t address)
{
    /* Flash Controller only accessible in secure world. */
#if defined(CONFIG_TRUSTED_EXECUTION_SECURE) || (CONFIG_TRUSTED_EXECUTION_SECURE != 0)
    if ((address < MXC_INFO_MEM_BASE) ||
        (address >= (MXC_INFO_MEM_BASE + (MXC_INFO_MEM_SIZE * 2)))) {
        return E_BAD_PARAM;
    }
#else
    return E_NOT_SUPPORTED;
#endif

    /* Make sure the info block is locked */
    MXC_FLC->actrl = 0x1234;

    /* Write the unlock sequence */
    MXC_FLC->actrl = 0x55bcbe69;
    MXC_FLC->actrl = 0x7688c189;
    MXC_FLC->actrl = 0x82306612;

    return E_NO_ERROR;
}

//******************************************************************************
int MXC_FLC_LockInfoBlock(uint32_t address)
{
    /* Flash Controller only accessible in secure world. */
#if defined(CONFIG_TRUSTED_EXECUTION_SECURE) || (CONFIG_TRUSTED_EXECUTION_SECURE != 0)
    if ((address < MXC_INFO_MEM_BASE) ||
        (address >= (MXC_INFO_MEM_BASE + (MXC_INFO_MEM_SIZE * 2)))) {
        return E_BAD_PARAM;
    }
#else
    return E_NOT_SUPPORTED;
#endif

    MXC_FLC->actrl = 0xDEADBEEF;
    return E_NO_ERROR;
}

//******************************************************************************
int MXC_FLC_Write(uint32_t address, uint32_t length, uint32_t *buffer)
{
    return MXC_FLC_Com_Write(address, length, buffer);
}

/* Using memcpy could cause the CPU to fetch instructions from the flash,
 * which will then update ECC data and flags. To avoid that, an in-RAM
 * version of memcpy is declared here.
 */
#if IAR_PRAGMAS
#pragma section = ".flashprog"
#else
__attribute__((section(".flashprog")))
#endif
static void MXC_FLC_MemCopy(uint8_t *dest, uint8_t *src, int len)
{
    for (int i = 0; i < len; i++) {
        dest[i] = src[i];
    }
}

/* Using memcpy could cause the CPU to fetch instructions from the flash,
 * which will then update ECC data and flags. To avoid that, an in-RAM
 * version of memcpy is declared here.
 */
#if IAR_PRAGMAS
#pragma section = ".flashprog"
#else
__attribute__((section(".flashprog")))
#endif
static void MXC_FLC_MemCopy32(uint32_t *dest, uint32_t *src, int len)
{
    for (int i = 0; i < len; i++) {
        dest[i] = src[i];
    }
}

#if IAR_PRAGMAS
#pragma section = ".flashprog"
#else
__attribute__((section(".flashprog")))
#endif
int MXC_FLC_ReadAligned32(uint32_t address, void *buffer)
{
    uint32_t eccdata;
    uint16_t ecc_odd, ecc_even;
    uint8_t *buf = buffer;

    /* Align the address to 32 bytes */
    uint32_t aligned = address & (~MXC_FLC_LINE_BUFFER_MASK);

    /* Do not use built-in memcpy, we do not want to jump to a flash section */
    MXC_FLC_MemCopy32((uint32_t *)buffer, (uint32_t *)aligned, (MXC_FLC_LINE_BUFFER_SIZE >> 2));

    if (MXC_GCR->eccerr & MXC_F_GCR_ECCERR_FLASH) {
        /* Get the ECC data */
        eccdata = MXC_FLC->eccdata;
        /* Clear the ECC error */
        MXC_GCR->eccerr = MXC_F_GCR_ECCERR_FLASH;

        if (MXC_GCR->eccced & MXC_F_GCR_ECCCED_FLASH) {
            /* Clear the ECC correctable error */
            MXC_GCR->eccced = MXC_F_GCR_ECCCED_FLASH;
        } else {
            return E_BAD_STATE;
        }

        /* Get the ECC odd and even */
        ecc_odd = (uint16_t)((eccdata >> 16) & 0x1FF);
        ecc_even = (uint16_t)(eccdata & 0x1FF);

        /* Check if the line is unwritten and correct the ECC flip if necessary */
        if (ecc_even == 0x1FF) {
            for (int i = 0; i < (MXC_FLC_LINE_BUFFER_SIZE / 2); i++) {
                if ((i == (MXC_FLC_LINE_BUFFER_SIZE / 2) - 1) && (buf[i] == 0xFD)) {
                    buf[(MXC_FLC_LINE_BUFFER_SIZE / 2) - 1] = 0xFF;
                } else if (buf[i] != 0xFF) {
                    break;
                }
            }
        }

        /* Check if the line is unwritten and correct the ECC flip if necessary */
        if (ecc_odd == 0x1FF) {
            for (int i = (MXC_FLC_LINE_BUFFER_SIZE / 2); i < MXC_FLC_LINE_BUFFER_SIZE; i++) {
                if ((i == (MXC_FLC_LINE_BUFFER_SIZE - 1)) && (buf[i] == 0xFD)) {
                    buf[MXC_FLC_LINE_BUFFER_SIZE - 1] = 0xFF;
                } else if (buf[i] != 0xFF) {
                    break;
                }
            }
        }
    }

    return E_SUCCESS;
}

//******************************************************************************
#if IAR_PRAGMAS
#pragma section = ".flashprog"
#else
__attribute__((section(".flashprog")))
#endif
int MXC_FLC_ReadECC(uint32_t address, void *buffer, int len)
{
    int ret;
    uint8_t buf[MXC_FLC_LINE_BUFFER_SIZE];
    /* Bytes to read before next 32-byte aligned address */
    uint8_t bytes_to_align;
    uint8_t *ptr = buffer;

    /* Align the read to 32-byte address boundary */
    if (address & MXC_FLC_LINE_BUFFER_MASK) {
        ret = MXC_FLC_ReadAligned32(address, buf);
        if (ret != E_SUCCESS) {
            return ret;
        }

        bytes_to_align = (address & ~MXC_FLC_LINE_BUFFER_MASK) + MXC_FLC_LINE_BUFFER_SIZE - address;
        bytes_to_align = (len < bytes_to_align) ? len : bytes_to_align;
        MXC_FLC_MemCopy(ptr, buf + (address & MXC_FLC_LINE_BUFFER_MASK), bytes_to_align);
        len -= bytes_to_align;
        address += bytes_to_align;
        ptr += bytes_to_align;
    }

    /* Read the data in chunks */
    while (len >= MXC_FLC_LINE_BUFFER_SIZE) {
        ret = MXC_FLC_ReadAligned32(address, buf);
        if (ret != E_SUCCESS) {
            return ret;
        }

        MXC_FLC_MemCopy32((uint32_t *)ptr, (uint32_t *)buf, MXC_FLC_LINE_BUFFER_SIZE >> 2);
        len -= MXC_FLC_LINE_BUFFER_SIZE;
        address += MXC_FLC_LINE_BUFFER_SIZE;
        ptr += MXC_FLC_LINE_BUFFER_SIZE;
    }

    /* Read the remaining bytes */
    if (len > 0) {
        ret = MXC_FLC_ReadAligned32(address, buf);
        if (ret != E_SUCCESS) {
            return ret;
        }

        MXC_FLC_MemCopy(ptr, buf, len);
    }

    return E_SUCCESS;
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
    return NULL;
}

//******************************************************************************
volatile uint32_t *MXC_FLC_GetRLR(uint32_t address, uint32_t page_num)
{
    return NULL;
}
