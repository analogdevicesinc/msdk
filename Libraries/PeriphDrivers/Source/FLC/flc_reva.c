/**
 * @file flc.h
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
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "flc_reva.h"
#include "flc.h"

// TODO(CM33): Check for secure vs non-secure accesses here.

/**
 * @ingroup flc
 * @{
 */

/* **** Definitions **** */

/* **** Globals **** */
#ifdef MXC_FLC0
static mxc_flc_reva_regs_t *flc_int = (mxc_flc_reva_regs_t *)MXC_FLC0;
#else
static mxc_flc_reva_regs_t *flc_int = (mxc_flc_reva_regs_t *)MXC_FLC;
#endif

/* **** Functions **** */

//******************************************************************************
#if IAR_PRAGMAS
#pragma section = ".flashprog"
#else
__attribute__((section(".flashprog")))
#endif
static int MXC_busy_flc(mxc_flc_reva_regs_t *flc)
{
    return (flc->ctrl &
            (MXC_F_FLC_REVA_CTRL_WR | MXC_F_FLC_REVA_CTRL_ME | MXC_F_FLC_REVA_CTRL_PGE));
}

//******************************************************************************
#if IAR_PRAGMAS
#pragma section = ".flashprog"
#else
__attribute__((section(".flashprog")))
#endif
static int MXC_prepare_flc(mxc_flc_reva_regs_t *flc)
{
    /* Check if the flash controller is busy */
    if (MXC_busy_flc(flc)) {
        return E_BUSY;
    }

    // Set flash clock divider to generate a 1MHz clock from the APB clock
    flc->clkdiv = SystemCoreClock / 1000000;

    /* Clear stale errors */
    if (flc->intr & MXC_F_FLC_REVA_INTR_AF) {
        flc->intr &= ~MXC_F_FLC_REVA_INTR_AF;
    }

    /* Unlock flash */
    flc->ctrl = (flc->ctrl & ~MXC_F_FLC_REVA_CTRL_UNLOCK) | MXC_S_FLC_REVA_CTRL_UNLOCK_UNLOCKED;

    return E_NO_ERROR;
}

//******************************************************************************
#if IAR_PRAGMAS
#pragma section = ".flashprog"
#else
__attribute__((section(".flashprog")))
#endif
int MXC_FLC_RevA_Busy(void)
{
    uint32_t flc_cn = 0;
    int i;
    mxc_flc_reva_regs_t *flc;

    for (i = 0; i < MXC_FLC_INSTANCES; i++) {
        flc = (mxc_flc_reva_regs_t *)MXC_FLC_GET_FLC(i);
        flc_cn = MXC_busy_flc(flc);

        if (flc_cn != 0) {
            break;
        }
    }

    return flc_cn;
}
//******************************************************************************
#if IAR_PRAGMAS
#pragma section = ".flashprog"
#else
__attribute__((section(".flashprog")))
#endif
int MXC_FLC_RevA_MassErase(mxc_flc_reva_regs_t *flc)
{
    int err;

    if ((err = MXC_prepare_flc(flc)) != E_NO_ERROR) {
        return err;
    }

    /* Write mass erase code */
    flc->ctrl = (flc->ctrl & ~MXC_F_FLC_REVA_CTRL_ERASE_CODE) |
                MXC_S_FLC_REVA_CTRL_ERASE_CODE_ERASEALL;

    /* Issue mass erase command */
    flc->ctrl |= MXC_F_FLC_REVA_CTRL_ME;

    /* Wait until flash operation is complete */
    while (MXC_busy_flc(flc)) {}
    while ((flc->intr & MXC_F_FLC_REVA_INTR_DONE) == 0) {}
    flc->intr &= ~MXC_F_FLC_REVA_INTR_DONE;

    /* Lock flash */
    flc->ctrl &= ~MXC_F_FLC_REVA_CTRL_UNLOCK;

    /* Check access violations */
    if (flc->intr & MXC_F_FLC_REVA_INTR_AF) {
        flc->intr &= ~MXC_F_FLC_REVA_INTR_AF;
        return E_BAD_STATE;
    }

    return E_NO_ERROR;
}

//******************************************************************************
#if IAR_PRAGMAS
#pragma section = ".flashprog"
#else
__attribute__((section(".flashprog")))
#endif
int MXC_FLC_RevA_PageErase(mxc_flc_reva_regs_t *flc, uint32_t addr)
{
    int err;

    if ((err = MXC_prepare_flc(flc)) != E_NO_ERROR) {
        return err;
    }

    /* Write page erase code */
    flc->ctrl = (flc->ctrl & ~MXC_F_FLC_REVA_CTRL_ERASE_CODE) |
                MXC_S_FLC_REVA_CTRL_ERASE_CODE_ERASEPAGE;
    /* Issue page erase command */
    flc->addr = addr;
    flc->ctrl |= MXC_F_FLC_REVA_CTRL_PGE;

    /* Wait until flash operation is complete */
    while (MXC_busy_flc(flc)) {}
    while ((flc->intr & MXC_F_FLC_REVA_INTR_DONE) == 0) {}
    flc->intr &= ~MXC_F_FLC_REVA_INTR_DONE;

    /* Lock flash */
    flc->ctrl &= ~MXC_F_FLC_REVA_CTRL_UNLOCK;

    /* Check access violations */
    if (flc->intr & MXC_F_FLC_REVA_INTR_AF) {
        flc->intr &= ~MXC_F_FLC_REVA_INTR_AF;
        return E_BAD_STATE;
    }

    return E_NO_ERROR;
}

//******************************************************************************
#if IAR_PRAGMAS
#pragma section = ".flashprog"
#else
__attribute__((section(".flashprog")))
#endif
// make sure to disable ICC with ICC_Disable(); before Running this function
int MXC_FLC_RevA_Write32(mxc_flc_reva_regs_t *flc, uint32_t logicAddr, uint32_t data,
                         uint32_t physicalAddr)
{
    int err;

    // Address checked if it is byte addressable
    if (logicAddr & 0x3) {
        return E_BAD_PARAM;
    }

    // Check if the location trying to be written has 1's in to be written to 0's
    if ((*(uint32_t *)logicAddr & data) != data) {
        return E_BAD_STATE;
    }

    // Align address to 32-bit word
    logicAddr = logicAddr & 0xfffffffc;

    if ((err = MXC_prepare_flc(flc)) != E_NO_ERROR) {
        return err;
    }

    // write 32-bits
    flc->ctrl |= MXC_F_FLC_REVA_CTRL_WDTH;

    // write the data
    flc->addr = logicAddr;
    flc->data[0] = data;
    flc->ctrl |= MXC_F_FLC_REVA_CTRL_WR;

    /* Wait until flash operation is complete */
    while ((flc->ctrl & MXC_F_FLC_REVA_CTRL_PEND) != 0) {}
    while (MXC_busy_flc(flc)) {}
    while ((flc->intr & MXC_F_FLC_REVA_INTR_DONE) == 0) {}
    flc->intr &= ~MXC_F_FLC_REVA_INTR_DONE;

    /* Lock flash */
    flc->ctrl &= ~MXC_F_FLC_REVA_CTRL_UNLOCK;

    /* Check access violations */
    if (flc->intr & MXC_F_FLC_REVA_INTR_AF) {
        flc->intr &= ~MXC_F_FLC_REVA_INTR_AF;
        return E_BAD_STATE;
    }

    return E_NO_ERROR;
}

//******************************************************************************

#include <stdio.h>
#if IAR_PRAGMAS
#pragma section = ".flashprog"
#else
__attribute__((section(".flashprog")))
#endif
// make sure to disable ICC with ICC_Disable(); before Running this function
int MXC_FLC_RevA_Write32Using128(mxc_flc_reva_regs_t *flc, uint32_t logicAddr, uint32_t data,
                                 uint32_t physicalAddr)
{
    int err, i = 0;
    uint32_t byte;
    volatile uint32_t *ptr;
    uint32_t current_data[4] = { 0, 0, 0, 0 };

    // Address checked if it is byte addressable
    if (logicAddr & 0x3) {
        return E_BAD_PARAM;
    }

    // Check if the location trying to be written has 1's in to be written to 0's
    uint32_t mem_contents = *(uint32_t *)logicAddr;
    // printf("@0x%08x: 0x%08x => err: %u\n", logicAddr, mem_contents, MXC_GCR->eccerr);

#if (TARGET_NUM == 32657)
    // There is a bug in the MAX32657 ECC where erasing flash will also erase the ECC bits.
    // Reading from erased memory will signal an ECC error that is falsely corrected from
    // 0xFF to 0xFD on 16th byte of each 128-bit line.
    // Workaround by setting 16th bit of each line to 0xFF when reading back.
    if (MXC_GCR->eccerr & MXC_F_GCR_ECCERR_FLASH) {
        // Clear ECC error and correct read bit.
        MXC_GCR->eccerr = MXC_F_GCR_ECCERR_FLASH;

        if (logicAddr & 0x0C && mem_contents == 0xFDFFFFFF) {
            mem_contents = 0xFFFFFFFF;
        }
    }
#endif

    if ((mem_contents & data) != data) {
        return E_BAD_STATE;
    }

    // Get byte idx within 128-bit word
    byte = (logicAddr & 0xf);
    // Align address to 128-bit word
    logicAddr = logicAddr & 0xfffffff0;

    if ((err = MXC_prepare_flc(flc)) != E_NO_ERROR) {
        return err;
    }

    // Get current data stored in flash
    for (ptr = (uint32_t *)logicAddr; ptr < (uint32_t *)(logicAddr + 16); ptr++, i++) {
        current_data[i] = *ptr;
    }

    // write the data
    flc->addr = physicalAddr;

    if (byte < 4) {
        current_data[0] = data;
    } else if (byte < 8) {
        current_data[1] = data;
    } else if (byte < 12) {
        current_data[2] = data;
    } else {
        current_data[3] = data;
    }

    return MXC_FLC_Write128(logicAddr, current_data);
}

//******************************************************************************
#if IAR_PRAGMAS
#pragma section = ".flashprog"
#else
__attribute__((section(".flashprog")))
#endif
// make sure to disable ICC with ICC_Disable(); before Running this function
int MXC_FLC_RevA_Write128(mxc_flc_reva_regs_t *flc, uint32_t addr, uint32_t *data)
{
    int err;

    // Address checked if it is 128-bit aligned
    if (addr & 0xF) {
        return E_BAD_PARAM;
    }

    if ((err = MXC_prepare_flc(flc)) != E_NO_ERROR) {
        return err;
    }

    // write 128-bits
    flc->ctrl &= ~MXC_F_FLC_REVA_CTRL_WDTH;

    // write the data
    flc->addr = addr;
    flc->data[0] = data[0];
    flc->data[1] = data[1];
    flc->data[2] = data[2];
    flc->data[3] = data[3];
    flc->ctrl |= MXC_F_FLC_REVA_CTRL_WR;

    /* Wait until flash operation is complete */
    while ((flc->ctrl & MXC_F_FLC_REVA_CTRL_PEND) != 0) {}
    while (MXC_busy_flc(flc)) {}
    while ((flc->intr & MXC_F_FLC_REVA_INTR_DONE) == 0) {}
    flc->intr &= ~MXC_F_FLC_REVA_INTR_DONE;

    /* Lock flash */
    flc->ctrl &= ~MXC_F_FLC_REVA_CTRL_UNLOCK;

    /* Check access violations */
    if (flc->intr & MXC_F_FLC_REVA_INTR_AF) {
        flc->intr &= ~MXC_F_FLC_REVA_INTR_AF;
        return E_BAD_STATE;
    }

    return E_NO_ERROR;
}

//******************************************************************************
void MXC_FLC_RevA_SetFLCInt(mxc_flc_reva_regs_t *flc)
{
    flc_int = flc;
}

//******************************************************************************
mxc_flc_reva_regs_t *MXC_FLC_RevA_GetFLCInt(void)
{
    return flc_int;
}

//******************************************************************************
int MXC_FLC_RevA_EnableInt(uint32_t mask)
{
    mask &= (MXC_F_FLC_REVA_INTR_DONEIE | MXC_F_FLC_REVA_INTR_AFIE);

    if (!mask) {
        /* No bits set? Wasn't something we can enable. */
        return E_BAD_PARAM;
    }

    /* Apply enables and write back, preserving the flags */
    flc_int->intr |= mask;

    return E_NO_ERROR;
}

//******************************************************************************
int MXC_FLC_RevA_DisableInt(uint32_t mask)
{
    mask &= (MXC_F_FLC_REVA_INTR_DONEIE | MXC_F_FLC_REVA_INTR_AFIE);

    if (!mask) {
        /* No bits set? Wasn't something we can disable. */
        return E_BAD_PARAM;
    }

    /* Apply disables and write back, preserving the flags */
    flc_int->intr &= ~mask;

    return E_NO_ERROR;
}

//******************************************************************************
int MXC_FLC_RevA_GetFlags(void)
{
    return (flc_int->intr & (MXC_F_FLC_REVA_INTR_DONE | MXC_F_FLC_REVA_INTR_AF));
}

//******************************************************************************
int MXC_FLC_RevA_ClearFlags(uint32_t mask)
{
    mask &= (MXC_F_FLC_REVA_INTR_DONE | MXC_F_FLC_REVA_INTR_AF);

    if (!mask) {
        /* No bits set? Wasn't something we can clear. */
        return E_BAD_PARAM;
    }

    /* Both flags are write zero clear */
    flc_int->intr ^= mask;

    return E_NO_ERROR;
}

//******************************************************************************
int MXC_FLC_RevA_UnlockInfoBlock(mxc_flc_reva_regs_t *flc, uint32_t address)
{
#if defined(CONFIG_TRUSTED_EXECUTION_SECURE) && (CONFIG_TRUSTED_EXECUTION_SECURE != 0) || \
    (TARGET_NUM != 32657)
    if ((address < MXC_INFO_MEM_BASE) ||
        (address >= (MXC_INFO_MEM_BASE + (MXC_INFO_MEM_SIZE * 2)))) {
        return E_BAD_PARAM;
    }
#endif

    /* Make sure the info block is locked */
    flc->actrl = 0x1234;

    /* Write the unlock sequence */
    flc->actrl = 0x3a7f5ca3;
    flc->actrl = 0xa1e34f20;
    flc->actrl = 0x9608b2c1;

    return E_NO_ERROR;
}

//******************************************************************************
int MXC_FLC_RevA_LockInfoBlock(mxc_flc_reva_regs_t *flc, uint32_t address)
{
#if defined(CONFIG_TRUSTED_EXECUTION_SECURE) && (CONFIG_TRUSTED_EXECUTION_SECURE != 0) || \
    (TARGET_NUM != 32657)
    if ((address < MXC_INFO_MEM_BASE) ||
        (address >= (MXC_INFO_MEM_BASE + (MXC_INFO_MEM_SIZE * 2)))) {
        return E_BAD_PARAM;
    }
#endif

    flc->actrl = 0xDEADBEEF;
    return E_NO_ERROR;
}

//******************************************************************************
int MXC_FLC_RevA_BlockPageWrite(uint32_t address, uint32_t bank_base)
{
    uint32_t page_num;
    page_num = address - bank_base; // Get page number in flash bank
    page_num /= MXC_FLASH_PAGE_SIZE;

    volatile uint32_t *welr = MXC_FLC_GetWELR(
        address, page_num); // Get pointer to WELR register containing corresponding page bit

    while (page_num > 31) { // Set corresponding bit in WELR register
        page_num -= 32;
    }
    *welr = (1 << page_num);

    return E_NO_ERROR;
}

//******************************************************************************
int MXC_FLC_RevA_BlockPageRead(uint32_t address, uint32_t bank_base)
{
    uint32_t page_num;
    page_num = address - bank_base; // Get page number in flash bank
    page_num /= MXC_FLASH_PAGE_SIZE;

    volatile uint32_t *rlr = MXC_FLC_GetRLR(
        address, page_num); // Get pointer to RLR register containing corresponding page bit

    while (page_num > 31) { // Set corresponding bit in WELR register
        page_num -= 32;
    }
    *rlr = (1 << page_num);

    return E_NO_ERROR;
}

/**@} end of group flc */
