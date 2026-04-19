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

/**
 * @file       mpc_me30.c
 * @brief      Memory Protection Controller (MPC) Driver for the MAX32657 (ME30).
 * @details    This driver is used to control the security policy of each
 *              memory region (SRAM0-4 and Flash).
 */

// TODO(MPC): Add Interrupt functionality.

/**
 *  The MPC is only accessible from Secure World.
 */
#if (CONFIG_TRUSTED_EXECUTION_SECURE == 1)

/**** Includes ****/

#include <stdbool.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mpc.h"
#include "spc.h"

/**** Definitions ****/
// Enumeration for security policy.
typedef enum { MXC_MPC_STATE_SECURE = 0, MXC_MPC_STATE_NONSECURE = 1 } mxc_mpc_state_t;

/**** Globals ****/

/**** Functions ****/

static int MXC_MPC_SetBlockSecurity(int start_addr, int end_addr, mxc_mpc_state_t state)
{
    int error;
    uint32_t phy_start_addr, phy_end_addr;
    uint32_t width;

    int i;
    uint32_t curr_addr, start_mpc_region_addr, end_mpc_region_addr;
    uint32_t start_block_idx;
    uint32_t end_block_idx;
    mxc_mpc_regs_t *start_mpc;
    mxc_mpc_regs_t *end_mpc;
    mxc_mpc_regs_t *curr_mpc;

    // Keep track of what memory type to help with sorting different MPCs.
    bool is_mem_flash;

    // Check for supported address ranges.
    if (start_addr > end_addr) {
        return E_BAD_PARAM;
    }

    // Get physical addresses from virtual secure/nonsecure addresses
    //  by clearing bit 28 - indicates the security state of address.
    phy_start_addr = start_addr & ~(1 << 28);
    phy_end_addr = end_addr & ~(1 << 28);

    // Ensure the start address is not past the end address.
    if (phy_start_addr > phy_end_addr) {
        return E_BAD_PARAM;
    }

    // Check addresses are within usable memory regions.
    error = MXC_MPC_CheckPhyBoundaries(phy_start_addr, phy_end_addr);
    if (error != E_NO_ERROR) {
        return error;
    }

    // Set all blocks within each MPC of the given memory regions to Secure.
    start_mpc = MXC_MPC_GetInstance(phy_start_addr);
    end_mpc = MXC_MPC_GetInstance(phy_end_addr);

    if (start_mpc == NULL || end_mpc == NULL) {
        // Given addresses are in memory space that doesn't support MPC.
        return E_BAD_PARAM;
    }

    curr_addr = phy_start_addr;

    // Select memory type for its corresponding instance.
    if (MXC_MPC_IS_FOR_FLASH(start_mpc)) {
        is_mem_flash = true;
    } else {
        is_mem_flash = false;
    }

    for (i = MXC_MPC_GET_IDX(start_mpc); i <= MXC_MPC_GET_IDX(end_mpc); i++) {
        // Get current MPC, whether in Flash or SRAM.
        if (is_mem_flash == true) {
            curr_mpc = MXC_MPC_FLASH_GET_BASE(i);
        } else {
            curr_mpc = MXC_MPC_SRAM_GET_BASE(i);
        }

        // Get the address range of the current MPC region.
        start_mpc_region_addr = curr_addr;

        // NOTE: Project Owner/Developer must be aware of the memory settings for Secure and Non-Secure
        //  boundaries do not share a single unit MPC block. An MPC block can only be set to one security
        //  policy (Secure or Non-Secure).
        if (i == MXC_MPC_GET_IDX(end_mpc)) {
            // The physical ending address is within the final MPC region - mark end with caller requested end address.
            end_mpc_region_addr = phy_end_addr;
        } else {
            // -1 to get the last address of the current region.
            end_mpc_region_addr =
                MXC_MPC_GET_PHY_MEM_BASE(curr_mpc) + MXC_MPC_GET_PHY_MEM_SIZE(curr_mpc) - 1;
        }

        start_block_idx = MXC_MPC_GetBlockIdx(curr_mpc, start_mpc_region_addr);
        end_block_idx = MXC_MPC_GetBlockIdx(curr_mpc, end_mpc_region_addr);

        // TODO(SW): Precautionary measure that needs to be confirmed - prevents the MPC_BLK_LUT[n]
        //  register from auto-incrementing after a full word read/write occurs.
        // There is only 1 available index (n) (in MPC_BLK_IDX register) for the ME30 because memory
        //  size does not exceed a single full word length of the MPC_BLK_LUT[n] register. Each bit
        //  in the MPC_BLK_LUT[n] corresponds to a block of memory with the block size given by the
        //  MPC_BLK_CFG register.
        // For example:
        //      - FLASH => 1MB; MPC_FLASH Block size => 32KB.
        //          32KB * 32 (bits in MPC_BLK_LUT[n] register) = 1MB.
        curr_mpc->ctrl &= ~MXC_F_MPC_CTRL_AUTO_INC;

        width = end_block_idx - start_block_idx + 1;

        // Set the security state of each block.
        if (state == MXC_MPC_STATE_NONSECURE) {
            curr_mpc->blk_lut |= (((1 << width) - 1) << start_block_idx);
        } else {
            curr_mpc->blk_lut &= ~(((1 << width) - 1) << start_block_idx);
        }

        // Update curr_addr to start at the beginning of the next MPC region for the next iteration of this for loop.
        curr_addr = MXC_MPC_GET_PHY_MEM_BASE(curr_mpc) + MXC_MPC_GET_PHY_MEM_SIZE(curr_mpc);
    }

    return E_NO_ERROR;
}

int MXC_MPC_CheckPhyBoundaries(uint32_t start_addr, uint32_t end_addr)
{
    uint32_t phy_start_addr, phy_end_addr;

    // Get physical addresses from virtual secure/nonsecure addresses
    //  by clearing bit 28 - indicates the security state of the address.
    phy_start_addr = start_addr & ~(1 << 28);
    phy_end_addr = end_addr & ~(1 << 28);

    // Check Flash Boundaries.
    if ((phy_start_addr >= MXC_PHY_FLASH_MEM_BASE) &&
        (phy_end_addr < (MXC_PHY_FLASH_MEM_BASE + MXC_PHY_FLASH_MEM_SIZE))) {
        return E_NO_ERROR;
    }

    // Check SRAM Boundaries.
    if ((phy_start_addr >= MXC_PHY_SRAM_MEM_BASE) &&
        (phy_end_addr < (MXC_PHY_SRAM_MEM_BASE + MXC_PHY_SRAM_MEM_SIZE))) {
        return E_NO_ERROR;
    }

    // Addresses are located outside of available/usable memory spaces.
    return E_BAD_PARAM;
}

mxc_mpc_regs_t *MXC_MPC_GetInstance(uint32_t addr)
{
    uint32_t phy_addr;

    // Get physical addresses from virtual secure/nonsecure addresses
    //  by clearing bit 28 - indicates the security state of the address.
    phy_addr = addr & ~(1 << 28);

    if ((phy_addr >= MXC_PHY_FLASH_MEM_BASE) &&
        (phy_addr < MXC_PHY_FLASH_MEM_BASE + MXC_PHY_FLASH_MEM_SIZE)) {
        return MXC_MPC_FLASH;
    } else if ((phy_addr >= MXC_PHY_SRAM0_MEM_BASE) &&
               (phy_addr < MXC_PHY_SRAM0_MEM_BASE + MXC_PHY_SRAM0_MEM_SIZE)) {
        return MXC_MPC_SRAM0;
    } else if ((phy_addr >= MXC_PHY_SRAM1_MEM_BASE) &&
               (phy_addr < MXC_PHY_SRAM1_MEM_BASE + MXC_PHY_SRAM1_MEM_SIZE)) {
        return MXC_MPC_SRAM1;
    } else if ((phy_addr >= MXC_PHY_SRAM2_MEM_BASE) &&
               (phy_addr < MXC_PHY_SRAM2_MEM_BASE + MXC_PHY_SRAM2_MEM_SIZE)) {
        return MXC_MPC_SRAM2;
    } else if ((phy_addr >= MXC_PHY_SRAM3_MEM_BASE) &&
               (phy_addr < MXC_PHY_SRAM3_MEM_BASE + MXC_PHY_SRAM3_MEM_SIZE)) {
        return MXC_MPC_SRAM3;
    } else if ((phy_addr >= MXC_PHY_SRAM4_MEM_BASE) &&
               (phy_addr < MXC_PHY_SRAM4_MEM_BASE + MXC_PHY_SRAM4_MEM_SIZE)) {
        return MXC_MPC_SRAM4;
    } else {
        // Addresses are located outside of memory spaces that don't have an
        //  associated MPC.
        return NULL;
    }
}

int MXC_MPC_GetBlockIdx(mxc_mpc_regs_t *mpc, uint32_t addr)
{
    uint32_t block_size;
    uint32_t start_mpc_region_addr, mpc_region_size;
    int base, block;

    if (mpc == NULL) {
        return E_NO_DEVICE;
    }

    start_mpc_region_addr = MXC_MPC_GET_PHY_MEM_BASE(mpc);
    mpc_region_size = MXC_MPC_GET_PHY_MEM_SIZE(mpc);
    if (!((addr >= start_mpc_region_addr) && (addr < start_mpc_region_addr + mpc_region_size))) {
        // Given address is not within the MPC region.
        return E_BAD_PARAM;
    }

    // Block size = 1 << (BLK_CFG.size + 5)
    block_size = 1 << ((mpc->blk_cfg & MXC_F_MPC_BLK_CFG_SIZE) + 5);

    base = addr - start_mpc_region_addr;
    block = base / block_size;

    return block;
}

int MXC_MPC_SetSecure(uint32_t start_addr, uint32_t end_addr)
{
    return MXC_MPC_SetBlockSecurity(start_addr, end_addr, MXC_MPC_STATE_SECURE);
}

int MXC_MPC_SetNonSecure(uint32_t start_addr, uint32_t end_addr)
{
    return MXC_MPC_SetBlockSecurity(start_addr, end_addr, MXC_MPC_STATE_NONSECURE);
}

int MXC_MPC_Lock(mxc_mpc_regs_t *mpc)
{
    mpc->ctrl |= MXC_F_MPC_CTRL_SEC_LOCKDOWN;

    return E_NO_ERROR;
}

#endif
