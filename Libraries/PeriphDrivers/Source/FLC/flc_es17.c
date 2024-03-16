/**
 * @file flc_es17.h
 * @brief      Flash Controller driver.
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
#include "flc.h"
#include "flc_reva.h"
#include "flc_common.h"
#include "ctb.h"
#include "mcr_regs.h" // For ECCEN registers.
#include "icc_regs.h"

static void mxc_aes_init(void);
static int mxc_encrypt_sequence(const uint8_t *pt, uint8_t *ct, uint32_t addr, int len);

//******************************************************************************
void MXC_FLC_Flash_Operation(void)
{
    volatile uint32_t *line_addr;
    volatile uint32_t __attribute__((unused)) line;

    // Clear the cache and leave the cache enable/disable state unchanged
    MXC_ICC->cache_ctrl ^= MXC_F_ICC_CACHE_CTRL_CACHE_EN;
    MXC_ICC->cache_ctrl ^= MXC_F_ICC_CACHE_CTRL_CACHE_EN;

    // Clear the line fill buffer
    line_addr = (uint32_t *)(MXC_FLASH_MEM_BASE);
    line = *line_addr;
    line_addr = (uint32_t *)(MXC_FLASH_MEM_BASE + MXC_FLASH_PAGE_SIZE);
    line = *line_addr;
}

//******************************************************************************
int MXC_FLC_GetByAddress(mxc_flc_regs_t **flc, uint32_t addr)
{
    if (addr < MXC_FLASH1_MEM_BASE && addr >= MXC_FLASH0_MEM_BASE) {
        *flc = MXC_FLC;
    } else if (addr >= MXC_FLASH1_MEM_BASE && addr < (MXC_FLASH1_MEM_BASE + MXC_FLASH_MEM_SIZE)) {
        *flc = MXC_FLC;
    } else if (addr >= MXC_INFO0_MEM_BASE && addr < (MXC_INFO0_MEM_BASE + MXC_INFO_MEM_SIZE)) {
        *flc = MXC_FLC;
    } else if (addr >= MXC_INFO1_MEM_BASE && addr < (MXC_INFO1_MEM_BASE + MXC_INFO_MEM_SIZE)) {
        *flc = MXC_FLC;
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

//******************************************************************************
int MXC_FLC_GetPhysicalAddress(uint32_t addr, uint32_t *result)
{
    if (addr < MXC_FLASH1_MEM_BASE && addr >= MXC_FLASH0_MEM_BASE) {
        *result = addr & (MXC_FLASH_MEM_SIZE - 1);
    } else if (addr >= MXC_FLASH1_MEM_BASE && addr < (MXC_FLASH1_MEM_BASE + MXC_FLASH_MEM_SIZE)) {
        *result = (addr - MXC_FLASH_MEM_SIZE) & (MXC_FLASH_MEM_SIZE - 1);
    } else if (addr >= MXC_INFO0_MEM_BASE && addr < (MXC_INFO0_MEM_BASE + MXC_INFO_MEM_SIZE)) {
        *result = addr;
    } else if (addr >= MXC_INFO1_MEM_BASE && addr < (MXC_INFO1_MEM_BASE + MXC_INFO_MEM_SIZE)) {
        *result = addr;
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
    mxc_flc_regs_t *flc = NULL;

    // Get FLC Instance
    if ((err = MXC_FLC_GetByAddress(&flc, address)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_FLC_GetPhysicalAddress(address, &addr)) < E_NO_ERROR) {
        return err;
    }

    err = MXC_FLC_RevA_PageErase((mxc_flc_reva_regs_t *)flc, addr);
    // Flush the cache
    MXC_FLC_Flash_Operation();

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

        MXC_FLC_Flash_Operation();
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
    if ((err = MXC_FLC_GetByAddress(&flc, address)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_FLC_GetPhysicalAddress(address, &addr)) < E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_FLC_RevA_Write128((mxc_flc_reva_regs_t *)flc, addr, data)) != E_NO_ERROR) {
        return err;
    }

    // Flush the cache
    MXC_FLC_Flash_Operation();

    if ((err = MXC_FLC_Com_VerifyData(address, 4, data)) != E_NO_ERROR) {
        return err;
    }

    return E_NO_ERROR;
}

//******************************************************************************
int MXC_FLC_Write32(uint32_t address, uint32_t data)
{
    uint32_t addr;
    int err;
    mxc_flc_regs_t *flc = NULL;

    // Address checked if it is byte addressable
    if (address & 0x3) {
        return E_BAD_PARAM;
    }

    // Get FLC Instance
    if ((err = MXC_FLC_GetByAddress(&flc, address)) != E_NO_ERROR) {
        return err;
    }

    if ((err = MXC_FLC_GetPhysicalAddress(address, &addr)) < E_NO_ERROR) {
        return err;
    }

    /* Check if the flash controller is busy */
    if (flc->flsh_cn & (MXC_F_FLC_FLSH_CN_WR | MXC_F_FLC_FLSH_CN_ME | MXC_F_FLC_FLSH_CN_PGE)) {
        return E_BUSY;
    }

    // Set flash clock divider to generate a 1MHz clock from the APB clock
    flc->flsh_clkdiv = SystemCoreClock / 1000000;

    /* Clear stale errors */
    if (flc->flsh_int & MXC_F_FLC_FLSH_INT_AF) {
        flc->flsh_int &= ~MXC_F_FLC_FLSH_INT_AF;
    }

    /* Unlock flash */
    flc->flsh_cn = (flc->flsh_cn & ~MXC_F_FLC_FLSH_CN_UNLOCK) | MXC_S_FLC_FLSH_CN_UNLOCK_UNLOCKED;

    // write 32-bits
    flc->flsh_cn |= MXC_F_FLC_REVA_CTRL_WDTH;
    // write the data
    flc->flsh_addr = addr;
    flc->flsh_data[0] = data;
    flc->flsh_cn |= MXC_F_FLC_FLSH_CN_WR;

    /* Wait until flash operation is complete */
    while (flc->flsh_cn & (MXC_F_FLC_FLSH_CN_WR | MXC_F_FLC_FLSH_CN_ME | MXC_F_FLC_FLSH_CN_PGE)) {}

    /* Lock flash */
    flc->flsh_cn &= ~MXC_F_FLC_FLSH_CN_UNLOCK;

    /* Check access violations */
    if (flc->flsh_int & MXC_F_FLC_FLSH_INT_AF) {
        flc->flsh_int &= ~MXC_F_FLC_FLSH_INT_AF;
        return E_BAD_STATE;
    }

    // Flush the cache
    MXC_FLC_Flash_Operation();

    if ((err = MXC_FLC_Com_VerifyData(address, 1, &data)) != E_NO_ERROR) {
        return err;
    }

    return E_NO_ERROR;
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
int MXC_FLC_UnlockInfoBlock(uint32_t address)
{
    int err;
    mxc_flc_regs_t *flc;

    if ((err = MXC_FLC_GetByAddress(&flc, address)) != E_NO_ERROR) {
        return err;
    }

    return MXC_FLC_RevA_UnlockInfoBlock((mxc_flc_reva_regs_t *)flc, address);
}

//******************************************************************************
int MXC_FLC_LockInfoBlock(uint32_t address)
{
    int err;
    mxc_flc_regs_t *flc;

    if ((err = MXC_FLC_GetByAddress(&flc, address)) != E_NO_ERROR) {
        return err;
    }

    return MXC_FLC_RevA_LockInfoBlock((mxc_flc_reva_regs_t *)flc, address);
}

//******************************************************************************
#if IAR_PRAGMAS
#pragma section = ".flashprog"
#else
__attribute__((section(".flashprog")))
#endif
// make sure to disable ICC with ICC_Disable(); before Running this function
int MXC_FLC_Write_Encrypted(uint32_t address, uint32_t length, uint32_t *buffer)
{
    int err;
    uint32_t encrypted_buffer[4];

    // Address checked if it is 128-bit aligned
    if (address & 0xF) {
        return E_BAD_PARAM;
    }

    // Length checked if it is a multiple of 128-bits.
    if (length & 0x3) {
        return E_BAD_PARAM;
    }

    // Initialize crypto and AES engine.
    mxc_aes_init();

    while (length >= 16) {
        // Encrypt 128-bits of buffer, output to encrypted_buffer
        if ((err = mxc_encrypt_sequence((uint8_t *)buffer, (uint8_t *)encrypted_buffer, address,
                                        16)) != E_NO_ERROR) {
            return err;
        }

        if ((err = MXC_FLC_Write128(address, encrypted_buffer)) != E_NO_ERROR) {
            /* NOTE: FLC_Write128() does not know how to verify writes. */
            /* Ignore E_BAD_STATE errors, we will do our own verify. */
            if (err != E_BAD_STATE) {
                return err;
            }
        }

        /* Verify read data vs. plaintext buffer. */
        if ((err = MXC_FLC_Com_VerifyData(address, 4, buffer)) != E_NO_ERROR) {
            return err;
        }

        address += 16;
        length -= 16;
        buffer += 4;
    }

    return E_NO_ERROR;
}

//******************************************************************************
#if IAR_PRAGMAS
#pragma section = ".flashprog"
#else
__attribute__((section(".flashprog")))
#endif
static void mxc_aes_init(void)
{
    // Make sure CTB clocks are enabled
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_CTB);

    // Reset Crypto block and clear state
    MXC_CTB->crypto_ctrl = MXC_F_CTB_CRYPTO_CTRL_RST;

    // Set the legacy bit
    MXC_CTB->crypto_ctrl |= MXC_F_CTB_CRYPTO_CTRL_FLAG_MODE;

    // Byte swap the input and output
    MXC_CTB->crypto_ctrl |= MXC_F_CTB_CRYPTO_CTRL_BSO;
    MXC_CTB->crypto_ctrl |= MXC_F_CTB_CRYPTO_CTRL_BSI;
}

//******************************************************************************
#if IAR_PRAGMAS
#pragma section = ".flashprog"
#else
__attribute__((section(".flashprog")))
#endif
static int mxc_encrypt_sequence(const uint8_t *pt, uint8_t *ct, uint32_t addr, int len)
{
    uint8_t pt_buf[16];
    uint8_t ct_buf[16];

    int i, j;

    // Check input length for multiple of 16 bytes (one AES block size)
    if (len & 0x0F) {
        return E_BAD_PARAM;
    }

    for (j = 0; j < len; j += 16) {
        // XOR with the address
        uint32_t xor_addr = addr + j;

        for (i = 0; i < 16; i += 4) {
            pt_buf[i + 0] = pt[j + i + 0] ^ (((xor_addr + i) >> 0) & 0xFF);
            pt_buf[i + 1] = pt[j + i + 1] ^ (((xor_addr + i) >> 8) & 0xFF);
            pt_buf[i + 2] = pt[j + i + 2] ^ (((xor_addr + i) >> 16) & 0xFF);
            pt_buf[i + 3] = pt[j + i + 3];
        }

        // Clear interrupt flags
        MXC_CTB->crypto_ctrl |= MXC_F_CTB_CRYPTO_CTRL_CPH_DONE;

        // ECB, AES-128, encrypt
        MXC_CTB->cipher_ctrl =
            ((0x0 << MXC_F_CTB_CIPHER_CTRL_MODE_POS) | (1 << MXC_F_CTB_CIPHER_CTRL_CIPHER_POS) |
             (0 << MXC_F_CTB_CIPHER_CTRL_ENC_POS));

        // Set the key source
        MXC_CTB->cipher_ctrl = ((MXC_CTB->cipher_ctrl & ~MXC_F_CTB_CIPHER_CTRL_SRC) |
                                (0x3 << MXC_F_CTB_CIPHER_CTRL_SRC_POS));

        // Copy data to start the operation
        memcpy((void *)&MXC_CTB->crypto_din[0], (void *)(pt_buf), 16);

        // Wait until operation is complete
        while (!(MXC_CTB->crypto_ctrl & MXC_F_CTB_CRYPTO_CTRL_CPH_DONE)) {}

        // Copy the data out
        memcpy((void *)(ct_buf), (void *)&MXC_CTB->crypto_dout[0], 16);

        // Put the bytes in the ct buffer
        for (i = 0; i < 16; i += 4) {
            ct[j + i + 0] = ct_buf[i + 0];
            ct[j + i + 1] = ct_buf[i + 1];
            ct[j + i + 2] = ct_buf[i + 2];
            ct[j + i + 3] = ct_buf[i + 3];
        }
    }

    return E_NO_ERROR;
}

//******************************************************************************
int MXC_FLC_BlockPageWrite(uint32_t address)
{
    /* MAX32520 does not support flash page read and write locks */
    return E_NOT_SUPPORTED;
}

//******************************************************************************
int MXC_FLC_BlockPageRead(uint32_t address)
{
    /* MAX32520 does not support flash page read and write locks */
    return E_NOT_SUPPORTED;
}

//******************************************************************************
volatile uint32_t *MXC_FLC_GetWELR(uint32_t address, uint32_t page_num)
{
    /* MAX32520 does not support flash page read and write locks */
    return NULL;
}

//******************************************************************************
volatile uint32_t *MXC_FLC_GetRLR(uint32_t address, uint32_t page_num)
{
    /* MAX32520 does not support flash page read and write locks */
    return NULL;
}
