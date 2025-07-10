/******************************************************************************
 *
 * Copyright (C) 2025 Analog Devices, Inc.
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

#include <stdint.h>
#include <stdbool.h>

#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_errors.h"
#include "mxc_sys.h"
#include "puf.h"
#include "puf_regs.h"

// Dependencies outside PUF
#include "ctb.h"  // For AES encrypt
#include "flc.h"  // For Information Block access
#include "string.h"  // For memcpy()
#include "mxc_delay.h"  // For MXC_Delay()

/***** Definitions *****/

#define PUF_KEY0_CHECK_VAL_ADDR (MXC_INFO0_MEM_BASE+0x1FC0)
#define PUF_KEY1_CHECK_VAL_ADDR (MXC_INFO0_MEM_BASE+0x1FD0)
#define RAW_USN_ADDR            (MXC_INFO0_MEM_BASE+0x0000)
#define PUF_ERROR_FLAGS (MXC_F_PUF_STAT_KEYGEN_EN_ERR | MXC_F_PUF_STAT_KEYGEN_ERR | \
                    MXC_F_PUF_STAT_KEY0_INIT_ERR | MXC_F_PUF_STAT_KEY0_CNST_ERR | \
                    MXC_F_PUF_STAT_KEY1_INIT_ERR | MXC_F_PUF_STAT_KEY1_CNST_ERR | \
                    MXC_F_PUF_STAT_MAGIC_ERR)

/***** Functions *****/
static void aes256_ecb_oneblock(mxc_ctb_cipher_key_t key,uint8_t *plaintext, uint8_t *ciphertext)
{
    uint32_t ctb_stat;

    // OK, now verify key was generated correctly by comparing against a check value.
    // Encrypt first 16 bytes of raw USN with the PUF key.
    // Check 4 bytes of ciphertext against stored PUF key check value.
    MXC_CTB_Init(MXC_CTB_FEATURE_CIPHER);
    MXC_CTB_Cipher_SetMode(MXC_CTB_MODE_ECB);
    MXC_CTB_Cipher_SetCipher(MXC_CTB_CIPHER_AES256);

    MXC_CTB_Cipher_SetKeySource(key);
    // Prepare and execute encryption
    mxc_ctb_cipher_req_t aesReq;
    aesReq.plaintext = plaintext;
    aesReq.ptLen = 16;
    aesReq.iv = NULL;
    aesReq.ciphertext = ciphertext;
    MXC_CTB_Cipher_Encrypt(&aesReq);
    do {
        ctb_stat = MXC_CTB->ctrl;
        MXC_Delay(5);
    } while (!(ctb_stat & MXC_F_CTB_CTRL_CPH_DONE) && !(ctb_stat & MXC_F_CTB_CTRL_DMA_DONE) &&
             !(ctb_stat & MXC_F_CTB_CTRL_DONE));
}
        
int MXC_PUF_Generate_Key(mxc_puf_key_t key)
{
    // Try a maximum of 2 times to generate requested PUF key
    int puf_iteration_max = 2;
    uint8_t raw_usn[16];
    uint8_t ciphertext[16];
    uint8_t key0_checkval[4];
    uint8_t key1_checkval[4];
    volatile uint8_t *infoblock_key0_checkval = (volatile uint8_t *)PUF_KEY0_CHECK_VAL_ADDR;
    volatile uint8_t *infoblock_key1_checkval = (volatile uint8_t *)PUF_KEY1_CHECK_VAL_ADDR;
    volatile uint8_t *infoblock_rawusn = (volatile uint8_t *)RAW_USN_ADDR;

    // Check for valid key generation selection.
    if ((MXC_PUF_KEY0 != key) && (MXC_PUF_KEY1 != key) && (MXC_PUF_KEY_BOTH != key))
    {
        return E_BAD_PARAM;
    }

    int infoblock_fetch_error = 0;
    // ********* Begin Access of Information Block *********
    // Be sure to lock the information block when done.
    MXC_FLC_UnlockInfoBlock(MXC_INFO0_MEM_BASE);
    // Fetch PUF key check values
    // Set error if check value is unprogrammed.
    // Lower 4 bytes should not be all 1, Byte 7 should have bit 7 cleared.
    if ((infoblock_key0_checkval[7] & 0x80) || (infoblock_key1_checkval[7] & 0x80))
    {
        infoblock_fetch_error = 1;
    }
    else
    {
        memcpy(key0_checkval,infoblock_key0_checkval,4);
        memcpy(key1_checkval,infoblock_key1_checkval,4);
    }
    // Fetch raw USN
    memcpy(raw_usn,infoblock_rawusn,16);
    MXC_FLC_LockInfoBlock(MXC_INFO0_MEM_BASE);
    // ********* End Access of Information Block *********
#warning FIXME: Re-enable data fetch error check when testing with trimmed parts.
#if 0
    // If error during infoblock data fetch, return a fatal error.
    if (infoblock_fetch_error)
    {
        return E_ABORT;
    }
#endif


    // Enable PUF peripheral clocks if not already enabled
    if (!(MXC_SYS_IsClockEnabled(MXC_SYS_PERIPH_CLOCK_PUF)))
    {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_PUF);
    }

    for (;puf_iteration_max > 0;puf_iteration_max--)
    {
        // Reset PUF peripheral
        MXC_SYS_Reset_Periph(MXC_SYS_RESET1_PUF);

        // Trigger key clear.
        MXC_PUF->ctrl |= MXC_F_PUF_CTRL_KEY_CLR_EN;

        // Clear any existing error flags
        MXC_PUF->stat = PUF_ERROR_FLAGS;

        // Setup which keys to generate.
        if ((MXC_PUF_KEY0 == key) || (MXC_PUF_KEY_BOTH == key))
        {
            // Clear key gen done bit
            MXC_PUF->stat |= MXC_F_PUF_STAT_KEY0_DN;
            // Enable key gen for this key
            MXC_PUF->ctrl |= MXC_F_PUF_CTRL_KEY0_GEN_EN;
        }
        else if ((MXC_PUF_KEY1 == key) || (MXC_PUF_KEY_BOTH == key))
        {
            // Clear key gen done bit
            MXC_PUF->stat |= MXC_F_PUF_STAT_KEY1_DN;
            // Enable key gen for this key
            MXC_PUF->ctrl |= MXC_F_PUF_CTRL_KEY1_GEN_EN;
        }
        else
        {
            return E_BAD_PARAM;
        }

        // Enable PUF peripheral.  Start the key generation.
        MXC_PUF->ctrl |= MXC_F_PUF_CTRL_PUF_EN;
#warning FIXME: Put in something to break out of infinite loop during key gen.  One failure mode has no error flags and no key gen flags, BUSY stays busy forever.
        while (!(MXC_PUF->stat & (MXC_F_PUF_STAT_KEY0_DN | MXC_F_PUF_STAT_KEY1_DN)) && !(MXC_PUF->stat & PUF_ERROR_FLAGS)){};

        // Check for any errors during key generation
        if (MXC_PUF->stat & PUF_ERROR_FLAGS)
        {
#warning FIXME: On keygen error flags, should we try again or return error?
            continue;
            // return E_FAIL;
        }


        MXC_CTB_Init(MXC_CTB_FEATURE_CIPHER);
        MXC_CTB_Cipher_SetMode(MXC_CTB_MODE_ECB);
        MXC_CTB_Cipher_SetCipher(MXC_CTB_CIPHER_AES256);

        // Setup which keys to generate.
        if ((MXC_PUF_KEY0 == key) || (MXC_PUF_KEY_BOTH == key))
        {
            // OK, now verify key was generated correctly by comparing against a check value.
            // Encrypt first 16 bytes of raw USN with the PUF key.
            // Check 4 bytes of ciphertext against stored PUF key check value.
            aes256_ecb_oneblock(MXC_CTB_CIPHER_KEY_AES_PUFKEY0,raw_usn,ciphertext);

#warning FIXME: Check endianness of AES plain/ciphertext and check value.
            if (!memcmp(key0_checkval,ciphertext,4))
            {
                return E_NO_ERROR;
            }
        }
        else if ((MXC_PUF_KEY1 == key) || (MXC_PUF_KEY_BOTH == key))
        {
            // OK, now verify key was generated correctly by comparing against a check value.
            // Encrypt first 16 bytes of raw USN with the PUF key.
            // Check 4 bytes of ciphertext against stored PUF key check value.
            aes256_ecb_oneblock(MXC_CTB_CIPHER_KEY_AES_PUFKEY1,raw_usn,ciphertext);

#warning FIXME: Check endianness of AES plain/ciphertext and check value.
            if (!memcmp(key1_checkval,ciphertext,4))
            {
                return E_NO_ERROR;
            }
        }
        else
        {
            return E_BAD_PARAM;
        }

        // If no match, fall through and try PUF key generation again, or fail if too many attempts.
    }

    return E_FAIL;
}

int MXC_PUF_Clear_Keys(void)
{
    // Enable peripheral clocks if not already enabled
    if (!(MXC_SYS_IsClockEnabled(MXC_SYS_PERIPH_CLOCK_PUF)))
    {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_PUF);
        MXC_SYS_Reset_Periph(MXC_SYS_RESET1_PUF);
    }

    // Enable PUF peripheral if not already enabled.
    if (!(MXC_PUF->ctrl & MXC_F_PUF_CTRL_PUF_EN))
    {
        MXC_PUF->ctrl |= MXC_F_PUF_CTRL_PUF_EN;
    }

    // Trigger key clear.
    MXC_PUF->ctrl |= MXC_F_PUF_CTRL_KEY_CLR_EN;

    return E_NO_ERROR;
}

