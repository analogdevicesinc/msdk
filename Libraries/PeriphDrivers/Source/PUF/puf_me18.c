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
#include "ctb.h" // For AES encrypt
#include "flc.h" // For Information Block access
#include "string.h" // For memcpy()

/***** Definitions *****/

#define PUF_KEY0_CHECK_VAL_ADDR (MXC_INFO0_MEM_BASE + 0x1FC0)
#define PUF_KEY1_CHECK_VAL_ADDR (MXC_INFO0_MEM_BASE + 0x1FD0)
#define RAW_USN_ADDR (MXC_INFO0_MEM_BASE + 0x0000)
#define PUF_ERROR_FLAGS                                                                           \
    (MXC_F_PUF_STAT_KEYGEN_EN_ERR | MXC_F_PUF_STAT_KEYGEN_ERR | MXC_F_PUF_STAT_KEY0_INIT_ERR |    \
     MXC_F_PUF_STAT_KEY0_CNST_ERR | MXC_F_PUF_STAT_KEY1_INIT_ERR | MXC_F_PUF_STAT_KEY1_CNST_ERR | \
     MXC_F_PUF_STAT_MAGIC_ERR)
#define PUF_CHECK_VAL_LENGTH 4

#define PUF_GEN_WAIT_MAXIMUM_LOOP_COUNT (IPO_FREQ / 100)

/***** Functions *****/
static void aes256_ecb_oneblock(mxc_ctb_cipher_key_t key, uint8_t *plaintext, uint8_t *ciphertext)
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
    } while (!(ctb_stat & MXC_F_CTB_CTRL_CPH_DONE) && !(ctb_stat & MXC_F_CTB_CTRL_DMA_DONE) &&
             !(ctb_stat & MXC_F_CTB_CTRL_DONE));
}

int MXC_PUF_Generate_Key(mxc_puf_key_t key)
{
    // Try a maximum of 2 times to generate requested PUF key
    int puf_iteration_max = 2;
    int puf_checkval_comparefail = 0;
    int puf_gen_loopcount;
    uint8_t raw_usn[16];
    uint8_t ciphertext[16];
    uint8_t key0_checkval[4];
    uint8_t key1_checkval[4];
    uint8_t *infoblock_key0_checkval = (uint8_t *)PUF_KEY0_CHECK_VAL_ADDR;
    uint8_t *infoblock_key1_checkval = (uint8_t *)PUF_KEY1_CHECK_VAL_ADDR;
    uint8_t *infoblock_rawusn = (uint8_t *)RAW_USN_ADDR;

    // Check for valid key generation selection.
    if ((MXC_PUF_KEY0 != key) && (MXC_PUF_KEY1 != key) && (MXC_PUF_KEY_BOTH != key)) {
        return E_BAD_PARAM;
    }

    int infoblock_fetch_error = 0;
    // ********* Begin Access of Information Block *********
    // Be sure to lock the information block when done.
    MXC_FLC_UnlockInfoBlock(MXC_INFO0_MEM_BASE);
    // Fetch PUF key check values
    // Set error if check value is unprogrammed.
    // Lower 4 bytes should not be all 1, Byte 7 should have bit 7 cleared.
    if ((infoblock_key0_checkval[7] & 0x80) || (infoblock_key1_checkval[7] & 0x80)) {
        infoblock_fetch_error = 1;
    } else {
        memcpy(key0_checkval, infoblock_key0_checkval, PUF_CHECK_VAL_LENGTH);
        memcpy(key1_checkval, infoblock_key1_checkval, PUF_CHECK_VAL_LENGTH);
    }
    // Fetch raw USN
    memcpy(raw_usn, infoblock_rawusn, 16);
    MXC_FLC_LockInfoBlock(MXC_INFO0_MEM_BASE);
    // ********* End Access of Information Block *********

    // If error during infoblock data fetch, return a fatal error.
    if (infoblock_fetch_error) {
        return E_ABORT;
    }

    // Enable PUF peripheral clocks if not already enabled
    if (!(MXC_SYS_IsClockEnabled(MXC_SYS_PERIPH_CLOCK_PUF))) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_PUF);
    }

    for (; puf_iteration_max > 0; puf_iteration_max--) {
        // Reset PUF peripheral
        MXC_SYS_Reset_Periph(MXC_SYS_RESET1_PUF);

        // Trigger key clear.
        MXC_PUF->ctrl |= MXC_F_PUF_CTRL_KEY_CLR_EN;

        // Clear any existing error flags
        MXC_PUF->stat = PUF_ERROR_FLAGS;

        // Setup which keys to generate.
        if ((MXC_PUF_KEY0 == key) || (MXC_PUF_KEY_BOTH == key)) {
            // Clear key gen done bit
            MXC_PUF->stat |= MXC_F_PUF_STAT_KEY0_DN;
            // Enable key gen for this key
            MXC_PUF->ctrl |= MXC_F_PUF_CTRL_KEY0_GEN_EN;
        }
        if ((MXC_PUF_KEY1 == key) || (MXC_PUF_KEY_BOTH == key)) {
            // Clear key gen done bit
            MXC_PUF->stat |= MXC_F_PUF_STAT_KEY1_DN;
            // Enable key gen for this key
            MXC_PUF->ctrl |= MXC_F_PUF_CTRL_KEY1_GEN_EN;
        }

        // Enable PUF peripheral.  Start the key generation.
        MXC_PUF->ctrl |= MXC_F_PUF_CTRL_PUF_EN;
        // Set maximum wait loops for key generation
        puf_gen_loopcount = PUF_GEN_WAIT_MAXIMUM_LOOP_COUNT;
        // Wait for key or keys to be generated.
        if (MXC_PUF_KEY_BOTH == key) {
            // Wait for both keys to complete, or error flags.
            while (((MXC_PUF->stat & (MXC_F_PUF_STAT_KEY0_DN | MXC_F_PUF_STAT_KEY1_DN)) !=
                    (MXC_F_PUF_STAT_KEY0_DN | MXC_F_PUF_STAT_KEY1_DN)) &&
                   !(MXC_PUF->stat & PUF_ERROR_FLAGS)) {
                // Return timeout if PUF key generation exceeds wait loop.
                puf_gen_loopcount--;
                if (puf_gen_loopcount <= 0) {
                    return E_TIME_OUT;
                }
            }
        } else {
            // Wait for one key to complete, or error flags.
            while (!(MXC_PUF->stat & (MXC_F_PUF_STAT_KEY0_DN | MXC_F_PUF_STAT_KEY1_DN)) &&
                   !(MXC_PUF->stat & PUF_ERROR_FLAGS)) {
                // Return timeout if PUF key generation exceeds wait loop.
                puf_gen_loopcount--;
                if (puf_gen_loopcount <= 0) {
                    return E_TIME_OUT;
                }
            }
        }

        // Check for any errors during key generation
        if (MXC_PUF->stat & PUF_ERROR_FLAGS) {
            // On keygen error flags return error
            return E_FAIL;
        }

        MXC_CTB_Init(MXC_CTB_FEATURE_CIPHER);
        MXC_CTB_Cipher_SetMode(MXC_CTB_MODE_ECB);
        MXC_CTB_Cipher_SetCipher(MXC_CTB_CIPHER_AES256);

        // Compare KEY0, KEY1 or both keys against the PUF check value.
        puf_checkval_comparefail = 0;
        if ((MXC_PUF_KEY0 == key) || (MXC_PUF_KEY_BOTH == key)) {
            // OK, now verify key was generated correctly by comparing against a check value.
            // Encrypt first 16 bytes of raw USN with the PUF key.
            // Check 4 bytes of ciphertext against stored PUF key check value.
            aes256_ecb_oneblock(MXC_CTB_CIPHER_KEY_AES_PUFKEY0, raw_usn, ciphertext);

            if (memcmp(key0_checkval, ciphertext, PUF_CHECK_VAL_LENGTH)) {
                // Signal at least one key comparison failed.
                puf_checkval_comparefail = 1;
            }
        } else if ((MXC_PUF_KEY1 == key) || (MXC_PUF_KEY_BOTH == key)) {
            // OK, now verify key was generated correctly by comparing against a check value.
            // Encrypt first 16 bytes of raw USN with the PUF key.
            // Check 4 bytes of ciphertext against stored PUF key check value.
            aes256_ecb_oneblock(MXC_CTB_CIPHER_KEY_AES_PUFKEY1, raw_usn, ciphertext);

            if (memcmp(key1_checkval, ciphertext, PUF_CHECK_VAL_LENGTH)) {
                // Signal at least one key comparison failed.
                puf_checkval_comparefail = 1;
            }
        } else {
            // Invalid key selection.
            return E_BAD_PARAM;
        }

        // If all requested keys have good check values, return immediately, else loop and try PUF key generation again.
        if (!puf_checkval_comparefail) {
            return E_NO_ERROR;
        }

        // If check value comparision fails, fall through and try PUF key generation again, or return failure if too many attempts.
    }

    return E_FAIL;
}

int MXC_PUF_Clear_Keys(void)
{
    // Enable peripheral clocks if not already enabled
    if (!(MXC_SYS_IsClockEnabled(MXC_SYS_PERIPH_CLOCK_PUF))) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_PUF);
    }
    // Reset PUF peripheral
    MXC_SYS_Reset_Periph(MXC_SYS_RESET1_PUF);

    // Trigger key clear.
    MXC_PUF->ctrl |= MXC_F_PUF_CTRL_KEY_CLR_EN;

    return E_NO_ERROR;
}
