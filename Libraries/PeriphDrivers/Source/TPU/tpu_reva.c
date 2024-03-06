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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_sys.h"
#include "tpu.h"
#include "tpu_reva.h"

/* **** Global Data **** */

/* **** Functions **** */

static int MXC_TPU_RevA_TRNG_Read_Status(mxc_trng_revc_regs_t *trng);
/* ************************************************************************ */
/* Prevent GCC from optimimzing this function to memset */
static void __attribute__((optimize("no-tree-loop-distribute-patterns")))
memset32(uint32_t *dst, uint32_t value, unsigned int len)
{
    while (len) {
        *dst = value;
        dst++;
        len -= 4;
    }
}

/* ************************************************************************ */
/* Prevent GCC from optimimzing this function to memcpy */
static void __attribute__((optimize("no-tree-loop-distribute-patterns")))
memcpy32(uint32_t *dst, uint32_t *src, unsigned int len)
{
    while (len) {
        *dst = *src;
        dst++;
        src++;
        len -= 4;
    }
}

/* ************************************************************************* */
/* Global Control/Configuration functions                                    */
/* ************************************************************************* */

// *********************************** Function to Clear all Done Flags ***************************************
void MXC_TPU_RevA_Clear_Done_Flags(mxc_tpu_reva_regs_t *tpu)
{
    // Clear all done flags
    tpu->ctrl |= MXC_F_TPU_REVA_CTRL_DMA_DONE | MXC_F_TPU_REVA_CTRL_GLS_DONE |
                 MXC_F_TPU_REVA_CTRL_HSH_DONE | MXC_F_TPU_REVA_CTRL_CPH_DONE |
                 MXC_F_TPU_REVA_CTRL_MAA_DONE;
}

// ************************************** Function to Clear Crypto Register ************************************
void MXC_TPU_RevA_Reset(mxc_tpu_reva_regs_t *tpu)
{
    MXC_TPU_Init(MXC_SYS_PERIPH_CLOCK_TPU);

    // Reset Crypto Accelerator
    tpu->ctrl = MXC_F_TPU_REVA_CTRL_RST;

    memset32((uint32_t *)tpu, 0, sizeof(mxc_tpu_reva_regs_t));

    // Set the legacy bit so done bits are W1C.
    tpu->ctrl |= MXC_F_TPU_REVA_CTRL_FLAG_MODE;

    MXC_TPU_RevA_Clear_Done_Flags(tpu);
}

/* ************************************************************************* */
/* Cyclic Redundancy Check (CRC) functions                                   */
/* ************************************************************************* */

// ************************************ Function to Configure CRC Algorithm ****************************************
int MXC_TPU_RevA_CRC_Config(mxc_tpu_reva_regs_t *tpu)
{
    // Reset Crypto Block
    MXC_TPU_Reset();

    // Set input and output FIFO modes
    tpu->ctrl |= MXC_S_TPU_REVA_CTRL_RDSRC_DMAORAPB;

    // Enable CRC
    tpu->crc_ctrl = MXC_F_TPU_REVA_CRC_CTRL_CRC_EN;

    return E_SUCCESS;
}

// ************************************* Function to Generate CRC value *******************************************
int MXC_TPU_RevA_CRC(mxc_tpu_reva_regs_t *tpu, const uint8_t *src, uint32_t len, uint32_t poly,
                     uint32_t *crc)
{
    int err;

    // Check if src is pointing to null
    if (src == NULL || crc == NULL) {
        return E_NULL_PTR;
    }

    // Check if data size is null;
    if (len == 0) {
        return E_INVALID;
    }

    err = MXC_TPU_CRC_Config();
    if (err != E_SUCCESS) {
        return err;
    }

    // Load CRC polynomial into crc polynomial register
    tpu->crc_poly = poly;

    // Setup DMA transfer
    tpu->dma_src = (uint32_t)src;
    tpu->dma_cnt = len;

    // Wait for dma done flag
    while (!(tpu->ctrl & MXC_F_TPU_REVA_CTRL_DMA_DONE)) {}

    // Wait until operation is complete
    while (!(tpu->ctrl & MXC_F_TPU_REVA_CTRL_GLS_DONE)) {}

    // Clear gls done flag
    tpu->ctrl |= MXC_F_TPU_REVA_CTRL_GLS_DONE;

    // Store the crc value
    *crc = tpu->crc_val;

    return E_SUCCESS;
}

// ************************************ Function to Configure HAM Algorithm ****************************************
int MXC_TPU_RevA_Ham_Config(mxc_tpu_reva_regs_t *tpu)
{
    // Reset Crypto Block
    MXC_TPU_Reset();

    tpu->ctrl |= MXC_F_TPU_REVA_CTRL_DMADNE_MSK;

    // Set input and output FIFO modes
    tpu->ctrl |= MXC_S_TPU_REVA_CTRL_RDSRC_DMAORAPB;

    // Reset hamming code
    tpu->crc_ctrl = MXC_F_TPU_REVA_CRC_CTRL_HRST;
    while (tpu->crc_ctrl & MXC_F_TPU_REVA_CRC_CTRL_HRST) {}

    // Enable Hamming code
    tpu->crc_ctrl |= MXC_F_TPU_REVA_CRC_CTRL_HAM;

    // Clear all done flags
    MXC_TPU_RevA_Clear_Done_Flags(tpu);

    return E_SUCCESS;
}

// ************************************* Function to Generate ECC value *******************************************
int MXC_TPU_RevA_Ham(mxc_tpu_reva_regs_t *tpu, const uint8_t *src, uint32_t len, uint32_t *ecc)
{
    int err;

    // Check if data is pointing to null
    if (src == NULL || ecc == NULL) {
        return E_NULL_PTR;
    }

    // Check if data size is null;
    if (len == 0) {
        return E_INVALID;
    }

    err = MXC_TPU_RevA_Ham_Config(tpu);
    if (err != E_SUCCESS) {
        return err;
    }

    // Setup DMA transfer
    tpu->dma_src = (uint32_t)src;
    tpu->dma_cnt = len;

    // Wait until operation is complete
    while (!(tpu->ctrl & MXC_F_TPU_REVA_CTRL_GLS_DONE)) {}

    // Wait for dma done flag
    while (!(tpu->ctrl & MXC_F_TPU_REVA_CTRL_DMA_DONE)) {}

    // Clear gls done flag
    tpu->ctrl |= MXC_F_TPU_REVA_CTRL_GLS_DONE;

    // Store the ecc value
    *ecc = tpu->ham_ecc;

    return E_SUCCESS;
}

/* ************************************************************************* */
/* Cipher functions                                                          */
/* ************************************************************************* */

// ************************************* Function to Get Number of Blocks **************************************
unsigned int MXC_TPU_RevA_Cipher_GetLength(mxc_tpu_ciphersel_t cipher, unsigned int data_size)
{
    unsigned int numBlocks, block_size;
    block_size = MXC_TPU_Cipher_Get_Block_Size(cipher);
    numBlocks = data_size / block_size;
    if ((data_size % block_size) > 0) {
        numBlocks++;
    }

    return numBlocks;
}

// ************************ Function to Enable Encrypt/Decrypt Cipher Operation *******************************
void MXC_TPU_RevA_Cipher_EncDecSelect(mxc_tpu_reva_regs_t *tpu, int enc)
{
    // Enable Encryption/Decryption Operation
    if (enc) {
        tpu->cipher_ctrl &= ~MXC_F_TPU_REVA_CIPHER_CTRL_ENC;
    } else {
        tpu->cipher_ctrl |= MXC_F_TPU_REVA_CIPHER_CTRL_ENC;
    }
}

// ******************************* Function to Configure Cipher Operation *************************************
int MXC_TPU_RevA_Cipher_Config(mxc_tpu_reva_regs_t *tpu, mxc_tpu_reva_modesel_t mode,
                               mxc_tpu_reva_ciphersel_t cipher)
{
    // Reset crypto block
    MXC_TPU_Reset();

    // Select cipher mode ECB/CBC/CFB/CTR
    // Select cipher operation AES_128/192/256/DES/TDES
    // Select user cipher key
    tpu->cipher_ctrl = (mode << MXC_F_TPU_REVA_CIPHER_CTRL_MODE_POS) |
                       (cipher << MXC_F_TPU_REVA_CIPHER_CTRL_CIPHER_POS) |
                       MXC_S_TPU_REVA_CIPHER_CTRL_SRC_CIPHERKEY;

    return E_SUCCESS;
}

// ******************************* Function to Select the Source of the Cipher Key *************************************
int MXC_TPU_RevA_Cipher_KeySelect(mxc_tpu_reva_regs_t *tpu, mxc_tpu_reva_keysrc_t key_src)
{
    MXC_SETFIELD(tpu->cipher_ctrl, MXC_F_TPU_REVA_CIPHER_CTRL_SRC, key_src);

    return E_SUCCESS;
}

// ************************************ Function to Test Cipher Algorithm ***********************************
int MXC_TPU_RevA_Cipher_DoOperation(mxc_tpu_reva_regs_t *tpu, const char *src, const char *iv,
                                    const char *key, mxc_tpu_ciphersel_t cipher,
                                    mxc_tpu_modesel_t mode, unsigned int data_size, char *outptr)
{
    unsigned int keyLength, dataLength, numBlocks, i;
    uint32_t key_src = tpu->cipher_ctrl & MXC_F_TPU_REVA_CIPHER_CTRL_SRC;

    if (data_size == 0) {
        return E_INVALID;
    }

    // Check if src, key, iv is a null pointer
    if (src == NULL || (iv == NULL && mode != (mxc_tpu_modesel_t)MXC_TPU_REVA_MODE_ECB)) {
        return E_NULL_PTR;
    } else if (key == NULL &&
               key_src ==
                   MXC_TPU_REVA_KEYSRC_KEY0) { // Key source 0 requires valid key to copy into CIPHER_KEY[0:7]
        return E_NULL_PTR;
    }

    numBlocks = MXC_TPU_Cipher_GetLength(cipher, data_size);

    keyLength = MXC_TPU_Cipher_Get_Key_Size(cipher);
    dataLength = MXC_TPU_Cipher_Get_Block_Size(cipher);

    // If using key source 0, copy key into CIPHER_KEY[0:7]
    if (key_src == MXC_TPU_REVA_KEYSRC_KEY0) {
        memcpy32((void *)&tpu->cipher_key[0], (void *)key, keyLength);
    }

    // Load Initial Vector if necessary
    if (mode != MXC_TPU_MODE_ECB) {
        memcpy32((void *)&tpu->cipher_init[0], (void *)iv, dataLength);
    }

    for (i = 0; i < numBlocks; i++) {
        // Wait until ready for data
        while (!(tpu->ctrl & MXC_F_TPU_REVA_CTRL_RDY)) {}

        // Load plaintext into data in register to start the operation
        memcpy32((void *)&tpu->data_in[0], (void *)src, dataLength);

        // Wait until operation is complete
        while (!(tpu->ctrl & MXC_F_TPU_REVA_CTRL_CPH_DONE)) {}

        // Copy data out
        memcpy32((void *)outptr, (void *)&tpu->data_out[0], dataLength);

        src += dataLength;
        outptr += dataLength;

        // Clear done flag so reading of the next block will be gated properly.
        tpu->ctrl |= MXC_F_TPU_REVA_CTRL_CPH_DONE;
    }

    return E_SUCCESS;
}

int MXC_TPU_RevA_Cipher_DES_Encrypt(const char *plaintext, const char *iv, const char *key,
                                    mxc_tpu_modesel_t mode, unsigned int data_size, char *outptr)
{
    // Enable cipher encrypt/decrypt process
    MXC_TPU_Cipher_EncDecSelect(1);

    // Start the cipher operation
    return MXC_TPU_Cipher_DoOperation(
        plaintext, iv, key, (mxc_tpu_ciphersel_t)MXC_TPU_REVA_CIPHER_DES, mode, data_size, outptr);
}

int MXC_TPU_RevA_Cipher_DES_Decrypt(const char *ciphertext, const char *iv, const char *key,
                                    mxc_tpu_modesel_t mode, unsigned int data_size, char *outptr)
{
    // Enable cipher encrypt/decrypt process
    MXC_TPU_Cipher_EncDecSelect(0);

    // Start the cipher operation
    return MXC_TPU_Cipher_DoOperation(
        ciphertext, iv, key, (mxc_tpu_ciphersel_t)MXC_TPU_REVA_CIPHER_DES, mode, data_size, outptr);
}

int MXC_TPU_RevA_Cipher_TDES_Encrypt(const char *plaintext, const char *iv, const char *key,
                                     mxc_tpu_modesel_t mode, unsigned int data_size, char *outptr)
{
    // Enable cipher encrypt/decrypt process
    MXC_TPU_Cipher_EncDecSelect(1);

    // Start the cipher operation
    return MXC_TPU_Cipher_DoOperation(
        plaintext, iv, key, (mxc_tpu_ciphersel_t)MXC_TPU_REVA_CIPHER_TDES, mode, data_size, outptr);
}

int MXC_TPU_RevA_Cipher_TDES_Decrypt(const char *ciphertext, const char *iv, const char *key,
                                     mxc_tpu_modesel_t mode, unsigned int data_size, char *outptr)
{
    // Enable cipher encrypt/decrypt process
    MXC_TPU_Cipher_EncDecSelect(0);

    // Start the cipher operation
    return MXC_TPU_Cipher_DoOperation(ciphertext, iv, key,
                                      (mxc_tpu_ciphersel_t)MXC_TPU_REVA_CIPHER_TDES, mode,
                                      data_size, outptr);
}

int MXC_TPU_RevA_Cipher_AES_Encrypt(const char *plaintext, const char *iv, const char *key,
                                    mxc_tpu_ciphersel_t cipher, mxc_tpu_modesel_t mode,
                                    unsigned int data_size, char *outptr)
{
    if ((cipher != MXC_TPU_CIPHER_AES128) && (cipher != MXC_TPU_CIPHER_AES192) &&
        (cipher != MXC_TPU_CIPHER_AES256)) {
        return E_BAD_PARAM;
    }

    // Enable cipher encrypt/decrypt process
    MXC_TPU_Cipher_EncDecSelect(1);

    // Start the cipher operation
    return MXC_TPU_Cipher_DoOperation(plaintext, iv, key, cipher, mode, data_size, outptr);
}

int MXC_TPU_RevA_Cipher_AES_Decrypt(const char *ciphertext, const char *iv, const char *key,
                                    mxc_tpu_ciphersel_t cipher, mxc_tpu_modesel_t mode,
                                    unsigned int data_size, char *outptr)
{
    if ((cipher != MXC_TPU_CIPHER_AES128) && (cipher != MXC_TPU_CIPHER_AES192) &&
        (cipher != MXC_TPU_CIPHER_AES256)) {
        return E_BAD_PARAM;
    }

    // Enable cipher encrypt/decrypt process
    MXC_TPU_Cipher_EncDecSelect(0);

    // Start the cipher operation
    return MXC_TPU_Cipher_DoOperation(ciphertext, iv, key, cipher, mode, data_size, outptr);
}

/* ************************************************************************* */
/* Hash functions                                                            */
/* ************************************************************************* */

void MXC_TPU_RevA_Hash_SHA_Size(unsigned int *blocks, unsigned int *length, unsigned int *lbyte,
                                mxc_tpu_hashfunsel_t fun)
{
    unsigned block_size_sha = MXC_TPU_Hash_Get_Block_Size_SHA(fun);

    if (*blocks == 0) {
        // Special case for 0 length message
        *blocks = 1;
        *lbyte = 1;
    } else {
        // Get size of last block of data
        *lbyte = ((*length) - (((*blocks) - 1) * block_size_sha));
    }
}

// ********************************** Function to Configure SHA Algorithm *************************************
int MXC_TPU_RevA_Hash_Config(mxc_tpu_reva_regs_t *tpu, mxc_tpu_hashfunsel_t func)
{
    int funcInt = (int)func;
    if ((funcInt < (int)MXC_V_TPU_REVA_HASH_CTRL_HASH_DIS) ||
        (funcInt > (int)MXC_V_TPU_REVA_HASH_CTRL_HASH_SHA512)) {
        return E_BAD_PARAM;
    }

    // Reset Crypto Block
    MXC_TPU_Reset();

    // Select the Hash Function
    tpu->hash_ctrl = func << MXC_F_TPU_REVA_HASH_CTRL_HASH_POS;

    // Initialize the hash values
    tpu->hash_ctrl |= MXC_F_TPU_REVA_HASH_CTRL_INIT;

    // Wait until operation is complete
    while (tpu->hash_ctrl & MXC_F_TPU_REVA_HASH_CTRL_INIT) {}

    return E_SUCCESS;
}

// ************************************ Function to test SHA Algorithm ****************************************
int MXC_TPU_RevA_Hash_SHA(mxc_tpu_reva_regs_t *tpu, const char *msg, mxc_tpu_hashfunsel_t fun,
                          unsigned int byteLen, char *digest)
{
    unsigned int last_byte = 0;
    unsigned int block, word, numBlocks, block_size_sha, dgst_size;
    int i;
    int err;

    // Check if msg, digest msg is a null pointer
    if (msg == NULL || digest == NULL) {
        return E_NULL_PTR;
    }

    if (byteLen == 0) {
        return E_INVALID;
    }

    err = MXC_TPU_Hash_Config(fun);
    if (err != E_SUCCESS) {
        return err;
    }
    block_size_sha = MXC_TPU_Hash_Get_Block_Size_SHA(fun);
    dgst_size = MXC_TPU_Hash_Get_Dgst_Size(fun);
    // Write message size
    tpu->hash_msg_sz[0] = byteLen;

    // Determine the size of message data without padding
    numBlocks = (byteLen + (block_size_sha - 1)) / block_size_sha;
    MXC_TPU_Hash_SHA_Size(&numBlocks, &byteLen, &last_byte, fun);

    for (block = 0; block < numBlocks; block++) {
        // Clear done flags
        MXC_TPU_RevA_Clear_Done_Flags(tpu);

        // Send data to the crypto data register 32-bits at a time
        if (block != numBlocks - 1) {
            for (i = block * block_size_sha, word = 0; word < block_size_sha; word += 4) {
                // Wait until ready for data
                while (!(tpu->ctrl & MXC_F_TPU_REVA_CTRL_RDY)) {}

                tpu->data_in[0] = (uint32_t)(msg[word + i]) | ((uint32_t)(msg[word + 1 + i]) << 8) |
                                  ((uint32_t)(msg[word + 2 + i]) << 16) |
                                  ((uint32_t)(msg[word + 3 + i]) << 24);
            }
        } else {
            // Determine the current block
            i = block * block_size_sha;

            // Set the last msg bit for auto padding the msg
            tpu->hash_ctrl |= MXC_F_TPU_REVA_HASH_CTRL_LAST;

            // Process the last block
            for (word = 0; word < last_byte; word += 4) {
                // Wait until ready for data
                while (!(tpu->ctrl & MXC_F_TPU_REVA_CTRL_RDY)) {}

                // Send data to the crypto data register
                if (last_byte >= (word + 4)) {
                    tpu->data_in[0] = (uint32_t)(msg[word + i]) +
                                      ((uint32_t)(msg[word + 1 + i]) << 8) +
                                      ((uint32_t)(msg[word + 2 + i]) << 16) +
                                      ((uint32_t)(msg[word + 3 + i]) << 24);
                } else if ((last_byte & 3) == 1) {
                    tpu->data_in[0] = msg[word + i];
                } else if ((last_byte & 3) == 2) {
                    tpu->data_in[0] =
                        (uint32_t)msg[word + i] + ((uint32_t)(msg[word + 1 + i]) << 8);
                } else if ((last_byte & 3) == 3) {
                    tpu->data_in[0] = (uint32_t)msg[word + i] +
                                      ((uint32_t)(msg[word + 1 + i]) << 8) +
                                      ((uint32_t)(msg[word + 2 + i]) << 16);
                }
            }
        }

        // Wait until operation is complete
        while (!(tpu->ctrl & MXC_F_TPU_REVA_CTRL_HSH_DONE)) {}
    }

    // Clear the done flags
    MXC_TPU_RevA_Clear_Done_Flags(tpu);

    // Get the msg digest
    memcpy((uint32_t *)digest, (uint32_t *)&tpu->hash_digest[0], dgst_size);

    return E_SUCCESS;
}

/* ************************************************************************* */
/* True Random Number Generator (TRNG) functions                             */
/* ************************************************************************* */

static int MXC_TPU_RevA_TRNG_Read_Status(mxc_trng_revc_regs_t *trng)
{
    //  Check if a random number is ready to be read
    if (trng->st & MXC_F_TRNG_REVC_ST_RND_RDY) {
        return E_NO_ERROR;
    } else {
        //  Return bad state if TRNG is disabled
        return E_BAD_STATE;
    }
}

uint8_t MXC_TPU_RevA_TRNG_Read8BIT(mxc_trng_revc_regs_t *trng)
{
    //  Wait for TRNG to be ready
    while (MXC_TPU_RevA_TRNG_Read_Status(trng) != E_NO_ERROR) {}

    return ((uint8_t)(trng->data & 0xFF));
    {
    }
}

uint16_t MXC_TPU_RevA_TRNG_Read16BIT(mxc_trng_revc_regs_t *trng)
{
    //  Wait for TRNG to be ready
    while (MXC_TPU_RevA_TRNG_Read_Status(trng) != E_NO_ERROR) {}

    return ((uint16_t)(trng->data & 0xFFFF));
}

uint32_t MXC_TPU_RevA_TRNG_Read32BIT(mxc_trng_revc_regs_t *trng)
{
    //   Wait for TRNG to be ready
    while (MXC_TPU_RevA_TRNG_Read_Status(trng) != E_NO_ERROR) {}

    return ((uint32_t)trng->data);
}

void MXC_TPU_RevA_TRNG_Read(mxc_trng_revc_regs_t *trng, uint8_t *data, int len)
{
    int i = 0;
    uint32_t temp;

    while (len >= 4) {
        temp = MXC_TPU_TRNG_Read32BIT((mxc_trng_regs_t *)trng);
        memcpy(&data[i], &temp, 4);
        i += 4;
        len -= 4;
    }
    while (len) {
        data[i++] = MXC_TPU_TRNG_Read8BIT((mxc_trng_regs_t *)trng);
        len--;
    }
}

void MXC_TPU_RevA_TRNG_Generate_AES(mxc_trng_revc_regs_t *trng)
{
    // Start key generation
    trng->cn |= MXC_F_TRNG_REVC_CN_AESKG_MEU;
    // Field will be auto-set to 0 while key generation is in progress.  Wait for it to complete.
    while ((trng->st & MXC_F_TRNG_REVC_ST_AESKGD_MEU_S) == 0) {}
}

/* ************************************************************************* */
/* Modular Arithmetic Accelerator (MAA) functions                             */
/* ************************************************************************* */

/* **** Definitions **** */
int exponent = 0;

// MAA crypto memory block address
#define MAA_A (MXC_BASE_TPU + 0x100) // MAA_A memory offset
#define MAA_B (MXC_BASE_TPU + 0x200) // MAA_B memory offset
#define MAA_R (MXC_BASE_TPU + 0x300) // MAA_R memory offset
#define MAA_T (MXC_BASE_TPU + 0x400) // MAA_T memory offset
#define MAA_E (MXC_BASE_TPU + 0x500) // MAA_E memory offset
#define MAA_M (MXC_BASE_TPU + 0x600) // MAA_M memory offset

// Define a pointer pointing to different MAA memory location
volatile unsigned int *maa_a = (volatile unsigned int *)MAA_A;
volatile unsigned int *maa_b = (volatile unsigned int *)MAA_B;
volatile unsigned int *maa_r = (volatile unsigned int *)MAA_R;
volatile unsigned int *maa_t = (volatile unsigned int *)MAA_T;
volatile unsigned int *maa_e = (volatile unsigned int *)MAA_E;
volatile unsigned int *maa_m = (volatile unsigned int *)MAA_M;

/* **** Functions **** */
void MXC_TPU_RevA_MAA_Mem_Clear(void)
{
    // Initialize MAA memory
    memset((uint32_t *)MAA_A, 0, MAA_MAX_SIZE);
    memset((uint32_t *)MAA_B, 0, MAA_MAX_SIZE);
    memset((uint32_t *)MAA_R, 0, MAA_MAX_SIZE);
    memset((uint32_t *)MAA_T, 0, MAA_MAX_SIZE);
    memset((uint32_t *)MAA_E, 0, MAA_MAX_SIZE);
    memset((uint32_t *)MAA_M, 0, MAA_MAX_SIZE);
}

void MXC_TPU_RevA_MAA_Reset(mxc_tpu_reva_regs_t *tpu)
{
    // Reset Crypto Accelerator
    tpu->ctrl = MXC_F_TPU_REVA_CTRL_RST;
    memset((uint32_t *)tpu, 0, sizeof(mxc_tpu_reva_regs_t));
    MXC_TPU_MAA_Mem_Clear();
}

int MXC_TPU_RevA_MAA_Init(mxc_tpu_reva_regs_t *tpu, unsigned int size)
{
    int err;

    // Check if size is within the valid range
    if (size > MAA_MAX_WORD_SIZE) {
        return E_BAD_PARAM;
    }

    err = MXC_TPU_Init(MXC_SYS_PERIPH_CLOCK_TPU);
    if (err != E_NO_ERROR) {
        return err;
    }

    // Reset crypto block
    MXC_TPU_MAA_Reset();

    // Set the legacy bit
    tpu->ctrl = MXC_F_TPU_REVA_CTRL_FLAG_MODE;

    // Clear interrupt flags
    tpu->ctrl &= ~MXC_F_TPU_REVA_CTRL_INT;

    // Set MAA word size
    tpu->maa_maws = (size << MXC_F_TPU_REVA_MAA_MAWS_MSGSZ_POS) & MXC_F_TPU_REVA_MAA_MAWS_MSGSZ;

    // Define memory locations for MAA
    tpu->maa_ctrl |=
        (0x6 << MXC_F_TPU_REVA_MAA_CTRL_TMA_POS) | (0x4 << MXC_F_TPU_REVA_MAA_CTRL_RMA_POS) |
        (0x2 << MXC_F_TPU_REVA_MAA_CTRL_BMA_POS) | (0X0 << MXC_F_TPU_REVA_MAA_CTRL_AMA_POS);

    MXC_TPU_RevA_Clear_Done_Flags(tpu);

    return E_SUCCESS;
}

int MXC_TPU_RevA_MAA_Compute(mxc_tpu_reva_regs_t *tpu, mxc_tpu_maa_clcsel_t clc, char *multiplier,
                             char *multiplicand, char *exp, char *mod, int *result,
                             unsigned int len)
{
    unsigned i;

    // Check that we're performing a valid operation
    if (clc >= 0x6) {
        return E_INVALID;
    }

    // Check if exp pointing to NULL for exponent calculation
    if ((clc == MXC_TPU_MAA_EXP) && (exp == NULL)) {
        return E_NULL_PTR;
    }

    // Check if multiplier, multipilicand & mod operands pointing to null
    if (multiplier == NULL || multiplicand == NULL || mod == NULL || result == NULL) {
        return E_NULL_PTR;
    }

    if (len == 0) {
        return E_INVALID;
    }

    // Initialize MAA memory
    MXC_TPU_MAA_Mem_Clear();

    // Copy operands into the memory
    if (clc == MXC_TPU_MAA_EXP) {
        memcpy((void *)MAA_E, (uint32_t *)exp, len);
    }
    memcpy((void *)MAA_A, (uint32_t *)multiplier, len);
    memcpy((void *)MAA_B, (uint32_t *)multiplicand, len);
    memcpy((void *)MAA_M, (uint32_t *)mod, len);

    // Start MAA
    MXC_SETFIELD(tpu->maa_ctrl, MXC_F_TPU_REVA_MAA_CTRL_CLC, clc);
    tpu->maa_ctrl |= MXC_F_TPU_REVA_MAA_CTRL_STC;

    // Check if MAA Error occurs
    if (tpu->maa_ctrl & MXC_F_TPU_REVA_MAA_CTRL_MAAER) {
        return E_BAD_STATE;
    }

    // Wait until operation is complete
    while (!(tpu->ctrl & MXC_F_TPU_REVA_CTRL_MAA_DONE)) {}

    // load the output buffer
    for (i = 0; i < (len / 4); i++) {
        // Copy 4 bytes data
        *(result + i) = (unsigned int)(*(maa_r + i));
    }

    MXC_TPU_RevA_Clear_Done_Flags(tpu);

    return E_SUCCESS;
}
