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
#include "mxc_errors.h"
#include "mxc_sys.h"
#include "tpu.h"
#include "tpu_reva.h"
#include "trng.h"

int MXC_TPU_Init(mxc_sys_periph_clock_t clock)
{
    if (!(MXC_GCR->clk_ctrl & MXC_F_GCR_CLK_CTRL_CRYPTO_EN)) {
        MXC_GCR->clk_ctrl |= MXC_F_GCR_CLK_CTRL_CRYPTO_EN;

        while (!(MXC_GCR->clk_ctrl & MXC_F_GCR_CLK_CTRL_CRYPTO_RDY)) {}
    }

    if (clock == MXC_SYS_PERIPH_CLOCK_TPU) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_TPU);
    } else if (clock == MXC_SYS_PERIPH_CLOCK_TRNG) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_TRNG);
    }
    return E_NO_ERROR;
}

int MXC_TPU_Shutdown(mxc_sys_periph_clock_t clock)
{
    if (clock == MXC_SYS_PERIPH_CLOCK_TPU) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_TPU);
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TPU);
    } else if (clock == MXC_SYS_PERIPH_CLOCK_TRNG) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TRNG);
    }
    return E_NO_ERROR;
}

void MXC_TPU_Reset(void)
{
    MXC_TPU_RevA_Reset((mxc_tpu_reva_regs_t *)MXC_TPU);
}

/* ************************************************************************* */
/* Cyclic Redundancy Check (CRC) functions                                   */
/* ************************************************************************* */

int MXC_TPU_CRC_Config(void)
{
    return MXC_TPU_RevA_CRC_Config((mxc_tpu_reva_regs_t *)MXC_TPU);
}

int MXC_TPU_CRC(const uint8_t *src, uint32_t len, uint32_t poly, uint32_t *crc)
{
    return MXC_TPU_RevA_CRC((mxc_tpu_reva_regs_t *)MXC_TPU, src, len, poly, crc);
}

int MXC_TPU_Ham_Config(void)
{
    return MXC_TPU_RevA_Ham_Config((mxc_tpu_reva_regs_t *)MXC_TPU);
}

int MXC_TPU_Ham(const uint8_t *src, uint32_t len, uint32_t *ecc)
{
    return MXC_TPU_RevA_Ham((mxc_tpu_reva_regs_t *)MXC_TPU, src, len, ecc);
}

/* ************************************************************************* */
/* Cipher functions                                                          */
/* ************************************************************************* */

unsigned int MXC_TPU_Cipher_Get_Key_Size(mxc_tpu_ciphersel_t cipher)
{
    // Key size indexed by 'opsel'
    switch (cipher) {
    case MXC_TPU_CIPHER_DIS:
        return 0;
    case MXC_TPU_CIPHER_AES128:
        return 16;
    case MXC_TPU_CIPHER_AES192:
        return 24;
    case MXC_TPU_CIPHER_AES256:
        return 32;
    case MXC_TPU_CIPHER_DES:
        return 8;
    case MXC_TPU_CIPHER_TDEA:
        return 24;
    }
    // if returns this bad param was passed in or disable.
    return 0;
}

unsigned int MXC_TPU_Cipher_Get_Block_Size(mxc_tpu_ciphersel_t cipher)
{
    switch (cipher) {
    case MXC_TPU_CIPHER_DIS:
        return 0;
    case MXC_TPU_CIPHER_AES128:
        return AES_DATA_LEN;
    case MXC_TPU_CIPHER_AES192:
        return AES_DATA_LEN;
    case MXC_TPU_CIPHER_AES256:
        return AES_DATA_LEN;
    case MXC_TPU_CIPHER_DES:
        return DES_DATA_LEN;
    case MXC_TPU_CIPHER_TDEA:
        return DES_DATA_LEN;
    }
    // if returns this bad param was passed in or disable.
    return 0;
}

unsigned int MXC_TPU_Cipher_GetLength(mxc_tpu_ciphersel_t cipher, unsigned int data_size)
{
    return MXC_TPU_RevA_Cipher_GetLength(cipher, data_size);
}

void MXC_TPU_Cipher_EncDecSelect(int enc)
{
    MXC_TPU_RevA_Cipher_EncDecSelect((mxc_tpu_reva_regs_t *)MXC_TPU, enc);
}

int MXC_TPU_Cipher_Config(mxc_tpu_modesel_t mode, mxc_tpu_ciphersel_t cipher)
{
    return MXC_TPU_RevA_Cipher_Config((mxc_tpu_reva_regs_t *)MXC_TPU, (mxc_tpu_reva_modesel_t)mode,
                                      (mxc_tpu_reva_ciphersel_t)cipher);
}

int MXC_TPU_Cipher_KeySelect(mxc_tpu_keysrc_t key_src)
{
    return MXC_TPU_RevA_Cipher_KeySelect((mxc_tpu_reva_regs_t *)MXC_TPU,
                                         (mxc_tpu_reva_keysrc_t)key_src);
}

int MXC_TPU_Cipher_DoOperation(const char *src, const char *iv, const char *key,
                               mxc_tpu_ciphersel_t cipher, mxc_tpu_modesel_t mode,
                               unsigned int data_size, char *outptr)
{
    return MXC_TPU_RevA_Cipher_DoOperation((mxc_tpu_reva_regs_t *)MXC_TPU, src, iv, key, cipher,
                                           mode, data_size, outptr);
}

int MXC_TPU_Cipher_DES_Encrypt(const char *plaintext, const char *iv, const char *key,
                               mxc_tpu_modesel_t mode, unsigned int data_size, char *outptr)
{
    return MXC_TPU_RevA_Cipher_DES_Encrypt(plaintext, iv, key, mode, data_size, outptr);
}

int MXC_TPU_Cipher_DES_Decrypt(const char *ciphertext, const char *iv, const char *key,
                               mxc_tpu_modesel_t mode, unsigned int data_size, char *outptr)
{
    return MXC_TPU_RevA_Cipher_DES_Decrypt(ciphertext, iv, key, mode, data_size, outptr);
}

int MXC_TPU_Cipher_TDES_Encrypt(const char *plaintext, const char *iv, const char *key,
                                mxc_tpu_modesel_t mode, unsigned int data_size, char *outptr)
{
    return MXC_TPU_RevA_Cipher_TDES_Encrypt(plaintext, iv, key, mode, data_size, outptr);
}

int MXC_TPU_Cipher_TDES_Decrypt(const char *ciphertext, const char *iv, const char *key,
                                mxc_tpu_modesel_t mode, unsigned int data_size, char *outptr)
{
    return MXC_TPU_RevA_Cipher_TDES_Decrypt(ciphertext, iv, key, mode, data_size, outptr);
}

int MXC_TPU_Cipher_AES_Encrypt(const char *plaintext, const char *iv, const char *key,
                               mxc_tpu_ciphersel_t cipher, mxc_tpu_modesel_t mode,
                               unsigned int data_size, char *outptr)
{
    return MXC_TPU_RevA_Cipher_AES_Encrypt(plaintext, iv, key, cipher, mode, data_size, outptr);
}

int MXC_TPU_Cipher_AES_Decrypt(const char *ciphertext, const char *iv, const char *key,
                               mxc_tpu_ciphersel_t cipher, mxc_tpu_modesel_t mode,
                               unsigned int data_size, char *outptr)
{
    return MXC_TPU_RevA_Cipher_AES_Decrypt(ciphertext, iv, key, cipher, mode, data_size, outptr);
}

/* ************************************************************************* */
/* Hash functions                                                            */
/* ************************************************************************* */

unsigned int MXC_TPU_Hash_Get_Block_Size_SHA(mxc_tpu_hashfunsel_t func)
{
    // Block size in bytes indexed by hash function
    switch (func) {
    case MXC_TPU_HASH_DIS:
        return 0;
    case MXC_TPU_HASH_SHA1:
        return 64;
    case MXC_TPU_HASH_SHA224:
        return 64;
    case MXC_TPU_HASH_SHA256:
        return 64;
    case MXC_TPU_HASH_SHA384:
        return 128;
    case MXC_TPU_HASH_SHA512:
        return 128;
    }
    // if returns this bad param was passed in or disable.
    return 0;
}

unsigned int MXC_TPU_Hash_Get_Dgst_Size(mxc_tpu_hashfunsel_t func)
{
    // Digest length in bytes indexed by hash function
    switch (func) {
    case MXC_TPU_HASH_DIS:
        return 0;
    case MXC_TPU_HASH_SHA1:
        return 20;
    case MXC_TPU_HASH_SHA224:
        return 28;
    case MXC_TPU_HASH_SHA256:
        return 32;
    case MXC_TPU_HASH_SHA384:
        return 48;
    case MXC_TPU_HASH_SHA512:
        return 64;
    }
    // if returns this bad param was passed in or disable.
    return 0;
}

void MXC_TPU_Hash_SHA_Size(unsigned int *blocks, unsigned int *length, unsigned int *lbyte,
                           mxc_tpu_hashfunsel_t fun)
{
    MXC_TPU_RevA_Hash_SHA_Size(blocks, length, lbyte, fun);
}

int MXC_TPU_Hash_Config(mxc_tpu_hashfunsel_t func)
{
    return MXC_TPU_RevA_Hash_Config((mxc_tpu_reva_regs_t *)MXC_TPU, func);
}

int MXC_TPU_Hash_SHA(const char *msg, mxc_tpu_hashfunsel_t fun, unsigned int byteLen, char *digest)
{
    return MXC_TPU_RevA_Hash_SHA((mxc_tpu_reva_regs_t *)MXC_TPU, msg, fun, byteLen, digest);
}

/* ************************************************************************* */
/* True Random Number Generator (TRNG) functions                             */
/* ************************************************************************* */

uint8_t MXC_TPU_TRNG_Read8BIT(mxc_trng_regs_t *trng)
{
    uint8_t rand_val;
    MXC_TRNG_Random(&rand_val, 1);
    return rand_val;
}

uint16_t MXC_TPU_TRNG_Read16BIT(mxc_trng_regs_t *trng)
{
    uint16_t rand_val;
    MXC_TRNG_Random((uint8_t *)&rand_val, 2);
    return rand_val;
}

uint32_t MXC_TPU_TRNG_Read32BIT(mxc_trng_regs_t *trng)
{
    return MXC_TRNG_RandomInt();
}

void MXC_TPU_TRNG_Read(mxc_trng_regs_t *trng, uint8_t *data, int len)
{
    MXC_TRNG_Random(data, len);
}

void MXC_TPU_TRNG_Generate_AES(mxc_trng_regs_t *trng)
{
    MXC_TRNG_GenerateKey();
}

/* ************************************************************************* */
/* Modular Arithmetic Accelerator (MAA) functions                             */
/* ************************************************************************* */

void MXC_TPU_MAA_Mem_Clear(void)
{
    MXC_TPU_RevA_MAA_Mem_Clear();
}

void MXC_TPU_MAA_Reset(void)
{
    MXC_TPU_RevA_MAA_Reset((mxc_tpu_reva_regs_t *)MXC_TPU);
}

int MXC_TPU_MAA_Init(unsigned int size)
{
    return MXC_TPU_RevA_MAA_Init((mxc_tpu_reva_regs_t *)MXC_TPU, size);
}

int MXC_TPU_MAA_Shutdown(void)
{
    return MXC_TPU_Shutdown(MXC_SYS_PERIPH_CLOCK_TPU);
}

int MXC_TPU_MAA_Compute(mxc_tpu_maa_clcsel_t clc, char *multiplier, char *multiplicand, char *exp,
                        char *mod, int *result, unsigned int len)
{
    return MXC_TPU_RevA_MAA_Compute((mxc_tpu_reva_regs_t *)MXC_TPU, clc, multiplier, multiplicand,
                                    exp, mod, result, len);
}
