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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_TPU_TPU_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_TPU_TPU_REVA_H_

#include "tpu.h"
#include "tpu_reva_regs.h"
#include "trng_revc_regs.h"

/***************************************************************************************************************
                  DATA STRUCTURES FOR CRYPTO INITIALIZATION
***************************************************************************************************************/
/**
  * Enumeration type for the Crypto Cipher Operation(128/192/256-bit key)
  */
typedef enum {
    MXC_TPU_REVA_CIPHER_DIS = MXC_V_TPU_REVA_CIPHER_CTRL_CIPHER_DIS, // Disable
    MXC_TPU_REVA_CIPHER_AES128 = MXC_V_TPU_REVA_CIPHER_CTRL_CIPHER_AES128, // Select AES-128
    MXC_TPU_REVA_CIPHER_AES192 = MXC_V_TPU_REVA_CIPHER_CTRL_CIPHER_AES192, // Select AES-192
    MXC_TPU_REVA_CIPHER_AES256 = MXC_V_TPU_REVA_CIPHER_CTRL_CIPHER_AES256, // Select AES-256
    MXC_TPU_REVA_CIPHER_DES = MXC_V_TPU_REVA_CIPHER_CTRL_CIPHER_DES, // Select DES
    MXC_TPU_REVA_CIPHER_TDES = MXC_V_TPU_REVA_CIPHER_CTRL_CIPHER_TDES // Select TDEA
} mxc_tpu_reva_ciphersel_t;

/**
  * Enumeration type to select which key is used in Cipher operations
  */
typedef enum {
    MXC_TPU_REVA_KEYSRC_KEY0 =
        MXC_S_TPU_REVA_CIPHER_CTRL_SRC_CIPHERKEY, // Use key in CIPHER_KEY[0:7]
    MXC_TPU_REVA_KEYSRC_KEY2 =
        MXC_S_TPU_REVA_CIPHER_CTRL_SRC_REGFILE, // Use key 0 in AES_KEY registers
    MXC_TPU_REVA_KEYSRC_KEY3 =
        MXC_S_TPU_REVA_CIPHER_CTRL_SRC_QSPIKEY_REGFILE // Use key 1 in AES_KEY registers
} mxc_tpu_reva_keysrc_t;

/**
  * Enumeration type for the Crypto Mode Select
  */
typedef enum {
    MXC_TPU_REVA_MODE_ECB = MXC_V_TPU_REVA_CIPHER_CTRL_MODE_ECB, // Select ECB
    MXC_TPU_REVA_MODE_CBC = MXC_V_TPU_REVA_CIPHER_CTRL_MODE_CBC, // Select CBC
    MXC_TPU_REVA_MODE_CFB = MXC_V_TPU_REVA_CIPHER_CTRL_MODE_CFB, // Select CFB
    MXC_TPU_REVA_MODE_CTR = MXC_V_TPU_REVA_CIPHER_CTRL_MODE_CTR // Select CTR
} mxc_tpu_reva_modesel_t;

/**
  * Enumeration type for Hash function Select
  */
typedef enum {
    MXC_TPU_REVA_HASH_DIS = MXC_V_TPU_REVA_HASH_CTRL_HASH_DIS, // Disable
    MXC_TPU_REVA_HASH_SHA1 = MXC_V_TPU_REVA_HASH_CTRL_HASH_SHA1, // Select SHA1
    MXC_TPU_REVA_HASH_SHA224 = MXC_V_TPU_REVA_HASH_CTRL_HASH_SHA224, // Select SHA224
    MXC_TPU_REVA_HASH_SHA256 = MXC_V_TPU_REVA_HASH_CTRL_HASH_SHA256, // Select SHA256
    MXC_TPU_REVA_HASH_SHA384 = MXC_V_TPU_REVA_HASH_CTRL_HASH_SHA384, // Select SHA384
    MXC_TPU_REVA_HASH_SHA512 = MXC_V_TPU_REVA_HASH_CTRL_HASH_SHA512 // Select SHA384
} mxc_tpu_reva_hashfunsel_t;

/**
  * Enumeration type for MAA initialization
  */
typedef enum {
    MXC_TPU_REVA_MAA_EXP = MXC_V_TPU_REVA_MAA_CTRL_CLC_EXP, // Select exponentiation operation
    MXC_TPU_REVA_MAA_SQ = MXC_V_TPU_REVA_MAA_CTRL_CLC_SQ, // Select square operation
    MXC_TPU_REVA_MAA_MUL = MXC_V_TPU_REVA_MAA_CTRL_CLC_MUL, // Select multiplication operation
    MXC_TPU_REVA_MAA_SQMUL =
        MXC_V_TPU_REVA_MAA_CTRL_CLC_SQMUL, // Select square followed by multiplication operation
    MXC_TPU_REVA_MAA_ADD = MXC_V_TPU_REVA_MAA_CTRL_CLC_ADD, // Select add operation
    MXC_TPU_REVA_MAA_SUB = MXC_V_TPU_REVA_MAA_CTRL_CLC_SUB // Select subtract operation
} mxc_tpu_reva_maa_clcsel_t;

/* ************************************************************************* */
/* Global Control/Configuration functions                                    */
/* ************************************************************************* */

void MXC_TPU_RevA_Clear_Done_Flags(mxc_tpu_reva_regs_t *tpu);
void MXC_TPU_RevA_Reset(mxc_tpu_reva_regs_t *tpu);

/* ************************************************************************* */
/* Cyclic Redundancy Check (CRC) functions                                   */
/* ************************************************************************* */

int MXC_TPU_RevA_CRC_Config(mxc_tpu_reva_regs_t *tpu);
int MXC_TPU_RevA_CRC(mxc_tpu_reva_regs_t *tpu, const uint8_t *src, uint32_t len, uint32_t poly,
                     uint32_t *crc);
int MXC_TPU_RevA_Ham_Config(mxc_tpu_reva_regs_t *tpu);
int MXC_TPU_RevA_Ham(mxc_tpu_reva_regs_t *tpu, const uint8_t *src, uint32_t len, uint32_t *ecc);

/* ************************************************************************* */
/* Cipher functions                                                          */
/* ************************************************************************* */

unsigned int MXC_TPU_RevA_Cipher_GetLength(mxc_tpu_ciphersel_t cipher, unsigned int data_size);
void MXC_TPU_RevA_Cipher_EncDecSelect(mxc_tpu_reva_regs_t *tpu, int enc);
int MXC_TPU_RevA_Cipher_Config(mxc_tpu_reva_regs_t *tpu, mxc_tpu_reva_modesel_t mode,
                               mxc_tpu_reva_ciphersel_t cipher);
int MXC_TPU_RevA_Cipher_KeySelect(mxc_tpu_reva_regs_t *tpu, mxc_tpu_reva_keysrc_t key_src);
int MXC_TPU_RevA_Cipher_DoOperation(mxc_tpu_reva_regs_t *tpu, const char *src, const char *iv,
                                    const char *key, mxc_tpu_ciphersel_t cipher,
                                    mxc_tpu_modesel_t mode, unsigned int data_size, char *outptr);
int MXC_TPU_RevA_Cipher_DES_Encrypt(const char *plaintext, const char *iv, const char *key,
                                    mxc_tpu_modesel_t mode, unsigned int data_size, char *outptr);
int MXC_TPU_RevA_Cipher_DES_Decrypt(const char *ciphertext, const char *iv, const char *key,
                                    mxc_tpu_modesel_t mode, unsigned int data_size, char *outptr);
int MXC_TPU_RevA_Cipher_TDES_Encrypt(const char *plaintext, const char *iv, const char *key,
                                     mxc_tpu_modesel_t mode, unsigned int data_size, char *outptr);
int MXC_TPU_RevA_Cipher_TDES_Decrypt(const char *ciphertext, const char *iv, const char *key,
                                     mxc_tpu_modesel_t mode, unsigned int data_size, char *outptr);
int MXC_TPU_RevA_Cipher_AES_Encrypt(const char *plaintext, const char *iv, const char *key,
                                    mxc_tpu_ciphersel_t cipher, mxc_tpu_modesel_t mode,
                                    unsigned int data_size, char *outptr);
int MXC_TPU_RevA_Cipher_AES_Decrypt(const char *ciphertext, const char *iv, const char *key,
                                    mxc_tpu_ciphersel_t cipher, mxc_tpu_modesel_t mode,
                                    unsigned int data_size, char *outptr);

/* ************************************************************************* */
/* Hash functions                                                            */
/* ************************************************************************* */

void MXC_TPU_RevA_Hash_SHA_Size(unsigned int *blocks, unsigned int *length, unsigned int *lbyte,
                                mxc_tpu_hashfunsel_t fun);
int MXC_TPU_RevA_Hash_Config(mxc_tpu_reva_regs_t *tpu, mxc_tpu_hashfunsel_t func);
int MXC_TPU_RevA_Hash_SHA(mxc_tpu_reva_regs_t *tpu, const char *msg, mxc_tpu_hashfunsel_t fun,
                          unsigned int byteLen, char *digest);

/* ************************************************************************* */
/* True Random Number Generator (TRNG) functions                             */
/* ************************************************************************* */

uint8_t MXC_TPU_RevA_TRNG_Read8BIT(mxc_trng_revc_regs_t *trng);
uint16_t MXC_TPU_RevA_TRNG_Read16BIT(mxc_trng_revc_regs_t *trng);
uint32_t MXC_TPU_RevA_TRNG_Read32BIT(mxc_trng_revc_regs_t *trng);
void MXC_TPU_RevA_TRNG_Read(mxc_trng_revc_regs_t *trng, uint8_t *data, int len);
void MXC_TPU_RevA_TRNG_Generate_AES(mxc_trng_revc_regs_t *trng);

/* ************************************************************************* */
/* Modular Arithmetic Accelerator (MAA) functions                             */
/* ************************************************************************* */

void MXC_TPU_RevA_MAA_Mem_Clear(void);
void MXC_TPU_RevA_MAA_Reset(mxc_tpu_reva_regs_t *tpu);
int MXC_TPU_RevA_MAA_Init(mxc_tpu_reva_regs_t *tpu, unsigned int size);
int MXC_TPU_RevA_MAA_Compute(mxc_tpu_reva_regs_t *tpu, mxc_tpu_maa_clcsel_t clc, char *multiplier,
                             char *multiplicand, char *exp, char *mod, int *result,
                             unsigned int len);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_TPU_TPU_REVA_H_
