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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_CTB_CTB_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_CTB_CTB_REVA_H_

#include "ctb_reva_regs.h"
#include "trng_reva_regs.h"

/* ************************************************************************* */
/* Definitions                                                           */
/* ************************************************************************* */

typedef void (*mxc_ctb_reva_complete_cb_t)(void *req, int result);

typedef enum {
    MXC_CTB_REVA_FEATURE_DMA = 1 << 0,
    MXC_CTB_REVA_FEATURE_ECC = 1 << 1,
    MXC_CTB_REVA_FEATURE_CRC = 1 << 2,
    MXC_CTB_REVA_FEATURE_HASH = 1 << 4,
    MXC_CTB_REVA_FEATURE_CIPHER = 1 << 5,
    MXC_CTB_REVA_FEATURE_TRNG = 1 << 6
} mxc_ctb_reva_features_t;

/* ************************************************************************* */
/* DMA Definitions                                                           */
/* ************************************************************************* */

struct _mxc_ctb_reva_dma_req_t {
    uint8_t *sourceBuffer; ///< pointer to source data
    uint8_t *destBuffer; ///< pointer to destination buffer
    uint32_t length; ///< length of source data
    mxc_ctb_reva_complete_cb_t callback; ///< Null callback indicates a blocking operation
} typedef mxc_ctb_reva_dma_req_t;

typedef enum {
    MXC_CTB_REVA_DMA_READ_FIFO_DMA = MXC_V_CTB_REVA_CTRL_RDSRC_DMAORAPB,
    MXC_CTB_REVA_DMA_READ_FIFO_RNG = MXC_V_CTB_REVA_CTRL_RDSRC_RNG
} mxc_ctb_reva_dma_read_source_t;

typedef enum {
    MXC_CTB_REVA_DMA_WRITE_FIFO_CIPHER = MXC_V_CTB_REVA_CTRL_WRSRC_CIPHEROUTPUT,
    MXC_CTB_REVA_DMA_WRITE_FIFO_READ_FIFO = MXC_V_CTB_REVA_CTRL_WRSRC_READFIFO,
    MXC_CTB_REVA_DMA_WRITE_FIFO_NONE = MXC_V_CTB_REVA_CTRL_WRSRC_NONE
} mxc_ctb_reva_dma_write_source_t;

/* ************************************************************************* */
/* ECC Definitions                                                           */
/* ************************************************************************* */

struct _mxc_ctb_reva_ecc_req_t {
    uint8_t *dataBuffer;
    uint32_t dataLen;
    uint32_t checksum;
    mxc_ctb_reva_complete_cb_t callback;
} typedef mxc_ctb_reva_ecc_req_t;

struct _mxc_ctb_reva_crc_req_t {
    uint8_t *dataBuffer;
    uint32_t dataLen;
    uint32_t resultCRC;
    mxc_ctb_reva_complete_cb_t callback;
} typedef mxc_ctb_reva_crc_req_t;

typedef enum { MXC_CTB_REVA_CRC_LSB_FIRST, MXC_CTB_REVA_CRC_MSB_FIRST } mxc_ctb_reva_crc_bitorder_t;

struct _mxc_ctb_reva_hash_req_t {
    uint8_t *msg;
    uint32_t len;
    uint8_t *hash;
    mxc_ctb_reva_complete_cb_t callback;
} typedef mxc_ctb_reva_hash_req_t;

typedef enum {
    MXC_CTB_REVA_HASH_DIS = MXC_V_CTB_REVA_HASH_CTRL_HASH_DIS, // Disable
    MXC_CTB_REVA_HASH_SHA1 = MXC_V_CTB_REVA_HASH_CTRL_HASH_SHA1, // Select SHA1
    MXC_CTB_REVA_HASH_SHA224 = MXC_V_CTB_REVA_HASH_CTRL_HASH_SHA224, // Select SHA224
    MXC_CTB_REVA_HASH_SHA256 = MXC_V_CTB_REVA_HASH_CTRL_HASH_SHA256, // Select SHA256
    MXC_CTB_REVA_HASH_SHA384 = MXC_V_CTB_REVA_HASH_CTRL_HASH_SHA384, // Select SHA384
    MXC_CTB_REVA_HASH_SHA512 = MXC_V_CTB_REVA_HASH_CTRL_HASH_SHA512 // Select SHA384
} mxc_ctb_reva_hash_func_t;

typedef enum {
    MXC_CTB_REVA_HASH_SOURCE_INFIFO = 0,
    MXC_CTB_REVA_HASH_SOURCE_OUTFIFO = 1
} mxc_ctb_reva_hash_source_t;

/* ************************************************************************* */
/* Cipher Definitions                                                           */
/* ************************************************************************* */

struct _mxc_ctb_reva_cipher_req_t {
    uint8_t *plaintext;
    uint32_t ptLen;
    uint8_t *iv;
    uint8_t *ciphertext;
    mxc_ctb_reva_complete_cb_t callback;
} typedef mxc_ctb_reva_cipher_req_t;

typedef enum {
    MXC_CTB_REVA_MODE_ECB = MXC_V_CTB_REVA_CIPHER_CTRL_MODE_ECB, ///< Electronic Code Book
    MXC_CTB_REVA_MODE_CBC = MXC_V_CTB_REVA_CIPHER_CTRL_MODE_CBC, ///< Cipher Block Chaining
    MXC_CTB_REVA_MODE_CFB = MXC_V_CTB_REVA_CIPHER_CTRL_MODE_CFB, ///< Cipher Feedback
    MXC_CTB_REVA_MODE_CTR = MXC_V_CTB_REVA_CIPHER_CTRL_MODE_CTR, ///< Counter
} mxc_ctb_reva_cipher_mode_t;

typedef enum {
    MXC_CTB_REVA_CIPHER_DIS = MXC_V_CTB_REVA_CIPHER_CTRL_CIPHER_DIS, ///< Disable
    MXC_CTB_REVA_CIPHER_AES128 = MXC_V_CTB_REVA_CIPHER_CTRL_CIPHER_AES128, ///< Select AES-128
    MXC_CTB_REVA_CIPHER_AES192 = MXC_V_CTB_REVA_CIPHER_CTRL_CIPHER_AES192, ///< Select AES-192
    MXC_CTB_REVA_CIPHER_AES256 = MXC_V_CTB_REVA_CIPHER_CTRL_CIPHER_AES256, ///< Select AES-256
    MXC_CTB_REVA_CIPHER_DES = MXC_V_CTB_REVA_CIPHER_CTRL_CIPHER_DES, ///< Select DES
    MXC_CTB_REVA_CIPHER_TDES = MXC_V_CTB_REVA_CIPHER_CTRL_CIPHER_TDES ///< Select TDES
} mxc_ctb_reva_cipher_t;

typedef enum {
    MXC_CTB_REVA_CIPHER_KEY_SOFTWARE = 0,
    MXC_CTB_REVA_CIPHER_KEY_AES_KEY2 = 2,
    MXC_CTB_REVA_CIPHER_KEY_AES_KEY3 = 3
} mxc_ctb_reva_cipher_key_t;

typedef enum {
    MXC_CTB_REVA_CIPHER_ENCRYPTION,
    MXC_CTB_REVA_CIPHER_DECRYPTION
} mxc_ctb_reva_cipher_operation_t;

/* ************************************************************************* */
/* Global Control/Configuration functions                                    */
/* ************************************************************************* */

int MXC_CTB_RevA_Init(mxc_ctb_reva_regs_t *ctb_regs, uint32_t features);
void MXC_CTB_RevA_EnableInt(mxc_ctb_reva_regs_t *ctb_regs);
void MXC_CTB_RevA_DisableInt(mxc_ctb_reva_regs_t *ctb_regs);
int MXC_CTB_RevA_Ready(mxc_ctb_reva_regs_t *ctb_regs);
void MXC_CTB_RevA_DoneClear(mxc_ctb_reva_regs_t *ctb_regs, uint32_t features);
uint32_t MXC_CTB_RevA_Done(mxc_ctb_reva_regs_t *ctb_regs);
void MXC_CTB_RevA_Reset(uint32_t features);
void MXC_CTB_RevA_CacheInvalidate(void);
int MXC_CTB_RevA_Shutdown(uint32_t features);
uint32_t MXC_CTB_RevA_GetEnabledFeatures(void);
void MXC_CTB_RevA_Handler(mxc_trng_reva_regs_t *trng);

/************************************/
/* CTB DMA - Used for all features  */
/************************************/

void MXC_CTB_RevA_DMA_SetReadSource(mxc_ctb_reva_regs_t *ctb_regs,
                                    mxc_ctb_reva_dma_read_source_t source);
mxc_ctb_reva_dma_read_source_t MXC_CTB_RevA_DMA_GetReadSource(mxc_ctb_reva_regs_t *ctb_regs);
void MXC_CTB_RevA_DMA_SetWriteSource(mxc_ctb_reva_regs_t *ctb_regs,
                                     mxc_ctb_reva_dma_write_source_t source);
mxc_ctb_reva_dma_write_source_t MXC_CTB_RevA_DMA_GetWriteSource(mxc_ctb_reva_regs_t *ctb_regs);
void MXC_CTB_RevA_DMA_SetSource(mxc_ctb_reva_regs_t *ctb_regs, uint8_t *source);
void MXC_CTB_RevA_DMA_SetDestination(mxc_ctb_reva_regs_t *ctb_regs, uint8_t *dest);
int MXC_CTB_RevA_DMA_SetupOperation(mxc_ctb_reva_dma_req_t *req);
int MXC_CTB_RevA_DMA_DoOperation(mxc_ctb_reva_dma_req_t *req);
void MXC_CTB_RevA_DMA_StartTransfer(mxc_ctb_reva_regs_t *ctb_regs, uint32_t length);

/* ************************************************************************* */
/* True Random Number Generator (TRNG) functions                             */
/* ************************************************************************* */

int MXC_CTB_RevA_TRNG_RandomInt(mxc_trng_reva_regs_t *trng);
int MXC_CTB_RevA_TRNG_Random(uint8_t *data, uint32_t len);
void MXC_CTB_RevA_TRNG_RandomAsync(mxc_trng_reva_regs_t *trng, uint8_t *data, uint32_t len,
                                   mxc_ctb_reva_complete_cb_t callback);

/* ************************************************************************* */
/* Error Correction Code (ECC) functions                                     */
/* ************************************************************************* */

/*******************************/
/* Low Level Functions         */
/*******************************/

void MXC_CTB_RevA_ECC_Enable(mxc_ctb_reva_regs_t *ctb_regs);
void MXC_CTB_RevA_ECC_Disable(mxc_ctb_reva_regs_t *ctb_regs);
uint32_t MXC_CTB_RevA_ECC_GetResult(mxc_ctb_reva_regs_t *ctb_regs);

/*******************************/
/* High Level Functions        */
/*******************************/

int MXC_CTB_RevA_ECC_Compute(mxc_ctb_reva_ecc_req_t *req);
int MXC_CTB_RevA_ECC_ErrorCheck(mxc_ctb_reva_ecc_req_t *req);
void MXC_CTB_RevA_ECC_ComputeAsync(mxc_ctb_reva_ecc_req_t *req);
void MXC_CTB_RevA_ECC_ErrorCheckAsync(mxc_ctb_reva_ecc_req_t *req);

/* ************************************************************************* */
/* Cyclic Redundancy Check (CRC) functions                                   */
/* ************************************************************************* */

/*******************************/
/* Low Level Functions         */
/*******************************/

void MXC_CTB_RevA_CRC_SetDirection(mxc_ctb_reva_regs_t *ctb_regs,
                                   mxc_ctb_reva_crc_bitorder_t bitOrder);
mxc_ctb_reva_crc_bitorder_t MXC_CTB_RevA_CRC_GetDirection(mxc_ctb_reva_regs_t *ctb_regs);
void MXC_CTB_RevA_CRC_SetPoly(mxc_ctb_reva_regs_t *ctb_regs, uint32_t poly);
uint32_t MXC_CTB_RevA_CRC_GetPoly(mxc_ctb_reva_regs_t *ctb_regs);
uint32_t MXC_CTB_RevA_CRC_GetResult(mxc_ctb_reva_regs_t *ctb_regs);
void MXC_CTB_RevA_CRC_SetInitialValue(uint32_t seed);
void MXC_CTB_RevA_CRC_SetFinalXORValue(uint32_t xor);

/*******************************/
/* High Level Functions        */
/*******************************/

int MXC_CTB_RevA_CRC_Compute(mxc_ctb_reva_regs_t *ctb_regs, mxc_ctb_reva_crc_req_t *req);
void MXC_CTB_RevA_CRC_ComputeAsync(mxc_ctb_reva_regs_t *ctb_regs, mxc_ctb_reva_crc_req_t *req);

/* ************************************************************************* */
/* Hash functions                                                            */
/* ************************************************************************* */

/***********************/
/* Low Level Functions */
/***********************/

void MXC_CTB_RevA_Hash_SetFunction(mxc_ctb_reva_regs_t *ctb_regs,
                                   mxc_ctb_reva_hash_func_t function);
mxc_ctb_reva_hash_func_t MXC_CTB_RevA_Hash_GetFunction(mxc_ctb_reva_regs_t *ctb_regs);
void MXC_CTB_RevA_Hash_SetAutoPad(mxc_ctb_reva_regs_t *ctb_regs, int pad);
int MXC_CTB_RevA_Hash_GetAutoPad(mxc_ctb_reva_regs_t *ctb_regs);
void MXC_CTB_RevA_Hash_GetResult(mxc_ctb_reva_regs_t *ctb_regs, uint8_t *digest, int *len);
void MXC_CTB_RevA_Hash_SetMessageSize(mxc_ctb_reva_regs_t *ctb_regs, uint32_t size);
void MXC_CTB_RevA_Hash_SetSource(mxc_ctb_reva_regs_t *ctb_regs, mxc_ctb_reva_hash_source_t source);
mxc_ctb_reva_hash_source_t MXC_CTB_RevA_Hash_GetSource(mxc_ctb_reva_regs_t *ctb_regs);
void MXC_CTB_RevA_Hash_InitializeHash(mxc_ctb_reva_regs_t *ctb_regs);

/************************/
/* High Level Functions */
/************************/

int MXC_CTB_RevA_Hash_Compute(mxc_ctb_reva_hash_req_t *req);
void MXC_CTB_RevA_Hash_ComputeAsync(mxc_ctb_reva_hash_req_t *req);

/* ************************************************************************* */
/* Cipher functions                                                          */
/* ************************************************************************* */

/************************/
/* Low Level Functions  */
/************************/

void MXC_CTB_RevA_Cipher_SetMode(mxc_ctb_reva_regs_t *ctb_regs, mxc_ctb_reva_cipher_mode_t mode);
mxc_ctb_reva_cipher_mode_t MXC_CTB_RevA_Cipher_GetMode(mxc_ctb_reva_regs_t *ctb_regs);
void MXC_CTB_RevA_Cipher_SetCipher(mxc_ctb_reva_regs_t *ctb_regs, mxc_ctb_reva_cipher_t cipher);
mxc_ctb_reva_cipher_t MXC_CTB_RevA_Cipher_GetCipher(mxc_ctb_reva_regs_t *ctb_regs);
void MXC_CTB_RevA_Cipher_SetKeySource(mxc_ctb_reva_regs_t *ctb_regs,
                                      mxc_ctb_reva_cipher_key_t source);
mxc_ctb_reva_cipher_key_t MXC_CTB_RevA_Cipher_GetKeySource(mxc_ctb_reva_regs_t *ctb_regs);
void MXC_CTB_RevA_Cipher_LoadKey(mxc_ctb_reva_regs_t *ctb_regs);
void MXC_CTB_RevA_Cipher_SetOperation(mxc_ctb_reva_regs_t *ctb_regs,
                                      mxc_ctb_reva_cipher_operation_t operation);
void MXC_CTB_RevA_Cipher_SetKey(mxc_ctb_reva_regs_t *ctb_regs, uint8_t *key, uint32_t len);
void MXC_CTB_RevA_Cipher_SetIV(mxc_ctb_reva_regs_t *ctb_regs, uint8_t *iv, uint32_t len);
void MXC_CTB_RevA_Cipher_GetIV(mxc_ctb_reva_regs_t *ctb_regs, uint8_t *ivOut, uint32_t len);

/************************/
/* High Level Functions */
/************************/

int MXC_CTB_RevA_Cipher_Encrypt(mxc_ctb_reva_cipher_req_t *req);
int MXC_CTB_RevA_Cipher_Decrypt(mxc_ctb_reva_cipher_req_t *req);
void MXC_CTB_RevA_Cipher_EncryptAsync(mxc_ctb_reva_cipher_req_t *req);
void MXC_CTB_RevA_Cipher_DecryptAsync(mxc_ctb_reva_cipher_req_t *req);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_CTB_CTB_REVA_H_
