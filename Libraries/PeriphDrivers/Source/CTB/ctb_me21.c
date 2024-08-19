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

#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "ctb_reva.h"
#include "ctb_common.h"
#include "trng_regs.h"

/* ************************************************************************* */
/* Global Control/Configuration functions                                    */
/* ************************************************************************* */

/********************************************************/

int MXC_CTB_Init(uint32_t features)
{
    if (features & ~MXC_CTB_FEATURE_TRNG) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_CTB);
    }

    MXC_CTB_RevA_Init((mxc_ctb_reva_regs_t *)MXC_CTB, features);

    if (features & MXC_CTB_FEATURE_TRNG) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_TRNG);
    }

    return E_NO_ERROR;
}

uint32_t MXC_CTB_CheckFeatures(void)
{
    return MXC_CTB_FEATURE_DMA | MXC_CTB_FEATURE_ECC | MXC_CTB_FEATURE_CIPHER |
           MXC_CTB_FEATURE_HASH | MXC_CTB_FEATURE_CRC | MXC_CTB_FEATURE_TRNG;
}

void MXC_CTB_EnableInt(void)
{
    MXC_CTB_RevA_EnableInt((mxc_ctb_reva_regs_t *)MXC_CTB);
}

void MXC_CTB_DisableInt(void)
{
    MXC_CTB_RevA_DisableInt((mxc_ctb_reva_regs_t *)MXC_CTB);
}

int MXC_CTB_Ready(void)
{
    return MXC_CTB_RevA_Ready((mxc_ctb_reva_regs_t *)MXC_CTB);
}

void MXC_CTB_DoneClear(uint32_t features)
{
    MXC_CTB_RevA_DoneClear((mxc_ctb_reva_regs_t *)MXC_CTB, features);
}

uint32_t MXC_CTB_Done(void)
{
    return MXC_CTB_RevA_Done((mxc_ctb_reva_regs_t *)MXC_CTB);
}

void MXC_CTB_Reset(uint32_t features)
{
    MXC_CTB_RevA_Reset(features);
}

void MXC_CTB_CacheInvalidate(void)
{
    MXC_CTB_RevA_CacheInvalidate();
}

int MXC_CTB_Shutdown(uint32_t features)
{
    uint32_t new_features;

    int error = MXC_CTB_RevA_Shutdown(features);

    if (error != E_NO_ERROR) {
        return error;
    }

    new_features = MXC_CTB_GetEnabledFeatures();

    // If CTB is not needed, disable clock
    if ((new_features & ~MXC_CTB_FEATURE_TRNG) == 0) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_CTB);
    }

    // If shutting down TRNG, disable clock
    if (features & MXC_CTB_FEATURE_TRNG) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_TRNG);
    }

    return E_NO_ERROR;
}

uint32_t MXC_CTB_GetEnabledFeatures(void)
{
    return MXC_CTB_RevA_GetEnabledFeatures();
}

void MXC_CTB_Handler(void)
{
    MXC_CTB_RevA_Handler((mxc_trng_reva_regs_t *)MXC_TRNG);
}

/************************************/
/* CTB DMA - Used for all features  */
/************************************/

void MXC_CTB_DMA_SetReadSource(mxc_ctb_dma_read_source_t source)
{
    MXC_CTB_RevA_DMA_SetReadSource((mxc_ctb_reva_regs_t *)MXC_CTB, source);
}

mxc_ctb_dma_read_source_t MXC_CTB_DMA_GetReadSource(void)
{
    return MXC_CTB_RevA_DMA_GetReadSource((mxc_ctb_reva_regs_t *)MXC_CTB);
}

void MXC_CTB_DMA_SetWriteSource(mxc_ctb_dma_write_source_t source)
{
    MXC_CTB_RevA_DMA_SetWriteSource((mxc_ctb_reva_regs_t *)MXC_CTB, source);
}

mxc_ctb_dma_write_source_t MXC_CTB_DMA_GetWriteSource(void)
{
    return MXC_CTB_RevA_DMA_GetWriteSource((mxc_ctb_reva_regs_t *)MXC_CTB);
}

void MXC_CTB_DMA_SetSource(uint8_t *source)
{
    MXC_ASSERT(((uint32_t)source > 0x100FFFFF) || ((uint32_t)source < 0x08000000));
    MXC_CTB_RevA_DMA_SetSource((mxc_ctb_reva_regs_t *)MXC_CTB, source);
}

void MXC_CTB_DMA_SetDestination(uint8_t *dest)
{
    MXC_CTB_RevA_DMA_SetDestination((mxc_ctb_reva_regs_t *)MXC_CTB, dest);
}

int MXC_CTB_DMA_SetupOperation(mxc_ctb_dma_req_t *req)
{
    return MXC_CTB_RevA_DMA_SetupOperation((mxc_ctb_reva_dma_req_t *)req);
}

int MXC_CTB_DMA_DoOperation(mxc_ctb_dma_req_t *req)
{
    return MXC_CTB_RevA_DMA_DoOperation((mxc_ctb_reva_dma_req_t *)req);
}

void MXC_CTB_DMA_StartTransfer(uint32_t length)
{
    MXC_CTB_RevA_DMA_StartTransfer((mxc_ctb_reva_regs_t *)MXC_CTB, length);
}

/* ************************************************************************* */
/* True Random Number Generator (TRNG) functions                             */
/* ************************************************************************* */

int MXC_CTB_TRNG_RandomInt(void)
{
    return MXC_CTB_RevA_TRNG_RandomInt((mxc_trng_reva_regs_t *)MXC_TRNG);
}

int MXC_CTB_TRNG_Random(uint8_t *data, uint32_t len)
{
    return MXC_CTB_RevA_TRNG_Random(data, len);
}

void MXC_CTB_TRNG_RandomAsync(uint8_t *data, uint32_t len, mxc_ctb_complete_cb_t callback)
{
    MXC_CTB_RevA_TRNG_RandomAsync((mxc_trng_reva_regs_t *)MXC_TRNG, data, len, callback);
}

/* ************************************************************************* */
/* Error Correction Code (ECC) functions                                     */
/* ************************************************************************* */

/*******************************/
/* Low Level Functions         */
/*******************************/

void MXC_CTB_ECC_Enable(void)
{
    MXC_CTB_RevA_ECC_Enable((mxc_ctb_reva_regs_t *)MXC_CTB);
}

void MXC_CTB_ECC_Disable(void)
{
    MXC_CTB_RevA_ECC_Disable((mxc_ctb_reva_regs_t *)MXC_CTB);
}

uint32_t MXC_CTB_ECC_GetResult(void)
{
    return MXC_CTB_RevA_ECC_GetResult((mxc_ctb_reva_regs_t *)MXC_CTB);
}

/*******************************/
/* High Level Functions        */
/*******************************/

int MXC_CTB_ECC_Compute(mxc_ctb_ecc_req_t *req)
{
    return MXC_CTB_RevA_ECC_Compute((mxc_ctb_reva_ecc_req_t *)req);
}

int MXC_CTB_ECC_ErrorCheck(mxc_ctb_ecc_req_t *req)
{
    return MXC_CTB_RevA_ECC_ErrorCheck((mxc_ctb_reva_ecc_req_t *)req);
}

void MXC_CTB_ECC_ComputeAsync(mxc_ctb_ecc_req_t *req)
{
    MXC_CTB_RevA_ECC_ComputeAsync((mxc_ctb_reva_ecc_req_t *)req);
}

void MXC_CTB_ECC_ErrorCheckAsync(mxc_ctb_ecc_req_t *req)
{
    MXC_CTB_RevA_ECC_ErrorCheckAsync((mxc_ctb_reva_ecc_req_t *)req);
}

/* ************************************************************************* */
/* Cyclic Redundancy Check (CRC) functions                                   */
/* ************************************************************************* */

/*******************************/
/* Low Level Functions         */
/*******************************/

void MXC_CTB_CRC_SetDirection(mxc_ctb_crc_bitorder_t bitOrder)
{
    MXC_CTB_RevA_CRC_SetDirection((mxc_ctb_reva_regs_t *)MXC_CTB, bitOrder);
}

mxc_ctb_crc_bitorder_t MXC_CTB_CRC_GetDirection(void)
{
    return MXC_CTB_RevA_CRC_GetDirection((mxc_ctb_reva_regs_t *)MXC_CTB);
}

void MXC_CTB_CRC_SetPoly(uint32_t poly)
{
    MXC_CTB_RevA_CRC_SetPoly((mxc_ctb_reva_regs_t *)MXC_CTB, poly);
}

uint32_t MXC_CTB_CRC_GetPoly(void)
{
    return MXC_CTB_RevA_CRC_GetPoly((mxc_ctb_reva_regs_t *)MXC_CTB);
}

uint32_t MXC_CTB_CRC_GetResult(void)
{
    return MXC_CTB_RevA_CRC_GetResult((mxc_ctb_reva_regs_t *)MXC_CTB);
}

void MXC_CTB_CRC_SetInitialValue(uint32_t seed)
{
    MXC_CTB_RevA_CRC_SetInitialValue(seed);
}

void MXC_CTB_CRC_SetFinalXORValue(uint32_t xor)
{
    MXC_CTB_RevA_CRC_SetFinalXORValue(xor);
}

/*******************************/
/* High Level Functions        */
/*******************************/

int MXC_CTB_CRC_Compute(mxc_ctb_crc_req_t *req)
{
    return MXC_CTB_RevA_CRC_Compute((mxc_ctb_reva_regs_t *)MXC_CTB, (mxc_ctb_reva_crc_req_t *)req);
}

void MXC_CTB_CRC_ComputeAsync(mxc_ctb_crc_req_t *req)
{
    MXC_CTB_RevA_CRC_ComputeAsync((mxc_ctb_reva_regs_t *)MXC_CTB, (mxc_ctb_reva_crc_req_t *)req);
}

/* ************************************************************************* */
/* Hash functions                                                            */
/* ************************************************************************* */

/***********************/
/* Low Level Functions */
/***********************/

unsigned int MXC_CTB_Hash_GetBlockSize(mxc_ctb_hash_func_t function)
{
    return MXC_CTB_Common_Hash_GetBlockSize(function);
}

unsigned int MXC_CTB_Hash_GetDigestSize(mxc_ctb_hash_func_t function)
{
    return MXC_CTB_Common_Hash_GetDigestSize(function);
}

void MXC_CTB_Hash_SetFunction(mxc_ctb_hash_func_t function)
{
    MXC_CTB_RevA_Hash_SetFunction((mxc_ctb_reva_regs_t *)MXC_CTB, function);
}

mxc_ctb_hash_func_t MXC_CTB_Hash_GetFunction(void)
{
    return MXC_CTB_RevA_Hash_GetFunction((mxc_ctb_reva_regs_t *)MXC_CTB);
}

void MXC_CTB_Hash_SetAutoPad(int pad)
{
    MXC_CTB_RevA_Hash_SetAutoPad((mxc_ctb_reva_regs_t *)MXC_CTB, pad);
}

int MXC_CTB_Hash_GetAutoPad(void)
{
    return MXC_CTB_RevA_Hash_GetAutoPad((mxc_ctb_reva_regs_t *)MXC_CTB);
}

void MXC_CTB_Hash_GetResult(uint8_t *digest, int *len)
{
    MXC_CTB_RevA_Hash_GetResult((mxc_ctb_reva_regs_t *)MXC_CTB, digest, len);
}

void MXC_CTB_Hash_SetMessageSize(uint32_t size)
{
    MXC_CTB_RevA_Hash_SetMessageSize((mxc_ctb_reva_regs_t *)MXC_CTB, size);
}

void MXC_CTB_Hash_SetSource(mxc_ctb_hash_source_t source)
{
    MXC_CTB_RevA_Hash_SetSource((mxc_ctb_reva_regs_t *)MXC_CTB, source);
}

mxc_ctb_hash_source_t MXC_CTB_Hash_GetSource(void)
{
    return MXC_CTB_RevA_Hash_GetSource((mxc_ctb_reva_regs_t *)MXC_CTB);
}

void MXC_CTB_Hash_InitializeHash(void)
{
    MXC_CTB_RevA_Hash_InitializeHash((mxc_ctb_reva_regs_t *)MXC_CTB);
}

/************************/
/* High Level Functions */
/************************/

int MXC_CTB_Hash_Compute(mxc_ctb_hash_req_t *req)
{
    return MXC_CTB_RevA_Hash_Compute((mxc_ctb_reva_hash_req_t *)req);
}

void MXC_CTB_Hash_ComputeAsync(mxc_ctb_hash_req_t *req)
{
    MXC_CTB_RevA_Hash_ComputeAsync((mxc_ctb_reva_hash_req_t *)req);
}

/* ************************************************************************* */
/* Cipher functions                                                          */
/* ************************************************************************* */

/************************/
/* Low Level Functions  */
/************************/

unsigned int MXC_CTB_Cipher_GetKeySize(mxc_ctb_cipher_t cipher)
{
    return MXC_CTB_Common_Cipher_GetKeySize(cipher);
}

unsigned int MXC_CTB_Cipher_GetBlockSize(mxc_ctb_cipher_t cipher)
{
    return MXC_CTB_Common_Cipher_GetBlockSize(cipher);
}

void MXC_CTB_Cipher_SetMode(mxc_ctb_cipher_mode_t mode)
{
    MXC_CTB_RevA_Cipher_SetMode((mxc_ctb_reva_regs_t *)MXC_CTB, mode);
}

mxc_ctb_cipher_mode_t MXC_CTB_Cipher_GetMode(void)
{
    return MXC_CTB_RevA_Cipher_GetMode((mxc_ctb_reva_regs_t *)MXC_CTB);
}

void MXC_CTB_Cipher_SetCipher(mxc_ctb_cipher_t cipher)
{
    MXC_CTB_RevA_Cipher_SetCipher((mxc_ctb_reva_regs_t *)MXC_CTB, cipher);
}

mxc_ctb_cipher_t MXC_CTB_Cipher_GetCipher(void)
{
    return MXC_CTB_RevA_Cipher_GetCipher((mxc_ctb_reva_regs_t *)MXC_CTB);
}

void MXC_CTB_Cipher_SetKeySource(mxc_ctb_cipher_key_t source)
{
    MXC_CTB_RevA_Cipher_SetKeySource((mxc_ctb_reva_regs_t *)MXC_CTB, source);
}

mxc_ctb_cipher_key_t MXC_CTB_Cipher_GetKeySource(void)
{
    return MXC_CTB_RevA_Cipher_GetKeySource((mxc_ctb_reva_regs_t *)MXC_CTB);
}

void MXC_CTB_Cipher_LoadKey(void)
{
    MXC_CTB_RevA_Cipher_LoadKey((mxc_ctb_reva_regs_t *)MXC_CTB);
}

void MXC_CTB_Cipher_SetOperation(mxc_ctb_cipher_operation_t operation)
{
    MXC_CTB_RevA_Cipher_SetOperation((mxc_ctb_reva_regs_t *)MXC_CTB, operation);
}

void MXC_CTB_Cipher_SetKey(uint8_t *key, uint32_t len)
{
    MXC_CTB_RevA_Cipher_SetKey((mxc_ctb_reva_regs_t *)MXC_CTB, key, len);
}

void MXC_CTB_Cipher_SetIV(uint8_t *iv, uint32_t len)
{
    MXC_CTB_RevA_Cipher_SetIV((mxc_ctb_reva_regs_t *)MXC_CTB, iv, len);
}

void MXC_CTB_Cipher_GetIV(uint8_t *ivOut, uint32_t len)
{
    MXC_CTB_RevA_Cipher_GetIV((mxc_ctb_reva_regs_t *)MXC_CTB, ivOut, len);
}

/************************/
/* High Level Functions */
/************************/

int MXC_CTB_Cipher_Encrypt(mxc_ctb_cipher_req_t *req)
{
    return MXC_CTB_RevA_Cipher_Encrypt((mxc_ctb_reva_cipher_req_t *)req);
}

int MXC_CTB_Cipher_Decrypt(mxc_ctb_cipher_req_t *req)
{
    return MXC_CTB_RevA_Cipher_Decrypt((mxc_ctb_reva_cipher_req_t *)req);
}

void MXC_CTB_Cipher_EncryptAsync(mxc_ctb_cipher_req_t *req)
{
    MXC_CTB_RevA_Cipher_EncryptAsync((mxc_ctb_reva_cipher_req_t *)req);
}

void MXC_CTB_Cipher_DecryptAsync(mxc_ctb_cipher_req_t *req)
{
    MXC_CTB_RevA_Cipher_DecryptAsync((mxc_ctb_reva_cipher_req_t *)req);
}
