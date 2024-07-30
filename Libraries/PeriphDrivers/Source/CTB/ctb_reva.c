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

#include <stdlib.h>
#include <string.h>

#include "mxc_sys.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_lock.h"

#include "ctb.h"
#include "ctb_reva.h"
#include "ctb_common.h"

#if (TARGET_NUM != 32572)
#include "icc_reva.h"
#endif

enum { DMA_ID, HSH_ID, CPH_ID, MAA_ID, RNG_ID, NUM_IDS };

typedef enum {
    DMA_CALLBACK_DISABLED = 0,
    DMA_CALLBACK_ECC_Compute = 1,
    DMA_CALLBACK_ECC_Error = 2,
    DMA_CALLBACK_CRC = 3
} mxc_ctb_reva_dma_callback_t;

/***** Global Variables *****/

static void *saved_requests[NUM_IDS];
static mxc_ctb_reva_complete_cb_t MXC_CTB_Callbacks[NUM_IDS];

static mxc_ctb_reva_dma_callback_t dma_cb_func;

static int async_dataLength, async_numBlocks, async_i, async_blockSize, async_lastBlockSize;
static uint32_t TRNG_count, TRNG_maxLength;
static uint8_t *TRNG_data;

static uint32_t enabled_features = 0;

static uint32_t crc_seed = 0xFFFFFFFF;
static uint32_t crc_xor = 0;

/***** Function Prototypes *****/

static int MXC_CTB_ECC_Compare(mxc_ctb_ecc_req_t *req);
static void MXC_CTB_Hash_SendBlock(mxc_ctb_hash_req_t *req);
static int MXC_CTB_Cipher_EncDecAsc(mxc_ctb_cipher_req_t *req);

/* ************************************************************************* */
/* Global Control/Configuration functions                                    */
/* ************************************************************************* */

int MXC_CTB_RevA_Init(mxc_ctb_reva_regs_t *ctb_regs, uint32_t features)
{
    ctb_regs->ctrl = MXC_F_CTB_REVA_CTRL_RST;
    while (ctb_regs->ctrl & MXC_F_CTB_REVA_CTRL_RST) {}

    ctb_regs->ctrl |= MXC_F_CTB_REVA_CTRL_FLAG_MODE;

    enabled_features |= features;

    if (features & MXC_CTB_REVA_FEATURE_DMA) {
        MXC_FreeLock((void *)&MXC_CTB_Callbacks[DMA_ID]);
        dma_cb_func = DMA_CALLBACK_DISABLED;
    }

    if (features & MXC_CTB_REVA_FEATURE_CRC) {
        ctb_regs->crc_ctrl |= MXC_F_CTB_REVA_CRC_CTRL_CRC;
    }

    if (features & MXC_CTB_REVA_FEATURE_HASH) {
        MXC_FreeLock((void *)&MXC_CTB_Callbacks[HSH_ID]);
    } else if (!(enabled_features & MXC_CTB_REVA_FEATURE_HASH)) {
        // Hash has not been initialized previously and is
        // not initialized in this call to MXC_CTB_Init,
        // clear hash_ctrl reg.
        ctb_regs->hash_ctrl = 0;
    }

    if (features & MXC_CTB_REVA_FEATURE_CIPHER) {
        MXC_FreeLock((void *)&MXC_CTB_Callbacks[CPH_ID]);
    }

    if (features & MXC_CTB_REVA_FEATURE_TRNG) {
        MXC_FreeLock((void *)&MXC_CTB_Callbacks[RNG_ID]);
    }

    // Clear done flags
    MXC_CTB_DoneClear(features);

    return E_NO_ERROR;
}

static int MXC_CTB_CheckInterrupts(void)
{
    return ((mxc_ctb_reva_regs_t *)MXC_CTB)->ctrl & MXC_F_CTB_REVA_CTRL_INTR;
}

void MXC_CTB_RevA_EnableInt(mxc_ctb_reva_regs_t *ctb_regs)
{
    // Clear pending interrupts
    MXC_CTB_DoneClear(MXC_CTB_REVA_FEATURE_DMA | MXC_CTB_REVA_FEATURE_ECC |
                      MXC_CTB_REVA_FEATURE_HASH | MXC_CTB_REVA_FEATURE_CRC |
                      MXC_CTB_REVA_FEATURE_CIPHER);

    // Enable device interrupts
    ctb_regs->ctrl |= MXC_F_CTB_REVA_CTRL_INTR;

// Enable IRQ
#ifndef __riscv
    NVIC_EnableIRQ(CRYPTO_IRQn);
#endif
}

void MXC_CTB_RevA_DisableInt(mxc_ctb_reva_regs_t *ctb_regs)
{
// Disable IRQ
#ifndef __riscv
    NVIC_DisableIRQ(CRYPTO_IRQn);
#endif

    // Disable device interrupts
    ctb_regs->ctrl &= ~MXC_F_CTB_REVA_CTRL_INTR;
}

int MXC_CTB_RevA_Ready(mxc_ctb_reva_regs_t *ctb_regs)
{
    return !!(ctb_regs->ctrl & MXC_F_CTB_REVA_CTRL_RDY);
}

void MXC_CTB_RevA_DoneClear(mxc_ctb_reva_regs_t *ctb_regs, uint32_t features)
{
    uint32_t mask = 0;
    uint32_t w1c = (MXC_F_CTB_REVA_CTRL_HSH_DONE | MXC_F_CTB_REVA_CTRL_CPH_DONE |
                    MXC_F_CTB_REVA_CTRL_GLS_DONE);

    if (features & MXC_CTB_REVA_FEATURE_DMA) {
        mask |= MXC_F_CTB_REVA_CTRL_DMA_DONE;
        mask |= MXC_F_CTB_REVA_CTRL_GLS_DONE;
    }

    if (features & MXC_CTB_REVA_FEATURE_HASH) {
        mask |= MXC_F_CTB_REVA_CTRL_HSH_DONE;
    }

    if (features & MXC_CTB_REVA_FEATURE_CIPHER) {
        mask |= MXC_F_CTB_REVA_CTRL_CPH_DONE;
    }

    ctb_regs->ctrl = (ctb_regs->ctrl & ~w1c) | mask;
}

uint32_t MXC_CTB_RevA_Done(mxc_ctb_reva_regs_t *ctb_regs)
{
    uint32_t features = 0;

    if (ctb_regs->ctrl & MXC_F_CTB_REVA_CTRL_DMA_DONE) {
        features |= MXC_CTB_REVA_FEATURE_DMA;
    }

    if (ctb_regs->ctrl & MXC_F_CTB_REVA_CTRL_HSH_DONE) {
        if (ctb_regs->hash_ctrl) { // set flag if only configured
            features |= MXC_CTB_REVA_FEATURE_HASH;
        }
    }

    if (ctb_regs->ctrl & MXC_F_CTB_REVA_CTRL_CPH_DONE) {
        if (ctb_regs->cipher_ctrl) { // set flag if only configured
            features |= MXC_CTB_REVA_FEATURE_CIPHER;
        }
    }

    return features;
}

void MXC_CTB_RevA_Reset(uint32_t features)
{
    MXC_CTB_Shutdown(features);
    MXC_CTB_Init(features);
}

#if (TARGET_NUM != 32572)
void MXC_CTB_RevA_CacheInvalidate(void)
{
    // Write non-zero value to invalidate
    ((mxc_icc_reva_regs_t *)MXC_ICC)->invalidate = 1;

    // Wait for completion of invalidation
    while ((((mxc_icc_reva_regs_t *)MXC_ICC)->ctrl & MXC_F_ICC_REVA_CTRL_RDY) == 0) {}
}
#endif

int MXC_CTB_RevA_Shutdown(uint32_t features)
{
    enabled_features &= ~features;

    if (features & MXC_CTB_REVA_FEATURE_CIPHER) {
        MXC_CTB_RevA_Cipher_SetCipher((mxc_ctb_reva_regs_t *)MXC_CTB, MXC_CTB_REVA_CIPHER_DIS);
    }

    if (features & MXC_CTB_REVA_FEATURE_HASH) {
        MXC_CTB_RevA_Hash_SetFunction((mxc_ctb_reva_regs_t *)MXC_CTB, MXC_CTB_REVA_HASH_DIS);
    }

    if (features & MXC_CTB_REVA_FEATURE_DMA) {
        dma_cb_func = DMA_CALLBACK_DISABLED;
    }

    return E_NO_ERROR;
}

uint32_t MXC_CTB_RevA_GetEnabledFeatures(void)
{
    return enabled_features;
}

void MXC_CTB_RevA_Handler(mxc_trng_reva_regs_t *trng)
{
    void *req;
    uint32_t temp;
    mxc_ctb_reva_complete_cb_t cb;

    uint32_t features = MXC_CTB_Done();

    if (features & MXC_CTB_REVA_FEATURE_HASH) {
        req = saved_requests[HSH_ID];
        MXC_CTB_DoneClear(MXC_CTB_REVA_FEATURE_HASH | MXC_CTB_REVA_FEATURE_DMA |
                          MXC_CTB_REVA_FEATURE_CIPHER);
        features &= ~(MXC_CTB_REVA_FEATURE_HASH | MXC_CTB_REVA_FEATURE_CIPHER);

        async_i++;

        if (async_i != async_numBlocks) {
            MXC_CTB_Hash_SendBlock((mxc_ctb_hash_req_t *)req);
        } else {
            MXC_CTB_Hash_GetResult(((mxc_ctb_hash_req_t *)req)->hash, (int *)&temp);
            cb = MXC_CTB_Callbacks[HSH_ID];
            MXC_FreeLock((void *)&MXC_CTB_Callbacks[HSH_ID]);
            cb(req, 0);
        }
    }

    if (features & MXC_CTB_REVA_FEATURE_CIPHER) {
        req = saved_requests[CPH_ID];
        MXC_CTB_DoneClear(MXC_CTB_REVA_FEATURE_CIPHER | MXC_CTB_REVA_FEATURE_DMA);
        features &= ~MXC_CTB_REVA_FEATURE_CIPHER;

        if (MXC_CTB_Cipher_EncDecAsc(req)) {
            cb = MXC_CTB_Callbacks[CPH_ID];
            MXC_FreeLock((void *)&MXC_CTB_Callbacks[CPH_ID]);
            cb(req, 0);
        }
    }

    if (features & MXC_CTB_REVA_FEATURE_DMA) {
        req = saved_requests[DMA_ID];
        MXC_CTB_DoneClear(MXC_CTB_REVA_FEATURE_DMA);

        switch (dma_cb_func) {
        case DMA_CALLBACK_ECC_Compute:
            ((mxc_ctb_reva_ecc_req_t *)req)->checksum = MXC_CTB_ECC_GetResult();
            cb = MXC_CTB_Callbacks[DMA_ID];
            MXC_FreeLock((void *)&MXC_CTB_Callbacks[DMA_ID]);
            cb(req, 0);
            break;

        case DMA_CALLBACK_ECC_Error:
            temp = MXC_CTB_ECC_Compare((mxc_ctb_ecc_req_t *)req);
            cb = MXC_CTB_Callbacks[DMA_ID];
            MXC_FreeLock((void *)&MXC_CTB_Callbacks[DMA_ID]);
            cb(req, temp);
            break;

        case DMA_CALLBACK_CRC:
            ((mxc_ctb_reva_crc_req_t *)req)->resultCRC = MXC_CTB_CRC_GetResult();
            cb = MXC_CTB_Callbacks[DMA_ID];
            MXC_FreeLock((void *)&MXC_CTB_Callbacks[DMA_ID]);
            cb(req, 0);
            break;

        case DMA_CALLBACK_DISABLED:
            // Another function is in progress, do nothing
            break;
        }
    }

    if ((features == 0) && MXC_CTB_Callbacks[RNG_ID]) { /* interrupt caused by TRNG */
        // if this is last block, disable interrupt before reading MXC_TRNG->data
        if (TRNG_maxLength <= TRNG_count + 4) {
            trng->ctrl &= ~MXC_F_TRNG_REVA_CTRL_RND_IE;
        }

        temp = trng->data;

        if ((TRNG_count + 3) < TRNG_maxLength) {
            memcpy(&(TRNG_data[TRNG_count]), (uint8_t *)(&temp), 4);
            TRNG_count += 4;
        } else {
            memcpy(&(TRNG_data[TRNG_count]), (uint8_t *)(&temp), TRNG_maxLength & 0x03);
            TRNG_count += (TRNG_maxLength & 0x03);
        }

        if (TRNG_maxLength == TRNG_count) {
            cb = MXC_CTB_Callbacks[RNG_ID];
            MXC_FreeLock((void *)&MXC_CTB_Callbacks[RNG_ID]);
            cb(0, 0);
        }
    }
}

/************************************/
/* CTB DMA - Used for all features  */
/************************************/

void MXC_CTB_RevA_DMA_SetReadSource(mxc_ctb_reva_regs_t *ctb_regs,
                                    mxc_ctb_reva_dma_read_source_t source)
{
    MXC_SETFIELD(ctb_regs->ctrl, MXC_F_CTB_REVA_CTRL_RDSRC,
                 source << MXC_F_CTB_REVA_CTRL_RDSRC_POS);
}

mxc_ctb_reva_dma_read_source_t MXC_CTB_RevA_DMA_GetReadSource(mxc_ctb_reva_regs_t *ctb_regs)
{
    return (mxc_ctb_reva_dma_read_source_t)((ctb_regs->ctrl & MXC_F_CTB_REVA_CTRL_RDSRC) >>
                                            MXC_F_CTB_REVA_CTRL_RDSRC_POS);
}

void MXC_CTB_RevA_DMA_SetWriteSource(mxc_ctb_reva_regs_t *ctb_regs,
                                     mxc_ctb_reva_dma_write_source_t source)
{
    MXC_SETFIELD(ctb_regs->ctrl, MXC_F_CTB_REVA_CTRL_WRSRC,
                 source << MXC_F_CTB_REVA_CTRL_WRSRC_POS);
}

mxc_ctb_reva_dma_write_source_t MXC_CTB_RevA_DMA_GetWriteSource(mxc_ctb_reva_regs_t *ctb_regs)
{
    return (mxc_ctb_reva_dma_write_source_t)((ctb_regs->ctrl & MXC_F_CTB_REVA_CTRL_WRSRC) >>
                                             MXC_F_CTB_REVA_CTRL_WRSRC_POS);
}

void MXC_CTB_RevA_DMA_SetSource(mxc_ctb_reva_regs_t *ctb_regs, uint8_t *source)
{
    ctb_regs->dma_src = (uint32_t)source;
}

void MXC_CTB_RevA_DMA_SetDestination(mxc_ctb_reva_regs_t *ctb_regs, uint8_t *dest)
{
    ctb_regs->dma_dest = (uint32_t)dest;
}

void MXC_CTB_RevA_DMA_StartTransfer(mxc_ctb_reva_regs_t *ctb_regs, uint32_t length)
{
    ctb_regs->dma_cnt = length;
}

int MXC_CTB_RevA_DMA_SetupOperation(mxc_ctb_reva_dma_req_t *req)
{
    if (req == NULL) {
        return E_NULL_PTR;
    }

    MXC_CTB_DMA_SetReadSource((mxc_ctb_dma_read_source_t)MXC_CTB_REVA_DMA_READ_FIFO_DMA);

    if (req->destBuffer == NULL) {
        MXC_CTB_DMA_SetWriteSource((mxc_ctb_dma_write_source_t)MXC_CTB_REVA_DMA_WRITE_FIFO_NONE);
    } else {
        MXC_CTB_DMA_SetWriteSource((mxc_ctb_dma_write_source_t)MXC_CTB_REVA_DMA_WRITE_FIFO_CIPHER);
    }

    MXC_CTB_DMA_SetSource(req->sourceBuffer);
    MXC_CTB_DMA_SetDestination(req->destBuffer);

    return E_NO_ERROR;
}

int MXC_CTB_RevA_DMA_DoOperation(mxc_ctb_reva_dma_req_t *req)
{
    if (req == NULL) {
        return E_NULL_PTR;
    }

    MXC_CTB_DMA_SetupOperation((mxc_ctb_dma_req_t *)req);

    MXC_CTB_DMA_StartTransfer(req->length);

    while (!(MXC_CTB_Done() & MXC_CTB_REVA_FEATURE_DMA)) {}

    MXC_CTB_DoneClear(MXC_CTB_REVA_FEATURE_DMA);

    return E_NO_ERROR;
}

/* ************************************************************************* */
/* True Random Number Generator(TRNG) functions                             */
/* ************************************************************************* */

int MXC_CTB_RevA_TRNG_RandomInt(mxc_trng_reva_regs_t *trng)
{
    while (!(trng->status & MXC_F_TRNG_REVA_STATUS_RDY)) {}

    return (int)trng->data;
}

int MXC_CTB_RevA_TRNG_Random(uint8_t *data, uint32_t len)
{
    unsigned int i, temp;

    if (data == NULL) {
        return E_NULL_PTR;
    }

    for (i = 0; i + 3 < len; i += 4) {
        temp = MXC_CTB_TRNG_RandomInt();
        memcpy(&(data[i]), (uint8_t *)(&temp), 4);
    }

    if (len & 0x03) {
        temp = MXC_CTB_TRNG_RandomInt();
        memcpy(&(data[i]), (uint8_t *)(&temp), len & 0x03);
    }

    return E_NO_ERROR;
}

void MXC_CTB_RevA_TRNG_RandomAsync(mxc_trng_reva_regs_t *trng, uint8_t *data, uint32_t len,
                                   mxc_ctb_reva_complete_cb_t callback)
{
    MXC_ASSERT(data && callback);

    if (len == 0) {
        return;
    }

    while (MXC_GetLock((void *)&MXC_CTB_Callbacks[RNG_ID], 1) != E_NO_ERROR) {}

#ifndef __riscv
    NVIC_DisableIRQ(TRNG_IRQn);
#endif

    TRNG_data = data;
    TRNG_count = 0;
    TRNG_maxLength = len;
    MXC_CTB_Callbacks[RNG_ID] = callback;

    // Enable interrupts
    trng->ctrl |= MXC_F_TRNG_REVA_CTRL_RND_IE;

#ifndef __riscv
    NVIC_EnableIRQ(TRNG_IRQn);
#endif
}

/* ************************************************************************* */
/* Error Correction Code(ECC) functions                                     */
/* ************************************************************************* */

/*******************************/
/* Low Level Functions         */
/*******************************/

void MXC_CTB_RevA_ECC_Enable(mxc_ctb_reva_regs_t *ctb_regs)
{
    ctb_regs->crc_ctrl |= MXC_F_CTB_REVA_CRC_CTRL_HRST;

    while (ctb_regs->crc_ctrl & MXC_F_CTB_REVA_CRC_CTRL_HRST) {}

    ctb_regs->crc_ctrl |= MXC_F_CTB_REVA_CRC_CTRL_HAM;
}

void MXC_CTB_RevA_ECC_Disable(mxc_ctb_reva_regs_t *ctb_regs)
{
    ctb_regs->crc_ctrl &= ~MXC_F_CTB_REVA_CRC_CTRL_HAM;
}

uint32_t MXC_CTB_RevA_ECC_GetResult(mxc_ctb_reva_regs_t *ctb_regs)
{
    // Include bit 16 to aid error correction
    return ctb_regs->ham_ecc;
}

/*******************************/
/* High Level Functions        */
/*******************************/

static int MXC_CTB_ECC_Compare(mxc_ctb_ecc_req_t *req)
{
    if (req == NULL) {
        return E_NULL_PTR;
    }

    uint32_t new_ecc = MXC_CTB_ECC_GetResult();

    uint32_t error = req->checksum ^ new_ecc;

    if (error & (1 << 16)) {
        // Correct single bit error
        // Location of failing bit is NOT of XOR
        uint16_t failing_bit_location = ~error;

        int byte = failing_bit_location >> 3;
        int bit = failing_bit_location & 0x7;

        req->dataBuffer[byte] ^= (1 << bit);

        return 1;

    } else if (error) {
        // Double bit error is not correctable with ECC
        return 2;
    }

    return E_NO_ERROR;
}

static int MXC_CTB_ECC_Setup(mxc_ctb_ecc_req_t *req)
{
    mxc_ctb_reva_dma_req_t dma_req;
    int enabled;

    if (req == NULL) {
        return E_NULL_PTR;
    }

    if (req->dataBuffer == NULL) {
        return E_NULL_PTR;
    }

    if (req->dataLen == 0) {
        return E_BAD_PARAM;
    }

    enabled = MXC_CTB_CheckInterrupts();
    MXC_CTB_DisableInt();

    MXC_CTB_ECC_Enable();

    dma_req.sourceBuffer = req->dataBuffer;
    dma_req.length = req->dataLen;

    MXC_CTB_DMA_DoOperation((mxc_ctb_dma_req_t *)&dma_req);

    if (enabled) {
        MXC_CTB_EnableInt();
    }

    return E_NO_ERROR;
}

int MXC_CTB_RevA_ECC_Compute(mxc_ctb_reva_ecc_req_t *req)
{
    uint32_t error = MXC_CTB_ECC_Setup((mxc_ctb_ecc_req_t *)req);

    if (error != E_NO_ERROR) {
        return error;
    }

    req->checksum = MXC_CTB_ECC_GetResult();
    return E_NO_ERROR;
}

int MXC_CTB_RevA_ECC_ErrorCheck(mxc_ctb_reva_ecc_req_t *req)
{
    uint32_t error = MXC_CTB_ECC_Setup((mxc_ctb_ecc_req_t *)req);

    if (error != E_NO_ERROR) {
        return error;
    }

    return MXC_CTB_ECC_Compare((mxc_ctb_ecc_req_t *)req);
}

static void MXC_CTB_ECC_SetupAsync(mxc_ctb_ecc_req_t *req)
{
    mxc_ctb_reva_dma_req_t dma_req;

    MXC_ASSERT(req);
    MXC_ASSERT(req->callback && req->dataBuffer);

    while (MXC_GetLock((void *)&MXC_CTB_Callbacks[DMA_ID], 1) != E_NO_ERROR) {}

    MXC_CTB_DisableInt();

    MXC_CTB_Callbacks[DMA_ID] = req->callback;
    saved_requests[DMA_ID] = req;

    dma_req.sourceBuffer = req->dataBuffer;

    MXC_CTB_DMA_SetupOperation((mxc_ctb_dma_req_t *)&dma_req);

    MXC_CTB_ECC_Enable();
    MXC_CTB_EnableInt();
}

void MXC_CTB_RevA_ECC_ComputeAsync(mxc_ctb_reva_ecc_req_t *req)
{
    MXC_CTB_ECC_SetupAsync((mxc_ctb_ecc_req_t *)req);

    dma_cb_func = DMA_CALLBACK_ECC_Compute;
    MXC_CTB_DMA_StartTransfer(req->dataLen);
}

void MXC_CTB_RevA_ECC_ErrorCheckAsync(mxc_ctb_reva_ecc_req_t *req)
{
    MXC_CTB_ECC_SetupAsync((mxc_ctb_ecc_req_t *)req);

    dma_cb_func = DMA_CALLBACK_ECC_Error;
    MXC_CTB_DMA_StartTransfer(req->dataLen);
}

/* ************************************************************************* */
/* Cyclic Redundancy Check(CRC) functions                                   */
/* ************************************************************************* */

/*******************************/
/* Low Level Functions         */
/*******************************/

void MXC_CTB_RevA_CRC_SetDirection(mxc_ctb_reva_regs_t *ctb_regs,
                                   mxc_ctb_reva_crc_bitorder_t bitOrder)
{
    MXC_SETFIELD(ctb_regs->crc_ctrl, MXC_F_CTB_REVA_CRC_CTRL_MSB,
                 bitOrder << MXC_F_CTB_REVA_CRC_CTRL_MSB_POS);
}

mxc_ctb_reva_crc_bitorder_t MXC_CTB_RevA_CRC_GetDirection(mxc_ctb_reva_regs_t *ctb_regs)
{
    return (mxc_ctb_reva_crc_bitorder_t)((ctb_regs->crc_ctrl & MXC_F_CTB_REVA_CRC_CTRL_MSB) >>
                                         MXC_F_CTB_REVA_CRC_CTRL_MSB_POS);
}

void MXC_CTB_RevA_CRC_SetPoly(mxc_ctb_reva_regs_t *ctb_regs, uint32_t poly)
{
    ctb_regs->crc_poly = poly;
}

uint32_t MXC_CTB_RevA_CRC_GetPoly(mxc_ctb_reva_regs_t *ctb_regs)
{
    return ctb_regs->crc_poly;
}

uint32_t MXC_CTB_RevA_CRC_GetResult(mxc_ctb_reva_regs_t *ctb_regs)
{
    return ctb_regs->crc_val ^ crc_xor;
}

void MXC_CTB_RevA_CRC_SetInitialValue(uint32_t seed)
{
    crc_seed = seed;
}

void MXC_CTB_RevA_CRC_SetFinalXORValue(uint32_t xor)
{
    crc_xor = xor;
}

/*******************************/
/* High Level Functions        */
/*******************************/

int MXC_CTB_RevA_CRC_Compute(mxc_ctb_reva_regs_t *ctb_regs, mxc_ctb_reva_crc_req_t *req)
{
    mxc_ctb_reva_dma_req_t dma_req;
    int enabled;

    if (req == NULL) {
        return E_NULL_PTR;
    }

    enabled = MXC_CTB_CheckInterrupts();
    MXC_CTB_DisableInt();

    ctb_regs->crc_val = crc_seed; // Preset CRC value to all 1's

    dma_req.sourceBuffer = req->dataBuffer;
    dma_req.destBuffer = NULL;
    dma_req.length = req->dataLen;

    MXC_CTB_DMA_DoOperation((mxc_ctb_dma_req_t *)&dma_req);

    // Store the crc value
    req->resultCRC = MXC_CTB_CRC_GetResult();

    if (enabled) {
        MXC_CTB_EnableInt();
    }

    return E_SUCCESS;
}

void MXC_CTB_RevA_CRC_ComputeAsync(mxc_ctb_reva_regs_t *ctb_regs, mxc_ctb_reva_crc_req_t *req)
{
    mxc_ctb_reva_dma_req_t dma_req;

    MXC_ASSERT(req);
    MXC_ASSERT(req->callback);

    while (MXC_GetLock((void *)&MXC_CTB_Callbacks[DMA_ID], 1) != E_NO_ERROR) {}

    MXC_CTB_DisableInt();

    MXC_CTB_Callbacks[DMA_ID] = req->callback;
    saved_requests[DMA_ID] = req;
    dma_cb_func = DMA_CALLBACK_CRC;

    ctb_regs->crc_val = crc_seed; // Preset CRC value to all 1's

    dma_req.sourceBuffer = req->dataBuffer;
    dma_req.destBuffer = NULL;

    MXC_CTB_DMA_SetupOperation((mxc_ctb_dma_req_t *)&dma_req);

    MXC_CTB_EnableInt();

    MXC_CTB_DMA_StartTransfer(req->dataLen);
}

/* ************************************************************************* */
/* Hash functions                                                            */
/* ************************************************************************* */

/***********************/
/* Low Level Functions */
/***********************/

void MXC_CTB_RevA_Hash_SetFunction(mxc_ctb_reva_regs_t *ctb_regs, mxc_ctb_reva_hash_func_t function)
{
    MXC_SETFIELD(ctb_regs->hash_ctrl, MXC_F_CTB_REVA_HASH_CTRL_HASH,
                 function << MXC_F_CTB_REVA_HASH_CTRL_HASH_POS);
}

mxc_ctb_reva_hash_func_t MXC_CTB_RevA_Hash_GetFunction(mxc_ctb_reva_regs_t *ctb_regs)
{
    return (mxc_ctb_reva_hash_func_t)((ctb_regs->hash_ctrl & MXC_F_CTB_REVA_HASH_CTRL_HASH) >>
                                      MXC_F_CTB_REVA_HASH_CTRL_HASH_POS);
}

void MXC_CTB_RevA_Hash_SetAutoPad(mxc_ctb_reva_regs_t *ctb_regs, int pad)
{
    MXC_SETFIELD(ctb_regs->hash_ctrl, MXC_F_CTB_REVA_HASH_CTRL_LAST,
                 (!!pad) << MXC_F_CTB_REVA_HASH_CTRL_LAST_POS);
}

int MXC_CTB_RevA_Hash_GetAutoPad(mxc_ctb_reva_regs_t *ctb_regs)
{
    return !!(ctb_regs->hash_ctrl & MXC_F_CTB_REVA_HASH_CTRL_LAST);
}

void MXC_CTB_RevA_Hash_GetResult(mxc_ctb_reva_regs_t *ctb_regs, uint8_t *digest, int *len)
{
    *len = MXC_CTB_Hash_GetDigestSize(MXC_CTB_Hash_GetFunction());
    memcpy(digest, (uint8_t *)&ctb_regs->hash_digest[0], *len);
}

void MXC_CTB_RevA_Hash_SetMessageSize(mxc_ctb_reva_regs_t *ctb_regs, uint32_t size)
{
    ctb_regs->hash_msg_sz[0] = size;
}

void MXC_CTB_RevA_Hash_SetSource(mxc_ctb_reva_regs_t *ctb_regs, mxc_ctb_reva_hash_source_t source)
{
    MXC_SETFIELD(ctb_regs->ctrl, MXC_F_CTB_REVA_CTRL_SRC,
                 (!!source) << MXC_F_CTB_REVA_CTRL_SRC_POS);
}

mxc_ctb_reva_hash_source_t MXC_CTB_RevA_Hash_GetSource(mxc_ctb_reva_regs_t *ctb_regs)
{
    return (mxc_ctb_reva_hash_source_t)((ctb_regs->ctrl & MXC_F_CTB_REVA_CTRL_SRC) >>
                                        MXC_F_CTB_REVA_CTRL_SRC_POS);
}

void MXC_CTB_RevA_Hash_InitializeHash(mxc_ctb_reva_regs_t *ctb_regs)
{
    ctb_regs->hash_ctrl |= MXC_F_CTB_REVA_HASH_CTRL_INIT;

    while (ctb_regs->hash_ctrl & MXC_F_CTB_REVA_HASH_CTRL_INIT) {}
}

/************************/
/* High Level Functions */
/************************/

int MXC_CTB_RevA_Hash_Compute(mxc_ctb_reva_hash_req_t *req)
{
    int i, block, numBlocks, blockSize, lastBlockSize;
    mxc_ctb_reva_dma_req_t dma_req;

    if (req == NULL) {
        return E_NULL_PTR;
    }

    if (!(req->msg && req->hash)) {
        return E_NULL_PTR;
    }

    if (req->len <= 0) {
        return E_INVALID;
    }

    int enabled = MXC_CTB_CheckInterrupts();
    MXC_CTB_DisableInt();

    MXC_CTB_Hash_SetMessageSize(req->len);
    MXC_CTB_Hash_SetSource((mxc_ctb_hash_source_t)MXC_CTB_REVA_HASH_SOURCE_INFIFO);
    MXC_CTB_Hash_InitializeHash();

    blockSize = MXC_CTB_Hash_GetBlockSize(MXC_CTB_Hash_GetFunction());
    numBlocks = ((int)req->len - 1) / blockSize + 1;
    lastBlockSize = req->len % blockSize;

    if (lastBlockSize == 0) {
        lastBlockSize = blockSize;
    }

    dma_req.sourceBuffer = req->msg;

    MXC_CTB_DMA_SetupOperation((mxc_ctb_dma_req_t *)&dma_req);

    for (block = 0; block < numBlocks; block++) {
        if (block != numBlocks - 1) {
            // Send data to the crypto data register 32-bits at a time
            MXC_CTB_DMA_StartTransfer(blockSize);

        } else {
            // Set the last req->msg bit for auto padding the msg
            MXC_CTB_Hash_SetAutoPad(1);

            MXC_CTB_DMA_StartTransfer(lastBlockSize);
        }

        // Wait until operation is complete
        while (!(MXC_CTB_Done() & MXC_CTB_REVA_FEATURE_HASH)) {}
    }

    // Get the msg digest
    // Note: i is throwaway
    MXC_CTB_Hash_GetResult(req->hash, &i);

    if (enabled) {
        MXC_CTB_EnableInt();
    }

    return E_NO_ERROR;
}

static void MXC_CTB_Hash_SendBlock(mxc_ctb_hash_req_t *req)
{
    if (async_i != async_numBlocks - 1) {
        MXC_CTB_DMA_StartTransfer(async_blockSize);
    } else {
        // Set the last bit for auto padding the msg
        MXC_CTB_Hash_SetAutoPad(1);

        MXC_CTB_DMA_StartTransfer(async_lastBlockSize);
    }
}

void MXC_CTB_RevA_Hash_ComputeAsync(mxc_ctb_reva_hash_req_t *req)
{
    mxc_ctb_reva_dma_req_t dma_req;

    MXC_ASSERT(req);
    MXC_ASSERT(req->msg && req->hash && req->callback);

    if (req->len <= 0) {
        return;
    }

    while (MXC_GetLock((void *)&MXC_CTB_Callbacks[HSH_ID], 1) != E_NO_ERROR) {}

    MXC_CTB_DisableInt();

    MXC_CTB_Callbacks[HSH_ID] = req->callback;
    saved_requests[HSH_ID] = req;

    MXC_CTB_Hash_SetMessageSize(req->len);
    MXC_CTB_Hash_SetSource((mxc_ctb_hash_source_t)MXC_CTB_REVA_HASH_SOURCE_INFIFO);
    MXC_CTB_Hash_InitializeHash();

    async_blockSize = MXC_CTB_Hash_GetBlockSize(MXC_CTB_Hash_GetFunction());
    async_numBlocks = ((int)req->len - 1) / async_blockSize + 1;
    async_lastBlockSize = req->len % async_blockSize;
    async_i = 0;

    if (async_lastBlockSize == 0) {
        async_lastBlockSize = async_blockSize;
    }

    dma_req.sourceBuffer = req->msg;
    MXC_CTB_DMA_SetupOperation((mxc_ctb_dma_req_t *)&dma_req);

    MXC_CTB_EnableInt();

    MXC_CTB_Hash_SendBlock((mxc_ctb_hash_req_t *)req);
}

/* ************************************************************************* */
/* Cipher functions                                                          */
/* ************************************************************************* */

/************************/
/* Low Level Functions  */
/************************/

void MXC_CTB_RevA_Cipher_SetMode(mxc_ctb_reva_regs_t *ctb_regs, mxc_ctb_reva_cipher_mode_t mode)
{
    MXC_SETFIELD(ctb_regs->cipher_ctrl, MXC_F_CTB_REVA_CIPHER_CTRL_MODE,
                 mode << MXC_F_CTB_REVA_CIPHER_CTRL_MODE_POS);
}

mxc_ctb_reva_cipher_mode_t MXC_CTB_RevA_Cipher_GetMode(mxc_ctb_reva_regs_t *ctb_regs)
{
    return (mxc_ctb_reva_cipher_mode_t)((ctb_regs->cipher_ctrl & MXC_F_CTB_REVA_CIPHER_CTRL_MODE) >>
                                        MXC_F_CTB_REVA_CIPHER_CTRL_MODE_POS);
}

void MXC_CTB_RevA_Cipher_SetCipher(mxc_ctb_reva_regs_t *ctb_regs, mxc_ctb_reva_cipher_t cipher)
{
    MXC_SETFIELD(ctb_regs->cipher_ctrl, MXC_F_CTB_REVA_CIPHER_CTRL_CIPHER,
                 cipher << MXC_F_CTB_REVA_CIPHER_CTRL_CIPHER_POS);
}

mxc_ctb_reva_cipher_t MXC_CTB_RevA_Cipher_GetCipher(mxc_ctb_reva_regs_t *ctb_regs)
{
    return (mxc_ctb_reva_cipher_t)((ctb_regs->cipher_ctrl & MXC_F_CTB_REVA_CIPHER_CTRL_CIPHER) >>
                                   MXC_F_CTB_REVA_CIPHER_CTRL_CIPHER_POS);
}

void MXC_CTB_RevA_Cipher_SetKeySource(mxc_ctb_reva_regs_t *ctb_regs,
                                      mxc_ctb_reva_cipher_key_t source)
{
    MXC_SETFIELD(ctb_regs->cipher_ctrl, MXC_F_CTB_REVA_CIPHER_CTRL_SRC,
                 source << MXC_F_CTB_REVA_CIPHER_CTRL_SRC_POS);
}

mxc_ctb_reva_cipher_key_t MXC_CTB_RevA_Cipher_GetKeySource(mxc_ctb_reva_regs_t *ctb_regs)
{
    return (mxc_ctb_reva_cipher_key_t)((ctb_regs->cipher_ctrl & MXC_F_CTB_REVA_CIPHER_CTRL_SRC) >>
                                       MXC_F_CTB_REVA_CIPHER_CTRL_SRC_POS);
}

void MXC_CTB_RevA_Cipher_LoadKey(mxc_ctb_reva_regs_t *ctb_regs)
{
    if ((mxc_ctb_reva_cipher_key_t)MXC_CTB_Cipher_GetKeySource() ==
        MXC_CTB_REVA_CIPHER_KEY_SOFTWARE) {
        return;
    }

    ctb_regs->cipher_ctrl |= MXC_F_CTB_REVA_CIPHER_CTRL_KEY;
}

void MXC_CTB_RevA_Cipher_SetOperation(mxc_ctb_reva_regs_t *ctb_regs,
                                      mxc_ctb_reva_cipher_operation_t operation)
{
    MXC_SETFIELD(ctb_regs->cipher_ctrl, MXC_F_CTB_REVA_CIPHER_CTRL_ENC,
                 operation << MXC_F_CTB_REVA_CIPHER_CTRL_ENC_POS);
}

void MXC_CTB_RevA_Cipher_SetKey(mxc_ctb_reva_regs_t *ctb_regs, uint8_t *key, uint32_t len)
{
    if ((mxc_ctb_reva_cipher_key_t)MXC_CTB_Cipher_GetKeySource() !=
        MXC_CTB_REVA_CIPHER_KEY_SOFTWARE) {
        return;
    }

    memcpy((uint8_t *)&ctb_regs->cipher_key[0], key, len);
}

void MXC_CTB_RevA_Cipher_SetIV(mxc_ctb_reva_regs_t *ctb_regs, uint8_t *iv, uint32_t len)
{
    memcpy((uint8_t *)&ctb_regs->cipher_init[0], iv, len);
}

void MXC_CTB_RevA_Cipher_GetIV(mxc_ctb_reva_regs_t *ctb_regs, uint8_t *ivOut, uint32_t len)
{
    memcpy(ivOut, (uint8_t *)&ctb_regs->cipher_init[0], len);
}

/************************/
/* High Level Functions */
/************************/

static int MXC_CTB_Cipher_Generic(mxc_ctb_cipher_req_t *req, int op)
{
    int i, enabled;
    mxc_ctb_reva_dma_req_t dma_req;

    if (req == NULL) {
        return E_NULL_PTR;
    }

    if (!(req->plaintext && req->ciphertext)) {
        return E_NULL_PTR;
    }

    if (req->ptLen == 0) {
        return E_INVALID;
    }

    enabled = MXC_CTB_CheckInterrupts();
    MXC_CTB_DisableInt();

    int dataLength = MXC_CTB_Cipher_GetBlockSize(MXC_CTB_Cipher_GetCipher());
    int numBlocks = ((int)req->ptLen - 1) / dataLength + 1;

    // Load Initial Vector if necessary
    if ((mxc_ctb_reva_cipher_mode_t)MXC_CTB_Cipher_GetMode() != MXC_CTB_REVA_MODE_ECB) {
        if (req->iv == NULL) {
            return E_NULL_PTR;
        }

        MXC_CTB_Cipher_SetIV(req->iv, dataLength);
    }

    // Configure for encryption/decryption
    MXC_CTB_Cipher_SetOperation((mxc_ctb_cipher_operation_t)op);

    dma_req.sourceBuffer = req->plaintext;
    dma_req.destBuffer = req->ciphertext;
    dma_req.length = dataLength;

    MXC_CTB_DMA_SetWriteSource((mxc_ctb_dma_write_source_t)MXC_CTB_REVA_DMA_WRITE_FIFO_CIPHER);
    MXC_CTB_DMA_SetupOperation((mxc_ctb_dma_req_t *)&dma_req);

    for (i = 0; i < numBlocks; i++) {
        // Wait until ready for data
        while (!MXC_CTB_Ready()) {}

        MXC_CTB_DMA_StartTransfer(dataLength);

        // Wait until operation is complete
        while (!(MXC_CTB_Done() & MXC_CTB_REVA_FEATURE_CIPHER)) {}
    }

    if (enabled) {
        MXC_CTB_EnableInt();
    }

    return E_NO_ERROR;
}

int MXC_CTB_RevA_Cipher_Encrypt(mxc_ctb_reva_cipher_req_t *req)
{
    return MXC_CTB_Cipher_Generic((mxc_ctb_cipher_req_t *)req, 0);
}

int MXC_CTB_RevA_Cipher_Decrypt(mxc_ctb_reva_cipher_req_t *req)
{
    return MXC_CTB_Cipher_Generic((mxc_ctb_cipher_req_t *)req, 1);
}

static int MXC_CTB_Cipher_EncDecAsc(mxc_ctb_cipher_req_t *req)
{
    async_i++;

    if (async_i == async_numBlocks) {
        return 1;
    }

    MXC_CTB_DMA_StartTransfer(async_dataLength);
    return 0;
}

static void MXC_CTB_Cipher_GenericAsync(mxc_ctb_cipher_req_t *req, int op)
{
    mxc_ctb_reva_dma_req_t dma_req;

    MXC_ASSERT(req);
    MXC_ASSERT(req->plaintext && req->ciphertext && req->callback);

    while (MXC_GetLock((void *)&MXC_CTB_Callbacks[CPH_ID], 1) != E_NO_ERROR) {}

    MXC_CTB_DisableInt();

    MXC_CTB_Callbacks[CPH_ID] = req->callback;
    saved_requests[CPH_ID] = req;

    async_dataLength = MXC_CTB_Cipher_GetBlockSize(MXC_CTB_Cipher_GetCipher());
    async_numBlocks = ((int)req->ptLen - 1) / async_dataLength + 1;
    async_i = 0;

    // Load Initial Vector if necessary
    if ((mxc_ctb_reva_cipher_mode_t)MXC_CTB_Cipher_GetMode() != MXC_CTB_REVA_MODE_ECB) {
        MXC_ASSERT(req->iv);

        MXC_CTB_Cipher_SetIV(req->iv, async_dataLength);
    }

    // Configure for encryption
    MXC_CTB_Cipher_SetOperation((mxc_ctb_cipher_operation_t)op);

    dma_req.sourceBuffer = req->plaintext;
    dma_req.destBuffer = req->ciphertext;

    MXC_CTB_DMA_SetWriteSource((mxc_ctb_dma_write_source_t)MXC_CTB_REVA_DMA_WRITE_FIFO_CIPHER);
    MXC_CTB_DMA_SetupOperation((mxc_ctb_dma_req_t *)&dma_req);

    MXC_CTB_EnableInt();

    // Wait until ready for data
    while (!MXC_CTB_Ready()) {}

    MXC_CTB_DMA_StartTransfer(async_dataLength);
}

void MXC_CTB_RevA_Cipher_EncryptAsync(mxc_ctb_reva_cipher_req_t *req)
{
    MXC_CTB_Cipher_GenericAsync((mxc_ctb_cipher_req_t *)req, 0);
}

void MXC_CTB_RevA_Cipher_DecryptAsync(mxc_ctb_reva_cipher_req_t *req)
{
    MXC_CTB_Cipher_GenericAsync((mxc_ctb_cipher_req_t *)req, 1);
}
