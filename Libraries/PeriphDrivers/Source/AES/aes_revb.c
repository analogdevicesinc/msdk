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
#include <stdbool.h>
#include <string.h>
#include "mxc_sys.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "dma.h"
#include "aes_regs.h"
#include "aes_revb.h"
#include "trng_revb.h"

/* **** Variable Declaration **** */

typedef struct {
    uint32_t *inputData; ///< Pointer to input data
    uint32_t *resultData; ///< Pointer to output data
    uint32_t remain; ///< Saves how many words are remaining for operation
    mxc_aes_revb_req_t *req; ///< Saves request struct during an operation
} mxc_aes_revb_intr_req_t;

volatile static mxc_aes_revb_intr_req_t aes_state;

typedef struct {
    uint8_t enc;
    mxc_dma_regs_t *dma;
    int8_t channelRX;
    int8_t channelTX;
    uint32_t remain;
    uint32_t *inputText;
    uint32_t *outputText;
    uint32_t inputBuffer[4];  ///< These 128-bit buffers are intended to format byte ordering for each DMA transaction
    uint32_t outputBuffer[4];
    mxc_aes_revb_req_t *req; ///< Saves request struct during an operation
} mxc_aes_revb_dma_req_t;

volatile static mxc_aes_revb_dma_req_t dma_state;

#define SWAP_BYTES(x)                                                                     \
    ((((x) >> 24) & 0x000000FF) | (((x) >> 8) & 0x0000FF00) | (((x) << 8) & 0x00FF0000) | \
     (((x) << 24) & 0xFF000000))

/* Prevent GCC from optimimzing this function to memcpy */
#if !(defined(__CC_ARM) || defined(__ARMCC_VERSION))
__attribute__((optimize("no-tree-loop-distribute-patterns")))
#endif
static void
memcpy32r(uint32_t *dst, const uint32_t *src, unsigned int len)
{
    uint32_t *dstr = dst + (len / 4) - 1;
    while (len) {
        *dstr = SWAP_BYTES(*src);
        dstr--;
        src++;
        len -= 4;
    }
}

int MXC_AES_RevB_Init(mxc_aes_revb_regs_t *aes, mxc_dma_regs_t *dma)
{
    aes->ctrl = 0x00;

    while (MXC_AES_RevB_IsBusy(aes) != E_NO_ERROR) {}

    aes->ctrl |= MXC_F_AES_REVB_CTRL_EN;

    // DMA is not used.
    if (dma == NULL) {
        return E_NO_ERROR;
    }

    dma_state.dma = dma;

    // Initial undefined state (-1) to indicate the AES driver to acquire channels
    dma_state.channelTX = -1;
    dma_state.channelRX = -1;

    return E_NO_ERROR;
}

int MXC_AES_RevB_Shutdown(mxc_aes_revb_regs_t *aes)
{
    MXC_AES_RevB_FlushInputFIFO(aes);
    MXC_AES_RevB_FlushOutputFIFO(aes);

    while (MXC_AES_RevB_IsBusy(aes) != E_NO_ERROR) {}

    aes->ctrl = 0x00;

    dma_state.dma = NULL;

    // Undefined state
    if (dma_state.channelTX != -1) {
        MXC_DMA_ReleaseChannel(dma_state.channelTX);
    }

    if (dma_state.channelRX != -1) {
        MXC_DMA_ReleaseChannel(dma_state.channelRX);
    } 

    return E_NO_ERROR;
}

int MXC_AES_RevB_IsBusy(mxc_aes_revb_regs_t *aes)
{
    if (aes->status & MXC_F_AES_REVB_STATUS_BUSY) {
        return E_BUSY;
    }

    return E_NO_ERROR;
}

void MXC_AES_RevB_SetKeySize(mxc_aes_revb_regs_t *aes, mxc_aes_revb_keys_t key)
{
    while (MXC_AES_IsBusy() != E_NO_ERROR) {}
    aes->ctrl |= key;
}

mxc_aes_keys_t MXC_AES_RevB_GetKeySize(mxc_aes_revb_regs_t *aes)
{
    return (aes->ctrl & MXC_F_AES_REVB_CTRL_KEY_SIZE);
}

void MXC_AES_RevB_FlushInputFIFO(mxc_aes_revb_regs_t *aes)
{
    while (MXC_AES_IsBusy() != E_NO_ERROR) {}
    aes->ctrl |= MXC_F_AES_REVB_CTRL_INPUT_FLUSH;
}

void MXC_AES_RevB_FlushOutputFIFO(mxc_aes_revb_regs_t *aes)
{
    while (MXC_AES_IsBusy() != E_NO_ERROR) {}
    aes->ctrl |= MXC_F_AES_REVB_CTRL_OUTPUT_FLUSH;
}

void MXC_AES_RevB_Start(mxc_aes_revb_regs_t *aes)
{
    while (MXC_AES_IsBusy() != E_NO_ERROR) {}
    aes->ctrl |= MXC_F_AES_REVB_CTRL_START;
}

void MXC_AES_RevB_EnableInt(mxc_aes_revb_regs_t *aes, uint32_t interrupt)
{
    aes->inten |= (interrupt & (MXC_F_AES_REVB_INTEN_DONE | MXC_F_AES_REVB_INTEN_KEY_CHANGE |
                                MXC_F_AES_REVB_INTEN_KEY_ZERO | MXC_F_AES_REVB_INTEN_OV));
}

void MXC_AES_RevB_DisableInt(mxc_aes_revb_regs_t *aes, uint32_t interrupt)
{
    aes->inten &= ~(interrupt & (MXC_F_AES_REVB_INTEN_DONE | MXC_F_AES_REVB_INTEN_KEY_CHANGE |
                                 MXC_F_AES_REVB_INTEN_KEY_ZERO | MXC_F_AES_REVB_INTEN_OV));
}

uint32_t MXC_AES_RevB_GetFlags(mxc_aes_revb_regs_t *aes)
{
    return aes->intfl;
}

void MXC_AES_RevB_ClearFlags(mxc_aes_revb_regs_t *aes, uint32_t flags)
{
    aes->intfl = (flags & (MXC_F_AES_REVB_INTFL_DONE | MXC_F_AES_REVB_INTFL_KEY_CHANGE |
                           MXC_F_AES_REVB_INTFL_KEY_ZERO | MXC_F_AES_REVB_INTFL_OV));
}

int MXC_AES_RevB_Generic(mxc_aes_revb_regs_t *aes, mxc_aes_revb_req_t *req)
{
    int i;
    int remain;

    if (req == NULL) {
        return E_NULL_PTR;
    }

    if (req->inputData == NULL || req->resultData == NULL) {
        return E_NULL_PTR;
    }

    if (req->length == 0) {
        return E_BAD_PARAM;
    }

    remain = req->length;

    MXC_AES_RevB_FlushInputFIFO(aes);
    MXC_AES_RevB_FlushOutputFIFO(aes);

    MXC_AES_RevB_SetKeySize(aes, req->keySize);

    while (MXC_AES_IsBusy() != E_NO_ERROR) {}

    MXC_SETFIELD(aes->ctrl, MXC_F_AES_REVB_CTRL_TYPE,
                 req->encryption << MXC_F_AES_REVB_CTRL_TYPE_POS);

    while (remain / 4) {
        for (i = 0; i < 4; i++) {
            aes->fifo = SWAP_BYTES(req->inputData[3 - i]);
        }
        req->inputData += 4;

        while (!(aes->intfl & MXC_F_AES_REVB_INTFL_DONE)) {}
        aes->intfl |= MXC_F_AES_REVB_INTFL_DONE;

        for (i = 0; i < 4; i++) {
            uint32_t tmp = aes->fifo;
            req->resultData[3 - i] = SWAP_BYTES(tmp);
        }
        req->resultData += 4;

        remain -= 4;
    }

    if (remain % 4) {
        for (i = 0; i < remain; i++) {
            aes->fifo = SWAP_BYTES(req->inputData[remain - 1 - i]);
        }
        req->inputData += remain;

        // Pad last block with 0's
        for (i = remain; i < 4; i++) {
            aes->fifo = 0;
        }

        while (!(aes->intfl & MXC_F_AES_REVB_INTFL_DONE)) {}
        aes->intfl |= MXC_F_AES_REVB_INTFL_DONE;

        for (i = 0; i < 4; i++) {
            uint32_t tmp = aes->fifo;
            req->resultData[3 - i] = SWAP_BYTES(tmp);
        }
        req->resultData += 4;
    }

    return E_NO_ERROR;
}

int MXC_AES_RevB_Encrypt(mxc_aes_revb_regs_t *aes, mxc_aes_revb_req_t *req)
{
    return MXC_AES_RevB_Generic(aes, req);
}

int MXC_AES_RevB_Decrypt(mxc_aes_revb_regs_t *aes, mxc_aes_revb_req_t *req)
{
    return MXC_AES_RevB_Generic(aes, req);
}

int MXC_AES_RevB_GenericAsync(mxc_aes_revb_regs_t *aes, mxc_aes_revb_req_t *req, uint8_t enc)
{
    int i;

    if (req == NULL) {
        return E_NULL_PTR;
    }

    if (req->inputData == NULL || req->resultData == NULL) {
        return E_NULL_PTR;
    }

    if (req->length == 0) {
        return E_BAD_PARAM;
    }

    // Keep track of request state internally to prevent driver
    //  from modifying caller's request struct.
    aes_state.inputData = req->inputData;
    aes_state.resultData = req->resultData;
    aes_state.remain = req->length;
    aes_state.req = req;

    MXC_AES_RevB_FlushInputFIFO(aes);
    MXC_AES_RevB_FlushOutputFIFO(aes);

    MXC_AES_RevB_SetKeySize(aes, req->keySize);

    while (MXC_AES_IsBusy() != E_NO_ERROR) {}

    MXC_SETFIELD(aes->ctrl, MXC_F_AES_REVB_CTRL_TYPE,
                 req->encryption << MXC_F_AES_REVB_CTRL_TYPE_POS);

    // Write four 32-bit words (128-bits) of data to FIFO to start AES calculation
    if (aes_state.remain / 4) {
        for (i = 0; i < 4; i++) {
            aes->fifo = SWAP_BYTES(aes_state.inputData[3 - i]);
        }
        aes_state.inputData += 4;

        aes_state.remain -= 4;

    } else {
        // For cases when block is less than 128-bits
        // Note, the overhead for an AES (interrupt) operation to context switch to the AES
        //  ISR may be greater than just using 'MXC_AES_Generic(...)', 'MXC_AES_Encrypt(...)',
        //  or 'MXC_AES_Decrypt(...)' for this case
        for (i = 0; i < aes_state.remain; i++) {
            aes->fifo = SWAP_BYTES(aes_state.inputData[aes_state.remain - 1 - i]);
        }

        // Pad last block with 0's
        for (i = aes_state.remain; i < 4; i++) {
            aes->fifo = 0;
        }
        
        // This is the last AES operation
        aes_state.remain -= aes_state.remain;
    }

    // MXC_AES_Handler(...) will handle remaining AES calculation until finished

    return E_NO_ERROR;
}

int MXC_AES_RevB_EncryptAsync(mxc_aes_revb_regs_t *aes, mxc_aes_revb_req_t *req)
{
    return MXC_AES_RevB_GenericAsync(aes, req, 0);
}

int MXC_AES_RevB_DecryptAsync(mxc_aes_revb_regs_t *aes, mxc_aes_revb_req_t *req)
{
    return MXC_AES_RevB_GenericAsync(aes, req, 1);
}

void MXC_AES_RevB_Handler(mxc_aes_revb_regs_t *aes)
{
    int i;

    aes->intfl |= MXC_F_AES_REVB_INTFL_DONE;

    // Latest AES block operation completed. Read 128-bit block from FIFO
    // The driver assumes the resultData buffer length is a multiple of 4
    for (i = 0; i < 4; i++) {
        uint32_t tmp = aes->fifo;
        aes_state.resultData[3 - i] = SWAP_BYTES(tmp);
    }
    aes_state.resultData += 4;

    // AES operation complete.
    if (aes_state.remain == 0) {
        if (aes_state.req->callback != NULL) {
            aes_state.req->callback(aes_state.req, E_NO_ERROR);
        }
    } else {
        // Continue to next operation by writing to FIFO if more blocks remaining.
        if (aes_state.remain / 4) {
            // 128-bit block
            for (i = 0; i < 4; i++) {
                aes->fifo = SWAP_BYTES(aes_state.inputData[3 - i]);
            }
            aes_state.inputData += 4;

            aes_state.remain -= 4;

        } else {
            // Less than 128-bits remaining (End of operation)
            for (i = 0; i < aes_state.remain; i++) {
                aes->fifo = SWAP_BYTES(aes_state.inputData[aes_state.remain - 1 - i]);
            }

            // Pad last block with 0's
            for (i = aes_state.remain; i < 4; i++) {
                aes->fifo = 0;
            }

            aes_state.remain -= aes_state.remain;
        }
    }
}

int MXC_AES_RevB_PreInitDMA(int8_t *rx_channel, int8_t *tx_channel) {
    if (dma_state.dma == NULL) {
        // A valid DMA instance was not passed in MXC_AES_Init(...)
        return E_BAD_STATE;
    }

    // MXC_DMA_INSTANCES is defined by the devices' header file (CMSIS)
#if (MXC_DMA_INSTANCES > 1)
    // Note, the DMA driver does not re-initialize () the DMA peripheral
    MXC_DMA_Init(dma_state.dma);

    if (dma_state.channelTX == -1) {
        dma_state.channelTX = MXC_DMA_AcquireChannel(dma_state.dma);
    }

    if (dma_state.channelRX == -1) {
        dma_state.channelRX = MXC_DMA_AcquireChannel(dma_state.dma);
    }
#else
    MXC_DMA_Init();

    if (dma_state.channelTX == -1) {
        dma_state.channelTX = MXC_DMA_AcquireChannel();
    }

    if (dma_state.channelRX == -1) {
        dma_state.channelRX = MXC_DMA_AcquireChannel();
    }
#endif

    if (tx_channel != NULL) {
        *tx_channel = dma_state.channelTX;
    }

    if (rx_channel != NULL) {
        *rx_channel = dma_state.channelRX;
    }

    return E_NO_ERROR;
}

int MXC_AES_RevB_TXDMAConfig(void *src_addr, int len, mxc_dma_regs_t *dma)
{
    int i;
    int8_t channel;
    mxc_dma_config_t config;
    mxc_dma_srcdst_t srcdst;
    mxc_dma_adv_config_t advconfig = { 0 };

    if (src_addr == NULL) {
        return E_NULL_PTR;
    }

    if (len == 0) {
        return E_BAD_PARAM;
    }

    if (dma_state.channelTX == -1) {
        // MXC_DMA_INSTANCES is defined by the devices' header file (CMSIS)
#if (MXC_DMA_INSTANCES > 1)
        MXC_DMA_Init(dma);

        channel = MXC_DMA_AcquireChannel(dma);
#else
        MXC_DMA_Init();

        channel = MXC_DMA_AcquireChannel();
#endif
        dma_state.channelTX = channel;
    } else {
        channel = dma_state.channelTX;
    }

    config.reqsel = MXC_DMA_REQUEST_AESTX;

    config.ch = channel;

    config.srcwd = MXC_DMA_WIDTH_WORD;
    config.dstwd = MXC_DMA_WIDTH_WORD;

    config.srcinc_en = 1;
    config.dstinc_en = 0;

    // Prepare input buffer with correct orientation.
    // Note, the src address increments with the DMA transaction.
    if (len < 4) {
        for (int i = 0; i < len; i++) {
            dma_state.inputBuffer[i] = SWAP_BYTES(((uint32_t *)src_addr)[len - 1 - i]);
        }

        // Pad last block with 0's
        for (i = len; i < 4; i++) {
            dma_state.inputBuffer[i] = 0;
        }

    } else {
        for (int i = 0; i < 4; i++) {
            dma_state.inputBuffer[i] = SWAP_BYTES(((uint32_t *)src_addr)[len - 1 - i]);
        }
    }

    srcdst.ch = channel;
    srcdst.source = dma_state.inputBuffer;
    srcdst.dest = NULL;
    srcdst.len = 4;

    advconfig.ch = channel;
    advconfig.burst_size = 4;

    MXC_DMA_ConfigChannel(config, srcdst);
    MXC_DMA_AdvConfigChannel(advconfig);
    MXC_DMA_SetCallback(channel, MXC_AES_RevB_DMACallback);

#if (MXC_DMA_INSTANCES > 1)
    MXC_DMA_EnableInt(dma, channel);
#else
    MXC_DMA_EnableInt(channel);
#endif

    MXC_DMA_Start(channel);
    //MXC_DMA->ch[channel].ctrl |= MXC_F_DMA_CTRL_CTZ_IE;
    MXC_DMA_SetChannelInterruptEn(channel, 0, 1);

    return E_NO_ERROR;
}

int MXC_AES_RevB_RXDMAConfig(void *dest_addr, int len, mxc_dma_regs_t *dma)
{
    if (dest_addr == NULL) {
        return E_NULL_PTR;
    }

    if (len == 0) {
        return E_BAD_PARAM;
    }

    int8_t channel;
    mxc_dma_config_t config;
    mxc_dma_srcdst_t srcdst;
    mxc_dma_adv_config_t advconfig = { 0 };

    if (dma_state.channelRX == -1) {
        // MXC_DMA_INSTANCES is defined by the devices' header file (CMSIS)
#if (MXC_DMA_INSTANCES > 1)
        MXC_DMA_Init(dma);

        channel = MXC_DMA_AcquireChannel(dma);
#else
        MXC_DMA_Init();

        channel = MXC_DMA_AcquireChannel();
#endif
        dma_state.channelRX = channel;
    } else {
        channel = dma_state.channelRX;
    }

    config.reqsel = MXC_DMA_REQUEST_AESRX;

    config.ch = channel;

    config.srcwd = MXC_DMA_WIDTH_WORD;
    config.dstwd = MXC_DMA_WIDTH_WORD;

    config.srcinc_en = 0;
    config.dstinc_en = 1;

    srcdst.ch = channel;
    srcdst.source = NULL;
    srcdst.dest = dest_addr;
    srcdst.len = 4;

    // if (dma_state.enc == 0) {
    //     srcdst.len = 4;
    // } else if (len > 4) {
    //     srcdst.len = 4;
    // } else {
    //     srcdst.len = len;
    // }

    advconfig.ch = channel;
    advconfig.burst_size = 4; //

    MXC_DMA_ConfigChannel(config, srcdst);
    MXC_DMA_AdvConfigChannel(advconfig);
    MXC_DMA_SetCallback(channel, MXC_AES_RevB_DMACallback);

#if (MXC_DMA_INSTANCES > 1)
    MXC_DMA_EnableInt(dma, channel);
#else
    MXC_DMA_EnableInt(channel);
#endif

    MXC_DMA_Start(channel);
    //MXC_DMA->ch[channel].ctrl |= MXC_F_DMA_CTRL_CTZ_IE;
    MXC_DMA_SetChannelInterruptEn(channel, 0, 1);

    return E_NO_ERROR;
}

int MXC_AES_RevB_GetTXDMAChannel(int8_t *channel) {
    if (dma_state.channelTX == -1) {
        return E_BAD_STATE;
    }

    if (channel == NULL) {
        return E_NULL_PTR;
    }

    *channel = dma_state.channelTX;

    return E_NO_ERROR;
}

int MXC_AES_RevB_GetRXDMAChannel(int8_t *channel) {
    if (dma_state.channelRX == -1) {
        return E_BAD_STATE;
    }

    if (channel == NULL) {
        return E_NULL_PTR;
    }

    *channel = dma_state.channelRX;

    return E_NO_ERROR;
}

int MXC_AES_RevB_GenericDMA(mxc_aes_revb_regs_t *aes, mxc_aes_revb_req_t *req, uint8_t enc)
{
    if (req == NULL) {
        return E_NULL_PTR;
    }

    if (req->inputData == NULL || req->resultData == NULL) {
        return E_NULL_PTR;
    }

    if (req->length == 0) {
        return E_BAD_PARAM;
    }

    MXC_AES_RevB_FlushInputFIFO(aes);
    MXC_AES_RevB_FlushOutputFIFO(aes);

    MXC_AES_RevB_SetKeySize(aes, req->keySize);

    MXC_AES_IsBusy();
    MXC_SETFIELD(aes->ctrl, MXC_F_AES_REVB_CTRL_TYPE,
                 req->encryption << MXC_F_AES_REVB_CTRL_TYPE_POS);

    aes->inten &= ~(MXC_F_AES_INTEN_DONE);

    dma_state.enc = enc;
    dma_state.remain = req->length;
    dma_state.inputText = req->inputData;
    dma_state.outputText = req->resultData;
    dma_state.req = req;

    aes->ctrl |= MXC_F_AES_REVB_CTRL_DMA_RX_EN; //Enable AES DMA
    aes->ctrl |= MXC_F_AES_REVB_CTRL_DMA_TX_EN; //Enable AES DMA

    if (MXC_AES_RevB_TXDMAConfig(dma_state.inputText, dma_state.remain, dma_state.dma) !=
        E_NO_ERROR) {
        return E_BAD_PARAM;
    }

    aes->ctrl |= MXC_F_AES_REVB_CTRL_START;

    return E_NO_ERROR;
}

int MXC_AES_RevB_EncryptDMA(mxc_aes_revb_regs_t *aes, mxc_aes_revb_req_t *req)
{
    return MXC_AES_RevB_GenericDMA(aes, req, 0);
}

int MXC_AES_RevB_DecryptDMA(mxc_aes_revb_regs_t *aes, mxc_aes_revb_req_t *req)
{
    return MXC_AES_RevB_GenericDMA(aes, req, 1);
}

void MXC_AES_RevB_DMACallback(int ch, int error)
{
    int i;

    if (error != E_NO_ERROR) {
    } else {
        if (dma_state.channelTX == ch) {
            // Maintains previous implemenation for backward compatibility:
            //  - Assumes DMA is not pre-initialized before an AES DMA operation
            //  - Acquires only one DMA channel (whether RX or TX) at a time
            if (dma_state.channelRX == -1) {
                MXC_DMA_ReleaseChannel(dma_state.channelTX);
                dma_state.channelTX = -1;
            }

            if (dma_state.remain < 4) {
                MXC_AES_Start();
            }

            MXC_AES_RevB_RXDMAConfig(dma_state.outputText, dma_state.remain, dma_state.dma);

        } else if (dma_state.channelRX == ch) {
            if (dma_state.remain > 4) {
                dma_state.remain -= 4;
            } else if (dma_state.remain > 0) {
                dma_state.remain = 0;
            }

            // Maintains previous implemenation for backward compatibility:
            //  - Assumes DMA is not pre-initialized before an AES DMA operation
            //  - Acquires only one DMA channel (whether RX or TX) at a time
            if (dma_state.channelTX == -1) {
                MXC_DMA_ReleaseChannel(dma_state.channelRX);
                dma_state.channelRX = -1;
            }

            if (dma_state.remain > 0) {
                MXC_AES_RevB_TXDMAConfig(dma_state.inputText, dma_state.remain, dma_state.dma);

                MXC_AES_Start();
            } else {
                // Follow byte ordering as other generic functions.
                // for (i = 0; i < dma_state.req->length / 2; i++) {
                //     dma_state.outputText[i] = SWAP_BYTES(dma_state.outputText[dma_state.req->length - i - 1]);
                // }

                // End of AES operation
                if (dma_state.req->callback != NULL) {
                    dma_state.req->callback(dma_state.req, E_NO_ERROR);
                }
            }
        }
    }
}

void MXC_AES_RevB_SetExtKey(mxc_aeskeys_revb_regs_t *aeskeys, const void *key, mxc_aes_keys_t len)
{
    int numBytes;

    if (len == MXC_AES_128BITS) {
        numBytes = 16;
    } else if (len == MXC_AES_192BITS) {
        numBytes = 24;
    } else {
        numBytes = 32;
    }

    /* TODO: Figure out if this is the correct byte ordering */
    memcpy32r((void *)&(aeskeys->key0), key, numBytes);
}
