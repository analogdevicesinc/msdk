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

/****** Includes *******/
#include <stddef.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "dma.h"
#include "dma_reva.h"
#include "dma_reva_regs.h"

/***** Definitions *****/
#define CHECK_HANDLE(x) ((x >= 0) && (x < MXC_DMA_CHANNELS) && (dma_resource[x].valid))

typedef struct {
    void *userCallback; // user given callback
    void *dest; // memcpy destination
} mxc_dma_highlevel_t;

typedef struct {
    unsigned int valid; // Flag to invalidate this resource
    unsigned int instance; // Hardware instance of this DMA controller
    unsigned int id; // Channel ID, which matches the index into the underlying hardware
    mxc_dma_reva_ch_regs_t *regs; // Pointer to the registers for this channel
    void (*cb)(int, int); // Pointer to a callback function type
} mxc_dma_channel_t;

/******* Globals *******/
static unsigned int dma_initialized[MXC_DMA_INSTANCES] = { 0 };
static mxc_dma_channel_t dma_resource[MXC_DMA_CHANNELS];
static mxc_dma_highlevel_t memcpy_resource[MXC_DMA_CHANNELS];
static uint32_t dma_lock;

/****** Functions ******/
static void memcpy_callback(int ch, int error);
static void transfer_callback(int ch, int error);

int MXC_DMA_RevA_Init(mxc_dma_reva_regs_t *dma)
{
    int i, numCh, offset;
    int dma_idx;

    dma_idx = MXC_DMA_GET_IDX((mxc_dma_regs_t *)dma);
    MXC_ASSERT(dma_idx >= 0);

#if TARGET_NUM == 32665
    numCh = MXC_DMA_CH_OFFSET;
    offset = numCh * dma_idx;
#else
    numCh = MXC_DMA_CHANNELS;
    offset = 0;
#endif

    if (dma_initialized[dma_idx]) {
        return E_BAD_STATE;
    }

#ifndef __riscv
    /* Initialize mutex */
    MXC_FreeLock(&dma_lock);

    if (MXC_GetLock(&dma_lock, 1) != E_NO_ERROR) {
        return E_BUSY;
    }
#endif

    /* Ensure all channels are disabled at start, clear flags, init handles */
    dma->inten = 0;

    for (i = offset; i < (offset + numCh); i++) {
        dma_resource[i].valid = 0;
        dma_resource[i].instance = 0;
        dma_resource[i].id = i;
        dma_resource[i].regs = (mxc_dma_reva_ch_regs_t *)&(dma->ch[(i % numCh)]);
        dma_resource[i].regs->ctrl = 0;
        dma_resource[i].regs->status = dma_resource[i].regs->status;

        dma_resource[i].cb = NULL;
    }

    dma_initialized[dma_idx]++;
#ifndef __riscv
    MXC_FreeLock(&dma_lock);
#endif

    return E_NO_ERROR;
}

void MXC_DMA_RevA_DeInit(mxc_dma_reva_regs_t *dma)
{
    int dma_idx;

    dma_idx = MXC_DMA_GET_IDX((mxc_dma_regs_t *)dma);
    MXC_ASSERT(dma_idx >= 0);

    dma_initialized[dma_idx] = 0;
}

int MXC_DMA_RevA_AcquireChannel(mxc_dma_reva_regs_t *dma)
{
    int i, channel, numCh, offset;
    int dma_idx;

    dma_idx = MXC_DMA_GET_IDX((mxc_dma_regs_t *)dma);
    MXC_ASSERT(dma_idx >= 0);

    /* Check for initialization */
    if (!dma_initialized[dma_idx]) {
        return E_BAD_STATE;
    }

#if TARGET_NUM == 32665
    numCh = MXC_DMA_CH_OFFSET;
    offset = MXC_DMA_CH_OFFSET * dma_idx;
#else
    numCh = MXC_DMA_CHANNELS;
    offset = 0;
#endif

#ifndef __riscv
    /* If DMA is locked return busy */
    if (MXC_GetLock(&dma_lock, 1) != E_NO_ERROR) {
        return E_BUSY;
    }
#endif
    /* Default is no channel available */
    channel = E_NONE_AVAIL;

    for (i = offset; i < (offset + numCh); i++) {
        if (!dma_resource[i].valid) {
            /* Found one */
            channel = i;
            dma_resource[i].valid = 1;
            dma_resource[i].regs->ctrl = 0;
            dma_resource[i].regs->cntrld = 0; /* Used by DMA_Start() to conditionally set RLDEN */
            break;
        }
    }
#ifndef __riscv
    MXC_FreeLock(&dma_lock);
#endif

    return channel;
}

int MXC_DMA_RevA_ReleaseChannel(int ch)
{
    if (CHECK_HANDLE(ch)) {
        if (MXC_GetLock(&dma_lock, 1) != E_NO_ERROR) {
            return E_BUSY;
        }

        dma_resource[ch].valid = 0;
        dma_resource[ch].regs->ctrl = 0;
        dma_resource[ch].regs->status = dma_resource[ch].regs->status;
        MXC_FreeLock(&dma_lock);
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

int MXC_DMA_RevA_ConfigChannel(mxc_dma_config_t config, mxc_dma_srcdst_t srcdst)
{
    if (CHECK_HANDLE(config.ch)) {
        /* Designed to be safe, not speedy. Should not be called often */
        dma_resource[config.ch].regs->ctrl =
            ((config.srcinc_en ? MXC_F_DMA_REVA_CTRL_SRCINC : 0) |
             (config.dstinc_en ? MXC_F_DMA_REVA_CTRL_DSTINC : 0) | config.reqsel |
             (config.srcwd << MXC_F_DMA_REVA_CTRL_SRCWD_POS) |
             (config.dstwd << MXC_F_DMA_REVA_CTRL_DSTWD_POS));
    } else {
        return E_BAD_PARAM;
    }

    return MXC_DMA_RevA_SetSrcDst(srcdst);
}

int MXC_DMA_RevA_AdvConfigChannel(mxc_dma_adv_config_t advConfig)
{
    if (CHECK_HANDLE(advConfig.ch) && (advConfig.burst_size > 0)) {
        dma_resource[advConfig.ch].regs->ctrl &= ~(0x1F00FC0C); // Clear all fields we set here
        /* Designed to be safe, not speedy. Should not be called often */
        dma_resource[advConfig.ch].regs->ctrl |=
            ((advConfig.reqwait_en ? MXC_F_DMA_REVA_CTRL_TO_WAIT : 0) | advConfig.prio |
             advConfig.tosel | advConfig.pssel |
             (((advConfig.burst_size - 1) << MXC_F_DMA_REVA_CTRL_BURST_SIZE_POS) &
              MXC_F_DMA_REVA_CTRL_BURST_SIZE));
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

int MXC_DMA_RevA_SetSrcDst(mxc_dma_srcdst_t srcdst)
{
    if (CHECK_HANDLE(srcdst.ch)) {
        dma_resource[srcdst.ch].regs->src = (unsigned int)srcdst.source;
        dma_resource[srcdst.ch].regs->dst = (unsigned int)srcdst.dest;
        dma_resource[srcdst.ch].regs->cnt = srcdst.len;
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

int MXC_DMA_RevA_GetSrcDst(mxc_dma_srcdst_t *srcdst)
{
    if (CHECK_HANDLE(srcdst->ch)) {
        srcdst->source = (void *)dma_resource[srcdst->ch].regs->src;
        srcdst->dest = (void *)dma_resource[srcdst->ch].regs->dst;
        srcdst->len = (dma_resource[srcdst->ch].regs->cnt) & ~MXC_F_DMA_REVA_CNTRLD_EN;
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

int MXC_DMA_RevA_SetSrcReload(mxc_dma_srcdst_t srcdst)
{
    if (CHECK_HANDLE(srcdst.ch)) {
        dma_resource[srcdst.ch].regs->srcrld = (unsigned int)srcdst.source;
        dma_resource[srcdst.ch].regs->dstrld = (unsigned int)srcdst.dest;

        if (dma_resource[srcdst.ch].regs->ctrl & MXC_F_DMA_REVA_CTRL_EN) {
            /* If channel is already running, set RLDEN to enable next reload */
            dma_resource[srcdst.ch].regs->cntrld = MXC_F_DMA_REVA_CNTRLD_EN | srcdst.len;
        } else {
            /* Otherwise, this is the initial setup, so DMA_Start() will handle setting that bit */
            dma_resource[srcdst.ch].regs->cntrld = srcdst.len;
        }
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

int MXC_DMA_RevA_GetSrcReload(mxc_dma_srcdst_t *srcdst)
{
    if (CHECK_HANDLE(srcdst->ch)) {
        srcdst->source = (void *)dma_resource[srcdst->ch].regs->srcrld;
        srcdst->dest = (void *)dma_resource[srcdst->ch].regs->dstrld;
        srcdst->len = (dma_resource[srcdst->ch].regs->cntrld) & ~MXC_F_DMA_REVA_CNTRLD_EN;
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

int MXC_DMA_RevA_SetCallback(int ch, void (*callback)(int, int))
{
    if (CHECK_HANDLE(ch)) {
        /* Callback for interrupt handler, no checking is done, as NULL is valid for(none)  */
        dma_resource[ch].cb = callback;
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

int MXC_DMA_RevA_SetChannelInterruptEn(int ch, bool chdis, bool ctz)
{
    if (CHECK_HANDLE(ch)) {
        if (chdis) {
            dma_resource[ch].regs->ctrl |= (MXC_F_DMA_REVA_CTRL_DIS_IE);
        }
        if (ctz) {
            dma_resource[ch].regs->ctrl |= (MXC_F_DMA_REVA_CTRL_CTZ_IE);
        }
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

int MXC_DMA_RevA_GetChannelInterruptEn(int ch)
{
    return E_NOT_SUPPORTED;
}

int MXC_DMA_RevA_ChannelEnableInt(int ch, int flags)
{
    if (CHECK_HANDLE(ch)) {
        dma_resource[ch].regs->ctrl |=
            (flags & (MXC_F_DMA_REVA_CTRL_DIS_IE | MXC_F_DMA_REVA_CTRL_CTZ_IE));
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

int MXC_DMA_RevA_ChannelDisableInt(int ch, int flags)
{
    if (CHECK_HANDLE(ch)) {
        dma_resource[ch].regs->ctrl &=
            ~(flags & (MXC_F_DMA_REVA_CTRL_DIS_IE | MXC_F_DMA_REVA_CTRL_CTZ_IE));
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

int MXC_DMA_RevA_EnableInt(mxc_dma_reva_regs_t *dma, int ch)
{
    if (CHECK_HANDLE(ch)) {
#if TARGET_NUM == 32665
        ch %= MXC_DMA_CH_OFFSET;
#endif
        dma->inten |= (1 << ch);
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

int MXC_DMA_RevA_DisableInt(mxc_dma_reva_regs_t *dma, int ch)
{
    if (CHECK_HANDLE(ch)) {
#if TARGET_NUM == 32665
        ch %= MXC_DMA_CH_OFFSET;
#endif
        dma->inten &= ~(1 << ch);
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

int MXC_DMA_RevA_ChannelGetFlags(int ch)
{
    if (CHECK_HANDLE(ch)) {
        return dma_resource[ch].regs->status;
    } else {
        return E_BAD_PARAM;
    }
}

int MXC_DMA_RevA_ChannelClearFlags(int ch, int flags)
{
    if (CHECK_HANDLE(ch)) {
        dma_resource[ch].regs->status |= (flags & 0x5F); // Mask for Interrupt flags
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

int MXC_DMA_RevA_Start(int ch)
{
    if (CHECK_HANDLE(ch)) {
        MXC_DMA_ChannelClearFlags(ch, MXC_DMA_RevA_ChannelGetFlags(ch));

        if (dma_resource[ch].regs->cntrld) {
            dma_resource[ch].regs->ctrl |= (MXC_F_DMA_REVA_CTRL_EN | MXC_F_DMA_REVA_CTRL_RLDEN);
        } else {
            dma_resource[ch].regs->ctrl |= MXC_F_DMA_REVA_CTRL_EN;
        }
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

int MXC_DMA_RevA_Stop(int ch)
{
    if (CHECK_HANDLE(ch)) {
        dma_resource[ch].regs->ctrl &= ~MXC_F_DMA_REVA_CTRL_EN;
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

mxc_dma_ch_regs_t *MXC_DMA_RevA_GetCHRegs(int ch)
{
    if (CHECK_HANDLE(ch)) {
        return (mxc_dma_ch_regs_t *)dma_resource[ch].regs;
    } else {
        return NULL;
    }
}

void MXC_DMA_RevA_Handler(mxc_dma_reva_regs_t *dma)
{
    int numCh = MXC_DMA_CHANNELS / MXC_DMA_INSTANCES;
    int dma_idx;
    int offset;

    dma_idx = MXC_DMA_GET_IDX((mxc_dma_regs_t *)dma);
    MXC_ASSERT(dma_idx >= 0);

    offset = numCh * dma_idx;
    /* Do callback, if enabled */
    for (int i = offset; i < (offset + numCh); i++) {
        if (CHECK_HANDLE(i)) {
            if (dma->intfl & (0x1 << (i % numCh))) {
                if (dma_resource[i].cb != NULL) {
                    dma_resource[i].cb(i, E_NO_ERROR);
                }

                MXC_DMA_ChannelClearFlags(i, MXC_DMA_RevA_ChannelGetFlags(i));

                // No need to check rest of the channels if no interrupt flags set.
                if (dma->intfl == 0) {
                    break;
                }
            }
        }
    }
}

void memcpy_callback(int ch, int error)
{
    mxc_dma_complete_cb_t callback;
    callback = (mxc_dma_complete_cb_t)memcpy_resource[ch].userCallback;

    if (error != E_NO_ERROR) {
        callback(NULL);
    }

    callback(memcpy_resource[ch].dest);

    // Release global objects and local resources
    callback = NULL;
    memcpy_resource[ch].userCallback = NULL;
    memcpy_resource[ch].dest = NULL;
    MXC_DMA_ReleaseChannel(ch);
}

int MXC_DMA_RevA_MemCpy(mxc_dma_reva_regs_t *dma, void *dest, void *src, int len,
                        mxc_dma_complete_cb_t callback)
{
    int retval;
    mxc_dma_config_t config;
    mxc_dma_srcdst_t transfer;
    int channel;

#if (TARGET_NUM == 32665 || TARGET_NUM == 32657)
    channel = MXC_DMA_AcquireChannel((mxc_dma_regs_t *)dma);
#else
    channel = MXC_DMA_AcquireChannel();
#endif

    if (memcpy_resource[channel].userCallback != NULL) {
        // We acquired a channel we haven't cleared yet
        MXC_DMA_ReleaseChannel(channel);
        return E_UNKNOWN;
    }

    transfer.ch = channel;
    transfer.source = src;
    transfer.dest = dest;
    transfer.len = len;

    config.ch = channel;
    config.reqsel = MXC_DMA_REQUEST_MEMTOMEM;
    config.srcwd = MXC_DMA_WIDTH_WORD;
    config.dstwd = MXC_DMA_WIDTH_WORD;
    config.srcinc_en = 1;
    config.dstinc_en = 1;

    retval = MXC_DMA_ConfigChannel(config, transfer);

    if (retval != E_NO_ERROR) {
        return retval;
    }

#if (TARGET_NUM == 32657)
    retval = MXC_DMA_EnableInt((mxc_dma_regs_t *)dma, channel);
#else
    retval = MXC_DMA_EnableInt(channel);
#endif

    if (retval != E_NO_ERROR) {
        return retval;
    }

    retval = MXC_DMA_ChannelEnableInt(channel, MXC_F_DMA_REVA_CTRL_CTZ_IE);

    if (retval != E_NO_ERROR) {
        return retval;
    }

    MXC_DMA_SetCallback(channel, memcpy_callback);

    memcpy_resource[channel].userCallback = (void *)callback;
    memcpy_resource[channel].dest = dest;

    return MXC_DMA_Start(channel);
}

void transfer_callback(int ch, int error)
{
    // Unimplemented
    // Check for reason
    // Call user callback for next transfer
    // determine whether to load into the transfer slot or reload slot
    // continue on or stop
    while (1) {}
}

int MXC_DMA_RevA_DoTransfer(mxc_dma_reva_regs_t *dma, mxc_dma_config_t config,
                            mxc_dma_srcdst_t firstSrcDst, mxc_dma_trans_chain_t callback)
{
    int retval, channel;

#if (TARGET_NUM == 32665 || TARGET_NUM == 32657)
    channel = MXC_DMA_AcquireChannel((mxc_dma_regs_t *)dma);
#else
    channel = MXC_DMA_AcquireChannel();
#endif

    if (memcpy_resource[channel].userCallback != NULL) {
        // We acquired a channel we haven't cleared yet
        MXC_DMA_ReleaseChannel(channel);
        return E_UNKNOWN;
    }

    retval = MXC_DMA_ConfigChannel(config, firstSrcDst);

    if (retval != E_NO_ERROR) {
        return retval;
    }

#if (TARGET_NUM == 32657)
    retval = MXC_DMA_EnableInt((mxc_dma_regs_t *)dma, channel);
#else
    retval = MXC_DMA_EnableInt(channel);
#endif

    if (retval != E_NO_ERROR) {
        return retval;
    }

    retval = MXC_DMA_ChannelEnableInt(channel, MXC_F_DMA_REVA_CTRL_CTZ_IE);

    if (retval != E_NO_ERROR) {
        return retval;
    }

    MXC_DMA_SetCallback(channel, transfer_callback);

    memcpy_resource[channel].userCallback = (void *)callback;

    return MXC_DMA_Start(channel);
}
