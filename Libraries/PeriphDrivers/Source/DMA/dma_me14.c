/* ****************************************************************************
 * Copyright(C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files(the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 *************************************************************************** */

/****** Includes *******/
#include "dma.h"
#include "dma_regs.h"
#include "dma_reva.h"
#include "mxc_assert.h"
#include "mxc_device.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/***** Definitions *****/

/******* Globals *******/

/****** Functions ******/

static mxc_dma_regs_t* getDMAInstance(int ch)
{
    if (ch < (MXC_DMA_CHANNELS / MXC_DMA_INSTANCES)) {
        return MXC_DMA0;
    } else if (ch >= (MXC_DMA_CHANNELS / MXC_DMA_INSTANCES) && ch < MXC_DMA_CHANNELS) {
        return MXC_DMA1;
    }

    return NULL;
}

int MXC_DMA_Init(mxc_dma_regs_t* dma)
{
    switch (MXC_DMA_GET_IDX(dma)) {
    case 0:
        if (!MXC_SYS_IsClockEnabled(MXC_SYS_PERIPH_CLOCK_DMA)) {
            MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_DMA);
            MXC_SYS_Reset_Periph(MXC_SYS_RESET_DMA0);
        }
        break;
    case 1:
        if (!MXC_SYS_IsClockEnabled(MXC_SYS_PERIPH_CLOCK_DMA1)) {
            MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_DMA1);
            MXC_SYS_Reset_Periph(MXC_SYS_RESET_DMA1);
        }
        break;
    default:
        return E_BAD_PARAM;
    }

    return MXC_DMA_RevA_Init((mxc_dma_reva_regs_t*)dma);
}

int MXC_DMA_AcquireChannel(mxc_dma_regs_t* dma)
{
    if (MXC_DMA_GET_IDX(dma) == -1) {
        return E_BAD_PARAM;
    }

    return MXC_DMA_RevA_AcquireChannel((mxc_dma_reva_regs_t*)dma);
}

int MXC_DMA_ReleaseChannel(int ch)
{
    return MXC_DMA_RevA_ReleaseChannel(ch);
}

int MXC_DMA_ConfigChannel(mxc_dma_config_t config, mxc_dma_srcdst_t srcdst)
{
    return MXC_DMA_RevA_ConfigChannel(config, srcdst);
}

int MXC_DMA_AdvConfigChannel(mxc_dma_adv_config_t advConfig)
{
    return MXC_DMA_RevA_AdvConfigChannel(advConfig);
}

int MXC_DMA_SetSrcDst(mxc_dma_srcdst_t srcdst)
{
    return MXC_DMA_RevA_SetSrcDst(srcdst);
}

int MXC_DMA_GetSrcDst(mxc_dma_srcdst_t* srcdst)
{
    return MXC_DMA_RevA_GetSrcDst(srcdst);
}

int MXC_DMA_SetSrcReload(mxc_dma_srcdst_t srcdst)
{
    return MXC_DMA_RevA_SetSrcReload(srcdst);
}

int MXC_DMA_GetSrcReload(mxc_dma_srcdst_t* srcdst)
{
    return MXC_DMA_RevA_GetSrcReload(srcdst);
}

int MXC_DMA_SetCallback(int ch, void (*callback)(int, int))
{
    return MXC_DMA_RevA_SetCallback(ch, callback);
}

int MXC_DMA_SetChannelInterruptEn(int ch, bool chdis, bool ctz)
{
    return MXC_DMA_RevA_SetChannelInterruptEn(ch, chdis, ctz);
}

int MXC_DMA_ChannelEnableInt(int ch, int flags)
{
    return MXC_DMA_RevA_ChannelEnableInt(ch, flags);
}

int MXC_DMA_ChannelDisableInt(int ch, int flags)
{
    return MXC_DMA_RevA_ChannelDisableInt(ch, flags);
}

int MXC_DMA_ChannelGetFlags(int ch)
{
    return MXC_DMA_RevA_ChannelGetFlags(ch);
}

int MXC_DMA_ChannelClearFlags(int ch, int flags)
{
    return MXC_DMA_RevA_ChannelClearFlags(ch, flags);
}

int MXC_DMA_EnableInt(int ch)
{
    mxc_dma_regs_t* dma = getDMAInstance(ch);
    return MXC_DMA_RevA_EnableInt((mxc_dma_reva_regs_t*)dma, ch);
}

int MXC_DMA_DisableInt(int ch)
{
    mxc_dma_regs_t* dma = getDMAInstance(ch);
    return MXC_DMA_RevA_DisableInt((mxc_dma_reva_regs_t*)dma, ch);
}

int MXC_DMA_Start(int ch)
{
    return MXC_DMA_RevA_Start(ch);
}

int MXC_DMA_Stop(int ch)
{
    return MXC_DMA_RevA_Stop(ch);
}

mxc_dma_ch_regs_t* MXC_DMA_GetCHRegs(int ch)
{
    return MXC_DMA_RevA_GetCHRegs(ch);
}

void MXC_DMA_Handler(mxc_dma_regs_t* dma)
{
    if (MXC_DMA_GET_IDX(dma) != -1) {
        MXC_DMA_RevA_Handler((mxc_dma_reva_regs_t*)dma);
    }
}

int MXC_DMA_MemCpy(
    mxc_dma_regs_t* dma, void* dest, void* src, int len, mxc_dma_complete_cb_t callback)
{
    return MXC_DMA_RevA_MemCpy((mxc_dma_reva_regs_t*)dma, dest, src, len, callback);
}

int MXC_DMA_DoTransfer(mxc_dma_regs_t* dma, mxc_dma_config_t config, mxc_dma_srcdst_t firstSrcDst,
    mxc_dma_trans_chain_t callback)
{
    return MXC_DMA_RevA_DoTransfer((mxc_dma_reva_regs_t*)dma, config, firstSrcDst, callback);
}
