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
#include <stdbool.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "dma.h"
#include "dma_reva.h"
#include "dma_regs.h"

/***** Definitions *****/

/******* Globals *******/

/****** Functions ******/

int MXC_DMA_Init(void)
{
#ifndef MSDK_NO_GPIO_CLK_INIT
    if (!MXC_SYS_IsClockEnabled(MXC_SYS_PERIPH_CLOCK_DMA)) {
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_DMA);
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_DMA);
    }
#endif

    return MXC_DMA_RevA_Init((mxc_dma_reva_regs_t *)MXC_DMA);
}

void MXC_DMA_DeInit(void)
{
    return MXC_DMA_RevA_DeInit((mxc_dma_reva_regs_t *)MXC_DMA);
}

int MXC_DMA_AcquireChannel(void)
{
    return MXC_DMA_RevA_AcquireChannel((mxc_dma_reva_regs_t *)MXC_DMA);
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

int MXC_DMA_GetSrcDst(mxc_dma_srcdst_t *srcdst)
{
    return MXC_DMA_RevA_GetSrcDst(srcdst);
}

int MXC_DMA_SetSrcReload(mxc_dma_srcdst_t srcdst)
{
    return MXC_DMA_RevA_SetSrcReload(srcdst);
}

int MXC_DMA_GetSrcReload(mxc_dma_srcdst_t *srcdst)
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
    return MXC_DMA_RevA_EnableInt((mxc_dma_reva_regs_t *)MXC_DMA, ch);
}

int MXC_DMA_DisableInt(int ch)
{
    return MXC_DMA_RevA_DisableInt((mxc_dma_reva_regs_t *)MXC_DMA, ch);
}

int MXC_DMA_Start(int ch)
{
    return MXC_DMA_RevA_Start(ch);
}

int MXC_DMA_Stop(int ch)
{
    return MXC_DMA_RevA_Stop(ch);
}

mxc_dma_ch_regs_t *MXC_DMA_GetCHRegs(int ch)
{
    return MXC_DMA_RevA_GetCHRegs(ch);
}

void MXC_DMA_Handler(void)
{
    MXC_DMA_RevA_Handler((mxc_dma_reva_regs_t *)MXC_DMA);
}

int MXC_DMA_MemCpy(void *dest, void *src, int len, mxc_dma_complete_cb_t callback)
{
    return MXC_DMA_RevA_MemCpy((mxc_dma_reva_regs_t *)MXC_DMA, dest, src, len, callback);
}

int MXC_DMA_DoTransfer(mxc_dma_config_t config, mxc_dma_srcdst_t firstSrcDst,
                       mxc_dma_trans_chain_t callback)
{
    return MXC_DMA_RevA_DoTransfer((mxc_dma_reva_regs_t *)MXC_DMA, config, firstSrcDst, callback);
}
