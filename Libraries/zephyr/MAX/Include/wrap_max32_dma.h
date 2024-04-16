/******************************************************************************
 *
 * Copyright (C) 2023 Analog Devices, Inc.
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

#ifndef LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_DMA_H_
#define LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_DMA_H_

/***** Includes *****/
#include <dma.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  MAX32665, MAX32666 related mapping
 */
#if defined(CONFIG_SOC_MAX32665) || (CONFIG_SOC_MAX32666)

#define ADI_MAX32_DMA_CTRL_DIS_IE MXC_F_DMA_CFG_CHDIEN
#define ADI_MAX32_DMA_CTRL_CTZIEN MXC_F_DMA_CFG_CTZIEN

#define ADI_MAX32_DMA_STATUS_IPEND MXC_F_DMA_ST_IPEND
#define ADI_MAX32_DMA_STATUS_BUS_ERR MXC_F_DMA_ST_BUS_ERR
#define ADI_MAX32_DMA_STATUS_TO_IF MXC_F_DMA_ST_TO_ST
#define ADI_MAX32_DMA_STATUS_ST MXC_F_DMA_ST_CH_ST

#define ADI_MAX32_DMA_CFG_REQ_POS MXC_F_DMA_CFG_REQSEL_POS

static inline int MXC_DMA_GetIntFlags(mxc_dma_regs_t *dma)
{
    return dma->intr;
}

static inline int Wrap_MXC_DMA_Init(mxc_dma_regs_t *dma)
{
    return MXC_DMA_Init(dma);
}

static inline void Wrap_MXC_DMA_DeInit(mxc_dma_regs_t *dma)
{
    MXC_DMA_DeInit(dma);
}

static inline int Wrap_MXC_DMA_AcquireChannel(mxc_dma_regs_t *dma)
{
    return MXC_DMA_AcquireChannel(dma);
}

static inline void Wrap_MXC_DMA_Handler(mxc_dma_regs_t *dma)
{
    MXC_DMA_Handler(dma);
}

static inline int Wrap_MXC_DMA_MemCpy(mxc_dma_regs_t *dma, void *dest, void *src, int len,
                                      mxc_dma_complete_cb_t callback)
{
    return MXC_DMA_MemCpy(dma, dest, src, len, callback);
}

static inline int Wrap_MXC_DMA_DoTransfer(mxc_dma_regs_t *dma, mxc_dma_config_t config,
                                          mxc_dma_srcdst_t firstSrcDst,
                                          mxc_dma_trans_chain_t callback)
{
    return MXC_DMA_DoTransfer(dma, config, firstSrcDst, callback);
}

/*
 *  MAX32690, MAX32655 related mapping
 */
#elif defined(CONFIG_SOC_MAX32690) || (CONFIG_SOC_MAX32655) || (CONFIG_SOC_MAX32670) || \
    (CONFIG_SOC_MAX32672) || (CONFIG_SOC_MAX32662) || (CONFIG_SOC_MAX32675) ||          \
    (CONFIG_SOC_MAX32680)

#define ADI_MAX32_DMA_CTRL_DIS_IE MXC_F_DMA_CTRL_DIS_IE
#define ADI_MAX32_DMA_CTRL_CTZIEN MXC_F_DMA_CTRL_CTZ_IE

#define ADI_MAX32_DMA_STATUS_IPEND MXC_F_DMA_STATUS_IPEND
#define ADI_MAX32_DMA_STATUS_BUS_ERR MXC_F_DMA_STATUS_BUS_ERR
#define ADI_MAX32_DMA_STATUS_TO_IF MXC_F_DMA_STATUS_TO_IF
#define ADI_MAX32_DMA_STATUS_ST MXC_F_DMA_STATUS_STATUS

#define ADI_MAX32_DMA_CFG_REQ_POS MXC_F_DMA_CTRL_REQUEST_POS

static inline int MXC_DMA_GetIntFlags(mxc_dma_regs_t *dma)
{
    return dma->intfl;
}

static inline int Wrap_MXC_DMA_Init(mxc_dma_regs_t *dma)
{
    (void)dma;
    return MXC_DMA_Init();
}

static inline void Wrap_MXC_DMA_DeInit(mxc_dma_regs_t *dma)
{
    (void)dma;
    MXC_DMA_DeInit();
}

static inline int Wrap_MXC_DMA_AcquireChannel(mxc_dma_regs_t *dma)
{
    (void)dma;
    return MXC_DMA_AcquireChannel();
}

static inline void Wrap_MXC_DMA_Handler(mxc_dma_regs_t *dma)
{
    (void)dma;
    MXC_DMA_Handler();
}

static inline int Wrap_MXC_DMA_MemCpy(mxc_dma_regs_t *dma, void *dest, void *src, int len,
                                      mxc_dma_complete_cb_t callback)
{
    (void)dma;
    return MXC_DMA_MemCpy(dest, src, len, callback);
}

static inline int Wrap_MXC_DMA_DoTransfer(mxc_dma_regs_t *dma, mxc_dma_config_t config,
                                          mxc_dma_srcdst_t firstSrcDst,
                                          mxc_dma_trans_chain_t callback)
{
    (void)dma;
    return MXC_DMA_DoTransfer(config, firstSrcDst, callback);
}

#endif // part number

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_DMA_H_
