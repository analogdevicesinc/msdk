/******************************************************************************
 *
 * Copyright (C) 2023-2025 Analog Devices, Inc.
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

#if defined(CONFIG_SOC_MAX32665) || defined(CONFIG_SOC_MAX32666) || defined(CONFIG_SOC_MAX32650)
#define ADI_MAX32_DMA_CTRL_DIS_IE MXC_F_DMA_CFG_CHDIEN
#define ADI_MAX32_DMA_CTRL_CTZIEN MXC_F_DMA_CFG_CTZIEN

#define ADI_MAX32_DMA_STATUS_IPEND MXC_F_DMA_ST_IPEND
#define ADI_MAX32_DMA_STATUS_BUS_ERR MXC_F_DMA_ST_BUS_ERR
#define ADI_MAX32_DMA_STATUS_TO_IF MXC_F_DMA_ST_TO_ST
#define ADI_MAX32_DMA_STATUS_ST MXC_F_DMA_ST_CH_ST

#define ADI_MAX32_DMA_CFG_REQ_POS MXC_F_DMA_CFG_REQSEL_POS
#else
#define ADI_MAX32_DMA_CTRL_DIS_IE MXC_F_DMA_CTRL_DIS_IE
#define ADI_MAX32_DMA_CTRL_CTZIEN MXC_F_DMA_CTRL_CTZ_IE

#define ADI_MAX32_DMA_STATUS_IPEND MXC_F_DMA_STATUS_IPEND
#define ADI_MAX32_DMA_STATUS_BUS_ERR MXC_F_DMA_STATUS_BUS_ERR
#define ADI_MAX32_DMA_STATUS_TO_IF MXC_F_DMA_STATUS_TO_IF
#define ADI_MAX32_DMA_STATUS_ST MXC_F_DMA_STATUS_STATUS

#define ADI_MAX32_DMA_CFG_REQ_POS MXC_F_DMA_CTRL_REQUEST_POS
#endif

static inline int MXC_DMA_GetIntFlags(mxc_dma_regs_t *dma)
{
#if defined(CONFIG_SOC_MAX32665) || defined(CONFIG_SOC_MAX32666) || defined(CONFIG_SOC_MAX32650)
    return dma->intr;
#elif defined(CONFIG_SOC_MAX32660)
    return dma->int_fl;
#else
    return dma->intfl;
#endif
}

static inline int Wrap_MXC_DMA_Init(mxc_dma_regs_t *dma)
{
#if defined(CONFIG_SOC_MAX32657) || defined(CONFIG_SOC_MAX32665) || defined(CONFIG_SOC_MAX32666)
    return MXC_DMA_Init(dma);
#else
    (void)dma;
    return MXC_DMA_Init();
#endif
}

static inline void Wrap_MXC_DMA_DeInit(mxc_dma_regs_t *dma)
{
#if defined(CONFIG_SOC_MAX32657) || defined(CONFIG_SOC_MAX32665) || defined(CONFIG_SOC_MAX32666)
    MXC_DMA_DeInit(dma);
#else
    (void)dma;
    MXC_DMA_DeInit();
#endif
}

static inline int Wrap_MXC_DMA_AcquireChannel(mxc_dma_regs_t *dma)
{
#if defined(CONFIG_SOC_MAX32657) || defined(CONFIG_SOC_MAX32665) || defined(CONFIG_SOC_MAX32666)
    return MXC_DMA_AcquireChannel(dma);
#else
    (void)dma;
    return MXC_DMA_AcquireChannel();
#endif
}

static inline int Wrap_MXC_DMA_GetChannelIndex(mxc_dma_regs_t *dma, uint8_t ch)
{
#if defined(CONFIG_SOC_MAX32657)
    (void)dma;
    return ch;
#else
    return (ch + MXC_DMA_GET_IDX(dma) * (MXC_DMA_CHANNELS / MXC_DMA_INSTANCES));
#endif
}

static inline void Wrap_MXC_DMA_Handler(mxc_dma_regs_t *dma)
{
#if defined(CONFIG_SOC_MAX32657) || defined(CONFIG_SOC_MAX32665) || defined(CONFIG_SOC_MAX32666)
    MXC_DMA_Handler(dma);
#else
    (void)dma;
    MXC_DMA_Handler();
#endif
}

static inline int Wrap_MXC_DMA_MemCpy(mxc_dma_regs_t *dma, void *dest, void *src, int len,
                                      mxc_dma_complete_cb_t callback)
{
#if defined(CONFIG_SOC_MAX32657) || defined(CONFIG_SOC_MAX32665) || defined(CONFIG_SOC_MAX32666)
    return MXC_DMA_MemCpy(dma, dest, src, len, callback);
#else
    (void)dma;
    return MXC_DMA_MemCpy(dest, src, len, callback);
#endif
}

static inline int Wrap_MXC_DMA_DoTransfer(mxc_dma_regs_t *dma, mxc_dma_config_t config,
                                          mxc_dma_srcdst_t firstSrcDst,
                                          mxc_dma_trans_chain_t callback)
{
#if defined(CONFIG_SOC_MAX32657) || defined(CONFIG_SOC_MAX32665) || defined(CONFIG_SOC_MAX32666)
    return MXC_DMA_DoTransfer(dma, config, firstSrcDst, callback);
#else
    (void)dma;
    return MXC_DMA_DoTransfer(config, firstSrcDst, callback);
#endif
}

static inline int Wrap_MXC_DMA_EnableInt(mxc_dma_regs_t *dma, int ch)
{
#if defined(CONFIG_SOC_MAX32657)
    return MXC_DMA_EnableInt(dma, ch);
#else
    (void)dma;
    return MXC_DMA_EnableInt(ch);
#endif
}

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_DMA_H_
