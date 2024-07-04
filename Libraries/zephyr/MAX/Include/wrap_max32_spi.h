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

#ifndef LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_SPI_H_
#define LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_SPI_H_

/***** Includes *****/
#include <spi.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  MAX32665, MAX32666 related mapping
 */
#if defined(CONFIG_SOC_MAX32665) || (CONFIG_SOC_MAX32666)

#define ADI_MAX32_SPI_CTRL_MASTER_MODE MXC_F_SPI_CTRL0_MASTER

#define ADI_MAX32_SPI_INT_FL_RX_UN MXC_F_SPI_INT_FL_RX_UND
#define ADI_MAX32_SPI_INT_FL_RX_OV MXC_F_SPI_INT_FL_RX_OVR
#define ADI_MAX32_SPI_INT_FL_TX_UN MXC_F_SPI_INT_FL_TX_UND
#define ADI_MAX32_SPI_INT_FL_TX_OV MXC_F_SPI_INT_FL_TX_OVR
#define ADI_MAX32_SPI_INT_FL_MST_DONE MXC_F_SPI_INT_FL_M_DONE
#define ADI_MAX32_SPI_INT_FL_ABORT MXC_F_SPI_INT_FL_ABORT
#define ADI_MAX32_SPI_INT_FL_FAULT MXC_F_SPI_INT_FL_FAULT
#define ADI_MAX32_SPI_INT_FL_SSD MXC_F_SPI_INT_FL_SSD
#define ADI_MAX32_SPI_INT_FL_SSA MXC_F_SPI_INT_FL_SSA
#define ADI_MAX32_SPI_INT_FL_RX_THD MXC_F_SPI_INT_FL_RX_THRESH
#define ADI_MAX32_SPI_INT_FL_TX_EMPTY MXC_F_SPI_INT_FL_TX_EMPTY
#define ADI_MAX32_SPI_INT_FL_TX_THD MXC_F_SPI_INT_FL_TX_THRESH

#define ADI_MAX32_SPI_INT_EN_RX_UN MXC_F_SPI_INT_EN_RX_UN
#define ADI_MAX32_SPI_INT_EN_RX_OV MXC_F_SPI_INT_EN_RX_OV
#define ADI_MAX32_SPI_INT_EN_TX_UN MXC_F_SPI_INT_EN_TX_UN
#define ADI_MAX32_SPI_INT_EN_TX_OV MXC_F_SPI_INT_EN_TX_OV
#define ADI_MAX32_SPI_INT_EN_MST_DONE MXC_F_SPI_INT_EN_M_DONE
#define ADI_MAX32_SPI_INT_EN_ABORT MXC_F_SPI_INT_EN_ABORT
#define ADI_MAX32_SPI_INT_EN_FAULT MXC_F_SPI_INT_EN_FAULT
#define ADI_MAX32_SPI_INT_EN_SSD MXC_F_SPI_INT_EN_SSD
#define ADI_MAX32_SPI_INT_EN_SSA MXC_F_SPI_INT_EN_SSA
#define ADI_MAX32_SPI_INT_EN_RX_THD MXC_F_SPI_INT_EN_RX_THRESH
#define ADI_MAX32_SPI_INT_EN_TX_EMPTY MXC_F_SPI_INT_EN_TX_EMPTY
#define ADI_MAX32_SPI_INT_EN_TX_THD MXC_F_SPI_INT_EN_TX_THRESH

#define ADI_MAX32_SPI_DMA_TX_FIFO_CLEAR MXC_F_SPI_DMA_TX_FIFO_CLEAR
#define ADI_MAX32_SPI_DMA_TX_DMA_EN MXC_F_SPI_DMA_TX_DMA_EN
#define ADI_MAX32_SPI_DMA_RX_FIFO_CLEAR MXC_F_SPI_DMA_RX_FIFO_CLEAR
#define ADI_MAX32_SPI_DMA_RX_DMA_EN MXC_F_SPI_DMA_RX_DMA_EN

static inline int Wrap_MXC_SPI_Init(mxc_spi_regs_t *spi, int masterMode, int quadModeUsed,
                                    int numSlaves, unsigned ssPolarity, unsigned int hz)
{
    return MXC_SPI_Init(spi, masterMode, quadModeUsed, numSlaves, ssPolarity, hz, (sys_map_t)0);
}

/*
 *  MAX32690, MAX32655 related mapping
 */
#elif defined(CONFIG_SOC_MAX32690) || (CONFIG_SOC_MAX32655) || (CONFIG_SOC_MAX32670) || \
    (CONFIG_SOC_MAX32672) || (CONFIG_SOC_MAX32662) || (CONFIG_SOC_MAX32675) ||          \
    (CONFIG_SOC_MAX32680) || (CONFIG_SOC_MAX32657)
#if defined(CONFIG_SOC_MAX32657)
#define ADI_MAX32_SPI_CTRL_MASTER_MODE MXC_F_SPI_CTRL0_CONT_MODE
#else
#define ADI_MAX32_SPI_CTRL_MASTER_MODE MXC_F_SPI_CTRL0_MST_MODE
#endif
#define ADI_MAX32_SPI_INT_FL_RX_UN MXC_F_SPI_INTFL_RX_UN
#define ADI_MAX32_SPI_INT_FL_RX_OV MXC_F_SPI_INTFL_RX_OV
#define ADI_MAX32_SPI_INT_FL_TX_UN MXC_F_SPI_INTFL_TX_UN
#define ADI_MAX32_SPI_INT_FL_TX_OV MXC_F_SPI_INTFL_TX_OV
#if defined(CONFIG_SOC_MAX32657)
#define ADI_MAX32_SPI_INT_FL_MST_DONE MXC_F_SPI_INTFL_CONT_DONE
#else
#define ADI_MAX32_SPI_INT_FL_MST_DONE MXC_F_SPI_INTFL_MST_DONE
#endif
#define ADI_MAX32_SPI_INT_FL_ABORT MXC_F_SPI_INTFL_ABORT
#define ADI_MAX32_SPI_INT_FL_FAULT MXC_F_SPI_INTFL_FAULT
#define ADI_MAX32_SPI_INT_FL_SSD MXC_F_SPI_INTFL_SSD
#define ADI_MAX32_SPI_INT_FL_SSA MXC_F_SPI_INTFL_SSA
#define ADI_MAX32_SPI_INT_FL_RX_THD MXC_F_SPI_INTFL_RX_THD
#define ADI_MAX32_SPI_INT_FL_TX_EMPTY MXC_F_SPI_INTFL_TX_EMPTY
#define ADI_MAX32_SPI_INT_FL_TX_THD MXC_F_SPI_INTFL_TX_THD

#define ADI_MAX32_SPI_INT_EN_RX_UN MXC_F_SPI_INTEN_RX_UN
#define ADI_MAX32_SPI_INT_EN_RX_OV MXC_F_SPI_INTEN_RX_OV
#define ADI_MAX32_SPI_INT_EN_TX_UN MXC_F_SPI_INTEN_TX_UN
#define ADI_MAX32_SPI_INT_EN_TX_OV MXC_F_SPI_INTEN_TX_OV
#if defined(CONFIG_SOC_MAX32657)
#define ADI_MAX32_SPI_INT_EN_MST_DONE MXC_F_SPI_INTEN_CONT_DONE
#else
#define ADI_MAX32_SPI_INT_EN_MST_DONE MXC_F_SPI_INTEN_MST_DONE
#endif
#define ADI_MAX32_SPI_INT_EN_ABORT MXC_F_SPI_INTEN_ABORT
#define ADI_MAX32_SPI_INT_EN_FAULT MXC_F_SPI_INTEN_FAULT
#define ADI_MAX32_SPI_INT_EN_SSD MXC_F_SPI_INTEN_SSD
#define ADI_MAX32_SPI_INT_EN_SSA MXC_F_SPI_INTEN_SSA
#define ADI_MAX32_SPI_INT_EN_RX_THD MXC_F_SPI_INTEN_RX_THD
#define ADI_MAX32_SPI_INT_EN_TX_EMPTY MXC_F_SPI_INTEN_TX_EMPTY
#define ADI_MAX32_SPI_INT_EN_TX_THD MXC_F_SPI_INTEN_TX_THD

#define ADI_MAX32_SPI_DMA_TX_FIFO_CLEAR MXC_F_SPI_DMA_TX_FLUSH
#define ADI_MAX32_SPI_DMA_TX_DMA_EN MXC_F_SPI_DMA_DMA_TX_EN
#define ADI_MAX32_SPI_DMA_RX_FIFO_CLEAR MXC_F_SPI_DMA_RX_FLUSH
#define ADI_MAX32_SPI_DMA_RX_DMA_EN MXC_F_SPI_DMA_DMA_RX_EN

static inline int Wrap_MXC_SPI_Init(mxc_spi_regs_t *spi, int masterMode, int quadModeUsed,
                                    int numSlaves, unsigned ssPolarity, unsigned int hz)
{
#if defined(CONFIG_SOC_MAX32670) || (CONFIG_SOC_MAX32672) || (CONFIG_SOC_MAX32675)
    return MXC_SPI_Init(spi, masterMode, quadModeUsed, numSlaves, ssPolarity, hz);
#else
    mxc_spi_pins_t tmp; // not used
    return MXC_SPI_Init(spi, masterMode, quadModeUsed, numSlaves, ssPolarity, hz, tmp);
#endif
}

#endif // part number

#if defined(CONFIG_SOC_MAX32657)
#define ADI_MAX32_SPI_CTRL0_SS_CTRL MXC_F_SPI_CTRL0_TS_CTRL
#else
#define ADI_MAX32_SPI_CTRL0_SS_CTRL MXC_F_SPI_CTRL0_SS_CTRL
#endif

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_ZEPHYR_MAX_INCLUDE_WRAP_MAX32_SPI_H_
