/**
 * @file    dma_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the DMA Peripheral Module.
 * @note    This file is @generated.
 * @ingroup dma_registers
 */

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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_DMA_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_DMA_REGS_H_

/* **** Includes **** */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined (__ICCARM__)
  #pragma system_include
#endif

#if defined (__CC_ARM)
  #pragma anon_unions
#endif
/// @cond
/*
    If types are not defined elsewhere (CMSIS) define them here
*/
#ifndef __IO
#define __IO volatile
#endif
#ifndef __I
#define __I  volatile const
#endif
#ifndef __O
#define __O  volatile
#endif
#ifndef __R
#define __R  volatile const
#endif
/// @endcond

/* **** Definitions **** */

/**
 * @ingroup     dma
 * @defgroup    dma_registers DMA_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the DMA Peripheral Module.
 * @details     DMA Controller Fully programmable, chaining capable DMA channels.
 */

/**
 * @ingroup dma_registers
 * Structure type to access the DMA Registers.
 */
typedef struct {
    __IO uint32_t cfg;                  /**< <tt>\b 0x000:</tt> DMA CFG Register */
    __IO uint32_t st;                   /**< <tt>\b 0x004:</tt> DMA ST Register */
    __IO uint32_t src;                  /**< <tt>\b 0x008:</tt> DMA SRC Register */
    __IO uint32_t dst;                  /**< <tt>\b 0x00C:</tt> DMA DST Register */
    __IO uint32_t cnt;                  /**< <tt>\b 0x010:</tt> DMA CNT Register */
    __IO uint32_t src_rld;              /**< <tt>\b 0x014:</tt> DMA SRC_RLD Register */
    __IO uint32_t dst_rld;              /**< <tt>\b 0x018:</tt> DMA DST_RLD Register */
    __IO uint32_t cnt_rld;              /**< <tt>\b 0x01C:</tt> DMA CNT_RLD Register */
} mxc_dma_ch_regs_t;

typedef struct {
    __IO uint32_t cn;                   /**< <tt>\b 0x000:</tt> DMA CN Register */
    __I  uint32_t intr;                 /**< <tt>\b 0x004:</tt> DMA INTR Register */
    __R  uint32_t rsv_0x8_0xff[62];
    __IO mxc_dma_ch_regs_t    ch[8];    /**< <tt>\b 0x100:</tt> DMA CH Register */
} mxc_dma_regs_t;

/* Register offsets for module DMA */
/**
 * @ingroup    dma_registers
 * @defgroup   DMA_Register_Offsets Register Offsets
 * @brief      DMA Peripheral Register Offsets from the DMA Base Peripheral Address.
 * @{
 */
#define MXC_R_DMA_CFG                      ((uint32_t)0x00000000UL) /**< Offset from DMA Base Address: <tt> 0x0000</tt> */
#define MXC_R_DMA_ST                       ((uint32_t)0x00000004UL) /**< Offset from DMA Base Address: <tt> 0x0004</tt> */
#define MXC_R_DMA_SRC                      ((uint32_t)0x00000008UL) /**< Offset from DMA Base Address: <tt> 0x0008</tt> */
#define MXC_R_DMA_DST                      ((uint32_t)0x0000000CUL) /**< Offset from DMA Base Address: <tt> 0x000C</tt> */
#define MXC_R_DMA_CNT                      ((uint32_t)0x00000010UL) /**< Offset from DMA Base Address: <tt> 0x0010</tt> */
#define MXC_R_DMA_SRC_RLD                  ((uint32_t)0x00000014UL) /**< Offset from DMA Base Address: <tt> 0x0014</tt> */
#define MXC_R_DMA_DST_RLD                  ((uint32_t)0x00000018UL) /**< Offset from DMA Base Address: <tt> 0x0018</tt> */
#define MXC_R_DMA_CNT_RLD                  ((uint32_t)0x0000001CUL) /**< Offset from DMA Base Address: <tt> 0x001C</tt> */
#define MXC_R_DMA_CN                       ((uint32_t)0x00000000UL) /**< Offset from DMA Base Address: <tt> 0x0000</tt> */
#define MXC_R_DMA_INTR                     ((uint32_t)0x00000004UL) /**< Offset from DMA Base Address: <tt> 0x0004</tt> */
#define MXC_R_DMA_CH                       ((uint32_t)0x00000100UL) /**< Offset from DMA Base Address: <tt> 0x0100</tt> */
/**@} end of group dma_registers */

/**
 * @ingroup  dma_registers
 * @defgroup DMA_CN DMA_CN
 * @brief    DMA Control Register.
 * @{
 */
#define MXC_F_DMA_CN_CH0_IEN_POS                       0 /**< CN_CH0_IEN Position */
#define MXC_F_DMA_CN_CH0_IEN                           ((uint32_t)(0x1UL << MXC_F_DMA_CN_CH0_IEN_POS)) /**< CN_CH0_IEN Mask */

#define MXC_F_DMA_CN_CH1_IEN_POS                       1 /**< CN_CH1_IEN Position */
#define MXC_F_DMA_CN_CH1_IEN                           ((uint32_t)(0x1UL << MXC_F_DMA_CN_CH1_IEN_POS)) /**< CN_CH1_IEN Mask */

#define MXC_F_DMA_CN_CH2_IEN_POS                       2 /**< CN_CH2_IEN Position */
#define MXC_F_DMA_CN_CH2_IEN                           ((uint32_t)(0x1UL << MXC_F_DMA_CN_CH2_IEN_POS)) /**< CN_CH2_IEN Mask */

#define MXC_F_DMA_CN_CH3_IEN_POS                       3 /**< CN_CH3_IEN Position */
#define MXC_F_DMA_CN_CH3_IEN                           ((uint32_t)(0x1UL << MXC_F_DMA_CN_CH3_IEN_POS)) /**< CN_CH3_IEN Mask */

#define MXC_F_DMA_CN_CH4_IEN_POS                       4 /**< CN_CH4_IEN Position */
#define MXC_F_DMA_CN_CH4_IEN                           ((uint32_t)(0x1UL << MXC_F_DMA_CN_CH4_IEN_POS)) /**< CN_CH4_IEN Mask */

#define MXC_F_DMA_CN_CH5_IEN_POS                       5 /**< CN_CH5_IEN Position */
#define MXC_F_DMA_CN_CH5_IEN                           ((uint32_t)(0x1UL << MXC_F_DMA_CN_CH5_IEN_POS)) /**< CN_CH5_IEN Mask */

#define MXC_F_DMA_CN_CH6_IEN_POS                       6 /**< CN_CH6_IEN Position */
#define MXC_F_DMA_CN_CH6_IEN                           ((uint32_t)(0x1UL << MXC_F_DMA_CN_CH6_IEN_POS)) /**< CN_CH6_IEN Mask */

#define MXC_F_DMA_CN_CH7_IEN_POS                       7 /**< CN_CH7_IEN Position */
#define MXC_F_DMA_CN_CH7_IEN                           ((uint32_t)(0x1UL << MXC_F_DMA_CN_CH7_IEN_POS)) /**< CN_CH7_IEN Mask */

/**@} end of group DMA_CN_Register */

/**
 * @ingroup  dma_registers
 * @defgroup DMA_INTR DMA_INTR
 * @brief    DMA Interrupt Register.
 * @{
 */
#define MXC_F_DMA_INTR_CH0_IPEND_POS                   0 /**< INTR_CH0_IPEND Position */
#define MXC_F_DMA_INTR_CH0_IPEND                       ((uint32_t)(0x1UL << MXC_F_DMA_INTR_CH0_IPEND_POS)) /**< INTR_CH0_IPEND Mask */

#define MXC_F_DMA_INTR_CH1_IPEND_POS                   1 /**< INTR_CH1_IPEND Position */
#define MXC_F_DMA_INTR_CH1_IPEND                       ((uint32_t)(0x1UL << MXC_F_DMA_INTR_CH1_IPEND_POS)) /**< INTR_CH1_IPEND Mask */

#define MXC_F_DMA_INTR_CH2_IPEND_POS                   2 /**< INTR_CH2_IPEND Position */
#define MXC_F_DMA_INTR_CH2_IPEND                       ((uint32_t)(0x1UL << MXC_F_DMA_INTR_CH2_IPEND_POS)) /**< INTR_CH2_IPEND Mask */

#define MXC_F_DMA_INTR_CH3_IPEND_POS                   3 /**< INTR_CH3_IPEND Position */
#define MXC_F_DMA_INTR_CH3_IPEND                       ((uint32_t)(0x1UL << MXC_F_DMA_INTR_CH3_IPEND_POS)) /**< INTR_CH3_IPEND Mask */

#define MXC_F_DMA_INTR_CH4_IPEND_POS                   4 /**< INTR_CH4_IPEND Position */
#define MXC_F_DMA_INTR_CH4_IPEND                       ((uint32_t)(0x1UL << MXC_F_DMA_INTR_CH4_IPEND_POS)) /**< INTR_CH4_IPEND Mask */

#define MXC_F_DMA_INTR_CH5_IPEND_POS                   5 /**< INTR_CH5_IPEND Position */
#define MXC_F_DMA_INTR_CH5_IPEND                       ((uint32_t)(0x1UL << MXC_F_DMA_INTR_CH5_IPEND_POS)) /**< INTR_CH5_IPEND Mask */

#define MXC_F_DMA_INTR_CH6_IPEND_POS                   6 /**< INTR_CH6_IPEND Position */
#define MXC_F_DMA_INTR_CH6_IPEND                       ((uint32_t)(0x1UL << MXC_F_DMA_INTR_CH6_IPEND_POS)) /**< INTR_CH6_IPEND Mask */

#define MXC_F_DMA_INTR_CH7_IPEND_POS                   7 /**< INTR_CH7_IPEND Position */
#define MXC_F_DMA_INTR_CH7_IPEND                       ((uint32_t)(0x1UL << MXC_F_DMA_INTR_CH7_IPEND_POS)) /**< INTR_CH7_IPEND Mask */

/**@} end of group DMA_INTR_Register */

/**
 * @ingroup  dma_registers
 * @defgroup DMA_CFG DMA_CFG
 * @brief    DMA Channel Configuration Register.
 * @{
 */
#define MXC_F_DMA_CFG_CHEN_POS                         0 /**< CFG_CHEN Position */
#define MXC_F_DMA_CFG_CHEN                             ((uint32_t)(0x1UL << MXC_F_DMA_CFG_CHEN_POS)) /**< CFG_CHEN Mask */

#define MXC_F_DMA_CFG_RLDEN_POS                        1 /**< CFG_RLDEN Position */
#define MXC_F_DMA_CFG_RLDEN                            ((uint32_t)(0x1UL << MXC_F_DMA_CFG_RLDEN_POS)) /**< CFG_RLDEN Mask */

#define MXC_F_DMA_CFG_PRI_POS                          2 /**< CFG_PRI Position */
#define MXC_F_DMA_CFG_PRI                              ((uint32_t)(0x3UL << MXC_F_DMA_CFG_PRI_POS)) /**< CFG_PRI Mask */
#define MXC_V_DMA_CFG_PRI_HIGH                         ((uint32_t)0x0UL) /**< CFG_PRI_HIGH Value */
#define MXC_S_DMA_CFG_PRI_HIGH                         (MXC_V_DMA_CFG_PRI_HIGH << MXC_F_DMA_CFG_PRI_POS) /**< CFG_PRI_HIGH Setting */
#define MXC_V_DMA_CFG_PRI_MEDHIGH                      ((uint32_t)0x1UL) /**< CFG_PRI_MEDHIGH Value */
#define MXC_S_DMA_CFG_PRI_MEDHIGH                      (MXC_V_DMA_CFG_PRI_MEDHIGH << MXC_F_DMA_CFG_PRI_POS) /**< CFG_PRI_MEDHIGH Setting */
#define MXC_V_DMA_CFG_PRI_MEDLOW                       ((uint32_t)0x2UL) /**< CFG_PRI_MEDLOW Value */
#define MXC_S_DMA_CFG_PRI_MEDLOW                       (MXC_V_DMA_CFG_PRI_MEDLOW << MXC_F_DMA_CFG_PRI_POS) /**< CFG_PRI_MEDLOW Setting */
#define MXC_V_DMA_CFG_PRI_LOW                          ((uint32_t)0x3UL) /**< CFG_PRI_LOW Value */
#define MXC_S_DMA_CFG_PRI_LOW                          (MXC_V_DMA_CFG_PRI_LOW << MXC_F_DMA_CFG_PRI_POS) /**< CFG_PRI_LOW Setting */

#define MXC_F_DMA_CFG_REQSEL_POS                       4 /**< CFG_REQSEL Position */
#define MXC_F_DMA_CFG_REQSEL                           ((uint32_t)(0x3FUL << MXC_F_DMA_CFG_REQSEL_POS)) /**< CFG_REQSEL Mask */
#define MXC_V_DMA_CFG_REQSEL_MEMTOMEM                  ((uint32_t)0x0UL) /**< CFG_REQSEL_MEMTOMEM Value */
#define MXC_S_DMA_CFG_REQSEL_MEMTOMEM                  (MXC_V_DMA_CFG_REQSEL_MEMTOMEM << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_MEMTOMEM Setting */
#define MXC_V_DMA_CFG_REQSEL_SPI1RX                    ((uint32_t)0x1UL) /**< CFG_REQSEL_SPI1RX Value */
#define MXC_S_DMA_CFG_REQSEL_SPI1RX                    (MXC_V_DMA_CFG_REQSEL_SPI1RX << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_SPI1RX Setting */
#define MXC_V_DMA_CFG_REQSEL_SPI2RX                    ((uint32_t)0x2UL) /**< CFG_REQSEL_SPI2RX Value */
#define MXC_S_DMA_CFG_REQSEL_SPI2RX                    (MXC_V_DMA_CFG_REQSEL_SPI2RX << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_SPI2RX Setting */
#define MXC_V_DMA_CFG_REQSEL_UART0RX                   ((uint32_t)0x4UL) /**< CFG_REQSEL_UART0RX Value */
#define MXC_S_DMA_CFG_REQSEL_UART0RX                   (MXC_V_DMA_CFG_REQSEL_UART0RX << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_UART0RX Setting */
#define MXC_V_DMA_CFG_REQSEL_UART1RX                   ((uint32_t)0x5UL) /**< CFG_REQSEL_UART1RX Value */
#define MXC_S_DMA_CFG_REQSEL_UART1RX                   (MXC_V_DMA_CFG_REQSEL_UART1RX << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_UART1RX Setting */
#define MXC_V_DMA_CFG_REQSEL_I2C0RX                    ((uint32_t)0x7UL) /**< CFG_REQSEL_I2C0RX Value */
#define MXC_S_DMA_CFG_REQSEL_I2C0RX                    (MXC_V_DMA_CFG_REQSEL_I2C0RX << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_I2C0RX Setting */
#define MXC_V_DMA_CFG_REQSEL_I2C1RX                    ((uint32_t)0x8UL) /**< CFG_REQSEL_I2C1RX Value */
#define MXC_S_DMA_CFG_REQSEL_I2C1RX                    (MXC_V_DMA_CFG_REQSEL_I2C1RX << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_I2C1RX Setting */
#define MXC_V_DMA_CFG_REQSEL_ADC                       ((uint32_t)0x9UL) /**< CFG_REQSEL_ADC Value */
#define MXC_S_DMA_CFG_REQSEL_ADC                       (MXC_V_DMA_CFG_REQSEL_ADC << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_ADC Setting */
#define MXC_V_DMA_CFG_REQSEL_I2C2RX                    ((uint32_t)0xAUL) /**< CFG_REQSEL_I2C2RX Value */
#define MXC_S_DMA_CFG_REQSEL_I2C2RX                    (MXC_V_DMA_CFG_REQSEL_I2C2RX << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_I2C2RX Setting */
#define MXC_V_DMA_CFG_REQSEL_UART2RX                   ((uint32_t)0xEUL) /**< CFG_REQSEL_UART2RX Value */
#define MXC_S_DMA_CFG_REQSEL_UART2RX                   (MXC_V_DMA_CFG_REQSEL_UART2RX << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_UART2RX Setting */
#define MXC_V_DMA_CFG_REQSEL_SPI0RX                    ((uint32_t)0xFUL) /**< CFG_REQSEL_SPI0RX Value */
#define MXC_S_DMA_CFG_REQSEL_SPI0RX                    (MXC_V_DMA_CFG_REQSEL_SPI0RX << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_SPI0RX Setting */
#define MXC_V_DMA_CFG_REQSEL_USBRXEP1                  ((uint32_t)0x11UL) /**< CFG_REQSEL_USBRXEP1 Value */
#define MXC_S_DMA_CFG_REQSEL_USBRXEP1                  (MXC_V_DMA_CFG_REQSEL_USBRXEP1 << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_USBRXEP1 Setting */
#define MXC_V_DMA_CFG_REQSEL_USBRXEP2                  ((uint32_t)0x12UL) /**< CFG_REQSEL_USBRXEP2 Value */
#define MXC_S_DMA_CFG_REQSEL_USBRXEP2                  (MXC_V_DMA_CFG_REQSEL_USBRXEP2 << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_USBRXEP2 Setting */
#define MXC_V_DMA_CFG_REQSEL_USBRXEP3                  ((uint32_t)0x13UL) /**< CFG_REQSEL_USBRXEP3 Value */
#define MXC_S_DMA_CFG_REQSEL_USBRXEP3                  (MXC_V_DMA_CFG_REQSEL_USBRXEP3 << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_USBRXEP3 Setting */
#define MXC_V_DMA_CFG_REQSEL_USBRXEP4                  ((uint32_t)0x14UL) /**< CFG_REQSEL_USBRXEP4 Value */
#define MXC_S_DMA_CFG_REQSEL_USBRXEP4                  (MXC_V_DMA_CFG_REQSEL_USBRXEP4 << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_USBRXEP4 Setting */
#define MXC_V_DMA_CFG_REQSEL_USBRXEP5                  ((uint32_t)0x15UL) /**< CFG_REQSEL_USBRXEP5 Value */
#define MXC_S_DMA_CFG_REQSEL_USBRXEP5                  (MXC_V_DMA_CFG_REQSEL_USBRXEP5 << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_USBRXEP5 Setting */
#define MXC_V_DMA_CFG_REQSEL_USBRXEP6                  ((uint32_t)0x16UL) /**< CFG_REQSEL_USBRXEP6 Value */
#define MXC_S_DMA_CFG_REQSEL_USBRXEP6                  (MXC_V_DMA_CFG_REQSEL_USBRXEP6 << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_USBRXEP6 Setting */
#define MXC_V_DMA_CFG_REQSEL_USBRXEP7                  ((uint32_t)0x17UL) /**< CFG_REQSEL_USBRXEP7 Value */
#define MXC_S_DMA_CFG_REQSEL_USBRXEP7                  (MXC_V_DMA_CFG_REQSEL_USBRXEP7 << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_USBRXEP7 Setting */
#define MXC_V_DMA_CFG_REQSEL_USBRXEP8                  ((uint32_t)0x18UL) /**< CFG_REQSEL_USBRXEP8 Value */
#define MXC_S_DMA_CFG_REQSEL_USBRXEP8                  (MXC_V_DMA_CFG_REQSEL_USBRXEP8 << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_USBRXEP8 Setting */
#define MXC_V_DMA_CFG_REQSEL_USBRXEP9                  ((uint32_t)0x19UL) /**< CFG_REQSEL_USBRXEP9 Value */
#define MXC_S_DMA_CFG_REQSEL_USBRXEP9                  (MXC_V_DMA_CFG_REQSEL_USBRXEP9 << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_USBRXEP9 Setting */
#define MXC_V_DMA_CFG_REQSEL_USBRXEP10                 ((uint32_t)0x1AUL) /**< CFG_REQSEL_USBRXEP10 Value */
#define MXC_S_DMA_CFG_REQSEL_USBRXEP10                 (MXC_V_DMA_CFG_REQSEL_USBRXEP10 << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_USBRXEP10 Setting */
#define MXC_V_DMA_CFG_REQSEL_USBRXEP11                 ((uint32_t)0x1BUL) /**< CFG_REQSEL_USBRXEP11 Value */
#define MXC_S_DMA_CFG_REQSEL_USBRXEP11                 (MXC_V_DMA_CFG_REQSEL_USBRXEP11 << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_USBRXEP11 Setting */
#define MXC_V_DMA_CFG_REQSEL_SPI1TX                    ((uint32_t)0x21UL) /**< CFG_REQSEL_SPI1TX Value */
#define MXC_S_DMA_CFG_REQSEL_SPI1TX                    (MXC_V_DMA_CFG_REQSEL_SPI1TX << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_SPI1TX Setting */
#define MXC_V_DMA_CFG_REQSEL_SPI2TX                    ((uint32_t)0x22UL) /**< CFG_REQSEL_SPI2TX Value */
#define MXC_S_DMA_CFG_REQSEL_SPI2TX                    (MXC_V_DMA_CFG_REQSEL_SPI2TX << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_SPI2TX Setting */
#define MXC_V_DMA_CFG_REQSEL_UART0TX                   ((uint32_t)0x24UL) /**< CFG_REQSEL_UART0TX Value */
#define MXC_S_DMA_CFG_REQSEL_UART0TX                   (MXC_V_DMA_CFG_REQSEL_UART0TX << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_UART0TX Setting */
#define MXC_V_DMA_CFG_REQSEL_UART1TX                   ((uint32_t)0x25UL) /**< CFG_REQSEL_UART1TX Value */
#define MXC_S_DMA_CFG_REQSEL_UART1TX                   (MXC_V_DMA_CFG_REQSEL_UART1TX << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_UART1TX Setting */
#define MXC_V_DMA_CFG_REQSEL_I2C0TX                    ((uint32_t)0x27UL) /**< CFG_REQSEL_I2C0TX Value */
#define MXC_S_DMA_CFG_REQSEL_I2C0TX                    (MXC_V_DMA_CFG_REQSEL_I2C0TX << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_I2C0TX Setting */
#define MXC_V_DMA_CFG_REQSEL_I2C1TX                    ((uint32_t)0x28UL) /**< CFG_REQSEL_I2C1TX Value */
#define MXC_S_DMA_CFG_REQSEL_I2C1TX                    (MXC_V_DMA_CFG_REQSEL_I2C1TX << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_I2C1TX Setting */
#define MXC_V_DMA_CFG_REQSEL_I2C2TX                    ((uint32_t)0x2AUL) /**< CFG_REQSEL_I2C2TX Value */
#define MXC_S_DMA_CFG_REQSEL_I2C2TX                    (MXC_V_DMA_CFG_REQSEL_I2C2TX << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_I2C2TX Setting */
#define MXC_V_DMA_CFG_REQSEL_UART2TX                   ((uint32_t)0x2EUL) /**< CFG_REQSEL_UART2TX Value */
#define MXC_S_DMA_CFG_REQSEL_UART2TX                   (MXC_V_DMA_CFG_REQSEL_UART2TX << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_UART2TX Setting */
#define MXC_V_DMA_CFG_REQSEL_SPI0TX                    ((uint32_t)0x2FUL) /**< CFG_REQSEL_SPI0TX Value */
#define MXC_S_DMA_CFG_REQSEL_SPI0TX                    (MXC_V_DMA_CFG_REQSEL_SPI0TX << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_SPI0TX Setting */
#define MXC_V_DMA_CFG_REQSEL_USBTXEP1                  ((uint32_t)0x31UL) /**< CFG_REQSEL_USBTXEP1 Value */
#define MXC_S_DMA_CFG_REQSEL_USBTXEP1                  (MXC_V_DMA_CFG_REQSEL_USBTXEP1 << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_USBTXEP1 Setting */
#define MXC_V_DMA_CFG_REQSEL_USBTXEP2                  ((uint32_t)0x32UL) /**< CFG_REQSEL_USBTXEP2 Value */
#define MXC_S_DMA_CFG_REQSEL_USBTXEP2                  (MXC_V_DMA_CFG_REQSEL_USBTXEP2 << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_USBTXEP2 Setting */
#define MXC_V_DMA_CFG_REQSEL_USBTXEP3                  ((uint32_t)0x33UL) /**< CFG_REQSEL_USBTXEP3 Value */
#define MXC_S_DMA_CFG_REQSEL_USBTXEP3                  (MXC_V_DMA_CFG_REQSEL_USBTXEP3 << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_USBTXEP3 Setting */
#define MXC_V_DMA_CFG_REQSEL_USBTXEP4                  ((uint32_t)0x34UL) /**< CFG_REQSEL_USBTXEP4 Value */
#define MXC_S_DMA_CFG_REQSEL_USBTXEP4                  (MXC_V_DMA_CFG_REQSEL_USBTXEP4 << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_USBTXEP4 Setting */
#define MXC_V_DMA_CFG_REQSEL_USBTXEP5                  ((uint32_t)0x35UL) /**< CFG_REQSEL_USBTXEP5 Value */
#define MXC_S_DMA_CFG_REQSEL_USBTXEP5                  (MXC_V_DMA_CFG_REQSEL_USBTXEP5 << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_USBTXEP5 Setting */
#define MXC_V_DMA_CFG_REQSEL_USBTXEP6                  ((uint32_t)0x36UL) /**< CFG_REQSEL_USBTXEP6 Value */
#define MXC_S_DMA_CFG_REQSEL_USBTXEP6                  (MXC_V_DMA_CFG_REQSEL_USBTXEP6 << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_USBTXEP6 Setting */
#define MXC_V_DMA_CFG_REQSEL_USBTXEP7                  ((uint32_t)0x37UL) /**< CFG_REQSEL_USBTXEP7 Value */
#define MXC_S_DMA_CFG_REQSEL_USBTXEP7                  (MXC_V_DMA_CFG_REQSEL_USBTXEP7 << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_USBTXEP7 Setting */
#define MXC_V_DMA_CFG_REQSEL_USBTXEP8                  ((uint32_t)0x38UL) /**< CFG_REQSEL_USBTXEP8 Value */
#define MXC_S_DMA_CFG_REQSEL_USBTXEP8                  (MXC_V_DMA_CFG_REQSEL_USBTXEP8 << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_USBTXEP8 Setting */
#define MXC_V_DMA_CFG_REQSEL_USBTXEP9                  ((uint32_t)0x39UL) /**< CFG_REQSEL_USBTXEP9 Value */
#define MXC_S_DMA_CFG_REQSEL_USBTXEP9                  (MXC_V_DMA_CFG_REQSEL_USBTXEP9 << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_USBTXEP9 Setting */
#define MXC_V_DMA_CFG_REQSEL_USBTXEP10                 ((uint32_t)0x3AUL) /**< CFG_REQSEL_USBTXEP10 Value */
#define MXC_S_DMA_CFG_REQSEL_USBTXEP10                 (MXC_V_DMA_CFG_REQSEL_USBTXEP10 << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_USBTXEP10 Setting */
#define MXC_V_DMA_CFG_REQSEL_USBTXEP11                 ((uint32_t)0x3BUL) /**< CFG_REQSEL_USBTXEP11 Value */
#define MXC_S_DMA_CFG_REQSEL_USBTXEP11                 (MXC_V_DMA_CFG_REQSEL_USBTXEP11 << MXC_F_DMA_CFG_REQSEL_POS) /**< CFG_REQSEL_USBTXEP11 Setting */

#define MXC_F_DMA_CFG_REQWAIT_POS                      10 /**< CFG_REQWAIT Position */
#define MXC_F_DMA_CFG_REQWAIT                          ((uint32_t)(0x1UL << MXC_F_DMA_CFG_REQWAIT_POS)) /**< CFG_REQWAIT Mask */

#define MXC_F_DMA_CFG_TOSEL_POS                        11 /**< CFG_TOSEL Position */
#define MXC_F_DMA_CFG_TOSEL                            ((uint32_t)(0x7UL << MXC_F_DMA_CFG_TOSEL_POS)) /**< CFG_TOSEL Mask */
#define MXC_V_DMA_CFG_TOSEL_TO4                        ((uint32_t)0x0UL) /**< CFG_TOSEL_TO4 Value */
#define MXC_S_DMA_CFG_TOSEL_TO4                        (MXC_V_DMA_CFG_TOSEL_TO4 << MXC_F_DMA_CFG_TOSEL_POS) /**< CFG_TOSEL_TO4 Setting */
#define MXC_V_DMA_CFG_TOSEL_TO8                        ((uint32_t)0x1UL) /**< CFG_TOSEL_TO8 Value */
#define MXC_S_DMA_CFG_TOSEL_TO8                        (MXC_V_DMA_CFG_TOSEL_TO8 << MXC_F_DMA_CFG_TOSEL_POS) /**< CFG_TOSEL_TO8 Setting */
#define MXC_V_DMA_CFG_TOSEL_TO16                       ((uint32_t)0x2UL) /**< CFG_TOSEL_TO16 Value */
#define MXC_S_DMA_CFG_TOSEL_TO16                       (MXC_V_DMA_CFG_TOSEL_TO16 << MXC_F_DMA_CFG_TOSEL_POS) /**< CFG_TOSEL_TO16 Setting */
#define MXC_V_DMA_CFG_TOSEL_TO32                       ((uint32_t)0x3UL) /**< CFG_TOSEL_TO32 Value */
#define MXC_S_DMA_CFG_TOSEL_TO32                       (MXC_V_DMA_CFG_TOSEL_TO32 << MXC_F_DMA_CFG_TOSEL_POS) /**< CFG_TOSEL_TO32 Setting */
#define MXC_V_DMA_CFG_TOSEL_TO64                       ((uint32_t)0x4UL) /**< CFG_TOSEL_TO64 Value */
#define MXC_S_DMA_CFG_TOSEL_TO64                       (MXC_V_DMA_CFG_TOSEL_TO64 << MXC_F_DMA_CFG_TOSEL_POS) /**< CFG_TOSEL_TO64 Setting */
#define MXC_V_DMA_CFG_TOSEL_TO128                      ((uint32_t)0x5UL) /**< CFG_TOSEL_TO128 Value */
#define MXC_S_DMA_CFG_TOSEL_TO128                      (MXC_V_DMA_CFG_TOSEL_TO128 << MXC_F_DMA_CFG_TOSEL_POS) /**< CFG_TOSEL_TO128 Setting */
#define MXC_V_DMA_CFG_TOSEL_TO256                      ((uint32_t)0x6UL) /**< CFG_TOSEL_TO256 Value */
#define MXC_S_DMA_CFG_TOSEL_TO256                      (MXC_V_DMA_CFG_TOSEL_TO256 << MXC_F_DMA_CFG_TOSEL_POS) /**< CFG_TOSEL_TO256 Setting */
#define MXC_V_DMA_CFG_TOSEL_TO512                      ((uint32_t)0x7UL) /**< CFG_TOSEL_TO512 Value */
#define MXC_S_DMA_CFG_TOSEL_TO512                      (MXC_V_DMA_CFG_TOSEL_TO512 << MXC_F_DMA_CFG_TOSEL_POS) /**< CFG_TOSEL_TO512 Setting */

#define MXC_F_DMA_CFG_PSSEL_POS                        14 /**< CFG_PSSEL Position */
#define MXC_F_DMA_CFG_PSSEL                            ((uint32_t)(0x3UL << MXC_F_DMA_CFG_PSSEL_POS)) /**< CFG_PSSEL Mask */
#define MXC_V_DMA_CFG_PSSEL_DIS                        ((uint32_t)0x0UL) /**< CFG_PSSEL_DIS Value */
#define MXC_S_DMA_CFG_PSSEL_DIS                        (MXC_V_DMA_CFG_PSSEL_DIS << MXC_F_DMA_CFG_PSSEL_POS) /**< CFG_PSSEL_DIS Setting */
#define MXC_V_DMA_CFG_PSSEL_DIV256                     ((uint32_t)0x1UL) /**< CFG_PSSEL_DIV256 Value */
#define MXC_S_DMA_CFG_PSSEL_DIV256                     (MXC_V_DMA_CFG_PSSEL_DIV256 << MXC_F_DMA_CFG_PSSEL_POS) /**< CFG_PSSEL_DIV256 Setting */
#define MXC_V_DMA_CFG_PSSEL_DIV64K                     ((uint32_t)0x2UL) /**< CFG_PSSEL_DIV64K Value */
#define MXC_S_DMA_CFG_PSSEL_DIV64K                     (MXC_V_DMA_CFG_PSSEL_DIV64K << MXC_F_DMA_CFG_PSSEL_POS) /**< CFG_PSSEL_DIV64K Setting */
#define MXC_V_DMA_CFG_PSSEL_DIV16M                     ((uint32_t)0x3UL) /**< CFG_PSSEL_DIV16M Value */
#define MXC_S_DMA_CFG_PSSEL_DIV16M                     (MXC_V_DMA_CFG_PSSEL_DIV16M << MXC_F_DMA_CFG_PSSEL_POS) /**< CFG_PSSEL_DIV16M Setting */

#define MXC_F_DMA_CFG_SRCWD_POS                        16 /**< CFG_SRCWD Position */
#define MXC_F_DMA_CFG_SRCWD                            ((uint32_t)(0x3UL << MXC_F_DMA_CFG_SRCWD_POS)) /**< CFG_SRCWD Mask */
#define MXC_V_DMA_CFG_SRCWD_BYTE                       ((uint32_t)0x0UL) /**< CFG_SRCWD_BYTE Value */
#define MXC_S_DMA_CFG_SRCWD_BYTE                       (MXC_V_DMA_CFG_SRCWD_BYTE << MXC_F_DMA_CFG_SRCWD_POS) /**< CFG_SRCWD_BYTE Setting */
#define MXC_V_DMA_CFG_SRCWD_HALFWORD                   ((uint32_t)0x1UL) /**< CFG_SRCWD_HALFWORD Value */
#define MXC_S_DMA_CFG_SRCWD_HALFWORD                   (MXC_V_DMA_CFG_SRCWD_HALFWORD << MXC_F_DMA_CFG_SRCWD_POS) /**< CFG_SRCWD_HALFWORD Setting */
#define MXC_V_DMA_CFG_SRCWD_WORD                       ((uint32_t)0x2UL) /**< CFG_SRCWD_WORD Value */
#define MXC_S_DMA_CFG_SRCWD_WORD                       (MXC_V_DMA_CFG_SRCWD_WORD << MXC_F_DMA_CFG_SRCWD_POS) /**< CFG_SRCWD_WORD Setting */

#define MXC_F_DMA_CFG_SRINC_POS                        18 /**< CFG_SRINC Position */
#define MXC_F_DMA_CFG_SRINC                            ((uint32_t)(0x1UL << MXC_F_DMA_CFG_SRINC_POS)) /**< CFG_SRINC Mask */

#define MXC_F_DMA_CFG_DSTWD_POS                        20 /**< CFG_DSTWD Position */
#define MXC_F_DMA_CFG_DSTWD                            ((uint32_t)(0x3UL << MXC_F_DMA_CFG_DSTWD_POS)) /**< CFG_DSTWD Mask */
#define MXC_V_DMA_CFG_DSTWD_BYTE                       ((uint32_t)0x0UL) /**< CFG_DSTWD_BYTE Value */
#define MXC_S_DMA_CFG_DSTWD_BYTE                       (MXC_V_DMA_CFG_DSTWD_BYTE << MXC_F_DMA_CFG_DSTWD_POS) /**< CFG_DSTWD_BYTE Setting */
#define MXC_V_DMA_CFG_DSTWD_HALFWORD                   ((uint32_t)0x1UL) /**< CFG_DSTWD_HALFWORD Value */
#define MXC_S_DMA_CFG_DSTWD_HALFWORD                   (MXC_V_DMA_CFG_DSTWD_HALFWORD << MXC_F_DMA_CFG_DSTWD_POS) /**< CFG_DSTWD_HALFWORD Setting */
#define MXC_V_DMA_CFG_DSTWD_WORD                       ((uint32_t)0x2UL) /**< CFG_DSTWD_WORD Value */
#define MXC_S_DMA_CFG_DSTWD_WORD                       (MXC_V_DMA_CFG_DSTWD_WORD << MXC_F_DMA_CFG_DSTWD_POS) /**< CFG_DSTWD_WORD Setting */

#define MXC_F_DMA_CFG_DISTINC_POS                      22 /**< CFG_DISTINC Position */
#define MXC_F_DMA_CFG_DISTINC                          ((uint32_t)(0x1UL << MXC_F_DMA_CFG_DISTINC_POS)) /**< CFG_DISTINC Mask */

#define MXC_F_DMA_CFG_BRST_POS                         24 /**< CFG_BRST Position */
#define MXC_F_DMA_CFG_BRST                             ((uint32_t)(0x1FUL << MXC_F_DMA_CFG_BRST_POS)) /**< CFG_BRST Mask */

#define MXC_F_DMA_CFG_CHDIEN_POS                       30 /**< CFG_CHDIEN Position */
#define MXC_F_DMA_CFG_CHDIEN                           ((uint32_t)(0x1UL << MXC_F_DMA_CFG_CHDIEN_POS)) /**< CFG_CHDIEN Mask */

#define MXC_F_DMA_CFG_CTZIEN_POS                       31 /**< CFG_CTZIEN Position */
#define MXC_F_DMA_CFG_CTZIEN                           ((uint32_t)(0x1UL << MXC_F_DMA_CFG_CTZIEN_POS)) /**< CFG_CTZIEN Mask */

/**@} end of group DMA_CFG_Register */

/**
 * @ingroup  dma_registers
 * @defgroup DMA_ST DMA_ST
 * @brief    DMA Channel Status Register.
 * @{
 */
#define MXC_F_DMA_ST_CH_ST_POS                         0 /**< ST_CH_ST Position */
#define MXC_F_DMA_ST_CH_ST                             ((uint32_t)(0x1UL << MXC_F_DMA_ST_CH_ST_POS)) /**< ST_CH_ST Mask */

#define MXC_F_DMA_ST_IPEND_POS                         1 /**< ST_IPEND Position */
#define MXC_F_DMA_ST_IPEND                             ((uint32_t)(0x1UL << MXC_F_DMA_ST_IPEND_POS)) /**< ST_IPEND Mask */

#define MXC_F_DMA_ST_CTZ_ST_POS                        2 /**< ST_CTZ_ST Position */
#define MXC_F_DMA_ST_CTZ_ST                            ((uint32_t)(0x1UL << MXC_F_DMA_ST_CTZ_ST_POS)) /**< ST_CTZ_ST Mask */

#define MXC_F_DMA_ST_RLD_ST_POS                        3 /**< ST_RLD_ST Position */
#define MXC_F_DMA_ST_RLD_ST                            ((uint32_t)(0x1UL << MXC_F_DMA_ST_RLD_ST_POS)) /**< ST_RLD_ST Mask */

#define MXC_F_DMA_ST_BUS_ERR_POS                       4 /**< ST_BUS_ERR Position */
#define MXC_F_DMA_ST_BUS_ERR                           ((uint32_t)(0x1UL << MXC_F_DMA_ST_BUS_ERR_POS)) /**< ST_BUS_ERR Mask */

#define MXC_F_DMA_ST_TO_ST_POS                         6 /**< ST_TO_ST Position */
#define MXC_F_DMA_ST_TO_ST                             ((uint32_t)(0x1UL << MXC_F_DMA_ST_TO_ST_POS)) /**< ST_TO_ST Mask */

/**@} end of group DMA_ST_Register */

/**
 * @ingroup  dma_registers
 * @defgroup DMA_SRC DMA_SRC
 * @brief    Source Device Address. If SRCINC=1, the counter bits are incremented by 1,2, or
 *           4, depending on the data width of each AHB cycle. For peripheral transfers, some
 *           or all of the actual address bits are fixed. If SRCINC=0, this register remains
 *           constant. In the case where a count-to-zero condition occurs while RLDEN=1, the
 *           register is reloaded with the contents of DMA_SRC_RLD.
 * @{
 */
#define MXC_F_DMA_SRC_ADDR_POS                         0 /**< SRC_ADDR Position */
#define MXC_F_DMA_SRC_ADDR                             ((uint32_t)(0xFFFFFFFFUL << MXC_F_DMA_SRC_ADDR_POS)) /**< SRC_ADDR Mask */

/**@} end of group DMA_SRC_Register */

/**
 * @ingroup  dma_registers
 * @defgroup DMA_DST DMA_DST
 * @brief    Destination Device Address. For peripheral transfers, some or all of the actual
 *           address bits are fixed. If DSTINC=1, this register is incremented on every AHB
 *           write out of the DMA FIFO. They are incremented by 1, 2, or 4, depending on the
 *           data width of each AHB cycle. In the case where a count-to-zero condition occurs
 *           while RLDEN=1, the register is reloaded with DMA_DST_RLD.
 * @{
 */
#define MXC_F_DMA_DST_ADDR_POS                         0 /**< DST_ADDR Position */
#define MXC_F_DMA_DST_ADDR                             ((uint32_t)(0xFFFFFFFFUL << MXC_F_DMA_DST_ADDR_POS)) /**< DST_ADDR Mask */

/**@} end of group DMA_DST_Register */

/**
 * @ingroup  dma_registers
 * @defgroup DMA_CNT DMA_CNT
 * @brief    DMA Counter. The user loads this register with the number of bytes to transfer.
 *           This counter decreases on every AHB cycle into the DMA FIFO. The decrement will
 *           be 1, 2, or 4 depending on the data width of each AHB cycle. When the counter
 *           reaches 0, a count-to-zero condition is triggered.
 * @{
 */
#define MXC_F_DMA_CNT_CNT_POS                          0 /**< CNT_CNT Position */
#define MXC_F_DMA_CNT_CNT                              ((uint32_t)(0xFFFFFFUL << MXC_F_DMA_CNT_CNT_POS)) /**< CNT_CNT Mask */

/**@} end of group DMA_CNT_Register */

/**
 * @ingroup  dma_registers
 * @defgroup DMA_SRC_RLD DMA_SRC_RLD
 * @brief    Source Address Reload Value. The value of this register is loaded into DMA0_SRC
 *           upon a count-to-zero condition.
 * @{
 */
#define MXC_F_DMA_SRC_RLD_SRC_RLD_POS                  0 /**< SRC_RLD_SRC_RLD Position */
#define MXC_F_DMA_SRC_RLD_SRC_RLD                      ((uint32_t)(0x7FFFFFFFUL << MXC_F_DMA_SRC_RLD_SRC_RLD_POS)) /**< SRC_RLD_SRC_RLD Mask */

/**@} end of group DMA_SRC_RLD_Register */

/**
 * @ingroup  dma_registers
 * @defgroup DMA_DST_RLD DMA_DST_RLD
 * @brief    Destination Address Reload Value. The value of this register is loaded into
 *           DMA0_DST upon a count-to-zero condition.
 * @{
 */
#define MXC_F_DMA_DST_RLD_DST_RLD_POS                  0 /**< DST_RLD_DST_RLD Position */
#define MXC_F_DMA_DST_RLD_DST_RLD                      ((uint32_t)(0x7FFFFFFFUL << MXC_F_DMA_DST_RLD_DST_RLD_POS)) /**< DST_RLD_DST_RLD Mask */

/**@} end of group DMA_DST_RLD_Register */

/**
 * @ingroup  dma_registers
 * @defgroup DMA_CNT_RLD DMA_CNT_RLD
 * @brief    DMA Channel Count Reload Register.
 * @{
 */
#define MXC_F_DMA_CNT_RLD_CNT_RLD_POS                  0 /**< CNT_RLD_CNT_RLD Position */
#define MXC_F_DMA_CNT_RLD_CNT_RLD                      ((uint32_t)(0xFFFFFFUL << MXC_F_DMA_CNT_RLD_CNT_RLD_POS)) /**< CNT_RLD_CNT_RLD Mask */

#define MXC_F_DMA_CNT_RLD_RLDEN_POS                    31 /**< CNT_RLD_RLDEN Position */
#define MXC_F_DMA_CNT_RLD_RLDEN                        ((uint32_t)(0x1UL << MXC_F_DMA_CNT_RLD_RLDEN_POS)) /**< CNT_RLD_RLDEN Mask */

/**@} end of group DMA_CNT_RLD_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_DMA_REGS_H_
