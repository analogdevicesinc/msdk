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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32662_INCLUDE_DMA_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32662_INCLUDE_DMA_REGS_H_

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
#ifdef __cplusplus
#define __I volatile
#else
#define __I volatile const
#endif
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
    __IO uint32_t ctrl;                 /**< <tt>\b 0x000:</tt> DMA CTRL Register */
    __IO uint32_t status;               /**< <tt>\b 0x004:</tt> DMA STATUS Register */
    __IO uint32_t src;                  /**< <tt>\b 0x008:</tt> DMA SRC Register */
    __IO uint32_t dst;                  /**< <tt>\b 0x00C:</tt> DMA DST Register */
    __IO uint32_t cnt;                  /**< <tt>\b 0x010:</tt> DMA CNT Register */
    __IO uint32_t srcrld;               /**< <tt>\b 0x014:</tt> DMA SRCRLD Register */
    __IO uint32_t dstrld;               /**< <tt>\b 0x018:</tt> DMA DSTRLD Register */
    __IO uint32_t cntrld;               /**< <tt>\b 0x01C:</tt> DMA CNTRLD Register */
} mxc_dma_ch_regs_t;

typedef struct {
    __IO uint32_t inten;                /**< <tt>\b 0x000:</tt> DMA INTEN Register */
    __I  uint32_t intfl;                /**< <tt>\b 0x004:</tt> DMA INTFL Register */
    __R  uint32_t rsv_0x8_0xff[62];
    __IO mxc_dma_ch_regs_t ch[4];       /**< <tt>\b 0x100:</tt> DMA CH Register */
} mxc_dma_regs_t;

/* Register offsets for module DMA */
/**
 * @ingroup    dma_registers
 * @defgroup   DMA_Register_Offsets Register Offsets
 * @brief      DMA Peripheral Register Offsets from the DMA Base Peripheral Address.
 * @{
 */
#define MXC_R_DMA_CTRL                     ((uint32_t)0x00000000UL) /**< Offset from DMA Base Address: <tt> 0x0000</tt> */
#define MXC_R_DMA_STATUS                   ((uint32_t)0x00000004UL) /**< Offset from DMA Base Address: <tt> 0x0004</tt> */
#define MXC_R_DMA_SRC                      ((uint32_t)0x00000008UL) /**< Offset from DMA Base Address: <tt> 0x0008</tt> */
#define MXC_R_DMA_DST                      ((uint32_t)0x0000000CUL) /**< Offset from DMA Base Address: <tt> 0x000C</tt> */
#define MXC_R_DMA_CNT                      ((uint32_t)0x00000010UL) /**< Offset from DMA Base Address: <tt> 0x0010</tt> */
#define MXC_R_DMA_SRCRLD                   ((uint32_t)0x00000014UL) /**< Offset from DMA Base Address: <tt> 0x0014</tt> */
#define MXC_R_DMA_DSTRLD                   ((uint32_t)0x00000018UL) /**< Offset from DMA Base Address: <tt> 0x0018</tt> */
#define MXC_R_DMA_CNTRLD                   ((uint32_t)0x0000001CUL) /**< Offset from DMA Base Address: <tt> 0x001C</tt> */
#define MXC_R_DMA_INTEN                    ((uint32_t)0x00000000UL) /**< Offset from DMA Base Address: <tt> 0x0000</tt> */
#define MXC_R_DMA_INTFL                    ((uint32_t)0x00000004UL) /**< Offset from DMA Base Address: <tt> 0x0004</tt> */
#define MXC_R_DMA_CH                       ((uint32_t)0x00000100UL) /**< Offset from DMA Base Address: <tt> 0x0100</tt> */
/**@} end of group dma_registers */

/**
 * @ingroup  dma_registers
 * @defgroup DMA_INTEN DMA_INTEN
 * @brief    DMA Control Register.
 * @{
 */
#define MXC_F_DMA_INTEN_CH0_POS                        0 /**< INTEN_CH0 Position */
#define MXC_F_DMA_INTEN_CH0                            ((uint32_t)(0x1UL << MXC_F_DMA_INTEN_CH0_POS)) /**< INTEN_CH0 Mask */

#define MXC_F_DMA_INTEN_CH1_POS                        1 /**< INTEN_CH1 Position */
#define MXC_F_DMA_INTEN_CH1                            ((uint32_t)(0x1UL << MXC_F_DMA_INTEN_CH1_POS)) /**< INTEN_CH1 Mask */

#define MXC_F_DMA_INTEN_CH2_POS                        2 /**< INTEN_CH2 Position */
#define MXC_F_DMA_INTEN_CH2                            ((uint32_t)(0x1UL << MXC_F_DMA_INTEN_CH2_POS)) /**< INTEN_CH2 Mask */

#define MXC_F_DMA_INTEN_CH3_POS                        3 /**< INTEN_CH3 Position */
#define MXC_F_DMA_INTEN_CH3                            ((uint32_t)(0x1UL << MXC_F_DMA_INTEN_CH3_POS)) /**< INTEN_CH3 Mask */

/**@} end of group DMA_INTEN_Register */

/**
 * @ingroup  dma_registers
 * @defgroup DMA_INTFL DMA_INTFL
 * @brief    DMA Interrupt Register.
 * @{
 */
#define MXC_F_DMA_INTFL_CH0_POS                        0 /**< INTFL_CH0 Position */
#define MXC_F_DMA_INTFL_CH0                            ((uint32_t)(0x1UL << MXC_F_DMA_INTFL_CH0_POS)) /**< INTFL_CH0 Mask */

#define MXC_F_DMA_INTFL_CH1_POS                        1 /**< INTFL_CH1 Position */
#define MXC_F_DMA_INTFL_CH1                            ((uint32_t)(0x1UL << MXC_F_DMA_INTFL_CH1_POS)) /**< INTFL_CH1 Mask */

#define MXC_F_DMA_INTFL_CH2_POS                        2 /**< INTFL_CH2 Position */
#define MXC_F_DMA_INTFL_CH2                            ((uint32_t)(0x1UL << MXC_F_DMA_INTFL_CH2_POS)) /**< INTFL_CH2 Mask */

#define MXC_F_DMA_INTFL_CH3_POS                        3 /**< INTFL_CH3 Position */
#define MXC_F_DMA_INTFL_CH3                            ((uint32_t)(0x1UL << MXC_F_DMA_INTFL_CH3_POS)) /**< INTFL_CH3 Mask */

/**@} end of group DMA_INTFL_Register */

/**
 * @ingroup  dma_registers
 * @defgroup DMA_CTRL DMA_CTRL
 * @brief    DMA Channel Control Register.
 * @{
 */
#define MXC_F_DMA_CTRL_EN_POS                          0 /**< CTRL_EN Position */
#define MXC_F_DMA_CTRL_EN                              ((uint32_t)(0x1UL << MXC_F_DMA_CTRL_EN_POS)) /**< CTRL_EN Mask */

#define MXC_F_DMA_CTRL_RLDEN_POS                       1 /**< CTRL_RLDEN Position */
#define MXC_F_DMA_CTRL_RLDEN                           ((uint32_t)(0x1UL << MXC_F_DMA_CTRL_RLDEN_POS)) /**< CTRL_RLDEN Mask */

#define MXC_F_DMA_CTRL_PRI_POS                         2 /**< CTRL_PRI Position */
#define MXC_F_DMA_CTRL_PRI                             ((uint32_t)(0x3UL << MXC_F_DMA_CTRL_PRI_POS)) /**< CTRL_PRI Mask */
#define MXC_V_DMA_CTRL_PRI_HIGH                        ((uint32_t)0x0UL) /**< CTRL_PRI_HIGH Value */
#define MXC_S_DMA_CTRL_PRI_HIGH                        (MXC_V_DMA_CTRL_PRI_HIGH << MXC_F_DMA_CTRL_PRI_POS) /**< CTRL_PRI_HIGH Setting */
#define MXC_V_DMA_CTRL_PRI_MEDHIGH                     ((uint32_t)0x1UL) /**< CTRL_PRI_MEDHIGH Value */
#define MXC_S_DMA_CTRL_PRI_MEDHIGH                     (MXC_V_DMA_CTRL_PRI_MEDHIGH << MXC_F_DMA_CTRL_PRI_POS) /**< CTRL_PRI_MEDHIGH Setting */
#define MXC_V_DMA_CTRL_PRI_MEDLOW                      ((uint32_t)0x2UL) /**< CTRL_PRI_MEDLOW Value */
#define MXC_S_DMA_CTRL_PRI_MEDLOW                      (MXC_V_DMA_CTRL_PRI_MEDLOW << MXC_F_DMA_CTRL_PRI_POS) /**< CTRL_PRI_MEDLOW Setting */
#define MXC_V_DMA_CTRL_PRI_LOW                         ((uint32_t)0x3UL) /**< CTRL_PRI_LOW Value */
#define MXC_S_DMA_CTRL_PRI_LOW                         (MXC_V_DMA_CTRL_PRI_LOW << MXC_F_DMA_CTRL_PRI_POS) /**< CTRL_PRI_LOW Setting */

#define MXC_F_DMA_CTRL_REQUEST_POS                     4 /**< CTRL_REQUEST Position */
#define MXC_F_DMA_CTRL_REQUEST                         ((uint32_t)(0x3FUL << MXC_F_DMA_CTRL_REQUEST_POS)) /**< CTRL_REQUEST Mask */
#define MXC_V_DMA_CTRL_REQUEST_MEMTOMEM                ((uint32_t)0x0UL) /**< CTRL_REQUEST_MEMTOMEM Value */
#define MXC_S_DMA_CTRL_REQUEST_MEMTOMEM                (MXC_V_DMA_CTRL_REQUEST_MEMTOMEM << MXC_F_DMA_CTRL_REQUEST_POS) /**< CTRL_REQUEST_MEMTOMEM Setting */
#define MXC_V_DMA_CTRL_REQUEST_SPI0RX                  ((uint32_t)0x1UL) /**< CTRL_REQUEST_SPI0RX Value */
#define MXC_S_DMA_CTRL_REQUEST_SPI0RX                  (MXC_V_DMA_CTRL_REQUEST_SPI0RX << MXC_F_DMA_CTRL_REQUEST_POS) /**< CTRL_REQUEST_SPI0RX Setting */
#define MXC_V_DMA_CTRL_REQUEST_SPI1RX                  ((uint32_t)0x2UL) /**< CTRL_REQUEST_SPI1RX Value */
#define MXC_S_DMA_CTRL_REQUEST_SPI1RX                  (MXC_V_DMA_CTRL_REQUEST_SPI1RX << MXC_F_DMA_CTRL_REQUEST_POS) /**< CTRL_REQUEST_SPI1RX Setting */
#define MXC_V_DMA_CTRL_REQUEST_UART0RX                 ((uint32_t)0x4UL) /**< CTRL_REQUEST_UART0RX Value */
#define MXC_S_DMA_CTRL_REQUEST_UART0RX                 (MXC_V_DMA_CTRL_REQUEST_UART0RX << MXC_F_DMA_CTRL_REQUEST_POS) /**< CTRL_REQUEST_UART0RX Setting */
#define MXC_V_DMA_CTRL_REQUEST_UART1RX                 ((uint32_t)0x5UL) /**< CTRL_REQUEST_UART1RX Value */
#define MXC_S_DMA_CTRL_REQUEST_UART1RX                 (MXC_V_DMA_CTRL_REQUEST_UART1RX << MXC_F_DMA_CTRL_REQUEST_POS) /**< CTRL_REQUEST_UART1RX Setting */
#define MXC_V_DMA_CTRL_REQUEST_CANRX                   ((uint32_t)0x6UL) /**< CTRL_REQUEST_CANRX Value */
#define MXC_S_DMA_CTRL_REQUEST_CANRX                   (MXC_V_DMA_CTRL_REQUEST_CANRX << MXC_F_DMA_CTRL_REQUEST_POS) /**< CTRL_REQUEST_CANRX Setting */
#define MXC_V_DMA_CTRL_REQUEST_I2C0RX                  ((uint32_t)0x7UL) /**< CTRL_REQUEST_I2C0RX Value */
#define MXC_S_DMA_CTRL_REQUEST_I2C0RX                  (MXC_V_DMA_CTRL_REQUEST_I2C0RX << MXC_F_DMA_CTRL_REQUEST_POS) /**< CTRL_REQUEST_I2C0RX Setting */
#define MXC_V_DMA_CTRL_REQUEST_I2C1RX                  ((uint32_t)0x8UL) /**< CTRL_REQUEST_I2C1RX Value */
#define MXC_S_DMA_CTRL_REQUEST_I2C1RX                  (MXC_V_DMA_CTRL_REQUEST_I2C1RX << MXC_F_DMA_CTRL_REQUEST_POS) /**< CTRL_REQUEST_I2C1RX Setting */
#define MXC_V_DMA_CTRL_REQUEST_ADC                     ((uint32_t)0x9UL) /**< CTRL_REQUEST_ADC Value */
#define MXC_S_DMA_CTRL_REQUEST_ADC                     (MXC_V_DMA_CTRL_REQUEST_ADC << MXC_F_DMA_CTRL_REQUEST_POS) /**< CTRL_REQUEST_ADC Setting */
#define MXC_V_DMA_CTRL_REQUEST_AESRX                   ((uint32_t)0x10UL) /**< CTRL_REQUEST_AESRX Value */
#define MXC_S_DMA_CTRL_REQUEST_AESRX                   (MXC_V_DMA_CTRL_REQUEST_AESRX << MXC_F_DMA_CTRL_REQUEST_POS) /**< CTRL_REQUEST_AESRX Setting */
#define MXC_V_DMA_CTRL_REQUEST_I2SRX                   ((uint32_t)0x1EUL) /**< CTRL_REQUEST_I2SRX Value */
#define MXC_S_DMA_CTRL_REQUEST_I2SRX                   (MXC_V_DMA_CTRL_REQUEST_I2SRX << MXC_F_DMA_CTRL_REQUEST_POS) /**< CTRL_REQUEST_I2SRX Setting */
#define MXC_V_DMA_CTRL_REQUEST_SPI0TX                  ((uint32_t)0x21UL) /**< CTRL_REQUEST_SPI0TX Value */
#define MXC_S_DMA_CTRL_REQUEST_SPI0TX                  (MXC_V_DMA_CTRL_REQUEST_SPI0TX << MXC_F_DMA_CTRL_REQUEST_POS) /**< CTRL_REQUEST_SPI0TX Setting */
#define MXC_V_DMA_CTRL_REQUEST_SPI1TX                  ((uint32_t)0x22UL) /**< CTRL_REQUEST_SPI1TX Value */
#define MXC_S_DMA_CTRL_REQUEST_SPI1TX                  (MXC_V_DMA_CTRL_REQUEST_SPI1TX << MXC_F_DMA_CTRL_REQUEST_POS) /**< CTRL_REQUEST_SPI1TX Setting */
#define MXC_V_DMA_CTRL_REQUEST_UART0TX                 ((uint32_t)0x24UL) /**< CTRL_REQUEST_UART0TX Value */
#define MXC_S_DMA_CTRL_REQUEST_UART0TX                 (MXC_V_DMA_CTRL_REQUEST_UART0TX << MXC_F_DMA_CTRL_REQUEST_POS) /**< CTRL_REQUEST_UART0TX Setting */
#define MXC_V_DMA_CTRL_REQUEST_UART1TX                 ((uint32_t)0x25UL) /**< CTRL_REQUEST_UART1TX Value */
#define MXC_S_DMA_CTRL_REQUEST_UART1TX                 (MXC_V_DMA_CTRL_REQUEST_UART1TX << MXC_F_DMA_CTRL_REQUEST_POS) /**< CTRL_REQUEST_UART1TX Setting */
#define MXC_V_DMA_CTRL_REQUEST_CANTX                   ((uint32_t)0x26UL) /**< CTRL_REQUEST_CANTX Value */
#define MXC_S_DMA_CTRL_REQUEST_CANTX                   (MXC_V_DMA_CTRL_REQUEST_CANTX << MXC_F_DMA_CTRL_REQUEST_POS) /**< CTRL_REQUEST_CANTX Setting */
#define MXC_V_DMA_CTRL_REQUEST_I2C0TX                  ((uint32_t)0x27UL) /**< CTRL_REQUEST_I2C0TX Value */
#define MXC_S_DMA_CTRL_REQUEST_I2C0TX                  (MXC_V_DMA_CTRL_REQUEST_I2C0TX << MXC_F_DMA_CTRL_REQUEST_POS) /**< CTRL_REQUEST_I2C0TX Setting */
#define MXC_V_DMA_CTRL_REQUEST_I2C1TX                  ((uint32_t)0x28UL) /**< CTRL_REQUEST_I2C1TX Value */
#define MXC_S_DMA_CTRL_REQUEST_I2C1TX                  (MXC_V_DMA_CTRL_REQUEST_I2C1TX << MXC_F_DMA_CTRL_REQUEST_POS) /**< CTRL_REQUEST_I2C1TX Setting */
#define MXC_V_DMA_CTRL_REQUEST_AESTX                   ((uint32_t)0x30UL) /**< CTRL_REQUEST_AESTX Value */
#define MXC_S_DMA_CTRL_REQUEST_AESTX                   (MXC_V_DMA_CTRL_REQUEST_AESTX << MXC_F_DMA_CTRL_REQUEST_POS) /**< CTRL_REQUEST_AESTX Setting */
#define MXC_V_DMA_CTRL_REQUEST_I2STX                   ((uint32_t)0x3EUL) /**< CTRL_REQUEST_I2STX Value */
#define MXC_S_DMA_CTRL_REQUEST_I2STX                   (MXC_V_DMA_CTRL_REQUEST_I2STX << MXC_F_DMA_CTRL_REQUEST_POS) /**< CTRL_REQUEST_I2STX Setting */

#define MXC_F_DMA_CTRL_TO_WAIT_POS                     10 /**< CTRL_TO_WAIT Position */
#define MXC_F_DMA_CTRL_TO_WAIT                         ((uint32_t)(0x1UL << MXC_F_DMA_CTRL_TO_WAIT_POS)) /**< CTRL_TO_WAIT Mask */

#define MXC_F_DMA_CTRL_TO_PER_POS                      11 /**< CTRL_TO_PER Position */
#define MXC_F_DMA_CTRL_TO_PER                          ((uint32_t)(0x7UL << MXC_F_DMA_CTRL_TO_PER_POS)) /**< CTRL_TO_PER Mask */
#define MXC_V_DMA_CTRL_TO_PER_TO4                      ((uint32_t)0x0UL) /**< CTRL_TO_PER_TO4 Value */
#define MXC_S_DMA_CTRL_TO_PER_TO4                      (MXC_V_DMA_CTRL_TO_PER_TO4 << MXC_F_DMA_CTRL_TO_PER_POS) /**< CTRL_TO_PER_TO4 Setting */
#define MXC_V_DMA_CTRL_TO_PER_TO8                      ((uint32_t)0x1UL) /**< CTRL_TO_PER_TO8 Value */
#define MXC_S_DMA_CTRL_TO_PER_TO8                      (MXC_V_DMA_CTRL_TO_PER_TO8 << MXC_F_DMA_CTRL_TO_PER_POS) /**< CTRL_TO_PER_TO8 Setting */
#define MXC_V_DMA_CTRL_TO_PER_TO16                     ((uint32_t)0x2UL) /**< CTRL_TO_PER_TO16 Value */
#define MXC_S_DMA_CTRL_TO_PER_TO16                     (MXC_V_DMA_CTRL_TO_PER_TO16 << MXC_F_DMA_CTRL_TO_PER_POS) /**< CTRL_TO_PER_TO16 Setting */
#define MXC_V_DMA_CTRL_TO_PER_TO32                     ((uint32_t)0x3UL) /**< CTRL_TO_PER_TO32 Value */
#define MXC_S_DMA_CTRL_TO_PER_TO32                     (MXC_V_DMA_CTRL_TO_PER_TO32 << MXC_F_DMA_CTRL_TO_PER_POS) /**< CTRL_TO_PER_TO32 Setting */
#define MXC_V_DMA_CTRL_TO_PER_TO64                     ((uint32_t)0x4UL) /**< CTRL_TO_PER_TO64 Value */
#define MXC_S_DMA_CTRL_TO_PER_TO64                     (MXC_V_DMA_CTRL_TO_PER_TO64 << MXC_F_DMA_CTRL_TO_PER_POS) /**< CTRL_TO_PER_TO64 Setting */
#define MXC_V_DMA_CTRL_TO_PER_TO128                    ((uint32_t)0x5UL) /**< CTRL_TO_PER_TO128 Value */
#define MXC_S_DMA_CTRL_TO_PER_TO128                    (MXC_V_DMA_CTRL_TO_PER_TO128 << MXC_F_DMA_CTRL_TO_PER_POS) /**< CTRL_TO_PER_TO128 Setting */
#define MXC_V_DMA_CTRL_TO_PER_TO256                    ((uint32_t)0x6UL) /**< CTRL_TO_PER_TO256 Value */
#define MXC_S_DMA_CTRL_TO_PER_TO256                    (MXC_V_DMA_CTRL_TO_PER_TO256 << MXC_F_DMA_CTRL_TO_PER_POS) /**< CTRL_TO_PER_TO256 Setting */
#define MXC_V_DMA_CTRL_TO_PER_TO512                    ((uint32_t)0x7UL) /**< CTRL_TO_PER_TO512 Value */
#define MXC_S_DMA_CTRL_TO_PER_TO512                    (MXC_V_DMA_CTRL_TO_PER_TO512 << MXC_F_DMA_CTRL_TO_PER_POS) /**< CTRL_TO_PER_TO512 Setting */

#define MXC_F_DMA_CTRL_TO_CLKDIV_POS                   14 /**< CTRL_TO_CLKDIV Position */
#define MXC_F_DMA_CTRL_TO_CLKDIV                       ((uint32_t)(0x3UL << MXC_F_DMA_CTRL_TO_CLKDIV_POS)) /**< CTRL_TO_CLKDIV Mask */
#define MXC_V_DMA_CTRL_TO_CLKDIV_DIS                   ((uint32_t)0x0UL) /**< CTRL_TO_CLKDIV_DIS Value */
#define MXC_S_DMA_CTRL_TO_CLKDIV_DIS                   (MXC_V_DMA_CTRL_TO_CLKDIV_DIS << MXC_F_DMA_CTRL_TO_CLKDIV_POS) /**< CTRL_TO_CLKDIV_DIS Setting */
#define MXC_V_DMA_CTRL_TO_CLKDIV_DIV256                ((uint32_t)0x1UL) /**< CTRL_TO_CLKDIV_DIV256 Value */
#define MXC_S_DMA_CTRL_TO_CLKDIV_DIV256                (MXC_V_DMA_CTRL_TO_CLKDIV_DIV256 << MXC_F_DMA_CTRL_TO_CLKDIV_POS) /**< CTRL_TO_CLKDIV_DIV256 Setting */
#define MXC_V_DMA_CTRL_TO_CLKDIV_DIV64K                ((uint32_t)0x2UL) /**< CTRL_TO_CLKDIV_DIV64K Value */
#define MXC_S_DMA_CTRL_TO_CLKDIV_DIV64K                (MXC_V_DMA_CTRL_TO_CLKDIV_DIV64K << MXC_F_DMA_CTRL_TO_CLKDIV_POS) /**< CTRL_TO_CLKDIV_DIV64K Setting */
#define MXC_V_DMA_CTRL_TO_CLKDIV_DIV16M                ((uint32_t)0x3UL) /**< CTRL_TO_CLKDIV_DIV16M Value */
#define MXC_S_DMA_CTRL_TO_CLKDIV_DIV16M                (MXC_V_DMA_CTRL_TO_CLKDIV_DIV16M << MXC_F_DMA_CTRL_TO_CLKDIV_POS) /**< CTRL_TO_CLKDIV_DIV16M Setting */

#define MXC_F_DMA_CTRL_SRCWD_POS                       16 /**< CTRL_SRCWD Position */
#define MXC_F_DMA_CTRL_SRCWD                           ((uint32_t)(0x3UL << MXC_F_DMA_CTRL_SRCWD_POS)) /**< CTRL_SRCWD Mask */
#define MXC_V_DMA_CTRL_SRCWD_BYTE                      ((uint32_t)0x0UL) /**< CTRL_SRCWD_BYTE Value */
#define MXC_S_DMA_CTRL_SRCWD_BYTE                      (MXC_V_DMA_CTRL_SRCWD_BYTE << MXC_F_DMA_CTRL_SRCWD_POS) /**< CTRL_SRCWD_BYTE Setting */
#define MXC_V_DMA_CTRL_SRCWD_HALFWORD                  ((uint32_t)0x1UL) /**< CTRL_SRCWD_HALFWORD Value */
#define MXC_S_DMA_CTRL_SRCWD_HALFWORD                  (MXC_V_DMA_CTRL_SRCWD_HALFWORD << MXC_F_DMA_CTRL_SRCWD_POS) /**< CTRL_SRCWD_HALFWORD Setting */
#define MXC_V_DMA_CTRL_SRCWD_WORD                      ((uint32_t)0x2UL) /**< CTRL_SRCWD_WORD Value */
#define MXC_S_DMA_CTRL_SRCWD_WORD                      (MXC_V_DMA_CTRL_SRCWD_WORD << MXC_F_DMA_CTRL_SRCWD_POS) /**< CTRL_SRCWD_WORD Setting */

#define MXC_F_DMA_CTRL_SRCINC_POS                      18 /**< CTRL_SRCINC Position */
#define MXC_F_DMA_CTRL_SRCINC                          ((uint32_t)(0x1UL << MXC_F_DMA_CTRL_SRCINC_POS)) /**< CTRL_SRCINC Mask */

#define MXC_F_DMA_CTRL_DSTWD_POS                       20 /**< CTRL_DSTWD Position */
#define MXC_F_DMA_CTRL_DSTWD                           ((uint32_t)(0x3UL << MXC_F_DMA_CTRL_DSTWD_POS)) /**< CTRL_DSTWD Mask */
#define MXC_V_DMA_CTRL_DSTWD_BYTE                      ((uint32_t)0x0UL) /**< CTRL_DSTWD_BYTE Value */
#define MXC_S_DMA_CTRL_DSTWD_BYTE                      (MXC_V_DMA_CTRL_DSTWD_BYTE << MXC_F_DMA_CTRL_DSTWD_POS) /**< CTRL_DSTWD_BYTE Setting */
#define MXC_V_DMA_CTRL_DSTWD_HALFWORD                  ((uint32_t)0x1UL) /**< CTRL_DSTWD_HALFWORD Value */
#define MXC_S_DMA_CTRL_DSTWD_HALFWORD                  (MXC_V_DMA_CTRL_DSTWD_HALFWORD << MXC_F_DMA_CTRL_DSTWD_POS) /**< CTRL_DSTWD_HALFWORD Setting */
#define MXC_V_DMA_CTRL_DSTWD_WORD                      ((uint32_t)0x2UL) /**< CTRL_DSTWD_WORD Value */
#define MXC_S_DMA_CTRL_DSTWD_WORD                      (MXC_V_DMA_CTRL_DSTWD_WORD << MXC_F_DMA_CTRL_DSTWD_POS) /**< CTRL_DSTWD_WORD Setting */

#define MXC_F_DMA_CTRL_DSTINC_POS                      22 /**< CTRL_DSTINC Position */
#define MXC_F_DMA_CTRL_DSTINC                          ((uint32_t)(0x1UL << MXC_F_DMA_CTRL_DSTINC_POS)) /**< CTRL_DSTINC Mask */

#define MXC_F_DMA_CTRL_BURST_SIZE_POS                  24 /**< CTRL_BURST_SIZE Position */
#define MXC_F_DMA_CTRL_BURST_SIZE                      ((uint32_t)(0x1FUL << MXC_F_DMA_CTRL_BURST_SIZE_POS)) /**< CTRL_BURST_SIZE Mask */

#define MXC_F_DMA_CTRL_DIS_IE_POS                      30 /**< CTRL_DIS_IE Position */
#define MXC_F_DMA_CTRL_DIS_IE                          ((uint32_t)(0x1UL << MXC_F_DMA_CTRL_DIS_IE_POS)) /**< CTRL_DIS_IE Mask */

#define MXC_F_DMA_CTRL_CTZ_IE_POS                      31 /**< CTRL_CTZ_IE Position */
#define MXC_F_DMA_CTRL_CTZ_IE                          ((uint32_t)(0x1UL << MXC_F_DMA_CTRL_CTZ_IE_POS)) /**< CTRL_CTZ_IE Mask */

/**@} end of group DMA_CTRL_Register */

/**
 * @ingroup  dma_registers
 * @defgroup DMA_STATUS DMA_STATUS
 * @brief    DMA Channel Status Register.
 * @{
 */
#define MXC_F_DMA_STATUS_STATUS_POS                    0 /**< STATUS_STATUS Position */
#define MXC_F_DMA_STATUS_STATUS                        ((uint32_t)(0x1UL << MXC_F_DMA_STATUS_STATUS_POS)) /**< STATUS_STATUS Mask */

#define MXC_F_DMA_STATUS_IPEND_POS                     1 /**< STATUS_IPEND Position */
#define MXC_F_DMA_STATUS_IPEND                         ((uint32_t)(0x1UL << MXC_F_DMA_STATUS_IPEND_POS)) /**< STATUS_IPEND Mask */

#define MXC_F_DMA_STATUS_CTZ_IF_POS                    2 /**< STATUS_CTZ_IF Position */
#define MXC_F_DMA_STATUS_CTZ_IF                        ((uint32_t)(0x1UL << MXC_F_DMA_STATUS_CTZ_IF_POS)) /**< STATUS_CTZ_IF Mask */

#define MXC_F_DMA_STATUS_RLD_IF_POS                    3 /**< STATUS_RLD_IF Position */
#define MXC_F_DMA_STATUS_RLD_IF                        ((uint32_t)(0x1UL << MXC_F_DMA_STATUS_RLD_IF_POS)) /**< STATUS_RLD_IF Mask */

#define MXC_F_DMA_STATUS_BUS_ERR_POS                   4 /**< STATUS_BUS_ERR Position */
#define MXC_F_DMA_STATUS_BUS_ERR                       ((uint32_t)(0x1UL << MXC_F_DMA_STATUS_BUS_ERR_POS)) /**< STATUS_BUS_ERR Mask */

#define MXC_F_DMA_STATUS_TO_IF_POS                     6 /**< STATUS_TO_IF Position */
#define MXC_F_DMA_STATUS_TO_IF                         ((uint32_t)(0x1UL << MXC_F_DMA_STATUS_TO_IF_POS)) /**< STATUS_TO_IF Mask */

/**@} end of group DMA_STATUS_Register */

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
 * @defgroup DMA_SRCRLD DMA_SRCRLD
 * @brief    Source Address Reload Value. The value of this register is loaded into DMA0_SRC
 *           upon a count-to-zero condition.
 * @{
 */
#define MXC_F_DMA_SRCRLD_ADDR_POS                      0 /**< SRCRLD_ADDR Position */
#define MXC_F_DMA_SRCRLD_ADDR                          ((uint32_t)(0x7FFFFFFFUL << MXC_F_DMA_SRCRLD_ADDR_POS)) /**< SRCRLD_ADDR Mask */

/**@} end of group DMA_SRCRLD_Register */

/**
 * @ingroup  dma_registers
 * @defgroup DMA_DSTRLD DMA_DSTRLD
 * @brief    Destination Address Reload Value. The value of this register is loaded into
 *           DMA0_DST upon a count-to-zero condition.
 * @{
 */
#define MXC_F_DMA_DSTRLD_ADDR_POS                      0 /**< DSTRLD_ADDR Position */
#define MXC_F_DMA_DSTRLD_ADDR                          ((uint32_t)(0x7FFFFFFFUL << MXC_F_DMA_DSTRLD_ADDR_POS)) /**< DSTRLD_ADDR Mask */

/**@} end of group DMA_DSTRLD_Register */

/**
 * @ingroup  dma_registers
 * @defgroup DMA_CNTRLD DMA_CNTRLD
 * @brief    DMA Channel Count Reload Register.
 * @{
 */
#define MXC_F_DMA_CNTRLD_CNT_POS                       0 /**< CNTRLD_CNT Position */
#define MXC_F_DMA_CNTRLD_CNT                           ((uint32_t)(0xFFFFFFUL << MXC_F_DMA_CNTRLD_CNT_POS)) /**< CNTRLD_CNT Mask */

#define MXC_F_DMA_CNTRLD_EN_POS                        31 /**< CNTRLD_EN Position */
#define MXC_F_DMA_CNTRLD_EN                            ((uint32_t)(0x1UL << MXC_F_DMA_CNTRLD_EN_POS)) /**< CNTRLD_EN Mask */

/**@} end of group DMA_CNTRLD_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32662_INCLUDE_DMA_REGS_H_
