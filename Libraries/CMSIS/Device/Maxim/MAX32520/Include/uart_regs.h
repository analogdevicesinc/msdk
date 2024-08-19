/**
 * @file    uart_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the UART Peripheral Module.
 * @note    This file is @generated.
 * @ingroup uart_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32520_INCLUDE_UART_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32520_INCLUDE_UART_REGS_H_

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
 * @ingroup     uart
 * @defgroup    uart_registers UART_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the UART Peripheral Module.
 * @details     UART
 */

/**
 * @ingroup uart_registers
 * Structure type to access the UART Registers.
 */
typedef struct {
    __IO uint32_t ctrl;                 /**< <tt>\b 0x00:</tt> UART CTRL Register */
    __I  uint32_t stat;                 /**< <tt>\b 0x04:</tt> UART STAT Register */
    __IO uint32_t int_en;               /**< <tt>\b 0x08:</tt> UART INT_EN Register */
    __IO uint32_t int_stat;             /**< <tt>\b 0x0C:</tt> UART INT_STAT Register */
    __IO uint32_t baud0;                /**< <tt>\b 0x10:</tt> UART BAUD0 Register */
    __IO uint32_t baud1;                /**< <tt>\b 0x14:</tt> UART BAUD1 Register */
    __R  uint32_t rsv_0x18_0x1f[2];
    __IO uint32_t data;                 /**< <tt>\b 0x20:</tt> UART DATA Register */
    __R  uint32_t rsv_0x24_0x2f[3];
    __IO uint32_t dma;                  /**< <tt>\b 0x30:</tt> UART DMA Register */
} mxc_uart_regs_t;

/* Register offsets for module UART */
/**
 * @ingroup    uart_registers
 * @defgroup   UART_Register_Offsets Register Offsets
 * @brief      UART Peripheral Register Offsets from the UART Base Peripheral Address.
 * @{
 */
#define MXC_R_UART_CTRL                    ((uint32_t)0x00000000UL) /**< Offset from UART Base Address: <tt> 0x0000</tt> */
#define MXC_R_UART_STAT                    ((uint32_t)0x00000004UL) /**< Offset from UART Base Address: <tt> 0x0004</tt> */
#define MXC_R_UART_INT_EN                  ((uint32_t)0x00000008UL) /**< Offset from UART Base Address: <tt> 0x0008</tt> */
#define MXC_R_UART_INT_STAT                ((uint32_t)0x0000000CUL) /**< Offset from UART Base Address: <tt> 0x000C</tt> */
#define MXC_R_UART_BAUD0                   ((uint32_t)0x00000010UL) /**< Offset from UART Base Address: <tt> 0x0010</tt> */
#define MXC_R_UART_BAUD1                   ((uint32_t)0x00000014UL) /**< Offset from UART Base Address: <tt> 0x0014</tt> */
#define MXC_R_UART_DATA                    ((uint32_t)0x00000020UL) /**< Offset from UART Base Address: <tt> 0x0020</tt> */
#define MXC_R_UART_DMA                     ((uint32_t)0x00000030UL) /**< Offset from UART Base Address: <tt> 0x0030</tt> */
/**@} end of group uart_registers */

/**
 * @ingroup  uart_registers
 * @defgroup UART_CTRL UART_CTRL
 * @brief    Control Register.
 * @{
 */
#define MXC_F_UART_CTRL_RXTHD_POS                      0 /**< CTRL_RXTHD Position */
#define MXC_F_UART_CTRL_RXTHD                          ((uint32_t)(0xFUL << MXC_F_UART_CTRL_RXTHD_POS)) /**< CTRL_RXTHD Mask */

#define MXC_F_UART_CTRL_PAREN_POS                      4 /**< CTRL_PAREN Position */
#define MXC_F_UART_CTRL_PAREN                          ((uint32_t)(0x1UL << MXC_F_UART_CTRL_PAREN_POS)) /**< CTRL_PAREN Mask */

#define MXC_F_UART_CTRL_PAREO_POS                      5 /**< CTRL_PAREO Position */
#define MXC_F_UART_CTRL_PAREO                          ((uint32_t)(0x1UL << MXC_F_UART_CTRL_PAREO_POS)) /**< CTRL_PAREO Mask */

#define MXC_F_UART_CTRL_PARMD_POS                      6 /**< CTRL_PARMD Position */
#define MXC_F_UART_CTRL_PARMD                          ((uint32_t)(0x1UL << MXC_F_UART_CTRL_PARMD_POS)) /**< CTRL_PARMD Mask */

#define MXC_F_UART_CTRL_TXFLUSH_POS                    8 /**< CTRL_TXFLUSH Position */
#define MXC_F_UART_CTRL_TXFLUSH                        ((uint32_t)(0x1UL << MXC_F_UART_CTRL_TXFLUSH_POS)) /**< CTRL_TXFLUSH Mask */

#define MXC_F_UART_CTRL_RXFLUSH_POS                    9 /**< CTRL_RXFLUSH Position */
#define MXC_F_UART_CTRL_RXFLUSH                        ((uint32_t)(0x1UL << MXC_F_UART_CTRL_RXFLUSH_POS)) /**< CTRL_RXFLUSH Mask */

#define MXC_F_UART_CTRL_SIZE_POS                       10 /**< CTRL_SIZE Position */
#define MXC_F_UART_CTRL_SIZE                           ((uint32_t)(0x3UL << MXC_F_UART_CTRL_SIZE_POS)) /**< CTRL_SIZE Mask */
#define MXC_V_UART_CTRL_SIZE_5                         ((uint32_t)0x0UL) /**< CTRL_SIZE_5 Value */
#define MXC_S_UART_CTRL_SIZE_5                         (MXC_V_UART_CTRL_SIZE_5 << MXC_F_UART_CTRL_SIZE_POS) /**< CTRL_SIZE_5 Setting */
#define MXC_V_UART_CTRL_SIZE_6                         ((uint32_t)0x1UL) /**< CTRL_SIZE_6 Value */
#define MXC_S_UART_CTRL_SIZE_6                         (MXC_V_UART_CTRL_SIZE_6 << MXC_F_UART_CTRL_SIZE_POS) /**< CTRL_SIZE_6 Setting */
#define MXC_V_UART_CTRL_SIZE_7                         ((uint32_t)0x2UL) /**< CTRL_SIZE_7 Value */
#define MXC_S_UART_CTRL_SIZE_7                         (MXC_V_UART_CTRL_SIZE_7 << MXC_F_UART_CTRL_SIZE_POS) /**< CTRL_SIZE_7 Setting */
#define MXC_V_UART_CTRL_SIZE_8                         ((uint32_t)0x3UL) /**< CTRL_SIZE_8 Value */
#define MXC_S_UART_CTRL_SIZE_8                         (MXC_V_UART_CTRL_SIZE_8 << MXC_F_UART_CTRL_SIZE_POS) /**< CTRL_SIZE_8 Setting */

#define MXC_F_UART_CTRL_STOP_POS                       12 /**< CTRL_STOP Position */
#define MXC_F_UART_CTRL_STOP                           ((uint32_t)(0x1UL << MXC_F_UART_CTRL_STOP_POS)) /**< CTRL_STOP Mask */

/**@} end of group UART_CTRL_Register */

/**
 * @ingroup  uart_registers
 * @defgroup UART_STAT UART_STAT
 * @brief    Status Register.
 * @{
 */
#define MXC_F_UART_STAT_TXBUSY_POS                     0 /**< STAT_TXBUSY Position */
#define MXC_F_UART_STAT_TXBUSY                         ((uint32_t)(0x1UL << MXC_F_UART_STAT_TXBUSY_POS)) /**< STAT_TXBUSY Mask */

#define MXC_F_UART_STAT_RXBUSY_POS                     1 /**< STAT_RXBUSY Position */
#define MXC_F_UART_STAT_RXBUSY                         ((uint32_t)(0x1UL << MXC_F_UART_STAT_RXBUSY_POS)) /**< STAT_RXBUSY Mask */

#define MXC_F_UART_STAT_RXEMPTY_POS                    4 /**< STAT_RXEMPTY Position */
#define MXC_F_UART_STAT_RXEMPTY                        ((uint32_t)(0x1UL << MXC_F_UART_STAT_RXEMPTY_POS)) /**< STAT_RXEMPTY Mask */

#define MXC_F_UART_STAT_RXFULL_POS                     5 /**< STAT_RXFULL Position */
#define MXC_F_UART_STAT_RXFULL                         ((uint32_t)(0x1UL << MXC_F_UART_STAT_RXFULL_POS)) /**< STAT_RXFULL Mask */

#define MXC_F_UART_STAT_TXEMPTY_POS                    6 /**< STAT_TXEMPTY Position */
#define MXC_F_UART_STAT_TXEMPTY                        ((uint32_t)(0x1UL << MXC_F_UART_STAT_TXEMPTY_POS)) /**< STAT_TXEMPTY Mask */

#define MXC_F_UART_STAT_TXFULL_POS                     7 /**< STAT_TXFULL Position */
#define MXC_F_UART_STAT_TXFULL                         ((uint32_t)(0x1UL << MXC_F_UART_STAT_TXFULL_POS)) /**< STAT_TXFULL Mask */

#define MXC_F_UART_STAT_RXELT_POS                      8 /**< STAT_RXELT Position */
#define MXC_F_UART_STAT_RXELT                          ((uint32_t)(0xFUL << MXC_F_UART_STAT_RXELT_POS)) /**< STAT_RXELT Mask */

#define MXC_F_UART_STAT_TXELT_POS                      12 /**< STAT_TXELT Position */
#define MXC_F_UART_STAT_TXELT                          ((uint32_t)(0xFUL << MXC_F_UART_STAT_TXELT_POS)) /**< STAT_TXELT Mask */

/**@} end of group UART_STAT_Register */

/**
 * @ingroup  uart_registers
 * @defgroup UART_INT_EN UART_INT_EN
 * @brief    Interrupt Enable Register.
 * @{
 */
#define MXC_F_UART_INT_EN_FRAMIE_POS                   0 /**< INT_EN_FRAMIE Position */
#define MXC_F_UART_INT_EN_FRAMIE                       ((uint32_t)(0x1UL << MXC_F_UART_INT_EN_FRAMIE_POS)) /**< INT_EN_FRAMIE Mask */

#define MXC_F_UART_INT_EN_PARITYIE_POS                 1 /**< INT_EN_PARITYIE Position */
#define MXC_F_UART_INT_EN_PARITYIE                     ((uint32_t)(0x1UL << MXC_F_UART_INT_EN_PARITYIE_POS)) /**< INT_EN_PARITYIE Mask */

#define MXC_F_UART_INT_EN_OVERIE_POS                   3 /**< INT_EN_OVERIE Position */
#define MXC_F_UART_INT_EN_OVERIE                       ((uint32_t)(0x1UL << MXC_F_UART_INT_EN_OVERIE_POS)) /**< INT_EN_OVERIE Mask */

#define MXC_F_UART_INT_EN_FFRXIE_POS                   4 /**< INT_EN_FFRXIE Position */
#define MXC_F_UART_INT_EN_FFRXIE                       ((uint32_t)(0x1UL << MXC_F_UART_INT_EN_FFRXIE_POS)) /**< INT_EN_FFRXIE Mask */

#define MXC_F_UART_INT_EN_FFTXOIE_POS                  5 /**< INT_EN_FFTXOIE Position */
#define MXC_F_UART_INT_EN_FFTXOIE                      ((uint32_t)(0x1UL << MXC_F_UART_INT_EN_FFTXOIE_POS)) /**< INT_EN_FFTXOIE Mask */

#define MXC_F_UART_INT_EN_FFTXHIE_POS                  6 /**< INT_EN_FFTXHIE Position */
#define MXC_F_UART_INT_EN_FFTXHIE                      ((uint32_t)(0x1UL << MXC_F_UART_INT_EN_FFTXHIE_POS)) /**< INT_EN_FFTXHIE Mask */

/**@} end of group UART_INT_EN_Register */

/**
 * @ingroup  uart_registers
 * @defgroup UART_INT_STAT UART_INT_STAT
 * @brief    Interrupt Status Flags.
 * @{
 */
#define MXC_F_UART_INT_STAT_FRAMIS_POS                 0 /**< INT_STAT_FRAMIS Position */
#define MXC_F_UART_INT_STAT_FRAMIS                     ((uint32_t)(0x1UL << MXC_F_UART_INT_STAT_FRAMIS_POS)) /**< INT_STAT_FRAMIS Mask */

#define MXC_F_UART_INT_STAT_PARITYIS_POS               1 /**< INT_STAT_PARITYIS Position */
#define MXC_F_UART_INT_STAT_PARITYIS                   ((uint32_t)(0x1UL << MXC_F_UART_INT_STAT_PARITYIS_POS)) /**< INT_STAT_PARITYIS Mask */

#define MXC_F_UART_INT_STAT_OVERIS_POS                 3 /**< INT_STAT_OVERIS Position */
#define MXC_F_UART_INT_STAT_OVERIS                     ((uint32_t)(0x1UL << MXC_F_UART_INT_STAT_OVERIS_POS)) /**< INT_STAT_OVERIS Mask */

#define MXC_F_UART_INT_STAT_FFRXIS_POS                 4 /**< INT_STAT_FFRXIS Position */
#define MXC_F_UART_INT_STAT_FFRXIS                     ((uint32_t)(0x1UL << MXC_F_UART_INT_STAT_FFRXIS_POS)) /**< INT_STAT_FFRXIS Mask */

#define MXC_F_UART_INT_STAT_FFTXOIS_POS                5 /**< INT_STAT_FFTXOIS Position */
#define MXC_F_UART_INT_STAT_FFTXOIS                    ((uint32_t)(0x1UL << MXC_F_UART_INT_STAT_FFTXOIS_POS)) /**< INT_STAT_FFTXOIS Mask */

#define MXC_F_UART_INT_STAT_FFTXHIS_POS                6 /**< INT_STAT_FFTXHIS Position */
#define MXC_F_UART_INT_STAT_FFTXHIS                    ((uint32_t)(0x1UL << MXC_F_UART_INT_STAT_FFTXHIS_POS)) /**< INT_STAT_FFTXHIS Mask */

/**@} end of group UART_INT_STAT_Register */

/**
 * @ingroup  uart_registers
 * @defgroup UART_BAUD0 UART_BAUD0
 * @brief    Baud rate register. Integer portion.
 * @{
 */
#define MXC_F_UART_BAUD0_IDIV_POS                      0 /**< BAUD0_IDIV Position */
#define MXC_F_UART_BAUD0_IDIV                          ((uint32_t)(0xFFFUL << MXC_F_UART_BAUD0_IDIV_POS)) /**< BAUD0_IDIV Mask */

/**@} end of group UART_BAUD0_Register */

/**
 * @ingroup  uart_registers
 * @defgroup UART_BAUD1 UART_BAUD1
 * @brief    Baud rate register. Decimal Setting.
 * @{
 */
#define MXC_F_UART_BAUD1_DDIV_POS                      0 /**< BAUD1_DDIV Position */
#define MXC_F_UART_BAUD1_DDIV                          ((uint32_t)(0x7FUL << MXC_F_UART_BAUD1_DDIV_POS)) /**< BAUD1_DDIV Mask */

/**@} end of group UART_BAUD1_Register */

/**
 * @ingroup  uart_registers
 * @defgroup UART_DATA UART_DATA
 * @brief    FIFO Data buffer.
 * @{
 */
#define MXC_F_UART_DATA_DATA_POS                       0 /**< DATA_DATA Position */
#define MXC_F_UART_DATA_DATA                           ((uint32_t)(0xFFUL << MXC_F_UART_DATA_DATA_POS)) /**< DATA_DATA Mask */

#define MXC_F_UART_DATA_PARITY_POS                     8 /**< DATA_PARITY Position */
#define MXC_F_UART_DATA_PARITY                         ((uint32_t)(0x1UL << MXC_F_UART_DATA_PARITY_POS)) /**< DATA_PARITY Mask */

/**@} end of group UART_DATA_Register */

/**
 * @ingroup  uart_registers
 * @defgroup UART_DMA UART_DMA
 * @brief    DMA Configuration.
 * @{
 */
#define MXC_F_UART_DMA_TXCNT_POS                       0 /**< DMA_TXCNT Position */
#define MXC_F_UART_DMA_TXCNT                           ((uint32_t)(0xFUL << MXC_F_UART_DMA_TXCNT_POS)) /**< DMA_TXCNT Mask */

#define MXC_F_UART_DMA_TXEN_POS                        4 /**< DMA_TXEN Position */
#define MXC_F_UART_DMA_TXEN                            ((uint32_t)(0x1UL << MXC_F_UART_DMA_TXEN_POS)) /**< DMA_TXEN Mask */

#define MXC_F_UART_DMA_RXCNT_POS                       5 /**< DMA_RXCNT Position */
#define MXC_F_UART_DMA_RXCNT                           ((uint32_t)(0xFUL << MXC_F_UART_DMA_RXCNT_POS)) /**< DMA_RXCNT Mask */

#define MXC_F_UART_DMA_RXEN_POS                        9 /**< DMA_RXEN Position */
#define MXC_F_UART_DMA_RXEN                            ((uint32_t)(0x1UL << MXC_F_UART_DMA_RXEN_POS)) /**< DMA_RXEN Mask */

/**@} end of group UART_DMA_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32520_INCLUDE_UART_REGS_H_
