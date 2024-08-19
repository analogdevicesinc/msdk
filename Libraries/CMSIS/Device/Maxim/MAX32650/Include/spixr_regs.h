/**
 * @file    spixr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SPIXR Peripheral Module.
 * @note    This file is @generated.
 * @ingroup spixr_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_SPIXR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_SPIXR_REGS_H_

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
 * @ingroup     spixr
 * @defgroup    spixr_registers SPIXR_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SPIXR Peripheral Module.
 * @details     SPIXR peripheral.
 */

/**
 * @ingroup spixr_registers
 * Structure type to access the SPIXR Registers.
 */
typedef struct {
    union {
        __IO uint32_t data32;           /**< <tt>\b 0x00:</tt> SPIXR DATA32 Register */
        __IO uint16_t data16[2];        /**< <tt>\b 0x00:</tt> SPIXR DATA16 Register */
        __IO uint8_t  data8[4];         /**< <tt>\b 0x00:</tt> SPIXR DATA8 Register */
    };
    __IO uint32_t ctrl1;                /**< <tt>\b 0x04:</tt> SPIXR CTRL1 Register */
    __IO uint32_t ctrl2;                /**< <tt>\b 0x08:</tt> SPIXR CTRL2 Register */
    __IO uint32_t ctrl3;                /**< <tt>\b 0x0C:</tt> SPIXR CTRL3 Register */
    __IO uint32_t ss_time;              /**< <tt>\b 0x10:</tt> SPIXR SS_TIME Register */
    __IO uint32_t brg_ctrl;             /**< <tt>\b 0x14:</tt> SPIXR BRG_CTRL Register */
    __R  uint32_t rsv_0x18;
    __IO uint32_t dma;                  /**< <tt>\b 0x1C:</tt> SPIXR DMA Register */
    __IO uint32_t int_fl;               /**< <tt>\b 0x20:</tt> SPIXR INT_FL Register */
    __IO uint32_t int_en;               /**< <tt>\b 0x24:</tt> SPIXR INT_EN Register */
    __IO uint32_t wake_fl;              /**< <tt>\b 0x28:</tt> SPIXR WAKE_FL Register */
    __IO uint32_t wake_en;              /**< <tt>\b 0x2C:</tt> SPIXR WAKE_EN Register */
    __I  uint32_t stat;                 /**< <tt>\b 0x30:</tt> SPIXR STAT Register */
    __IO uint32_t xmem_ctrl;            /**< <tt>\b 0x34:</tt> SPIXR XMEM_CTRL Register */
} mxc_spixr_regs_t;

/* Register offsets for module SPIXR */
/**
 * @ingroup    spixr_registers
 * @defgroup   SPIXR_Register_Offsets Register Offsets
 * @brief      SPIXR Peripheral Register Offsets from the SPIXR Base Peripheral Address.
 * @{
 */
#define MXC_R_SPIXR_DATA32                 ((uint32_t)0x00000000UL) /**< Offset from SPIXR Base Address: <tt> 0x0000</tt> */
#define MXC_R_SPIXR_DATA16                 ((uint32_t)0x00000000UL) /**< Offset from SPIXR Base Address: <tt> 0x0000</tt> */
#define MXC_R_SPIXR_DATA8                  ((uint32_t)0x00000000UL) /**< Offset from SPIXR Base Address: <tt> 0x0000</tt> */
#define MXC_R_SPIXR_CTRL1                  ((uint32_t)0x00000004UL) /**< Offset from SPIXR Base Address: <tt> 0x0004</tt> */
#define MXC_R_SPIXR_CTRL2                  ((uint32_t)0x00000008UL) /**< Offset from SPIXR Base Address: <tt> 0x0008</tt> */
#define MXC_R_SPIXR_CTRL3                  ((uint32_t)0x0000000CUL) /**< Offset from SPIXR Base Address: <tt> 0x000C</tt> */
#define MXC_R_SPIXR_SS_TIME                ((uint32_t)0x00000010UL) /**< Offset from SPIXR Base Address: <tt> 0x0010</tt> */
#define MXC_R_SPIXR_BRG_CTRL               ((uint32_t)0x00000014UL) /**< Offset from SPIXR Base Address: <tt> 0x0014</tt> */
#define MXC_R_SPIXR_DMA                    ((uint32_t)0x0000001CUL) /**< Offset from SPIXR Base Address: <tt> 0x001C</tt> */
#define MXC_R_SPIXR_INT_FL                 ((uint32_t)0x00000020UL) /**< Offset from SPIXR Base Address: <tt> 0x0020</tt> */
#define MXC_R_SPIXR_INT_EN                 ((uint32_t)0x00000024UL) /**< Offset from SPIXR Base Address: <tt> 0x0024</tt> */
#define MXC_R_SPIXR_WAKE_FL                ((uint32_t)0x00000028UL) /**< Offset from SPIXR Base Address: <tt> 0x0028</tt> */
#define MXC_R_SPIXR_WAKE_EN                ((uint32_t)0x0000002CUL) /**< Offset from SPIXR Base Address: <tt> 0x002C</tt> */
#define MXC_R_SPIXR_STAT                   ((uint32_t)0x00000030UL) /**< Offset from SPIXR Base Address: <tt> 0x0030</tt> */
#define MXC_R_SPIXR_XMEM_CTRL              ((uint32_t)0x00000034UL) /**< Offset from SPIXR Base Address: <tt> 0x0034</tt> */
/**@} end of group spixr_registers */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_DATA32 SPIXR_DATA32
 * @brief    Register for reading and writing the FIFO.
 * @{
 */
#define MXC_F_SPIXR_DATA32_DATA_POS                    0 /**< DATA32_DATA Position */
#define MXC_F_SPIXR_DATA32_DATA                        ((uint32_t)(0xFFFFFFFFUL << MXC_F_SPIXR_DATA32_DATA_POS)) /**< DATA32_DATA Mask */

/**@} end of group SPIXR_DATA32_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_DATA16 SPIXR_DATA16
 * @brief    Register for reading and writing the FIFO.
 * @{
 */
#define MXC_F_SPIXR_DATA16_DATA_POS                    0 /**< DATA16_DATA Position */
#define MXC_F_SPIXR_DATA16_DATA                        ((uint16_t)(0xFFFFUL << MXC_F_SPIXR_DATA16_DATA_POS)) /**< DATA16_DATA Mask */

/**@} end of group SPIXR_DATA16_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_DATA8 SPIXR_DATA8
 * @brief    Register for reading and writing the FIFO.
 * @{
 */
#define MXC_F_SPIXR_DATA8_DATA_POS                     0 /**< DATA8_DATA Position */
#define MXC_F_SPIXR_DATA8_DATA                         ((uint8_t)(0xFFUL << MXC_F_SPIXR_DATA8_DATA_POS)) /**< DATA8_DATA Mask */

/**@} end of group SPIXR_DATA8_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_CTRL1 SPIXR_CTRL1
 * @brief    Register for controlling SPI peripheral.
 * @{
 */
#define MXC_F_SPIXR_CTRL1_ENABLE_POS                   0 /**< CTRL1_ENABLE Position */
#define MXC_F_SPIXR_CTRL1_ENABLE                       ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL1_ENABLE_POS)) /**< CTRL1_ENABLE Mask */
#define MXC_V_SPIXR_CTRL1_ENABLE_DIS                   ((uint32_t)0x0UL) /**< CTRL1_ENABLE_DIS Value */
#define MXC_S_SPIXR_CTRL1_ENABLE_DIS                   (MXC_V_SPIXR_CTRL1_ENABLE_DIS << MXC_F_SPIXR_CTRL1_ENABLE_POS) /**< CTRL1_ENABLE_DIS Setting */
#define MXC_V_SPIXR_CTRL1_ENABLE_EN                    ((uint32_t)0x1UL) /**< CTRL1_ENABLE_EN Value */
#define MXC_S_SPIXR_CTRL1_ENABLE_EN                    (MXC_V_SPIXR_CTRL1_ENABLE_EN << MXC_F_SPIXR_CTRL1_ENABLE_POS) /**< CTRL1_ENABLE_EN Setting */

#define MXC_F_SPIXR_CTRL1_MASTER_POS                   1 /**< CTRL1_MASTER Position */
#define MXC_F_SPIXR_CTRL1_MASTER                       ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL1_MASTER_POS)) /**< CTRL1_MASTER Mask */
#define MXC_V_SPIXR_CTRL1_MASTER_DIS                   ((uint32_t)0x0UL) /**< CTRL1_MASTER_DIS Value */
#define MXC_S_SPIXR_CTRL1_MASTER_DIS                   (MXC_V_SPIXR_CTRL1_MASTER_DIS << MXC_F_SPIXR_CTRL1_MASTER_POS) /**< CTRL1_MASTER_DIS Setting */
#define MXC_V_SPIXR_CTRL1_MASTER_EN                    ((uint32_t)0x1UL) /**< CTRL1_MASTER_EN Value */
#define MXC_S_SPIXR_CTRL1_MASTER_EN                    (MXC_V_SPIXR_CTRL1_MASTER_EN << MXC_F_SPIXR_CTRL1_MASTER_POS) /**< CTRL1_MASTER_EN Setting */

#define MXC_F_SPIXR_CTRL1_SS_IO_POS                    4 /**< CTRL1_SS_IO Position */
#define MXC_F_SPIXR_CTRL1_SS_IO                        ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL1_SS_IO_POS)) /**< CTRL1_SS_IO Mask */
#define MXC_V_SPIXR_CTRL1_SS_IO_OUTPUT                 ((uint32_t)0x0UL) /**< CTRL1_SS_IO_OUTPUT Value */
#define MXC_S_SPIXR_CTRL1_SS_IO_OUTPUT                 (MXC_V_SPIXR_CTRL1_SS_IO_OUTPUT << MXC_F_SPIXR_CTRL1_SS_IO_POS) /**< CTRL1_SS_IO_OUTPUT Setting */
#define MXC_V_SPIXR_CTRL1_SS_IO_INPUT                  ((uint32_t)0x1UL) /**< CTRL1_SS_IO_INPUT Value */
#define MXC_S_SPIXR_CTRL1_SS_IO_INPUT                  (MXC_V_SPIXR_CTRL1_SS_IO_INPUT << MXC_F_SPIXR_CTRL1_SS_IO_POS) /**< CTRL1_SS_IO_INPUT Setting */

#define MXC_F_SPIXR_CTRL1_START_POS                    5 /**< CTRL1_START Position */
#define MXC_F_SPIXR_CTRL1_START                        ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL1_START_POS)) /**< CTRL1_START Mask */
#define MXC_V_SPIXR_CTRL1_START_START                  ((uint32_t)0x1UL) /**< CTRL1_START_START Value */
#define MXC_S_SPIXR_CTRL1_START_START                  (MXC_V_SPIXR_CTRL1_START_START << MXC_F_SPIXR_CTRL1_START_POS) /**< CTRL1_START_START Setting */

#define MXC_F_SPIXR_CTRL1_SS_CTRL_POS                  8 /**< CTRL1_SS_CTRL Position */
#define MXC_F_SPIXR_CTRL1_SS_CTRL                      ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL1_SS_CTRL_POS)) /**< CTRL1_SS_CTRL Mask */
#define MXC_V_SPIXR_CTRL1_SS_CTRL_DEASSERT             ((uint32_t)0x0UL) /**< CTRL1_SS_CTRL_DEASSERT Value */
#define MXC_S_SPIXR_CTRL1_SS_CTRL_DEASSERT             (MXC_V_SPIXR_CTRL1_SS_CTRL_DEASSERT << MXC_F_SPIXR_CTRL1_SS_CTRL_POS) /**< CTRL1_SS_CTRL_DEASSERT Setting */
#define MXC_V_SPIXR_CTRL1_SS_CTRL_ASSERT               ((uint32_t)0x1UL) /**< CTRL1_SS_CTRL_ASSERT Value */
#define MXC_S_SPIXR_CTRL1_SS_CTRL_ASSERT               (MXC_V_SPIXR_CTRL1_SS_CTRL_ASSERT << MXC_F_SPIXR_CTRL1_SS_CTRL_POS) /**< CTRL1_SS_CTRL_ASSERT Setting */

#define MXC_F_SPIXR_CTRL1_SS_POS                       16 /**< CTRL1_SS Position */
#define MXC_F_SPIXR_CTRL1_SS                           ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL1_SS_POS)) /**< CTRL1_SS Mask */

/**@} end of group SPIXR_CTRL1_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_CTRL2 SPIXR_CTRL2
 * @brief    Register for controlling SPI peripheral.
 * @{
 */
#define MXC_F_SPIXR_CTRL2_TX_NUM_CHAR_POS              0 /**< CTRL2_TX_NUM_CHAR Position */
#define MXC_F_SPIXR_CTRL2_TX_NUM_CHAR                  ((uint32_t)(0xFFFFUL << MXC_F_SPIXR_CTRL2_TX_NUM_CHAR_POS)) /**< CTRL2_TX_NUM_CHAR Mask */

#define MXC_F_SPIXR_CTRL2_RX_NUM_CHAR_POS              16 /**< CTRL2_RX_NUM_CHAR Position */
#define MXC_F_SPIXR_CTRL2_RX_NUM_CHAR                  ((uint32_t)(0xFFFFUL << MXC_F_SPIXR_CTRL2_RX_NUM_CHAR_POS)) /**< CTRL2_RX_NUM_CHAR Mask */

/**@} end of group SPIXR_CTRL2_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_CTRL3 SPIXR_CTRL3
 * @brief    Register for controlling SPI peripheral.
 * @{
 */
#define MXC_F_SPIXR_CTRL3_CPHA_POS                     0 /**< CTRL3_CPHA Position */
#define MXC_F_SPIXR_CTRL3_CPHA                         ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL3_CPHA_POS)) /**< CTRL3_CPHA Mask */
#define MXC_V_SPIXR_CTRL3_CPHA_RISINGEDGE              ((uint32_t)0x0UL) /**< CTRL3_CPHA_RISINGEDGE Value */
#define MXC_S_SPIXR_CTRL3_CPHA_RISINGEDGE              (MXC_V_SPIXR_CTRL3_CPHA_RISINGEDGE << MXC_F_SPIXR_CTRL3_CPHA_POS) /**< CTRL3_CPHA_RISINGEDGE Setting */
#define MXC_V_SPIXR_CTRL3_CPHA_FALLINGEDGE             ((uint32_t)0x1UL) /**< CTRL3_CPHA_FALLINGEDGE Value */
#define MXC_S_SPIXR_CTRL3_CPHA_FALLINGEDGE             (MXC_V_SPIXR_CTRL3_CPHA_FALLINGEDGE << MXC_F_SPIXR_CTRL3_CPHA_POS) /**< CTRL3_CPHA_FALLINGEDGE Setting */

#define MXC_F_SPIXR_CTRL3_CPOL_POS                     1 /**< CTRL3_CPOL Position */
#define MXC_F_SPIXR_CTRL3_CPOL                         ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL3_CPOL_POS)) /**< CTRL3_CPOL Mask */
#define MXC_V_SPIXR_CTRL3_CPOL_NORMAL                  ((uint32_t)0x0UL) /**< CTRL3_CPOL_NORMAL Value */
#define MXC_S_SPIXR_CTRL3_CPOL_NORMAL                  (MXC_V_SPIXR_CTRL3_CPOL_NORMAL << MXC_F_SPIXR_CTRL3_CPOL_POS) /**< CTRL3_CPOL_NORMAL Setting */
#define MXC_V_SPIXR_CTRL3_CPOL_INVERTED                ((uint32_t)0x1UL) /**< CTRL3_CPOL_INVERTED Value */
#define MXC_S_SPIXR_CTRL3_CPOL_INVERTED                (MXC_V_SPIXR_CTRL3_CPOL_INVERTED << MXC_F_SPIXR_CTRL3_CPOL_POS) /**< CTRL3_CPOL_INVERTED Setting */

#define MXC_F_SPIXR_CTRL3_SCLK_INV_POS                 4 /**< CTRL3_SCLK_INV Position */
#define MXC_F_SPIXR_CTRL3_SCLK_INV                     ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL3_SCLK_INV_POS)) /**< CTRL3_SCLK_INV Mask */
#define MXC_V_SPIXR_CTRL3_SCLK_INV_NORMAL              ((uint32_t)0x0UL) /**< CTRL3_SCLK_INV_NORMAL Value */
#define MXC_S_SPIXR_CTRL3_SCLK_INV_NORMAL              (MXC_V_SPIXR_CTRL3_SCLK_INV_NORMAL << MXC_F_SPIXR_CTRL3_SCLK_INV_POS) /**< CTRL3_SCLK_INV_NORMAL Setting */

#define MXC_F_SPIXR_CTRL3_NUMBITS_POS                  8 /**< CTRL3_NUMBITS Position */
#define MXC_F_SPIXR_CTRL3_NUMBITS                      ((uint32_t)(0xFUL << MXC_F_SPIXR_CTRL3_NUMBITS_POS)) /**< CTRL3_NUMBITS Mask */
#define MXC_V_SPIXR_CTRL3_NUMBITS_16BITS               ((uint32_t)0x0UL) /**< CTRL3_NUMBITS_16BITS Value */
#define MXC_S_SPIXR_CTRL3_NUMBITS_16BITS               (MXC_V_SPIXR_CTRL3_NUMBITS_16BITS << MXC_F_SPIXR_CTRL3_NUMBITS_POS) /**< CTRL3_NUMBITS_16BITS Setting */
#define MXC_V_SPIXR_CTRL3_NUMBITS_1BITS                ((uint32_t)0x1UL) /**< CTRL3_NUMBITS_1BITS Value */
#define MXC_S_SPIXR_CTRL3_NUMBITS_1BITS                (MXC_V_SPIXR_CTRL3_NUMBITS_1BITS << MXC_F_SPIXR_CTRL3_NUMBITS_POS) /**< CTRL3_NUMBITS_1BITS Setting */
#define MXC_V_SPIXR_CTRL3_NUMBITS_2BITS                ((uint32_t)0x2UL) /**< CTRL3_NUMBITS_2BITS Value */
#define MXC_S_SPIXR_CTRL3_NUMBITS_2BITS                (MXC_V_SPIXR_CTRL3_NUMBITS_2BITS << MXC_F_SPIXR_CTRL3_NUMBITS_POS) /**< CTRL3_NUMBITS_2BITS Setting */
#define MXC_V_SPIXR_CTRL3_NUMBITS_3BITS                ((uint32_t)0x3UL) /**< CTRL3_NUMBITS_3BITS Value */
#define MXC_S_SPIXR_CTRL3_NUMBITS_3BITS                (MXC_V_SPIXR_CTRL3_NUMBITS_3BITS << MXC_F_SPIXR_CTRL3_NUMBITS_POS) /**< CTRL3_NUMBITS_3BITS Setting */
#define MXC_V_SPIXR_CTRL3_NUMBITS_4BITS                ((uint32_t)0x4UL) /**< CTRL3_NUMBITS_4BITS Value */
#define MXC_S_SPIXR_CTRL3_NUMBITS_4BITS                (MXC_V_SPIXR_CTRL3_NUMBITS_4BITS << MXC_F_SPIXR_CTRL3_NUMBITS_POS) /**< CTRL3_NUMBITS_4BITS Setting */
#define MXC_V_SPIXR_CTRL3_NUMBITS_5BITS                ((uint32_t)0x5UL) /**< CTRL3_NUMBITS_5BITS Value */
#define MXC_S_SPIXR_CTRL3_NUMBITS_5BITS                (MXC_V_SPIXR_CTRL3_NUMBITS_5BITS << MXC_F_SPIXR_CTRL3_NUMBITS_POS) /**< CTRL3_NUMBITS_5BITS Setting */
#define MXC_V_SPIXR_CTRL3_NUMBITS_6BITS                ((uint32_t)0x6UL) /**< CTRL3_NUMBITS_6BITS Value */
#define MXC_S_SPIXR_CTRL3_NUMBITS_6BITS                (MXC_V_SPIXR_CTRL3_NUMBITS_6BITS << MXC_F_SPIXR_CTRL3_NUMBITS_POS) /**< CTRL3_NUMBITS_6BITS Setting */
#define MXC_V_SPIXR_CTRL3_NUMBITS_7BITS                ((uint32_t)0x7UL) /**< CTRL3_NUMBITS_7BITS Value */
#define MXC_S_SPIXR_CTRL3_NUMBITS_7BITS                (MXC_V_SPIXR_CTRL3_NUMBITS_7BITS << MXC_F_SPIXR_CTRL3_NUMBITS_POS) /**< CTRL3_NUMBITS_7BITS Setting */
#define MXC_V_SPIXR_CTRL3_NUMBITS_8BITS                ((uint32_t)0x8UL) /**< CTRL3_NUMBITS_8BITS Value */
#define MXC_S_SPIXR_CTRL3_NUMBITS_8BITS                (MXC_V_SPIXR_CTRL3_NUMBITS_8BITS << MXC_F_SPIXR_CTRL3_NUMBITS_POS) /**< CTRL3_NUMBITS_8BITS Setting */
#define MXC_V_SPIXR_CTRL3_NUMBITS_9BITS                ((uint32_t)0x9UL) /**< CTRL3_NUMBITS_9BITS Value */
#define MXC_S_SPIXR_CTRL3_NUMBITS_9BITS                (MXC_V_SPIXR_CTRL3_NUMBITS_9BITS << MXC_F_SPIXR_CTRL3_NUMBITS_POS) /**< CTRL3_NUMBITS_9BITS Setting */
#define MXC_V_SPIXR_CTRL3_NUMBITS_10BITS               ((uint32_t)0xAUL) /**< CTRL3_NUMBITS_10BITS Value */
#define MXC_S_SPIXR_CTRL3_NUMBITS_10BITS               (MXC_V_SPIXR_CTRL3_NUMBITS_10BITS << MXC_F_SPIXR_CTRL3_NUMBITS_POS) /**< CTRL3_NUMBITS_10BITS Setting */
#define MXC_V_SPIXR_CTRL3_NUMBITS_11BITS               ((uint32_t)0xBUL) /**< CTRL3_NUMBITS_11BITS Value */
#define MXC_S_SPIXR_CTRL3_NUMBITS_11BITS               (MXC_V_SPIXR_CTRL3_NUMBITS_11BITS << MXC_F_SPIXR_CTRL3_NUMBITS_POS) /**< CTRL3_NUMBITS_11BITS Setting */
#define MXC_V_SPIXR_CTRL3_NUMBITS_12BITS               ((uint32_t)0xCUL) /**< CTRL3_NUMBITS_12BITS Value */
#define MXC_S_SPIXR_CTRL3_NUMBITS_12BITS               (MXC_V_SPIXR_CTRL3_NUMBITS_12BITS << MXC_F_SPIXR_CTRL3_NUMBITS_POS) /**< CTRL3_NUMBITS_12BITS Setting */
#define MXC_V_SPIXR_CTRL3_NUMBITS_13BITS               ((uint32_t)0xDUL) /**< CTRL3_NUMBITS_13BITS Value */
#define MXC_S_SPIXR_CTRL3_NUMBITS_13BITS               (MXC_V_SPIXR_CTRL3_NUMBITS_13BITS << MXC_F_SPIXR_CTRL3_NUMBITS_POS) /**< CTRL3_NUMBITS_13BITS Setting */
#define MXC_V_SPIXR_CTRL3_NUMBITS_14BITS               ((uint32_t)0xEUL) /**< CTRL3_NUMBITS_14BITS Value */
#define MXC_S_SPIXR_CTRL3_NUMBITS_14BITS               (MXC_V_SPIXR_CTRL3_NUMBITS_14BITS << MXC_F_SPIXR_CTRL3_NUMBITS_POS) /**< CTRL3_NUMBITS_14BITS Setting */
#define MXC_V_SPIXR_CTRL3_NUMBITS_15BITS               ((uint32_t)0xFUL) /**< CTRL3_NUMBITS_15BITS Value */
#define MXC_S_SPIXR_CTRL3_NUMBITS_15BITS               (MXC_V_SPIXR_CTRL3_NUMBITS_15BITS << MXC_F_SPIXR_CTRL3_NUMBITS_POS) /**< CTRL3_NUMBITS_15BITS Setting */

#define MXC_F_SPIXR_CTRL3_DATA_WIDTH_POS               12 /**< CTRL3_DATA_WIDTH Position */
#define MXC_F_SPIXR_CTRL3_DATA_WIDTH                   ((uint32_t)(0x3UL << MXC_F_SPIXR_CTRL3_DATA_WIDTH_POS)) /**< CTRL3_DATA_WIDTH Mask */
#define MXC_V_SPIXR_CTRL3_DATA_WIDTH_MONO              ((uint32_t)0x0UL) /**< CTRL3_DATA_WIDTH_MONO Value */
#define MXC_S_SPIXR_CTRL3_DATA_WIDTH_MONO              (MXC_V_SPIXR_CTRL3_DATA_WIDTH_MONO << MXC_F_SPIXR_CTRL3_DATA_WIDTH_POS) /**< CTRL3_DATA_WIDTH_MONO Setting */
#define MXC_V_SPIXR_CTRL3_DATA_WIDTH_DUAL              ((uint32_t)0x1UL) /**< CTRL3_DATA_WIDTH_DUAL Value */
#define MXC_S_SPIXR_CTRL3_DATA_WIDTH_DUAL              (MXC_V_SPIXR_CTRL3_DATA_WIDTH_DUAL << MXC_F_SPIXR_CTRL3_DATA_WIDTH_POS) /**< CTRL3_DATA_WIDTH_DUAL Setting */
#define MXC_V_SPIXR_CTRL3_DATA_WIDTH_QUAD              ((uint32_t)0x2UL) /**< CTRL3_DATA_WIDTH_QUAD Value */
#define MXC_S_SPIXR_CTRL3_DATA_WIDTH_QUAD              (MXC_V_SPIXR_CTRL3_DATA_WIDTH_QUAD << MXC_F_SPIXR_CTRL3_DATA_WIDTH_POS) /**< CTRL3_DATA_WIDTH_QUAD Setting */

#define MXC_F_SPIXR_CTRL3_THREE_WIRE_POS               15 /**< CTRL3_THREE_WIRE Position */
#define MXC_F_SPIXR_CTRL3_THREE_WIRE                   ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL3_THREE_WIRE_POS)) /**< CTRL3_THREE_WIRE Mask */
#define MXC_V_SPIXR_CTRL3_THREE_WIRE_4WIRE             ((uint32_t)0x0UL) /**< CTRL3_THREE_WIRE_4WIRE Value */
#define MXC_S_SPIXR_CTRL3_THREE_WIRE_4WIRE             (MXC_V_SPIXR_CTRL3_THREE_WIRE_4WIRE << MXC_F_SPIXR_CTRL3_THREE_WIRE_POS) /**< CTRL3_THREE_WIRE_4WIRE Setting */
#define MXC_V_SPIXR_CTRL3_THREE_WIRE_3WIRE             ((uint32_t)0x1UL) /**< CTRL3_THREE_WIRE_3WIRE Value */
#define MXC_S_SPIXR_CTRL3_THREE_WIRE_3WIRE             (MXC_V_SPIXR_CTRL3_THREE_WIRE_3WIRE << MXC_F_SPIXR_CTRL3_THREE_WIRE_POS) /**< CTRL3_THREE_WIRE_3WIRE Setting */

#define MXC_F_SPIXR_CTRL3_SSPOL_POS                    16 /**< CTRL3_SSPOL Position */
#define MXC_F_SPIXR_CTRL3_SSPOL                        ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL3_SSPOL_POS)) /**< CTRL3_SSPOL Mask */
#define MXC_V_SPIXR_CTRL3_SSPOL_ACTIVELOW              ((uint32_t)0x0UL) /**< CTRL3_SSPOL_ACTIVELOW Value */
#define MXC_S_SPIXR_CTRL3_SSPOL_ACTIVELOW              (MXC_V_SPIXR_CTRL3_SSPOL_ACTIVELOW << MXC_F_SPIXR_CTRL3_SSPOL_POS) /**< CTRL3_SSPOL_ACTIVELOW Setting */
#define MXC_V_SPIXR_CTRL3_SSPOL_ACTIVEHIGH             ((uint32_t)0x1UL) /**< CTRL3_SSPOL_ACTIVEHIGH Value */
#define MXC_S_SPIXR_CTRL3_SSPOL_ACTIVEHIGH             (MXC_V_SPIXR_CTRL3_SSPOL_ACTIVEHIGH << MXC_F_SPIXR_CTRL3_SSPOL_POS) /**< CTRL3_SSPOL_ACTIVEHIGH Setting */

/**@} end of group SPIXR_CTRL3_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_SS_TIME SPIXR_SS_TIME
 * @brief    Register for controlling SPI peripheral.
 * @{
 */
#define MXC_F_SPIXR_SS_TIME_SSACT1_POS                 0 /**< SS_TIME_SSACT1 Position */
#define MXC_F_SPIXR_SS_TIME_SSACT1                     ((uint32_t)(0xFFUL << MXC_F_SPIXR_SS_TIME_SSACT1_POS)) /**< SS_TIME_SSACT1 Mask */

#define MXC_F_SPIXR_SS_TIME_SSACT2_POS                 8 /**< SS_TIME_SSACT2 Position */
#define MXC_F_SPIXR_SS_TIME_SSACT2                     ((uint32_t)(0xFFUL << MXC_F_SPIXR_SS_TIME_SSACT2_POS)) /**< SS_TIME_SSACT2 Mask */

#define MXC_F_SPIXR_SS_TIME_SSINACT_POS                16 /**< SS_TIME_SSINACT Position */
#define MXC_F_SPIXR_SS_TIME_SSINACT                    ((uint32_t)(0xFFUL << MXC_F_SPIXR_SS_TIME_SSINACT_POS)) /**< SS_TIME_SSINACT Mask */

/**@} end of group SPIXR_SS_TIME_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_BRG_CTRL SPIXR_BRG_CTRL
 * @brief    Register for controlling SPI clock rate.
 * @{
 */
#define MXC_F_SPIXR_BRG_CTRL_LO_POS                    0 /**< BRG_CTRL_LO Position */
#define MXC_F_SPIXR_BRG_CTRL_LO                        ((uint32_t)(0xFFUL << MXC_F_SPIXR_BRG_CTRL_LO_POS)) /**< BRG_CTRL_LO Mask */

#define MXC_F_SPIXR_BRG_CTRL_HI_POS                    8 /**< BRG_CTRL_HI Position */
#define MXC_F_SPIXR_BRG_CTRL_HI                        ((uint32_t)(0xFFUL << MXC_F_SPIXR_BRG_CTRL_HI_POS)) /**< BRG_CTRL_HI Mask */

#define MXC_F_SPIXR_BRG_CTRL_SCALE_POS                 16 /**< BRG_CTRL_SCALE Position */
#define MXC_F_SPIXR_BRG_CTRL_SCALE                     ((uint32_t)(0xFUL << MXC_F_SPIXR_BRG_CTRL_SCALE_POS)) /**< BRG_CTRL_SCALE Mask */
#define MXC_V_SPIXR_BRG_CTRL_SCALE_DIV1                ((uint32_t)0x0UL) /**< BRG_CTRL_SCALE_DIV1 Value */
#define MXC_S_SPIXR_BRG_CTRL_SCALE_DIV1                (MXC_V_SPIXR_BRG_CTRL_SCALE_DIV1 << MXC_F_SPIXR_BRG_CTRL_SCALE_POS) /**< BRG_CTRL_SCALE_DIV1 Setting */
#define MXC_V_SPIXR_BRG_CTRL_SCALE_DIV2                ((uint32_t)0x1UL) /**< BRG_CTRL_SCALE_DIV2 Value */
#define MXC_S_SPIXR_BRG_CTRL_SCALE_DIV2                (MXC_V_SPIXR_BRG_CTRL_SCALE_DIV2 << MXC_F_SPIXR_BRG_CTRL_SCALE_POS) /**< BRG_CTRL_SCALE_DIV2 Setting */
#define MXC_V_SPIXR_BRG_CTRL_SCALE_DIV4                ((uint32_t)0x2UL) /**< BRG_CTRL_SCALE_DIV4 Value */
#define MXC_S_SPIXR_BRG_CTRL_SCALE_DIV4                (MXC_V_SPIXR_BRG_CTRL_SCALE_DIV4 << MXC_F_SPIXR_BRG_CTRL_SCALE_POS) /**< BRG_CTRL_SCALE_DIV4 Setting */
#define MXC_V_SPIXR_BRG_CTRL_SCALE_DIV8                ((uint32_t)0x3UL) /**< BRG_CTRL_SCALE_DIV8 Value */
#define MXC_S_SPIXR_BRG_CTRL_SCALE_DIV8                (MXC_V_SPIXR_BRG_CTRL_SCALE_DIV8 << MXC_F_SPIXR_BRG_CTRL_SCALE_POS) /**< BRG_CTRL_SCALE_DIV8 Setting */
#define MXC_V_SPIXR_BRG_CTRL_SCALE_DIV16               ((uint32_t)0x4UL) /**< BRG_CTRL_SCALE_DIV16 Value */
#define MXC_S_SPIXR_BRG_CTRL_SCALE_DIV16               (MXC_V_SPIXR_BRG_CTRL_SCALE_DIV16 << MXC_F_SPIXR_BRG_CTRL_SCALE_POS) /**< BRG_CTRL_SCALE_DIV16 Setting */
#define MXC_V_SPIXR_BRG_CTRL_SCALE_DIV32               ((uint32_t)0x5UL) /**< BRG_CTRL_SCALE_DIV32 Value */
#define MXC_S_SPIXR_BRG_CTRL_SCALE_DIV32               (MXC_V_SPIXR_BRG_CTRL_SCALE_DIV32 << MXC_F_SPIXR_BRG_CTRL_SCALE_POS) /**< BRG_CTRL_SCALE_DIV32 Setting */
#define MXC_V_SPIXR_BRG_CTRL_SCALE_DIV64               ((uint32_t)0x6UL) /**< BRG_CTRL_SCALE_DIV64 Value */
#define MXC_S_SPIXR_BRG_CTRL_SCALE_DIV64               (MXC_V_SPIXR_BRG_CTRL_SCALE_DIV64 << MXC_F_SPIXR_BRG_CTRL_SCALE_POS) /**< BRG_CTRL_SCALE_DIV64 Setting */
#define MXC_V_SPIXR_BRG_CTRL_SCALE_DIV128              ((uint32_t)0x7UL) /**< BRG_CTRL_SCALE_DIV128 Value */
#define MXC_S_SPIXR_BRG_CTRL_SCALE_DIV128              (MXC_V_SPIXR_BRG_CTRL_SCALE_DIV128 << MXC_F_SPIXR_BRG_CTRL_SCALE_POS) /**< BRG_CTRL_SCALE_DIV128 Setting */
#define MXC_V_SPIXR_BRG_CTRL_SCALE_DIV256              ((uint32_t)0x8UL) /**< BRG_CTRL_SCALE_DIV256 Value */
#define MXC_S_SPIXR_BRG_CTRL_SCALE_DIV256              (MXC_V_SPIXR_BRG_CTRL_SCALE_DIV256 << MXC_F_SPIXR_BRG_CTRL_SCALE_POS) /**< BRG_CTRL_SCALE_DIV256 Setting */

/**@} end of group SPIXR_BRG_CTRL_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_DMA SPIXR_DMA
 * @brief    Register for controlling DMA.
 * @{
 */
#define MXC_F_SPIXR_DMA_TX_FIFO_LEVEL_POS              0 /**< DMA_TX_FIFO_LEVEL Position */
#define MXC_F_SPIXR_DMA_TX_FIFO_LEVEL                  ((uint32_t)(0x1FUL << MXC_F_SPIXR_DMA_TX_FIFO_LEVEL_POS)) /**< DMA_TX_FIFO_LEVEL Mask */

#define MXC_F_SPIXR_DMA_TX_FIFO_EN_POS                 6 /**< DMA_TX_FIFO_EN Position */
#define MXC_F_SPIXR_DMA_TX_FIFO_EN                     ((uint32_t)(0x1UL << MXC_F_SPIXR_DMA_TX_FIFO_EN_POS)) /**< DMA_TX_FIFO_EN Mask */
#define MXC_V_SPIXR_DMA_TX_FIFO_EN_DIS                 ((uint32_t)0x0UL) /**< DMA_TX_FIFO_EN_DIS Value */
#define MXC_S_SPIXR_DMA_TX_FIFO_EN_DIS                 (MXC_V_SPIXR_DMA_TX_FIFO_EN_DIS << MXC_F_SPIXR_DMA_TX_FIFO_EN_POS) /**< DMA_TX_FIFO_EN_DIS Setting */
#define MXC_V_SPIXR_DMA_TX_FIFO_EN_EN                  ((uint32_t)0x1UL) /**< DMA_TX_FIFO_EN_EN Value */
#define MXC_S_SPIXR_DMA_TX_FIFO_EN_EN                  (MXC_V_SPIXR_DMA_TX_FIFO_EN_EN << MXC_F_SPIXR_DMA_TX_FIFO_EN_POS) /**< DMA_TX_FIFO_EN_EN Setting */

#define MXC_F_SPIXR_DMA_TX_FIFO_CLEAR_POS              7 /**< DMA_TX_FIFO_CLEAR Position */
#define MXC_F_SPIXR_DMA_TX_FIFO_CLEAR                  ((uint32_t)(0x1UL << MXC_F_SPIXR_DMA_TX_FIFO_CLEAR_POS)) /**< DMA_TX_FIFO_CLEAR Mask */
#define MXC_V_SPIXR_DMA_TX_FIFO_CLEAR_CLEAR            ((uint32_t)0x1UL) /**< DMA_TX_FIFO_CLEAR_CLEAR Value */
#define MXC_S_SPIXR_DMA_TX_FIFO_CLEAR_CLEAR            (MXC_V_SPIXR_DMA_TX_FIFO_CLEAR_CLEAR << MXC_F_SPIXR_DMA_TX_FIFO_CLEAR_POS) /**< DMA_TX_FIFO_CLEAR_CLEAR Setting */

#define MXC_F_SPIXR_DMA_TX_FIFO_CNT_POS                8 /**< DMA_TX_FIFO_CNT Position */
#define MXC_F_SPIXR_DMA_TX_FIFO_CNT                    ((uint32_t)(0x3FUL << MXC_F_SPIXR_DMA_TX_FIFO_CNT_POS)) /**< DMA_TX_FIFO_CNT Mask */

#define MXC_F_SPIXR_DMA_TX_DMA_EN_POS                  15 /**< DMA_TX_DMA_EN Position */
#define MXC_F_SPIXR_DMA_TX_DMA_EN                      ((uint32_t)(0x1UL << MXC_F_SPIXR_DMA_TX_DMA_EN_POS)) /**< DMA_TX_DMA_EN Mask */
#define MXC_V_SPIXR_DMA_TX_DMA_EN_DIS                  ((uint32_t)0x0UL) /**< DMA_TX_DMA_EN_DIS Value */
#define MXC_S_SPIXR_DMA_TX_DMA_EN_DIS                  (MXC_V_SPIXR_DMA_TX_DMA_EN_DIS << MXC_F_SPIXR_DMA_TX_DMA_EN_POS) /**< DMA_TX_DMA_EN_DIS Setting */
#define MXC_V_SPIXR_DMA_TX_DMA_EN_EN                   ((uint32_t)0x1UL) /**< DMA_TX_DMA_EN_EN Value */
#define MXC_S_SPIXR_DMA_TX_DMA_EN_EN                   (MXC_V_SPIXR_DMA_TX_DMA_EN_EN << MXC_F_SPIXR_DMA_TX_DMA_EN_POS) /**< DMA_TX_DMA_EN_EN Setting */

#define MXC_F_SPIXR_DMA_RX_FIFO_LEVEL_POS              16 /**< DMA_RX_FIFO_LEVEL Position */
#define MXC_F_SPIXR_DMA_RX_FIFO_LEVEL                  ((uint32_t)(0x1FUL << MXC_F_SPIXR_DMA_RX_FIFO_LEVEL_POS)) /**< DMA_RX_FIFO_LEVEL Mask */

#define MXC_F_SPIXR_DMA_RX_FIFO_EN_POS                 22 /**< DMA_RX_FIFO_EN Position */
#define MXC_F_SPIXR_DMA_RX_FIFO_EN                     ((uint32_t)(0x1UL << MXC_F_SPIXR_DMA_RX_FIFO_EN_POS)) /**< DMA_RX_FIFO_EN Mask */
#define MXC_V_SPIXR_DMA_RX_FIFO_EN_DIS                 ((uint32_t)0x0UL) /**< DMA_RX_FIFO_EN_DIS Value */
#define MXC_S_SPIXR_DMA_RX_FIFO_EN_DIS                 (MXC_V_SPIXR_DMA_RX_FIFO_EN_DIS << MXC_F_SPIXR_DMA_RX_FIFO_EN_POS) /**< DMA_RX_FIFO_EN_DIS Setting */
#define MXC_V_SPIXR_DMA_RX_FIFO_EN_EN                  ((uint32_t)0x1UL) /**< DMA_RX_FIFO_EN_EN Value */
#define MXC_S_SPIXR_DMA_RX_FIFO_EN_EN                  (MXC_V_SPIXR_DMA_RX_FIFO_EN_EN << MXC_F_SPIXR_DMA_RX_FIFO_EN_POS) /**< DMA_RX_FIFO_EN_EN Setting */

#define MXC_F_SPIXR_DMA_RX_FIFO_CLEAR_POS              23 /**< DMA_RX_FIFO_CLEAR Position */
#define MXC_F_SPIXR_DMA_RX_FIFO_CLEAR                  ((uint32_t)(0x1UL << MXC_F_SPIXR_DMA_RX_FIFO_CLEAR_POS)) /**< DMA_RX_FIFO_CLEAR Mask */
#define MXC_V_SPIXR_DMA_RX_FIFO_CLEAR_CLEAR            ((uint32_t)0x1UL) /**< DMA_RX_FIFO_CLEAR_CLEAR Value */
#define MXC_S_SPIXR_DMA_RX_FIFO_CLEAR_CLEAR            (MXC_V_SPIXR_DMA_RX_FIFO_CLEAR_CLEAR << MXC_F_SPIXR_DMA_RX_FIFO_CLEAR_POS) /**< DMA_RX_FIFO_CLEAR_CLEAR Setting */

#define MXC_F_SPIXR_DMA_RX_FIFO_CNT_POS                24 /**< DMA_RX_FIFO_CNT Position */
#define MXC_F_SPIXR_DMA_RX_FIFO_CNT                    ((uint32_t)(0x3FUL << MXC_F_SPIXR_DMA_RX_FIFO_CNT_POS)) /**< DMA_RX_FIFO_CNT Mask */

#define MXC_F_SPIXR_DMA_RX_DMA_EN_POS                  31 /**< DMA_RX_DMA_EN Position */
#define MXC_F_SPIXR_DMA_RX_DMA_EN                      ((uint32_t)(0x1UL << MXC_F_SPIXR_DMA_RX_DMA_EN_POS)) /**< DMA_RX_DMA_EN Mask */
#define MXC_V_SPIXR_DMA_RX_DMA_EN_DIS                  ((uint32_t)0x0UL) /**< DMA_RX_DMA_EN_DIS Value */
#define MXC_S_SPIXR_DMA_RX_DMA_EN_DIS                  (MXC_V_SPIXR_DMA_RX_DMA_EN_DIS << MXC_F_SPIXR_DMA_RX_DMA_EN_POS) /**< DMA_RX_DMA_EN_DIS Setting */
#define MXC_V_SPIXR_DMA_RX_DMA_EN_EN                   ((uint32_t)0x1UL) /**< DMA_RX_DMA_EN_EN Value */
#define MXC_S_SPIXR_DMA_RX_DMA_EN_EN                   (MXC_V_SPIXR_DMA_RX_DMA_EN_EN << MXC_F_SPIXR_DMA_RX_DMA_EN_POS) /**< DMA_RX_DMA_EN_EN Setting */

/**@} end of group SPIXR_DMA_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_INT_FL SPIXR_INT_FL
 * @brief    Register for reading and clearing interrupt flags. All bits are write 1 to
 *           clear.
 * @{
 */
#define MXC_F_SPIXR_INT_FL_TX_LEVEL_POS                0 /**< INT_FL_TX_LEVEL Position */
#define MXC_F_SPIXR_INT_FL_TX_LEVEL                    ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_TX_LEVEL_POS)) /**< INT_FL_TX_LEVEL Mask */
#define MXC_V_SPIXR_INT_FL_TX_LEVEL_CLEAR              ((uint32_t)0x1UL) /**< INT_FL_TX_LEVEL_CLEAR Value */
#define MXC_S_SPIXR_INT_FL_TX_LEVEL_CLEAR              (MXC_V_SPIXR_INT_FL_TX_LEVEL_CLEAR << MXC_F_SPIXR_INT_FL_TX_LEVEL_POS) /**< INT_FL_TX_LEVEL_CLEAR Setting */

#define MXC_F_SPIXR_INT_FL_TX_EMPTY_POS                1 /**< INT_FL_TX_EMPTY Position */
#define MXC_F_SPIXR_INT_FL_TX_EMPTY                    ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_TX_EMPTY_POS)) /**< INT_FL_TX_EMPTY Mask */
#define MXC_V_SPIXR_INT_FL_TX_EMPTY_CLEAR              ((uint32_t)0x1UL) /**< INT_FL_TX_EMPTY_CLEAR Value */
#define MXC_S_SPIXR_INT_FL_TX_EMPTY_CLEAR              (MXC_V_SPIXR_INT_FL_TX_EMPTY_CLEAR << MXC_F_SPIXR_INT_FL_TX_EMPTY_POS) /**< INT_FL_TX_EMPTY_CLEAR Setting */

#define MXC_F_SPIXR_INT_FL_RX_LEVEL_POS                2 /**< INT_FL_RX_LEVEL Position */
#define MXC_F_SPIXR_INT_FL_RX_LEVEL                    ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_RX_LEVEL_POS)) /**< INT_FL_RX_LEVEL Mask */
#define MXC_V_SPIXR_INT_FL_RX_LEVEL_CLEAR              ((uint32_t)0x1UL) /**< INT_FL_RX_LEVEL_CLEAR Value */
#define MXC_S_SPIXR_INT_FL_RX_LEVEL_CLEAR              (MXC_V_SPIXR_INT_FL_RX_LEVEL_CLEAR << MXC_F_SPIXR_INT_FL_RX_LEVEL_POS) /**< INT_FL_RX_LEVEL_CLEAR Setting */

#define MXC_F_SPIXR_INT_FL_RX_FULL_POS                 3 /**< INT_FL_RX_FULL Position */
#define MXC_F_SPIXR_INT_FL_RX_FULL                     ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_RX_FULL_POS)) /**< INT_FL_RX_FULL Mask */
#define MXC_V_SPIXR_INT_FL_RX_FULL_CLEAR               ((uint32_t)0x1UL) /**< INT_FL_RX_FULL_CLEAR Value */
#define MXC_S_SPIXR_INT_FL_RX_FULL_CLEAR               (MXC_V_SPIXR_INT_FL_RX_FULL_CLEAR << MXC_F_SPIXR_INT_FL_RX_FULL_POS) /**< INT_FL_RX_FULL_CLEAR Setting */

#define MXC_F_SPIXR_INT_FL_SSA_POS                     4 /**< INT_FL_SSA Position */
#define MXC_F_SPIXR_INT_FL_SSA                         ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_SSA_POS)) /**< INT_FL_SSA Mask */
#define MXC_V_SPIXR_INT_FL_SSA_CLEAR                   ((uint32_t)0x1UL) /**< INT_FL_SSA_CLEAR Value */
#define MXC_S_SPIXR_INT_FL_SSA_CLEAR                   (MXC_V_SPIXR_INT_FL_SSA_CLEAR << MXC_F_SPIXR_INT_FL_SSA_POS) /**< INT_FL_SSA_CLEAR Setting */

#define MXC_F_SPIXR_INT_FL_SSD_POS                     5 /**< INT_FL_SSD Position */
#define MXC_F_SPIXR_INT_FL_SSD                         ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_SSD_POS)) /**< INT_FL_SSD Mask */
#define MXC_V_SPIXR_INT_FL_SSD_CLEAR                   ((uint32_t)0x1UL) /**< INT_FL_SSD_CLEAR Value */
#define MXC_S_SPIXR_INT_FL_SSD_CLEAR                   (MXC_V_SPIXR_INT_FL_SSD_CLEAR << MXC_F_SPIXR_INT_FL_SSD_POS) /**< INT_FL_SSD_CLEAR Setting */

#define MXC_F_SPIXR_INT_FL_FAULT_POS                   8 /**< INT_FL_FAULT Position */
#define MXC_F_SPIXR_INT_FL_FAULT                       ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_FAULT_POS)) /**< INT_FL_FAULT Mask */
#define MXC_V_SPIXR_INT_FL_FAULT_CLEAR                 ((uint32_t)0x1UL) /**< INT_FL_FAULT_CLEAR Value */
#define MXC_S_SPIXR_INT_FL_FAULT_CLEAR                 (MXC_V_SPIXR_INT_FL_FAULT_CLEAR << MXC_F_SPIXR_INT_FL_FAULT_POS) /**< INT_FL_FAULT_CLEAR Setting */

#define MXC_F_SPIXR_INT_FL_ABORT_POS                   9 /**< INT_FL_ABORT Position */
#define MXC_F_SPIXR_INT_FL_ABORT                       ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_ABORT_POS)) /**< INT_FL_ABORT Mask */
#define MXC_V_SPIXR_INT_FL_ABORT_CLEAR                 ((uint32_t)0x1UL) /**< INT_FL_ABORT_CLEAR Value */
#define MXC_S_SPIXR_INT_FL_ABORT_CLEAR                 (MXC_V_SPIXR_INT_FL_ABORT_CLEAR << MXC_F_SPIXR_INT_FL_ABORT_POS) /**< INT_FL_ABORT_CLEAR Setting */

#define MXC_F_SPIXR_INT_FL_M_DONE_POS                  11 /**< INT_FL_M_DONE Position */
#define MXC_F_SPIXR_INT_FL_M_DONE                      ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_M_DONE_POS)) /**< INT_FL_M_DONE Mask */
#define MXC_V_SPIXR_INT_FL_M_DONE_CLEAR                ((uint32_t)0x1UL) /**< INT_FL_M_DONE_CLEAR Value */
#define MXC_S_SPIXR_INT_FL_M_DONE_CLEAR                (MXC_V_SPIXR_INT_FL_M_DONE_CLEAR << MXC_F_SPIXR_INT_FL_M_DONE_POS) /**< INT_FL_M_DONE_CLEAR Setting */

#define MXC_F_SPIXR_INT_FL_TX_OVR_POS                  12 /**< INT_FL_TX_OVR Position */
#define MXC_F_SPIXR_INT_FL_TX_OVR                      ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_TX_OVR_POS)) /**< INT_FL_TX_OVR Mask */
#define MXC_V_SPIXR_INT_FL_TX_OVR_CLEAR                ((uint32_t)0x1UL) /**< INT_FL_TX_OVR_CLEAR Value */
#define MXC_S_SPIXR_INT_FL_TX_OVR_CLEAR                (MXC_V_SPIXR_INT_FL_TX_OVR_CLEAR << MXC_F_SPIXR_INT_FL_TX_OVR_POS) /**< INT_FL_TX_OVR_CLEAR Setting */

#define MXC_F_SPIXR_INT_FL_TX_UND_POS                  13 /**< INT_FL_TX_UND Position */
#define MXC_F_SPIXR_INT_FL_TX_UND                      ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_TX_UND_POS)) /**< INT_FL_TX_UND Mask */
#define MXC_V_SPIXR_INT_FL_TX_UND_CLEAR                ((uint32_t)0x1UL) /**< INT_FL_TX_UND_CLEAR Value */
#define MXC_S_SPIXR_INT_FL_TX_UND_CLEAR                (MXC_V_SPIXR_INT_FL_TX_UND_CLEAR << MXC_F_SPIXR_INT_FL_TX_UND_POS) /**< INT_FL_TX_UND_CLEAR Setting */

#define MXC_F_SPIXR_INT_FL_RX_OVR_POS                  14 /**< INT_FL_RX_OVR Position */
#define MXC_F_SPIXR_INT_FL_RX_OVR                      ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_RX_OVR_POS)) /**< INT_FL_RX_OVR Mask */
#define MXC_V_SPIXR_INT_FL_RX_OVR_CLEAR                ((uint32_t)0x1UL) /**< INT_FL_RX_OVR_CLEAR Value */
#define MXC_S_SPIXR_INT_FL_RX_OVR_CLEAR                (MXC_V_SPIXR_INT_FL_RX_OVR_CLEAR << MXC_F_SPIXR_INT_FL_RX_OVR_POS) /**< INT_FL_RX_OVR_CLEAR Setting */

#define MXC_F_SPIXR_INT_FL_RX_UND_POS                  15 /**< INT_FL_RX_UND Position */
#define MXC_F_SPIXR_INT_FL_RX_UND                      ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_RX_UND_POS)) /**< INT_FL_RX_UND Mask */
#define MXC_V_SPIXR_INT_FL_RX_UND_CLEAR                ((uint32_t)0x1UL) /**< INT_FL_RX_UND_CLEAR Value */
#define MXC_S_SPIXR_INT_FL_RX_UND_CLEAR                (MXC_V_SPIXR_INT_FL_RX_UND_CLEAR << MXC_F_SPIXR_INT_FL_RX_UND_POS) /**< INT_FL_RX_UND_CLEAR Setting */

/**@} end of group SPIXR_INT_FL_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_INT_EN SPIXR_INT_EN
 * @brief    Register for enabling interrupts.
 * @{
 */
#define MXC_F_SPIXR_INT_EN_TX_LEVEL_POS                0 /**< INT_EN_TX_LEVEL Position */
#define MXC_F_SPIXR_INT_EN_TX_LEVEL                    ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_TX_LEVEL_POS)) /**< INT_EN_TX_LEVEL Mask */
#define MXC_V_SPIXR_INT_EN_TX_LEVEL_DIS                ((uint32_t)0x0UL) /**< INT_EN_TX_LEVEL_DIS Value */
#define MXC_S_SPIXR_INT_EN_TX_LEVEL_DIS                (MXC_V_SPIXR_INT_EN_TX_LEVEL_DIS << MXC_F_SPIXR_INT_EN_TX_LEVEL_POS) /**< INT_EN_TX_LEVEL_DIS Setting */
#define MXC_V_SPIXR_INT_EN_TX_LEVEL_EN                 ((uint32_t)0x1UL) /**< INT_EN_TX_LEVEL_EN Value */
#define MXC_S_SPIXR_INT_EN_TX_LEVEL_EN                 (MXC_V_SPIXR_INT_EN_TX_LEVEL_EN << MXC_F_SPIXR_INT_EN_TX_LEVEL_POS) /**< INT_EN_TX_LEVEL_EN Setting */

#define MXC_F_SPIXR_INT_EN_TX_EMPTY_POS                1 /**< INT_EN_TX_EMPTY Position */
#define MXC_F_SPIXR_INT_EN_TX_EMPTY                    ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_TX_EMPTY_POS)) /**< INT_EN_TX_EMPTY Mask */
#define MXC_V_SPIXR_INT_EN_TX_EMPTY_DIS                ((uint32_t)0x0UL) /**< INT_EN_TX_EMPTY_DIS Value */
#define MXC_S_SPIXR_INT_EN_TX_EMPTY_DIS                (MXC_V_SPIXR_INT_EN_TX_EMPTY_DIS << MXC_F_SPIXR_INT_EN_TX_EMPTY_POS) /**< INT_EN_TX_EMPTY_DIS Setting */
#define MXC_V_SPIXR_INT_EN_TX_EMPTY_EN                 ((uint32_t)0x1UL) /**< INT_EN_TX_EMPTY_EN Value */
#define MXC_S_SPIXR_INT_EN_TX_EMPTY_EN                 (MXC_V_SPIXR_INT_EN_TX_EMPTY_EN << MXC_F_SPIXR_INT_EN_TX_EMPTY_POS) /**< INT_EN_TX_EMPTY_EN Setting */

#define MXC_F_SPIXR_INT_EN_RX_LEVEL_POS                2 /**< INT_EN_RX_LEVEL Position */
#define MXC_F_SPIXR_INT_EN_RX_LEVEL                    ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_RX_LEVEL_POS)) /**< INT_EN_RX_LEVEL Mask */
#define MXC_V_SPIXR_INT_EN_RX_LEVEL_DIS                ((uint32_t)0x0UL) /**< INT_EN_RX_LEVEL_DIS Value */
#define MXC_S_SPIXR_INT_EN_RX_LEVEL_DIS                (MXC_V_SPIXR_INT_EN_RX_LEVEL_DIS << MXC_F_SPIXR_INT_EN_RX_LEVEL_POS) /**< INT_EN_RX_LEVEL_DIS Setting */
#define MXC_V_SPIXR_INT_EN_RX_LEVEL_EN                 ((uint32_t)0x1UL) /**< INT_EN_RX_LEVEL_EN Value */
#define MXC_S_SPIXR_INT_EN_RX_LEVEL_EN                 (MXC_V_SPIXR_INT_EN_RX_LEVEL_EN << MXC_F_SPIXR_INT_EN_RX_LEVEL_POS) /**< INT_EN_RX_LEVEL_EN Setting */

#define MXC_F_SPIXR_INT_EN_RX_FULL_POS                 3 /**< INT_EN_RX_FULL Position */
#define MXC_F_SPIXR_INT_EN_RX_FULL                     ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_RX_FULL_POS)) /**< INT_EN_RX_FULL Mask */
#define MXC_V_SPIXR_INT_EN_RX_FULL_DIS                 ((uint32_t)0x0UL) /**< INT_EN_RX_FULL_DIS Value */
#define MXC_S_SPIXR_INT_EN_RX_FULL_DIS                 (MXC_V_SPIXR_INT_EN_RX_FULL_DIS << MXC_F_SPIXR_INT_EN_RX_FULL_POS) /**< INT_EN_RX_FULL_DIS Setting */
#define MXC_V_SPIXR_INT_EN_RX_FULL_EN                  ((uint32_t)0x1UL) /**< INT_EN_RX_FULL_EN Value */
#define MXC_S_SPIXR_INT_EN_RX_FULL_EN                  (MXC_V_SPIXR_INT_EN_RX_FULL_EN << MXC_F_SPIXR_INT_EN_RX_FULL_POS) /**< INT_EN_RX_FULL_EN Setting */

#define MXC_F_SPIXR_INT_EN_SSA_POS                     4 /**< INT_EN_SSA Position */
#define MXC_F_SPIXR_INT_EN_SSA                         ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_SSA_POS)) /**< INT_EN_SSA Mask */
#define MXC_V_SPIXR_INT_EN_SSA_DIS                     ((uint32_t)0x0UL) /**< INT_EN_SSA_DIS Value */
#define MXC_S_SPIXR_INT_EN_SSA_DIS                     (MXC_V_SPIXR_INT_EN_SSA_DIS << MXC_F_SPIXR_INT_EN_SSA_POS) /**< INT_EN_SSA_DIS Setting */
#define MXC_V_SPIXR_INT_EN_SSA_EN                      ((uint32_t)0x1UL) /**< INT_EN_SSA_EN Value */
#define MXC_S_SPIXR_INT_EN_SSA_EN                      (MXC_V_SPIXR_INT_EN_SSA_EN << MXC_F_SPIXR_INT_EN_SSA_POS) /**< INT_EN_SSA_EN Setting */

#define MXC_F_SPIXR_INT_EN_SSD_POS                     5 /**< INT_EN_SSD Position */
#define MXC_F_SPIXR_INT_EN_SSD                         ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_SSD_POS)) /**< INT_EN_SSD Mask */
#define MXC_V_SPIXR_INT_EN_SSD_DIS                     ((uint32_t)0x0UL) /**< INT_EN_SSD_DIS Value */
#define MXC_S_SPIXR_INT_EN_SSD_DIS                     (MXC_V_SPIXR_INT_EN_SSD_DIS << MXC_F_SPIXR_INT_EN_SSD_POS) /**< INT_EN_SSD_DIS Setting */
#define MXC_V_SPIXR_INT_EN_SSD_EN                      ((uint32_t)0x1UL) /**< INT_EN_SSD_EN Value */
#define MXC_S_SPIXR_INT_EN_SSD_EN                      (MXC_V_SPIXR_INT_EN_SSD_EN << MXC_F_SPIXR_INT_EN_SSD_POS) /**< INT_EN_SSD_EN Setting */

#define MXC_F_SPIXR_INT_EN_FAULT_POS                   8 /**< INT_EN_FAULT Position */
#define MXC_F_SPIXR_INT_EN_FAULT                       ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_FAULT_POS)) /**< INT_EN_FAULT Mask */
#define MXC_V_SPIXR_INT_EN_FAULT_DIS                   ((uint32_t)0x0UL) /**< INT_EN_FAULT_DIS Value */
#define MXC_S_SPIXR_INT_EN_FAULT_DIS                   (MXC_V_SPIXR_INT_EN_FAULT_DIS << MXC_F_SPIXR_INT_EN_FAULT_POS) /**< INT_EN_FAULT_DIS Setting */
#define MXC_V_SPIXR_INT_EN_FAULT_EN                    ((uint32_t)0x1UL) /**< INT_EN_FAULT_EN Value */
#define MXC_S_SPIXR_INT_EN_FAULT_EN                    (MXC_V_SPIXR_INT_EN_FAULT_EN << MXC_F_SPIXR_INT_EN_FAULT_POS) /**< INT_EN_FAULT_EN Setting */

#define MXC_F_SPIXR_INT_EN_ABORT_POS                   9 /**< INT_EN_ABORT Position */
#define MXC_F_SPIXR_INT_EN_ABORT                       ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_ABORT_POS)) /**< INT_EN_ABORT Mask */
#define MXC_V_SPIXR_INT_EN_ABORT_DIS                   ((uint32_t)0x0UL) /**< INT_EN_ABORT_DIS Value */
#define MXC_S_SPIXR_INT_EN_ABORT_DIS                   (MXC_V_SPIXR_INT_EN_ABORT_DIS << MXC_F_SPIXR_INT_EN_ABORT_POS) /**< INT_EN_ABORT_DIS Setting */
#define MXC_V_SPIXR_INT_EN_ABORT_EN                    ((uint32_t)0x1UL) /**< INT_EN_ABORT_EN Value */
#define MXC_S_SPIXR_INT_EN_ABORT_EN                    (MXC_V_SPIXR_INT_EN_ABORT_EN << MXC_F_SPIXR_INT_EN_ABORT_POS) /**< INT_EN_ABORT_EN Setting */

#define MXC_F_SPIXR_INT_EN_M_DONE_POS                  11 /**< INT_EN_M_DONE Position */
#define MXC_F_SPIXR_INT_EN_M_DONE                      ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_M_DONE_POS)) /**< INT_EN_M_DONE Mask */
#define MXC_V_SPIXR_INT_EN_M_DONE_DIS                  ((uint32_t)0x0UL) /**< INT_EN_M_DONE_DIS Value */
#define MXC_S_SPIXR_INT_EN_M_DONE_DIS                  (MXC_V_SPIXR_INT_EN_M_DONE_DIS << MXC_F_SPIXR_INT_EN_M_DONE_POS) /**< INT_EN_M_DONE_DIS Setting */
#define MXC_V_SPIXR_INT_EN_M_DONE_EN                   ((uint32_t)0x1UL) /**< INT_EN_M_DONE_EN Value */
#define MXC_S_SPIXR_INT_EN_M_DONE_EN                   (MXC_V_SPIXR_INT_EN_M_DONE_EN << MXC_F_SPIXR_INT_EN_M_DONE_POS) /**< INT_EN_M_DONE_EN Setting */

#define MXC_F_SPIXR_INT_EN_TX_OVR_POS                  12 /**< INT_EN_TX_OVR Position */
#define MXC_F_SPIXR_INT_EN_TX_OVR                      ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_TX_OVR_POS)) /**< INT_EN_TX_OVR Mask */
#define MXC_V_SPIXR_INT_EN_TX_OVR_DIS                  ((uint32_t)0x0UL) /**< INT_EN_TX_OVR_DIS Value */
#define MXC_S_SPIXR_INT_EN_TX_OVR_DIS                  (MXC_V_SPIXR_INT_EN_TX_OVR_DIS << MXC_F_SPIXR_INT_EN_TX_OVR_POS) /**< INT_EN_TX_OVR_DIS Setting */
#define MXC_V_SPIXR_INT_EN_TX_OVR_EN                   ((uint32_t)0x1UL) /**< INT_EN_TX_OVR_EN Value */
#define MXC_S_SPIXR_INT_EN_TX_OVR_EN                   (MXC_V_SPIXR_INT_EN_TX_OVR_EN << MXC_F_SPIXR_INT_EN_TX_OVR_POS) /**< INT_EN_TX_OVR_EN Setting */

#define MXC_F_SPIXR_INT_EN_TX_UND_POS                  13 /**< INT_EN_TX_UND Position */
#define MXC_F_SPIXR_INT_EN_TX_UND                      ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_TX_UND_POS)) /**< INT_EN_TX_UND Mask */
#define MXC_V_SPIXR_INT_EN_TX_UND_DIS                  ((uint32_t)0x0UL) /**< INT_EN_TX_UND_DIS Value */
#define MXC_S_SPIXR_INT_EN_TX_UND_DIS                  (MXC_V_SPIXR_INT_EN_TX_UND_DIS << MXC_F_SPIXR_INT_EN_TX_UND_POS) /**< INT_EN_TX_UND_DIS Setting */
#define MXC_V_SPIXR_INT_EN_TX_UND_EN                   ((uint32_t)0x1UL) /**< INT_EN_TX_UND_EN Value */
#define MXC_S_SPIXR_INT_EN_TX_UND_EN                   (MXC_V_SPIXR_INT_EN_TX_UND_EN << MXC_F_SPIXR_INT_EN_TX_UND_POS) /**< INT_EN_TX_UND_EN Setting */

#define MXC_F_SPIXR_INT_EN_RX_OVR_POS                  14 /**< INT_EN_RX_OVR Position */
#define MXC_F_SPIXR_INT_EN_RX_OVR                      ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_RX_OVR_POS)) /**< INT_EN_RX_OVR Mask */
#define MXC_V_SPIXR_INT_EN_RX_OVR_DIS                  ((uint32_t)0x0UL) /**< INT_EN_RX_OVR_DIS Value */
#define MXC_S_SPIXR_INT_EN_RX_OVR_DIS                  (MXC_V_SPIXR_INT_EN_RX_OVR_DIS << MXC_F_SPIXR_INT_EN_RX_OVR_POS) /**< INT_EN_RX_OVR_DIS Setting */
#define MXC_V_SPIXR_INT_EN_RX_OVR_EN                   ((uint32_t)0x1UL) /**< INT_EN_RX_OVR_EN Value */
#define MXC_S_SPIXR_INT_EN_RX_OVR_EN                   (MXC_V_SPIXR_INT_EN_RX_OVR_EN << MXC_F_SPIXR_INT_EN_RX_OVR_POS) /**< INT_EN_RX_OVR_EN Setting */

#define MXC_F_SPIXR_INT_EN_RX_UND_POS                  15 /**< INT_EN_RX_UND Position */
#define MXC_F_SPIXR_INT_EN_RX_UND                      ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_RX_UND_POS)) /**< INT_EN_RX_UND Mask */
#define MXC_V_SPIXR_INT_EN_RX_UND_DIS                  ((uint32_t)0x0UL) /**< INT_EN_RX_UND_DIS Value */
#define MXC_S_SPIXR_INT_EN_RX_UND_DIS                  (MXC_V_SPIXR_INT_EN_RX_UND_DIS << MXC_F_SPIXR_INT_EN_RX_UND_POS) /**< INT_EN_RX_UND_DIS Setting */
#define MXC_V_SPIXR_INT_EN_RX_UND_EN                   ((uint32_t)0x1UL) /**< INT_EN_RX_UND_EN Value */
#define MXC_S_SPIXR_INT_EN_RX_UND_EN                   (MXC_V_SPIXR_INT_EN_RX_UND_EN << MXC_F_SPIXR_INT_EN_RX_UND_POS) /**< INT_EN_RX_UND_EN Setting */

/**@} end of group SPIXR_INT_EN_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_WAKE_FL SPIXR_WAKE_FL
 * @brief    Register for wake up flags. All bits in this register are write 1 to clear.
 * @{
 */
#define MXC_F_SPIXR_WAKE_FL_TX_LEVEL_POS               0 /**< WAKE_FL_TX_LEVEL Position */
#define MXC_F_SPIXR_WAKE_FL_TX_LEVEL                   ((uint32_t)(0x1UL << MXC_F_SPIXR_WAKE_FL_TX_LEVEL_POS)) /**< WAKE_FL_TX_LEVEL Mask */
#define MXC_V_SPIXR_WAKE_FL_TX_LEVEL_CLEAR             ((uint32_t)0x1UL) /**< WAKE_FL_TX_LEVEL_CLEAR Value */
#define MXC_S_SPIXR_WAKE_FL_TX_LEVEL_CLEAR             (MXC_V_SPIXR_WAKE_FL_TX_LEVEL_CLEAR << MXC_F_SPIXR_WAKE_FL_TX_LEVEL_POS) /**< WAKE_FL_TX_LEVEL_CLEAR Setting */

#define MXC_F_SPIXR_WAKE_FL_TX_EMPTY_POS               1 /**< WAKE_FL_TX_EMPTY Position */
#define MXC_F_SPIXR_WAKE_FL_TX_EMPTY                   ((uint32_t)(0x1UL << MXC_F_SPIXR_WAKE_FL_TX_EMPTY_POS)) /**< WAKE_FL_TX_EMPTY Mask */
#define MXC_V_SPIXR_WAKE_FL_TX_EMPTY_CLEAR             ((uint32_t)0x1UL) /**< WAKE_FL_TX_EMPTY_CLEAR Value */
#define MXC_S_SPIXR_WAKE_FL_TX_EMPTY_CLEAR             (MXC_V_SPIXR_WAKE_FL_TX_EMPTY_CLEAR << MXC_F_SPIXR_WAKE_FL_TX_EMPTY_POS) /**< WAKE_FL_TX_EMPTY_CLEAR Setting */

#define MXC_F_SPIXR_WAKE_FL_RX_LEVEL_POS               2 /**< WAKE_FL_RX_LEVEL Position */
#define MXC_F_SPIXR_WAKE_FL_RX_LEVEL                   ((uint32_t)(0x1UL << MXC_F_SPIXR_WAKE_FL_RX_LEVEL_POS)) /**< WAKE_FL_RX_LEVEL Mask */
#define MXC_V_SPIXR_WAKE_FL_RX_LEVEL_CLEAR             ((uint32_t)0x1UL) /**< WAKE_FL_RX_LEVEL_CLEAR Value */
#define MXC_S_SPIXR_WAKE_FL_RX_LEVEL_CLEAR             (MXC_V_SPIXR_WAKE_FL_RX_LEVEL_CLEAR << MXC_F_SPIXR_WAKE_FL_RX_LEVEL_POS) /**< WAKE_FL_RX_LEVEL_CLEAR Setting */

#define MXC_F_SPIXR_WAKE_FL_RX_FULL_POS                3 /**< WAKE_FL_RX_FULL Position */
#define MXC_F_SPIXR_WAKE_FL_RX_FULL                    ((uint32_t)(0x1UL << MXC_F_SPIXR_WAKE_FL_RX_FULL_POS)) /**< WAKE_FL_RX_FULL Mask */
#define MXC_V_SPIXR_WAKE_FL_RX_FULL_CLEAR              ((uint32_t)0x1UL) /**< WAKE_FL_RX_FULL_CLEAR Value */
#define MXC_S_SPIXR_WAKE_FL_RX_FULL_CLEAR              (MXC_V_SPIXR_WAKE_FL_RX_FULL_CLEAR << MXC_F_SPIXR_WAKE_FL_RX_FULL_POS) /**< WAKE_FL_RX_FULL_CLEAR Setting */

/**@} end of group SPIXR_WAKE_FL_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_WAKE_EN SPIXR_WAKE_EN
 * @brief    Register for wake up enable.
 * @{
 */
#define MXC_F_SPIXR_WAKE_EN_TX_LEVEL_POS               0 /**< WAKE_EN_TX_LEVEL Position */
#define MXC_F_SPIXR_WAKE_EN_TX_LEVEL                   ((uint32_t)(0x1UL << MXC_F_SPIXR_WAKE_EN_TX_LEVEL_POS)) /**< WAKE_EN_TX_LEVEL Mask */
#define MXC_V_SPIXR_WAKE_EN_TX_LEVEL_DIS               ((uint32_t)0x0UL) /**< WAKE_EN_TX_LEVEL_DIS Value */
#define MXC_S_SPIXR_WAKE_EN_TX_LEVEL_DIS               (MXC_V_SPIXR_WAKE_EN_TX_LEVEL_DIS << MXC_F_SPIXR_WAKE_EN_TX_LEVEL_POS) /**< WAKE_EN_TX_LEVEL_DIS Setting */
#define MXC_V_SPIXR_WAKE_EN_TX_LEVEL_EN                ((uint32_t)0x1UL) /**< WAKE_EN_TX_LEVEL_EN Value */
#define MXC_S_SPIXR_WAKE_EN_TX_LEVEL_EN                (MXC_V_SPIXR_WAKE_EN_TX_LEVEL_EN << MXC_F_SPIXR_WAKE_EN_TX_LEVEL_POS) /**< WAKE_EN_TX_LEVEL_EN Setting */

#define MXC_F_SPIXR_WAKE_EN_TX_EMPTY_POS               1 /**< WAKE_EN_TX_EMPTY Position */
#define MXC_F_SPIXR_WAKE_EN_TX_EMPTY                   ((uint32_t)(0x1UL << MXC_F_SPIXR_WAKE_EN_TX_EMPTY_POS)) /**< WAKE_EN_TX_EMPTY Mask */
#define MXC_V_SPIXR_WAKE_EN_TX_EMPTY_DIS               ((uint32_t)0x0UL) /**< WAKE_EN_TX_EMPTY_DIS Value */
#define MXC_S_SPIXR_WAKE_EN_TX_EMPTY_DIS               (MXC_V_SPIXR_WAKE_EN_TX_EMPTY_DIS << MXC_F_SPIXR_WAKE_EN_TX_EMPTY_POS) /**< WAKE_EN_TX_EMPTY_DIS Setting */
#define MXC_V_SPIXR_WAKE_EN_TX_EMPTY_EN                ((uint32_t)0x1UL) /**< WAKE_EN_TX_EMPTY_EN Value */
#define MXC_S_SPIXR_WAKE_EN_TX_EMPTY_EN                (MXC_V_SPIXR_WAKE_EN_TX_EMPTY_EN << MXC_F_SPIXR_WAKE_EN_TX_EMPTY_POS) /**< WAKE_EN_TX_EMPTY_EN Setting */

#define MXC_F_SPIXR_WAKE_EN_RX_LEVEL_POS               2 /**< WAKE_EN_RX_LEVEL Position */
#define MXC_F_SPIXR_WAKE_EN_RX_LEVEL                   ((uint32_t)(0x1UL << MXC_F_SPIXR_WAKE_EN_RX_LEVEL_POS)) /**< WAKE_EN_RX_LEVEL Mask */
#define MXC_V_SPIXR_WAKE_EN_RX_LEVEL_DIS               ((uint32_t)0x0UL) /**< WAKE_EN_RX_LEVEL_DIS Value */
#define MXC_S_SPIXR_WAKE_EN_RX_LEVEL_DIS               (MXC_V_SPIXR_WAKE_EN_RX_LEVEL_DIS << MXC_F_SPIXR_WAKE_EN_RX_LEVEL_POS) /**< WAKE_EN_RX_LEVEL_DIS Setting */
#define MXC_V_SPIXR_WAKE_EN_RX_LEVEL_EN                ((uint32_t)0x1UL) /**< WAKE_EN_RX_LEVEL_EN Value */
#define MXC_S_SPIXR_WAKE_EN_RX_LEVEL_EN                (MXC_V_SPIXR_WAKE_EN_RX_LEVEL_EN << MXC_F_SPIXR_WAKE_EN_RX_LEVEL_POS) /**< WAKE_EN_RX_LEVEL_EN Setting */

#define MXC_F_SPIXR_WAKE_EN_RX_FULL_POS                3 /**< WAKE_EN_RX_FULL Position */
#define MXC_F_SPIXR_WAKE_EN_RX_FULL                    ((uint32_t)(0x1UL << MXC_F_SPIXR_WAKE_EN_RX_FULL_POS)) /**< WAKE_EN_RX_FULL Mask */
#define MXC_V_SPIXR_WAKE_EN_RX_FULL_DIS                ((uint32_t)0x0UL) /**< WAKE_EN_RX_FULL_DIS Value */
#define MXC_S_SPIXR_WAKE_EN_RX_FULL_DIS                (MXC_V_SPIXR_WAKE_EN_RX_FULL_DIS << MXC_F_SPIXR_WAKE_EN_RX_FULL_POS) /**< WAKE_EN_RX_FULL_DIS Setting */
#define MXC_V_SPIXR_WAKE_EN_RX_FULL_EN                 ((uint32_t)0x1UL) /**< WAKE_EN_RX_FULL_EN Value */
#define MXC_S_SPIXR_WAKE_EN_RX_FULL_EN                 (MXC_V_SPIXR_WAKE_EN_RX_FULL_EN << MXC_F_SPIXR_WAKE_EN_RX_FULL_POS) /**< WAKE_EN_RX_FULL_EN Setting */

/**@} end of group SPIXR_WAKE_EN_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_STAT SPIXR_STAT
 * @brief    SPI Status register.
 * @{
 */
#define MXC_F_SPIXR_STAT_BUSY_POS                      0 /**< STAT_BUSY Position */
#define MXC_F_SPIXR_STAT_BUSY                          ((uint32_t)(0x1UL << MXC_F_SPIXR_STAT_BUSY_POS)) /**< STAT_BUSY Mask */
#define MXC_V_SPIXR_STAT_BUSY_NOTACTIVE                ((uint32_t)0x0UL) /**< STAT_BUSY_NOTACTIVE Value */
#define MXC_S_SPIXR_STAT_BUSY_NOTACTIVE                (MXC_V_SPIXR_STAT_BUSY_NOTACTIVE << MXC_F_SPIXR_STAT_BUSY_POS) /**< STAT_BUSY_NOTACTIVE Setting */
#define MXC_V_SPIXR_STAT_BUSY_ACTIVE                   ((uint32_t)0x1UL) /**< STAT_BUSY_ACTIVE Value */
#define MXC_S_SPIXR_STAT_BUSY_ACTIVE                   (MXC_V_SPIXR_STAT_BUSY_ACTIVE << MXC_F_SPIXR_STAT_BUSY_POS) /**< STAT_BUSY_ACTIVE Setting */

/**@} end of group SPIXR_STAT_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_XMEM_CTRL SPIXR_XMEM_CTRL
 * @brief    Register to control external memory.
 * @{
 */
#define MXC_F_SPIXR_XMEM_CTRL_XMEM_RD_CMD_POS          0 /**< XMEM_CTRL_XMEM_RD_CMD Position */
#define MXC_F_SPIXR_XMEM_CTRL_XMEM_RD_CMD              ((uint32_t)(0xFFUL << MXC_F_SPIXR_XMEM_CTRL_XMEM_RD_CMD_POS)) /**< XMEM_CTRL_XMEM_RD_CMD Mask */

#define MXC_F_SPIXR_XMEM_CTRL_XMEM_WR_CMD_POS          8 /**< XMEM_CTRL_XMEM_WR_CMD Position */
#define MXC_F_SPIXR_XMEM_CTRL_XMEM_WR_CMD              ((uint32_t)(0xFFUL << MXC_F_SPIXR_XMEM_CTRL_XMEM_WR_CMD_POS)) /**< XMEM_CTRL_XMEM_WR_CMD Mask */

#define MXC_F_SPIXR_XMEM_CTRL_XMEM_DCLKS_POS           16 /**< XMEM_CTRL_XMEM_DCLKS Position */
#define MXC_F_SPIXR_XMEM_CTRL_XMEM_DCLKS               ((uint32_t)(0xFFUL << MXC_F_SPIXR_XMEM_CTRL_XMEM_DCLKS_POS)) /**< XMEM_CTRL_XMEM_DCLKS Mask */

#define MXC_F_SPIXR_XMEM_CTRL_XMEM_EN_POS              31 /**< XMEM_CTRL_XMEM_EN Position */
#define MXC_F_SPIXR_XMEM_CTRL_XMEM_EN                  ((uint32_t)(0x1UL << MXC_F_SPIXR_XMEM_CTRL_XMEM_EN_POS)) /**< XMEM_CTRL_XMEM_EN Mask */
#define MXC_V_SPIXR_XMEM_CTRL_XMEM_EN_DIS              ((uint32_t)0x0UL) /**< XMEM_CTRL_XMEM_EN_DIS Value */
#define MXC_S_SPIXR_XMEM_CTRL_XMEM_EN_DIS              (MXC_V_SPIXR_XMEM_CTRL_XMEM_EN_DIS << MXC_F_SPIXR_XMEM_CTRL_XMEM_EN_POS) /**< XMEM_CTRL_XMEM_EN_DIS Setting */
#define MXC_V_SPIXR_XMEM_CTRL_XMEM_EN_EN               ((uint32_t)0x1UL) /**< XMEM_CTRL_XMEM_EN_EN Value */
#define MXC_S_SPIXR_XMEM_CTRL_XMEM_EN_EN               (MXC_V_SPIXR_XMEM_CTRL_XMEM_EN_EN << MXC_F_SPIXR_XMEM_CTRL_XMEM_EN_POS) /**< XMEM_CTRL_XMEM_EN_EN Setting */

/**@} end of group SPIXR_XMEM_CTRL_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_SPIXR_REGS_H_
