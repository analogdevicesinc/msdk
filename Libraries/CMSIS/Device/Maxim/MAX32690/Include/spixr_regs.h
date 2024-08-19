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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_SPIXR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_SPIXR_REGS_H_

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
    __IO uint32_t ctrl0;                /**< <tt>\b 0x04:</tt> SPIXR CTRL0 Register */
    __IO uint32_t ctrl1;                /**< <tt>\b 0x08:</tt> SPIXR CTRL1 Register */
    __IO uint32_t ctrl2;                /**< <tt>\b 0x0C:</tt> SPIXR CTRL2 Register */
    __IO uint32_t ctrl3;                /**< <tt>\b 0x10:</tt> SPIXR CTRL3 Register */
    __IO uint32_t brgctrl;              /**< <tt>\b 0x14:</tt> SPIXR BRGCTRL Register */
    __R  uint32_t rsv_0x18;
    __IO uint32_t dma;                  /**< <tt>\b 0x1C:</tt> SPIXR DMA Register */
    __IO uint32_t intfl;                /**< <tt>\b 0x20:</tt> SPIXR INTFL Register */
    __IO uint32_t inten;                /**< <tt>\b 0x24:</tt> SPIXR INTEN Register */
    __IO uint32_t wkfl;                 /**< <tt>\b 0x28:</tt> SPIXR WKFL Register */
    __IO uint32_t wken;                 /**< <tt>\b 0x2C:</tt> SPIXR WKEN Register */
    __I  uint32_t stat;                 /**< <tt>\b 0x30:</tt> SPIXR STAT Register */
    __IO uint32_t xmemctrl;             /**< <tt>\b 0x34:</tt> SPIXR XMEMCTRL Register */
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
#define MXC_R_SPIXR_CTRL0                  ((uint32_t)0x00000004UL) /**< Offset from SPIXR Base Address: <tt> 0x0004</tt> */
#define MXC_R_SPIXR_CTRL1                  ((uint32_t)0x00000008UL) /**< Offset from SPIXR Base Address: <tt> 0x0008</tt> */
#define MXC_R_SPIXR_CTRL2                  ((uint32_t)0x0000000CUL) /**< Offset from SPIXR Base Address: <tt> 0x000C</tt> */
#define MXC_R_SPIXR_CTRL3                  ((uint32_t)0x00000010UL) /**< Offset from SPIXR Base Address: <tt> 0x0010</tt> */
#define MXC_R_SPIXR_BRGCTRL                ((uint32_t)0x00000014UL) /**< Offset from SPIXR Base Address: <tt> 0x0014</tt> */
#define MXC_R_SPIXR_DMA                    ((uint32_t)0x0000001CUL) /**< Offset from SPIXR Base Address: <tt> 0x001C</tt> */
#define MXC_R_SPIXR_INTFL                  ((uint32_t)0x00000020UL) /**< Offset from SPIXR Base Address: <tt> 0x0020</tt> */
#define MXC_R_SPIXR_INTEN                  ((uint32_t)0x00000024UL) /**< Offset from SPIXR Base Address: <tt> 0x0024</tt> */
#define MXC_R_SPIXR_WKFL                   ((uint32_t)0x00000028UL) /**< Offset from SPIXR Base Address: <tt> 0x0028</tt> */
#define MXC_R_SPIXR_WKEN                   ((uint32_t)0x0000002CUL) /**< Offset from SPIXR Base Address: <tt> 0x002C</tt> */
#define MXC_R_SPIXR_STAT                   ((uint32_t)0x00000030UL) /**< Offset from SPIXR Base Address: <tt> 0x0030</tt> */
#define MXC_R_SPIXR_XMEMCTRL               ((uint32_t)0x00000034UL) /**< Offset from SPIXR Base Address: <tt> 0x0034</tt> */
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
 * @defgroup SPIXR_CTRL0 SPIXR_CTRL0
 * @brief    Register for controlling SPI peripheral.
 * @{
 */
#define MXC_F_SPIXR_CTRL0_EN_POS                       0 /**< CTRL0_EN Position */
#define MXC_F_SPIXR_CTRL0_EN                           ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL0_EN_POS)) /**< CTRL0_EN Mask */

#define MXC_F_SPIXR_CTRL0_MSTR_EN_POS                  1 /**< CTRL0_MSTR_EN Position */
#define MXC_F_SPIXR_CTRL0_MSTR_EN                      ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL0_MSTR_EN_POS)) /**< CTRL0_MSTR_EN Mask */

#define MXC_F_SPIXR_CTRL0_SSIO_POS                     4 /**< CTRL0_SSIO Position */
#define MXC_F_SPIXR_CTRL0_SSIO                         ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL0_SSIO_POS)) /**< CTRL0_SSIO Mask */

#define MXC_F_SPIXR_CTRL0_TX_START_POS                 5 /**< CTRL0_TX_START Position */
#define MXC_F_SPIXR_CTRL0_TX_START                     ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL0_TX_START_POS)) /**< CTRL0_TX_START Mask */

#define MXC_F_SPIXR_CTRL0_SS_CTRL_POS                  8 /**< CTRL0_SS_CTRL Position */
#define MXC_F_SPIXR_CTRL0_SS_CTRL                      ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL0_SS_CTRL_POS)) /**< CTRL0_SS_CTRL Mask */

#define MXC_F_SPIXR_CTRL0_SS_POS                       16 /**< CTRL0_SS Position */
#define MXC_F_SPIXR_CTRL0_SS                           ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL0_SS_POS)) /**< CTRL0_SS Mask */

/**@} end of group SPIXR_CTRL0_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_CTRL1 SPIXR_CTRL1
 * @brief    Register for controlling SPI peripheral.
 * @{
 */
#define MXC_F_SPIXR_CTRL1_TX_NUM_CHAR_POS              0 /**< CTRL1_TX_NUM_CHAR Position */
#define MXC_F_SPIXR_CTRL1_TX_NUM_CHAR                  ((uint32_t)(0xFFFFUL << MXC_F_SPIXR_CTRL1_TX_NUM_CHAR_POS)) /**< CTRL1_TX_NUM_CHAR Mask */

#define MXC_F_SPIXR_CTRL1_RX_NUM_CHAR_POS              16 /**< CTRL1_RX_NUM_CHAR Position */
#define MXC_F_SPIXR_CTRL1_RX_NUM_CHAR                  ((uint32_t)(0xFFFFUL << MXC_F_SPIXR_CTRL1_RX_NUM_CHAR_POS)) /**< CTRL1_RX_NUM_CHAR Mask */

/**@} end of group SPIXR_CTRL1_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_CTRL2 SPIXR_CTRL2
 * @brief    Register for controlling SPI peripheral.
 * @{
 */
#define MXC_F_SPIXR_CTRL2_CPHA_POS                     0 /**< CTRL2_CPHA Position */
#define MXC_F_SPIXR_CTRL2_CPHA                         ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL2_CPHA_POS)) /**< CTRL2_CPHA Mask */

#define MXC_F_SPIXR_CTRL2_CPOL_POS                     1 /**< CTRL2_CPOL Position */
#define MXC_F_SPIXR_CTRL2_CPOL                         ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL2_CPOL_POS)) /**< CTRL2_CPOL Mask */

#define MXC_F_SPIXR_CTRL2_SCLK_FB_INV_POS              4 /**< CTRL2_SCLK_FB_INV Position */
#define MXC_F_SPIXR_CTRL2_SCLK_FB_INV                  ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL2_SCLK_FB_INV_POS)) /**< CTRL2_SCLK_FB_INV Mask */

#define MXC_F_SPIXR_CTRL2_NUMBITS_POS                  8 /**< CTRL2_NUMBITS Position */
#define MXC_F_SPIXR_CTRL2_NUMBITS                      ((uint32_t)(0xFUL << MXC_F_SPIXR_CTRL2_NUMBITS_POS)) /**< CTRL2_NUMBITS Mask */
#define MXC_V_SPIXR_CTRL2_NUMBITS_0                    ((uint32_t)0x0UL) /**< CTRL2_NUMBITS_0 Value */
#define MXC_S_SPIXR_CTRL2_NUMBITS_0                    (MXC_V_SPIXR_CTRL2_NUMBITS_0 << MXC_F_SPIXR_CTRL2_NUMBITS_POS) /**< CTRL2_NUMBITS_0 Setting */

#define MXC_F_SPIXR_CTRL2_DATA_WIDTH_POS               12 /**< CTRL2_DATA_WIDTH Position */
#define MXC_F_SPIXR_CTRL2_DATA_WIDTH                   ((uint32_t)(0x3UL << MXC_F_SPIXR_CTRL2_DATA_WIDTH_POS)) /**< CTRL2_DATA_WIDTH Mask */
#define MXC_V_SPIXR_CTRL2_DATA_WIDTH_MONO              ((uint32_t)0x0UL) /**< CTRL2_DATA_WIDTH_MONO Value */
#define MXC_S_SPIXR_CTRL2_DATA_WIDTH_MONO              (MXC_V_SPIXR_CTRL2_DATA_WIDTH_MONO << MXC_F_SPIXR_CTRL2_DATA_WIDTH_POS) /**< CTRL2_DATA_WIDTH_MONO Setting */
#define MXC_V_SPIXR_CTRL2_DATA_WIDTH_DUAL              ((uint32_t)0x1UL) /**< CTRL2_DATA_WIDTH_DUAL Value */
#define MXC_S_SPIXR_CTRL2_DATA_WIDTH_DUAL              (MXC_V_SPIXR_CTRL2_DATA_WIDTH_DUAL << MXC_F_SPIXR_CTRL2_DATA_WIDTH_POS) /**< CTRL2_DATA_WIDTH_DUAL Setting */
#define MXC_V_SPIXR_CTRL2_DATA_WIDTH_QUAD              ((uint32_t)0x2UL) /**< CTRL2_DATA_WIDTH_QUAD Value */
#define MXC_S_SPIXR_CTRL2_DATA_WIDTH_QUAD              (MXC_V_SPIXR_CTRL2_DATA_WIDTH_QUAD << MXC_F_SPIXR_CTRL2_DATA_WIDTH_POS) /**< CTRL2_DATA_WIDTH_QUAD Setting */

#define MXC_F_SPIXR_CTRL2_THREE_WIRE_POS               15 /**< CTRL2_THREE_WIRE Position */
#define MXC_F_SPIXR_CTRL2_THREE_WIRE                   ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL2_THREE_WIRE_POS)) /**< CTRL2_THREE_WIRE Mask */

#define MXC_F_SPIXR_CTRL2_SSPOL_POS                    16 /**< CTRL2_SSPOL Position */
#define MXC_F_SPIXR_CTRL2_SSPOL                        ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL2_SSPOL_POS)) /**< CTRL2_SSPOL Mask */

/**@} end of group SPIXR_CTRL2_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_CTRL3 SPIXR_CTRL3
 * @brief    Register for controlling SPI peripheral.
 * @{
 */
#define MXC_F_SPIXR_CTRL3_SSACT1_POS                   0 /**< CTRL3_SSACT1 Position */
#define MXC_F_SPIXR_CTRL3_SSACT1                       ((uint32_t)(0xFFUL << MXC_F_SPIXR_CTRL3_SSACT1_POS)) /**< CTRL3_SSACT1 Mask */
#define MXC_V_SPIXR_CTRL3_SSACT1_256                   ((uint32_t)0x0UL) /**< CTRL3_SSACT1_256 Value */
#define MXC_S_SPIXR_CTRL3_SSACT1_256                   (MXC_V_SPIXR_CTRL3_SSACT1_256 << MXC_F_SPIXR_CTRL3_SSACT1_POS) /**< CTRL3_SSACT1_256 Setting */

#define MXC_F_SPIXR_CTRL3_SSACT2_POS                   8 /**< CTRL3_SSACT2 Position */
#define MXC_F_SPIXR_CTRL3_SSACT2                       ((uint32_t)(0xFFUL << MXC_F_SPIXR_CTRL3_SSACT2_POS)) /**< CTRL3_SSACT2 Mask */
#define MXC_V_SPIXR_CTRL3_SSACT2_256                   ((uint32_t)0x0UL) /**< CTRL3_SSACT2_256 Value */
#define MXC_S_SPIXR_CTRL3_SSACT2_256                   (MXC_V_SPIXR_CTRL3_SSACT2_256 << MXC_F_SPIXR_CTRL3_SSACT2_POS) /**< CTRL3_SSACT2_256 Setting */

#define MXC_F_SPIXR_CTRL3_SSIACT_POS                   16 /**< CTRL3_SSIACT Position */
#define MXC_F_SPIXR_CTRL3_SSIACT                       ((uint32_t)(0xFFUL << MXC_F_SPIXR_CTRL3_SSIACT_POS)) /**< CTRL3_SSIACT Mask */
#define MXC_V_SPIXR_CTRL3_SSIACT_256                   ((uint32_t)0x0UL) /**< CTRL3_SSIACT_256 Value */
#define MXC_S_SPIXR_CTRL3_SSIACT_256                   (MXC_V_SPIXR_CTRL3_SSIACT_256 << MXC_F_SPIXR_CTRL3_SSIACT_POS) /**< CTRL3_SSIACT_256 Setting */

/**@} end of group SPIXR_CTRL3_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_BRGCTRL SPIXR_BRGCTRL
 * @brief    Register for controlling SPI clock rate.
 * @{
 */
#define MXC_F_SPIXR_BRGCTRL_LOW_POS                    0 /**< BRGCTRL_LOW Position */
#define MXC_F_SPIXR_BRGCTRL_LOW                        ((uint32_t)(0xFFUL << MXC_F_SPIXR_BRGCTRL_LOW_POS)) /**< BRGCTRL_LOW Mask */
#define MXC_V_SPIXR_BRGCTRL_LOW_DIS                    ((uint32_t)0x0UL) /**< BRGCTRL_LOW_DIS Value */
#define MXC_S_SPIXR_BRGCTRL_LOW_DIS                    (MXC_V_SPIXR_BRGCTRL_LOW_DIS << MXC_F_SPIXR_BRGCTRL_LOW_POS) /**< BRGCTRL_LOW_DIS Setting */

#define MXC_F_SPIXR_BRGCTRL_HIGH_POS                   8 /**< BRGCTRL_HIGH Position */
#define MXC_F_SPIXR_BRGCTRL_HIGH                       ((uint32_t)(0xFFUL << MXC_F_SPIXR_BRGCTRL_HIGH_POS)) /**< BRGCTRL_HIGH Mask */
#define MXC_V_SPIXR_BRGCTRL_HIGH_DIS                   ((uint32_t)0x0UL) /**< BRGCTRL_HIGH_DIS Value */
#define MXC_S_SPIXR_BRGCTRL_HIGH_DIS                   (MXC_V_SPIXR_BRGCTRL_HIGH_DIS << MXC_F_SPIXR_BRGCTRL_HIGH_POS) /**< BRGCTRL_HIGH_DIS Setting */

#define MXC_F_SPIXR_BRGCTRL_SCALE_POS                  16 /**< BRGCTRL_SCALE Position */
#define MXC_F_SPIXR_BRGCTRL_SCALE                      ((uint32_t)(0xFUL << MXC_F_SPIXR_BRGCTRL_SCALE_POS)) /**< BRGCTRL_SCALE Mask */

/**@} end of group SPIXR_BRGCTRL_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_DMA SPIXR_DMA
 * @brief    Register for controlling DMA.
 * @{
 */
#define MXC_F_SPIXR_DMA_TX_FIFO_LVL_POS                0 /**< DMA_TX_FIFO_LVL Position */
#define MXC_F_SPIXR_DMA_TX_FIFO_LVL                    ((uint32_t)(0x3FUL << MXC_F_SPIXR_DMA_TX_FIFO_LVL_POS)) /**< DMA_TX_FIFO_LVL Mask */

#define MXC_F_SPIXR_DMA_TX_FIFO_EN_POS                 6 /**< DMA_TX_FIFO_EN Position */
#define MXC_F_SPIXR_DMA_TX_FIFO_EN                     ((uint32_t)(0x1UL << MXC_F_SPIXR_DMA_TX_FIFO_EN_POS)) /**< DMA_TX_FIFO_EN Mask */

#define MXC_F_SPIXR_DMA_TX_FIFO_CLEAR_POS              7 /**< DMA_TX_FIFO_CLEAR Position */
#define MXC_F_SPIXR_DMA_TX_FIFO_CLEAR                  ((uint32_t)(0x1UL << MXC_F_SPIXR_DMA_TX_FIFO_CLEAR_POS)) /**< DMA_TX_FIFO_CLEAR Mask */

#define MXC_F_SPIXR_DMA_TX_FIFO_CNT_POS                8 /**< DMA_TX_FIFO_CNT Position */
#define MXC_F_SPIXR_DMA_TX_FIFO_CNT                    ((uint32_t)(0x1FUL << MXC_F_SPIXR_DMA_TX_FIFO_CNT_POS)) /**< DMA_TX_FIFO_CNT Mask */

#define MXC_F_SPIXR_DMA_DMA_TX_EN_POS                  15 /**< DMA_DMA_TX_EN Position */
#define MXC_F_SPIXR_DMA_DMA_TX_EN                      ((uint32_t)(0x1UL << MXC_F_SPIXR_DMA_DMA_TX_EN_POS)) /**< DMA_DMA_TX_EN Mask */

#define MXC_F_SPIXR_DMA_RX_FIFO_LVL_POS                16 /**< DMA_RX_FIFO_LVL Position */
#define MXC_F_SPIXR_DMA_RX_FIFO_LVL                    ((uint32_t)(0x3FUL << MXC_F_SPIXR_DMA_RX_FIFO_LVL_POS)) /**< DMA_RX_FIFO_LVL Mask */

#define MXC_F_SPIXR_DMA_RX_FIFO_EN_POS                 22 /**< DMA_RX_FIFO_EN Position */
#define MXC_F_SPIXR_DMA_RX_FIFO_EN                     ((uint32_t)(0x1UL << MXC_F_SPIXR_DMA_RX_FIFO_EN_POS)) /**< DMA_RX_FIFO_EN Mask */

#define MXC_F_SPIXR_DMA_RX_FIFO_CLR_POS                23 /**< DMA_RX_FIFO_CLR Position */
#define MXC_F_SPIXR_DMA_RX_FIFO_CLR                    ((uint32_t)(0x1UL << MXC_F_SPIXR_DMA_RX_FIFO_CLR_POS)) /**< DMA_RX_FIFO_CLR Mask */

#define MXC_F_SPIXR_DMA_RX_FIFO_CNT_POS                24 /**< DMA_RX_FIFO_CNT Position */
#define MXC_F_SPIXR_DMA_RX_FIFO_CNT                    ((uint32_t)(0x3FUL << MXC_F_SPIXR_DMA_RX_FIFO_CNT_POS)) /**< DMA_RX_FIFO_CNT Mask */

#define MXC_F_SPIXR_DMA_DMA_RX_EN_POS                  31 /**< DMA_DMA_RX_EN Position */
#define MXC_F_SPIXR_DMA_DMA_RX_EN                      ((uint32_t)(0x1UL << MXC_F_SPIXR_DMA_DMA_RX_EN_POS)) /**< DMA_DMA_RX_EN Mask */

/**@} end of group SPIXR_DMA_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_INTFL SPIXR_INTFL
 * @brief    Register for reading and clearing interrupt flags. All bits are write 1 to
 *           clear.
 * @{
 */
#define MXC_F_SPIXR_INTFL_TX_THRESH_POS                0 /**< INTFL_TX_THRESH Position */
#define MXC_F_SPIXR_INTFL_TX_THRESH                    ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_TX_THRESH_POS)) /**< INTFL_TX_THRESH Mask */

#define MXC_F_SPIXR_INTFL_TX_EMPTY_POS                 1 /**< INTFL_TX_EMPTY Position */
#define MXC_F_SPIXR_INTFL_TX_EMPTY                     ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_TX_EMPTY_POS)) /**< INTFL_TX_EMPTY Mask */

#define MXC_F_SPIXR_INTFL_RX_THRESH_POS                2 /**< INTFL_RX_THRESH Position */
#define MXC_F_SPIXR_INTFL_RX_THRESH                    ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_RX_THRESH_POS)) /**< INTFL_RX_THRESH Mask */

#define MXC_F_SPIXR_INTFL_RX_FULL_POS                  3 /**< INTFL_RX_FULL Position */
#define MXC_F_SPIXR_INTFL_RX_FULL                      ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_RX_FULL_POS)) /**< INTFL_RX_FULL Mask */

#define MXC_F_SPIXR_INTFL_SSA_POS                      4 /**< INTFL_SSA Position */
#define MXC_F_SPIXR_INTFL_SSA                          ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_SSA_POS)) /**< INTFL_SSA Mask */

#define MXC_F_SPIXR_INTFL_SSD_POS                      5 /**< INTFL_SSD Position */
#define MXC_F_SPIXR_INTFL_SSD                          ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_SSD_POS)) /**< INTFL_SSD Mask */

#define MXC_F_SPIXR_INTFL_FAULT_POS                    8 /**< INTFL_FAULT Position */
#define MXC_F_SPIXR_INTFL_FAULT                        ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_FAULT_POS)) /**< INTFL_FAULT Mask */

#define MXC_F_SPIXR_INTFL_ABORT_POS                    9 /**< INTFL_ABORT Position */
#define MXC_F_SPIXR_INTFL_ABORT                        ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_ABORT_POS)) /**< INTFL_ABORT Mask */

#define MXC_F_SPIXR_INTFL_M_DONE_POS                   11 /**< INTFL_M_DONE Position */
#define MXC_F_SPIXR_INTFL_M_DONE                       ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_M_DONE_POS)) /**< INTFL_M_DONE Mask */

#define MXC_F_SPIXR_INTFL_TX_OVR_POS                   12 /**< INTFL_TX_OVR Position */
#define MXC_F_SPIXR_INTFL_TX_OVR                       ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_TX_OVR_POS)) /**< INTFL_TX_OVR Mask */

#define MXC_F_SPIXR_INTFL_TX_UND_POS                   13 /**< INTFL_TX_UND Position */
#define MXC_F_SPIXR_INTFL_TX_UND                       ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_TX_UND_POS)) /**< INTFL_TX_UND Mask */

#define MXC_F_SPIXR_INTFL_RX_OVR_POS                   14 /**< INTFL_RX_OVR Position */
#define MXC_F_SPIXR_INTFL_RX_OVR                       ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_RX_OVR_POS)) /**< INTFL_RX_OVR Mask */

#define MXC_F_SPIXR_INTFL_RX_UND_POS                   15 /**< INTFL_RX_UND Position */
#define MXC_F_SPIXR_INTFL_RX_UND                       ((uint32_t)(0x1UL << MXC_F_SPIXR_INTFL_RX_UND_POS)) /**< INTFL_RX_UND Mask */

/**@} end of group SPIXR_INTFL_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_INTEN SPIXR_INTEN
 * @brief    Register for enabling interrupts.
 * @{
 */
#define MXC_F_SPIXR_INTEN_TX_THRESH_POS                0 /**< INTEN_TX_THRESH Position */
#define MXC_F_SPIXR_INTEN_TX_THRESH                    ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_TX_THRESH_POS)) /**< INTEN_TX_THRESH Mask */

#define MXC_F_SPIXR_INTEN_TX_EMPTY_POS                 1 /**< INTEN_TX_EMPTY Position */
#define MXC_F_SPIXR_INTEN_TX_EMPTY                     ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_TX_EMPTY_POS)) /**< INTEN_TX_EMPTY Mask */

#define MXC_F_SPIXR_INTEN_RX_THRESH_POS                2 /**< INTEN_RX_THRESH Position */
#define MXC_F_SPIXR_INTEN_RX_THRESH                    ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_RX_THRESH_POS)) /**< INTEN_RX_THRESH Mask */

#define MXC_F_SPIXR_INTEN_RX_FULL_POS                  3 /**< INTEN_RX_FULL Position */
#define MXC_F_SPIXR_INTEN_RX_FULL                      ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_RX_FULL_POS)) /**< INTEN_RX_FULL Mask */

#define MXC_F_SPIXR_INTEN_SSA_POS                      4 /**< INTEN_SSA Position */
#define MXC_F_SPIXR_INTEN_SSA                          ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_SSA_POS)) /**< INTEN_SSA Mask */

#define MXC_F_SPIXR_INTEN_SSD_POS                      5 /**< INTEN_SSD Position */
#define MXC_F_SPIXR_INTEN_SSD                          ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_SSD_POS)) /**< INTEN_SSD Mask */

#define MXC_F_SPIXR_INTEN_FAULT_POS                    8 /**< INTEN_FAULT Position */
#define MXC_F_SPIXR_INTEN_FAULT                        ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_FAULT_POS)) /**< INTEN_FAULT Mask */

#define MXC_F_SPIXR_INTEN_ABORT_POS                    9 /**< INTEN_ABORT Position */
#define MXC_F_SPIXR_INTEN_ABORT                        ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_ABORT_POS)) /**< INTEN_ABORT Mask */

#define MXC_F_SPIXR_INTEN_M_DONE_POS                   11 /**< INTEN_M_DONE Position */
#define MXC_F_SPIXR_INTEN_M_DONE                       ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_M_DONE_POS)) /**< INTEN_M_DONE Mask */

#define MXC_F_SPIXR_INTEN_TX_OVR_POS                   12 /**< INTEN_TX_OVR Position */
#define MXC_F_SPIXR_INTEN_TX_OVR                       ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_TX_OVR_POS)) /**< INTEN_TX_OVR Mask */

#define MXC_F_SPIXR_INTEN_TX_UND_POS                   13 /**< INTEN_TX_UND Position */
#define MXC_F_SPIXR_INTEN_TX_UND                       ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_TX_UND_POS)) /**< INTEN_TX_UND Mask */

#define MXC_F_SPIXR_INTEN_RX_OVR_POS                   14 /**< INTEN_RX_OVR Position */
#define MXC_F_SPIXR_INTEN_RX_OVR                       ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_RX_OVR_POS)) /**< INTEN_RX_OVR Mask */

#define MXC_F_SPIXR_INTEN_RX_UND_POS                   15 /**< INTEN_RX_UND Position */
#define MXC_F_SPIXR_INTEN_RX_UND                       ((uint32_t)(0x1UL << MXC_F_SPIXR_INTEN_RX_UND_POS)) /**< INTEN_RX_UND Mask */

/**@} end of group SPIXR_INTEN_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_WKFL SPIXR_WKFL
 * @brief    Register for wake up flags. All bits in this register are write 1 to clear.
 * @{
 */
#define MXC_F_SPIXR_WKFL_TX_THRESH_POS                 0 /**< WKFL_TX_THRESH Position */
#define MXC_F_SPIXR_WKFL_TX_THRESH                     ((uint32_t)(0x1UL << MXC_F_SPIXR_WKFL_TX_THRESH_POS)) /**< WKFL_TX_THRESH Mask */

#define MXC_F_SPIXR_WKFL_TX_EM_POS                     1 /**< WKFL_TX_EM Position */
#define MXC_F_SPIXR_WKFL_TX_EM                         ((uint32_t)(0x1UL << MXC_F_SPIXR_WKFL_TX_EM_POS)) /**< WKFL_TX_EM Mask */

#define MXC_F_SPIXR_WKFL_RX_THRESH_POS                 2 /**< WKFL_RX_THRESH Position */
#define MXC_F_SPIXR_WKFL_RX_THRESH                     ((uint32_t)(0x1UL << MXC_F_SPIXR_WKFL_RX_THRESH_POS)) /**< WKFL_RX_THRESH Mask */

#define MXC_F_SPIXR_WKFL_RX_FULL_POS                   3 /**< WKFL_RX_FULL Position */
#define MXC_F_SPIXR_WKFL_RX_FULL                       ((uint32_t)(0x1UL << MXC_F_SPIXR_WKFL_RX_FULL_POS)) /**< WKFL_RX_FULL Mask */

/**@} end of group SPIXR_WKFL_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_WKEN SPIXR_WKEN
 * @brief    Register for wake up enable.
 * @{
 */
#define MXC_F_SPIXR_WKEN_TX_THRESH_POS                 0 /**< WKEN_TX_THRESH Position */
#define MXC_F_SPIXR_WKEN_TX_THRESH                     ((uint32_t)(0x1UL << MXC_F_SPIXR_WKEN_TX_THRESH_POS)) /**< WKEN_TX_THRESH Mask */

#define MXC_F_SPIXR_WKEN_TX_EM_POS                     1 /**< WKEN_TX_EM Position */
#define MXC_F_SPIXR_WKEN_TX_EM                         ((uint32_t)(0x1UL << MXC_F_SPIXR_WKEN_TX_EM_POS)) /**< WKEN_TX_EM Mask */

#define MXC_F_SPIXR_WKEN_RX_THRESH_POS                 2 /**< WKEN_RX_THRESH Position */
#define MXC_F_SPIXR_WKEN_RX_THRESH                     ((uint32_t)(0x1UL << MXC_F_SPIXR_WKEN_RX_THRESH_POS)) /**< WKEN_RX_THRESH Mask */

#define MXC_F_SPIXR_WKEN_RX_FULL_POS                   3 /**< WKEN_RX_FULL Position */
#define MXC_F_SPIXR_WKEN_RX_FULL                       ((uint32_t)(0x1UL << MXC_F_SPIXR_WKEN_RX_FULL_POS)) /**< WKEN_RX_FULL Mask */

/**@} end of group SPIXR_WKEN_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_STAT SPIXR_STAT
 * @brief    SPI Status register.
 * @{
 */
#define MXC_F_SPIXR_STAT_BUSY_POS                      0 /**< STAT_BUSY Position */
#define MXC_F_SPIXR_STAT_BUSY                          ((uint32_t)(0x1UL << MXC_F_SPIXR_STAT_BUSY_POS)) /**< STAT_BUSY Mask */

/**@} end of group SPIXR_STAT_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_XMEMCTRL SPIXR_XMEMCTRL
 * @brief    Register to control external memory.
 * @{
 */
#define MXC_F_SPIXR_XMEMCTRL_RD_CMD_POS                0 /**< XMEMCTRL_RD_CMD Position */
#define MXC_F_SPIXR_XMEMCTRL_RD_CMD                    ((uint32_t)(0xFFUL << MXC_F_SPIXR_XMEMCTRL_RD_CMD_POS)) /**< XMEMCTRL_RD_CMD Mask */

#define MXC_F_SPIXR_XMEMCTRL_WR_CMD_POS                8 /**< XMEMCTRL_WR_CMD Position */
#define MXC_F_SPIXR_XMEMCTRL_WR_CMD                    ((uint32_t)(0xFFUL << MXC_F_SPIXR_XMEMCTRL_WR_CMD_POS)) /**< XMEMCTRL_WR_CMD Mask */

#define MXC_F_SPIXR_XMEMCTRL_DUMMY_CLK_POS             16 /**< XMEMCTRL_DUMMY_CLK Position */
#define MXC_F_SPIXR_XMEMCTRL_DUMMY_CLK                 ((uint32_t)(0xFFUL << MXC_F_SPIXR_XMEMCTRL_DUMMY_CLK_POS)) /**< XMEMCTRL_DUMMY_CLK Mask */

#define MXC_F_SPIXR_XMEMCTRL_XMEM_EN_POS               31 /**< XMEMCTRL_XMEM_EN Position */
#define MXC_F_SPIXR_XMEMCTRL_XMEM_EN                   ((uint32_t)(0x1UL << MXC_F_SPIXR_XMEMCTRL_XMEM_EN_POS)) /**< XMEMCTRL_XMEM_EN Mask */

/**@} end of group SPIXR_XMEMCTRL_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_SPIXR_REGS_H_
