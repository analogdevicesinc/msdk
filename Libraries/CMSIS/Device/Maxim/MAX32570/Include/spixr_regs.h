/**
 * @file    spixr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SPIXR Peripheral Module.
 * @note    This file is @generated.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_SPIXR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_SPIXR_REGS_H_

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
#define MXC_F_SPIXR_CTRL1_SPIEN_POS                    0 /**< CTRL1_SPIEN Position */
#define MXC_F_SPIXR_CTRL1_SPIEN                        ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL1_SPIEN_POS)) /**< CTRL1_SPIEN Mask */

#define MXC_F_SPIXR_CTRL1_MMEN_POS                     1 /**< CTRL1_MMEN Position */
#define MXC_F_SPIXR_CTRL1_MMEN                         ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL1_MMEN_POS)) /**< CTRL1_MMEN Mask */

#define MXC_F_SPIXR_CTRL1_SSIO_POS                     4 /**< CTRL1_SSIO Position */
#define MXC_F_SPIXR_CTRL1_SSIO                         ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL1_SSIO_POS)) /**< CTRL1_SSIO Mask */

#define MXC_F_SPIXR_CTRL1_TX_START_POS                 5 /**< CTRL1_TX_START Position */
#define MXC_F_SPIXR_CTRL1_TX_START                     ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL1_TX_START_POS)) /**< CTRL1_TX_START Mask */

#define MXC_F_SPIXR_CTRL1_SS_CTRL_POS                  8 /**< CTRL1_SS_CTRL Position */
#define MXC_F_SPIXR_CTRL1_SS_CTRL                      ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL1_SS_CTRL_POS)) /**< CTRL1_SS_CTRL Mask */

#define MXC_F_SPIXR_CTRL1_SS_POS                       16 /**< CTRL1_SS Position */
#define MXC_F_SPIXR_CTRL1_SS                           ((uint32_t)(0xFFUL << MXC_F_SPIXR_CTRL1_SS_POS)) /**< CTRL1_SS Mask */
#define MXC_V_SPIXR_CTRL1_SS_SS0                       ((uint32_t)0x1UL) /**< CTRL1_SS_SS0 Value */
#define MXC_S_SPIXR_CTRL1_SS_SS0                       (MXC_V_SPIXR_CTRL1_SS_SS0 << MXC_F_SPIXR_CTRL1_SS_POS) /**< CTRL1_SS_SS0 Setting */
#define MXC_V_SPIXR_CTRL1_SS_SS1                       ((uint32_t)0x2UL) /**< CTRL1_SS_SS1 Value */
#define MXC_S_SPIXR_CTRL1_SS_SS1                       (MXC_V_SPIXR_CTRL1_SS_SS1 << MXC_F_SPIXR_CTRL1_SS_POS) /**< CTRL1_SS_SS1 Setting */
#define MXC_V_SPIXR_CTRL1_SS_SS2                       ((uint32_t)0x4UL) /**< CTRL1_SS_SS2 Value */
#define MXC_S_SPIXR_CTRL1_SS_SS2                       (MXC_V_SPIXR_CTRL1_SS_SS2 << MXC_F_SPIXR_CTRL1_SS_POS) /**< CTRL1_SS_SS2 Setting */
#define MXC_V_SPIXR_CTRL1_SS_SS3                       ((uint32_t)0x8UL) /**< CTRL1_SS_SS3 Value */
#define MXC_S_SPIXR_CTRL1_SS_SS3                       (MXC_V_SPIXR_CTRL1_SS_SS3 << MXC_F_SPIXR_CTRL1_SS_POS) /**< CTRL1_SS_SS3 Setting */
#define MXC_V_SPIXR_CTRL1_SS_SS4                       ((uint32_t)0x10UL) /**< CTRL1_SS_SS4 Value */
#define MXC_S_SPIXR_CTRL1_SS_SS4                       (MXC_V_SPIXR_CTRL1_SS_SS4 << MXC_F_SPIXR_CTRL1_SS_POS) /**< CTRL1_SS_SS4 Setting */
#define MXC_V_SPIXR_CTRL1_SS_SS5                       ((uint32_t)0x20UL) /**< CTRL1_SS_SS5 Value */
#define MXC_S_SPIXR_CTRL1_SS_SS5                       (MXC_V_SPIXR_CTRL1_SS_SS5 << MXC_F_SPIXR_CTRL1_SS_POS) /**< CTRL1_SS_SS5 Setting */
#define MXC_V_SPIXR_CTRL1_SS_SS6                       ((uint32_t)0x40UL) /**< CTRL1_SS_SS6 Value */
#define MXC_S_SPIXR_CTRL1_SS_SS6                       (MXC_V_SPIXR_CTRL1_SS_SS6 << MXC_F_SPIXR_CTRL1_SS_POS) /**< CTRL1_SS_SS6 Setting */
#define MXC_V_SPIXR_CTRL1_SS_SS7                       ((uint32_t)0x80UL) /**< CTRL1_SS_SS7 Value */
#define MXC_S_SPIXR_CTRL1_SS_SS7                       (MXC_V_SPIXR_CTRL1_SS_SS7 << MXC_F_SPIXR_CTRL1_SS_POS) /**< CTRL1_SS_SS7 Setting */

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

#define MXC_F_SPIXR_CTRL3_CPOL_POS                     1 /**< CTRL3_CPOL Position */
#define MXC_F_SPIXR_CTRL3_CPOL                         ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL3_CPOL_POS)) /**< CTRL3_CPOL Mask */

#define MXC_F_SPIXR_CTRL3_SCLK_FB_INV_POS              4 /**< CTRL3_SCLK_FB_INV Position */
#define MXC_F_SPIXR_CTRL3_SCLK_FB_INV                  ((uint32_t)(0x1UL << MXC_F_SPIXR_CTRL3_SCLK_FB_INV_POS)) /**< CTRL3_SCLK_FB_INV Mask */

#define MXC_F_SPIXR_CTRL3_NUMBITS_POS                  8 /**< CTRL3_NUMBITS Position */
#define MXC_F_SPIXR_CTRL3_NUMBITS                      ((uint32_t)(0xFUL << MXC_F_SPIXR_CTRL3_NUMBITS_POS)) /**< CTRL3_NUMBITS Mask */
#define MXC_V_SPIXR_CTRL3_NUMBITS_0                    ((uint32_t)0x0UL) /**< CTRL3_NUMBITS_0 Value */
#define MXC_S_SPIXR_CTRL3_NUMBITS_0                    (MXC_V_SPIXR_CTRL3_NUMBITS_0 << MXC_F_SPIXR_CTRL3_NUMBITS_POS) /**< CTRL3_NUMBITS_0 Setting */

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

#define MXC_F_SPIXR_CTRL3_SSPOL_POS                    16 /**< CTRL3_SSPOL Position */
#define MXC_F_SPIXR_CTRL3_SSPOL                        ((uint32_t)(0xFFUL << MXC_F_SPIXR_CTRL3_SSPOL_POS)) /**< CTRL3_SSPOL Mask */
#define MXC_V_SPIXR_CTRL3_SSPOL_SS0_HIGH               ((uint32_t)0x1UL) /**< CTRL3_SSPOL_SS0_HIGH Value */
#define MXC_S_SPIXR_CTRL3_SSPOL_SS0_HIGH               (MXC_V_SPIXR_CTRL3_SSPOL_SS0_HIGH << MXC_F_SPIXR_CTRL3_SSPOL_POS) /**< CTRL3_SSPOL_SS0_HIGH Setting */
#define MXC_V_SPIXR_CTRL3_SSPOL_SS1_HIGH               ((uint32_t)0x2UL) /**< CTRL3_SSPOL_SS1_HIGH Value */
#define MXC_S_SPIXR_CTRL3_SSPOL_SS1_HIGH               (MXC_V_SPIXR_CTRL3_SSPOL_SS1_HIGH << MXC_F_SPIXR_CTRL3_SSPOL_POS) /**< CTRL3_SSPOL_SS1_HIGH Setting */
#define MXC_V_SPIXR_CTRL3_SSPOL_SS2_HIGH               ((uint32_t)0x4UL) /**< CTRL3_SSPOL_SS2_HIGH Value */
#define MXC_S_SPIXR_CTRL3_SSPOL_SS2_HIGH               (MXC_V_SPIXR_CTRL3_SSPOL_SS2_HIGH << MXC_F_SPIXR_CTRL3_SSPOL_POS) /**< CTRL3_SSPOL_SS2_HIGH Setting */
#define MXC_V_SPIXR_CTRL3_SSPOL_SS3_HIGH               ((uint32_t)0x8UL) /**< CTRL3_SSPOL_SS3_HIGH Value */
#define MXC_S_SPIXR_CTRL3_SSPOL_SS3_HIGH               (MXC_V_SPIXR_CTRL3_SSPOL_SS3_HIGH << MXC_F_SPIXR_CTRL3_SSPOL_POS) /**< CTRL3_SSPOL_SS3_HIGH Setting */
#define MXC_V_SPIXR_CTRL3_SSPOL_SS4_HIGH               ((uint32_t)0x10UL) /**< CTRL3_SSPOL_SS4_HIGH Value */
#define MXC_S_SPIXR_CTRL3_SSPOL_SS4_HIGH               (MXC_V_SPIXR_CTRL3_SSPOL_SS4_HIGH << MXC_F_SPIXR_CTRL3_SSPOL_POS) /**< CTRL3_SSPOL_SS4_HIGH Setting */
#define MXC_V_SPIXR_CTRL3_SSPOL_SS5_HIGH               ((uint32_t)0x20UL) /**< CTRL3_SSPOL_SS5_HIGH Value */
#define MXC_S_SPIXR_CTRL3_SSPOL_SS5_HIGH               (MXC_V_SPIXR_CTRL3_SSPOL_SS5_HIGH << MXC_F_SPIXR_CTRL3_SSPOL_POS) /**< CTRL3_SSPOL_SS5_HIGH Setting */
#define MXC_V_SPIXR_CTRL3_SSPOL_SS6_HIGH               ((uint32_t)0x40UL) /**< CTRL3_SSPOL_SS6_HIGH Value */
#define MXC_S_SPIXR_CTRL3_SSPOL_SS6_HIGH               (MXC_V_SPIXR_CTRL3_SSPOL_SS6_HIGH << MXC_F_SPIXR_CTRL3_SSPOL_POS) /**< CTRL3_SSPOL_SS6_HIGH Setting */
#define MXC_V_SPIXR_CTRL3_SSPOL_SS7_HIGH               ((uint32_t)0x80UL) /**< CTRL3_SSPOL_SS7_HIGH Value */
#define MXC_S_SPIXR_CTRL3_SSPOL_SS7_HIGH               (MXC_V_SPIXR_CTRL3_SSPOL_SS7_HIGH << MXC_F_SPIXR_CTRL3_SSPOL_POS) /**< CTRL3_SSPOL_SS7_HIGH Setting */

/**@} end of group SPIXR_CTRL3_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_SS_TIME SPIXR_SS_TIME
 * @brief    Register for controlling SPI peripheral.
 * @{
 */
#define MXC_F_SPIXR_SS_TIME_SSACT1_POS                 0 /**< SS_TIME_SSACT1 Position */
#define MXC_F_SPIXR_SS_TIME_SSACT1                     ((uint32_t)(0xFFUL << MXC_F_SPIXR_SS_TIME_SSACT1_POS)) /**< SS_TIME_SSACT1 Mask */
#define MXC_V_SPIXR_SS_TIME_SSACT1_256                 ((uint32_t)0x0UL) /**< SS_TIME_SSACT1_256 Value */
#define MXC_S_SPIXR_SS_TIME_SSACT1_256                 (MXC_V_SPIXR_SS_TIME_SSACT1_256 << MXC_F_SPIXR_SS_TIME_SSACT1_POS) /**< SS_TIME_SSACT1_256 Setting */

#define MXC_F_SPIXR_SS_TIME_SSACT2_POS                 8 /**< SS_TIME_SSACT2 Position */
#define MXC_F_SPIXR_SS_TIME_SSACT2                     ((uint32_t)(0xFFUL << MXC_F_SPIXR_SS_TIME_SSACT2_POS)) /**< SS_TIME_SSACT2 Mask */
#define MXC_V_SPIXR_SS_TIME_SSACT2_256                 ((uint32_t)0x0UL) /**< SS_TIME_SSACT2_256 Value */
#define MXC_S_SPIXR_SS_TIME_SSACT2_256                 (MXC_V_SPIXR_SS_TIME_SSACT2_256 << MXC_F_SPIXR_SS_TIME_SSACT2_POS) /**< SS_TIME_SSACT2_256 Setting */

#define MXC_F_SPIXR_SS_TIME_SSINACT_POS                16 /**< SS_TIME_SSINACT Position */
#define MXC_F_SPIXR_SS_TIME_SSINACT                    ((uint32_t)(0xFFUL << MXC_F_SPIXR_SS_TIME_SSINACT_POS)) /**< SS_TIME_SSINACT Mask */
#define MXC_V_SPIXR_SS_TIME_SSINACT_256                ((uint32_t)0x0UL) /**< SS_TIME_SSINACT_256 Value */
#define MXC_S_SPIXR_SS_TIME_SSINACT_256                (MXC_V_SPIXR_SS_TIME_SSINACT_256 << MXC_F_SPIXR_SS_TIME_SSINACT_POS) /**< SS_TIME_SSINACT_256 Setting */

/**@} end of group SPIXR_SS_TIME_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_BRG_CTRL SPIXR_BRG_CTRL
 * @brief    Register for controlling SPI clock rate.
 * @{
 */
#define MXC_F_SPIXR_BRG_CTRL_LOW_POS                   0 /**< BRG_CTRL_LOW Position */
#define MXC_F_SPIXR_BRG_CTRL_LOW                       ((uint32_t)(0xFFUL << MXC_F_SPIXR_BRG_CTRL_LOW_POS)) /**< BRG_CTRL_LOW Mask */
#define MXC_V_SPIXR_BRG_CTRL_LOW_DIS                   ((uint32_t)0x0UL) /**< BRG_CTRL_LOW_DIS Value */
#define MXC_S_SPIXR_BRG_CTRL_LOW_DIS                   (MXC_V_SPIXR_BRG_CTRL_LOW_DIS << MXC_F_SPIXR_BRG_CTRL_LOW_POS) /**< BRG_CTRL_LOW_DIS Setting */

#define MXC_F_SPIXR_BRG_CTRL_HI_POS                    8 /**< BRG_CTRL_HI Position */
#define MXC_F_SPIXR_BRG_CTRL_HI                        ((uint32_t)(0xFFUL << MXC_F_SPIXR_BRG_CTRL_HI_POS)) /**< BRG_CTRL_HI Mask */
#define MXC_V_SPIXR_BRG_CTRL_HI_DIS                    ((uint32_t)0x0UL) /**< BRG_CTRL_HI_DIS Value */
#define MXC_S_SPIXR_BRG_CTRL_HI_DIS                    (MXC_V_SPIXR_BRG_CTRL_HI_DIS << MXC_F_SPIXR_BRG_CTRL_HI_POS) /**< BRG_CTRL_HI_DIS Setting */

#define MXC_F_SPIXR_BRG_CTRL_SCALE_POS                 16 /**< BRG_CTRL_SCALE Position */
#define MXC_F_SPIXR_BRG_CTRL_SCALE                     ((uint32_t)(0xFUL << MXC_F_SPIXR_BRG_CTRL_SCALE_POS)) /**< BRG_CTRL_SCALE Mask */

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

#define MXC_F_SPIXR_DMA_TX_FIFO_CLEAR_POS              7 /**< DMA_TX_FIFO_CLEAR Position */
#define MXC_F_SPIXR_DMA_TX_FIFO_CLEAR                  ((uint32_t)(0x1UL << MXC_F_SPIXR_DMA_TX_FIFO_CLEAR_POS)) /**< DMA_TX_FIFO_CLEAR Mask */

#define MXC_F_SPIXR_DMA_TX_FIFO_CNT_POS                8 /**< DMA_TX_FIFO_CNT Position */
#define MXC_F_SPIXR_DMA_TX_FIFO_CNT                    ((uint32_t)(0x1FUL << MXC_F_SPIXR_DMA_TX_FIFO_CNT_POS)) /**< DMA_TX_FIFO_CNT Mask */

#define MXC_F_SPIXR_DMA_TX_DMA_EN_POS                  15 /**< DMA_TX_DMA_EN Position */
#define MXC_F_SPIXR_DMA_TX_DMA_EN                      ((uint32_t)(0x1UL << MXC_F_SPIXR_DMA_TX_DMA_EN_POS)) /**< DMA_TX_DMA_EN Mask */

#define MXC_F_SPIXR_DMA_RX_FIFO_LEVEL_POS              16 /**< DMA_RX_FIFO_LEVEL Position */
#define MXC_F_SPIXR_DMA_RX_FIFO_LEVEL                  ((uint32_t)(0x3FUL << MXC_F_SPIXR_DMA_RX_FIFO_LEVEL_POS)) /**< DMA_RX_FIFO_LEVEL Mask */

#define MXC_F_SPIXR_DMA_RX_FIFO_EN_POS                 22 /**< DMA_RX_FIFO_EN Position */
#define MXC_F_SPIXR_DMA_RX_FIFO_EN                     ((uint32_t)(0x1UL << MXC_F_SPIXR_DMA_RX_FIFO_EN_POS)) /**< DMA_RX_FIFO_EN Mask */

#define MXC_F_SPIXR_DMA_RX_FIFO_CLEAR_POS              23 /**< DMA_RX_FIFO_CLEAR Position */
#define MXC_F_SPIXR_DMA_RX_FIFO_CLEAR                  ((uint32_t)(0x1UL << MXC_F_SPIXR_DMA_RX_FIFO_CLEAR_POS)) /**< DMA_RX_FIFO_CLEAR Mask */

#define MXC_F_SPIXR_DMA_RX_FIFO_CNT_POS                24 /**< DMA_RX_FIFO_CNT Position */
#define MXC_F_SPIXR_DMA_RX_FIFO_CNT                    ((uint32_t)(0x3FUL << MXC_F_SPIXR_DMA_RX_FIFO_CNT_POS)) /**< DMA_RX_FIFO_CNT Mask */

#define MXC_F_SPIXR_DMA_RX_DMA_EN_POS                  31 /**< DMA_RX_DMA_EN Position */
#define MXC_F_SPIXR_DMA_RX_DMA_EN                      ((uint32_t)(0x1UL << MXC_F_SPIXR_DMA_RX_DMA_EN_POS)) /**< DMA_RX_DMA_EN Mask */

/**@} end of group SPIXR_DMA_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_INT_FL SPIXR_INT_FL
 * @brief    Register for reading and clearing interrupt flags. All bits are write 1 to
 *           clear.
 * @{
 */
#define MXC_F_SPIXR_INT_FL_TX_THRESH_POS               0 /**< INT_FL_TX_THRESH Position */
#define MXC_F_SPIXR_INT_FL_TX_THRESH                   ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_TX_THRESH_POS)) /**< INT_FL_TX_THRESH Mask */

#define MXC_F_SPIXR_INT_FL_TX_EMPTY_POS                1 /**< INT_FL_TX_EMPTY Position */
#define MXC_F_SPIXR_INT_FL_TX_EMPTY                    ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_TX_EMPTY_POS)) /**< INT_FL_TX_EMPTY Mask */

#define MXC_F_SPIXR_INT_FL_RX_THRESH_POS               2 /**< INT_FL_RX_THRESH Position */
#define MXC_F_SPIXR_INT_FL_RX_THRESH                   ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_RX_THRESH_POS)) /**< INT_FL_RX_THRESH Mask */

#define MXC_F_SPIXR_INT_FL_RX_FULL_POS                 3 /**< INT_FL_RX_FULL Position */
#define MXC_F_SPIXR_INT_FL_RX_FULL                     ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_RX_FULL_POS)) /**< INT_FL_RX_FULL Mask */

#define MXC_F_SPIXR_INT_FL_SSA_POS                     4 /**< INT_FL_SSA Position */
#define MXC_F_SPIXR_INT_FL_SSA                         ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_SSA_POS)) /**< INT_FL_SSA Mask */

#define MXC_F_SPIXR_INT_FL_SSD_POS                     5 /**< INT_FL_SSD Position */
#define MXC_F_SPIXR_INT_FL_SSD                         ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_SSD_POS)) /**< INT_FL_SSD Mask */

#define MXC_F_SPIXR_INT_FL_FAULT_POS                   8 /**< INT_FL_FAULT Position */
#define MXC_F_SPIXR_INT_FL_FAULT                       ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_FAULT_POS)) /**< INT_FL_FAULT Mask */

#define MXC_F_SPIXR_INT_FL_ABORT_POS                   9 /**< INT_FL_ABORT Position */
#define MXC_F_SPIXR_INT_FL_ABORT                       ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_ABORT_POS)) /**< INT_FL_ABORT Mask */

#define MXC_F_SPIXR_INT_FL_M_DONE_POS                  11 /**< INT_FL_M_DONE Position */
#define MXC_F_SPIXR_INT_FL_M_DONE                      ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_M_DONE_POS)) /**< INT_FL_M_DONE Mask */

#define MXC_F_SPIXR_INT_FL_TX_OVR_POS                  12 /**< INT_FL_TX_OVR Position */
#define MXC_F_SPIXR_INT_FL_TX_OVR                      ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_TX_OVR_POS)) /**< INT_FL_TX_OVR Mask */

#define MXC_F_SPIXR_INT_FL_TX_UND_POS                  13 /**< INT_FL_TX_UND Position */
#define MXC_F_SPIXR_INT_FL_TX_UND                      ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_TX_UND_POS)) /**< INT_FL_TX_UND Mask */

#define MXC_F_SPIXR_INT_FL_RX_OVR_POS                  14 /**< INT_FL_RX_OVR Position */
#define MXC_F_SPIXR_INT_FL_RX_OVR                      ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_RX_OVR_POS)) /**< INT_FL_RX_OVR Mask */

#define MXC_F_SPIXR_INT_FL_RX_UND_POS                  15 /**< INT_FL_RX_UND Position */
#define MXC_F_SPIXR_INT_FL_RX_UND                      ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_FL_RX_UND_POS)) /**< INT_FL_RX_UND Mask */

/**@} end of group SPIXR_INT_FL_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_INT_EN SPIXR_INT_EN
 * @brief    Register for enabling interrupts.
 * @{
 */
#define MXC_F_SPIXR_INT_EN_TX_THRESH_POS               0 /**< INT_EN_TX_THRESH Position */
#define MXC_F_SPIXR_INT_EN_TX_THRESH                   ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_TX_THRESH_POS)) /**< INT_EN_TX_THRESH Mask */

#define MXC_F_SPIXR_INT_EN_TX_EMPTY_POS                1 /**< INT_EN_TX_EMPTY Position */
#define MXC_F_SPIXR_INT_EN_TX_EMPTY                    ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_TX_EMPTY_POS)) /**< INT_EN_TX_EMPTY Mask */

#define MXC_F_SPIXR_INT_EN_RX_THRESH_POS               2 /**< INT_EN_RX_THRESH Position */
#define MXC_F_SPIXR_INT_EN_RX_THRESH                   ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_RX_THRESH_POS)) /**< INT_EN_RX_THRESH Mask */

#define MXC_F_SPIXR_INT_EN_RX_FULL_POS                 3 /**< INT_EN_RX_FULL Position */
#define MXC_F_SPIXR_INT_EN_RX_FULL                     ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_RX_FULL_POS)) /**< INT_EN_RX_FULL Mask */

#define MXC_F_SPIXR_INT_EN_SSA_POS                     4 /**< INT_EN_SSA Position */
#define MXC_F_SPIXR_INT_EN_SSA                         ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_SSA_POS)) /**< INT_EN_SSA Mask */

#define MXC_F_SPIXR_INT_EN_SSD_POS                     5 /**< INT_EN_SSD Position */
#define MXC_F_SPIXR_INT_EN_SSD                         ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_SSD_POS)) /**< INT_EN_SSD Mask */

#define MXC_F_SPIXR_INT_EN_FAULT_POS                   8 /**< INT_EN_FAULT Position */
#define MXC_F_SPIXR_INT_EN_FAULT                       ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_FAULT_POS)) /**< INT_EN_FAULT Mask */

#define MXC_F_SPIXR_INT_EN_ABORT_POS                   9 /**< INT_EN_ABORT Position */
#define MXC_F_SPIXR_INT_EN_ABORT                       ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_ABORT_POS)) /**< INT_EN_ABORT Mask */

#define MXC_F_SPIXR_INT_EN_M_DONE_POS                  11 /**< INT_EN_M_DONE Position */
#define MXC_F_SPIXR_INT_EN_M_DONE                      ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_M_DONE_POS)) /**< INT_EN_M_DONE Mask */

#define MXC_F_SPIXR_INT_EN_TX_OVR_POS                  12 /**< INT_EN_TX_OVR Position */
#define MXC_F_SPIXR_INT_EN_TX_OVR                      ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_TX_OVR_POS)) /**< INT_EN_TX_OVR Mask */

#define MXC_F_SPIXR_INT_EN_TX_UND_POS                  13 /**< INT_EN_TX_UND Position */
#define MXC_F_SPIXR_INT_EN_TX_UND                      ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_TX_UND_POS)) /**< INT_EN_TX_UND Mask */

#define MXC_F_SPIXR_INT_EN_RX_OVR_POS                  14 /**< INT_EN_RX_OVR Position */
#define MXC_F_SPIXR_INT_EN_RX_OVR                      ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_RX_OVR_POS)) /**< INT_EN_RX_OVR Mask */

#define MXC_F_SPIXR_INT_EN_RX_UND_POS                  15 /**< INT_EN_RX_UND Position */
#define MXC_F_SPIXR_INT_EN_RX_UND                      ((uint32_t)(0x1UL << MXC_F_SPIXR_INT_EN_RX_UND_POS)) /**< INT_EN_RX_UND Mask */

/**@} end of group SPIXR_INT_EN_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_WAKE_FL SPIXR_WAKE_FL
 * @brief    Register for wake up flags. All bits in this register are write 1 to clear.
 * @{
 */
#define MXC_F_SPIXR_WAKE_FL_TX_THRESH_POS              0 /**< WAKE_FL_TX_THRESH Position */
#define MXC_F_SPIXR_WAKE_FL_TX_THRESH                  ((uint32_t)(0x1UL << MXC_F_SPIXR_WAKE_FL_TX_THRESH_POS)) /**< WAKE_FL_TX_THRESH Mask */

#define MXC_F_SPIXR_WAKE_FL_TX_EMPTY_POS               1 /**< WAKE_FL_TX_EMPTY Position */
#define MXC_F_SPIXR_WAKE_FL_TX_EMPTY                   ((uint32_t)(0x1UL << MXC_F_SPIXR_WAKE_FL_TX_EMPTY_POS)) /**< WAKE_FL_TX_EMPTY Mask */

#define MXC_F_SPIXR_WAKE_FL_RX_THRESH_POS              2 /**< WAKE_FL_RX_THRESH Position */
#define MXC_F_SPIXR_WAKE_FL_RX_THRESH                  ((uint32_t)(0x1UL << MXC_F_SPIXR_WAKE_FL_RX_THRESH_POS)) /**< WAKE_FL_RX_THRESH Mask */

#define MXC_F_SPIXR_WAKE_FL_RX_FULL_POS                3 /**< WAKE_FL_RX_FULL Position */
#define MXC_F_SPIXR_WAKE_FL_RX_FULL                    ((uint32_t)(0x1UL << MXC_F_SPIXR_WAKE_FL_RX_FULL_POS)) /**< WAKE_FL_RX_FULL Mask */

/**@} end of group SPIXR_WAKE_FL_Register */

/**
 * @ingroup  spixr_registers
 * @defgroup SPIXR_WAKE_EN SPIXR_WAKE_EN
 * @brief    Register for wake up enable.
 * @{
 */
#define MXC_F_SPIXR_WAKE_EN_TX_THRESH_POS              0 /**< WAKE_EN_TX_THRESH Position */
#define MXC_F_SPIXR_WAKE_EN_TX_THRESH                  ((uint32_t)(0x1UL << MXC_F_SPIXR_WAKE_EN_TX_THRESH_POS)) /**< WAKE_EN_TX_THRESH Mask */

#define MXC_F_SPIXR_WAKE_EN_TX_EMPTY_POS               1 /**< WAKE_EN_TX_EMPTY Position */
#define MXC_F_SPIXR_WAKE_EN_TX_EMPTY                   ((uint32_t)(0x1UL << MXC_F_SPIXR_WAKE_EN_TX_EMPTY_POS)) /**< WAKE_EN_TX_EMPTY Mask */

#define MXC_F_SPIXR_WAKE_EN_RX_THRESH_POS              2 /**< WAKE_EN_RX_THRESH Position */
#define MXC_F_SPIXR_WAKE_EN_RX_THRESH                  ((uint32_t)(0x1UL << MXC_F_SPIXR_WAKE_EN_RX_THRESH_POS)) /**< WAKE_EN_RX_THRESH Mask */

#define MXC_F_SPIXR_WAKE_EN_RX_FULL_POS                3 /**< WAKE_EN_RX_FULL Position */
#define MXC_F_SPIXR_WAKE_EN_RX_FULL                    ((uint32_t)(0x1UL << MXC_F_SPIXR_WAKE_EN_RX_FULL_POS)) /**< WAKE_EN_RX_FULL Mask */

/**@} end of group SPIXR_WAKE_EN_Register */

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
 * @defgroup SPIXR_XMEM_CTRL SPIXR_XMEM_CTRL
 * @brief    Register to control external memory.
 * @{
 */
#define MXC_F_SPIXR_XMEM_CTRL_RD_CMD_POS               0 /**< XMEM_CTRL_RD_CMD Position */
#define MXC_F_SPIXR_XMEM_CTRL_RD_CMD                   ((uint32_t)(0xFFUL << MXC_F_SPIXR_XMEM_CTRL_RD_CMD_POS)) /**< XMEM_CTRL_RD_CMD Mask */

#define MXC_F_SPIXR_XMEM_CTRL_WR_CMD_POS               8 /**< XMEM_CTRL_WR_CMD Position */
#define MXC_F_SPIXR_XMEM_CTRL_WR_CMD                   ((uint32_t)(0xFFUL << MXC_F_SPIXR_XMEM_CTRL_WR_CMD_POS)) /**< XMEM_CTRL_WR_CMD Mask */

#define MXC_F_SPIXR_XMEM_CTRL_DUMMY_CLK_POS            16 /**< XMEM_CTRL_DUMMY_CLK Position */
#define MXC_F_SPIXR_XMEM_CTRL_DUMMY_CLK                ((uint32_t)(0xFFUL << MXC_F_SPIXR_XMEM_CTRL_DUMMY_CLK_POS)) /**< XMEM_CTRL_DUMMY_CLK Mask */

#define MXC_F_SPIXR_XMEM_CTRL_XMEM_EN_POS              31 /**< XMEM_CTRL_XMEM_EN Position */
#define MXC_F_SPIXR_XMEM_CTRL_XMEM_EN                  ((uint32_t)(0x1UL << MXC_F_SPIXR_XMEM_CTRL_XMEM_EN_POS)) /**< XMEM_CTRL_XMEM_EN Mask */

/**@} end of group SPIXR_XMEM_CTRL_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_SPIXR_REGS_H_
