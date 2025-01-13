/**
 * @file    spi_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SPI Peripheral Module.
 * @note    This file is @generated.
 * @ingroup spi_registers
 */

/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_SPI_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_SPI_REGS_H_

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
 * @ingroup     spi
 * @defgroup    spi_registers SPI_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SPI Peripheral Module.
 * @details     SPI peripheral.
 */

/**
 * @ingroup spi_registers
 * Structure type to access the SPI Registers.
 */
typedef struct {
    union {
        __IO uint32_t fifo32;           /**< <tt>\b 0x00:</tt> SPI FIFO32 Register */
        __IO uint16_t fifo16[2];        /**< <tt>\b 0x00:</tt> SPI FIFO16 Register */
        __IO uint8_t  fifo8[4];         /**< <tt>\b 0x00:</tt> SPI FIFO8 Register */
    };
    __IO uint32_t ctrl0;                /**< <tt>\b 0x04:</tt> SPI CTRL0 Register */
    __IO uint32_t ctrl1;                /**< <tt>\b 0x08:</tt> SPI CTRL1 Register */
    __IO uint32_t ctrl2;                /**< <tt>\b 0x0C:</tt> SPI CTRL2 Register */
    __IO uint32_t tstime;               /**< <tt>\b 0x10:</tt> SPI TSTIME Register */
    __IO uint32_t clkctrl;              /**< <tt>\b 0x14:</tt> SPI CLKCTRL Register */
    __R  uint32_t rsv_0x18;
    __IO uint32_t dma;                  /**< <tt>\b 0x1C:</tt> SPI DMA Register */
    __IO uint32_t intfl;                /**< <tt>\b 0x20:</tt> SPI INTFL Register */
    __IO uint32_t inten;                /**< <tt>\b 0x24:</tt> SPI INTEN Register */
    __IO uint32_t wkfl;                 /**< <tt>\b 0x28:</tt> SPI WKFL Register */
    __IO uint32_t wken;                 /**< <tt>\b 0x2C:</tt> SPI WKEN Register */
    __I  uint32_t status;               /**< <tt>\b 0x30:</tt> SPI STATUS Register */
} mxc_spi_regs_t;

/* Register offsets for module SPI */
/**
 * @ingroup    spi_registers
 * @defgroup   SPI_Register_Offsets Register Offsets
 * @brief      SPI Peripheral Register Offsets from the SPI Base Peripheral Address.
 * @{
 */
#define MXC_R_SPI_FIFO32                   ((uint32_t)0x00000000UL) /**< Offset from SPI Base Address: <tt> 0x0000</tt> */
#define MXC_R_SPI_FIFO16                   ((uint32_t)0x00000000UL) /**< Offset from SPI Base Address: <tt> 0x0000</tt> */
#define MXC_R_SPI_FIFO8                    ((uint32_t)0x00000000UL) /**< Offset from SPI Base Address: <tt> 0x0000</tt> */
#define MXC_R_SPI_CTRL0                    ((uint32_t)0x00000004UL) /**< Offset from SPI Base Address: <tt> 0x0004</tt> */
#define MXC_R_SPI_CTRL1                    ((uint32_t)0x00000008UL) /**< Offset from SPI Base Address: <tt> 0x0008</tt> */
#define MXC_R_SPI_CTRL2                    ((uint32_t)0x0000000CUL) /**< Offset from SPI Base Address: <tt> 0x000C</tt> */
#define MXC_R_SPI_TSTIME                   ((uint32_t)0x00000010UL) /**< Offset from SPI Base Address: <tt> 0x0010</tt> */
#define MXC_R_SPI_CLKCTRL                  ((uint32_t)0x00000014UL) /**< Offset from SPI Base Address: <tt> 0x0014</tt> */
#define MXC_R_SPI_DMA                      ((uint32_t)0x0000001CUL) /**< Offset from SPI Base Address: <tt> 0x001C</tt> */
#define MXC_R_SPI_INTFL                    ((uint32_t)0x00000020UL) /**< Offset from SPI Base Address: <tt> 0x0020</tt> */
#define MXC_R_SPI_INTEN                    ((uint32_t)0x00000024UL) /**< Offset from SPI Base Address: <tt> 0x0024</tt> */
#define MXC_R_SPI_WKFL                     ((uint32_t)0x00000028UL) /**< Offset from SPI Base Address: <tt> 0x0028</tt> */
#define MXC_R_SPI_WKEN                     ((uint32_t)0x0000002CUL) /**< Offset from SPI Base Address: <tt> 0x002C</tt> */
#define MXC_R_SPI_STATUS                   ((uint32_t)0x00000030UL) /**< Offset from SPI Base Address: <tt> 0x0030</tt> */
/**@} end of group spi_registers */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_FIFO32 SPI_FIFO32
 * @brief    Register for reading and writing the FIFO.
 * @{
 */
#define MXC_F_SPI_FIFO32_DATA_POS                      0 /**< FIFO32_DATA Position */
#define MXC_F_SPI_FIFO32_DATA                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_SPI_FIFO32_DATA_POS)) /**< FIFO32_DATA Mask */

/**@} end of group SPI_FIFO32_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_FIFO16 SPI_FIFO16
 * @brief    Register for reading and writing the FIFO.
 * @{
 */
#define MXC_F_SPI_FIFO16_DATA_POS                      0 /**< FIFO16_DATA Position */
#define MXC_F_SPI_FIFO16_DATA                          ((uint16_t)(0xFFFFUL << MXC_F_SPI_FIFO16_DATA_POS)) /**< FIFO16_DATA Mask */

/**@} end of group SPI_FIFO16_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_FIFO8 SPI_FIFO8
 * @brief    Register for reading and writing the FIFO.
 * @{
 */
#define MXC_F_SPI_FIFO8_DATA_POS                       0 /**< FIFO8_DATA Position */
#define MXC_F_SPI_FIFO8_DATA                           ((uint8_t)(0xFFUL << MXC_F_SPI_FIFO8_DATA_POS)) /**< FIFO8_DATA Mask */

/**@} end of group SPI_FIFO8_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_CTRL0 SPI_CTRL0
 * @brief    Register for controlling SPI peripheral.
 * @{
 */
#define MXC_F_SPI_CTRL0_EN_POS                         0 /**< CTRL0_EN Position */
#define MXC_F_SPI_CTRL0_EN                             ((uint32_t)(0x1UL << MXC_F_SPI_CTRL0_EN_POS)) /**< CTRL0_EN Mask */

#define MXC_F_SPI_CTRL0_CONT_MODE_POS                  1 /**< CTRL0_CONT_MODE Position */
#define MXC_F_SPI_CTRL0_CONT_MODE                      ((uint32_t)(0x1UL << MXC_F_SPI_CTRL0_CONT_MODE_POS)) /**< CTRL0_CONT_MODE Mask */

#define MXC_F_SPI_CTRL0_TS_IO_POS                      4 /**< CTRL0_TS_IO Position */
#define MXC_F_SPI_CTRL0_TS_IO                          ((uint32_t)(0x1UL << MXC_F_SPI_CTRL0_TS_IO_POS)) /**< CTRL0_TS_IO Mask */

#define MXC_F_SPI_CTRL0_START_POS                      5 /**< CTRL0_START Position */
#define MXC_F_SPI_CTRL0_START                          ((uint32_t)(0x1UL << MXC_F_SPI_CTRL0_START_POS)) /**< CTRL0_START Mask */

#define MXC_F_SPI_CTRL0_TS_CTRL_POS                    8 /**< CTRL0_TS_CTRL Position */
#define MXC_F_SPI_CTRL0_TS_CTRL                        ((uint32_t)(0x1UL << MXC_F_SPI_CTRL0_TS_CTRL_POS)) /**< CTRL0_TS_CTRL Mask */

#define MXC_F_SPI_CTRL0_TS_ACTIVE_POS                  16 /**< CTRL0_TS_ACTIVE Position */
#define MXC_F_SPI_CTRL0_TS_ACTIVE                      ((uint32_t)(0xFUL << MXC_F_SPI_CTRL0_TS_ACTIVE_POS)) /**< CTRL0_TS_ACTIVE Mask */
#define MXC_V_SPI_CTRL0_TS_ACTIVE_TS0                  ((uint32_t)0x1UL) /**< CTRL0_TS_ACTIVE_TS0 Value */
#define MXC_S_SPI_CTRL0_TS_ACTIVE_TS0                  (MXC_V_SPI_CTRL0_TS_ACTIVE_TS0 << MXC_F_SPI_CTRL0_TS_ACTIVE_POS) /**< CTRL0_TS_ACTIVE_TS0 Setting */
#define MXC_V_SPI_CTRL0_TS_ACTIVE_TS1                  ((uint32_t)0x2UL) /**< CTRL0_TS_ACTIVE_TS1 Value */
#define MXC_S_SPI_CTRL0_TS_ACTIVE_TS1                  (MXC_V_SPI_CTRL0_TS_ACTIVE_TS1 << MXC_F_SPI_CTRL0_TS_ACTIVE_POS) /**< CTRL0_TS_ACTIVE_TS1 Setting */
#define MXC_V_SPI_CTRL0_TS_ACTIVE_TS2                  ((uint32_t)0x4UL) /**< CTRL0_TS_ACTIVE_TS2 Value */
#define MXC_S_SPI_CTRL0_TS_ACTIVE_TS2                  (MXC_V_SPI_CTRL0_TS_ACTIVE_TS2 << MXC_F_SPI_CTRL0_TS_ACTIVE_POS) /**< CTRL0_TS_ACTIVE_TS2 Setting */

/**@} end of group SPI_CTRL0_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_CTRL1 SPI_CTRL1
 * @brief    Register for controlling SPI peripheral.
 * @{
 */
#define MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS                0 /**< CTRL1_TX_NUM_CHAR Position */
#define MXC_F_SPI_CTRL1_TX_NUM_CHAR                    ((uint32_t)(0xFFFFUL << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS)) /**< CTRL1_TX_NUM_CHAR Mask */

#define MXC_F_SPI_CTRL1_RX_NUM_CHAR_POS                16 /**< CTRL1_RX_NUM_CHAR Position */
#define MXC_F_SPI_CTRL1_RX_NUM_CHAR                    ((uint32_t)(0xFFFFUL << MXC_F_SPI_CTRL1_RX_NUM_CHAR_POS)) /**< CTRL1_RX_NUM_CHAR Mask */

/**@} end of group SPI_CTRL1_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_CTRL2 SPI_CTRL2
 * @brief    Register for controlling SPI peripheral.
 * @{
 */
#define MXC_F_SPI_CTRL2_CLKPHA_POS                     0 /**< CTRL2_CLKPHA Position */
#define MXC_F_SPI_CTRL2_CLKPHA                         ((uint32_t)(0x1UL << MXC_F_SPI_CTRL2_CLKPHA_POS)) /**< CTRL2_CLKPHA Mask */

#define MXC_F_SPI_CTRL2_CLKPOL_POS                     1 /**< CTRL2_CLKPOL Position */
#define MXC_F_SPI_CTRL2_CLKPOL                         ((uint32_t)(0x1UL << MXC_F_SPI_CTRL2_CLKPOL_POS)) /**< CTRL2_CLKPOL Mask */

#define MXC_F_SPI_CTRL2_SCLK_FB_INV_POS                4 /**< CTRL2_SCLK_FB_INV Position */
#define MXC_F_SPI_CTRL2_SCLK_FB_INV                    ((uint32_t)(0x1UL << MXC_F_SPI_CTRL2_SCLK_FB_INV_POS)) /**< CTRL2_SCLK_FB_INV Mask */

#define MXC_F_SPI_CTRL2_NUMBITS_POS                    8 /**< CTRL2_NUMBITS Position */
#define MXC_F_SPI_CTRL2_NUMBITS                        ((uint32_t)(0xFUL << MXC_F_SPI_CTRL2_NUMBITS_POS)) /**< CTRL2_NUMBITS Mask */
#define MXC_V_SPI_CTRL2_NUMBITS_16                     ((uint32_t)0x0UL) /**< CTRL2_NUMBITS_16 Value */
#define MXC_S_SPI_CTRL2_NUMBITS_16                     (MXC_V_SPI_CTRL2_NUMBITS_16 << MXC_F_SPI_CTRL2_NUMBITS_POS) /**< CTRL2_NUMBITS_16 Setting */
#define MXC_V_SPI_CTRL2_NUMBITS_1                      ((uint32_t)0x1UL) /**< CTRL2_NUMBITS_1 Value */
#define MXC_S_SPI_CTRL2_NUMBITS_1                      (MXC_V_SPI_CTRL2_NUMBITS_1 << MXC_F_SPI_CTRL2_NUMBITS_POS) /**< CTRL2_NUMBITS_1 Setting */
#define MXC_V_SPI_CTRL2_NUMBITS_2                      ((uint32_t)0x2UL) /**< CTRL2_NUMBITS_2 Value */
#define MXC_S_SPI_CTRL2_NUMBITS_2                      (MXC_V_SPI_CTRL2_NUMBITS_2 << MXC_F_SPI_CTRL2_NUMBITS_POS) /**< CTRL2_NUMBITS_2 Setting */
#define MXC_V_SPI_CTRL2_NUMBITS_3                      ((uint32_t)0x3UL) /**< CTRL2_NUMBITS_3 Value */
#define MXC_S_SPI_CTRL2_NUMBITS_3                      (MXC_V_SPI_CTRL2_NUMBITS_3 << MXC_F_SPI_CTRL2_NUMBITS_POS) /**< CTRL2_NUMBITS_3 Setting */
#define MXC_V_SPI_CTRL2_NUMBITS_4                      ((uint32_t)0x4UL) /**< CTRL2_NUMBITS_4 Value */
#define MXC_S_SPI_CTRL2_NUMBITS_4                      (MXC_V_SPI_CTRL2_NUMBITS_4 << MXC_F_SPI_CTRL2_NUMBITS_POS) /**< CTRL2_NUMBITS_4 Setting */
#define MXC_V_SPI_CTRL2_NUMBITS_5                      ((uint32_t)0x5UL) /**< CTRL2_NUMBITS_5 Value */
#define MXC_S_SPI_CTRL2_NUMBITS_5                      (MXC_V_SPI_CTRL2_NUMBITS_5 << MXC_F_SPI_CTRL2_NUMBITS_POS) /**< CTRL2_NUMBITS_5 Setting */
#define MXC_V_SPI_CTRL2_NUMBITS_6                      ((uint32_t)0x6UL) /**< CTRL2_NUMBITS_6 Value */
#define MXC_S_SPI_CTRL2_NUMBITS_6                      (MXC_V_SPI_CTRL2_NUMBITS_6 << MXC_F_SPI_CTRL2_NUMBITS_POS) /**< CTRL2_NUMBITS_6 Setting */
#define MXC_V_SPI_CTRL2_NUMBITS_7                      ((uint32_t)0x7UL) /**< CTRL2_NUMBITS_7 Value */
#define MXC_S_SPI_CTRL2_NUMBITS_7                      (MXC_V_SPI_CTRL2_NUMBITS_7 << MXC_F_SPI_CTRL2_NUMBITS_POS) /**< CTRL2_NUMBITS_7 Setting */
#define MXC_V_SPI_CTRL2_NUMBITS_8                      ((uint32_t)0x8UL) /**< CTRL2_NUMBITS_8 Value */
#define MXC_S_SPI_CTRL2_NUMBITS_8                      (MXC_V_SPI_CTRL2_NUMBITS_8 << MXC_F_SPI_CTRL2_NUMBITS_POS) /**< CTRL2_NUMBITS_8 Setting */
#define MXC_V_SPI_CTRL2_NUMBITS_9                      ((uint32_t)0x9UL) /**< CTRL2_NUMBITS_9 Value */
#define MXC_S_SPI_CTRL2_NUMBITS_9                      (MXC_V_SPI_CTRL2_NUMBITS_9 << MXC_F_SPI_CTRL2_NUMBITS_POS) /**< CTRL2_NUMBITS_9 Setting */
#define MXC_V_SPI_CTRL2_NUMBITS_10                     ((uint32_t)0xAUL) /**< CTRL2_NUMBITS_10 Value */
#define MXC_S_SPI_CTRL2_NUMBITS_10                     (MXC_V_SPI_CTRL2_NUMBITS_10 << MXC_F_SPI_CTRL2_NUMBITS_POS) /**< CTRL2_NUMBITS_10 Setting */
#define MXC_V_SPI_CTRL2_NUMBITS_11                     ((uint32_t)0xBUL) /**< CTRL2_NUMBITS_11 Value */
#define MXC_S_SPI_CTRL2_NUMBITS_11                     (MXC_V_SPI_CTRL2_NUMBITS_11 << MXC_F_SPI_CTRL2_NUMBITS_POS) /**< CTRL2_NUMBITS_11 Setting */
#define MXC_V_SPI_CTRL2_NUMBITS_12                     ((uint32_t)0xCUL) /**< CTRL2_NUMBITS_12 Value */
#define MXC_S_SPI_CTRL2_NUMBITS_12                     (MXC_V_SPI_CTRL2_NUMBITS_12 << MXC_F_SPI_CTRL2_NUMBITS_POS) /**< CTRL2_NUMBITS_12 Setting */
#define MXC_V_SPI_CTRL2_NUMBITS_13                     ((uint32_t)0xDUL) /**< CTRL2_NUMBITS_13 Value */
#define MXC_S_SPI_CTRL2_NUMBITS_13                     (MXC_V_SPI_CTRL2_NUMBITS_13 << MXC_F_SPI_CTRL2_NUMBITS_POS) /**< CTRL2_NUMBITS_13 Setting */
#define MXC_V_SPI_CTRL2_NUMBITS_14                     ((uint32_t)0xEUL) /**< CTRL2_NUMBITS_14 Value */
#define MXC_S_SPI_CTRL2_NUMBITS_14                     (MXC_V_SPI_CTRL2_NUMBITS_14 << MXC_F_SPI_CTRL2_NUMBITS_POS) /**< CTRL2_NUMBITS_14 Setting */
#define MXC_V_SPI_CTRL2_NUMBITS_15                     ((uint32_t)0xFUL) /**< CTRL2_NUMBITS_15 Value */
#define MXC_S_SPI_CTRL2_NUMBITS_15                     (MXC_V_SPI_CTRL2_NUMBITS_15 << MXC_F_SPI_CTRL2_NUMBITS_POS) /**< CTRL2_NUMBITS_15 Setting */

#define MXC_F_SPI_CTRL2_DATA_WIDTH_POS                 12 /**< CTRL2_DATA_WIDTH Position */
#define MXC_F_SPI_CTRL2_DATA_WIDTH                     ((uint32_t)(0x3UL << MXC_F_SPI_CTRL2_DATA_WIDTH_POS)) /**< CTRL2_DATA_WIDTH Mask */
#define MXC_V_SPI_CTRL2_DATA_WIDTH_MONO                ((uint32_t)0x0UL) /**< CTRL2_DATA_WIDTH_MONO Value */
#define MXC_S_SPI_CTRL2_DATA_WIDTH_MONO                (MXC_V_SPI_CTRL2_DATA_WIDTH_MONO << MXC_F_SPI_CTRL2_DATA_WIDTH_POS) /**< CTRL2_DATA_WIDTH_MONO Setting */
#define MXC_V_SPI_CTRL2_DATA_WIDTH_DUAL                ((uint32_t)0x1UL) /**< CTRL2_DATA_WIDTH_DUAL Value */
#define MXC_S_SPI_CTRL2_DATA_WIDTH_DUAL                (MXC_V_SPI_CTRL2_DATA_WIDTH_DUAL << MXC_F_SPI_CTRL2_DATA_WIDTH_POS) /**< CTRL2_DATA_WIDTH_DUAL Setting */
#define MXC_V_SPI_CTRL2_DATA_WIDTH_QUAD                ((uint32_t)0x2UL) /**< CTRL2_DATA_WIDTH_QUAD Value */
#define MXC_S_SPI_CTRL2_DATA_WIDTH_QUAD                (MXC_V_SPI_CTRL2_DATA_WIDTH_QUAD << MXC_F_SPI_CTRL2_DATA_WIDTH_POS) /**< CTRL2_DATA_WIDTH_QUAD Setting */

#define MXC_F_SPI_CTRL2_THREE_WIRE_POS                 15 /**< CTRL2_THREE_WIRE Position */
#define MXC_F_SPI_CTRL2_THREE_WIRE                     ((uint32_t)(0x1UL << MXC_F_SPI_CTRL2_THREE_WIRE_POS)) /**< CTRL2_THREE_WIRE Mask */

#define MXC_F_SPI_CTRL2_TSPOL_POS                      16 /**< CTRL2_TSPOL Position */
#define MXC_F_SPI_CTRL2_TSPOL                          ((uint32_t)(0x7UL << MXC_F_SPI_CTRL2_TSPOL_POS)) /**< CTRL2_TSPOL Mask */
#define MXC_V_SPI_CTRL2_TSPOL_TS0_HIGH                 ((uint32_t)0x1UL) /**< CTRL2_TSPOL_TS0_HIGH Value */
#define MXC_S_SPI_CTRL2_TSPOL_TS0_HIGH                 (MXC_V_SPI_CTRL2_TSPOL_TS0_HIGH << MXC_F_SPI_CTRL2_TSPOL_POS) /**< CTRL2_TSPOL_TS0_HIGH Setting */
#define MXC_V_SPI_CTRL2_TSPOL_TS1_HIGH                 ((uint32_t)0x2UL) /**< CTRL2_TSPOL_TS1_HIGH Value */
#define MXC_S_SPI_CTRL2_TSPOL_TS1_HIGH                 (MXC_V_SPI_CTRL2_TSPOL_TS1_HIGH << MXC_F_SPI_CTRL2_TSPOL_POS) /**< CTRL2_TSPOL_TS1_HIGH Setting */
#define MXC_V_SPI_CTRL2_TSPOL_TS2_HIGH                 ((uint32_t)0x4UL) /**< CTRL2_TSPOL_TS2_HIGH Value */
#define MXC_S_SPI_CTRL2_TSPOL_TS2_HIGH                 (MXC_V_SPI_CTRL2_TSPOL_TS2_HIGH << MXC_F_SPI_CTRL2_TSPOL_POS) /**< CTRL2_TSPOL_TS2_HIGH Setting */

/**@} end of group SPI_CTRL2_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_TSTIME SPI_TSTIME
 * @brief    Register for controlling SPI peripheral/Target Select Timing.
 * @{
 */
#define MXC_F_SPI_TSTIME_PRE_POS                       0 /**< TSTIME_PRE Position */
#define MXC_F_SPI_TSTIME_PRE                           ((uint32_t)(0xFFUL << MXC_F_SPI_TSTIME_PRE_POS)) /**< TSTIME_PRE Mask */
#define MXC_V_SPI_TSTIME_PRE_256                       ((uint32_t)0x0UL) /**< TSTIME_PRE_256 Value */
#define MXC_S_SPI_TSTIME_PRE_256                       (MXC_V_SPI_TSTIME_PRE_256 << MXC_F_SPI_TSTIME_PRE_POS) /**< TSTIME_PRE_256 Setting */

#define MXC_F_SPI_TSTIME_POST_POS                      8 /**< TSTIME_POST Position */
#define MXC_F_SPI_TSTIME_POST                          ((uint32_t)(0xFFUL << MXC_F_SPI_TSTIME_POST_POS)) /**< TSTIME_POST Mask */
#define MXC_V_SPI_TSTIME_POST_256                      ((uint32_t)0x0UL) /**< TSTIME_POST_256 Value */
#define MXC_S_SPI_TSTIME_POST_256                      (MXC_V_SPI_TSTIME_POST_256 << MXC_F_SPI_TSTIME_POST_POS) /**< TSTIME_POST_256 Setting */

#define MXC_F_SPI_TSTIME_INACT_POS                     16 /**< TSTIME_INACT Position */
#define MXC_F_SPI_TSTIME_INACT                         ((uint32_t)(0xFFUL << MXC_F_SPI_TSTIME_INACT_POS)) /**< TSTIME_INACT Mask */
#define MXC_V_SPI_TSTIME_INACT_256                     ((uint32_t)0x0UL) /**< TSTIME_INACT_256 Value */
#define MXC_S_SPI_TSTIME_INACT_256                     (MXC_V_SPI_TSTIME_INACT_256 << MXC_F_SPI_TSTIME_INACT_POS) /**< TSTIME_INACT_256 Setting */

/**@} end of group SPI_TSTIME_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_CLKCTRL SPI_CLKCTRL
 * @brief    Register for controlling SPI clock rate.
 * @{
 */
#define MXC_F_SPI_CLKCTRL_LO_POS                       0 /**< CLKCTRL_LO Position */
#define MXC_F_SPI_CLKCTRL_LO                           ((uint32_t)(0xFFUL << MXC_F_SPI_CLKCTRL_LO_POS)) /**< CLKCTRL_LO Mask */
#define MXC_V_SPI_CLKCTRL_LO_DIS                       ((uint32_t)0x0UL) /**< CLKCTRL_LO_DIS Value */
#define MXC_S_SPI_CLKCTRL_LO_DIS                       (MXC_V_SPI_CLKCTRL_LO_DIS << MXC_F_SPI_CLKCTRL_LO_POS) /**< CLKCTRL_LO_DIS Setting */

#define MXC_F_SPI_CLKCTRL_HI_POS                       8 /**< CLKCTRL_HI Position */
#define MXC_F_SPI_CLKCTRL_HI                           ((uint32_t)(0xFFUL << MXC_F_SPI_CLKCTRL_HI_POS)) /**< CLKCTRL_HI Mask */
#define MXC_V_SPI_CLKCTRL_HI_DIS                       ((uint32_t)0x0UL) /**< CLKCTRL_HI_DIS Value */
#define MXC_S_SPI_CLKCTRL_HI_DIS                       (MXC_V_SPI_CLKCTRL_HI_DIS << MXC_F_SPI_CLKCTRL_HI_POS) /**< CLKCTRL_HI_DIS Setting */

#define MXC_F_SPI_CLKCTRL_CLKDIV_POS                   16 /**< CLKCTRL_CLKDIV Position */
#define MXC_F_SPI_CLKCTRL_CLKDIV                       ((uint32_t)(0xFUL << MXC_F_SPI_CLKCTRL_CLKDIV_POS)) /**< CLKCTRL_CLKDIV Mask */

/**@} end of group SPI_CLKCTRL_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_DMA SPI_DMA
 * @brief    Register for controlling DMA.
 * @{
 */
#define MXC_F_SPI_DMA_TX_THD_VAL_POS                   0 /**< DMA_TX_THD_VAL Position */
#define MXC_F_SPI_DMA_TX_THD_VAL                       ((uint32_t)(0x1FUL << MXC_F_SPI_DMA_TX_THD_VAL_POS)) /**< DMA_TX_THD_VAL Mask */

#define MXC_F_SPI_DMA_TX_FIFO_EN_POS                   6 /**< DMA_TX_FIFO_EN Position */
#define MXC_F_SPI_DMA_TX_FIFO_EN                       ((uint32_t)(0x1UL << MXC_F_SPI_DMA_TX_FIFO_EN_POS)) /**< DMA_TX_FIFO_EN Mask */

#define MXC_F_SPI_DMA_TX_FLUSH_POS                     7 /**< DMA_TX_FLUSH Position */
#define MXC_F_SPI_DMA_TX_FLUSH                         ((uint32_t)(0x1UL << MXC_F_SPI_DMA_TX_FLUSH_POS)) /**< DMA_TX_FLUSH Mask */

#define MXC_F_SPI_DMA_TX_LVL_POS                       8 /**< DMA_TX_LVL Position */
#define MXC_F_SPI_DMA_TX_LVL                           ((uint32_t)(0x3FUL << MXC_F_SPI_DMA_TX_LVL_POS)) /**< DMA_TX_LVL Mask */

#define MXC_F_SPI_DMA_TX_EN_POS                        15 /**< DMA_TX_EN Position */
#define MXC_F_SPI_DMA_TX_EN                            ((uint32_t)(0x1UL << MXC_F_SPI_DMA_TX_EN_POS)) /**< DMA_TX_EN Mask */

#define MXC_F_SPI_DMA_RX_THD_VAL_POS                   16 /**< DMA_RX_THD_VAL Position */
#define MXC_F_SPI_DMA_RX_THD_VAL                       ((uint32_t)(0x1FUL << MXC_F_SPI_DMA_RX_THD_VAL_POS)) /**< DMA_RX_THD_VAL Mask */

#define MXC_F_SPI_DMA_RX_FIFO_EN_POS                   22 /**< DMA_RX_FIFO_EN Position */
#define MXC_F_SPI_DMA_RX_FIFO_EN                       ((uint32_t)(0x1UL << MXC_F_SPI_DMA_RX_FIFO_EN_POS)) /**< DMA_RX_FIFO_EN Mask */

#define MXC_F_SPI_DMA_RX_FLUSH_POS                     23 /**< DMA_RX_FLUSH Position */
#define MXC_F_SPI_DMA_RX_FLUSH                         ((uint32_t)(0x1UL << MXC_F_SPI_DMA_RX_FLUSH_POS)) /**< DMA_RX_FLUSH Mask */

#define MXC_F_SPI_DMA_RX_LVL_POS                       24 /**< DMA_RX_LVL Position */
#define MXC_F_SPI_DMA_RX_LVL                           ((uint32_t)(0x3FUL << MXC_F_SPI_DMA_RX_LVL_POS)) /**< DMA_RX_LVL Mask */

#define MXC_F_SPI_DMA_RX_EN_POS                        31 /**< DMA_RX_EN Position */
#define MXC_F_SPI_DMA_RX_EN                            ((uint32_t)(0x1UL << MXC_F_SPI_DMA_RX_EN_POS)) /**< DMA_RX_EN Mask */

/**@} end of group SPI_DMA_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_INTFL SPI_INTFL
 * @brief    Register for reading and clearing interrupt flags. All bits are write 1 to
 *           clear.
 * @{
 */
#define MXC_F_SPI_INTFL_TX_THD_POS                     0 /**< INTFL_TX_THD Position */
#define MXC_F_SPI_INTFL_TX_THD                         ((uint32_t)(0x1UL << MXC_F_SPI_INTFL_TX_THD_POS)) /**< INTFL_TX_THD Mask */

#define MXC_F_SPI_INTFL_TX_EM_POS                      1 /**< INTFL_TX_EM Position */
#define MXC_F_SPI_INTFL_TX_EM                          ((uint32_t)(0x1UL << MXC_F_SPI_INTFL_TX_EM_POS)) /**< INTFL_TX_EM Mask */

#define MXC_F_SPI_INTFL_RX_THD_POS                     2 /**< INTFL_RX_THD Position */
#define MXC_F_SPI_INTFL_RX_THD                         ((uint32_t)(0x1UL << MXC_F_SPI_INTFL_RX_THD_POS)) /**< INTFL_RX_THD Mask */

#define MXC_F_SPI_INTFL_RX_FULL_POS                    3 /**< INTFL_RX_FULL Position */
#define MXC_F_SPI_INTFL_RX_FULL                        ((uint32_t)(0x1UL << MXC_F_SPI_INTFL_RX_FULL_POS)) /**< INTFL_RX_FULL Mask */

#define MXC_F_SPI_INTFL_TSA_POS                        4 /**< INTFL_TSA Position */
#define MXC_F_SPI_INTFL_TSA                            ((uint32_t)(0x1UL << MXC_F_SPI_INTFL_TSA_POS)) /**< INTFL_TSA Mask */

#define MXC_F_SPI_INTFL_TSD_POS                        5 /**< INTFL_TSD Position */
#define MXC_F_SPI_INTFL_TSD                            ((uint32_t)(0x1UL << MXC_F_SPI_INTFL_TSD_POS)) /**< INTFL_TSD Mask */

#define MXC_F_SPI_INTFL_FAULT_POS                      8 /**< INTFL_FAULT Position */
#define MXC_F_SPI_INTFL_FAULT                          ((uint32_t)(0x1UL << MXC_F_SPI_INTFL_FAULT_POS)) /**< INTFL_FAULT Mask */

#define MXC_F_SPI_INTFL_ABORT_POS                      9 /**< INTFL_ABORT Position */
#define MXC_F_SPI_INTFL_ABORT                          ((uint32_t)(0x1UL << MXC_F_SPI_INTFL_ABORT_POS)) /**< INTFL_ABORT Mask */

#define MXC_F_SPI_INTFL_CONT_DONE_POS                  11 /**< INTFL_CONT_DONE Position */
#define MXC_F_SPI_INTFL_CONT_DONE                      ((uint32_t)(0x1UL << MXC_F_SPI_INTFL_CONT_DONE_POS)) /**< INTFL_CONT_DONE Mask */

#define MXC_F_SPI_INTFL_TX_OV_POS                      12 /**< INTFL_TX_OV Position */
#define MXC_F_SPI_INTFL_TX_OV                          ((uint32_t)(0x1UL << MXC_F_SPI_INTFL_TX_OV_POS)) /**< INTFL_TX_OV Mask */

#define MXC_F_SPI_INTFL_TX_UN_POS                      13 /**< INTFL_TX_UN Position */
#define MXC_F_SPI_INTFL_TX_UN                          ((uint32_t)(0x1UL << MXC_F_SPI_INTFL_TX_UN_POS)) /**< INTFL_TX_UN Mask */

#define MXC_F_SPI_INTFL_RX_OV_POS                      14 /**< INTFL_RX_OV Position */
#define MXC_F_SPI_INTFL_RX_OV                          ((uint32_t)(0x1UL << MXC_F_SPI_INTFL_RX_OV_POS)) /**< INTFL_RX_OV Mask */

#define MXC_F_SPI_INTFL_RX_UN_POS                      15 /**< INTFL_RX_UN Position */
#define MXC_F_SPI_INTFL_RX_UN                          ((uint32_t)(0x1UL << MXC_F_SPI_INTFL_RX_UN_POS)) /**< INTFL_RX_UN Mask */

/**@} end of group SPI_INTFL_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_INTEN SPI_INTEN
 * @brief    Register for enabling interrupts.
 * @{
 */
#define MXC_F_SPI_INTEN_TX_THD_POS                     0 /**< INTEN_TX_THD Position */
#define MXC_F_SPI_INTEN_TX_THD                         ((uint32_t)(0x1UL << MXC_F_SPI_INTEN_TX_THD_POS)) /**< INTEN_TX_THD Mask */

#define MXC_F_SPI_INTEN_TX_EM_POS                      1 /**< INTEN_TX_EM Position */
#define MXC_F_SPI_INTEN_TX_EM                          ((uint32_t)(0x1UL << MXC_F_SPI_INTEN_TX_EM_POS)) /**< INTEN_TX_EM Mask */

#define MXC_F_SPI_INTEN_RX_THD_POS                     2 /**< INTEN_RX_THD Position */
#define MXC_F_SPI_INTEN_RX_THD                         ((uint32_t)(0x1UL << MXC_F_SPI_INTEN_RX_THD_POS)) /**< INTEN_RX_THD Mask */

#define MXC_F_SPI_INTEN_RX_FULL_POS                    3 /**< INTEN_RX_FULL Position */
#define MXC_F_SPI_INTEN_RX_FULL                        ((uint32_t)(0x1UL << MXC_F_SPI_INTEN_RX_FULL_POS)) /**< INTEN_RX_FULL Mask */

#define MXC_F_SPI_INTEN_TSA_POS                        4 /**< INTEN_TSA Position */
#define MXC_F_SPI_INTEN_TSA                            ((uint32_t)(0x1UL << MXC_F_SPI_INTEN_TSA_POS)) /**< INTEN_TSA Mask */

#define MXC_F_SPI_INTEN_TSD_POS                        5 /**< INTEN_TSD Position */
#define MXC_F_SPI_INTEN_TSD                            ((uint32_t)(0x1UL << MXC_F_SPI_INTEN_TSD_POS)) /**< INTEN_TSD Mask */

#define MXC_F_SPI_INTEN_FAULT_POS                      8 /**< INTEN_FAULT Position */
#define MXC_F_SPI_INTEN_FAULT                          ((uint32_t)(0x1UL << MXC_F_SPI_INTEN_FAULT_POS)) /**< INTEN_FAULT Mask */

#define MXC_F_SPI_INTEN_ABORT_POS                      9 /**< INTEN_ABORT Position */
#define MXC_F_SPI_INTEN_ABORT                          ((uint32_t)(0x1UL << MXC_F_SPI_INTEN_ABORT_POS)) /**< INTEN_ABORT Mask */

#define MXC_F_SPI_INTEN_CONT_DONE_POS                  11 /**< INTEN_CONT_DONE Position */
#define MXC_F_SPI_INTEN_CONT_DONE                      ((uint32_t)(0x1UL << MXC_F_SPI_INTEN_CONT_DONE_POS)) /**< INTEN_CONT_DONE Mask */

#define MXC_F_SPI_INTEN_TX_OV_POS                      12 /**< INTEN_TX_OV Position */
#define MXC_F_SPI_INTEN_TX_OV                          ((uint32_t)(0x1UL << MXC_F_SPI_INTEN_TX_OV_POS)) /**< INTEN_TX_OV Mask */

#define MXC_F_SPI_INTEN_TX_UN_POS                      13 /**< INTEN_TX_UN Position */
#define MXC_F_SPI_INTEN_TX_UN                          ((uint32_t)(0x1UL << MXC_F_SPI_INTEN_TX_UN_POS)) /**< INTEN_TX_UN Mask */

#define MXC_F_SPI_INTEN_RX_OV_POS                      14 /**< INTEN_RX_OV Position */
#define MXC_F_SPI_INTEN_RX_OV                          ((uint32_t)(0x1UL << MXC_F_SPI_INTEN_RX_OV_POS)) /**< INTEN_RX_OV Mask */

#define MXC_F_SPI_INTEN_RX_UN_POS                      15 /**< INTEN_RX_UN Position */
#define MXC_F_SPI_INTEN_RX_UN                          ((uint32_t)(0x1UL << MXC_F_SPI_INTEN_RX_UN_POS)) /**< INTEN_RX_UN Mask */

/**@} end of group SPI_INTEN_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_WKFL SPI_WKFL
 * @brief    Register for wake up flags. All bits in this register are write 1 to clear.
 * @{
 */
#define MXC_F_SPI_WKFL_TX_THD_POS                      0 /**< WKFL_TX_THD Position */
#define MXC_F_SPI_WKFL_TX_THD                          ((uint32_t)(0x1UL << MXC_F_SPI_WKFL_TX_THD_POS)) /**< WKFL_TX_THD Mask */

#define MXC_F_SPI_WKFL_TX_EM_POS                       1 /**< WKFL_TX_EM Position */
#define MXC_F_SPI_WKFL_TX_EM                           ((uint32_t)(0x1UL << MXC_F_SPI_WKFL_TX_EM_POS)) /**< WKFL_TX_EM Mask */

#define MXC_F_SPI_WKFL_RX_THD_POS                      2 /**< WKFL_RX_THD Position */
#define MXC_F_SPI_WKFL_RX_THD                          ((uint32_t)(0x1UL << MXC_F_SPI_WKFL_RX_THD_POS)) /**< WKFL_RX_THD Mask */

#define MXC_F_SPI_WKFL_RX_FULL_POS                     3 /**< WKFL_RX_FULL Position */
#define MXC_F_SPI_WKFL_RX_FULL                         ((uint32_t)(0x1UL << MXC_F_SPI_WKFL_RX_FULL_POS)) /**< WKFL_RX_FULL Mask */

/**@} end of group SPI_WKFL_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_WKEN SPI_WKEN
 * @brief    Register for wake up enable.
 * @{
 */
#define MXC_F_SPI_WKEN_TX_THD_POS                      0 /**< WKEN_TX_THD Position */
#define MXC_F_SPI_WKEN_TX_THD                          ((uint32_t)(0x1UL << MXC_F_SPI_WKEN_TX_THD_POS)) /**< WKEN_TX_THD Mask */

#define MXC_F_SPI_WKEN_TX_EM_POS                       1 /**< WKEN_TX_EM Position */
#define MXC_F_SPI_WKEN_TX_EM                           ((uint32_t)(0x1UL << MXC_F_SPI_WKEN_TX_EM_POS)) /**< WKEN_TX_EM Mask */

#define MXC_F_SPI_WKEN_RX_THD_POS                      2 /**< WKEN_RX_THD Position */
#define MXC_F_SPI_WKEN_RX_THD                          ((uint32_t)(0x1UL << MXC_F_SPI_WKEN_RX_THD_POS)) /**< WKEN_RX_THD Mask */

#define MXC_F_SPI_WKEN_RX_FULL_POS                     3 /**< WKEN_RX_FULL Position */
#define MXC_F_SPI_WKEN_RX_FULL                         ((uint32_t)(0x1UL << MXC_F_SPI_WKEN_RX_FULL_POS)) /**< WKEN_RX_FULL Mask */

/**@} end of group SPI_WKEN_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_STATUS SPI_STATUS
 * @brief    SPI Status register.
 * @{
 */
#define MXC_F_SPI_STATUS_BUSY_POS                      0 /**< STATUS_BUSY Position */
#define MXC_F_SPI_STATUS_BUSY                          ((uint32_t)(0x1UL << MXC_F_SPI_STATUS_BUSY_POS)) /**< STATUS_BUSY Mask */

/**@} end of group SPI_STATUS_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_SPI_REGS_H_
