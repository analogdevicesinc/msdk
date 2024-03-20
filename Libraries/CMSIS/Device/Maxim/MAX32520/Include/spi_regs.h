/**
 * @file    spi_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SPI Peripheral Module.
 * @note    This file is @generated.
 * @ingroup spi_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32520_INCLUDE_SPI_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32520_INCLUDE_SPI_REGS_H_

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
        __IO uint32_t data32;           /**< <tt>\b 0x00:</tt> SPI DATA32 Register */
        __IO uint16_t data16[2];        /**< <tt>\b 0x00:</tt> SPI DATA16 Register */
        __IO uint8_t  data8[4];         /**< <tt>\b 0x00:</tt> SPI DATA8 Register */
    };
    __IO uint32_t mstr_cntl;            /**< <tt>\b 0x04:</tt> SPI MSTR_CNTL Register */
    __IO uint32_t trnmt_size;           /**< <tt>\b 0x08:</tt> SPI TRNMT_SIZE Register */
    __IO uint32_t static_config;        /**< <tt>\b 0x0C:</tt> SPI STATIC_CONFIG Register */
    __IO uint32_t ss_time;              /**< <tt>\b 0x10:</tt> SPI SS_TIME Register */
    __IO uint32_t clk_config;           /**< <tt>\b 0x14:</tt> SPI CLK_CONFIG Register */
    __R  uint32_t rsv_0x18;
    __IO uint32_t dma;                  /**< <tt>\b 0x1C:</tt> SPI DMA Register */
    __IO uint32_t int_fl;               /**< <tt>\b 0x20:</tt> SPI INT_FL Register */
    __IO uint32_t int_en;               /**< <tt>\b 0x24:</tt> SPI INT_EN Register */
    __IO uint32_t wake_fl;              /**< <tt>\b 0x28:</tt> SPI WAKE_FL Register */
    __IO uint32_t wake_en;              /**< <tt>\b 0x2C:</tt> SPI WAKE_EN Register */
    __I  uint32_t stat;                 /**< <tt>\b 0x30:</tt> SPI STAT Register */
} mxc_spi_regs_t;

/* Register offsets for module SPI */
/**
 * @ingroup    spi_registers
 * @defgroup   SPI_Register_Offsets Register Offsets
 * @brief      SPI Peripheral Register Offsets from the SPI Base Peripheral Address.
 * @{
 */
#define MXC_R_SPI_DATA32                   ((uint32_t)0x00000000UL) /**< Offset from SPI Base Address: <tt> 0x0000</tt> */
#define MXC_R_SPI_DATA16                   ((uint32_t)0x00000000UL) /**< Offset from SPI Base Address: <tt> 0x0000</tt> */
#define MXC_R_SPI_DATA8                    ((uint32_t)0x00000000UL) /**< Offset from SPI Base Address: <tt> 0x0000</tt> */
#define MXC_R_SPI_MSTR_CNTL                ((uint32_t)0x00000004UL) /**< Offset from SPI Base Address: <tt> 0x0004</tt> */
#define MXC_R_SPI_TRNMT_SIZE               ((uint32_t)0x00000008UL) /**< Offset from SPI Base Address: <tt> 0x0008</tt> */
#define MXC_R_SPI_STATIC_CONFIG            ((uint32_t)0x0000000CUL) /**< Offset from SPI Base Address: <tt> 0x000C</tt> */
#define MXC_R_SPI_SS_TIME                  ((uint32_t)0x00000010UL) /**< Offset from SPI Base Address: <tt> 0x0010</tt> */
#define MXC_R_SPI_CLK_CONFIG               ((uint32_t)0x00000014UL) /**< Offset from SPI Base Address: <tt> 0x0014</tt> */
#define MXC_R_SPI_DMA                      ((uint32_t)0x0000001CUL) /**< Offset from SPI Base Address: <tt> 0x001C</tt> */
#define MXC_R_SPI_INT_FL                   ((uint32_t)0x00000020UL) /**< Offset from SPI Base Address: <tt> 0x0020</tt> */
#define MXC_R_SPI_INT_EN                   ((uint32_t)0x00000024UL) /**< Offset from SPI Base Address: <tt> 0x0024</tt> */
#define MXC_R_SPI_WAKE_FL                  ((uint32_t)0x00000028UL) /**< Offset from SPI Base Address: <tt> 0x0028</tt> */
#define MXC_R_SPI_WAKE_EN                  ((uint32_t)0x0000002CUL) /**< Offset from SPI Base Address: <tt> 0x002C</tt> */
#define MXC_R_SPI_STAT                     ((uint32_t)0x00000030UL) /**< Offset from SPI Base Address: <tt> 0x0030</tt> */
/**@} end of group spi_registers */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_DATA32 SPI_DATA32
 * @brief    Register for reading and writing the FIFO.
 * @{
 */
#define MXC_F_SPI_DATA32_DATA_POS                      0 /**< DATA32_DATA Position */
#define MXC_F_SPI_DATA32_DATA                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_SPI_DATA32_DATA_POS)) /**< DATA32_DATA Mask */

/**@} end of group SPI_DATA32_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_DATA16 SPI_DATA16
 * @brief    Register for reading and writing the FIFO.
 * @{
 */
#define MXC_F_SPI_DATA16_DATA_POS                      0 /**< DATA16_DATA Position */
#define MXC_F_SPI_DATA16_DATA                          ((uint16_t)(0xFFFFUL << MXC_F_SPI_DATA16_DATA_POS)) /**< DATA16_DATA Mask */

/**@} end of group SPI_DATA16_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_DATA8 SPI_DATA8
 * @brief    Register for reading and writing the FIFO.
 * @{
 */
#define MXC_F_SPI_DATA8_DATA_POS                       0 /**< DATA8_DATA Position */
#define MXC_F_SPI_DATA8_DATA                           ((uint8_t)(0xFFUL << MXC_F_SPI_DATA8_DATA_POS)) /**< DATA8_DATA Mask */

/**@} end of group SPI_DATA8_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_MSTR_CNTL SPI_MSTR_CNTL
 * @brief    Register for controlling SPI peripheral.
 * @{
 */
#define MXC_F_SPI_MSTR_CNTL_SPIEN_POS                  0 /**< MSTR_CNTL_SPIEN Position */
#define MXC_F_SPI_MSTR_CNTL_SPIEN                      ((uint32_t)(0x1UL << MXC_F_SPI_MSTR_CNTL_SPIEN_POS)) /**< MSTR_CNTL_SPIEN Mask */

#define MXC_F_SPI_MSTR_CNTL_MMEN_POS                   1 /**< MSTR_CNTL_MMEN Position */
#define MXC_F_SPI_MSTR_CNTL_MMEN                       ((uint32_t)(0x1UL << MXC_F_SPI_MSTR_CNTL_MMEN_POS)) /**< MSTR_CNTL_MMEN Mask */

#define MXC_F_SPI_MSTR_CNTL_SSIO_POS                   4 /**< MSTR_CNTL_SSIO Position */
#define MXC_F_SPI_MSTR_CNTL_SSIO                       ((uint32_t)(0x1UL << MXC_F_SPI_MSTR_CNTL_SSIO_POS)) /**< MSTR_CNTL_SSIO Mask */

#define MXC_F_SPI_MSTR_CNTL_START_POS                  5 /**< MSTR_CNTL_START Position */
#define MXC_F_SPI_MSTR_CNTL_START                      ((uint32_t)(0x1UL << MXC_F_SPI_MSTR_CNTL_START_POS)) /**< MSTR_CNTL_START Mask */

#define MXC_F_SPI_MSTR_CNTL_SSCTRL_POS                 8 /**< MSTR_CNTL_SSCTRL Position */
#define MXC_F_SPI_MSTR_CNTL_SSCTRL                     ((uint32_t)(0x1UL << MXC_F_SPI_MSTR_CNTL_SSCTRL_POS)) /**< MSTR_CNTL_SSCTRL Mask */

#define MXC_F_SPI_MSTR_CNTL_SS_POS                     16 /**< MSTR_CNTL_SS Position */
#define MXC_F_SPI_MSTR_CNTL_SS                         ((uint32_t)(0x7UL << MXC_F_SPI_MSTR_CNTL_SS_POS)) /**< MSTR_CNTL_SS Mask */
#define MXC_V_SPI_MSTR_CNTL_SS_SS0                     ((uint32_t)0x1UL) /**< MSTR_CNTL_SS_SS0 Value */
#define MXC_S_SPI_MSTR_CNTL_SS_SS0                     (MXC_V_SPI_MSTR_CNTL_SS_SS0 << MXC_F_SPI_MSTR_CNTL_SS_POS) /**< MSTR_CNTL_SS_SS0 Setting */
#define MXC_V_SPI_MSTR_CNTL_SS_SS1                     ((uint32_t)0x2UL) /**< MSTR_CNTL_SS_SS1 Value */
#define MXC_S_SPI_MSTR_CNTL_SS_SS1                     (MXC_V_SPI_MSTR_CNTL_SS_SS1 << MXC_F_SPI_MSTR_CNTL_SS_POS) /**< MSTR_CNTL_SS_SS1 Setting */
#define MXC_V_SPI_MSTR_CNTL_SS_SS2                     ((uint32_t)0x4UL) /**< MSTR_CNTL_SS_SS2 Value */
#define MXC_S_SPI_MSTR_CNTL_SS_SS2                     (MXC_V_SPI_MSTR_CNTL_SS_SS2 << MXC_F_SPI_MSTR_CNTL_SS_POS) /**< MSTR_CNTL_SS_SS2 Setting */

/**@} end of group SPI_MSTR_CNTL_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_TRNMT_SIZE SPI_TRNMT_SIZE
 * @brief    Register for controlling SPI peripheral.
 * @{
 */
#define MXC_F_SPI_TRNMT_SIZE_TX_NUM_CHAR_POS           0 /**< TRNMT_SIZE_TX_NUM_CHAR Position */
#define MXC_F_SPI_TRNMT_SIZE_TX_NUM_CHAR               ((uint32_t)(0xFFFFUL << MXC_F_SPI_TRNMT_SIZE_TX_NUM_CHAR_POS)) /**< TRNMT_SIZE_TX_NUM_CHAR Mask */

#define MXC_F_SPI_TRNMT_SIZE_RX_NUM_CHAR_POS           16 /**< TRNMT_SIZE_RX_NUM_CHAR Position */
#define MXC_F_SPI_TRNMT_SIZE_RX_NUM_CHAR               ((uint32_t)(0xFFFFUL << MXC_F_SPI_TRNMT_SIZE_RX_NUM_CHAR_POS)) /**< TRNMT_SIZE_RX_NUM_CHAR Mask */

/**@} end of group SPI_TRNMT_SIZE_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_STATIC_CONFIG SPI_STATIC_CONFIG
 * @brief    Register for controlling SPI peripheral.
 * @{
 */
#define MXC_F_SPI_STATIC_CONFIG_PHASE_POS              0 /**< STATIC_CONFIG_PHASE Position */
#define MXC_F_SPI_STATIC_CONFIG_PHASE                  ((uint32_t)(0x1UL << MXC_F_SPI_STATIC_CONFIG_PHASE_POS)) /**< STATIC_CONFIG_PHASE Mask */

#define MXC_F_SPI_STATIC_CONFIG_CLKPOL_POS             1 /**< STATIC_CONFIG_CLKPOL Position */
#define MXC_F_SPI_STATIC_CONFIG_CLKPOL                 ((uint32_t)(0x1UL << MXC_F_SPI_STATIC_CONFIG_CLKPOL_POS)) /**< STATIC_CONFIG_CLKPOL Mask */

#define MXC_F_SPI_STATIC_CONFIG_NUMBITS_POS            8 /**< STATIC_CONFIG_NUMBITS Position */
#define MXC_F_SPI_STATIC_CONFIG_NUMBITS                ((uint32_t)(0xFUL << MXC_F_SPI_STATIC_CONFIG_NUMBITS_POS)) /**< STATIC_CONFIG_NUMBITS Mask */
#define MXC_V_SPI_STATIC_CONFIG_NUMBITS_0              ((uint32_t)0x0UL) /**< STATIC_CONFIG_NUMBITS_0 Value */
#define MXC_S_SPI_STATIC_CONFIG_NUMBITS_0              (MXC_V_SPI_STATIC_CONFIG_NUMBITS_0 << MXC_F_SPI_STATIC_CONFIG_NUMBITS_POS) /**< STATIC_CONFIG_NUMBITS_0 Setting */

#define MXC_F_SPI_STATIC_CONFIG_DATAWIDTH_POS          12 /**< STATIC_CONFIG_DATAWIDTH Position */
#define MXC_F_SPI_STATIC_CONFIG_DATAWIDTH              ((uint32_t)(0x3UL << MXC_F_SPI_STATIC_CONFIG_DATAWIDTH_POS)) /**< STATIC_CONFIG_DATAWIDTH Mask */
#define MXC_V_SPI_STATIC_CONFIG_DATAWIDTH_MONO         ((uint32_t)0x0UL) /**< STATIC_CONFIG_DATAWIDTH_MONO Value */
#define MXC_S_SPI_STATIC_CONFIG_DATAWIDTH_MONO         (MXC_V_SPI_STATIC_CONFIG_DATAWIDTH_MONO << MXC_F_SPI_STATIC_CONFIG_DATAWIDTH_POS) /**< STATIC_CONFIG_DATAWIDTH_MONO Setting */
#define MXC_V_SPI_STATIC_CONFIG_DATAWIDTH_DUAL         ((uint32_t)0x1UL) /**< STATIC_CONFIG_DATAWIDTH_DUAL Value */
#define MXC_S_SPI_STATIC_CONFIG_DATAWIDTH_DUAL         (MXC_V_SPI_STATIC_CONFIG_DATAWIDTH_DUAL << MXC_F_SPI_STATIC_CONFIG_DATAWIDTH_POS) /**< STATIC_CONFIG_DATAWIDTH_DUAL Setting */
#define MXC_V_SPI_STATIC_CONFIG_DATAWIDTH_QUAD         ((uint32_t)0x2UL) /**< STATIC_CONFIG_DATAWIDTH_QUAD Value */
#define MXC_S_SPI_STATIC_CONFIG_DATAWIDTH_QUAD         (MXC_V_SPI_STATIC_CONFIG_DATAWIDTH_QUAD << MXC_F_SPI_STATIC_CONFIG_DATAWIDTH_POS) /**< STATIC_CONFIG_DATAWIDTH_QUAD Setting */

#define MXC_F_SPI_STATIC_CONFIG_3WIRE_POS              15 /**< STATIC_CONFIG_3WIRE Position */
#define MXC_F_SPI_STATIC_CONFIG_3WIRE                  ((uint32_t)(0x1UL << MXC_F_SPI_STATIC_CONFIG_3WIRE_POS)) /**< STATIC_CONFIG_3WIRE Mask */

#define MXC_F_SPI_STATIC_CONFIG_SSPOL_POS              16 /**< STATIC_CONFIG_SSPOL Position */
#define MXC_F_SPI_STATIC_CONFIG_SSPOL                  ((uint32_t)(0xFFUL << MXC_F_SPI_STATIC_CONFIG_SSPOL_POS)) /**< STATIC_CONFIG_SSPOL Mask */
#define MXC_V_SPI_STATIC_CONFIG_SSPOL_SS0_HIGH         ((uint32_t)0x1UL) /**< STATIC_CONFIG_SSPOL_SS0_HIGH Value */
#define MXC_S_SPI_STATIC_CONFIG_SSPOL_SS0_HIGH         (MXC_V_SPI_STATIC_CONFIG_SSPOL_SS0_HIGH << MXC_F_SPI_STATIC_CONFIG_SSPOL_POS) /**< STATIC_CONFIG_SSPOL_SS0_HIGH Setting */
#define MXC_V_SPI_STATIC_CONFIG_SSPOL_SS1_HIGH         ((uint32_t)0x2UL) /**< STATIC_CONFIG_SSPOL_SS1_HIGH Value */
#define MXC_S_SPI_STATIC_CONFIG_SSPOL_SS1_HIGH         (MXC_V_SPI_STATIC_CONFIG_SSPOL_SS1_HIGH << MXC_F_SPI_STATIC_CONFIG_SSPOL_POS) /**< STATIC_CONFIG_SSPOL_SS1_HIGH Setting */
#define MXC_V_SPI_STATIC_CONFIG_SSPOL_SS2_HIGH         ((uint32_t)0x4UL) /**< STATIC_CONFIG_SSPOL_SS2_HIGH Value */
#define MXC_S_SPI_STATIC_CONFIG_SSPOL_SS2_HIGH         (MXC_V_SPI_STATIC_CONFIG_SSPOL_SS2_HIGH << MXC_F_SPI_STATIC_CONFIG_SSPOL_POS) /**< STATIC_CONFIG_SSPOL_SS2_HIGH Setting */

/**@} end of group SPI_STATIC_CONFIG_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_SS_TIME SPI_SS_TIME
 * @brief    Register for controlling SPI peripheral/Slave Select Timing.
 * @{
 */
#define MXC_F_SPI_SS_TIME_PRE_POS                      0 /**< SS_TIME_PRE Position */
#define MXC_F_SPI_SS_TIME_PRE                          ((uint32_t)(0xFFUL << MXC_F_SPI_SS_TIME_PRE_POS)) /**< SS_TIME_PRE Mask */
#define MXC_V_SPI_SS_TIME_PRE_256                      ((uint32_t)0x0UL) /**< SS_TIME_PRE_256 Value */
#define MXC_S_SPI_SS_TIME_PRE_256                      (MXC_V_SPI_SS_TIME_PRE_256 << MXC_F_SPI_SS_TIME_PRE_POS) /**< SS_TIME_PRE_256 Setting */

#define MXC_F_SPI_SS_TIME_POST_POS                     8 /**< SS_TIME_POST Position */
#define MXC_F_SPI_SS_TIME_POST                         ((uint32_t)(0xFFUL << MXC_F_SPI_SS_TIME_POST_POS)) /**< SS_TIME_POST Mask */
#define MXC_V_SPI_SS_TIME_POST_256                     ((uint32_t)0x0UL) /**< SS_TIME_POST_256 Value */
#define MXC_S_SPI_SS_TIME_POST_256                     (MXC_V_SPI_SS_TIME_POST_256 << MXC_F_SPI_SS_TIME_POST_POS) /**< SS_TIME_POST_256 Setting */

#define MXC_F_SPI_SS_TIME_INACT_POS                    16 /**< SS_TIME_INACT Position */
#define MXC_F_SPI_SS_TIME_INACT                        ((uint32_t)(0xFFUL << MXC_F_SPI_SS_TIME_INACT_POS)) /**< SS_TIME_INACT Mask */
#define MXC_V_SPI_SS_TIME_INACT_256                    ((uint32_t)0x0UL) /**< SS_TIME_INACT_256 Value */
#define MXC_S_SPI_SS_TIME_INACT_256                    (MXC_V_SPI_SS_TIME_INACT_256 << MXC_F_SPI_SS_TIME_INACT_POS) /**< SS_TIME_INACT_256 Setting */

/**@} end of group SPI_SS_TIME_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_CLK_CONFIG SPI_CLK_CONFIG
 * @brief    Register for controlling SPI clock rate.
 * @{
 */
#define MXC_F_SPI_CLK_CONFIG_LOW_POS                   0 /**< CLK_CONFIG_LOW Position */
#define MXC_F_SPI_CLK_CONFIG_LOW                       ((uint32_t)(0xFFUL << MXC_F_SPI_CLK_CONFIG_LOW_POS)) /**< CLK_CONFIG_LOW Mask */
#define MXC_V_SPI_CLK_CONFIG_LOW_DIS                   ((uint32_t)0x0UL) /**< CLK_CONFIG_LOW_DIS Value */
#define MXC_S_SPI_CLK_CONFIG_LOW_DIS                   (MXC_V_SPI_CLK_CONFIG_LOW_DIS << MXC_F_SPI_CLK_CONFIG_LOW_POS) /**< CLK_CONFIG_LOW_DIS Setting */

#define MXC_F_SPI_CLK_CONFIG_HIGH_POS                  8 /**< CLK_CONFIG_HIGH Position */
#define MXC_F_SPI_CLK_CONFIG_HIGH                      ((uint32_t)(0xFFUL << MXC_F_SPI_CLK_CONFIG_HIGH_POS)) /**< CLK_CONFIG_HIGH Mask */
#define MXC_V_SPI_CLK_CONFIG_HIGH_DIS                  ((uint32_t)0x0UL) /**< CLK_CONFIG_HIGH_DIS Value */
#define MXC_S_SPI_CLK_CONFIG_HIGH_DIS                  (MXC_V_SPI_CLK_CONFIG_HIGH_DIS << MXC_F_SPI_CLK_CONFIG_HIGH_POS) /**< CLK_CONFIG_HIGH_DIS Setting */

#define MXC_F_SPI_CLK_CONFIG_SCALE_POS                 16 /**< CLK_CONFIG_SCALE Position */
#define MXC_F_SPI_CLK_CONFIG_SCALE                     ((uint32_t)(0xFUL << MXC_F_SPI_CLK_CONFIG_SCALE_POS)) /**< CLK_CONFIG_SCALE Mask */

/**@} end of group SPI_CLK_CONFIG_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_DMA SPI_DMA
 * @brief    Register for controlling DMA.
 * @{
 */
#define MXC_F_SPI_DMA_TX_FIFO_LEVEL_POS                0 /**< DMA_TX_FIFO_LEVEL Position */
#define MXC_F_SPI_DMA_TX_FIFO_LEVEL                    ((uint32_t)(0x1FUL << MXC_F_SPI_DMA_TX_FIFO_LEVEL_POS)) /**< DMA_TX_FIFO_LEVEL Mask */

#define MXC_F_SPI_DMA_TX_FIFO_EN_POS                   6 /**< DMA_TX_FIFO_EN Position */
#define MXC_F_SPI_DMA_TX_FIFO_EN                       ((uint32_t)(0x1UL << MXC_F_SPI_DMA_TX_FIFO_EN_POS)) /**< DMA_TX_FIFO_EN Mask */

#define MXC_F_SPI_DMA_TX_FIFO_CLEAR_POS                7 /**< DMA_TX_FIFO_CLEAR Position */
#define MXC_F_SPI_DMA_TX_FIFO_CLEAR                    ((uint32_t)(0x1UL << MXC_F_SPI_DMA_TX_FIFO_CLEAR_POS)) /**< DMA_TX_FIFO_CLEAR Mask */

#define MXC_F_SPI_DMA_TX_FIFO_CNT_POS                  8 /**< DMA_TX_FIFO_CNT Position */
#define MXC_F_SPI_DMA_TX_FIFO_CNT                      ((uint32_t)(0x3FUL << MXC_F_SPI_DMA_TX_FIFO_CNT_POS)) /**< DMA_TX_FIFO_CNT Mask */

#define MXC_F_SPI_DMA_TX_DMA_EN_POS                    15 /**< DMA_TX_DMA_EN Position */
#define MXC_F_SPI_DMA_TX_DMA_EN                        ((uint32_t)(0x1UL << MXC_F_SPI_DMA_TX_DMA_EN_POS)) /**< DMA_TX_DMA_EN Mask */

#define MXC_F_SPI_DMA_RX_FIFO_LEVEL_POS                16 /**< DMA_RX_FIFO_LEVEL Position */
#define MXC_F_SPI_DMA_RX_FIFO_LEVEL                    ((uint32_t)(0x1FUL << MXC_F_SPI_DMA_RX_FIFO_LEVEL_POS)) /**< DMA_RX_FIFO_LEVEL Mask */

#define MXC_F_SPI_DMA_RX_FIFO_EN_POS                   22 /**< DMA_RX_FIFO_EN Position */
#define MXC_F_SPI_DMA_RX_FIFO_EN                       ((uint32_t)(0x1UL << MXC_F_SPI_DMA_RX_FIFO_EN_POS)) /**< DMA_RX_FIFO_EN Mask */

#define MXC_F_SPI_DMA_RX_FIFO_CLEAR_POS                23 /**< DMA_RX_FIFO_CLEAR Position */
#define MXC_F_SPI_DMA_RX_FIFO_CLEAR                    ((uint32_t)(0x1UL << MXC_F_SPI_DMA_RX_FIFO_CLEAR_POS)) /**< DMA_RX_FIFO_CLEAR Mask */

#define MXC_F_SPI_DMA_RX_FIFO_CNT_POS                  24 /**< DMA_RX_FIFO_CNT Position */
#define MXC_F_SPI_DMA_RX_FIFO_CNT                      ((uint32_t)(0x3FUL << MXC_F_SPI_DMA_RX_FIFO_CNT_POS)) /**< DMA_RX_FIFO_CNT Mask */

#define MXC_F_SPI_DMA_RX_DMA_EN_POS                    31 /**< DMA_RX_DMA_EN Position */
#define MXC_F_SPI_DMA_RX_DMA_EN                        ((uint32_t)(0x1UL << MXC_F_SPI_DMA_RX_DMA_EN_POS)) /**< DMA_RX_DMA_EN Mask */

/**@} end of group SPI_DMA_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_INT_FL SPI_INT_FL
 * @brief    Register for reading and clearing interrupt flags. All bits are write 1 to
 *           clear.
 * @{
 */
#define MXC_F_SPI_INT_FL_TXTHRLD_POS                   0 /**< INT_FL_TXTHRLD Position */
#define MXC_F_SPI_INT_FL_TXTHRLD                       ((uint32_t)(0x1UL << MXC_F_SPI_INT_FL_TXTHRLD_POS)) /**< INT_FL_TXTHRLD Mask */

#define MXC_F_SPI_INT_FL_TXEMPTY_POS                   1 /**< INT_FL_TXEMPTY Position */
#define MXC_F_SPI_INT_FL_TXEMPTY                       ((uint32_t)(0x1UL << MXC_F_SPI_INT_FL_TXEMPTY_POS)) /**< INT_FL_TXEMPTY Mask */

#define MXC_F_SPI_INT_FL_RXTHRLD_POS                   2 /**< INT_FL_RXTHRLD Position */
#define MXC_F_SPI_INT_FL_RXTHRLD                       ((uint32_t)(0x1UL << MXC_F_SPI_INT_FL_RXTHRLD_POS)) /**< INT_FL_RXTHRLD Mask */

#define MXC_F_SPI_INT_FL_RXFULL_POS                    3 /**< INT_FL_RXFULL Position */
#define MXC_F_SPI_INT_FL_RXFULL                        ((uint32_t)(0x1UL << MXC_F_SPI_INT_FL_RXFULL_POS)) /**< INT_FL_RXFULL Mask */

#define MXC_F_SPI_INT_FL_SSA_POS                       4 /**< INT_FL_SSA Position */
#define MXC_F_SPI_INT_FL_SSA                           ((uint32_t)(0x1UL << MXC_F_SPI_INT_FL_SSA_POS)) /**< INT_FL_SSA Mask */

#define MXC_F_SPI_INT_FL_SSD_POS                       5 /**< INT_FL_SSD Position */
#define MXC_F_SPI_INT_FL_SSD                           ((uint32_t)(0x1UL << MXC_F_SPI_INT_FL_SSD_POS)) /**< INT_FL_SSD Mask */

#define MXC_F_SPI_INT_FL_FAULT_POS                     8 /**< INT_FL_FAULT Position */
#define MXC_F_SPI_INT_FL_FAULT                         ((uint32_t)(0x1UL << MXC_F_SPI_INT_FL_FAULT_POS)) /**< INT_FL_FAULT Mask */

#define MXC_F_SPI_INT_FL_ABORT_POS                     9 /**< INT_FL_ABORT Position */
#define MXC_F_SPI_INT_FL_ABORT                         ((uint32_t)(0x1UL << MXC_F_SPI_INT_FL_ABORT_POS)) /**< INT_FL_ABORT Mask */

#define MXC_F_SPI_INT_FL_MSTRDONE_POS                  11 /**< INT_FL_MSTRDONE Position */
#define MXC_F_SPI_INT_FL_MSTRDONE                      ((uint32_t)(0x1UL << MXC_F_SPI_INT_FL_MSTRDONE_POS)) /**< INT_FL_MSTRDONE Mask */

#define MXC_F_SPI_INT_FL_TXOVR_POS                     12 /**< INT_FL_TXOVR Position */
#define MXC_F_SPI_INT_FL_TXOVR                         ((uint32_t)(0x1UL << MXC_F_SPI_INT_FL_TXOVR_POS)) /**< INT_FL_TXOVR Mask */

#define MXC_F_SPI_INT_FL_TXUNDR_POS                    13 /**< INT_FL_TXUNDR Position */
#define MXC_F_SPI_INT_FL_TXUNDR                        ((uint32_t)(0x1UL << MXC_F_SPI_INT_FL_TXUNDR_POS)) /**< INT_FL_TXUNDR Mask */

#define MXC_F_SPI_INT_FL_RXOVR_POS                     14 /**< INT_FL_RXOVR Position */
#define MXC_F_SPI_INT_FL_RXOVR                         ((uint32_t)(0x1UL << MXC_F_SPI_INT_FL_RXOVR_POS)) /**< INT_FL_RXOVR Mask */

#define MXC_F_SPI_INT_FL_RXUNDR_POS                    15 /**< INT_FL_RXUNDR Position */
#define MXC_F_SPI_INT_FL_RXUNDR                        ((uint32_t)(0x1UL << MXC_F_SPI_INT_FL_RXUNDR_POS)) /**< INT_FL_RXUNDR Mask */

/**@} end of group SPI_INT_FL_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_INT_EN SPI_INT_EN
 * @brief    Register for enabling interrupts.
 * @{
 */
#define MXC_F_SPI_INT_EN_TXTHRLD_POS                   0 /**< INT_EN_TXTHRLD Position */
#define MXC_F_SPI_INT_EN_TXTHRLD                       ((uint32_t)(0x1UL << MXC_F_SPI_INT_EN_TXTHRLD_POS)) /**< INT_EN_TXTHRLD Mask */

#define MXC_F_SPI_INT_EN_TXEMPTY_POS                   1 /**< INT_EN_TXEMPTY Position */
#define MXC_F_SPI_INT_EN_TXEMPTY                       ((uint32_t)(0x1UL << MXC_F_SPI_INT_EN_TXEMPTY_POS)) /**< INT_EN_TXEMPTY Mask */

#define MXC_F_SPI_INT_EN_RXTHRLD_POS                   2 /**< INT_EN_RXTHRLD Position */
#define MXC_F_SPI_INT_EN_RXTHRLD                       ((uint32_t)(0x1UL << MXC_F_SPI_INT_EN_RXTHRLD_POS)) /**< INT_EN_RXTHRLD Mask */

#define MXC_F_SPI_INT_EN_RXFULL_POS                    3 /**< INT_EN_RXFULL Position */
#define MXC_F_SPI_INT_EN_RXFULL                        ((uint32_t)(0x1UL << MXC_F_SPI_INT_EN_RXFULL_POS)) /**< INT_EN_RXFULL Mask */

#define MXC_F_SPI_INT_EN_SSA_POS                       4 /**< INT_EN_SSA Position */
#define MXC_F_SPI_INT_EN_SSA                           ((uint32_t)(0x1UL << MXC_F_SPI_INT_EN_SSA_POS)) /**< INT_EN_SSA Mask */

#define MXC_F_SPI_INT_EN_SSD_POS                       5 /**< INT_EN_SSD Position */
#define MXC_F_SPI_INT_EN_SSD                           ((uint32_t)(0x1UL << MXC_F_SPI_INT_EN_SSD_POS)) /**< INT_EN_SSD Mask */

#define MXC_F_SPI_INT_EN_FAULT_POS                     8 /**< INT_EN_FAULT Position */
#define MXC_F_SPI_INT_EN_FAULT                         ((uint32_t)(0x1UL << MXC_F_SPI_INT_EN_FAULT_POS)) /**< INT_EN_FAULT Mask */

#define MXC_F_SPI_INT_EN_ABORT_POS                     9 /**< INT_EN_ABORT Position */
#define MXC_F_SPI_INT_EN_ABORT                         ((uint32_t)(0x1UL << MXC_F_SPI_INT_EN_ABORT_POS)) /**< INT_EN_ABORT Mask */

#define MXC_F_SPI_INT_EN_MSTRDONE_POS                  11 /**< INT_EN_MSTRDONE Position */
#define MXC_F_SPI_INT_EN_MSTRDONE                      ((uint32_t)(0x1UL << MXC_F_SPI_INT_EN_MSTRDONE_POS)) /**< INT_EN_MSTRDONE Mask */

#define MXC_F_SPI_INT_EN_TXOVR_POS                     12 /**< INT_EN_TXOVR Position */
#define MXC_F_SPI_INT_EN_TXOVR                         ((uint32_t)(0x1UL << MXC_F_SPI_INT_EN_TXOVR_POS)) /**< INT_EN_TXOVR Mask */

#define MXC_F_SPI_INT_EN_TXUNDR_POS                    13 /**< INT_EN_TXUNDR Position */
#define MXC_F_SPI_INT_EN_TXUNDR                        ((uint32_t)(0x1UL << MXC_F_SPI_INT_EN_TXUNDR_POS)) /**< INT_EN_TXUNDR Mask */

#define MXC_F_SPI_INT_EN_RXOVR_POS                     14 /**< INT_EN_RXOVR Position */
#define MXC_F_SPI_INT_EN_RXOVR                         ((uint32_t)(0x1UL << MXC_F_SPI_INT_EN_RXOVR_POS)) /**< INT_EN_RXOVR Mask */

#define MXC_F_SPI_INT_EN_RXUNDR_POS                    15 /**< INT_EN_RXUNDR Position */
#define MXC_F_SPI_INT_EN_RXUNDR                        ((uint32_t)(0x1UL << MXC_F_SPI_INT_EN_RXUNDR_POS)) /**< INT_EN_RXUNDR Mask */

/**@} end of group SPI_INT_EN_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_WAKE_FL SPI_WAKE_FL
 * @brief    Register for wake up flags. All bits in this register are write 1 to clear.
 * @{
 */
#define MXC_F_SPI_WAKE_FL_TXTHRLD_POS                  0 /**< WAKE_FL_TXTHRLD Position */
#define MXC_F_SPI_WAKE_FL_TXTHRLD                      ((uint32_t)(0x1UL << MXC_F_SPI_WAKE_FL_TXTHRLD_POS)) /**< WAKE_FL_TXTHRLD Mask */

#define MXC_F_SPI_WAKE_FL_TXEMPTY_POS                  1 /**< WAKE_FL_TXEMPTY Position */
#define MXC_F_SPI_WAKE_FL_TXEMPTY                      ((uint32_t)(0x1UL << MXC_F_SPI_WAKE_FL_TXEMPTY_POS)) /**< WAKE_FL_TXEMPTY Mask */

#define MXC_F_SPI_WAKE_FL_RXTHRLD_POS                  2 /**< WAKE_FL_RXTHRLD Position */
#define MXC_F_SPI_WAKE_FL_RXTHRLD                      ((uint32_t)(0x1UL << MXC_F_SPI_WAKE_FL_RXTHRLD_POS)) /**< WAKE_FL_RXTHRLD Mask */

#define MXC_F_SPI_WAKE_FL_RXFULL_POS                   3 /**< WAKE_FL_RXFULL Position */
#define MXC_F_SPI_WAKE_FL_RXFULL                       ((uint32_t)(0x1UL << MXC_F_SPI_WAKE_FL_RXFULL_POS)) /**< WAKE_FL_RXFULL Mask */

/**@} end of group SPI_WAKE_FL_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_WAKE_EN SPI_WAKE_EN
 * @brief    Register for wake up enable.
 * @{
 */
#define MXC_F_SPI_WAKE_EN_TXTHRLD_POS                  0 /**< WAKE_EN_TXTHRLD Position */
#define MXC_F_SPI_WAKE_EN_TXTHRLD                      ((uint32_t)(0x1UL << MXC_F_SPI_WAKE_EN_TXTHRLD_POS)) /**< WAKE_EN_TXTHRLD Mask */

#define MXC_F_SPI_WAKE_EN_TXEMPTY_POS                  1 /**< WAKE_EN_TXEMPTY Position */
#define MXC_F_SPI_WAKE_EN_TXEMPTY                      ((uint32_t)(0x1UL << MXC_F_SPI_WAKE_EN_TXEMPTY_POS)) /**< WAKE_EN_TXEMPTY Mask */

#define MXC_F_SPI_WAKE_EN_RXTHRLD_POS                  2 /**< WAKE_EN_RXTHRLD Position */
#define MXC_F_SPI_WAKE_EN_RXTHRLD                      ((uint32_t)(0x1UL << MXC_F_SPI_WAKE_EN_RXTHRLD_POS)) /**< WAKE_EN_RXTHRLD Mask */

#define MXC_F_SPI_WAKE_EN_RXFULL_POS                   3 /**< WAKE_EN_RXFULL Position */
#define MXC_F_SPI_WAKE_EN_RXFULL                       ((uint32_t)(0x1UL << MXC_F_SPI_WAKE_EN_RXFULL_POS)) /**< WAKE_EN_RXFULL Mask */

/**@} end of group SPI_WAKE_EN_Register */

/**
 * @ingroup  spi_registers
 * @defgroup SPI_STAT SPI_STAT
 * @brief    SPI Status register.
 * @{
 */
#define MXC_F_SPI_STAT_BUSY_POS                        0 /**< STAT_BUSY Position */
#define MXC_F_SPI_STAT_BUSY                            ((uint32_t)(0x1UL << MXC_F_SPI_STAT_BUSY_POS)) /**< STAT_BUSY Mask */

/**@} end of group SPI_STAT_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32520_INCLUDE_SPI_REGS_H_
