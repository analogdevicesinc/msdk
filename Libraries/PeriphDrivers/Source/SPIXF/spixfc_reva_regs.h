/**
 * @file    spixfc_reva_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SPIXFC_REVA Peripheral Module.
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

#ifndef _SPIXFC_REVA_REGS_H_
#define _SPIXFC_REVA_REGS_H_

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
 * @ingroup     spixfc_reva
 * @defgroup    spixfc_reva_registers SPIXFC_REVA_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SPIXFC_REVA Peripheral Module.
 * @details SPI XiP Flash Configuration Controller
 */

/**
 * @ingroup spixfc_reva_registers
 * Structure type to access the SPIXFC_REVA Registers.
 */
typedef struct {
    __IO uint32_t cfg;                  /**< <tt>\b 0x00:</tt> SPIXFC_REVA CFG Register */
    __IO uint32_t ss_pol;               /**< <tt>\b 0x04:</tt> SPIXFC_REVA SS_POL Register */
    __IO uint32_t gen_ctrl;             /**< <tt>\b 0x08:</tt> SPIXFC_REVA GEN_CTRL Register */
    __IO uint32_t fifo_ctrl;            /**< <tt>\b 0x0C:</tt> SPIXFC_REVA FIFO_CTRL Register */
    __IO uint32_t sp_ctrl;              /**< <tt>\b 0x10:</tt> SPIXFC_REVA SP_CTRL Register */
    __IO uint32_t int_fl;               /**< <tt>\b 0x14:</tt> SPIXFC_REVA INT_FL Register */
    __IO uint32_t int_en;               /**< <tt>\b 0x18:</tt> SPIXFC_REVA INT_EN Register */
} mxc_spixfc_reva_regs_t;

/* Register offsets for module SPIXFC_REVA */
/**
 * @ingroup    spixfc_reva_registers
 * @defgroup   SPIXFC_REVA_Register_Offsets Register Offsets
 * @brief      SPIXFC_REVA Peripheral Register Offsets from the SPIXFC_REVA Base Peripheral Address. 
 * @{
 */
 #define MXC_R_SPIXFC_REVA_CFG              ((uint32_t)0x00000000UL) /**< Offset from SPIXFC_REVA Base Address: <tt> 0x0000</tt> */ 
 #define MXC_R_SPIXFC_REVA_SS_POL           ((uint32_t)0x00000004UL) /**< Offset from SPIXFC_REVA Base Address: <tt> 0x0004</tt> */ 
 #define MXC_R_SPIXFC_REVA_GEN_CTRL         ((uint32_t)0x00000008UL) /**< Offset from SPIXFC_REVA Base Address: <tt> 0x0008</tt> */ 
 #define MXC_R_SPIXFC_REVA_FIFO_CTRL        ((uint32_t)0x0000000CUL) /**< Offset from SPIXFC_REVA Base Address: <tt> 0x000C</tt> */ 
 #define MXC_R_SPIXFC_REVA_SP_CTRL          ((uint32_t)0x00000010UL) /**< Offset from SPIXFC_REVA Base Address: <tt> 0x0010</tt> */ 
 #define MXC_R_SPIXFC_REVA_INT_FL           ((uint32_t)0x00000014UL) /**< Offset from SPIXFC_REVA Base Address: <tt> 0x0014</tt> */ 
 #define MXC_R_SPIXFC_REVA_INT_EN           ((uint32_t)0x00000018UL) /**< Offset from SPIXFC_REVA Base Address: <tt> 0x0018</tt> */ 
/**@} end of group spixfc_reva_registers */

/**
 * @ingroup  spixfc_reva_registers
 * @defgroup SPIXFC_REVA_CFG SPIXFC_REVA_CFG
 * @brief    Configuration Register.
 * @{
 */
 #define MXC_F_SPIXFC_REVA_CFG_SSEL_POS                 0 /**< CFG_SSEL Position */
 #define MXC_F_SPIXFC_REVA_CFG_SSEL                     ((uint32_t)(0x7UL << MXC_F_SPIXFC_REVA_CFG_SSEL_POS)) /**< CFG_SSEL Mask */
 #define MXC_V_SPIXFC_REVA_CFG_SSEL_SLAVE_0             ((uint32_t)0x0UL) /**< CFG_SSEL_SLAVE_0 Value */
 #define MXC_S_SPIXFC_REVA_CFG_SSEL_SLAVE_0             (MXC_V_SPIXFC_REVA_CFG_SSEL_SLAVE_0 << MXC_F_SPIXFC_REVA_CFG_SSEL_POS) /**< CFG_SSEL_SLAVE_0 Setting */
 #define MXC_V_SPIXFC_REVA_CFG_SSEL_SLAVE_1             ((uint32_t)0x1UL) /**< CFG_SSEL_SLAVE_1 Value */
 #define MXC_S_SPIXFC_REVA_CFG_SSEL_SLAVE_1             (MXC_V_SPIXFC_REVA_CFG_SSEL_SLAVE_1 << MXC_F_SPIXFC_REVA_CFG_SSEL_POS) /**< CFG_SSEL_SLAVE_1 Setting */

 #define MXC_F_SPIXFC_REVA_CFG_MODE_POS                 4 /**< CFG_MODE Position */
 #define MXC_F_SPIXFC_REVA_CFG_MODE                     ((uint32_t)(0x3UL << MXC_F_SPIXFC_REVA_CFG_MODE_POS)) /**< CFG_MODE Mask */
 #define MXC_V_SPIXFC_REVA_CFG_MODE_SPIX_MODE_0         ((uint32_t)0x0UL) /**< CFG_MODE_SPIX_MODE_0 Value */
 #define MXC_S_SPIXFC_REVA_CFG_MODE_SPIX_MODE_0         (MXC_V_SPIXFC_REVA_CFG_MODE_SPIX_MODE_0 << MXC_F_SPIXFC_REVA_CFG_MODE_POS) /**< CFG_MODE_SPIX_MODE_0 Setting */
 #define MXC_V_SPIXFC_REVA_CFG_MODE_SPIX_MODE_3         ((uint32_t)0x3UL) /**< CFG_MODE_SPIX_MODE_3 Value */
 #define MXC_S_SPIXFC_REVA_CFG_MODE_SPIX_MODE_3         (MXC_V_SPIXFC_REVA_CFG_MODE_SPIX_MODE_3 << MXC_F_SPIXFC_REVA_CFG_MODE_POS) /**< CFG_MODE_SPIX_MODE_3 Setting */

 #define MXC_F_SPIXFC_REVA_CFG_PAGE_SIZE_POS            6 /**< CFG_PAGE_SIZE Position */
 #define MXC_F_SPIXFC_REVA_CFG_PAGE_SIZE                ((uint32_t)(0x3UL << MXC_F_SPIXFC_REVA_CFG_PAGE_SIZE_POS)) /**< CFG_PAGE_SIZE Mask */
 #define MXC_V_SPIXFC_REVA_CFG_PAGE_SIZE_4_BYTES        ((uint32_t)0x0UL) /**< CFG_PAGE_SIZE_4_BYTES Value */
 #define MXC_S_SPIXFC_REVA_CFG_PAGE_SIZE_4_BYTES        (MXC_V_SPIXFC_REVA_CFG_PAGE_SIZE_4_BYTES << MXC_F_SPIXFC_REVA_CFG_PAGE_SIZE_POS) /**< CFG_PAGE_SIZE_4_BYTES Setting */
 #define MXC_V_SPIXFC_REVA_CFG_PAGE_SIZE_8_BYTES        ((uint32_t)0x1UL) /**< CFG_PAGE_SIZE_8_BYTES Value */
 #define MXC_S_SPIXFC_REVA_CFG_PAGE_SIZE_8_BYTES        (MXC_V_SPIXFC_REVA_CFG_PAGE_SIZE_8_BYTES << MXC_F_SPIXFC_REVA_CFG_PAGE_SIZE_POS) /**< CFG_PAGE_SIZE_8_BYTES Setting */
 #define MXC_V_SPIXFC_REVA_CFG_PAGE_SIZE_16_BYTES       ((uint32_t)0x2UL) /**< CFG_PAGE_SIZE_16_BYTES Value */
 #define MXC_S_SPIXFC_REVA_CFG_PAGE_SIZE_16_BYTES       (MXC_V_SPIXFC_REVA_CFG_PAGE_SIZE_16_BYTES << MXC_F_SPIXFC_REVA_CFG_PAGE_SIZE_POS) /**< CFG_PAGE_SIZE_16_BYTES Setting */
 #define MXC_V_SPIXFC_REVA_CFG_PAGE_SIZE_32_BYTES       ((uint32_t)0x3UL) /**< CFG_PAGE_SIZE_32_BYTES Value */
 #define MXC_S_SPIXFC_REVA_CFG_PAGE_SIZE_32_BYTES       (MXC_V_SPIXFC_REVA_CFG_PAGE_SIZE_32_BYTES << MXC_F_SPIXFC_REVA_CFG_PAGE_SIZE_POS) /**< CFG_PAGE_SIZE_32_BYTES Setting */

 #define MXC_F_SPIXFC_REVA_CFG_HI_CLK_POS               8 /**< CFG_HI_CLK Position */
 #define MXC_F_SPIXFC_REVA_CFG_HI_CLK                   ((uint32_t)(0xFUL << MXC_F_SPIXFC_REVA_CFG_HI_CLK_POS)) /**< CFG_HI_CLK Mask */
 #define MXC_V_SPIXFC_REVA_CFG_HI_CLK_16_SCLK           ((uint32_t)0x0UL) /**< CFG_HI_CLK_16_SCLK Value */
 #define MXC_S_SPIXFC_REVA_CFG_HI_CLK_16_SCLK           (MXC_V_SPIXFC_REVA_CFG_HI_CLK_16_SCLK << MXC_F_SPIXFC_REVA_CFG_HI_CLK_POS) /**< CFG_HI_CLK_16_SCLK Setting */

 #define MXC_F_SPIXFC_REVA_CFG_LO_CLK_POS               12 /**< CFG_LO_CLK Position */
 #define MXC_F_SPIXFC_REVA_CFG_LO_CLK                   ((uint32_t)(0xFUL << MXC_F_SPIXFC_REVA_CFG_LO_CLK_POS)) /**< CFG_LO_CLK Mask */
 #define MXC_V_SPIXFC_REVA_CFG_LO_CLK_16_SCLK           ((uint32_t)0x0UL) /**< CFG_LO_CLK_16_SCLK Value */
 #define MXC_S_SPIXFC_REVA_CFG_LO_CLK_16_SCLK           (MXC_V_SPIXFC_REVA_CFG_LO_CLK_16_SCLK << MXC_F_SPIXFC_REVA_CFG_LO_CLK_POS) /**< CFG_LO_CLK_16_SCLK Setting */

 #define MXC_F_SPIXFC_REVA_CFG_SSACT_POS                16 /**< CFG_SSACT Position */
 #define MXC_F_SPIXFC_REVA_CFG_SSACT                    ((uint32_t)(0x3UL << MXC_F_SPIXFC_REVA_CFG_SSACT_POS)) /**< CFG_SSACT Mask */
 #define MXC_V_SPIXFC_REVA_CFG_SSACT_0_CLKS             ((uint32_t)0x0UL) /**< CFG_SSACT_0_CLKS Value */
 #define MXC_S_SPIXFC_REVA_CFG_SSACT_0_CLKS             (MXC_V_SPIXFC_REVA_CFG_SSACT_0_CLKS << MXC_F_SPIXFC_REVA_CFG_SSACT_POS) /**< CFG_SSACT_0_CLKS Setting */
 #define MXC_V_SPIXFC_REVA_CFG_SSACT_2_CLKS             ((uint32_t)0x1UL) /**< CFG_SSACT_2_CLKS Value */
 #define MXC_S_SPIXFC_REVA_CFG_SSACT_2_CLKS             (MXC_V_SPIXFC_REVA_CFG_SSACT_2_CLKS << MXC_F_SPIXFC_REVA_CFG_SSACT_POS) /**< CFG_SSACT_2_CLKS Setting */
 #define MXC_V_SPIXFC_REVA_CFG_SSACT_4_CLKS             ((uint32_t)0x2UL) /**< CFG_SSACT_4_CLKS Value */
 #define MXC_S_SPIXFC_REVA_CFG_SSACT_4_CLKS             (MXC_V_SPIXFC_REVA_CFG_SSACT_4_CLKS << MXC_F_SPIXFC_REVA_CFG_SSACT_POS) /**< CFG_SSACT_4_CLKS Setting */
 #define MXC_V_SPIXFC_REVA_CFG_SSACT_8_CLKS             ((uint32_t)0x3UL) /**< CFG_SSACT_8_CLKS Value */
 #define MXC_S_SPIXFC_REVA_CFG_SSACT_8_CLKS             (MXC_V_SPIXFC_REVA_CFG_SSACT_8_CLKS << MXC_F_SPIXFC_REVA_CFG_SSACT_POS) /**< CFG_SSACT_8_CLKS Setting */

 #define MXC_F_SPIXFC_REVA_CFG_SSIACT_POS               18 /**< CFG_SSIACT Position */
 #define MXC_F_SPIXFC_REVA_CFG_SSIACT                   ((uint32_t)(0x3UL << MXC_F_SPIXFC_REVA_CFG_SSIACT_POS)) /**< CFG_SSIACT Mask */
 #define MXC_V_SPIXFC_REVA_CFG_SSIACT_4_CLKS            ((uint32_t)0x0UL) /**< CFG_SSIACT_4_CLKS Value */
 #define MXC_S_SPIXFC_REVA_CFG_SSIACT_4_CLKS            (MXC_V_SPIXFC_REVA_CFG_SSIACT_4_CLKS << MXC_F_SPIXFC_REVA_CFG_SSIACT_POS) /**< CFG_SSIACT_4_CLKS Setting */
 #define MXC_V_SPIXFC_REVA_CFG_SSIACT_6_CLKS            ((uint32_t)0x1UL) /**< CFG_SSIACT_6_CLKS Value */
 #define MXC_S_SPIXFC_REVA_CFG_SSIACT_6_CLKS            (MXC_V_SPIXFC_REVA_CFG_SSIACT_6_CLKS << MXC_F_SPIXFC_REVA_CFG_SSIACT_POS) /**< CFG_SSIACT_6_CLKS Setting */
 #define MXC_V_SPIXFC_REVA_CFG_SSIACT_8_CLKS            ((uint32_t)0x2UL) /**< CFG_SSIACT_8_CLKS Value */
 #define MXC_S_SPIXFC_REVA_CFG_SSIACT_8_CLKS            (MXC_V_SPIXFC_REVA_CFG_SSIACT_8_CLKS << MXC_F_SPIXFC_REVA_CFG_SSIACT_POS) /**< CFG_SSIACT_8_CLKS Setting */
 #define MXC_V_SPIXFC_REVA_CFG_SSIACT_12_CLKS           ((uint32_t)0x3UL) /**< CFG_SSIACT_12_CLKS Value */
 #define MXC_S_SPIXFC_REVA_CFG_SSIACT_12_CLKS           (MXC_V_SPIXFC_REVA_CFG_SSIACT_12_CLKS << MXC_F_SPIXFC_REVA_CFG_SSIACT_POS) /**< CFG_SSIACT_12_CLKS Setting */

 #define MXC_F_SPIXFC_REVA_CFG_IOSMPL_POS               20 /**< CFG_IOSMPL Position */
 #define MXC_F_SPIXFC_REVA_CFG_IOSMPL                   ((uint32_t)(0xFUL << MXC_F_SPIXFC_REVA_CFG_IOSMPL_POS)) /**< CFG_IOSMPL Mask */

/**@} end of group SPIXFC_REVA_CFG_Register */

/**
 * @ingroup  spixfc_reva_registers
 * @defgroup SPIXFC_REVA_SS_POL SPIXFC_REVA_SS_POL
 * @brief    SPIX Controller Slave Select Polarity Register.
 * @{
 */
 #define MXC_F_SPIXFC_REVA_SS_POL_SS_POLARITY_POS       0 /**< SS_POL_SS_POLARITY Position */
 #define MXC_F_SPIXFC_REVA_SS_POL_SS_POLARITY           ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_SS_POL_SS_POLARITY_POS)) /**< SS_POL_SS_POLARITY Mask */

/**@} end of group SPIXFC_REVA_SS_POL_Register */

/**
 * @ingroup  spixfc_reva_registers
 * @defgroup SPIXFC_REVA_GEN_CTRL SPIXFC_REVA_GEN_CTRL
 * @brief    SPIX Controller General Controller Register.
 * @{
 */
 #define MXC_F_SPIXFC_REVA_GEN_CTRL_ENABLE_POS          0 /**< GEN_CTRL_ENABLE Position */
 #define MXC_F_SPIXFC_REVA_GEN_CTRL_ENABLE              ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_GEN_CTRL_ENABLE_POS)) /**< GEN_CTRL_ENABLE Mask */

 #define MXC_F_SPIXFC_REVA_GEN_CTRL_TX_FIFO_EN_POS      1 /**< GEN_CTRL_TX_FIFO_EN Position */
 #define MXC_F_SPIXFC_REVA_GEN_CTRL_TX_FIFO_EN          ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_GEN_CTRL_TX_FIFO_EN_POS)) /**< GEN_CTRL_TX_FIFO_EN Mask */

 #define MXC_F_SPIXFC_REVA_GEN_CTRL_RX_FIFO_EN_POS      2 /**< GEN_CTRL_RX_FIFO_EN Position */
 #define MXC_F_SPIXFC_REVA_GEN_CTRL_RX_FIFO_EN          ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_GEN_CTRL_RX_FIFO_EN_POS)) /**< GEN_CTRL_RX_FIFO_EN Mask */

 #define MXC_F_SPIXFC_REVA_GEN_CTRL_BBMODE_POS          3 /**< GEN_CTRL_BBMODE Position */
 #define MXC_F_SPIXFC_REVA_GEN_CTRL_BBMODE              ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_GEN_CTRL_BBMODE_POS)) /**< GEN_CTRL_BBMODE Mask */

 #define MXC_F_SPIXFC_REVA_GEN_CTRL_SSDR_POS            4 /**< GEN_CTRL_SSDR Position */
 #define MXC_F_SPIXFC_REVA_GEN_CTRL_SSDR                ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_GEN_CTRL_SSDR_POS)) /**< GEN_CTRL_SSDR Mask */

 #define MXC_F_SPIXFC_REVA_GEN_CTRL_SCLK_DR_POS         6 /**< GEN_CTRL_SCLK_DR Position */
 #define MXC_F_SPIXFC_REVA_GEN_CTRL_SCLK_DR             ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_GEN_CTRL_SCLK_DR_POS)) /**< GEN_CTRL_SCLK_DR Mask */

 #define MXC_F_SPIXFC_REVA_GEN_CTRL_SDIO_DATA_IN_POS    8 /**< GEN_CTRL_SDIO_DATA_IN Position */
 #define MXC_F_SPIXFC_REVA_GEN_CTRL_SDIO_DATA_IN        ((uint32_t)(0xFUL << MXC_F_SPIXFC_REVA_GEN_CTRL_SDIO_DATA_IN_POS)) /**< GEN_CTRL_SDIO_DATA_IN Mask */
 #define MXC_V_SPIXFC_REVA_GEN_CTRL_SDIO_DATA_IN_SDIO0  ((uint32_t)0x0UL) /**< GEN_CTRL_SDIO_DATA_IN_SDIO0 Value */
 #define MXC_S_SPIXFC_REVA_GEN_CTRL_SDIO_DATA_IN_SDIO0  (MXC_V_SPIXFC_REVA_GEN_CTRL_SDIO_DATA_IN_SDIO0 << MXC_F_SPIXFC_REVA_GEN_CTRL_SDIO_DATA_IN_POS) /**< GEN_CTRL_SDIO_DATA_IN_SDIO0 Setting */
 #define MXC_V_SPIXFC_REVA_GEN_CTRL_SDIO_DATA_IN_SDIO1  ((uint32_t)0x1UL) /**< GEN_CTRL_SDIO_DATA_IN_SDIO1 Value */
 #define MXC_S_SPIXFC_REVA_GEN_CTRL_SDIO_DATA_IN_SDIO1  (MXC_V_SPIXFC_REVA_GEN_CTRL_SDIO_DATA_IN_SDIO1 << MXC_F_SPIXFC_REVA_GEN_CTRL_SDIO_DATA_IN_POS) /**< GEN_CTRL_SDIO_DATA_IN_SDIO1 Setting */
 #define MXC_V_SPIXFC_REVA_GEN_CTRL_SDIO_DATA_IN_SDIO2  ((uint32_t)0x2UL) /**< GEN_CTRL_SDIO_DATA_IN_SDIO2 Value */
 #define MXC_S_SPIXFC_REVA_GEN_CTRL_SDIO_DATA_IN_SDIO2  (MXC_V_SPIXFC_REVA_GEN_CTRL_SDIO_DATA_IN_SDIO2 << MXC_F_SPIXFC_REVA_GEN_CTRL_SDIO_DATA_IN_POS) /**< GEN_CTRL_SDIO_DATA_IN_SDIO2 Setting */
 #define MXC_V_SPIXFC_REVA_GEN_CTRL_SDIO_DATA_IN_SDIO3  ((uint32_t)0x3UL) /**< GEN_CTRL_SDIO_DATA_IN_SDIO3 Value */
 #define MXC_S_SPIXFC_REVA_GEN_CTRL_SDIO_DATA_IN_SDIO3  (MXC_V_SPIXFC_REVA_GEN_CTRL_SDIO_DATA_IN_SDIO3 << MXC_F_SPIXFC_REVA_GEN_CTRL_SDIO_DATA_IN_POS) /**< GEN_CTRL_SDIO_DATA_IN_SDIO3 Setting */

 #define MXC_F_SPIXFC_REVA_GEN_CTRL_BB_DATA_POS         12 /**< GEN_CTRL_BB_DATA Position */
 #define MXC_F_SPIXFC_REVA_GEN_CTRL_BB_DATA             ((uint32_t)(0xFUL << MXC_F_SPIXFC_REVA_GEN_CTRL_BB_DATA_POS)) /**< GEN_CTRL_BB_DATA Mask */
 #define MXC_V_SPIXFC_REVA_GEN_CTRL_BB_DATA_SDIO0       ((uint32_t)0x0UL) /**< GEN_CTRL_BB_DATA_SDIO0 Value */
 #define MXC_S_SPIXFC_REVA_GEN_CTRL_BB_DATA_SDIO0       (MXC_V_SPIXFC_REVA_GEN_CTRL_BB_DATA_SDIO0 << MXC_F_SPIXFC_REVA_GEN_CTRL_BB_DATA_POS) /**< GEN_CTRL_BB_DATA_SDIO0 Setting */
 #define MXC_V_SPIXFC_REVA_GEN_CTRL_BB_DATA_SDIO1       ((uint32_t)0x1UL) /**< GEN_CTRL_BB_DATA_SDIO1 Value */
 #define MXC_S_SPIXFC_REVA_GEN_CTRL_BB_DATA_SDIO1       (MXC_V_SPIXFC_REVA_GEN_CTRL_BB_DATA_SDIO1 << MXC_F_SPIXFC_REVA_GEN_CTRL_BB_DATA_POS) /**< GEN_CTRL_BB_DATA_SDIO1 Setting */
 #define MXC_V_SPIXFC_REVA_GEN_CTRL_BB_DATA_SDIO2       ((uint32_t)0x2UL) /**< GEN_CTRL_BB_DATA_SDIO2 Value */
 #define MXC_S_SPIXFC_REVA_GEN_CTRL_BB_DATA_SDIO2       (MXC_V_SPIXFC_REVA_GEN_CTRL_BB_DATA_SDIO2 << MXC_F_SPIXFC_REVA_GEN_CTRL_BB_DATA_POS) /**< GEN_CTRL_BB_DATA_SDIO2 Setting */
 #define MXC_V_SPIXFC_REVA_GEN_CTRL_BB_DATA_SDIO3       ((uint32_t)0x3UL) /**< GEN_CTRL_BB_DATA_SDIO3 Value */
 #define MXC_S_SPIXFC_REVA_GEN_CTRL_BB_DATA_SDIO3       (MXC_V_SPIXFC_REVA_GEN_CTRL_BB_DATA_SDIO3 << MXC_F_SPIXFC_REVA_GEN_CTRL_BB_DATA_POS) /**< GEN_CTRL_BB_DATA_SDIO3 Setting */

 #define MXC_F_SPIXFC_REVA_GEN_CTRL_BB_DATA_OUT_EN_POS  16 /**< GEN_CTRL_BB_DATA_OUT_EN Position */
 #define MXC_F_SPIXFC_REVA_GEN_CTRL_BB_DATA_OUT_EN      ((uint32_t)(0xFUL << MXC_F_SPIXFC_REVA_GEN_CTRL_BB_DATA_OUT_EN_POS)) /**< GEN_CTRL_BB_DATA_OUT_EN Mask */
 #define MXC_V_SPIXFC_REVA_GEN_CTRL_BB_DATA_OUT_EN_SDIO0 ((uint32_t)0x0UL) /**< GEN_CTRL_BB_DATA_OUT_EN_SDIO0 Value */
 #define MXC_S_SPIXFC_REVA_GEN_CTRL_BB_DATA_OUT_EN_SDIO0 (MXC_V_SPIXFC_REVA_GEN_CTRL_BB_DATA_OUT_EN_SDIO0 << MXC_F_SPIXFC_REVA_GEN_CTRL_BB_DATA_OUT_EN_POS) /**< GEN_CTRL_BB_DATA_OUT_EN_SDIO0 Setting */
 #define MXC_V_SPIXFC_REVA_GEN_CTRL_BB_DATA_OUT_EN_SDIO1 ((uint32_t)0x1UL) /**< GEN_CTRL_BB_DATA_OUT_EN_SDIO1 Value */
 #define MXC_S_SPIXFC_REVA_GEN_CTRL_BB_DATA_OUT_EN_SDIO1 (MXC_V_SPIXFC_REVA_GEN_CTRL_BB_DATA_OUT_EN_SDIO1 << MXC_F_SPIXFC_REVA_GEN_CTRL_BB_DATA_OUT_EN_POS) /**< GEN_CTRL_BB_DATA_OUT_EN_SDIO1 Setting */
 #define MXC_V_SPIXFC_REVA_GEN_CTRL_BB_DATA_OUT_EN_SDIO2 ((uint32_t)0x2UL) /**< GEN_CTRL_BB_DATA_OUT_EN_SDIO2 Value */
 #define MXC_S_SPIXFC_REVA_GEN_CTRL_BB_DATA_OUT_EN_SDIO2 (MXC_V_SPIXFC_REVA_GEN_CTRL_BB_DATA_OUT_EN_SDIO2 << MXC_F_SPIXFC_REVA_GEN_CTRL_BB_DATA_OUT_EN_POS) /**< GEN_CTRL_BB_DATA_OUT_EN_SDIO2 Setting */
 #define MXC_V_SPIXFC_REVA_GEN_CTRL_BB_DATA_OUT_EN_SDIO3 ((uint32_t)0x3UL) /**< GEN_CTRL_BB_DATA_OUT_EN_SDIO3 Value */
 #define MXC_S_SPIXFC_REVA_GEN_CTRL_BB_DATA_OUT_EN_SDIO3 (MXC_V_SPIXFC_REVA_GEN_CTRL_BB_DATA_OUT_EN_SDIO3 << MXC_F_SPIXFC_REVA_GEN_CTRL_BB_DATA_OUT_EN_POS) /**< GEN_CTRL_BB_DATA_OUT_EN_SDIO3 Setting */

 #define MXC_F_SPIXFC_REVA_GEN_CTRL_SIMPLE_POS          20 /**< GEN_CTRL_SIMPLE Position */
 #define MXC_F_SPIXFC_REVA_GEN_CTRL_SIMPLE              ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_GEN_CTRL_SIMPLE_POS)) /**< GEN_CTRL_SIMPLE Mask */

 #define MXC_F_SPIXFC_REVA_GEN_CTRL_SIMPLE_RX_POS       21 /**< GEN_CTRL_SIMPLE_RX Position */
 #define MXC_F_SPIXFC_REVA_GEN_CTRL_SIMPLE_RX           ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_GEN_CTRL_SIMPLE_RX_POS)) /**< GEN_CTRL_SIMPLE_RX Mask */

 #define MXC_F_SPIXFC_REVA_GEN_CTRL_SIMPLE_SS_POS       22 /**< GEN_CTRL_SIMPLE_SS Position */
 #define MXC_F_SPIXFC_REVA_GEN_CTRL_SIMPLE_SS           ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_GEN_CTRL_SIMPLE_SS_POS)) /**< GEN_CTRL_SIMPLE_SS Mask */

 #define MXC_F_SPIXFC_REVA_GEN_CTRL_SCLK_FB_POS         24 /**< GEN_CTRL_SCLK_FB Position */
 #define MXC_F_SPIXFC_REVA_GEN_CTRL_SCLK_FB             ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_GEN_CTRL_SCLK_FB_POS)) /**< GEN_CTRL_SCLK_FB Mask */

 #define MXC_F_SPIXFC_REVA_GEN_CTRL_SCLK_FB_INVERT_POS  25 /**< GEN_CTRL_SCLK_FB_INVERT Position */
 #define MXC_F_SPIXFC_REVA_GEN_CTRL_SCLK_FB_INVERT      ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_GEN_CTRL_SCLK_FB_INVERT_POS)) /**< GEN_CTRL_SCLK_FB_INVERT Mask */

/**@} end of group SPIXFC_REVA_GEN_CTRL_Register */

/**
 * @ingroup  spixfc_reva_registers
 * @defgroup SPIXFC_REVA_FIFO_CTRL SPIXFC_REVA_FIFO_CTRL
 * @brief    SPIX Controller FIFO Control and Status Register.
 * @{
 */
 #define MXC_F_SPIXFC_REVA_FIFO_CTRL_TX_FIFO_AE_LVL_POS 0 /**< FIFO_CTRL_TX_FIFO_AE_LVL Position */
 #define MXC_F_SPIXFC_REVA_FIFO_CTRL_TX_FIFO_AE_LVL     ((uint32_t)(0xFUL << MXC_F_SPIXFC_REVA_FIFO_CTRL_TX_FIFO_AE_LVL_POS)) /**< FIFO_CTRL_TX_FIFO_AE_LVL Mask */

 #define MXC_F_SPIXFC_REVA_FIFO_CTRL_TX_FIFO_CNT_POS    8 /**< FIFO_CTRL_TX_FIFO_CNT Position */
 #define MXC_F_SPIXFC_REVA_FIFO_CTRL_TX_FIFO_CNT        ((uint32_t)(0x1FUL << MXC_F_SPIXFC_REVA_FIFO_CTRL_TX_FIFO_CNT_POS)) /**< FIFO_CTRL_TX_FIFO_CNT Mask */

 #define MXC_F_SPIXFC_REVA_FIFO_CTRL_RX_FIFO_AF_LVL_POS 16 /**< FIFO_CTRL_RX_FIFO_AF_LVL Position */
 #define MXC_F_SPIXFC_REVA_FIFO_CTRL_RX_FIFO_AF_LVL     ((uint32_t)(0x1FUL << MXC_F_SPIXFC_REVA_FIFO_CTRL_RX_FIFO_AF_LVL_POS)) /**< FIFO_CTRL_RX_FIFO_AF_LVL Mask */

 #define MXC_F_SPIXFC_REVA_FIFO_CTRL_RX_FIFO_CNT_POS    24 /**< FIFO_CTRL_RX_FIFO_CNT Position */
 #define MXC_F_SPIXFC_REVA_FIFO_CTRL_RX_FIFO_CNT        ((uint32_t)(0x3FUL << MXC_F_SPIXFC_REVA_FIFO_CTRL_RX_FIFO_CNT_POS)) /**< FIFO_CTRL_RX_FIFO_CNT Mask */

/**@} end of group SPIXFC_REVA_FIFO_CTRL_Register */

/**
 * @ingroup  spixfc_reva_registers
 * @defgroup SPIXFC_REVA_SP_CTRL SPIXFC_REVA_SP_CTRL
 * @brief    SPIX Controller Special Control Register.
 * @{
 */
 #define MXC_F_SPIXFC_REVA_SP_CTRL_SAMPL_POS            0 /**< SP_CTRL_SAMPL Position */
 #define MXC_F_SPIXFC_REVA_SP_CTRL_SAMPL                ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_SP_CTRL_SAMPL_POS)) /**< SP_CTRL_SAMPL Mask */

 #define MXC_F_SPIXFC_REVA_SP_CTRL_SDIO_OUT_POS         4 /**< SP_CTRL_SDIO_OUT Position */
 #define MXC_F_SPIXFC_REVA_SP_CTRL_SDIO_OUT             ((uint32_t)(0xFUL << MXC_F_SPIXFC_REVA_SP_CTRL_SDIO_OUT_POS)) /**< SP_CTRL_SDIO_OUT Mask */

 #define MXC_F_SPIXFC_REVA_SP_CTRL_SDIO_OUT_EN_POS      8 /**< SP_CTRL_SDIO_OUT_EN Position */
 #define MXC_F_SPIXFC_REVA_SP_CTRL_SDIO_OUT_EN          ((uint32_t)(0xFUL << MXC_F_SPIXFC_REVA_SP_CTRL_SDIO_OUT_EN_POS)) /**< SP_CTRL_SDIO_OUT_EN Mask */

 #define MXC_F_SPIXFC_REVA_SP_CTRL_SCLKINH3_POS         16 /**< SP_CTRL_SCLKINH3 Position */
 #define MXC_F_SPIXFC_REVA_SP_CTRL_SCLKINH3             ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_SP_CTRL_SCLKINH3_POS)) /**< SP_CTRL_SCLKINH3 Mask */

/**@} end of group SPIXFC_REVA_SP_CTRL_Register */

/**
 * @ingroup  spixfc_reva_registers
 * @defgroup SPIXFC_REVA_INT_FL SPIXFC_REVA_INT_FL
 * @brief    SPIX Controller Interrupt Status Register.
 * @{
 */
 #define MXC_F_SPIXFC_REVA_INT_FL_TX_STALLED_POS        0 /**< INT_FL_TX_STALLED Position */
 #define MXC_F_SPIXFC_REVA_INT_FL_TX_STALLED            ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_INT_FL_TX_STALLED_POS)) /**< INT_FL_TX_STALLED Mask */

 #define MXC_F_SPIXFC_REVA_INT_FL_RX_STALLED_POS        1 /**< INT_FL_RX_STALLED Position */
 #define MXC_F_SPIXFC_REVA_INT_FL_RX_STALLED            ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_INT_FL_RX_STALLED_POS)) /**< INT_FL_RX_STALLED Mask */

 #define MXC_F_SPIXFC_REVA_INT_FL_TX_READY_POS          2 /**< INT_FL_TX_READY Position */
 #define MXC_F_SPIXFC_REVA_INT_FL_TX_READY              ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_INT_FL_TX_READY_POS)) /**< INT_FL_TX_READY Mask */

 #define MXC_F_SPIXFC_REVA_INT_FL_RX_DONE_POS           3 /**< INT_FL_RX_DONE Position */
 #define MXC_F_SPIXFC_REVA_INT_FL_RX_DONE               ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_INT_FL_RX_DONE_POS)) /**< INT_FL_RX_DONE Mask */

 #define MXC_F_SPIXFC_REVA_INT_FL_TX_FIFO_AE_POS        4 /**< INT_FL_TX_FIFO_AE Position */
 #define MXC_F_SPIXFC_REVA_INT_FL_TX_FIFO_AE            ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_INT_FL_TX_FIFO_AE_POS)) /**< INT_FL_TX_FIFO_AE Mask */

 #define MXC_F_SPIXFC_REVA_INT_FL_RX_FIFO_AF_POS        5 /**< INT_FL_RX_FIFO_AF Position */
 #define MXC_F_SPIXFC_REVA_INT_FL_RX_FIFO_AF            ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_INT_FL_RX_FIFO_AF_POS)) /**< INT_FL_RX_FIFO_AF Mask */

/**@} end of group SPIXFC_REVA_INT_FL_Register */

/**
 * @ingroup  spixfc_reva_registers
 * @defgroup SPIXFC_REVA_INT_EN SPIXFC_REVA_INT_EN
 * @brief    SPIX Controller Interrupt Enable Register.
 * @{
 */
 #define MXC_F_SPIXFC_REVA_INT_EN_TX_STALLED_POS        0 /**< INT_EN_TX_STALLED Position */
 #define MXC_F_SPIXFC_REVA_INT_EN_TX_STALLED            ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_INT_EN_TX_STALLED_POS)) /**< INT_EN_TX_STALLED Mask */

 #define MXC_F_SPIXFC_REVA_INT_EN_RX_STALLED_POS        1 /**< INT_EN_RX_STALLED Position */
 #define MXC_F_SPIXFC_REVA_INT_EN_RX_STALLED            ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_INT_EN_RX_STALLED_POS)) /**< INT_EN_RX_STALLED Mask */

 #define MXC_F_SPIXFC_REVA_INT_EN_TX_READY_POS          2 /**< INT_EN_TX_READY Position */
 #define MXC_F_SPIXFC_REVA_INT_EN_TX_READY              ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_INT_EN_TX_READY_POS)) /**< INT_EN_TX_READY Mask */

 #define MXC_F_SPIXFC_REVA_INT_EN_RX_DONE_POS           3 /**< INT_EN_RX_DONE Position */
 #define MXC_F_SPIXFC_REVA_INT_EN_RX_DONE               ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_INT_EN_RX_DONE_POS)) /**< INT_EN_RX_DONE Mask */

 #define MXC_F_SPIXFC_REVA_INT_EN_TX_FIFO_AE_POS        4 /**< INT_EN_TX_FIFO_AE Position */
 #define MXC_F_SPIXFC_REVA_INT_EN_TX_FIFO_AE            ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_INT_EN_TX_FIFO_AE_POS)) /**< INT_EN_TX_FIFO_AE Mask */

 #define MXC_F_SPIXFC_REVA_INT_EN_RX_FIFO_AF_POS        5 /**< INT_EN_RX_FIFO_AF Position */
 #define MXC_F_SPIXFC_REVA_INT_EN_RX_FIFO_AF            ((uint32_t)(0x1UL << MXC_F_SPIXFC_REVA_INT_EN_RX_FIFO_AF_POS)) /**< INT_EN_RX_FIFO_AF Mask */

/**@} end of group SPIXFC_REVA_INT_EN_Register */

#ifdef __cplusplus
}
#endif

#endif /* _SPIXFC_REVA_REGS_H_ */
