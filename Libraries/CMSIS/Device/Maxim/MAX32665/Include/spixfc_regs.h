/**
 * @file    spixfc_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SPIXFC Peripheral Module.
 * @note    This file is @generated.
 * @ingroup spixfc_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_SPIXFC_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_SPIXFC_REGS_H_

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
 * @ingroup     spixfc
 * @defgroup    spixfc_registers SPIXFC_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SPIXFC Peripheral Module.
 * @details     SPI XiP Flash Configuration Controller
 */

/**
 * @ingroup spixfc_registers
 * Structure type to access the SPIXFC Registers.
 */
typedef struct {
    __IO uint32_t config;               /**< <tt>\b 0x00:</tt> SPIXFC CONFIG Register */
    __IO uint32_t ss_pol;               /**< <tt>\b 0x04:</tt> SPIXFC SS_POL Register */
    __IO uint32_t gen_ctrl;             /**< <tt>\b 0x08:</tt> SPIXFC GEN_CTRL Register */
    __IO uint32_t fifo_ctrl;            /**< <tt>\b 0x0C:</tt> SPIXFC FIFO_CTRL Register */
    __IO uint32_t spctrl;               /**< <tt>\b 0x10:</tt> SPIXFC SPCTRL Register */
    __IO uint32_t intfl;                /**< <tt>\b 0x14:</tt> SPIXFC INTFL Register */
    __IO uint32_t inten;                /**< <tt>\b 0x18:</tt> SPIXFC INTEN Register */
} mxc_spixfc_regs_t;

/* Register offsets for module SPIXFC */
/**
 * @ingroup    spixfc_registers
 * @defgroup   SPIXFC_Register_Offsets Register Offsets
 * @brief      SPIXFC Peripheral Register Offsets from the SPIXFC Base Peripheral Address.
 * @{
 */
#define MXC_R_SPIXFC_CONFIG                ((uint32_t)0x00000000UL) /**< Offset from SPIXFC Base Address: <tt> 0x0000</tt> */
#define MXC_R_SPIXFC_SS_POL                ((uint32_t)0x00000004UL) /**< Offset from SPIXFC Base Address: <tt> 0x0004</tt> */
#define MXC_R_SPIXFC_GEN_CTRL              ((uint32_t)0x00000008UL) /**< Offset from SPIXFC Base Address: <tt> 0x0008</tt> */
#define MXC_R_SPIXFC_FIFO_CTRL             ((uint32_t)0x0000000CUL) /**< Offset from SPIXFC Base Address: <tt> 0x000C</tt> */
#define MXC_R_SPIXFC_SPCTRL                ((uint32_t)0x00000010UL) /**< Offset from SPIXFC Base Address: <tt> 0x0010</tt> */
#define MXC_R_SPIXFC_INTFL                 ((uint32_t)0x00000014UL) /**< Offset from SPIXFC Base Address: <tt> 0x0014</tt> */
#define MXC_R_SPIXFC_INTEN                 ((uint32_t)0x00000018UL) /**< Offset from SPIXFC Base Address: <tt> 0x0018</tt> */
/**@} end of group spixfc_registers */

/**
 * @ingroup  spixfc_registers
 * @defgroup SPIXFC_CONFIG SPIXFC_CONFIG
 * @brief    Configuration Register.
 * @{
 */
#define MXC_F_SPIXFC_CONFIG_SSEL_POS                   0 /**< CONFIG_SSEL Position */
#define MXC_F_SPIXFC_CONFIG_SSEL                       ((uint32_t)(0x7UL << MXC_F_SPIXFC_CONFIG_SSEL_POS)) /**< CONFIG_SSEL Mask */
#define MXC_V_SPIXFC_CONFIG_SSEL_SLAVE_0               ((uint32_t)0x0UL) /**< CONFIG_SSEL_SLAVE_0 Value */
#define MXC_S_SPIXFC_CONFIG_SSEL_SLAVE_0               (MXC_V_SPIXFC_CONFIG_SSEL_SLAVE_0 << MXC_F_SPIXFC_CONFIG_SSEL_POS) /**< CONFIG_SSEL_SLAVE_0 Setting */
#define MXC_V_SPIXFC_CONFIG_SSEL_SLAVE_1               ((uint32_t)0x1UL) /**< CONFIG_SSEL_SLAVE_1 Value */
#define MXC_S_SPIXFC_CONFIG_SSEL_SLAVE_1               (MXC_V_SPIXFC_CONFIG_SSEL_SLAVE_1 << MXC_F_SPIXFC_CONFIG_SSEL_POS) /**< CONFIG_SSEL_SLAVE_1 Setting */

#define MXC_F_SPIXFC_CONFIG_MODE_POS                   4 /**< CONFIG_MODE Position */
#define MXC_F_SPIXFC_CONFIG_MODE                       ((uint32_t)(0x3UL << MXC_F_SPIXFC_CONFIG_MODE_POS)) /**< CONFIG_MODE Mask */
#define MXC_V_SPIXFC_CONFIG_MODE_SPIX_MODE_0           ((uint32_t)0x0UL) /**< CONFIG_MODE_SPIX_MODE_0 Value */
#define MXC_S_SPIXFC_CONFIG_MODE_SPIX_MODE_0           (MXC_V_SPIXFC_CONFIG_MODE_SPIX_MODE_0 << MXC_F_SPIXFC_CONFIG_MODE_POS) /**< CONFIG_MODE_SPIX_MODE_0 Setting */
#define MXC_V_SPIXFC_CONFIG_MODE_SPIX_MODE_3           ((uint32_t)0x3UL) /**< CONFIG_MODE_SPIX_MODE_3 Value */
#define MXC_S_SPIXFC_CONFIG_MODE_SPIX_MODE_3           (MXC_V_SPIXFC_CONFIG_MODE_SPIX_MODE_3 << MXC_F_SPIXFC_CONFIG_MODE_POS) /**< CONFIG_MODE_SPIX_MODE_3 Setting */

#define MXC_F_SPIXFC_CONFIG_PAGE_SIZE_POS              6 /**< CONFIG_PAGE_SIZE Position */
#define MXC_F_SPIXFC_CONFIG_PAGE_SIZE                  ((uint32_t)(0x3UL << MXC_F_SPIXFC_CONFIG_PAGE_SIZE_POS)) /**< CONFIG_PAGE_SIZE Mask */
#define MXC_V_SPIXFC_CONFIG_PAGE_SIZE_4_BYTES          ((uint32_t)0x0UL) /**< CONFIG_PAGE_SIZE_4_BYTES Value */
#define MXC_S_SPIXFC_CONFIG_PAGE_SIZE_4_BYTES          (MXC_V_SPIXFC_CONFIG_PAGE_SIZE_4_BYTES << MXC_F_SPIXFC_CONFIG_PAGE_SIZE_POS) /**< CONFIG_PAGE_SIZE_4_BYTES Setting */
#define MXC_V_SPIXFC_CONFIG_PAGE_SIZE_8_BYTES          ((uint32_t)0x1UL) /**< CONFIG_PAGE_SIZE_8_BYTES Value */
#define MXC_S_SPIXFC_CONFIG_PAGE_SIZE_8_BYTES          (MXC_V_SPIXFC_CONFIG_PAGE_SIZE_8_BYTES << MXC_F_SPIXFC_CONFIG_PAGE_SIZE_POS) /**< CONFIG_PAGE_SIZE_8_BYTES Setting */
#define MXC_V_SPIXFC_CONFIG_PAGE_SIZE_16_BYTES         ((uint32_t)0x2UL) /**< CONFIG_PAGE_SIZE_16_BYTES Value */
#define MXC_S_SPIXFC_CONFIG_PAGE_SIZE_16_BYTES         (MXC_V_SPIXFC_CONFIG_PAGE_SIZE_16_BYTES << MXC_F_SPIXFC_CONFIG_PAGE_SIZE_POS) /**< CONFIG_PAGE_SIZE_16_BYTES Setting */
#define MXC_V_SPIXFC_CONFIG_PAGE_SIZE_32_BYTES         ((uint32_t)0x3UL) /**< CONFIG_PAGE_SIZE_32_BYTES Value */
#define MXC_S_SPIXFC_CONFIG_PAGE_SIZE_32_BYTES         (MXC_V_SPIXFC_CONFIG_PAGE_SIZE_32_BYTES << MXC_F_SPIXFC_CONFIG_PAGE_SIZE_POS) /**< CONFIG_PAGE_SIZE_32_BYTES Setting */

#define MXC_F_SPIXFC_CONFIG_HI_CLK_POS                 8 /**< CONFIG_HI_CLK Position */
#define MXC_F_SPIXFC_CONFIG_HI_CLK                     ((uint32_t)(0xFUL << MXC_F_SPIXFC_CONFIG_HI_CLK_POS)) /**< CONFIG_HI_CLK Mask */
#define MXC_V_SPIXFC_CONFIG_HI_CLK_16_SCLK             ((uint32_t)0x0UL) /**< CONFIG_HI_CLK_16_SCLK Value */
#define MXC_S_SPIXFC_CONFIG_HI_CLK_16_SCLK             (MXC_V_SPIXFC_CONFIG_HI_CLK_16_SCLK << MXC_F_SPIXFC_CONFIG_HI_CLK_POS) /**< CONFIG_HI_CLK_16_SCLK Setting */

#define MXC_F_SPIXFC_CONFIG_LO_CLK_POS                 12 /**< CONFIG_LO_CLK Position */
#define MXC_F_SPIXFC_CONFIG_LO_CLK                     ((uint32_t)(0xFUL << MXC_F_SPIXFC_CONFIG_LO_CLK_POS)) /**< CONFIG_LO_CLK Mask */
#define MXC_V_SPIXFC_CONFIG_LO_CLK_16_SCLK             ((uint32_t)0x0UL) /**< CONFIG_LO_CLK_16_SCLK Value */
#define MXC_S_SPIXFC_CONFIG_LO_CLK_16_SCLK             (MXC_V_SPIXFC_CONFIG_LO_CLK_16_SCLK << MXC_F_SPIXFC_CONFIG_LO_CLK_POS) /**< CONFIG_LO_CLK_16_SCLK Setting */

#define MXC_F_SPIXFC_CONFIG_SS_ACT_POS                 16 /**< CONFIG_SS_ACT Position */
#define MXC_F_SPIXFC_CONFIG_SS_ACT                     ((uint32_t)(0x3UL << MXC_F_SPIXFC_CONFIG_SS_ACT_POS)) /**< CONFIG_SS_ACT Mask */
#define MXC_V_SPIXFC_CONFIG_SS_ACT_0_CLKS              ((uint32_t)0x0UL) /**< CONFIG_SS_ACT_0_CLKS Value */
#define MXC_S_SPIXFC_CONFIG_SS_ACT_0_CLKS              (MXC_V_SPIXFC_CONFIG_SS_ACT_0_CLKS << MXC_F_SPIXFC_CONFIG_SS_ACT_POS) /**< CONFIG_SS_ACT_0_CLKS Setting */
#define MXC_V_SPIXFC_CONFIG_SS_ACT_2_CLKS              ((uint32_t)0x1UL) /**< CONFIG_SS_ACT_2_CLKS Value */
#define MXC_S_SPIXFC_CONFIG_SS_ACT_2_CLKS              (MXC_V_SPIXFC_CONFIG_SS_ACT_2_CLKS << MXC_F_SPIXFC_CONFIG_SS_ACT_POS) /**< CONFIG_SS_ACT_2_CLKS Setting */
#define MXC_V_SPIXFC_CONFIG_SS_ACT_4_CLKS              ((uint32_t)0x2UL) /**< CONFIG_SS_ACT_4_CLKS Value */
#define MXC_S_SPIXFC_CONFIG_SS_ACT_4_CLKS              (MXC_V_SPIXFC_CONFIG_SS_ACT_4_CLKS << MXC_F_SPIXFC_CONFIG_SS_ACT_POS) /**< CONFIG_SS_ACT_4_CLKS Setting */
#define MXC_V_SPIXFC_CONFIG_SS_ACT_8_CLKS              ((uint32_t)0x3UL) /**< CONFIG_SS_ACT_8_CLKS Value */
#define MXC_S_SPIXFC_CONFIG_SS_ACT_8_CLKS              (MXC_V_SPIXFC_CONFIG_SS_ACT_8_CLKS << MXC_F_SPIXFC_CONFIG_SS_ACT_POS) /**< CONFIG_SS_ACT_8_CLKS Setting */

#define MXC_F_SPIXFC_CONFIG_SS_INACT_POS               18 /**< CONFIG_SS_INACT Position */
#define MXC_F_SPIXFC_CONFIG_SS_INACT                   ((uint32_t)(0x3UL << MXC_F_SPIXFC_CONFIG_SS_INACT_POS)) /**< CONFIG_SS_INACT Mask */
#define MXC_V_SPIXFC_CONFIG_SS_INACT_4_CLKS            ((uint32_t)0x0UL) /**< CONFIG_SS_INACT_4_CLKS Value */
#define MXC_S_SPIXFC_CONFIG_SS_INACT_4_CLKS            (MXC_V_SPIXFC_CONFIG_SS_INACT_4_CLKS << MXC_F_SPIXFC_CONFIG_SS_INACT_POS) /**< CONFIG_SS_INACT_4_CLKS Setting */
#define MXC_V_SPIXFC_CONFIG_SS_INACT_6_CLKS            ((uint32_t)0x1UL) /**< CONFIG_SS_INACT_6_CLKS Value */
#define MXC_S_SPIXFC_CONFIG_SS_INACT_6_CLKS            (MXC_V_SPIXFC_CONFIG_SS_INACT_6_CLKS << MXC_F_SPIXFC_CONFIG_SS_INACT_POS) /**< CONFIG_SS_INACT_6_CLKS Setting */
#define MXC_V_SPIXFC_CONFIG_SS_INACT_8_CLKS            ((uint32_t)0x2UL) /**< CONFIG_SS_INACT_8_CLKS Value */
#define MXC_S_SPIXFC_CONFIG_SS_INACT_8_CLKS            (MXC_V_SPIXFC_CONFIG_SS_INACT_8_CLKS << MXC_F_SPIXFC_CONFIG_SS_INACT_POS) /**< CONFIG_SS_INACT_8_CLKS Setting */
#define MXC_V_SPIXFC_CONFIG_SS_INACT_12_CLKS           ((uint32_t)0x3UL) /**< CONFIG_SS_INACT_12_CLKS Value */
#define MXC_S_SPIXFC_CONFIG_SS_INACT_12_CLKS           (MXC_V_SPIXFC_CONFIG_SS_INACT_12_CLKS << MXC_F_SPIXFC_CONFIG_SS_INACT_POS) /**< CONFIG_SS_INACT_12_CLKS Setting */

#define MXC_F_SPIXFC_CONFIG_IOSMPL_POS                 20 /**< CONFIG_IOSMPL Position */
#define MXC_F_SPIXFC_CONFIG_IOSMPL                     ((uint32_t)(0xFUL << MXC_F_SPIXFC_CONFIG_IOSMPL_POS)) /**< CONFIG_IOSMPL Mask */

/**@} end of group SPIXFC_CONFIG_Register */

/**
 * @ingroup  spixfc_registers
 * @defgroup SPIXFC_SS_POL SPIXFC_SS_POL
 * @brief    SPIX Controller Slave Select Polarity Register.
 * @{
 */
#define MXC_F_SPIXFC_SS_POL_SSPOL_0_POS                0 /**< SS_POL_SSPOL_0 Position */
#define MXC_F_SPIXFC_SS_POL_SSPOL_0                    ((uint32_t)(0x1UL << MXC_F_SPIXFC_SS_POL_SSPOL_0_POS)) /**< SS_POL_SSPOL_0 Mask */

/**@} end of group SPIXFC_SS_POL_Register */

/**
 * @ingroup  spixfc_registers
 * @defgroup SPIXFC_GEN_CTRL SPIXFC_GEN_CTRL
 * @brief    SPIX Controller General Controller Register.
 * @{
 */
#define MXC_F_SPIXFC_GEN_CTRL_ENABLE_POS               0 /**< GEN_CTRL_ENABLE Position */
#define MXC_F_SPIXFC_GEN_CTRL_ENABLE                   ((uint32_t)(0x1UL << MXC_F_SPIXFC_GEN_CTRL_ENABLE_POS)) /**< GEN_CTRL_ENABLE Mask */

#define MXC_F_SPIXFC_GEN_CTRL_TX_FIFO_EN_POS           1 /**< GEN_CTRL_TX_FIFO_EN Position */
#define MXC_F_SPIXFC_GEN_CTRL_TX_FIFO_EN               ((uint32_t)(0x1UL << MXC_F_SPIXFC_GEN_CTRL_TX_FIFO_EN_POS)) /**< GEN_CTRL_TX_FIFO_EN Mask */

#define MXC_F_SPIXFC_GEN_CTRL_RX_FIFO_EN_POS           2 /**< GEN_CTRL_RX_FIFO_EN Position */
#define MXC_F_SPIXFC_GEN_CTRL_RX_FIFO_EN               ((uint32_t)(0x1UL << MXC_F_SPIXFC_GEN_CTRL_RX_FIFO_EN_POS)) /**< GEN_CTRL_RX_FIFO_EN Mask */

#define MXC_F_SPIXFC_GEN_CTRL_BBMODE_POS               3 /**< GEN_CTRL_BBMODE Position */
#define MXC_F_SPIXFC_GEN_CTRL_BBMODE                   ((uint32_t)(0x1UL << MXC_F_SPIXFC_GEN_CTRL_BBMODE_POS)) /**< GEN_CTRL_BBMODE Mask */

#define MXC_F_SPIXFC_GEN_CTRL_SSDR_POS                 4 /**< GEN_CTRL_SSDR Position */
#define MXC_F_SPIXFC_GEN_CTRL_SSDR                     ((uint32_t)(0x1UL << MXC_F_SPIXFC_GEN_CTRL_SSDR_POS)) /**< GEN_CTRL_SSDR Mask */

#define MXC_F_SPIXFC_GEN_CTRL_SCLK_DR_POS              6 /**< GEN_CTRL_SCLK_DR Position */
#define MXC_F_SPIXFC_GEN_CTRL_SCLK_DR                  ((uint32_t)(0x1UL << MXC_F_SPIXFC_GEN_CTRL_SCLK_DR_POS)) /**< GEN_CTRL_SCLK_DR Mask */

#define MXC_F_SPIXFC_GEN_CTRL_SDIO_DATA_IN_POS         8 /**< GEN_CTRL_SDIO_DATA_IN Position */
#define MXC_F_SPIXFC_GEN_CTRL_SDIO_DATA_IN             ((uint32_t)(0xFUL << MXC_F_SPIXFC_GEN_CTRL_SDIO_DATA_IN_POS)) /**< GEN_CTRL_SDIO_DATA_IN Mask */
#define MXC_V_SPIXFC_GEN_CTRL_SDIO_DATA_IN_SDIO0       ((uint32_t)0x0UL) /**< GEN_CTRL_SDIO_DATA_IN_SDIO0 Value */
#define MXC_S_SPIXFC_GEN_CTRL_SDIO_DATA_IN_SDIO0       (MXC_V_SPIXFC_GEN_CTRL_SDIO_DATA_IN_SDIO0 << MXC_F_SPIXFC_GEN_CTRL_SDIO_DATA_IN_POS) /**< GEN_CTRL_SDIO_DATA_IN_SDIO0 Setting */
#define MXC_V_SPIXFC_GEN_CTRL_SDIO_DATA_IN_SDIO1       ((uint32_t)0x1UL) /**< GEN_CTRL_SDIO_DATA_IN_SDIO1 Value */
#define MXC_S_SPIXFC_GEN_CTRL_SDIO_DATA_IN_SDIO1       (MXC_V_SPIXFC_GEN_CTRL_SDIO_DATA_IN_SDIO1 << MXC_F_SPIXFC_GEN_CTRL_SDIO_DATA_IN_POS) /**< GEN_CTRL_SDIO_DATA_IN_SDIO1 Setting */
#define MXC_V_SPIXFC_GEN_CTRL_SDIO_DATA_IN_SDIO2       ((uint32_t)0x2UL) /**< GEN_CTRL_SDIO_DATA_IN_SDIO2 Value */
#define MXC_S_SPIXFC_GEN_CTRL_SDIO_DATA_IN_SDIO2       (MXC_V_SPIXFC_GEN_CTRL_SDIO_DATA_IN_SDIO2 << MXC_F_SPIXFC_GEN_CTRL_SDIO_DATA_IN_POS) /**< GEN_CTRL_SDIO_DATA_IN_SDIO2 Setting */
#define MXC_V_SPIXFC_GEN_CTRL_SDIO_DATA_IN_SDIO3       ((uint32_t)0x3UL) /**< GEN_CTRL_SDIO_DATA_IN_SDIO3 Value */
#define MXC_S_SPIXFC_GEN_CTRL_SDIO_DATA_IN_SDIO3       (MXC_V_SPIXFC_GEN_CTRL_SDIO_DATA_IN_SDIO3 << MXC_F_SPIXFC_GEN_CTRL_SDIO_DATA_IN_POS) /**< GEN_CTRL_SDIO_DATA_IN_SDIO3 Setting */

#define MXC_F_SPIXFC_GEN_CTRL_BB_DATA_POS              12 /**< GEN_CTRL_BB_DATA Position */
#define MXC_F_SPIXFC_GEN_CTRL_BB_DATA                  ((uint32_t)(0xFUL << MXC_F_SPIXFC_GEN_CTRL_BB_DATA_POS)) /**< GEN_CTRL_BB_DATA Mask */
#define MXC_V_SPIXFC_GEN_CTRL_BB_DATA_SDIO0            ((uint32_t)0x0UL) /**< GEN_CTRL_BB_DATA_SDIO0 Value */
#define MXC_S_SPIXFC_GEN_CTRL_BB_DATA_SDIO0            (MXC_V_SPIXFC_GEN_CTRL_BB_DATA_SDIO0 << MXC_F_SPIXFC_GEN_CTRL_BB_DATA_POS) /**< GEN_CTRL_BB_DATA_SDIO0 Setting */
#define MXC_V_SPIXFC_GEN_CTRL_BB_DATA_SDIO1            ((uint32_t)0x1UL) /**< GEN_CTRL_BB_DATA_SDIO1 Value */
#define MXC_S_SPIXFC_GEN_CTRL_BB_DATA_SDIO1            (MXC_V_SPIXFC_GEN_CTRL_BB_DATA_SDIO1 << MXC_F_SPIXFC_GEN_CTRL_BB_DATA_POS) /**< GEN_CTRL_BB_DATA_SDIO1 Setting */
#define MXC_V_SPIXFC_GEN_CTRL_BB_DATA_SDIO2            ((uint32_t)0x2UL) /**< GEN_CTRL_BB_DATA_SDIO2 Value */
#define MXC_S_SPIXFC_GEN_CTRL_BB_DATA_SDIO2            (MXC_V_SPIXFC_GEN_CTRL_BB_DATA_SDIO2 << MXC_F_SPIXFC_GEN_CTRL_BB_DATA_POS) /**< GEN_CTRL_BB_DATA_SDIO2 Setting */
#define MXC_V_SPIXFC_GEN_CTRL_BB_DATA_SDIO3            ((uint32_t)0x3UL) /**< GEN_CTRL_BB_DATA_SDIO3 Value */
#define MXC_S_SPIXFC_GEN_CTRL_BB_DATA_SDIO3            (MXC_V_SPIXFC_GEN_CTRL_BB_DATA_SDIO3 << MXC_F_SPIXFC_GEN_CTRL_BB_DATA_POS) /**< GEN_CTRL_BB_DATA_SDIO3 Setting */

#define MXC_F_SPIXFC_GEN_CTRL_BB_DATA_OUT_EN_POS       16 /**< GEN_CTRL_BB_DATA_OUT_EN Position */
#define MXC_F_SPIXFC_GEN_CTRL_BB_DATA_OUT_EN           ((uint32_t)(0xFUL << MXC_F_SPIXFC_GEN_CTRL_BB_DATA_OUT_EN_POS)) /**< GEN_CTRL_BB_DATA_OUT_EN Mask */
#define MXC_V_SPIXFC_GEN_CTRL_BB_DATA_OUT_EN_SDIO0     ((uint32_t)0x0UL) /**< GEN_CTRL_BB_DATA_OUT_EN_SDIO0 Value */
#define MXC_S_SPIXFC_GEN_CTRL_BB_DATA_OUT_EN_SDIO0     (MXC_V_SPIXFC_GEN_CTRL_BB_DATA_OUT_EN_SDIO0 << MXC_F_SPIXFC_GEN_CTRL_BB_DATA_OUT_EN_POS) /**< GEN_CTRL_BB_DATA_OUT_EN_SDIO0 Setting */
#define MXC_V_SPIXFC_GEN_CTRL_BB_DATA_OUT_EN_SDIO1     ((uint32_t)0x1UL) /**< GEN_CTRL_BB_DATA_OUT_EN_SDIO1 Value */
#define MXC_S_SPIXFC_GEN_CTRL_BB_DATA_OUT_EN_SDIO1     (MXC_V_SPIXFC_GEN_CTRL_BB_DATA_OUT_EN_SDIO1 << MXC_F_SPIXFC_GEN_CTRL_BB_DATA_OUT_EN_POS) /**< GEN_CTRL_BB_DATA_OUT_EN_SDIO1 Setting */
#define MXC_V_SPIXFC_GEN_CTRL_BB_DATA_OUT_EN_SDIO2     ((uint32_t)0x2UL) /**< GEN_CTRL_BB_DATA_OUT_EN_SDIO2 Value */
#define MXC_S_SPIXFC_GEN_CTRL_BB_DATA_OUT_EN_SDIO2     (MXC_V_SPIXFC_GEN_CTRL_BB_DATA_OUT_EN_SDIO2 << MXC_F_SPIXFC_GEN_CTRL_BB_DATA_OUT_EN_POS) /**< GEN_CTRL_BB_DATA_OUT_EN_SDIO2 Setting */
#define MXC_V_SPIXFC_GEN_CTRL_BB_DATA_OUT_EN_SDIO3     ((uint32_t)0x3UL) /**< GEN_CTRL_BB_DATA_OUT_EN_SDIO3 Value */
#define MXC_S_SPIXFC_GEN_CTRL_BB_DATA_OUT_EN_SDIO3     (MXC_V_SPIXFC_GEN_CTRL_BB_DATA_OUT_EN_SDIO3 << MXC_F_SPIXFC_GEN_CTRL_BB_DATA_OUT_EN_POS) /**< GEN_CTRL_BB_DATA_OUT_EN_SDIO3 Setting */

#define MXC_F_SPIXFC_GEN_CTRL_SIMPLE_POS               20 /**< GEN_CTRL_SIMPLE Position */
#define MXC_F_SPIXFC_GEN_CTRL_SIMPLE                   ((uint32_t)(0x1UL << MXC_F_SPIXFC_GEN_CTRL_SIMPLE_POS)) /**< GEN_CTRL_SIMPLE Mask */

#define MXC_F_SPIXFC_GEN_CTRL_SIMPLERX_POS             21 /**< GEN_CTRL_SIMPLERX Position */
#define MXC_F_SPIXFC_GEN_CTRL_SIMPLERX                 ((uint32_t)(0x1UL << MXC_F_SPIXFC_GEN_CTRL_SIMPLERX_POS)) /**< GEN_CTRL_SIMPLERX Mask */

#define MXC_F_SPIXFC_GEN_CTRL_SMPLSS_POS               22 /**< GEN_CTRL_SMPLSS Position */
#define MXC_F_SPIXFC_GEN_CTRL_SMPLSS                   ((uint32_t)(0x1UL << MXC_F_SPIXFC_GEN_CTRL_SMPLSS_POS)) /**< GEN_CTRL_SMPLSS Mask */

#define MXC_F_SPIXFC_GEN_CTRL_SCLK_FB_POS              24 /**< GEN_CTRL_SCLK_FB Position */
#define MXC_F_SPIXFC_GEN_CTRL_SCLK_FB                  ((uint32_t)(0x1UL << MXC_F_SPIXFC_GEN_CTRL_SCLK_FB_POS)) /**< GEN_CTRL_SCLK_FB Mask */

#define MXC_F_SPIXFC_GEN_CTRL_SCKFBINV_POS             25 /**< GEN_CTRL_SCKFBINV Position */
#define MXC_F_SPIXFC_GEN_CTRL_SCKFBINV                 ((uint32_t)(0x1UL << MXC_F_SPIXFC_GEN_CTRL_SCKFBINV_POS)) /**< GEN_CTRL_SCKFBINV Mask */

/**@} end of group SPIXFC_GEN_CTRL_Register */

/**
 * @ingroup  spixfc_registers
 * @defgroup SPIXFC_FIFO_CTRL SPIXFC_FIFO_CTRL
 * @brief    SPIX Controller FIFO Control and Status Register.
 * @{
 */
#define MXC_F_SPIXFC_FIFO_CTRL_TX_FIFO_AE_LVL_POS      0 /**< FIFO_CTRL_TX_FIFO_AE_LVL Position */
#define MXC_F_SPIXFC_FIFO_CTRL_TX_FIFO_AE_LVL          ((uint32_t)(0xFUL << MXC_F_SPIXFC_FIFO_CTRL_TX_FIFO_AE_LVL_POS)) /**< FIFO_CTRL_TX_FIFO_AE_LVL Mask */

#define MXC_F_SPIXFC_FIFO_CTRL_TX_FIFO_CNT_POS         8 /**< FIFO_CTRL_TX_FIFO_CNT Position */
#define MXC_F_SPIXFC_FIFO_CTRL_TX_FIFO_CNT             ((uint32_t)(0x1FUL << MXC_F_SPIXFC_FIFO_CTRL_TX_FIFO_CNT_POS)) /**< FIFO_CTRL_TX_FIFO_CNT Mask */

#define MXC_F_SPIXFC_FIFO_CTRL_RX_FIFO_AF_LVL_POS      16 /**< FIFO_CTRL_RX_FIFO_AF_LVL Position */
#define MXC_F_SPIXFC_FIFO_CTRL_RX_FIFO_AF_LVL          ((uint32_t)(0x1FUL << MXC_F_SPIXFC_FIFO_CTRL_RX_FIFO_AF_LVL_POS)) /**< FIFO_CTRL_RX_FIFO_AF_LVL Mask */

#define MXC_F_SPIXFC_FIFO_CTRL_RX_FIFO_CNT_POS         24 /**< FIFO_CTRL_RX_FIFO_CNT Position */
#define MXC_F_SPIXFC_FIFO_CTRL_RX_FIFO_CNT             ((uint32_t)(0x3FUL << MXC_F_SPIXFC_FIFO_CTRL_RX_FIFO_CNT_POS)) /**< FIFO_CTRL_RX_FIFO_CNT Mask */

/**@} end of group SPIXFC_FIFO_CTRL_Register */

/**
 * @ingroup  spixfc_registers
 * @defgroup SPIXFC_SPCTRL SPIXFC_SPCTRL
 * @brief    SPIX Controller Special Control Register.
 * @{
 */
#define MXC_F_SPIXFC_SPCTRL_SAMPL_POS                  0 /**< SPCTRL_SAMPL Position */
#define MXC_F_SPIXFC_SPCTRL_SAMPL                      ((uint32_t)(0x1UL << MXC_F_SPIXFC_SPCTRL_SAMPL_POS)) /**< SPCTRL_SAMPL Mask */

#define MXC_F_SPIXFC_SPCTRL_SDIOOUT_POS                4 /**< SPCTRL_SDIOOUT Position */
#define MXC_F_SPIXFC_SPCTRL_SDIOOUT                    ((uint32_t)(0xFUL << MXC_F_SPIXFC_SPCTRL_SDIOOUT_POS)) /**< SPCTRL_SDIOOUT Mask */
#define MXC_V_SPIXFC_SPCTRL_SDIOOUT_SDIO0              ((uint32_t)0x0UL) /**< SPCTRL_SDIOOUT_SDIO0 Value */
#define MXC_S_SPIXFC_SPCTRL_SDIOOUT_SDIO0              (MXC_V_SPIXFC_SPCTRL_SDIOOUT_SDIO0 << MXC_F_SPIXFC_SPCTRL_SDIOOUT_POS) /**< SPCTRL_SDIOOUT_SDIO0 Setting */
#define MXC_V_SPIXFC_SPCTRL_SDIOOUT_SDIO1              ((uint32_t)0x1UL) /**< SPCTRL_SDIOOUT_SDIO1 Value */
#define MXC_S_SPIXFC_SPCTRL_SDIOOUT_SDIO1              (MXC_V_SPIXFC_SPCTRL_SDIOOUT_SDIO1 << MXC_F_SPIXFC_SPCTRL_SDIOOUT_POS) /**< SPCTRL_SDIOOUT_SDIO1 Setting */
#define MXC_V_SPIXFC_SPCTRL_SDIOOUT_SDIO2              ((uint32_t)0x2UL) /**< SPCTRL_SDIOOUT_SDIO2 Value */
#define MXC_S_SPIXFC_SPCTRL_SDIOOUT_SDIO2              (MXC_V_SPIXFC_SPCTRL_SDIOOUT_SDIO2 << MXC_F_SPIXFC_SPCTRL_SDIOOUT_POS) /**< SPCTRL_SDIOOUT_SDIO2 Setting */
#define MXC_V_SPIXFC_SPCTRL_SDIOOUT_SDIO3              ((uint32_t)0x3UL) /**< SPCTRL_SDIOOUT_SDIO3 Value */
#define MXC_S_SPIXFC_SPCTRL_SDIOOUT_SDIO3              (MXC_V_SPIXFC_SPCTRL_SDIOOUT_SDIO3 << MXC_F_SPIXFC_SPCTRL_SDIOOUT_POS) /**< SPCTRL_SDIOOUT_SDIO3 Setting */

#define MXC_F_SPIXFC_SPCTRL_SDIOOE_POS                 8 /**< SPCTRL_SDIOOE Position */
#define MXC_F_SPIXFC_SPCTRL_SDIOOE                     ((uint32_t)(0xFUL << MXC_F_SPIXFC_SPCTRL_SDIOOE_POS)) /**< SPCTRL_SDIOOE Mask */
#define MXC_V_SPIXFC_SPCTRL_SDIOOE_SDIO0               ((uint32_t)0x0UL) /**< SPCTRL_SDIOOE_SDIO0 Value */
#define MXC_S_SPIXFC_SPCTRL_SDIOOE_SDIO0               (MXC_V_SPIXFC_SPCTRL_SDIOOE_SDIO0 << MXC_F_SPIXFC_SPCTRL_SDIOOE_POS) /**< SPCTRL_SDIOOE_SDIO0 Setting */
#define MXC_V_SPIXFC_SPCTRL_SDIOOE_SDIO1               ((uint32_t)0x1UL) /**< SPCTRL_SDIOOE_SDIO1 Value */
#define MXC_S_SPIXFC_SPCTRL_SDIOOE_SDIO1               (MXC_V_SPIXFC_SPCTRL_SDIOOE_SDIO1 << MXC_F_SPIXFC_SPCTRL_SDIOOE_POS) /**< SPCTRL_SDIOOE_SDIO1 Setting */
#define MXC_V_SPIXFC_SPCTRL_SDIOOE_SDIO2               ((uint32_t)0x2UL) /**< SPCTRL_SDIOOE_SDIO2 Value */
#define MXC_S_SPIXFC_SPCTRL_SDIOOE_SDIO2               (MXC_V_SPIXFC_SPCTRL_SDIOOE_SDIO2 << MXC_F_SPIXFC_SPCTRL_SDIOOE_POS) /**< SPCTRL_SDIOOE_SDIO2 Setting */
#define MXC_V_SPIXFC_SPCTRL_SDIOOE_SDIO3               ((uint32_t)0x3UL) /**< SPCTRL_SDIOOE_SDIO3 Value */
#define MXC_S_SPIXFC_SPCTRL_SDIOOE_SDIO3               (MXC_V_SPIXFC_SPCTRL_SDIOOE_SDIO3 << MXC_F_SPIXFC_SPCTRL_SDIOOE_POS) /**< SPCTRL_SDIOOE_SDIO3 Setting */

#define MXC_F_SPIXFC_SPCTRL_SCLKINH3_POS               16 /**< SPCTRL_SCLKINH3 Position */
#define MXC_F_SPIXFC_SPCTRL_SCLKINH3                   ((uint32_t)(0x1UL << MXC_F_SPIXFC_SPCTRL_SCLKINH3_POS)) /**< SPCTRL_SCLKINH3 Mask */

/**@} end of group SPIXFC_SPCTRL_Register */

/**
 * @ingroup  spixfc_registers
 * @defgroup SPIXFC_INTFL SPIXFC_INTFL
 * @brief    SPIX Controller Interrupt Status Register.
 * @{
 */
#define MXC_F_SPIXFC_INTFL_TX_STALLED_POS              0 /**< INTFL_TX_STALLED Position */
#define MXC_F_SPIXFC_INTFL_TX_STALLED                  ((uint32_t)(0x1UL << MXC_F_SPIXFC_INTFL_TX_STALLED_POS)) /**< INTFL_TX_STALLED Mask */

#define MXC_F_SPIXFC_INTFL_RX_STALLED_POS              1 /**< INTFL_RX_STALLED Position */
#define MXC_F_SPIXFC_INTFL_RX_STALLED                  ((uint32_t)(0x1UL << MXC_F_SPIXFC_INTFL_RX_STALLED_POS)) /**< INTFL_RX_STALLED Mask */

#define MXC_F_SPIXFC_INTFL_TX_READY_POS                2 /**< INTFL_TX_READY Position */
#define MXC_F_SPIXFC_INTFL_TX_READY                    ((uint32_t)(0x1UL << MXC_F_SPIXFC_INTFL_TX_READY_POS)) /**< INTFL_TX_READY Mask */

#define MXC_F_SPIXFC_INTFL_RX_DONE_POS                 3 /**< INTFL_RX_DONE Position */
#define MXC_F_SPIXFC_INTFL_RX_DONE                     ((uint32_t)(0x1UL << MXC_F_SPIXFC_INTFL_RX_DONE_POS)) /**< INTFL_RX_DONE Mask */

#define MXC_F_SPIXFC_INTFL_TX_FIFO_AE_POS              4 /**< INTFL_TX_FIFO_AE Position */
#define MXC_F_SPIXFC_INTFL_TX_FIFO_AE                  ((uint32_t)(0x1UL << MXC_F_SPIXFC_INTFL_TX_FIFO_AE_POS)) /**< INTFL_TX_FIFO_AE Mask */

#define MXC_F_SPIXFC_INTFL_RX_FIFO_AF_POS              5 /**< INTFL_RX_FIFO_AF Position */
#define MXC_F_SPIXFC_INTFL_RX_FIFO_AF                  ((uint32_t)(0x1UL << MXC_F_SPIXFC_INTFL_RX_FIFO_AF_POS)) /**< INTFL_RX_FIFO_AF Mask */

/**@} end of group SPIXFC_INTFL_Register */

/**
 * @ingroup  spixfc_registers
 * @defgroup SPIXFC_INTEN SPIXFC_INTEN
 * @brief    SPIX Controller Interrupt Enable Register.
 * @{
 */
#define MXC_F_SPIXFC_INTEN_TX_STALLED_POS              0 /**< INTEN_TX_STALLED Position */
#define MXC_F_SPIXFC_INTEN_TX_STALLED                  ((uint32_t)(0x1UL << MXC_F_SPIXFC_INTEN_TX_STALLED_POS)) /**< INTEN_TX_STALLED Mask */

#define MXC_F_SPIXFC_INTEN_RX_STALLED_POS              1 /**< INTEN_RX_STALLED Position */
#define MXC_F_SPIXFC_INTEN_RX_STALLED                  ((uint32_t)(0x1UL << MXC_F_SPIXFC_INTEN_RX_STALLED_POS)) /**< INTEN_RX_STALLED Mask */

#define MXC_F_SPIXFC_INTEN_TX_READY_POS                2 /**< INTEN_TX_READY Position */
#define MXC_F_SPIXFC_INTEN_TX_READY                    ((uint32_t)(0x1UL << MXC_F_SPIXFC_INTEN_TX_READY_POS)) /**< INTEN_TX_READY Mask */

#define MXC_F_SPIXFC_INTEN_RX_DONE_POS                 3 /**< INTEN_RX_DONE Position */
#define MXC_F_SPIXFC_INTEN_RX_DONE                     ((uint32_t)(0x1UL << MXC_F_SPIXFC_INTEN_RX_DONE_POS)) /**< INTEN_RX_DONE Mask */

#define MXC_F_SPIXFC_INTEN_TX_FIFO_AE_POS              4 /**< INTEN_TX_FIFO_AE Position */
#define MXC_F_SPIXFC_INTEN_TX_FIFO_AE                  ((uint32_t)(0x1UL << MXC_F_SPIXFC_INTEN_TX_FIFO_AE_POS)) /**< INTEN_TX_FIFO_AE Mask */

#define MXC_F_SPIXFC_INTEN_RX_FIFO_AF_POS              5 /**< INTEN_RX_FIFO_AF Position */
#define MXC_F_SPIXFC_INTEN_RX_FIFO_AF                  ((uint32_t)(0x1UL << MXC_F_SPIXFC_INTEN_RX_FIFO_AF_POS)) /**< INTEN_RX_FIFO_AF Mask */

/**@} end of group SPIXFC_INTEN_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_SPIXFC_REGS_H_
