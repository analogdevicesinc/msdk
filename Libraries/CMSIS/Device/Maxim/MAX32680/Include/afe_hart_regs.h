/**
 * @file    afe_hart_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the AFE_HART Peripheral Module.
 * @note    This file is @generated.
 * @ingroup afe_hart_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32680_INCLUDE_AFE_HART_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32680_INCLUDE_AFE_HART_REGS_H_

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
 * @ingroup     afe_hart
 * @defgroup    afe_hart_registers AFE_HART_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the AFE_HART Peripheral Module.
 * @details     Analog Front End HART Registers on Stacked Die via SPI
 */

/* Register offsets for module AFE_HART */
/**
 * @ingroup    afe_hart_registers
 * @defgroup   AFE_HART_Register_Offsets Register Offsets
 * @brief      AFE_HART Peripheral Register Offsets from the AFE_HART Base Peripheral Address.
 * @{
 */
#define MXC_R_AFE_HART_CTRL                ((uint32_t)0x01800003UL) /**< Offset from AFE_HART Base Address: <tt> 0x1800003</tt> */
#define MXC_R_AFE_HART_RX_TX_CTL           ((uint32_t)0x01810003UL) /**< Offset from AFE_HART Base Address: <tt> 0x1810003</tt> */
#define MXC_R_AFE_HART_RX_CTL_EXT1         ((uint32_t)0x01820003UL) /**< Offset from AFE_HART Base Address: <tt> 0x1820003</tt> */
#define MXC_R_AFE_HART_RX_CTL_EXT2         ((uint32_t)0x01830003UL) /**< Offset from AFE_HART Base Address: <tt> 0x1830003</tt> */
#define MXC_R_AFE_HART_RX_DB_THRSHLD       ((uint32_t)0x01840003UL) /**< Offset from AFE_HART Base Address: <tt> 0x1840003</tt> */
#define MXC_R_AFE_HART_RX_CRD_UP_THRSHLD   ((uint32_t)0x01850003UL) /**< Offset from AFE_HART Base Address: <tt> 0x1850003</tt> */
#define MXC_R_AFE_HART_RX_CRD_DN_THRSHLD   ((uint32_t)0x01860003UL) /**< Offset from AFE_HART Base Address: <tt> 0x1860003</tt> */
#define MXC_R_AFE_HART_RX_CRD_DOUT_THRSHLD ((uint32_t)0x01870003UL) /**< Offset from AFE_HART Base Address: <tt> 0x1870003</tt> */
#define MXC_R_AFE_HART_TX_MARKSPACE_CNT    ((uint32_t)0x01880003UL) /**< Offset from AFE_HART Base Address: <tt> 0x1880003</tt> */
#define MXC_R_AFE_HART_STAT                ((uint32_t)0x01890003UL) /**< Offset from AFE_HART Base Address: <tt> 0x1890003</tt> */
#define MXC_R_AFE_HART_TRIM                ((uint32_t)0x018A0003UL) /**< Offset from AFE_HART Base Address: <tt> 0x18A0003</tt> */
#define MXC_R_AFE_HART_TM                  ((uint32_t)0x018B0003UL) /**< Offset from AFE_HART Base Address: <tt> 0x18B0003</tt> */
/**@} end of group afe_hart_registers */

/**
 * @ingroup  afe_hart_registers
 * @defgroup AFE_HART_CTRL AFE_HART_CTRL
 * @brief    HART Control
 * @{
 */
#define MXC_F_AFE_HART_CTRL_ADM_TM_EN_POS              0 /**< CTRL_ADM_TM_EN Position */
#define MXC_F_AFE_HART_CTRL_ADM_TM_EN                  ((uint32_t)(0x1UL << MXC_F_AFE_HART_CTRL_ADM_TM_EN_POS)) /**< CTRL_ADM_TM_EN Mask */

/**@} end of group AFE_HART_CTRL_Register */

/**
 * @ingroup  afe_hart_registers
 * @defgroup AFE_HART_RX_TX_CTL AFE_HART_RX_TX_CTL
 * @brief    Control HART Transmit and Receive Functions
 * @{
 */
#define MXC_F_AFE_HART_RX_TX_CTL_RX_ADC_REF_EN_POS     0 /**< RX_TX_CTL_RX_ADC_REF_EN Position */
#define MXC_F_AFE_HART_RX_TX_CTL_RX_ADC_REF_EN         ((uint32_t)(0x1UL << MXC_F_AFE_HART_RX_TX_CTL_RX_ADC_REF_EN_POS)) /**< RX_TX_CTL_RX_ADC_REF_EN Mask */

#define MXC_F_AFE_HART_RX_TX_CTL_RX_ADC_REFBUF_EN_POS  1 /**< RX_TX_CTL_RX_ADC_REFBUF_EN Position */
#define MXC_F_AFE_HART_RX_TX_CTL_RX_ADC_REFBUF_EN      ((uint32_t)(0x1UL << MXC_F_AFE_HART_RX_TX_CTL_RX_ADC_REFBUF_EN_POS)) /**< RX_TX_CTL_RX_ADC_REFBUF_EN Mask */

#define MXC_F_AFE_HART_RX_TX_CTL_RX_ADC_OFFSET_SEL_POS 2 /**< RX_TX_CTL_RX_ADC_OFFSET_SEL Position */
#define MXC_F_AFE_HART_RX_TX_CTL_RX_ADC_OFFSET_SEL     ((uint32_t)(0x1UL << MXC_F_AFE_HART_RX_TX_CTL_RX_ADC_OFFSET_SEL_POS)) /**< RX_TX_CTL_RX_ADC_OFFSET_SEL Mask */

#define MXC_F_AFE_HART_RX_TX_CTL_RX_DOUT_UART_EN_POS   3 /**< RX_TX_CTL_RX_DOUT_UART_EN Position */
#define MXC_F_AFE_HART_RX_TX_CTL_RX_DOUT_UART_EN       ((uint32_t)(0x1UL << MXC_F_AFE_HART_RX_TX_CTL_RX_DOUT_UART_EN_POS)) /**< RX_TX_CTL_RX_DOUT_UART_EN Mask */

#define MXC_F_AFE_HART_RX_TX_CTL_RX_ADC_PWR_UP_SMP_IGNR_POS 4 /**< RX_TX_CTL_RX_ADC_PWR_UP_SMP_IGNR Position */
#define MXC_F_AFE_HART_RX_TX_CTL_RX_ADC_PWR_UP_SMP_IGNR ((uint32_t)(0xFUL << MXC_F_AFE_HART_RX_TX_CTL_RX_ADC_PWR_UP_SMP_IGNR_POS)) /**< RX_TX_CTL_RX_ADC_PWR_UP_SMP_IGNR Mask */

#define MXC_F_AFE_HART_RX_TX_CTL_RX_BP_SETTLE_CNT_POS  8 /**< RX_TX_CTL_RX_BP_SETTLE_CNT Position */
#define MXC_F_AFE_HART_RX_TX_CTL_RX_BP_SETTLE_CNT      ((uint32_t)(0xFFUL << MXC_F_AFE_HART_RX_TX_CTL_RX_BP_SETTLE_CNT_POS)) /**< RX_TX_CTL_RX_BP_SETTLE_CNT Mask */

#define MXC_F_AFE_HART_RX_TX_CTL_RX_ADC_PWR_DLY_CNT_POS 16 /**< RX_TX_CTL_RX_ADC_PWR_DLY_CNT Position */
#define MXC_F_AFE_HART_RX_TX_CTL_RX_ADC_PWR_DLY_CNT    ((uint32_t)(0xFUL << MXC_F_AFE_HART_RX_TX_CTL_RX_ADC_PWR_DLY_CNT_POS)) /**< RX_TX_CTL_RX_ADC_PWR_DLY_CNT Mask */

#define MXC_F_AFE_HART_RX_TX_CTL_TX_BUF_EN_POS         20 /**< RX_TX_CTL_TX_BUF_EN Position */
#define MXC_F_AFE_HART_RX_TX_CTL_TX_BUF_EN             ((uint32_t)(0x1UL << MXC_F_AFE_HART_RX_TX_CTL_TX_BUF_EN_POS)) /**< RX_TX_CTL_TX_BUF_EN Mask */

#define MXC_F_AFE_HART_RX_TX_CTL_TX_BUS_DCL_EN_POS     21 /**< RX_TX_CTL_TX_BUS_DCL_EN Position */
#define MXC_F_AFE_HART_RX_TX_CTL_TX_BUS_DCL_EN         ((uint32_t)(0x1UL << MXC_F_AFE_HART_RX_TX_CTL_TX_BUS_DCL_EN_POS)) /**< RX_TX_CTL_TX_BUS_DCL_EN Mask */

#define MXC_F_AFE_HART_RX_TX_CTL_TX_WS_DIS_RS_POS      22 /**< RX_TX_CTL_TX_WS_DIS_RS Position */
#define MXC_F_AFE_HART_RX_TX_CTL_TX_WS_DIS_RS          ((uint32_t)(0x1UL << MXC_F_AFE_HART_RX_TX_CTL_TX_WS_DIS_RS_POS)) /**< RX_TX_CTL_TX_WS_DIS_RS Mask */

#define MXC_F_AFE_HART_RX_TX_CTL_TX_4MHZ_CLK_EN_POS    23 /**< RX_TX_CTL_TX_4MHZ_CLK_EN Position */
#define MXC_F_AFE_HART_RX_TX_CTL_TX_4MHZ_CLK_EN        ((uint32_t)(0x1UL << MXC_F_AFE_HART_RX_TX_CTL_TX_4MHZ_CLK_EN_POS)) /**< RX_TX_CTL_TX_4MHZ_CLK_EN Mask */

/**@} end of group AFE_HART_RX_TX_CTL_Register */

/**
 * @ingroup  afe_hart_registers
 * @defgroup AFE_HART_RX_CTL_EXT1 AFE_HART_RX_CTL_EXT1
 * @brief    Receive Control Extension Register 1
 * @{
 */
#define MXC_F_AFE_HART_RX_CTL_EXT1_RX_AN_INIT_VAL_POS  0 /**< RX_CTL_EXT1_RX_AN_INIT_VAL Position */
#define MXC_F_AFE_HART_RX_CTL_EXT1_RX_AN_INIT_VAL      ((uint32_t)(0x7FFFFUL << MXC_F_AFE_HART_RX_CTL_EXT1_RX_AN_INIT_VAL_POS)) /**< RX_CTL_EXT1_RX_AN_INIT_VAL Mask */

/**@} end of group AFE_HART_RX_CTL_EXT1_Register */

/**
 * @ingroup  afe_hart_registers
 * @defgroup AFE_HART_RX_CTL_EXT2 AFE_HART_RX_CTL_EXT2
 * @brief    Receive Control Extension Register 2
 * @{
 */
#define MXC_F_AFE_HART_RX_CTL_EXT2_RX_ARN_INIT_VAL_POS 0 /**< RX_CTL_EXT2_RX_ARN_INIT_VAL Position */
#define MXC_F_AFE_HART_RX_CTL_EXT2_RX_ARN_INIT_VAL     ((uint32_t)(0x7FFFUL << MXC_F_AFE_HART_RX_CTL_EXT2_RX_ARN_INIT_VAL_POS)) /**< RX_CTL_EXT2_RX_ARN_INIT_VAL Mask */

#define MXC_F_AFE_HART_RX_CTL_EXT2_RX_ZC_IGN_VAL_POS   16 /**< RX_CTL_EXT2_RX_ZC_IGN_VAL Position */
#define MXC_F_AFE_HART_RX_CTL_EXT2_RX_ZC_IGN_VAL       ((uint32_t)(0x3UL << MXC_F_AFE_HART_RX_CTL_EXT2_RX_ZC_IGN_VAL_POS)) /**< RX_CTL_EXT2_RX_ZC_IGN_VAL Mask */

#define MXC_F_AFE_HART_RX_CTL_EXT2_RX_UART_TIMER_SYN_ALWS_EN_POS 20 /**< RX_CTL_EXT2_RX_UART_TIMER_SYN_ALWS_EN Position */
#define MXC_F_AFE_HART_RX_CTL_EXT2_RX_UART_TIMER_SYN_ALWS_EN ((uint32_t)(0x1UL << MXC_F_AFE_HART_RX_CTL_EXT2_RX_UART_TIMER_SYN_ALWS_EN_POS)) /**< RX_CTL_EXT2_RX_UART_TIMER_SYN_ALWS_EN Mask */

#define MXC_F_AFE_HART_RX_CTL_EXT2_RX_UART_TIMER_FAST_CNT_EN_POS 21 /**< RX_CTL_EXT2_RX_UART_TIMER_FAST_CNT_EN Position */
#define MXC_F_AFE_HART_RX_CTL_EXT2_RX_UART_TIMER_FAST_CNT_EN ((uint32_t)(0x1UL << MXC_F_AFE_HART_RX_CTL_EXT2_RX_UART_TIMER_FAST_CNT_EN_POS)) /**< RX_CTL_EXT2_RX_UART_TIMER_FAST_CNT_EN Mask */

/**@} end of group AFE_HART_RX_CTL_EXT2_Register */

/**
 * @ingroup  afe_hart_registers
 * @defgroup AFE_HART_RX_DB_THRSHLD AFE_HART_RX_DB_THRSHLD
 * @brief    Receive Bit-Detect/Demodulation Threshold
 * @{
 */
#define MXC_F_AFE_HART_RX_DB_THRSHLD_RX_BITDTCT_DN_THRSHLD_POS 0 /**< RX_DB_THRSHLD_RX_BITDTCT_DN_THRSHLD Position */
#define MXC_F_AFE_HART_RX_DB_THRSHLD_RX_BITDTCT_DN_THRSHLD ((uint32_t)(0x1FFUL << MXC_F_AFE_HART_RX_DB_THRSHLD_RX_BITDTCT_DN_THRSHLD_POS)) /**< RX_DB_THRSHLD_RX_BITDTCT_DN_THRSHLD Mask */

#define MXC_F_AFE_HART_RX_DB_THRSHLD_RX_BITDTCT_UP_THRSHLD_POS 12 /**< RX_DB_THRSHLD_RX_BITDTCT_UP_THRSHLD Position */
#define MXC_F_AFE_HART_RX_DB_THRSHLD_RX_BITDTCT_UP_THRSHLD ((uint32_t)(0x1FFUL << MXC_F_AFE_HART_RX_DB_THRSHLD_RX_BITDTCT_UP_THRSHLD_POS)) /**< RX_DB_THRSHLD_RX_BITDTCT_UP_THRSHLD Mask */

/**@} end of group AFE_HART_RX_DB_THRSHLD_Register */

/**
 * @ingroup  afe_hart_registers
 * @defgroup AFE_HART_RX_CRD_UP_THRSHLD AFE_HART_RX_CRD_UP_THRSHLD
 * @brief    Receive Carrier Detect Up Threshold Register
 * @{
 */
#define MXC_F_AFE_HART_RX_CRD_UP_THRSHLD_RX_CRD_UP_THRSHLD_POS 0 /**< RX_CRD_UP_THRSHLD_RX_CRD_UP_THRSHLD Position */
#define MXC_F_AFE_HART_RX_CRD_UP_THRSHLD_RX_CRD_UP_THRSHLD ((uint32_t)(0x7FFFFUL << MXC_F_AFE_HART_RX_CRD_UP_THRSHLD_RX_CRD_UP_THRSHLD_POS)) /**< RX_CRD_UP_THRSHLD_RX_CRD_UP_THRSHLD Mask */

/**@} end of group AFE_HART_RX_CRD_UP_THRSHLD_Register */

/**
 * @ingroup  afe_hart_registers
 * @defgroup AFE_HART_RX_CRD_DN_THRSHLD AFE_HART_RX_CRD_DN_THRSHLD
 * @brief    Receive Carrier Detect Down Threshold Register
 * @{
 */
#define MXC_F_AFE_HART_RX_CRD_DN_THRSHLD_RX_CRD_DN_THRSHLD_POS 0 /**< RX_CRD_DN_THRSHLD_RX_CRD_DN_THRSHLD Position */
#define MXC_F_AFE_HART_RX_CRD_DN_THRSHLD_RX_CRD_DN_THRSHLD ((uint32_t)(0x7FFFFUL << MXC_F_AFE_HART_RX_CRD_DN_THRSHLD_RX_CRD_DN_THRSHLD_POS)) /**< RX_CRD_DN_THRSHLD_RX_CRD_DN_THRSHLD Mask */

/**@} end of group AFE_HART_RX_CRD_DN_THRSHLD_Register */

/**
 * @ingroup  afe_hart_registers
 * @defgroup AFE_HART_RX_CRD_DOUT_THRSHLD AFE_HART_RX_CRD_DOUT_THRSHLD
 * @brief    Receive Carrier Detect DOUT Threshold
 * @{
 */
#define MXC_F_AFE_HART_RX_CRD_DOUT_THRSHLD_RX_CRD_DOUT_THRSHLD_POS 0 /**< RX_CRD_DOUT_THRSHLD_RX_CRD_DOUT_THRSHLD Position */
#define MXC_F_AFE_HART_RX_CRD_DOUT_THRSHLD_RX_CRD_DOUT_THRSHLD ((uint32_t)(0x7FFFFUL << MXC_F_AFE_HART_RX_CRD_DOUT_THRSHLD_RX_CRD_DOUT_THRSHLD_POS)) /**< RX_CRD_DOUT_THRSHLD_RX_CRD_DOUT_THRSHLD Mask */

/**@} end of group AFE_HART_RX_CRD_DOUT_THRSHLD_Register */

/**
 * @ingroup  afe_hart_registers
 * @defgroup AFE_HART_TX_MARKSPACE_CNT AFE_HART_TX_MARKSPACE_CNT
 * @brief    Transmit Mark-Space Count Values
 * @{
 */
#define MXC_F_AFE_HART_TX_MARKSPACE_CNT_TX_SPACE_CNT_POS 0 /**< TX_MARKSPACE_CNT_TX_SPACE_CNT Position */
#define MXC_F_AFE_HART_TX_MARKSPACE_CNT_TX_SPACE_CNT   ((uint32_t)(0x3FFUL << MXC_F_AFE_HART_TX_MARKSPACE_CNT_TX_SPACE_CNT_POS)) /**< TX_MARKSPACE_CNT_TX_SPACE_CNT Mask */

#define MXC_F_AFE_HART_TX_MARKSPACE_CNT_TX_MARK_CNT_POS 12 /**< TX_MARKSPACE_CNT_TX_MARK_CNT Position */
#define MXC_F_AFE_HART_TX_MARKSPACE_CNT_TX_MARK_CNT    ((uint32_t)(0x3FFUL << MXC_F_AFE_HART_TX_MARKSPACE_CNT_TX_MARK_CNT_POS)) /**< TX_MARKSPACE_CNT_TX_MARK_CNT Mask */

/**@} end of group AFE_HART_TX_MARKSPACE_CNT_Register */

/**
 * @ingroup  afe_hart_registers
 * @defgroup AFE_HART_TRIM AFE_HART_TRIM
 * @brief    HART Trim Register
 * @{
 */
#define MXC_F_AFE_HART_TRIM_TRIM_BIAS_POS              0 /**< TRIM_TRIM_BIAS Position */
#define MXC_F_AFE_HART_TRIM_TRIM_BIAS                  ((uint32_t)(0x1FUL << MXC_F_AFE_HART_TRIM_TRIM_BIAS_POS)) /**< TRIM_TRIM_BIAS Mask */

#define MXC_F_AFE_HART_TRIM_TRIM_BG_POS                8 /**< TRIM_TRIM_BG Position */
#define MXC_F_AFE_HART_TRIM_TRIM_BG                    ((uint32_t)(0x3FUL << MXC_F_AFE_HART_TRIM_TRIM_BG_POS)) /**< TRIM_TRIM_BG Mask */

#define MXC_F_AFE_HART_TRIM_TRIM_TX_SR_POS             16 /**< TRIM_TRIM_TX_SR Position */
#define MXC_F_AFE_HART_TRIM_TRIM_TX_SR                 ((uint32_t)(0xFUL << MXC_F_AFE_HART_TRIM_TRIM_TX_SR_POS)) /**< TRIM_TRIM_TX_SR Mask */

/**@} end of group AFE_HART_TRIM_Register */

/**
 * @ingroup  afe_hart_registers
 * @defgroup AFE_HART_TM AFE_HART_TM
 * @brief    Testmode
 * @{
 */
#define MXC_F_AFE_HART_TM_TM_EN_POS                    0 /**< TM_TM_EN Position */
#define MXC_F_AFE_HART_TM_TM_EN                        ((uint32_t)(0x1UL << MXC_F_AFE_HART_TM_TM_EN_POS)) /**< TM_TM_EN Mask */

#define MXC_F_AFE_HART_TM_TM_BIAS_EN_POS               1 /**< TM_TM_BIAS_EN Position */
#define MXC_F_AFE_HART_TM_TM_BIAS_EN                   ((uint32_t)(0x1UL << MXC_F_AFE_HART_TM_TM_BIAS_EN_POS)) /**< TM_TM_BIAS_EN Mask */

#define MXC_F_AFE_HART_TM_TM_BG_EN_POS                 3 /**< TM_TM_BG_EN Position */
#define MXC_F_AFE_HART_TM_TM_BG_EN                     ((uint32_t)(0x1UL << MXC_F_AFE_HART_TM_TM_BG_EN_POS)) /**< TM_TM_BG_EN Mask */

#define MXC_F_AFE_HART_TM_TM_VREF_EN_POS               3 /**< TM_TM_VREF_EN Position */
#define MXC_F_AFE_HART_TM_TM_VREF_EN                   ((uint32_t)(0x1UL << MXC_F_AFE_HART_TM_TM_VREF_EN_POS)) /**< TM_TM_VREF_EN Mask */

/**@} end of group AFE_HART_TM_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32680_INCLUDE_AFE_HART_REGS_H_
