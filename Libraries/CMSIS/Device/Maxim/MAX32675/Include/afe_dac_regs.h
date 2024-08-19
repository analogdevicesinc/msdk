/**
 * @file    afe_dac_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the AFE_DAC Peripheral Module.
 * @note    This file is @generated.
 * @ingroup afe_dac_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32675_INCLUDE_AFE_DAC_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32675_INCLUDE_AFE_DAC_REGS_H_

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
 * @ingroup     afe_dac
 * @defgroup    afe_dac_registers AFE_DAC_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the AFE_DAC Peripheral Module.
 * @details     Analog Front End DAC on Stacked Die via SPI
 */

/* Register offsets for module AFE_DAC */
/**
 * @ingroup    afe_dac_registers
 * @defgroup   AFE_DAC_Register_Offsets Register Offsets
 * @brief      AFE_DAC Peripheral Register Offsets from the AFE_DAC Base Peripheral Address.
 * @{
 */
#define MXC_R_AFE_DAC_CTRL                 ((uint32_t)0x01000004UL) /**< Offset from AFE_DAC Base Address: <tt> 0x1000004</tt> */
#define MXC_R_AFE_DAC_RATE                 ((uint32_t)0x01010004UL) /**< Offset from AFE_DAC Base Address: <tt> 0x1010004</tt> */
#define MXC_R_AFE_DAC_INT                  ((uint32_t)0x01020004UL) /**< Offset from AFE_DAC Base Address: <tt> 0x1020004</tt> */
#define MXC_R_AFE_DAC_REG                  ((uint32_t)0x01030004UL) /**< Offset from AFE_DAC Base Address: <tt> 0x1030004</tt> */
#define MXC_R_AFE_DAC_TRIM                 ((uint32_t)0x01040004UL) /**< Offset from AFE_DAC Base Address: <tt> 0x1040004</tt> */
#define MXC_R_AFE_DAC_VREF_CTRL            ((uint32_t)0x01050002UL) /**< Offset from AFE_DAC Base Address: <tt> 0x1050002</tt> */
#define MXC_R_AFE_DAC_FIFO                 ((uint32_t)0x01060002UL) /**< Offset from AFE_DAC Base Address: <tt> 0x1060002</tt> */
#define MXC_R_AFE_DAC_VREF_TRIM            ((uint32_t)0x01070002UL) /**< Offset from AFE_DAC Base Address: <tt> 0x1070002</tt> */
/**@} end of group afe_dac_registers */

/**
 * @ingroup  afe_dac_registers
 * @defgroup AFE_DAC_CTRL AFE_DAC_CTRL
 * @brief    Control Register
 * @{
 */
#define MXC_F_AFE_DAC_CTRL_FIFO_AE_CNT_POS             0 /**< CTRL_FIFO_AE_CNT Position */
#define MXC_F_AFE_DAC_CTRL_FIFO_AE_CNT                 ((uint32_t)(0xFUL << MXC_F_AFE_DAC_CTRL_FIFO_AE_CNT_POS)) /**< CTRL_FIFO_AE_CNT Mask */

#define MXC_F_AFE_DAC_CTRL_FIFO_ALMOST_FULL_POS        5 /**< CTRL_FIFO_ALMOST_FULL Position */
#define MXC_F_AFE_DAC_CTRL_FIFO_ALMOST_FULL            ((uint32_t)(0x1UL << MXC_F_AFE_DAC_CTRL_FIFO_ALMOST_FULL_POS)) /**< CTRL_FIFO_ALMOST_FULL Mask */

#define MXC_F_AFE_DAC_CTRL_FIFO_EMPTY_POS              6 /**< CTRL_FIFO_EMPTY Position */
#define MXC_F_AFE_DAC_CTRL_FIFO_EMPTY                  ((uint32_t)(0x1UL << MXC_F_AFE_DAC_CTRL_FIFO_EMPTY_POS)) /**< CTRL_FIFO_EMPTY Mask */

#define MXC_F_AFE_DAC_CTRL_FIFO_ALMOST_EMPTY_POS       7 /**< CTRL_FIFO_ALMOST_EMPTY Position */
#define MXC_F_AFE_DAC_CTRL_FIFO_ALMOST_EMPTY           ((uint32_t)(0x1UL << MXC_F_AFE_DAC_CTRL_FIFO_ALMOST_EMPTY_POS)) /**< CTRL_FIFO_ALMOST_EMPTY Mask */

#define MXC_F_AFE_DAC_CTRL_INTERP_MODE_POS             8 /**< CTRL_INTERP_MODE Position */
#define MXC_F_AFE_DAC_CTRL_INTERP_MODE                 ((uint32_t)(0x7UL << MXC_F_AFE_DAC_CTRL_INTERP_MODE_POS)) /**< CTRL_INTERP_MODE Mask */
#define MXC_V_AFE_DAC_CTRL_INTERP_MODE_DISABLED        ((uint32_t)0x0UL) /**< CTRL_INTERP_MODE_DISABLED Value */
#define MXC_S_AFE_DAC_CTRL_INTERP_MODE_DISABLED        (MXC_V_AFE_DAC_CTRL_INTERP_MODE_DISABLED << MXC_F_AFE_DAC_CTRL_INTERP_MODE_POS) /**< CTRL_INTERP_MODE_DISABLED Setting */
#define MXC_V_AFE_DAC_CTRL_INTERP_MODE_2_TO_1_INTERPOLATION ((uint32_t)0x1UL) /**< CTRL_INTERP_MODE_2_TO_1_INTERPOLATION Value */
#define MXC_S_AFE_DAC_CTRL_INTERP_MODE_2_TO_1_INTERPOLATION (MXC_V_AFE_DAC_CTRL_INTERP_MODE_2_TO_1_INTERPOLATION << MXC_F_AFE_DAC_CTRL_INTERP_MODE_POS) /**< CTRL_INTERP_MODE_2_TO_1_INTERPOLATION Setting */
#define MXC_V_AFE_DAC_CTRL_INTERP_MODE_4_TO_1_INTERPOLATION ((uint32_t)0x2UL) /**< CTRL_INTERP_MODE_4_TO_1_INTERPOLATION Value */
#define MXC_S_AFE_DAC_CTRL_INTERP_MODE_4_TO_1_INTERPOLATION (MXC_V_AFE_DAC_CTRL_INTERP_MODE_4_TO_1_INTERPOLATION << MXC_F_AFE_DAC_CTRL_INTERP_MODE_POS) /**< CTRL_INTERP_MODE_4_TO_1_INTERPOLATION Setting */
#define MXC_V_AFE_DAC_CTRL_INTERP_MODE_8_TO_1_INTERPOLATION ((uint32_t)0x3UL) /**< CTRL_INTERP_MODE_8_TO_1_INTERPOLATION Value */
#define MXC_S_AFE_DAC_CTRL_INTERP_MODE_8_TO_1_INTERPOLATION (MXC_V_AFE_DAC_CTRL_INTERP_MODE_8_TO_1_INTERPOLATION << MXC_F_AFE_DAC_CTRL_INTERP_MODE_POS) /**< CTRL_INTERP_MODE_8_TO_1_INTERPOLATION Setting */

#define MXC_F_AFE_DAC_CTRL_FIFO_AF_CNT_POS             12 /**< CTRL_FIFO_AF_CNT Position */
#define MXC_F_AFE_DAC_CTRL_FIFO_AF_CNT                 ((uint32_t)(0xFUL << MXC_F_AFE_DAC_CTRL_FIFO_AF_CNT_POS)) /**< CTRL_FIFO_AF_CNT Mask */

#define MXC_F_AFE_DAC_CTRL_START_MODE_POS              16 /**< CTRL_START_MODE Position */
#define MXC_F_AFE_DAC_CTRL_START_MODE                  ((uint32_t)(0x3UL << MXC_F_AFE_DAC_CTRL_START_MODE_POS)) /**< CTRL_START_MODE Mask */
#define MXC_V_AFE_DAC_CTRL_START_MODE_START_WHEN_FIFO_NOT_EMPTY ((uint32_t)0x0UL) /**< CTRL_START_MODE_START_WHEN_FIFO_NOT_EMPTY Value */
#define MXC_S_AFE_DAC_CTRL_START_MODE_START_WHEN_FIFO_NOT_EMPTY (MXC_V_AFE_DAC_CTRL_START_MODE_START_WHEN_FIFO_NOT_EMPTY << MXC_F_AFE_DAC_CTRL_START_MODE_POS) /**< CTRL_START_MODE_START_WHEN_FIFO_NOT_EMPTY Setting */
#define MXC_V_AFE_DAC_CTRL_START_MODE_START_ON_ADC_START_STROBE ((uint32_t)0x1UL) /**< CTRL_START_MODE_START_ON_ADC_START_STROBE Value */
#define MXC_S_AFE_DAC_CTRL_START_MODE_START_ON_ADC_START_STROBE (MXC_V_AFE_DAC_CTRL_START_MODE_START_ON_ADC_START_STROBE << MXC_F_AFE_DAC_CTRL_START_MODE_POS) /**< CTRL_START_MODE_START_ON_ADC_START_STROBE Setting */
#define MXC_V_AFE_DAC_CTRL_START_MODE_START_WHEN_CPU_START_WRITTEN ((uint32_t)0x2UL) /**< CTRL_START_MODE_START_WHEN_CPU_START_WRITTEN Value */
#define MXC_S_AFE_DAC_CTRL_START_MODE_START_WHEN_CPU_START_WRITTEN (MXC_V_AFE_DAC_CTRL_START_MODE_START_WHEN_CPU_START_WRITTEN << MXC_F_AFE_DAC_CTRL_START_MODE_POS) /**< CTRL_START_MODE_START_WHEN_CPU_START_WRITTEN Setting */
#define MXC_V_AFE_DAC_CTRL_START_MODE_RESERVED         ((uint32_t)0x3UL) /**< CTRL_START_MODE_RESERVED Value */
#define MXC_S_AFE_DAC_CTRL_START_MODE_RESERVED         (MXC_V_AFE_DAC_CTRL_START_MODE_RESERVED << MXC_F_AFE_DAC_CTRL_START_MODE_POS) /**< CTRL_START_MODE_RESERVED Setting */

#define MXC_F_AFE_DAC_CTRL_ACTIVE_POS                  18 /**< CTRL_ACTIVE Position */
#define MXC_F_AFE_DAC_CTRL_ACTIVE                      ((uint32_t)(0x1UL << MXC_F_AFE_DAC_CTRL_ACTIVE_POS)) /**< CTRL_ACTIVE Mask */

#define MXC_F_AFE_DAC_CTRL_BIN2GRAY_BYPASS_POS         19 /**< CTRL_BIN2GRAY_BYPASS Position */
#define MXC_F_AFE_DAC_CTRL_BIN2GRAY_BYPASS             ((uint32_t)(0x1UL << MXC_F_AFE_DAC_CTRL_BIN2GRAY_BYPASS_POS)) /**< CTRL_BIN2GRAY_BYPASS Mask */

#define MXC_F_AFE_DAC_CTRL_CPU_START_POS               20 /**< CTRL_CPU_START Position */
#define MXC_F_AFE_DAC_CTRL_CPU_START                   ((uint32_t)(0x1UL << MXC_F_AFE_DAC_CTRL_CPU_START_POS)) /**< CTRL_CPU_START Mask */

#define MXC_F_AFE_DAC_CTRL_OP_MODE_POS                 24 /**< CTRL_OP_MODE Position */
#define MXC_F_AFE_DAC_CTRL_OP_MODE                     ((uint32_t)(0x3UL << MXC_F_AFE_DAC_CTRL_OP_MODE_POS)) /**< CTRL_OP_MODE Mask */
#define MXC_V_AFE_DAC_CTRL_OP_MODE_OUTPUT_WHEN_FIFO_AVAIL ((uint32_t)0x0UL) /**< CTRL_OP_MODE_OUTPUT_WHEN_FIFO_AVAIL Value */
#define MXC_S_AFE_DAC_CTRL_OP_MODE_OUTPUT_WHEN_FIFO_AVAIL (MXC_V_AFE_DAC_CTRL_OP_MODE_OUTPUT_WHEN_FIFO_AVAIL << MXC_F_AFE_DAC_CTRL_OP_MODE_POS) /**< CTRL_OP_MODE_OUTPUT_WHEN_FIFO_AVAIL Setting */
#define MXC_V_AFE_DAC_CTRL_OP_MODE_OUTPUT_ONCE_AT_RATE_CNT ((uint32_t)0x1UL) /**< CTRL_OP_MODE_OUTPUT_ONCE_AT_RATE_CNT Value */
#define MXC_S_AFE_DAC_CTRL_OP_MODE_OUTPUT_ONCE_AT_RATE_CNT (MXC_V_AFE_DAC_CTRL_OP_MODE_OUTPUT_ONCE_AT_RATE_CNT << MXC_F_AFE_DAC_CTRL_OP_MODE_POS) /**< CTRL_OP_MODE_OUTPUT_ONCE_AT_RATE_CNT Setting */
#define MXC_V_AFE_DAC_CTRL_OP_MODE_RESERVED            ((uint32_t)0x2UL) /**< CTRL_OP_MODE_RESERVED Value */
#define MXC_S_AFE_DAC_CTRL_OP_MODE_RESERVED            (MXC_V_AFE_DAC_CTRL_OP_MODE_RESERVED << MXC_F_AFE_DAC_CTRL_OP_MODE_POS) /**< CTRL_OP_MODE_RESERVED Setting */
#define MXC_V_AFE_DAC_CTRL_OP_MODE_OUTPUT_SAMPLE_CNT_AT_RATE_CNT ((uint32_t)0x3UL) /**< CTRL_OP_MODE_OUTPUT_SAMPLE_CNT_AT_RATE_CNT Value */
#define MXC_S_AFE_DAC_CTRL_OP_MODE_OUTPUT_SAMPLE_CNT_AT_RATE_CNT (MXC_V_AFE_DAC_CTRL_OP_MODE_OUTPUT_SAMPLE_CNT_AT_RATE_CNT << MXC_F_AFE_DAC_CTRL_OP_MODE_POS) /**< CTRL_OP_MODE_OUTPUT_SAMPLE_CNT_AT_RATE_CNT Setting */

#define MXC_F_AFE_DAC_CTRL_POWER_MODE_1_0_POS          26 /**< CTRL_POWER_MODE_1_0 Position */
#define MXC_F_AFE_DAC_CTRL_POWER_MODE_1_0              ((uint32_t)(0x3UL << MXC_F_AFE_DAC_CTRL_POWER_MODE_1_0_POS)) /**< CTRL_POWER_MODE_1_0 Mask */
#define MXC_V_AFE_DAC_CTRL_POWER_MODE_1_0_POWLVL0      ((uint32_t)0x1UL) /**< CTRL_POWER_MODE_1_0_POWLVL0 Value */
#define MXC_S_AFE_DAC_CTRL_POWER_MODE_1_0_POWLVL0      (MXC_V_AFE_DAC_CTRL_POWER_MODE_1_0_POWLVL0 << MXC_F_AFE_DAC_CTRL_POWER_MODE_1_0_POS) /**< CTRL_POWER_MODE_1_0_POWLVL0 Setting */
#define MXC_V_AFE_DAC_CTRL_POWER_MODE_1_0_POWLVL1      ((uint32_t)0x3UL) /**< CTRL_POWER_MODE_1_0_POWLVL1 Value */
#define MXC_S_AFE_DAC_CTRL_POWER_MODE_1_0_POWLVL1      (MXC_V_AFE_DAC_CTRL_POWER_MODE_1_0_POWLVL1 << MXC_F_AFE_DAC_CTRL_POWER_MODE_1_0_POS) /**< CTRL_POWER_MODE_1_0_POWLVL1 Setting */
#define MXC_V_AFE_DAC_CTRL_POWER_MODE_1_0_POWLVL2      ((uint32_t)0x1UL) /**< CTRL_POWER_MODE_1_0_POWLVL2 Value */
#define MXC_S_AFE_DAC_CTRL_POWER_MODE_1_0_POWLVL2      (MXC_V_AFE_DAC_CTRL_POWER_MODE_1_0_POWLVL2 << MXC_F_AFE_DAC_CTRL_POWER_MODE_1_0_POS) /**< CTRL_POWER_MODE_1_0_POWLVL2 Setting */
#define MXC_V_AFE_DAC_CTRL_POWER_MODE_1_0_POWLVL3      ((uint32_t)0x3UL) /**< CTRL_POWER_MODE_1_0_POWLVL3 Value */
#define MXC_S_AFE_DAC_CTRL_POWER_MODE_1_0_POWLVL3      (MXC_V_AFE_DAC_CTRL_POWER_MODE_1_0_POWLVL3 << MXC_F_AFE_DAC_CTRL_POWER_MODE_1_0_POS) /**< CTRL_POWER_MODE_1_0_POWLVL3 Setting */

#define MXC_F_AFE_DAC_CTRL_POWER_ON_POS                28 /**< CTRL_POWER_ON Position */
#define MXC_F_AFE_DAC_CTRL_POWER_ON                    ((uint32_t)(0x1UL << MXC_F_AFE_DAC_CTRL_POWER_ON_POS)) /**< CTRL_POWER_ON Mask */

#define MXC_F_AFE_DAC_CTRL_CLOCK_GATE_EN_POS           29 /**< CTRL_CLOCK_GATE_EN Position */
#define MXC_F_AFE_DAC_CTRL_CLOCK_GATE_EN               ((uint32_t)(0x1UL << MXC_F_AFE_DAC_CTRL_CLOCK_GATE_EN_POS)) /**< CTRL_CLOCK_GATE_EN Mask */

#define MXC_F_AFE_DAC_CTRL_POWER_MODE_2_POS            30 /**< CTRL_POWER_MODE_2 Position */
#define MXC_F_AFE_DAC_CTRL_POWER_MODE_2                ((uint32_t)(0x1UL << MXC_F_AFE_DAC_CTRL_POWER_MODE_2_POS)) /**< CTRL_POWER_MODE_2 Mask */

#define MXC_F_AFE_DAC_CTRL_RESET_POS                   31 /**< CTRL_RESET Position */
#define MXC_F_AFE_DAC_CTRL_RESET                       ((uint32_t)(0x1UL << MXC_F_AFE_DAC_CTRL_RESET_POS)) /**< CTRL_RESET Mask */

/**@} end of group AFE_DAC_CTRL_Register */

/**
 * @ingroup  afe_dac_registers
 * @defgroup AFE_DAC_RATE AFE_DAC_RATE
 * @brief    Rate/Sample Control
 * @{
 */
#define MXC_F_AFE_DAC_RATE_RATE_CNT_POS                0 /**< RATE_RATE_CNT Position */
#define MXC_F_AFE_DAC_RATE_RATE_CNT                    ((uint32_t)(0xFFFFUL << MXC_F_AFE_DAC_RATE_RATE_CNT_POS)) /**< RATE_RATE_CNT Mask */

#define MXC_F_AFE_DAC_RATE_SAMPLE_CNT_POS              16 /**< RATE_SAMPLE_CNT Position */
#define MXC_F_AFE_DAC_RATE_SAMPLE_CNT                  ((uint32_t)(0xFFFFUL << MXC_F_AFE_DAC_RATE_SAMPLE_CNT_POS)) /**< RATE_SAMPLE_CNT Mask */

/**@} end of group AFE_DAC_RATE_Register */

/**
 * @ingroup  afe_dac_registers
 * @defgroup AFE_DAC_INT AFE_DAC_INT
 * @brief    Interrupt Flags and Enable/Disable
 * @{
 */
#define MXC_F_AFE_DAC_INT_OUT_DONE_IF_POS              0 /**< INT_OUT_DONE_IF Position */
#define MXC_F_AFE_DAC_INT_OUT_DONE_IF                  ((uint32_t)(0x1UL << MXC_F_AFE_DAC_INT_OUT_DONE_IF_POS)) /**< INT_OUT_DONE_IF Mask */

#define MXC_F_AFE_DAC_INT_UNDERFLOW_IF_POS             1 /**< INT_UNDERFLOW_IF Position */
#define MXC_F_AFE_DAC_INT_UNDERFLOW_IF                 ((uint32_t)(0x1UL << MXC_F_AFE_DAC_INT_UNDERFLOW_IF_POS)) /**< INT_UNDERFLOW_IF Mask */

#define MXC_F_AFE_DAC_INT_ALMOST_EMPTY_IF_POS          2 /**< INT_ALMOST_EMPTY_IF Position */
#define MXC_F_AFE_DAC_INT_ALMOST_EMPTY_IF              ((uint32_t)(0x1UL << MXC_F_AFE_DAC_INT_ALMOST_EMPTY_IF_POS)) /**< INT_ALMOST_EMPTY_IF Mask */

#define MXC_F_AFE_DAC_INT_UNDERLFOW_POS                3 /**< INT_UNDERLFOW Position */
#define MXC_F_AFE_DAC_INT_UNDERLFOW                    ((uint32_t)(0x1UL << MXC_F_AFE_DAC_INT_UNDERLFOW_POS)) /**< INT_UNDERLFOW Mask */

#define MXC_F_AFE_DAC_INT_OUT_DONE_IE_POS              16 /**< INT_OUT_DONE_IE Position */
#define MXC_F_AFE_DAC_INT_OUT_DONE_IE                  ((uint32_t)(0x1UL << MXC_F_AFE_DAC_INT_OUT_DONE_IE_POS)) /**< INT_OUT_DONE_IE Mask */

#define MXC_F_AFE_DAC_INT_UNDERFLOW_IE_POS             17 /**< INT_UNDERFLOW_IE Position */
#define MXC_F_AFE_DAC_INT_UNDERFLOW_IE                 ((uint32_t)(0x1UL << MXC_F_AFE_DAC_INT_UNDERFLOW_IE_POS)) /**< INT_UNDERFLOW_IE Mask */

#define MXC_F_AFE_DAC_INT_AHB_CG_DISABLE_POS           28 /**< INT_AHB_CG_DISABLE Position */
#define MXC_F_AFE_DAC_INT_AHB_CG_DISABLE               ((uint32_t)(0x1UL << MXC_F_AFE_DAC_INT_AHB_CG_DISABLE_POS)) /**< INT_AHB_CG_DISABLE Mask */

#define MXC_F_AFE_DAC_INT_APB_CG_DISABLE_POS           29 /**< INT_APB_CG_DISABLE Position */
#define MXC_F_AFE_DAC_INT_APB_CG_DISABLE               ((uint32_t)(0x1UL << MXC_F_AFE_DAC_INT_APB_CG_DISABLE_POS)) /**< INT_APB_CG_DISABLE Mask */

/**@} end of group AFE_DAC_INT_Register */

/**
 * @ingroup  afe_dac_registers
 * @defgroup AFE_DAC_VREF_CTRL AFE_DAC_VREF_CTRL
 * @brief    DAC VREF Control
 * @{
 */
#define MXC_F_AFE_DAC_VREF_CTRL_REF_DAC_FAST_PD_POS    0 /**< VREF_CTRL_REF_DAC_FAST_PD Position */
#define MXC_F_AFE_DAC_VREF_CTRL_REF_DAC_FAST_PD        ((uint16_t)(0x1UL << MXC_F_AFE_DAC_VREF_CTRL_REF_DAC_FAST_PD_POS)) /**< VREF_CTRL_REF_DAC_FAST_PD Mask */

#define MXC_F_AFE_DAC_VREF_CTRL_DACREFSEL_POS          1 /**< VREF_CTRL_DACREFSEL Position */
#define MXC_F_AFE_DAC_VREF_CTRL_DACREFSEL              ((uint16_t)(0x3UL << MXC_F_AFE_DAC_VREF_CTRL_DACREFSEL_POS)) /**< VREF_CTRL_DACREFSEL Mask */
#define MXC_V_AFE_DAC_VREF_CTRL_DACREFSEL_VOLTS_1_024  ((uint16_t)0x0UL) /**< VREF_CTRL_DACREFSEL_VOLTS_1_024 Value */
#define MXC_S_AFE_DAC_VREF_CTRL_DACREFSEL_VOLTS_1_024  (MXC_V_AFE_DAC_VREF_CTRL_DACREFSEL_VOLTS_1_024 << MXC_F_AFE_DAC_VREF_CTRL_DACREFSEL_POS) /**< VREF_CTRL_DACREFSEL_VOLTS_1_024 Setting */
#define MXC_V_AFE_DAC_VREF_CTRL_DACREFSEL_VOLTS_1_500  ((uint16_t)0x1UL) /**< VREF_CTRL_DACREFSEL_VOLTS_1_500 Value */
#define MXC_S_AFE_DAC_VREF_CTRL_DACREFSEL_VOLTS_1_500  (MXC_V_AFE_DAC_VREF_CTRL_DACREFSEL_VOLTS_1_500 << MXC_F_AFE_DAC_VREF_CTRL_DACREFSEL_POS) /**< VREF_CTRL_DACREFSEL_VOLTS_1_500 Setting */
#define MXC_V_AFE_DAC_VREF_CTRL_DACREFSEL_VOLTS_2_048  ((uint16_t)0x2UL) /**< VREF_CTRL_DACREFSEL_VOLTS_2_048 Value */
#define MXC_S_AFE_DAC_VREF_CTRL_DACREFSEL_VOLTS_2_048  (MXC_V_AFE_DAC_VREF_CTRL_DACREFSEL_VOLTS_2_048 << MXC_F_AFE_DAC_VREF_CTRL_DACREFSEL_POS) /**< VREF_CTRL_DACREFSEL_VOLTS_2_048 Setting */
#define MXC_V_AFE_DAC_VREF_CTRL_DACREFSEL_VOLTS_2_500  ((uint16_t)0x3UL) /**< VREF_CTRL_DACREFSEL_VOLTS_2_500 Value */
#define MXC_S_AFE_DAC_VREF_CTRL_DACREFSEL_VOLTS_2_500  (MXC_V_AFE_DAC_VREF_CTRL_DACREFSEL_VOLTS_2_500 << MXC_F_AFE_DAC_VREF_CTRL_DACREFSEL_POS) /**< VREF_CTRL_DACREFSEL_VOLTS_2_500 Setting */

#define MXC_F_AFE_DAC_VREF_CTRL_REFDAC_OUTEN_POS       3 /**< VREF_CTRL_REFDAC_OUTEN Position */
#define MXC_F_AFE_DAC_VREF_CTRL_REFDAC_OUTEN           ((uint16_t)(0x1UL << MXC_F_AFE_DAC_VREF_CTRL_REFDAC_OUTEN_POS)) /**< VREF_CTRL_REFDAC_OUTEN Mask */

#define MXC_F_AFE_DAC_VREF_CTRL_REF_PU_POS             4 /**< VREF_CTRL_REF_PU Position */
#define MXC_F_AFE_DAC_VREF_CTRL_REF_PU                 ((uint16_t)(0x1UL << MXC_F_AFE_DAC_VREF_CTRL_REF_PU_POS)) /**< VREF_CTRL_REF_PU Mask */

#define MXC_F_AFE_DAC_VREF_CTRL_REFDAC_CP_POS          5 /**< VREF_CTRL_REFDAC_CP Position */
#define MXC_F_AFE_DAC_VREF_CTRL_REFDAC_CP              ((uint16_t)(0x1UL << MXC_F_AFE_DAC_VREF_CTRL_REFDAC_CP_POS)) /**< VREF_CTRL_REFDAC_CP Mask */

#define MXC_F_AFE_DAC_VREF_CTRL_REFDAC_GAIN_POS        6 /**< VREF_CTRL_REFDAC_GAIN Position */
#define MXC_F_AFE_DAC_VREF_CTRL_REFDAC_GAIN            ((uint16_t)(0x3UL << MXC_F_AFE_DAC_VREF_CTRL_REFDAC_GAIN_POS)) /**< VREF_CTRL_REFDAC_GAIN Mask */
#define MXC_V_AFE_DAC_VREF_CTRL_REFDAC_GAIN_DEFAULT_GAIN ((uint16_t)0x0UL) /**< VREF_CTRL_REFDAC_GAIN_DEFAULT_GAIN Value */
#define MXC_S_AFE_DAC_VREF_CTRL_REFDAC_GAIN_DEFAULT_GAIN (MXC_V_AFE_DAC_VREF_CTRL_REFDAC_GAIN_DEFAULT_GAIN << MXC_F_AFE_DAC_VREF_CTRL_REFDAC_GAIN_POS) /**< VREF_CTRL_REFDAC_GAIN_DEFAULT_GAIN Setting */
#define MXC_V_AFE_DAC_VREF_CTRL_REFDAC_GAIN_HIGHEST_GAIN ((uint16_t)0x1UL) /**< VREF_CTRL_REFDAC_GAIN_HIGHEST_GAIN Value */
#define MXC_S_AFE_DAC_VREF_CTRL_REFDAC_GAIN_HIGHEST_GAIN (MXC_V_AFE_DAC_VREF_CTRL_REFDAC_GAIN_HIGHEST_GAIN << MXC_F_AFE_DAC_VREF_CTRL_REFDAC_GAIN_POS) /**< VREF_CTRL_REFDAC_GAIN_HIGHEST_GAIN Setting */

#define MXC_F_AFE_DAC_VREF_CTRL_REF_ABUS_POS           8 /**< VREF_CTRL_REF_ABUS Position */
#define MXC_F_AFE_DAC_VREF_CTRL_REF_ABUS               ((uint16_t)(0xFUL << MXC_F_AFE_DAC_VREF_CTRL_REF_ABUS_POS)) /**< VREF_CTRL_REF_ABUS Mask */

/**@} end of group AFE_DAC_VREF_CTRL_Register */

/**
 * @ingroup  afe_dac_registers
 * @defgroup AFE_DAC_FIFO AFE_DAC_FIFO
 * @brief    DAC FIFO
 * @{
 */
#define MXC_F_AFE_DAC_FIFO_FIFO_DATA_POS               0 /**< FIFO_FIFO_DATA Position */
#define MXC_F_AFE_DAC_FIFO_FIFO_DATA                   ((uint16_t)(0xFFFFUL << MXC_F_AFE_DAC_FIFO_FIFO_DATA_POS)) /**< FIFO_FIFO_DATA Mask */

/**@} end of group AFE_DAC_FIFO_Register */

/**
 * @ingroup  afe_dac_registers
 * @defgroup AFE_DAC_VREF_TRIM AFE_DAC_VREF_TRIM
 * @brief    Control behavior of GPIO1
 * @{
 */
#define MXC_F_AFE_DAC_VREF_TRIM_V1_TRIM_POS            0 /**< VREF_TRIM_V1_TRIM Position */
#define MXC_F_AFE_DAC_VREF_TRIM_V1_TRIM                ((uint16_t)(0x1FUL << MXC_F_AFE_DAC_VREF_TRIM_V1_TRIM_POS)) /**< VREF_TRIM_V1_TRIM Mask */

#define MXC_F_AFE_DAC_VREF_TRIM_REF_BG_TRIM_POS        5 /**< VREF_TRIM_REF_BG_TRIM Position */
#define MXC_F_AFE_DAC_VREF_TRIM_REF_BG_TRIM            ((uint16_t)(0x3FUL << MXC_F_AFE_DAC_VREF_TRIM_REF_BG_TRIM_POS)) /**< VREF_TRIM_REF_BG_TRIM Mask */

/**@} end of group AFE_DAC_VREF_TRIM_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32675_INCLUDE_AFE_DAC_REGS_H_
