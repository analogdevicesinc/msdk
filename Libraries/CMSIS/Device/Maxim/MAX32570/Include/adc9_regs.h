/**
 * @file    adc9_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the ADC9 Peripheral Module.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_ADC9_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_ADC9_REGS_H_

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
 * @ingroup     adc9
 * @defgroup    adc9_registers ADC9_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the ADC9 Peripheral Module.
 * @details     Magnetic Strip Reader - 9 bit ADC
 */

/**
 * @ingroup adc9_registers
 * Structure type to access the ADC9 Registers.
 */
typedef struct {
    __IO uint32_t cfg;                  /**< <tt>\b 0x0000:</tt> ADC9 CFG Register */
    __IO uint32_t cmd;                  /**< <tt>\b 0x0004:</tt> ADC9 CMD Register */
    __IO uint32_t fifo;                 /**< <tt>\b 0x0008:</tt> ADC9 FIFO Register */
    __IO uint32_t intr;                 /**< <tt>\b 0x000C:</tt> ADC9 INTR Register */
    __IO uint32_t stat;                 /**< <tt>\b 0x0010:</tt> ADC9 STAT Register */
} mxc_adc9_regs_t;

/* Register offsets for module ADC9 */
/**
 * @ingroup    adc9_registers
 * @defgroup   ADC9_Register_Offsets Register Offsets
 * @brief      ADC9 Peripheral Register Offsets from the ADC9 Base Peripheral Address.
 * @{
 */
#define MXC_R_ADC9_CFG                     ((uint32_t)0x00000000UL) /**< Offset from ADC9 Base Address: <tt> 0x0000</tt> */
#define MXC_R_ADC9_CMD                     ((uint32_t)0x00000004UL) /**< Offset from ADC9 Base Address: <tt> 0x0004</tt> */
#define MXC_R_ADC9_FIFO                    ((uint32_t)0x00000008UL) /**< Offset from ADC9 Base Address: <tt> 0x0008</tt> */
#define MXC_R_ADC9_INTR                    ((uint32_t)0x0000000CUL) /**< Offset from ADC9 Base Address: <tt> 0x000C</tt> */
#define MXC_R_ADC9_STAT                    ((uint32_t)0x00000010UL) /**< Offset from ADC9 Base Address: <tt> 0x0010</tt> */
/**@} end of group adc9_registers */

/**
 * @ingroup  adc9_registers
 * @defgroup ADC9_CFG ADC9_CFG
 * @brief    ADC Control
 * @{
 */
#define MXC_F_ADC9_CFG_CLKDIV_POS                      0 /**< CFG_CLKDIV Position */
#define MXC_F_ADC9_CFG_CLKDIV                          ((uint32_t)(0xFFUL << MXC_F_ADC9_CFG_CLKDIV_POS)) /**< CFG_CLKDIV Mask */

#define MXC_F_ADC9_CFG_ACHSEL_POS                      8 /**< CFG_ACHSEL Position */
#define MXC_F_ADC9_CFG_ACHSEL                          ((uint32_t)(0x7UL << MXC_F_ADC9_CFG_ACHSEL_POS)) /**< CFG_ACHSEL Mask */
#define MXC_V_ADC9_CFG_ACHSEL_INVALID_000              ((uint32_t)0x0UL) /**< CFG_ACHSEL_INVALID_000 Value */
#define MXC_S_ADC9_CFG_ACHSEL_INVALID_000              (MXC_V_ADC9_CFG_ACHSEL_INVALID_000 << MXC_F_ADC9_CFG_ACHSEL_POS) /**< CFG_ACHSEL_INVALID_000 Setting */
#define MXC_V_ADC9_CFG_ACHSEL_IN0                      ((uint32_t)0x1UL) /**< CFG_ACHSEL_IN0 Value */
#define MXC_S_ADC9_CFG_ACHSEL_IN0                      (MXC_V_ADC9_CFG_ACHSEL_IN0 << MXC_F_ADC9_CFG_ACHSEL_POS) /**< CFG_ACHSEL_IN0 Setting */
#define MXC_V_ADC9_CFG_ACHSEL_IN1                      ((uint32_t)0x2UL) /**< CFG_ACHSEL_IN1 Value */
#define MXC_S_ADC9_CFG_ACHSEL_IN1                      (MXC_V_ADC9_CFG_ACHSEL_IN1 << MXC_F_ADC9_CFG_ACHSEL_POS) /**< CFG_ACHSEL_IN1 Setting */
#define MXC_V_ADC9_CFG_ACHSEL_IN2                      ((uint32_t)0x3UL) /**< CFG_ACHSEL_IN2 Value */
#define MXC_S_ADC9_CFG_ACHSEL_IN2                      (MXC_V_ADC9_CFG_ACHSEL_IN2 << MXC_F_ADC9_CFG_ACHSEL_POS) /**< CFG_ACHSEL_IN2 Setting */
#define MXC_V_ADC9_CFG_ACHSEL_IN3                      ((uint32_t)0x4UL) /**< CFG_ACHSEL_IN3 Value */
#define MXC_S_ADC9_CFG_ACHSEL_IN3                      (MXC_V_ADC9_CFG_ACHSEL_IN3 << MXC_F_ADC9_CFG_ACHSEL_POS) /**< CFG_ACHSEL_IN3 Setting */
#define MXC_V_ADC9_CFG_ACHSEL_IN4                      ((uint32_t)0x5UL) /**< CFG_ACHSEL_IN4 Value */
#define MXC_S_ADC9_CFG_ACHSEL_IN4                      (MXC_V_ADC9_CFG_ACHSEL_IN4 << MXC_F_ADC9_CFG_ACHSEL_POS) /**< CFG_ACHSEL_IN4 Setting */
#define MXC_V_ADC9_CFG_ACHSEL_IN5                      ((uint32_t)0x6UL) /**< CFG_ACHSEL_IN5 Value */
#define MXC_S_ADC9_CFG_ACHSEL_IN5                      (MXC_V_ADC9_CFG_ACHSEL_IN5 << MXC_F_ADC9_CFG_ACHSEL_POS) /**< CFG_ACHSEL_IN5 Setting */
#define MXC_V_ADC9_CFG_ACHSEL_INVALID_111              ((uint32_t)0x7UL) /**< CFG_ACHSEL_INVALID_111 Value */
#define MXC_S_ADC9_CFG_ACHSEL_INVALID_111              (MXC_V_ADC9_CFG_ACHSEL_INVALID_111 << MXC_F_ADC9_CFG_ACHSEL_POS) /**< CFG_ACHSEL_INVALID_111 Setting */

#define MXC_F_ADC9_CFG_BCHSEL_POS                      11 /**< CFG_BCHSEL Position */
#define MXC_F_ADC9_CFG_BCHSEL                          ((uint32_t)(0x7UL << MXC_F_ADC9_CFG_BCHSEL_POS)) /**< CFG_BCHSEL Mask */

#define MXC_F_ADC9_CFG_CCHSEL_POS                      14 /**< CFG_CCHSEL Position */
#define MXC_F_ADC9_CFG_CCHSEL                          ((uint32_t)(0x7UL << MXC_F_ADC9_CFG_CCHSEL_POS)) /**< CFG_CCHSEL Mask */

#define MXC_F_ADC9_CFG_DCHSEL_POS                      17 /**< CFG_DCHSEL Position */
#define MXC_F_ADC9_CFG_DCHSEL                          ((uint32_t)(0x7UL << MXC_F_ADC9_CFG_DCHSEL_POS)) /**< CFG_DCHSEL Mask */

#define MXC_F_ADC9_CFG_ECHSEL_POS                      20 /**< CFG_ECHSEL Position */
#define MXC_F_ADC9_CFG_ECHSEL                          ((uint32_t)(0x7UL << MXC_F_ADC9_CFG_ECHSEL_POS)) /**< CFG_ECHSEL Mask */

#define MXC_F_ADC9_CFG_FCHSEL_POS                      23 /**< CFG_FCHSEL Position */
#define MXC_F_ADC9_CFG_FCHSEL                          ((uint32_t)(0x7UL << MXC_F_ADC9_CFG_FCHSEL_POS)) /**< CFG_FCHSEL Mask */

#define MXC_F_ADC9_CFG_GCHSEL_POS                      26 /**< CFG_GCHSEL Position */
#define MXC_F_ADC9_CFG_GCHSEL                          ((uint32_t)(0x7UL << MXC_F_ADC9_CFG_GCHSEL_POS)) /**< CFG_GCHSEL Mask */

#define MXC_F_ADC9_CFG_HCHSEL_POS                      29 /**< CFG_HCHSEL Position */
#define MXC_F_ADC9_CFG_HCHSEL                          ((uint32_t)(0x7UL << MXC_F_ADC9_CFG_HCHSEL_POS)) /**< CFG_HCHSEL Mask */

/**@} end of group ADC9_CFG_Register */

/**
 * @ingroup  adc9_registers
 * @defgroup ADC9_CMD ADC9_CMD
 * @brief    MSRADC Command
 * @{
 */
#define MXC_F_ADC9_CMD_RST_POS                         0 /**< CMD_RST Position */
#define MXC_F_ADC9_CMD_RST                             ((uint32_t)(0x1UL << MXC_F_ADC9_CMD_RST_POS)) /**< CMD_RST Mask */

#define MXC_F_ADC9_CMD_SNGLSMPL_POS                    1 /**< CMD_SNGLSMPL Position */
#define MXC_F_ADC9_CMD_SNGLSMPL                        ((uint32_t)(0x1UL << MXC_F_ADC9_CMD_SNGLSMPL_POS)) /**< CMD_SNGLSMPL Mask */

#define MXC_F_ADC9_CMD_CONTSMPL_POS                    2 /**< CMD_CONTSMPL Position */
#define MXC_F_ADC9_CMD_CONTSMPL                        ((uint32_t)(0x1UL << MXC_F_ADC9_CMD_CONTSMPL_POS)) /**< CMD_CONTSMPL Mask */

#define MXC_F_ADC9_CMD_ROTLIMIT_POS                    4 /**< CMD_ROTLIMIT Position */
#define MXC_F_ADC9_CMD_ROTLIMIT                        ((uint32_t)(0x7UL << MXC_F_ADC9_CMD_ROTLIMIT_POS)) /**< CMD_ROTLIMIT Mask */
#define MXC_V_ADC9_CMD_ROTLIMIT_1_CHANNEL              ((uint32_t)0x0UL) /**< CMD_ROTLIMIT_1_CHANNEL Value */
#define MXC_S_ADC9_CMD_ROTLIMIT_1_CHANNEL              (MXC_V_ADC9_CMD_ROTLIMIT_1_CHANNEL << MXC_F_ADC9_CMD_ROTLIMIT_POS) /**< CMD_ROTLIMIT_1_CHANNEL Setting */
#define MXC_V_ADC9_CMD_ROTLIMIT_2_CHANNELS             ((uint32_t)0x1UL) /**< CMD_ROTLIMIT_2_CHANNELS Value */
#define MXC_S_ADC9_CMD_ROTLIMIT_2_CHANNELS             (MXC_V_ADC9_CMD_ROTLIMIT_2_CHANNELS << MXC_F_ADC9_CMD_ROTLIMIT_POS) /**< CMD_ROTLIMIT_2_CHANNELS Setting */
#define MXC_V_ADC9_CMD_ROTLIMIT_3_CHANNELS             ((uint32_t)0x2UL) /**< CMD_ROTLIMIT_3_CHANNELS Value */
#define MXC_S_ADC9_CMD_ROTLIMIT_3_CHANNELS             (MXC_V_ADC9_CMD_ROTLIMIT_3_CHANNELS << MXC_F_ADC9_CMD_ROTLIMIT_POS) /**< CMD_ROTLIMIT_3_CHANNELS Setting */
#define MXC_V_ADC9_CMD_ROTLIMIT_4_CHANNELS             ((uint32_t)0x3UL) /**< CMD_ROTLIMIT_4_CHANNELS Value */
#define MXC_S_ADC9_CMD_ROTLIMIT_4_CHANNELS             (MXC_V_ADC9_CMD_ROTLIMIT_4_CHANNELS << MXC_F_ADC9_CMD_ROTLIMIT_POS) /**< CMD_ROTLIMIT_4_CHANNELS Setting */
#define MXC_V_ADC9_CMD_ROTLIMIT_5_CHANNELS             ((uint32_t)0x4UL) /**< CMD_ROTLIMIT_5_CHANNELS Value */
#define MXC_S_ADC9_CMD_ROTLIMIT_5_CHANNELS             (MXC_V_ADC9_CMD_ROTLIMIT_5_CHANNELS << MXC_F_ADC9_CMD_ROTLIMIT_POS) /**< CMD_ROTLIMIT_5_CHANNELS Setting */
#define MXC_V_ADC9_CMD_ROTLIMIT_6_CHANNELS             ((uint32_t)0x5UL) /**< CMD_ROTLIMIT_6_CHANNELS Value */
#define MXC_S_ADC9_CMD_ROTLIMIT_6_CHANNELS             (MXC_V_ADC9_CMD_ROTLIMIT_6_CHANNELS << MXC_F_ADC9_CMD_ROTLIMIT_POS) /**< CMD_ROTLIMIT_6_CHANNELS Setting */
#define MXC_V_ADC9_CMD_ROTLIMIT_7_CHANNELS             ((uint32_t)0x6UL) /**< CMD_ROTLIMIT_7_CHANNELS Value */
#define MXC_S_ADC9_CMD_ROTLIMIT_7_CHANNELS             (MXC_V_ADC9_CMD_ROTLIMIT_7_CHANNELS << MXC_F_ADC9_CMD_ROTLIMIT_POS) /**< CMD_ROTLIMIT_7_CHANNELS Setting */
#define MXC_V_ADC9_CMD_ROTLIMIT_8_CHANNELS             ((uint32_t)0x7UL) /**< CMD_ROTLIMIT_8_CHANNELS Value */
#define MXC_S_ADC9_CMD_ROTLIMIT_8_CHANNELS             (MXC_V_ADC9_CMD_ROTLIMIT_8_CHANNELS << MXC_F_ADC9_CMD_ROTLIMIT_POS) /**< CMD_ROTLIMIT_8_CHANNELS Setting */

#define MXC_F_ADC9_CMD_CLKSEL_POS                      8 /**< CMD_CLKSEL Position */
#define MXC_F_ADC9_CMD_CLKSEL                          ((uint32_t)(0x7UL << MXC_F_ADC9_CMD_CLKSEL_POS)) /**< CMD_CLKSEL Mask */
#define MXC_V_ADC9_CMD_CLKSEL_3_SAMPLES                ((uint32_t)0x0UL) /**< CMD_CLKSEL_3_SAMPLES Value */
#define MXC_S_ADC9_CMD_CLKSEL_3_SAMPLES                (MXC_V_ADC9_CMD_CLKSEL_3_SAMPLES << MXC_F_ADC9_CMD_CLKSEL_POS) /**< CMD_CLKSEL_3_SAMPLES Setting */
#define MXC_V_ADC9_CMD_CLKSEL_5_SAMPLES                ((uint32_t)0x1UL) /**< CMD_CLKSEL_5_SAMPLES Value */
#define MXC_S_ADC9_CMD_CLKSEL_5_SAMPLES                (MXC_V_ADC9_CMD_CLKSEL_5_SAMPLES << MXC_F_ADC9_CMD_CLKSEL_POS) /**< CMD_CLKSEL_5_SAMPLES Setting */
#define MXC_V_ADC9_CMD_CLKSEL_4_SAMPLES                ((uint32_t)0x2UL) /**< CMD_CLKSEL_4_SAMPLES Value */
#define MXC_S_ADC9_CMD_CLKSEL_4_SAMPLES                (MXC_V_ADC9_CMD_CLKSEL_4_SAMPLES << MXC_F_ADC9_CMD_CLKSEL_POS) /**< CMD_CLKSEL_4_SAMPLES Setting */
#define MXC_V_ADC9_CMD_CLKSEL_8_SAMPLES                ((uint32_t)0x3UL) /**< CMD_CLKSEL_8_SAMPLES Value */
#define MXC_S_ADC9_CMD_CLKSEL_8_SAMPLES                (MXC_V_ADC9_CMD_CLKSEL_8_SAMPLES << MXC_F_ADC9_CMD_CLKSEL_POS) /**< CMD_CLKSEL_8_SAMPLES Setting */
#define MXC_V_ADC9_CMD_CLKSEL_16_SAMPLES               ((uint32_t)0x4UL) /**< CMD_CLKSEL_16_SAMPLES Value */
#define MXC_S_ADC9_CMD_CLKSEL_16_SAMPLES               (MXC_V_ADC9_CMD_CLKSEL_16_SAMPLES << MXC_F_ADC9_CMD_CLKSEL_POS) /**< CMD_CLKSEL_16_SAMPLES Setting */
#define MXC_V_ADC9_CMD_CLKSEL_32_SAMPLES               ((uint32_t)0x5UL) /**< CMD_CLKSEL_32_SAMPLES Value */
#define MXC_S_ADC9_CMD_CLKSEL_32_SAMPLES               (MXC_V_ADC9_CMD_CLKSEL_32_SAMPLES << MXC_F_ADC9_CMD_CLKSEL_POS) /**< CMD_CLKSEL_32_SAMPLES Setting */
#define MXC_V_ADC9_CMD_CLKSEL_64_SAMPLES               ((uint32_t)0x6UL) /**< CMD_CLKSEL_64_SAMPLES Value */
#define MXC_S_ADC9_CMD_CLKSEL_64_SAMPLES               (MXC_V_ADC9_CMD_CLKSEL_64_SAMPLES << MXC_F_ADC9_CMD_CLKSEL_POS) /**< CMD_CLKSEL_64_SAMPLES Setting */
#define MXC_V_ADC9_CMD_CLKSEL_128_SAMPLES              ((uint32_t)0x7UL) /**< CMD_CLKSEL_128_SAMPLES Value */
#define MXC_S_ADC9_CMD_CLKSEL_128_SAMPLES              (MXC_V_ADC9_CMD_CLKSEL_128_SAMPLES << MXC_F_ADC9_CMD_CLKSEL_POS) /**< CMD_CLKSEL_128_SAMPLES Setting */

/**@} end of group ADC9_CMD_Register */

/**
 * @ingroup  adc9_registers
 * @defgroup ADC9_FIFO ADC9_FIFO
 * @brief    ADC FIFO
 * @{
 */
#define MXC_F_ADC9_FIFO_SAMPLE_POS                     0 /**< FIFO_SAMPLE Position */
#define MXC_F_ADC9_FIFO_SAMPLE                         ((uint32_t)(0x1FFUL << MXC_F_ADC9_FIFO_SAMPLE_POS)) /**< FIFO_SAMPLE Mask */

#define MXC_F_ADC9_FIFO_SMPLIN_POS                     9 /**< FIFO_SMPLIN Position */
#define MXC_F_ADC9_FIFO_SMPLIN                         ((uint32_t)(0x7UL << MXC_F_ADC9_FIFO_SMPLIN_POS)) /**< FIFO_SMPLIN Mask */
#define MXC_V_ADC9_FIFO_SMPLIN_INVALID_000             ((uint32_t)0x0UL) /**< FIFO_SMPLIN_INVALID_000 Value */
#define MXC_S_ADC9_FIFO_SMPLIN_INVALID_000             (MXC_V_ADC9_FIFO_SMPLIN_INVALID_000 << MXC_F_ADC9_FIFO_SMPLIN_POS) /**< FIFO_SMPLIN_INVALID_000 Setting */
#define MXC_V_ADC9_FIFO_SMPLIN_IN0                     ((uint32_t)0x1UL) /**< FIFO_SMPLIN_IN0 Value */
#define MXC_S_ADC9_FIFO_SMPLIN_IN0                     (MXC_V_ADC9_FIFO_SMPLIN_IN0 << MXC_F_ADC9_FIFO_SMPLIN_POS) /**< FIFO_SMPLIN_IN0 Setting */
#define MXC_V_ADC9_FIFO_SMPLIN_IN1                     ((uint32_t)0x2UL) /**< FIFO_SMPLIN_IN1 Value */
#define MXC_S_ADC9_FIFO_SMPLIN_IN1                     (MXC_V_ADC9_FIFO_SMPLIN_IN1 << MXC_F_ADC9_FIFO_SMPLIN_POS) /**< FIFO_SMPLIN_IN1 Setting */
#define MXC_V_ADC9_FIFO_SMPLIN_IN2                     ((uint32_t)0x3UL) /**< FIFO_SMPLIN_IN2 Value */
#define MXC_S_ADC9_FIFO_SMPLIN_IN2                     (MXC_V_ADC9_FIFO_SMPLIN_IN2 << MXC_F_ADC9_FIFO_SMPLIN_POS) /**< FIFO_SMPLIN_IN2 Setting */
#define MXC_V_ADC9_FIFO_SMPLIN_IN3                     ((uint32_t)0x4UL) /**< FIFO_SMPLIN_IN3 Value */
#define MXC_S_ADC9_FIFO_SMPLIN_IN3                     (MXC_V_ADC9_FIFO_SMPLIN_IN3 << MXC_F_ADC9_FIFO_SMPLIN_POS) /**< FIFO_SMPLIN_IN3 Setting */
#define MXC_V_ADC9_FIFO_SMPLIN_IN4                     ((uint32_t)0x5UL) /**< FIFO_SMPLIN_IN4 Value */
#define MXC_S_ADC9_FIFO_SMPLIN_IN4                     (MXC_V_ADC9_FIFO_SMPLIN_IN4 << MXC_F_ADC9_FIFO_SMPLIN_POS) /**< FIFO_SMPLIN_IN4 Setting */
#define MXC_V_ADC9_FIFO_SMPLIN_IN5                     ((uint32_t)0x6UL) /**< FIFO_SMPLIN_IN5 Value */
#define MXC_S_ADC9_FIFO_SMPLIN_IN5                     (MXC_V_ADC9_FIFO_SMPLIN_IN5 << MXC_F_ADC9_FIFO_SMPLIN_POS) /**< FIFO_SMPLIN_IN5 Setting */
#define MXC_V_ADC9_FIFO_SMPLIN_INVALID_111             ((uint32_t)0x7UL) /**< FIFO_SMPLIN_INVALID_111 Value */
#define MXC_S_ADC9_FIFO_SMPLIN_INVALID_111             (MXC_V_ADC9_FIFO_SMPLIN_INVALID_111 << MXC_F_ADC9_FIFO_SMPLIN_POS) /**< FIFO_SMPLIN_INVALID_111 Setting */

/**@} end of group ADC9_FIFO_Register */

/**
 * @ingroup  adc9_registers
 * @defgroup ADC9_INTR ADC9_INTR
 * @brief    ADC Interrupt Enable Register
 * @{
 */
#define MXC_F_ADC9_INTR_FIFOLVL_POS                    0 /**< INTR_FIFOLVL Position */
#define MXC_F_ADC9_INTR_FIFOLVL                        ((uint32_t)(0x7UL << MXC_F_ADC9_INTR_FIFOLVL_POS)) /**< INTR_FIFOLVL Mask */
#define MXC_V_ADC9_INTR_FIFOLVL_AT_LEAST_1             ((uint32_t)0x0UL) /**< INTR_FIFOLVL_AT_LEAST_1 Value */
#define MXC_S_ADC9_INTR_FIFOLVL_AT_LEAST_1             (MXC_V_ADC9_INTR_FIFOLVL_AT_LEAST_1 << MXC_F_ADC9_INTR_FIFOLVL_POS) /**< INTR_FIFOLVL_AT_LEAST_1 Setting */
#define MXC_V_ADC9_INTR_FIFOLVL_AT_LEAST_2             ((uint32_t)0x1UL) /**< INTR_FIFOLVL_AT_LEAST_2 Value */
#define MXC_S_ADC9_INTR_FIFOLVL_AT_LEAST_2             (MXC_V_ADC9_INTR_FIFOLVL_AT_LEAST_2 << MXC_F_ADC9_INTR_FIFOLVL_POS) /**< INTR_FIFOLVL_AT_LEAST_2 Setting */
#define MXC_V_ADC9_INTR_FIFOLVL_AT_LEAST_3             ((uint32_t)0x2UL) /**< INTR_FIFOLVL_AT_LEAST_3 Value */
#define MXC_S_ADC9_INTR_FIFOLVL_AT_LEAST_3             (MXC_V_ADC9_INTR_FIFOLVL_AT_LEAST_3 << MXC_F_ADC9_INTR_FIFOLVL_POS) /**< INTR_FIFOLVL_AT_LEAST_3 Setting */
#define MXC_V_ADC9_INTR_FIFOLVL_AT_LEAST_4             ((uint32_t)0x3UL) /**< INTR_FIFOLVL_AT_LEAST_4 Value */
#define MXC_S_ADC9_INTR_FIFOLVL_AT_LEAST_4             (MXC_V_ADC9_INTR_FIFOLVL_AT_LEAST_4 << MXC_F_ADC9_INTR_FIFOLVL_POS) /**< INTR_FIFOLVL_AT_LEAST_4 Setting */
#define MXC_V_ADC9_INTR_FIFOLVL_AT_LEAST_5             ((uint32_t)0x4UL) /**< INTR_FIFOLVL_AT_LEAST_5 Value */
#define MXC_S_ADC9_INTR_FIFOLVL_AT_LEAST_5             (MXC_V_ADC9_INTR_FIFOLVL_AT_LEAST_5 << MXC_F_ADC9_INTR_FIFOLVL_POS) /**< INTR_FIFOLVL_AT_LEAST_5 Setting */
#define MXC_V_ADC9_INTR_FIFOLVL_AT_LEAST_6             ((uint32_t)0x5UL) /**< INTR_FIFOLVL_AT_LEAST_6 Value */
#define MXC_S_ADC9_INTR_FIFOLVL_AT_LEAST_6             (MXC_V_ADC9_INTR_FIFOLVL_AT_LEAST_6 << MXC_F_ADC9_INTR_FIFOLVL_POS) /**< INTR_FIFOLVL_AT_LEAST_6 Setting */
#define MXC_V_ADC9_INTR_FIFOLVL_AT_LEAST_7             ((uint32_t)0x6UL) /**< INTR_FIFOLVL_AT_LEAST_7 Value */
#define MXC_S_ADC9_INTR_FIFOLVL_AT_LEAST_7             (MXC_V_ADC9_INTR_FIFOLVL_AT_LEAST_7 << MXC_F_ADC9_INTR_FIFOLVL_POS) /**< INTR_FIFOLVL_AT_LEAST_7 Setting */
#define MXC_V_ADC9_INTR_FIFOLVL_AT_LEAST_8             ((uint32_t)0x7UL) /**< INTR_FIFOLVL_AT_LEAST_8 Value */
#define MXC_S_ADC9_INTR_FIFOLVL_AT_LEAST_8             (MXC_V_ADC9_INTR_FIFOLVL_AT_LEAST_8 << MXC_F_ADC9_INTR_FIFOLVL_POS) /**< INTR_FIFOLVL_AT_LEAST_8 Setting */

#define MXC_F_ADC9_INTR_DMAREQEN_POS                   3 /**< INTR_DMAREQEN Position */
#define MXC_F_ADC9_INTR_DMAREQEN                       ((uint32_t)(0x1UL << MXC_F_ADC9_INTR_DMAREQEN_POS)) /**< INTR_DMAREQEN Mask */

#define MXC_F_ADC9_INTR_OVERFIE_POS                    6 /**< INTR_OVERFIE Position */
#define MXC_F_ADC9_INTR_OVERFIE                        ((uint32_t)(0x1UL << MXC_F_ADC9_INTR_OVERFIE_POS)) /**< INTR_OVERFIE Mask */

#define MXC_F_ADC9_INTR_UNDRFIE_POS                    7 /**< INTR_UNDRFIE Position */
#define MXC_F_ADC9_INTR_UNDRFIE                        ((uint32_t)(0x1UL << MXC_F_ADC9_INTR_UNDRFIE_POS)) /**< INTR_UNDRFIE Mask */

#define MXC_F_ADC9_INTR_FIFOLVLIE_POS                  8 /**< INTR_FIFOLVLIE Position */
#define MXC_F_ADC9_INTR_FIFOLVLIE                      ((uint32_t)(0x1UL << MXC_F_ADC9_INTR_FIFOLVLIE_POS)) /**< INTR_FIFOLVLIE Mask */

#define MXC_F_ADC9_INTR_GLOBIE_POS                     9 /**< INTR_GLOBIE Position */
#define MXC_F_ADC9_INTR_GLOBIE                         ((uint32_t)(0x1UL << MXC_F_ADC9_INTR_GLOBIE_POS)) /**< INTR_GLOBIE Mask */

/**@} end of group ADC9_INTR_Register */

/**
 * @ingroup  adc9_registers
 * @defgroup ADC9_STAT ADC9_STAT
 * @brief    ADC Interrupt Flag Register.
 * @{
 */
#define MXC_F_ADC9_STAT_FIFOCNT_POS                    0 /**< STAT_FIFOCNT Position */
#define MXC_F_ADC9_STAT_FIFOCNT                        ((uint32_t)(0xFUL << MXC_F_ADC9_STAT_FIFOCNT_POS)) /**< STAT_FIFOCNT Mask */
#define MXC_V_ADC9_STAT_FIFOCNT_FIFO_EMPTY             ((uint32_t)0x0UL) /**< STAT_FIFOCNT_FIFO_EMPTY Value */
#define MXC_S_ADC9_STAT_FIFOCNT_FIFO_EMPTY             (MXC_V_ADC9_STAT_FIFOCNT_FIFO_EMPTY << MXC_F_ADC9_STAT_FIFOCNT_POS) /**< STAT_FIFOCNT_FIFO_EMPTY Setting */
#define MXC_V_ADC9_STAT_FIFOCNT_ONE_SAMPLE             ((uint32_t)0x1UL) /**< STAT_FIFOCNT_ONE_SAMPLE Value */
#define MXC_S_ADC9_STAT_FIFOCNT_ONE_SAMPLE             (MXC_V_ADC9_STAT_FIFOCNT_ONE_SAMPLE << MXC_F_ADC9_STAT_FIFOCNT_POS) /**< STAT_FIFOCNT_ONE_SAMPLE Setting */
#define MXC_V_ADC9_STAT_FIFOCNT_TWO_SAMPLE             ((uint32_t)0x2UL) /**< STAT_FIFOCNT_TWO_SAMPLE Value */
#define MXC_S_ADC9_STAT_FIFOCNT_TWO_SAMPLE             (MXC_V_ADC9_STAT_FIFOCNT_TWO_SAMPLE << MXC_F_ADC9_STAT_FIFOCNT_POS) /**< STAT_FIFOCNT_TWO_SAMPLE Setting */
#define MXC_V_ADC9_STAT_FIFOCNT_THREE_SAMPLE           ((uint32_t)0x3UL) /**< STAT_FIFOCNT_THREE_SAMPLE Value */
#define MXC_S_ADC9_STAT_FIFOCNT_THREE_SAMPLE           (MXC_V_ADC9_STAT_FIFOCNT_THREE_SAMPLE << MXC_F_ADC9_STAT_FIFOCNT_POS) /**< STAT_FIFOCNT_THREE_SAMPLE Setting */
#define MXC_V_ADC9_STAT_FIFOCNT_FOUR_SAMPLE            ((uint32_t)0x4UL) /**< STAT_FIFOCNT_FOUR_SAMPLE Value */
#define MXC_S_ADC9_STAT_FIFOCNT_FOUR_SAMPLE            (MXC_V_ADC9_STAT_FIFOCNT_FOUR_SAMPLE << MXC_F_ADC9_STAT_FIFOCNT_POS) /**< STAT_FIFOCNT_FOUR_SAMPLE Setting */
#define MXC_V_ADC9_STAT_FIFOCNT_FIVE_SAMPLE            ((uint32_t)0x5UL) /**< STAT_FIFOCNT_FIVE_SAMPLE Value */
#define MXC_S_ADC9_STAT_FIFOCNT_FIVE_SAMPLE            (MXC_V_ADC9_STAT_FIFOCNT_FIVE_SAMPLE << MXC_F_ADC9_STAT_FIFOCNT_POS) /**< STAT_FIFOCNT_FIVE_SAMPLE Setting */
#define MXC_V_ADC9_STAT_FIFOCNT_SIX_SAMPLE             ((uint32_t)0x6UL) /**< STAT_FIFOCNT_SIX_SAMPLE Value */
#define MXC_S_ADC9_STAT_FIFOCNT_SIX_SAMPLE             (MXC_V_ADC9_STAT_FIFOCNT_SIX_SAMPLE << MXC_F_ADC9_STAT_FIFOCNT_POS) /**< STAT_FIFOCNT_SIX_SAMPLE Setting */
#define MXC_V_ADC9_STAT_FIFOCNT_SEVEN_SAMPLE           ((uint32_t)0x7UL) /**< STAT_FIFOCNT_SEVEN_SAMPLE Value */
#define MXC_S_ADC9_STAT_FIFOCNT_SEVEN_SAMPLE           (MXC_V_ADC9_STAT_FIFOCNT_SEVEN_SAMPLE << MXC_F_ADC9_STAT_FIFOCNT_POS) /**< STAT_FIFOCNT_SEVEN_SAMPLE Setting */
#define MXC_V_ADC9_STAT_FIFOCNT_EIGHT_SAMPLE           ((uint32_t)0x8UL) /**< STAT_FIFOCNT_EIGHT_SAMPLE Value */
#define MXC_S_ADC9_STAT_FIFOCNT_EIGHT_SAMPLE           (MXC_V_ADC9_STAT_FIFOCNT_EIGHT_SAMPLE << MXC_F_ADC9_STAT_FIFOCNT_POS) /**< STAT_FIFOCNT_EIGHT_SAMPLE Setting */

#define MXC_F_ADC9_STAT_FULL_POS                       4 /**< STAT_FULL Position */
#define MXC_F_ADC9_STAT_FULL                           ((uint32_t)(0x1UL << MXC_F_ADC9_STAT_FULL_POS)) /**< STAT_FULL Mask */

#define MXC_F_ADC9_STAT_EMPTY_POS                      5 /**< STAT_EMPTY Position */
#define MXC_F_ADC9_STAT_EMPTY                          ((uint32_t)(0x1UL << MXC_F_ADC9_STAT_EMPTY_POS)) /**< STAT_EMPTY Mask */

#define MXC_F_ADC9_STAT_OVERFINT_POS                   6 /**< STAT_OVERFINT Position */
#define MXC_F_ADC9_STAT_OVERFINT                       ((uint32_t)(0x1UL << MXC_F_ADC9_STAT_OVERFINT_POS)) /**< STAT_OVERFINT Mask */

#define MXC_F_ADC9_STAT_UNDRFINT_POS                   7 /**< STAT_UNDRFINT Position */
#define MXC_F_ADC9_STAT_UNDRFINT                       ((uint32_t)(0x1UL << MXC_F_ADC9_STAT_UNDRFINT_POS)) /**< STAT_UNDRFINT Mask */

#define MXC_F_ADC9_STAT_FIFOLVLST_POS                  8 /**< STAT_FIFOLVLST Position */
#define MXC_F_ADC9_STAT_FIFOLVLST                      ((uint32_t)(0x1UL << MXC_F_ADC9_STAT_FIFOLVLST_POS)) /**< STAT_FIFOLVLST Mask */

#define MXC_F_ADC9_STAT_GLOBINT_POS                    9 /**< STAT_GLOBINT Position */
#define MXC_F_ADC9_STAT_GLOBINT                        ((uint32_t)(0x1UL << MXC_F_ADC9_STAT_GLOBINT_POS)) /**< STAT_GLOBINT Mask */

/**@} end of group ADC9_STAT_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_ADC9_REGS_H_
