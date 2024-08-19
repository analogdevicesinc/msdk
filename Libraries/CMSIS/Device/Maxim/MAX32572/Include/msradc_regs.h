/**
 * @file    msradc_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the MSRADC Peripheral Module.
 * @note    This file is @generated.
 * @ingroup msradc_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_MSRADC_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_MSRADC_REGS_H_

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
 * @ingroup     msradc
 * @defgroup    msradc_registers MSRADC_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the MSRADC Peripheral Module.
 * @details     Magnetic Strip Reader - 9 bit ADC
 */

/**
 * @ingroup msradc_registers
 * Structure type to access the MSRADC Registers.
 */
typedef struct {
    __IO uint32_t ctrl;                 /**< <tt>\b 0x0000:</tt> MSRADC CTRL Register */
    __IO uint32_t cmd;                  /**< <tt>\b 0x0004:</tt> MSRADC CMD Register */
    __IO uint32_t fifo;                 /**< <tt>\b 0x0008:</tt> MSRADC FIFO Register */
    __IO uint32_t inten;                /**< <tt>\b 0x000C:</tt> MSRADC INTEN Register */
    __IO uint32_t intfl;                /**< <tt>\b 0x0010:</tt> MSRADC INTFL Register */
} mxc_msradc_regs_t;

/* Register offsets for module MSRADC */
/**
 * @ingroup    msradc_registers
 * @defgroup   MSRADC_Register_Offsets Register Offsets
 * @brief      MSRADC Peripheral Register Offsets from the MSRADC Base Peripheral Address.
 * @{
 */
#define MXC_R_MSRADC_CTRL                  ((uint32_t)0x00000000UL) /**< Offset from MSRADC Base Address: <tt> 0x0000</tt> */
#define MXC_R_MSRADC_CMD                   ((uint32_t)0x00000004UL) /**< Offset from MSRADC Base Address: <tt> 0x0004</tt> */
#define MXC_R_MSRADC_FIFO                  ((uint32_t)0x00000008UL) /**< Offset from MSRADC Base Address: <tt> 0x0008</tt> */
#define MXC_R_MSRADC_INTEN                 ((uint32_t)0x0000000CUL) /**< Offset from MSRADC Base Address: <tt> 0x000C</tt> */
#define MXC_R_MSRADC_INTFL                 ((uint32_t)0x00000010UL) /**< Offset from MSRADC Base Address: <tt> 0x0010</tt> */
/**@} end of group msradc_registers */

/**
 * @ingroup  msradc_registers
 * @defgroup MSRADC_CTRL MSRADC_CTRL
 * @brief    ADC Control
 * @{
 */
#define MXC_F_MSRADC_CTRL_CLKDIV_POS                   0 /**< CTRL_CLKDIV Position */
#define MXC_F_MSRADC_CTRL_CLKDIV                       ((uint32_t)(0xFFUL << MXC_F_MSRADC_CTRL_CLKDIV_POS)) /**< CTRL_CLKDIV Mask */

#define MXC_F_MSRADC_CTRL_ACHSEL_POS                   8 /**< CTRL_ACHSEL Position */
#define MXC_F_MSRADC_CTRL_ACHSEL                       ((uint32_t)(0x7UL << MXC_F_MSRADC_CTRL_ACHSEL_POS)) /**< CTRL_ACHSEL Mask */
#define MXC_V_MSRADC_CTRL_ACHSEL_INVALID               ((uint32_t)0x0UL) /**< CTRL_ACHSEL_INVALID Value */
#define MXC_S_MSRADC_CTRL_ACHSEL_INVALID               (MXC_V_MSRADC_CTRL_ACHSEL_INVALID << MXC_F_MSRADC_CTRL_ACHSEL_POS) /**< CTRL_ACHSEL_INVALID Setting */
#define MXC_V_MSRADC_CTRL_ACHSEL_CH1_IN                ((uint32_t)0x1UL) /**< CTRL_ACHSEL_CH1_IN Value */
#define MXC_S_MSRADC_CTRL_ACHSEL_CH1_IN                (MXC_V_MSRADC_CTRL_ACHSEL_CH1_IN << MXC_F_MSRADC_CTRL_ACHSEL_POS) /**< CTRL_ACHSEL_CH1_IN Setting */
#define MXC_V_MSRADC_CTRL_ACHSEL_CH2_IN                ((uint32_t)0x2UL) /**< CTRL_ACHSEL_CH2_IN Value */
#define MXC_S_MSRADC_CTRL_ACHSEL_CH2_IN                (MXC_V_MSRADC_CTRL_ACHSEL_CH2_IN << MXC_F_MSRADC_CTRL_ACHSEL_POS) /**< CTRL_ACHSEL_CH2_IN Setting */
#define MXC_V_MSRADC_CTRL_ACHSEL_CH3_IN                ((uint32_t)0x3UL) /**< CTRL_ACHSEL_CH3_IN Value */
#define MXC_S_MSRADC_CTRL_ACHSEL_CH3_IN                (MXC_V_MSRADC_CTRL_ACHSEL_CH3_IN << MXC_F_MSRADC_CTRL_ACHSEL_POS) /**< CTRL_ACHSEL_CH3_IN Setting */

#define MXC_F_MSRADC_CTRL_BCHSEL_POS                   11 /**< CTRL_BCHSEL Position */
#define MXC_F_MSRADC_CTRL_BCHSEL                       ((uint32_t)(0x7UL << MXC_F_MSRADC_CTRL_BCHSEL_POS)) /**< CTRL_BCHSEL Mask */

#define MXC_F_MSRADC_CTRL_CCHSEL_POS                   14 /**< CTRL_CCHSEL Position */
#define MXC_F_MSRADC_CTRL_CCHSEL                       ((uint32_t)(0x7UL << MXC_F_MSRADC_CTRL_CCHSEL_POS)) /**< CTRL_CCHSEL Mask */

#define MXC_F_MSRADC_CTRL_DCHSEL_POS                   17 /**< CTRL_DCHSEL Position */
#define MXC_F_MSRADC_CTRL_DCHSEL                       ((uint32_t)(0x7UL << MXC_F_MSRADC_CTRL_DCHSEL_POS)) /**< CTRL_DCHSEL Mask */

#define MXC_F_MSRADC_CTRL_ECHSEL_POS                   20 /**< CTRL_ECHSEL Position */
#define MXC_F_MSRADC_CTRL_ECHSEL                       ((uint32_t)(0x7UL << MXC_F_MSRADC_CTRL_ECHSEL_POS)) /**< CTRL_ECHSEL Mask */

#define MXC_F_MSRADC_CTRL_FCHSEL_POS                   23 /**< CTRL_FCHSEL Position */
#define MXC_F_MSRADC_CTRL_FCHSEL                       ((uint32_t)(0x7UL << MXC_F_MSRADC_CTRL_FCHSEL_POS)) /**< CTRL_FCHSEL Mask */

#define MXC_F_MSRADC_CTRL_GCHSEL_POS                   26 /**< CTRL_GCHSEL Position */
#define MXC_F_MSRADC_CTRL_GCHSEL                       ((uint32_t)(0x7UL << MXC_F_MSRADC_CTRL_GCHSEL_POS)) /**< CTRL_GCHSEL Mask */

#define MXC_F_MSRADC_CTRL_HCHSEL_POS                   29 /**< CTRL_HCHSEL Position */
#define MXC_F_MSRADC_CTRL_HCHSEL                       ((uint32_t)(0x7UL << MXC_F_MSRADC_CTRL_HCHSEL_POS)) /**< CTRL_HCHSEL Mask */

/**@} end of group MSRADC_CTRL_Register */

/**
 * @ingroup  msradc_registers
 * @defgroup MSRADC_CMD MSRADC_CMD
 * @brief    MSRADC Command
 * @{
 */
#define MXC_F_MSRADC_CMD_RST_POS                       0 /**< CMD_RST Position */
#define MXC_F_MSRADC_CMD_RST                           ((uint32_t)(0x1UL << MXC_F_MSRADC_CMD_RST_POS)) /**< CMD_RST Mask */

#define MXC_F_MSRADC_CMD_SNGLSMPL_POS                  1 /**< CMD_SNGLSMPL Position */
#define MXC_F_MSRADC_CMD_SNGLSMPL                      ((uint32_t)(0x1UL << MXC_F_MSRADC_CMD_SNGLSMPL_POS)) /**< CMD_SNGLSMPL Mask */

#define MXC_F_MSRADC_CMD_CONTSMPL_POS                  2 /**< CMD_CONTSMPL Position */
#define MXC_F_MSRADC_CMD_CONTSMPL                      ((uint32_t)(0x1UL << MXC_F_MSRADC_CMD_CONTSMPL_POS)) /**< CMD_CONTSMPL Mask */

#define MXC_F_MSRADC_CMD_ROTLIMIT_POS                  4 /**< CMD_ROTLIMIT Position */
#define MXC_F_MSRADC_CMD_ROTLIMIT                      ((uint32_t)(0x7UL << MXC_F_MSRADC_CMD_ROTLIMIT_POS)) /**< CMD_ROTLIMIT Mask */
#define MXC_V_MSRADC_CMD_ROTLIMIT_1_CHANNEL            ((uint32_t)0x0UL) /**< CMD_ROTLIMIT_1_CHANNEL Value */
#define MXC_S_MSRADC_CMD_ROTLIMIT_1_CHANNEL            (MXC_V_MSRADC_CMD_ROTLIMIT_1_CHANNEL << MXC_F_MSRADC_CMD_ROTLIMIT_POS) /**< CMD_ROTLIMIT_1_CHANNEL Setting */
#define MXC_V_MSRADC_CMD_ROTLIMIT_2_CHANNELS           ((uint32_t)0x1UL) /**< CMD_ROTLIMIT_2_CHANNELS Value */
#define MXC_S_MSRADC_CMD_ROTLIMIT_2_CHANNELS           (MXC_V_MSRADC_CMD_ROTLIMIT_2_CHANNELS << MXC_F_MSRADC_CMD_ROTLIMIT_POS) /**< CMD_ROTLIMIT_2_CHANNELS Setting */
#define MXC_V_MSRADC_CMD_ROTLIMIT_3_CHANNELS           ((uint32_t)0x2UL) /**< CMD_ROTLIMIT_3_CHANNELS Value */
#define MXC_S_MSRADC_CMD_ROTLIMIT_3_CHANNELS           (MXC_V_MSRADC_CMD_ROTLIMIT_3_CHANNELS << MXC_F_MSRADC_CMD_ROTLIMIT_POS) /**< CMD_ROTLIMIT_3_CHANNELS Setting */
#define MXC_V_MSRADC_CMD_ROTLIMIT_4_CHANNELS           ((uint32_t)0x3UL) /**< CMD_ROTLIMIT_4_CHANNELS Value */
#define MXC_S_MSRADC_CMD_ROTLIMIT_4_CHANNELS           (MXC_V_MSRADC_CMD_ROTLIMIT_4_CHANNELS << MXC_F_MSRADC_CMD_ROTLIMIT_POS) /**< CMD_ROTLIMIT_4_CHANNELS Setting */
#define MXC_V_MSRADC_CMD_ROTLIMIT_5_CHANNELS           ((uint32_t)0x4UL) /**< CMD_ROTLIMIT_5_CHANNELS Value */
#define MXC_S_MSRADC_CMD_ROTLIMIT_5_CHANNELS           (MXC_V_MSRADC_CMD_ROTLIMIT_5_CHANNELS << MXC_F_MSRADC_CMD_ROTLIMIT_POS) /**< CMD_ROTLIMIT_5_CHANNELS Setting */
#define MXC_V_MSRADC_CMD_ROTLIMIT_6_CHANNELS           ((uint32_t)0x5UL) /**< CMD_ROTLIMIT_6_CHANNELS Value */
#define MXC_S_MSRADC_CMD_ROTLIMIT_6_CHANNELS           (MXC_V_MSRADC_CMD_ROTLIMIT_6_CHANNELS << MXC_F_MSRADC_CMD_ROTLIMIT_POS) /**< CMD_ROTLIMIT_6_CHANNELS Setting */
#define MXC_V_MSRADC_CMD_ROTLIMIT_7_CHANNELS           ((uint32_t)0x6UL) /**< CMD_ROTLIMIT_7_CHANNELS Value */
#define MXC_S_MSRADC_CMD_ROTLIMIT_7_CHANNELS           (MXC_V_MSRADC_CMD_ROTLIMIT_7_CHANNELS << MXC_F_MSRADC_CMD_ROTLIMIT_POS) /**< CMD_ROTLIMIT_7_CHANNELS Setting */
#define MXC_V_MSRADC_CMD_ROTLIMIT_8_CHANNELS           ((uint32_t)0x7UL) /**< CMD_ROTLIMIT_8_CHANNELS Value */
#define MXC_S_MSRADC_CMD_ROTLIMIT_8_CHANNELS           (MXC_V_MSRADC_CMD_ROTLIMIT_8_CHANNELS << MXC_F_MSRADC_CMD_ROTLIMIT_POS) /**< CMD_ROTLIMIT_8_CHANNELS Setting */

#define MXC_F_MSRADC_CMD_CLKSEL_POS                    8 /**< CMD_CLKSEL Position */
#define MXC_F_MSRADC_CMD_CLKSEL                        ((uint32_t)(0x7UL << MXC_F_MSRADC_CMD_CLKSEL_POS)) /**< CMD_CLKSEL Mask */
#define MXC_V_MSRADC_CMD_CLKSEL_3_SAMPLES              ((uint32_t)0x0UL) /**< CMD_CLKSEL_3_SAMPLES Value */
#define MXC_S_MSRADC_CMD_CLKSEL_3_SAMPLES              (MXC_V_MSRADC_CMD_CLKSEL_3_SAMPLES << MXC_F_MSRADC_CMD_CLKSEL_POS) /**< CMD_CLKSEL_3_SAMPLES Setting */
#define MXC_V_MSRADC_CMD_CLKSEL_5_SAMPLES              ((uint32_t)0x1UL) /**< CMD_CLKSEL_5_SAMPLES Value */
#define MXC_S_MSRADC_CMD_CLKSEL_5_SAMPLES              (MXC_V_MSRADC_CMD_CLKSEL_5_SAMPLES << MXC_F_MSRADC_CMD_CLKSEL_POS) /**< CMD_CLKSEL_5_SAMPLES Setting */
#define MXC_V_MSRADC_CMD_CLKSEL_4_SAMPLES              ((uint32_t)0x2UL) /**< CMD_CLKSEL_4_SAMPLES Value */
#define MXC_S_MSRADC_CMD_CLKSEL_4_SAMPLES              (MXC_V_MSRADC_CMD_CLKSEL_4_SAMPLES << MXC_F_MSRADC_CMD_CLKSEL_POS) /**< CMD_CLKSEL_4_SAMPLES Setting */
#define MXC_V_MSRADC_CMD_CLKSEL_8_SAMPLES              ((uint32_t)0x3UL) /**< CMD_CLKSEL_8_SAMPLES Value */
#define MXC_S_MSRADC_CMD_CLKSEL_8_SAMPLES              (MXC_V_MSRADC_CMD_CLKSEL_8_SAMPLES << MXC_F_MSRADC_CMD_CLKSEL_POS) /**< CMD_CLKSEL_8_SAMPLES Setting */
#define MXC_V_MSRADC_CMD_CLKSEL_16_SAMPLES             ((uint32_t)0x4UL) /**< CMD_CLKSEL_16_SAMPLES Value */
#define MXC_S_MSRADC_CMD_CLKSEL_16_SAMPLES             (MXC_V_MSRADC_CMD_CLKSEL_16_SAMPLES << MXC_F_MSRADC_CMD_CLKSEL_POS) /**< CMD_CLKSEL_16_SAMPLES Setting */
#define MXC_V_MSRADC_CMD_CLKSEL_32_SAMPLES             ((uint32_t)0x5UL) /**< CMD_CLKSEL_32_SAMPLES Value */
#define MXC_S_MSRADC_CMD_CLKSEL_32_SAMPLES             (MXC_V_MSRADC_CMD_CLKSEL_32_SAMPLES << MXC_F_MSRADC_CMD_CLKSEL_POS) /**< CMD_CLKSEL_32_SAMPLES Setting */
#define MXC_V_MSRADC_CMD_CLKSEL_64_SAMPLES             ((uint32_t)0x6UL) /**< CMD_CLKSEL_64_SAMPLES Value */
#define MXC_S_MSRADC_CMD_CLKSEL_64_SAMPLES             (MXC_V_MSRADC_CMD_CLKSEL_64_SAMPLES << MXC_F_MSRADC_CMD_CLKSEL_POS) /**< CMD_CLKSEL_64_SAMPLES Setting */
#define MXC_V_MSRADC_CMD_CLKSEL_128_SAMPLES            ((uint32_t)0x7UL) /**< CMD_CLKSEL_128_SAMPLES Value */
#define MXC_S_MSRADC_CMD_CLKSEL_128_SAMPLES            (MXC_V_MSRADC_CMD_CLKSEL_128_SAMPLES << MXC_F_MSRADC_CMD_CLKSEL_POS) /**< CMD_CLKSEL_128_SAMPLES Setting */

/**@} end of group MSRADC_CMD_Register */

/**
 * @ingroup  msradc_registers
 * @defgroup MSRADC_FIFO MSRADC_FIFO
 * @brief    ADC FIFO
 * @{
 */
#define MXC_F_MSRADC_FIFO_SAMPLE_POS                   0 /**< FIFO_SAMPLE Position */
#define MXC_F_MSRADC_FIFO_SAMPLE                       ((uint32_t)(0x1FFUL << MXC_F_MSRADC_FIFO_SAMPLE_POS)) /**< FIFO_SAMPLE Mask */

#define MXC_F_MSRADC_FIFO_INPUT_POS                    9 /**< FIFO_INPUT Position */
#define MXC_F_MSRADC_FIFO_INPUT                        ((uint32_t)(0x7UL << MXC_F_MSRADC_FIFO_INPUT_POS)) /**< FIFO_INPUT Mask */
#define MXC_V_MSRADC_FIFO_INPUT_INVALID                ((uint32_t)0x0UL) /**< FIFO_INPUT_INVALID Value */
#define MXC_S_MSRADC_FIFO_INPUT_INVALID                (MXC_V_MSRADC_FIFO_INPUT_INVALID << MXC_F_MSRADC_FIFO_INPUT_POS) /**< FIFO_INPUT_INVALID Setting */
#define MXC_V_MSRADC_FIFO_INPUT_CH1_IN                 ((uint32_t)0x1UL) /**< FIFO_INPUT_CH1_IN Value */
#define MXC_S_MSRADC_FIFO_INPUT_CH1_IN                 (MXC_V_MSRADC_FIFO_INPUT_CH1_IN << MXC_F_MSRADC_FIFO_INPUT_POS) /**< FIFO_INPUT_CH1_IN Setting */
#define MXC_V_MSRADC_FIFO_INPUT_CH2_IN                 ((uint32_t)0x2UL) /**< FIFO_INPUT_CH2_IN Value */
#define MXC_S_MSRADC_FIFO_INPUT_CH2_IN                 (MXC_V_MSRADC_FIFO_INPUT_CH2_IN << MXC_F_MSRADC_FIFO_INPUT_POS) /**< FIFO_INPUT_CH2_IN Setting */
#define MXC_V_MSRADC_FIFO_INPUT_CH3_IN                 ((uint32_t)0x3UL) /**< FIFO_INPUT_CH3_IN Value */
#define MXC_S_MSRADC_FIFO_INPUT_CH3_IN                 (MXC_V_MSRADC_FIFO_INPUT_CH3_IN << MXC_F_MSRADC_FIFO_INPUT_POS) /**< FIFO_INPUT_CH3_IN Setting */
#define MXC_V_MSRADC_FIFO_INPUT_CH4_IN                 ((uint32_t)0x4UL) /**< FIFO_INPUT_CH4_IN Value */
#define MXC_S_MSRADC_FIFO_INPUT_CH4_IN                 (MXC_V_MSRADC_FIFO_INPUT_CH4_IN << MXC_F_MSRADC_FIFO_INPUT_POS) /**< FIFO_INPUT_CH4_IN Setting */
#define MXC_V_MSRADC_FIFO_INPUT_CH5_IN                 ((uint32_t)0x5UL) /**< FIFO_INPUT_CH5_IN Value */
#define MXC_S_MSRADC_FIFO_INPUT_CH5_IN                 (MXC_V_MSRADC_FIFO_INPUT_CH5_IN << MXC_F_MSRADC_FIFO_INPUT_POS) /**< FIFO_INPUT_CH5_IN Setting */
#define MXC_V_MSRADC_FIFO_INPUT_CH6_IN                 ((uint32_t)0x6UL) /**< FIFO_INPUT_CH6_IN Value */
#define MXC_S_MSRADC_FIFO_INPUT_CH6_IN                 (MXC_V_MSRADC_FIFO_INPUT_CH6_IN << MXC_F_MSRADC_FIFO_INPUT_POS) /**< FIFO_INPUT_CH6_IN Setting */
#define MXC_V_MSRADC_FIFO_INPUT_CH7_IN                 ((uint32_t)0x7UL) /**< FIFO_INPUT_CH7_IN Value */
#define MXC_S_MSRADC_FIFO_INPUT_CH7_IN                 (MXC_V_MSRADC_FIFO_INPUT_CH7_IN << MXC_F_MSRADC_FIFO_INPUT_POS) /**< FIFO_INPUT_CH7_IN Setting */

#define MXC_F_MSRADC_FIFO_INCOMPLETE_POS               12 /**< FIFO_INCOMPLETE Position */
#define MXC_F_MSRADC_FIFO_INCOMPLETE                   ((uint32_t)(0x1UL << MXC_F_MSRADC_FIFO_INCOMPLETE_POS)) /**< FIFO_INCOMPLETE Mask */

/**@} end of group MSRADC_FIFO_Register */

/**
 * @ingroup  msradc_registers
 * @defgroup MSRADC_INTEN MSRADC_INTEN
 * @brief    ADC Interrupt Enable Register
 * @{
 */
#define MXC_F_MSRADC_INTEN_SET_FIFOLVL_POS             0 /**< INTEN_SET_FIFOLVL Position */
#define MXC_F_MSRADC_INTEN_SET_FIFOLVL                 ((uint32_t)(0x1FUL << MXC_F_MSRADC_INTEN_SET_FIFOLVL_POS)) /**< INTEN_SET_FIFOLVL Mask */
#define MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_1      ((uint32_t)0x0UL) /**< INTEN_SET_FIFOLVL_AT_LEAST_1 Value */
#define MXC_S_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_1      (MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_1 << MXC_F_MSRADC_INTEN_SET_FIFOLVL_POS) /**< INTEN_SET_FIFOLVL_AT_LEAST_1 Setting */
#define MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_2      ((uint32_t)0x1UL) /**< INTEN_SET_FIFOLVL_AT_LEAST_2 Value */
#define MXC_S_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_2      (MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_2 << MXC_F_MSRADC_INTEN_SET_FIFOLVL_POS) /**< INTEN_SET_FIFOLVL_AT_LEAST_2 Setting */
#define MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_3      ((uint32_t)0x2UL) /**< INTEN_SET_FIFOLVL_AT_LEAST_3 Value */
#define MXC_S_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_3      (MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_3 << MXC_F_MSRADC_INTEN_SET_FIFOLVL_POS) /**< INTEN_SET_FIFOLVL_AT_LEAST_3 Setting */
#define MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_4      ((uint32_t)0x3UL) /**< INTEN_SET_FIFOLVL_AT_LEAST_4 Value */
#define MXC_S_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_4      (MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_4 << MXC_F_MSRADC_INTEN_SET_FIFOLVL_POS) /**< INTEN_SET_FIFOLVL_AT_LEAST_4 Setting */
#define MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_5      ((uint32_t)0x4UL) /**< INTEN_SET_FIFOLVL_AT_LEAST_5 Value */
#define MXC_S_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_5      (MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_5 << MXC_F_MSRADC_INTEN_SET_FIFOLVL_POS) /**< INTEN_SET_FIFOLVL_AT_LEAST_5 Setting */
#define MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_6      ((uint32_t)0x5UL) /**< INTEN_SET_FIFOLVL_AT_LEAST_6 Value */
#define MXC_S_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_6      (MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_6 << MXC_F_MSRADC_INTEN_SET_FIFOLVL_POS) /**< INTEN_SET_FIFOLVL_AT_LEAST_6 Setting */
#define MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_7      ((uint32_t)0x6UL) /**< INTEN_SET_FIFOLVL_AT_LEAST_7 Value */
#define MXC_S_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_7      (MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_7 << MXC_F_MSRADC_INTEN_SET_FIFOLVL_POS) /**< INTEN_SET_FIFOLVL_AT_LEAST_7 Setting */
#define MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_8      ((uint32_t)0x7UL) /**< INTEN_SET_FIFOLVL_AT_LEAST_8 Value */
#define MXC_S_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_8      (MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_8 << MXC_F_MSRADC_INTEN_SET_FIFOLVL_POS) /**< INTEN_SET_FIFOLVL_AT_LEAST_8 Setting */
#define MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_9      ((uint32_t)0x8UL) /**< INTEN_SET_FIFOLVL_AT_LEAST_9 Value */
#define MXC_S_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_9      (MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_9 << MXC_F_MSRADC_INTEN_SET_FIFOLVL_POS) /**< INTEN_SET_FIFOLVL_AT_LEAST_9 Setting */
#define MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_10     ((uint32_t)0x9UL) /**< INTEN_SET_FIFOLVL_AT_LEAST_10 Value */
#define MXC_S_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_10     (MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_10 << MXC_F_MSRADC_INTEN_SET_FIFOLVL_POS) /**< INTEN_SET_FIFOLVL_AT_LEAST_10 Setting */
#define MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_11     ((uint32_t)0xAUL) /**< INTEN_SET_FIFOLVL_AT_LEAST_11 Value */
#define MXC_S_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_11     (MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_11 << MXC_F_MSRADC_INTEN_SET_FIFOLVL_POS) /**< INTEN_SET_FIFOLVL_AT_LEAST_11 Setting */
#define MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_12     ((uint32_t)0xBUL) /**< INTEN_SET_FIFOLVL_AT_LEAST_12 Value */
#define MXC_S_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_12     (MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_12 << MXC_F_MSRADC_INTEN_SET_FIFOLVL_POS) /**< INTEN_SET_FIFOLVL_AT_LEAST_12 Setting */
#define MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_13     ((uint32_t)0xCUL) /**< INTEN_SET_FIFOLVL_AT_LEAST_13 Value */
#define MXC_S_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_13     (MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_13 << MXC_F_MSRADC_INTEN_SET_FIFOLVL_POS) /**< INTEN_SET_FIFOLVL_AT_LEAST_13 Setting */
#define MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_14     ((uint32_t)0xDUL) /**< INTEN_SET_FIFOLVL_AT_LEAST_14 Value */
#define MXC_S_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_14     (MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_14 << MXC_F_MSRADC_INTEN_SET_FIFOLVL_POS) /**< INTEN_SET_FIFOLVL_AT_LEAST_14 Setting */
#define MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_15     ((uint32_t)0xEUL) /**< INTEN_SET_FIFOLVL_AT_LEAST_15 Value */
#define MXC_S_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_15     (MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_15 << MXC_F_MSRADC_INTEN_SET_FIFOLVL_POS) /**< INTEN_SET_FIFOLVL_AT_LEAST_15 Setting */
#define MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_16     ((uint32_t)0xFUL) /**< INTEN_SET_FIFOLVL_AT_LEAST_16 Value */
#define MXC_S_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_16     (MXC_V_MSRADC_INTEN_SET_FIFOLVL_AT_LEAST_16 << MXC_F_MSRADC_INTEN_SET_FIFOLVL_POS) /**< INTEN_SET_FIFOLVL_AT_LEAST_16 Setting */

#define MXC_F_MSRADC_INTEN_DMAREQ_POS                  5 /**< INTEN_DMAREQ Position */
#define MXC_F_MSRADC_INTEN_DMAREQ                      ((uint32_t)(0x1UL << MXC_F_MSRADC_INTEN_DMAREQ_POS)) /**< INTEN_DMAREQ Mask */

#define MXC_F_MSRADC_INTEN_FIFO_OV_POS                 6 /**< INTEN_FIFO_OV Position */
#define MXC_F_MSRADC_INTEN_FIFO_OV                     ((uint32_t)(0x1UL << MXC_F_MSRADC_INTEN_FIFO_OV_POS)) /**< INTEN_FIFO_OV Mask */

#define MXC_F_MSRADC_INTEN_FIFO_UN_POS                 7 /**< INTEN_FIFO_UN Position */
#define MXC_F_MSRADC_INTEN_FIFO_UN                     ((uint32_t)(0x1UL << MXC_F_MSRADC_INTEN_FIFO_UN_POS)) /**< INTEN_FIFO_UN Mask */

#define MXC_F_MSRADC_INTEN_FIFO_LVL_POS                8 /**< INTEN_FIFO_LVL Position */
#define MXC_F_MSRADC_INTEN_FIFO_LVL                    ((uint32_t)(0x1UL << MXC_F_MSRADC_INTEN_FIFO_LVL_POS)) /**< INTEN_FIFO_LVL Mask */

#define MXC_F_MSRADC_INTEN_GLOBAL_POS                  9 /**< INTEN_GLOBAL Position */
#define MXC_F_MSRADC_INTEN_GLOBAL                      ((uint32_t)(0x1UL << MXC_F_MSRADC_INTEN_GLOBAL_POS)) /**< INTEN_GLOBAL Mask */

/**@} end of group MSRADC_INTEN_Register */

/**
 * @ingroup  msradc_registers
 * @defgroup MSRADC_INTFL MSRADC_INTFL
 * @brief    ADC Interrupt Flag Register.
 * @{
 */
#define MXC_F_MSRADC_INTFL_FIFOCNT_POS                 0 /**< INTFL_FIFOCNT Position */
#define MXC_F_MSRADC_INTFL_FIFOCNT                     ((uint32_t)(0x3FUL << MXC_F_MSRADC_INTFL_FIFOCNT_POS)) /**< INTFL_FIFOCNT Mask */

#define MXC_F_MSRADC_INTFL_FIFO_FULL_ST_POS            6 /**< INTFL_FIFO_FULL_ST Position */
#define MXC_F_MSRADC_INTFL_FIFO_FULL_ST                ((uint32_t)(0x1UL << MXC_F_MSRADC_INTFL_FIFO_FULL_ST_POS)) /**< INTFL_FIFO_FULL_ST Mask */

#define MXC_F_MSRADC_INTFL_FIFO_EM_ST_POS              7 /**< INTFL_FIFO_EM_ST Position */
#define MXC_F_MSRADC_INTFL_FIFO_EM_ST                  ((uint32_t)(0x1UL << MXC_F_MSRADC_INTFL_FIFO_EM_ST_POS)) /**< INTFL_FIFO_EM_ST Mask */

#define MXC_F_MSRADC_INTFL_FIFO_OV_POS                 8 /**< INTFL_FIFO_OV Position */
#define MXC_F_MSRADC_INTFL_FIFO_OV                     ((uint32_t)(0x1UL << MXC_F_MSRADC_INTFL_FIFO_OV_POS)) /**< INTFL_FIFO_OV Mask */

#define MXC_F_MSRADC_INTFL_FIFO_UN_POS                 9 /**< INTFL_FIFO_UN Position */
#define MXC_F_MSRADC_INTFL_FIFO_UN                     ((uint32_t)(0x1UL << MXC_F_MSRADC_INTFL_FIFO_UN_POS)) /**< INTFL_FIFO_UN Mask */

#define MXC_F_MSRADC_INTFL_FIFO_LVL_POS                10 /**< INTFL_FIFO_LVL Position */
#define MXC_F_MSRADC_INTFL_FIFO_LVL                    ((uint32_t)(0x1UL << MXC_F_MSRADC_INTFL_FIFO_LVL_POS)) /**< INTFL_FIFO_LVL Mask */

#define MXC_F_MSRADC_INTFL_GLOBAL_POS                  11 /**< INTFL_GLOBAL Position */
#define MXC_F_MSRADC_INTFL_GLOBAL                      ((uint32_t)(0x1UL << MXC_F_MSRADC_INTFL_GLOBAL_POS)) /**< INTFL_GLOBAL Mask */

/**@} end of group MSRADC_INTFL_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_MSRADC_REGS_H_
