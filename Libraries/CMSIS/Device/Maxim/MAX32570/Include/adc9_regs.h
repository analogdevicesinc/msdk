/**
 * @file    adc_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the MSRADC Peripheral Module.
 */

/* ****************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 * $Date: 2017-10-25 16:40:04 -0500 (Wed, 25 Oct 2017) $
 * $Revision: 31589 $
 *
 *************************************************************************** */

#ifndef _MSRADC_REGS_H_
#define _MSRADC_REGS_H_

/* **** Includes **** */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
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
 * @ingroup     adc
 * @defgroup    adc_registers ADC_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the MSRADC Peripheral Module.
 * @description Analog to Digital Converter
 */

/**
 * @ingroup adc_registers
 * Structure type to access the MSRADC Registers.
 */
typedef struct {
    __IO uint32_t cfg;                  /**< <tt>\b 0x00:<\tt> MSRADC CFG Register */
    __IO uint32_t cmd;                  /**< <tt>\b 0x04:<\tt> MSRADC CMD Register */
    __IO uint32_t fifo;                 /**< <tt>\b 0x08:<\tt> MSRADC FIFO Register */
    __IO uint32_t intr;                  /**< <tt>\b 0x0C:<\tt> MSRADC INTR Register */
    __IO uint32_t stat;                 /**< <tt>\b 0x10:<\tt> MSRADC STAT Register */
} mxc_msradc_regs_t;

/* Register offsets for module MSRADC */
/**
 * @ingroup    adc_registers
 * @defgroup   MSRADC_Register_Offsets Register Offsets
 * @brief      MSRADC Peripheral Register Offsets from the MSRADC Base Peripheral Address. 
 * @{
 */
 #define MXC_R_MSRADC_CFG                      ((uint32_t)0x00000000UL) /**< Offset from MSRADC Base Address: <tt> 0x0x000 */ 
 #define MXC_R_MSRADC_CMD                      ((uint32_t)0x00000004UL) /**< Offset from MSRADC Base Address: <tt> 0x0x004 */ 
 #define MXC_R_MSRADC_FIFO                     ((uint32_t)0x00000008UL) /**< Offset from MSRADC Base Address: <tt> 0x0x008 */ 
 #define MXC_R_MSRADC_INTR                      ((uint32_t)0x0000000CUL) /**< Offset from MSRADC Base Address: <tt> 0x0x00C */ 
 #define MXC_R_MSRADC_STAT                     ((uint32_t)0x00000010UL) /**< Offset from MSRADC Base Address: <tt> 0x0x010 */ 
/**@} end of group adc_registers */

/**
 * @ingroup  adc_registers
 * @defgroup CFG_Register
 * @brief    MSRADC Configuration Register
 * @{
 */
 #define MXC_F_MSRADC_CFG_CLKDIV_POS                    (0) /**< CFG_MSRADCCLKDIV Position */
 #define MXC_F_MSRADC_CFG_CLKDIV                        ((uint32_t)(0xFF << MXC_F_MSRADC_CFG_CLKDIV_POS)) /**< CFG_MSRADCCLKDIV Mask */

 #define MXC_F_MSRADC_CFG_ACHSEL_POS                       (8) /**< CFG_ACHSEL Position */
 #define MXC_F_MSRADC_CFG_ACHSEL                           ((uint32_t)(0x7 << MXC_F_MSRADC_CFG_ACHSEL_POS)) /**< CFG_ACHSEL Mask */
 #define MXC_V_MSRADC_CFG_ACHSEL_INVALID_000               ((uint32_t)0x0) /**< CFG_ACHSEL_INVALID_000 Value */
 #define MXC_S_MSRADC_CFG_ACHSEL_INVALID_000               (MXC_V_MSRADC_CFG_ACHSEL_INVALID_000 << MXC_F_MSRADC_CFG_ACHSEL_POS) /**< CFG_ACHSEL_INVALID_000 Setting */
 #define MXC_V_MSRADC_CFG_ACHSEL_IN0                  ((uint32_t)0x1) /**< CFG_ACHSEL_IN0 Value */
 #define MXC_S_MSRADC_CFG_ACHSEL_IN0                   (MXC_V_MSRADC_CFG_ACHSEL_IN0 << MXC_F_MSRADC_CFG_ACHSEL_POS) /**< CFG_ACHSEL_IN0 Setting */
 #define MXC_V_MSRADC_CFG_ACHSEL_IN1                   ((uint32_t)0x2) /**< CFG_ACHSEL_IN1 Value */
 #define MXC_S_MSRADC_CFG_ACHSEL_IN1                   (MXC_V_MSRADC_CFG_ACHSEL_IN1 << MXC_F_MSRADC_CFG_ACHSEL_POS) /**< CFG_ACHSEL_IN1 Setting */
 #define MXC_V_MSRADC_CFG_ACHSEL_IN2                   ((uint32_t)0x3) /**< CFG_ACHSEL_IN2 Value */
 #define MXC_S_MSRADC_CFG_ACHSEL_IN2                   (MXC_V_MSRADC_CFG_ACHSEL_IN2 << MXC_F_MSRADC_CFG_ACHSEL_POS) /**< CFG_ACHSEL_IN2 Setting */
 #define MXC_V_MSRADC_CFG_ACHSEL_IN3                   ((uint32_t)0x4) /**< CFG_ACHSEL_IN3 Value */
 #define MXC_S_MSRADC_CFG_ACHSEL_IN3                   (MXC_V_MSRADC_CFG_ACHSEL_IN3 << MXC_F_MSRADC_CFG_ACHSEL_POS) /**< CFG_ACHSEL_IN3 Setting */
 #define MXC_V_MSRADC_CFG_ACHSEL_IN4                   ((uint32_t)0x5) /**< CFG_ACHSEL_IN4 Value */
 #define MXC_S_MSRADC_CFG_ACHSEL_IN4                   (MXC_V_MSRADC_CFG_ACHSEL_IN4 << MXC_F_MSRADC_CFG_ACHSEL_POS) /**< CFG_ACHSEL_IN4 Setting */
 #define MXC_V_MSRADC_CFG_ACHSEL_IN5                   ((uint32_t)0x6) /**< CFG_ACHSEL_IN5 Value */
 #define MXC_S_MSRADC_CFG_ACHSEL_IN5                   (MXC_V_MSRADC_CFG_ACHSEL_IN5 << MXC_F_MSRADC_CFG_ACHSEL_POS) /**< CFG_ACHSEL_IN5 Setting */
 #define MXC_V_MSRADC_CFG_ACHSEL_INVALID_111               ((uint32_t)0x7) /**< CFG_ACHSEL_INVALID_111 Value */
 #define MXC_S_MSRADC_CFG_ACHSEL_INVALID_111               (MXC_V_MSRADC_CFG_ACHSEL_INVALID_111 << MXC_F_MSRADC_CFG_ACHSEL_POS) /**< CFG_ACHSEL_INVALID_111 Setting */

 #define MXC_F_MSRADC_CFG_BCHSEL_POS                       (11) /**< CFG_BCHSEL Position */
 #define MXC_F_MSRADC_CFG_BCHSEL                           ((uint32_t)(0x7 << MXC_F_MSRADC_CFG_BCHSEL_POS)) /**< CFG_BCHSEL Mask */

 #define MXC_F_MSRADC_CFG_CCHSEL_POS                       (14) /**< CFG_CCHSEL Position */
 #define MXC_F_MSRADC_CFG_CCHSEL                           ((uint32_t)(0x7 << MXC_F_MSRADC_CFG_CCHSEL_POS)) /**< CFG_CCHSEL Mask */

 #define MXC_F_MSRADC_CFG_DCHSEL_POS                       (17) /**< CFG_DCHSEL Position */
 #define MXC_F_MSRADC_CFG_DCHSEL                           ((uint32_t)(0x7 << MXC_F_MSRADC_CFG_DCHSEL_POS)) /**< CFG_DCHSEL Mask */

 #define MXC_F_MSRADC_CFG_ECHSEL_POS                       (20) /**< CFG_ECHSEL Position */
 #define MXC_F_MSRADC_CFG_ECHSEL                           ((uint32_t)(0x7 << MXC_F_MSRADC_CFG_ECHSEL_POS)) /**< CFG_ECHSEL Mask */

 #define MXC_F_MSRADC_CFG_FCHSEL_POS                       (23) /**< CFG_FCHSEL Position */
 #define MXC_F_MSRADC_CFG_FCHSEL                           ((uint32_t)(0x7 << MXC_F_MSRADC_CFG_FCHSEL_POS)) /**< CFG_FCHSEL Mask */

 #define MXC_F_MSRADC_CFG_GCHSEL_POS                       (26) /**< CFG_GCHSEL Position */
 #define MXC_F_MSRADC_CFG_GCHSEL                           ((uint32_t)(0x7 << MXC_F_MSRADC_CFG_GCHSEL_POS)) /**< CFG_GCHSEL Mask */

 #define MXC_F_MSRADC_CFG_HCHSEL_POS                       (29) /**< CFG_HCHSEL Position */
 #define MXC_F_MSRADC_CFG_HCHSEL                           ((uint32_t)(0x7 << MXC_F_MSRADC_CFG_HCHSEL_POS)) /**< CFG_HCHSEL Mask */

/**@} end of group CFG_Register */

/**
 * @ingroup  adc_registers
 * @defgroup CMD_Register
 * @brief    MSRADC Command Register
 * @{
 */
 #define MXC_F_MSRADC_CMD_RST_POS                       (0) /**< CMD_RST Position */
 #define MXC_F_MSRADC_CMD_RST                           ((uint32_t)(0x1 << MXC_F_MSRADC_CMD_RST_POS)) /**< CMD_RST Mask */
 #define MXC_V_MSRADC_CMD_RST_NO_RESET                  ((uint32_t)0x0) /**< CMD_RST_NO_RESET Value */
 #define MXC_S_MSRADC_CMD_RST_NO_RESET                  (MXC_V_MSRADC_CMD_RST_NO_RESET << MXC_F_MSRADC_CMD_RST_POS) /**< CMD_RST_NO_RESET Setting */
 #define MXC_V_MSRADC_CMD_RST_RESET                     ((uint32_t)0x1) /**< CMD_RST_RESET Value */
 #define MXC_S_MSRADC_CMD_RST_RESET                     (MXC_V_MSRADC_CMD_RST_RESET << MXC_F_MSRADC_CMD_RST_POS) /**< CMD_RST_RESET Setting */

 #define MXC_F_MSRADC_CMD_SNGLSMPL_POS                     (1) /**< CMD_SNGLSMPL Position */
 #define MXC_F_MSRADC_CMD_SNGLSMPL                         ((uint32_t)(0x1 << MXC_F_MSRADC_CMD_SNGLSMPL_POS)) /**< CMD_SNGLSMPL Mask */
 #define MXC_V_MSRADC_CMD_SNGLSMPL_NO_EFFECT               ((uint32_t)0x0) /**< CMD_SNGLSMPL_NO_EFFECT Value */
 #define MXC_S_MSRADC_CMD_SNGLSMPL_NO_EFFECT               (MXC_V_MSRADC_CMD_SNGLSMPL_NO_EFFECT << MXC_F_MSRADC_CMD_SNGLSMPL_POS) /**< CMD_SNGLSMPL_NO_EFFECT Setting */
 #define MXC_V_MSRADC_CMD_SNGLSMPL_SINGLE_SMPL             ((uint32_t)0x1) /**< CMD_SNGLSMPL_SINGLE_SMPL Value */
 #define MXC_S_MSRADC_CMD_SNGLSMPL_SINGLE_SMPL             (MXC_V_MSRADC_CMD_SNGLSMPL_SINGLE_SMPL << MXC_F_MSRADC_CMD_SNGLSMPL_POS) /**< CMD_SNGLSMPL_SINGLE_SMPL Setting */

 #define MXC_F_MSRADC_CMD_CONTSMPL_POS                     (2) /**< CMD_CONTSMPL Position */
 #define MXC_F_MSRADC_CMD_CONTSMPL                         ((uint32_t)(0x1 << MXC_F_MSRADC_CMD_CONTSMPL_POS)) /**< CMD_CONTSMPL Mask */
 #define MXC_V_MSRADC_CMD_CONTSMPL_NOT_CONTINUOUS_SMPL_MODE ((uint32_t)0x0) /**< CMD_CONTSMPL_NOT_CONTINUOUS_SMPL_MODE Value */
 #define MXC_S_MSRADC_CMD_CONTSMPL_NOT_CONTINUOUS_SMPL_MODE (MXC_V_MSRADC_CMD_CONTSMPL_NOT_CONTINUOUS_SMPL_MODE << MXC_F_MSRADC_CMD_CONTSMPL_POS) /**< CMD_CONTSMPL_NOT_CONTINUOUS_SMPL_MODE Setting */
 #define MXC_V_MSRADC_CMD_CONTSMPL_CONTINUOUS_SMPL_MODE    ((uint32_t)0x1) /**< CMD_CONTSMPL_CONTINUOUS_SMPL_MODE Value */
 #define MXC_S_MSRADC_CMD_CONTSMPL_CONTINUOUS_SMPL_MODE    (MXC_V_MSRADC_CMD_CONTSMPL_CONTINUOUS_SMPL_MODE << MXC_F_MSRADC_CMD_CONTSMPL_POS) /**< CMD_CONTSMPL_CONTINUOUS_SMPL_MODE Setting */

 #define MXC_F_MSRADC_CMD_ROTLIMIT_POS                     (4) /**< CMD_ROTLIMIT Position */
 #define MXC_F_MSRADC_CMD_ROTLIMIT                         ((uint32_t)(0x7 << MXC_F_MSRADC_CMD_ROTLIMIT_POS)) /**< CMD_ROTLIMIT Mask */
 #define MXC_V_MSRADC_CMD_ROTLIMIT_1_CHANNEL               ((uint32_t)0x0) /**< CMD_ROTLIMIT_1_CHANNEL Value */
 #define MXC_S_MSRADC_CMD_ROTLIMIT_1_CHANNEL               (MXC_V_MSRADC_CMD_ROTLIMIT_1_CHANNEL << MXC_F_MSRADC_CMD_ROTLIMIT_POS) /**< CMD_ROTLIMIT_1_CHANNEL Setting */
 #define MXC_V_MSRADC_CMD_ROTLIMIT_2_CHANNELS              ((uint32_t)0x1) /**< CMD_ROTLIMIT_2_CHANNELS Value */
 #define MXC_S_MSRADC_CMD_ROTLIMIT_2_CHANNELS              (MXC_V_MSRADC_CMD_ROTLIMIT_2_CHANNELS << MXC_F_MSRADC_CMD_ROTLIMIT_POS) /**< CMD_ROTLIMIT_2_CHANNELS Setting */
 #define MXC_V_MSRADC_CMD_ROTLIMIT_3_CHANNELS              ((uint32_t)0x2) /**< CMD_ROTLIMIT_3_CHANNELS Value */
 #define MXC_S_MSRADC_CMD_ROTLIMIT_3_CHANNELS              (MXC_V_MSRADC_CMD_ROTLIMIT_3_CHANNELS << MXC_F_MSRADC_CMD_ROTLIMIT_POS) /**< CMD_ROTLIMIT_3_CHANNELS Setting */
 #define MXC_V_MSRADC_CMD_ROTLIMIT_4_CHANNELS              ((uint32_t)0x3) /**< CMD_ROTLIMIT_4_CHANNELS Value */
 #define MXC_S_MSRADC_CMD_ROTLIMIT_4_CHANNELS              (MXC_V_MSRADC_CMD_ROTLIMIT_4_CHANNELS << MXC_F_MSRADC_CMD_ROTLIMIT_POS) /**< CMD_ROTLIMIT_4_CHANNELS Setting */
 #define MXC_V_MSRADC_CMD_ROTLIMIT_5_CHANNELS              ((uint32_t)0x4) /**< CMD_ROTLIMIT_5_CHANNELS Value */
 #define MXC_S_MSRADC_CMD_ROTLIMIT_5_CHANNELS              (MXC_V_MSRADC_CMD_ROTLIMIT_5_CHANNELS << MXC_F_MSRADC_CMD_ROTLIMIT_POS) /**< CMD_ROTLIMIT_5_CHANNELS Setting */
 #define MXC_V_MSRADC_CMD_ROTLIMIT_6_CHANNELS              ((uint32_t)0x5) /**< CMD_ROTLIMIT_6_CHANNELS Value */
 #define MXC_S_MSRADC_CMD_ROTLIMIT_6_CHANNELS              (MXC_V_MSRADC_CMD_ROTLIMIT_6_CHANNELS << MXC_F_MSRADC_CMD_ROTLIMIT_POS) /**< CMD_ROTLIMIT_6_CHANNELS Setting */
 #define MXC_V_MSRADC_CMD_ROTLIMIT_7_CHANNELS              ((uint32_t)0x6) /**< CMD_ROTLIMIT_7_CHANNELS Value */
 #define MXC_S_MSRADC_CMD_ROTLIMIT_7_CHANNELS              (MXC_V_MSRADC_CMD_ROTLIMIT_7_CHANNELS << MXC_F_MSRADC_CMD_ROTLIMIT_POS) /**< CMD_ROTLIMIT_7_CHANNELS Setting */
 #define MXC_V_MSRADC_CMD_ROTLIMIT_8_CHANNELS              ((uint32_t)0x7) /**< CMD_ROTLIMIT_8_CHANNELS Value */
 #define MXC_S_MSRADC_CMD_ROTLIMIT_8_CHANNELS              (MXC_V_MSRADC_CMD_ROTLIMIT_8_CHANNELS << MXC_F_MSRADC_CMD_ROTLIMIT_POS) /**< CMD_ROTLIMIT_8_CHANNELS Setting */

 #define MXC_F_MSRADC_CMD_CLKSEL_POS                       (8) /**< CMD_CLKSEL Position */
 #define MXC_F_MSRADC_CMD_CLKSEL                           ((uint32_t)(0x7 << MXC_F_MSRADC_CMD_CLKSEL_POS)) /**< CMD_CLKSEL Mask */
 #define MXC_V_MSRADC_CMD_CLKSEL_3_SAMPLES                 ((uint32_t)0x0) /**< CMD_CLKSEL_3_SAMPLES Value */
 #define MXC_S_MSRADC_CMD_CLKSEL_3_SAMPLES                 (MXC_V_MSRADC_CMD_CLKSEL_3_SAMPLES << MXC_F_MSRADC_CMD_CLKSEL_POS) /**< CMD_CLKSEL_3_SAMPLES Setting */
 #define MXC_V_MSRADC_CMD_CLKSEL_5_SAMPLES                 ((uint32_t)0x1) /**< CMD_CLKSEL_5_SAMPLES Value */
 #define MXC_S_MSRADC_CMD_CLKSEL_5_SAMPLES                 (MXC_V_MSRADC_CMD_CLKSEL_5_SAMPLES << MXC_F_MSRADC_CMD_CLKSEL_POS) /**< CMD_CLKSEL_5_SAMPLES Setting */
 #define MXC_V_MSRADC_CMD_CLKSEL_4_SAMPLES                 ((uint32_t)0x2) /**< CMD_CLKSEL_4_SAMPLES Value */
 #define MXC_S_MSRADC_CMD_CLKSEL_4_SAMPLES                 (MXC_V_MSRADC_CMD_CLKSEL_4_SAMPLES << MXC_F_MSRADC_CMD_CLKSEL_POS) /**< CMD_CLKSEL_4_SAMPLES Setting */
 #define MXC_V_MSRADC_CMD_CLKSEL_8_SAMPLES                 ((uint32_t)0x3) /**< CMD_CLKSEL_8_SAMPLES Value */
 #define MXC_S_MSRADC_CMD_CLKSEL_8_SAMPLES                 (MXC_V_MSRADC_CMD_CLKSEL_8_SAMPLES << MXC_F_MSRADC_CMD_CLKSEL_POS) /**< CMD_CLKSEL_8_SAMPLES Setting */
 #define MXC_V_MSRADC_CMD_CLKSEL_16_SAMPLES                ((uint32_t)0x4) /**< CMD_CLKSEL_16_SAMPLES Value */
 #define MXC_S_MSRADC_CMD_CLKSEL_16_SAMPLES                (MXC_V_MSRADC_CMD_CLKSEL_16_SAMPLES << MXC_F_MSRADC_CMD_CLKSEL_POS) /**< CMD_CLKSEL_16_SAMPLES Setting */
 #define MXC_V_MSRADC_CMD_CLKSEL_32_SAMPLES                ((uint32_t)0x5) /**< CMD_CLKSEL_32_SAMPLES Value */
 #define MXC_S_MSRADC_CMD_CLKSEL_32_SAMPLES                (MXC_V_MSRADC_CMD_CLKSEL_32_SAMPLES << MXC_F_MSRADC_CMD_CLKSEL_POS) /**< CMD_CLKSEL_32_SAMPLES Setting */
 #define MXC_V_MSRADC_CMD_CLKSEL_64_SAMPLES                ((uint32_t)0x6) /**< CMD_CLKSEL_64_SAMPLES Value */
 #define MXC_S_MSRADC_CMD_CLKSEL_64_SAMPLES                (MXC_V_MSRADC_CMD_CLKSEL_64_SAMPLES << MXC_F_MSRADC_CMD_CLKSEL_POS) /**< CMD_CLKSEL_64_SAMPLES Setting */
 #define MXC_V_MSRADC_CMD_CLKSEL_128_SAMPLES               ((uint32_t)0x7) /**< CMD_CLKSEL_128_SAMPLES Value */
 #define MXC_S_MSRADC_CMD_CLKSEL_128_SAMPLES               (MXC_V_MSRADC_CMD_CLKSEL_128_SAMPLES << MXC_F_MSRADC_CMD_CLKSEL_POS) /**< CMD_CLKSEL_128_SAMPLES Setting */

/**@} end of group CMD_Register */

/**
 * @ingroup  adc_registers
 * @defgroup FIFO_Register
 * @brief    MSRADC FIFO Register
 * @{
 */
 #define MXC_F_MSRADC_FIFO_SAMPLE_POS                   (0) /**< FIFO_SAMPLE Position */
 #define MXC_F_MSRADC_FIFO_SAMPLE                       ((uint32_t)(0x1FF << MXC_F_MSRADC_FIFO_SAMPLE_POS)) /**< FIFO_SAMPLE Mask */

 #define MXC_F_MSRADC_FIFO_SMPLPIN_POS                     (9) /**< FIFO_SMPLPIN Position */
 #define MXC_F_MSRADC_FIFO_SMPLPIN                         ((uint32_t)(0x7 << MXC_F_MSRADC_FIFO_SMPLPIN_POS)) /**< FIFO_SMPLPIN Mask */
 #define MXC_V_MSRADC_FIFO_SMPLPIN_INVALID_000             ((uint32_t)0x0) /**< FIFO_SMPLPIN_INVALID_000 Value */
 #define MXC_S_MSRADC_FIFO_SMPLPIN_INVALID_000             (MXC_V_MSRADC_FIFO_SMPLPIN_INVALID_000 << MXC_F_MSRADC_FIFO_SMPLPIN_POS) /**< FIFO_SMPLPIN_INVALID_000 Setting */
 #define MXC_V_MSRADC_FIFO_SMPLPIN_IN0                 ((uint32_t)0x1) /**< FIFO_SMPLPIN_IN0 Value */
 #define MXC_S_MSRADC_FIFO_SMPLPIN_IN0                 (MXC_V_MSRADC_FIFO_SMPLPIN_IN0 << MXC_F_MSRADC_FIFO_SMPLPIN_POS) /**< FIFO_SMPLPIN_IN0 Setting */
 #define MXC_V_MSRADC_FIFO_SMPLPIN_IN1                 ((uint32_t)0x2) /**< FIFO_SMPLPIN_IN1 Value */
 #define MXC_S_MSRADC_FIFO_SMPLPIN_IN1                 (MXC_V_MSRADC_FIFO_SMPLPIN_IN1 << MXC_F_MSRADC_FIFO_SMPLPIN_POS) /**< FIFO_SMPLPIN_IN1 Setting */
 #define MXC_V_MSRADC_FIFO_SMPLPIN_IN2                 ((uint32_t)0x3) /**< FIFO_SMPLPIN_IN2 Value */
 #define MXC_S_MSRADC_FIFO_SMPLPIN_IN2                 (MXC_V_MSRADC_FIFO_SMPLPIN_IN2 << MXC_F_MSRADC_FIFO_SMPLPIN_POS) /**< FIFO_SMPLPIN_IN2 Setting */
 #define MXC_V_MSRADC_FIFO_SMPLPIN_IN3                 ((uint32_t)0x4) /**< FIFO_SMPLPIN_IN3 Value */
 #define MXC_S_MSRADC_FIFO_SMPLPIN_IN3                 (MXC_V_MSRADC_FIFO_SMPLPIN_IN3 << MXC_F_MSRADC_FIFO_SMPLPIN_POS) /**< FIFO_SMPLPIN_IN3 Setting */
 #define MXC_V_MSRADC_FIFO_SMPLPIN_IN4                 ((uint32_t)0x5) /**< FIFO_SMPLPIN_IN4 Value */
 #define MXC_S_MSRADC_FIFO_SMPLPIN_IN4                 (MXC_V_MSRADC_FIFO_SMPLPIN_IN4 << MXC_F_MSRADC_FIFO_SMPLPIN_POS) /**< FIFO_SMPLPIN_IN4 Setting */
 #define MXC_V_MSRADC_FIFO_SMPLPIN_IN5                 ((uint32_t)0x6) /**< FIFO_SMPLPIN_IN5 Value */
 #define MXC_S_MSRADC_FIFO_SMPLPIN_IN5                 (MXC_V_MSRADC_FIFO_SMPLPIN_IN5 << MXC_F_MSRADC_FIFO_SMPLPIN_POS) /**< FIFO_SMPLPIN_IN5 Setting */
 #define MXC_V_MSRADC_FIFO_SMPLPIN_INVALID_111             ((uint32_t)0x7) /**< FIFO_SMPLPIN_INVALID_111 Value */
 #define MXC_S_MSRADC_FIFO_SMPLPIN_INVALID_111             (MXC_V_MSRADC_FIFO_SMPLPIN_INVALID_111 << MXC_F_MSRADC_FIFO_SMPLPIN_POS) /**< FIFO_SMPLPIN_INVALID_111 Setting */

/**@} end of group FIFO_Register */

/**
 * @ingroup  adc_registers
 * @defgroup INT_Register
 * @brief    MSRADC Interrupt Register
 * @{
 */
 #define MXC_F_MSRADC_INTR_FIFOLVL_POS                      (0) /**< INT_FIFOLVL Position */
 #define MXC_F_MSRADC_INTR_FIFOLVL                          ((uint32_t)(0x7 << MXC_F_MSRADC_INTR_FIFOLVL_POS)) /**< INT_FIFOLVL Mask */
 #define MXC_V_MSRADC_INTR_FIFOLVL_AT_LEAST_ONE             ((uint32_t)0x0) /**< INT_FIFOLVL_AT_LEAST_ONE Value */
 #define MXC_S_MSRADC_INTR_FIFOLVL_AT_LEAST_ONE             (MXC_V_MSRADC_INTR_FIFOLVL_AT_LEAST_ONE << MXC_F_MSRADC_INTR_FIFOLVL_POS) /**< INT_FIFOLVL_AT_LEAST_ONE Setting */
 #define MXC_V_MSRADC_INTR_FIFOLVL_AT_LEAST_TWO             ((uint32_t)0x1) /**< INT_FIFOLVL_AT_LEAST_TWO Value */
 #define MXC_S_MSRADC_INTR_FIFOLVL_AT_LEAST_TWO             (MXC_V_MSRADC_INTR_FIFOLVL_AT_LEAST_TWO << MXC_F_MSRADC_INTR_FIFOLVL_POS) /**< INT_FIFOLVL_AT_LEAST_TWO Setting */
 #define MXC_V_MSRADC_INTR_FIFOLVL_AT_LEAST_THREE           ((uint32_t)0x2) /**< INT_FIFOLVL_AT_LEAST_THREE Value */
 #define MXC_S_MSRADC_INTR_FIFOLVL_AT_LEAST_THREE           (MXC_V_MSRADC_INTR_FIFOLVL_AT_LEAST_THREE << MXC_F_MSRADC_INTR_FIFOLVL_POS) /**< INT_FIFOLVL_AT_LEAST_THREE Setting */
 #define MXC_V_MSRADC_INTR_FIFOLVL_AT_LEAST_FOUR            ((uint32_t)0x3) /**< INT_FIFOLVL_AT_LEAST_FOUR Value */
 #define MXC_S_MSRADC_INTR_FIFOLVL_AT_LEAST_FOUR            (MXC_V_MSRADC_INTR_FIFOLVL_AT_LEAST_FOUR << MXC_F_MSRADC_INTR_FIFOLVL_POS) /**< INT_FIFOLVL_AT_LEAST_FOUR Setting */
 #define MXC_V_MSRADC_INTR_FIFOLVL_AT_LEAST_FIVE            ((uint32_t)0x4) /**< INT_FIFOLVL_AT_LEAST_FIVE Value */
 #define MXC_S_MSRADC_INTR_FIFOLVL_AT_LEAST_FIVE            (MXC_V_MSRADC_INTR_FIFOLVL_AT_LEAST_FIVE << MXC_F_MSRADC_INTR_FIFOLVL_POS) /**< INT_FIFOLVL_AT_LEAST_FIVE Setting */
 #define MXC_V_MSRADC_INTR_FIFOLVL_AT_LEAST_SIX             ((uint32_t)0x5) /**< INT_FIFOLVL_AT_LEAST_SIX Value */
 #define MXC_S_MSRADC_INTR_FIFOLVL_AT_LEAST_SIX             (MXC_V_MSRADC_INTR_FIFOLVL_AT_LEAST_SIX << MXC_F_MSRADC_INTR_FIFOLVL_POS) /**< INT_FIFOLVL_AT_LEAST_SIX Setting */
 #define MXC_V_MSRADC_INTR_FIFOLVL_AT_LEAST_SEVEN           ((uint32_t)0x6) /**< INT_FIFOLVL_AT_LEAST_SEVEN Value */
 #define MXC_S_MSRADC_INTR_FIFOLVL_AT_LEAST_SEVEN           (MXC_V_MSRADC_INTR_FIFOLVL_AT_LEAST_SEVEN << MXC_F_MSRADC_INTR_FIFOLVL_POS) /**< INT_FIFOLVL_AT_LEAST_SEVEN Setting */
 #define MXC_V_MSRADC_INTR_FIFOLVL_AT_LEAST_EIGHT           ((uint32_t)0x7) /**< INT_FIFOLVL_AT_LEAST_EIGHT Value */
 #define MXC_S_MSRADC_INTR_FIFOLVL_AT_LEAST_EIGHT           (MXC_V_MSRADC_INTR_FIFOLVL_AT_LEAST_EIGHT << MXC_F_MSRADC_INTR_FIFOLVL_POS) /**< INT_FIFOLVL_AT_LEAST_EIGHT Setting */

 #define MXC_F_MSRADC_INTR_DMAREQEN_POS                     (3) /**< INT_DMAREQEN Position */
 #define MXC_F_MSRADC_INTR_DMAREQEN                         ((uint32_t)(0x1 << MXC_F_MSRADC_INTR_DMAREQEN_POS)) /**< INT_DMAREQEN Mask */
 #define MXC_V_MSRADC_INTR_DMAREQEN_DISABLED                ((uint32_t)0x0) /**< INT_DMAREQEN_DISABLED Value */
 #define MXC_S_MSRADC_INTR_DMAREQEN_DISABLED                (MXC_V_MSRADC_INTR_DMAREQEN_DISABLED << MXC_F_MSRADC_INTR_DMAREQEN_POS) /**< INT_DMAREQEN_DISABLED Setting */
 #define MXC_V_MSRADC_INTR_DMAREQEN_ENABLED                 ((uint32_t)0x1) /**< INT_DMAREQEN_ENABLED Value */
 #define MXC_S_MSRADC_INTR_DMAREQEN_ENABLED                 (MXC_V_MSRADC_INTR_DMAREQEN_ENABLED << MXC_F_MSRADC_INTR_DMAREQEN_POS) /**< INT_DMAREQEN_ENABLED Setting */

 #define MXC_F_MSRADC_INTR_OVERFIE_POS                      (6) /**< INT_OVERFIE Position */
 #define MXC_F_MSRADC_INTR_OVERFIE                          ((uint32_t)(0x1 << MXC_F_MSRADC_INTR_OVERFIE_POS)) /**< INT_OVERFIE Mask */
 #define MXC_V_MSRADC_INTR_OVERFIE_DISABLED                 ((uint32_t)0x0) /**< INT_OVERFIE_DISABLED Value */
 #define MXC_S_MSRADC_INTR_OVERFIE_DISABLED                 (MXC_V_MSRADC_INTR_OVERFIE_DISABLED << MXC_F_MSRADC_INTR_OVERFIE_POS) /**< INT_OVERFIE_DISABLED Setting */
 #define MXC_V_MSRADC_INTR_OVERFIE_ENABLED                  ((uint32_t)0x1) /**< INT_OVERFIE_ENABLED Value */
 #define MXC_S_MSRADC_INTR_OVERFIE_ENABLED                  (MXC_V_MSRADC_INTR_OVERFIE_ENABLED << MXC_F_MSRADC_INTR_OVERFIE_POS) /**< INT_OVERFIE_ENABLED Setting */

 #define MXC_F_MSRADC_INTR_UNDRFIE_POS                      (7) /**< INT_UNDRFIE Position */
 #define MXC_F_MSRADC_INTR_UNDRFIE                          ((uint32_t)(0x1 << MXC_F_MSRADC_INTR_UNDRFIE_POS)) /**< INT_UNDRFIE Mask */
 #define MXC_V_MSRADC_INTR_UNDRFIE_DISABLED                 ((uint32_t)0x0) /**< INT_UNDRFIE_DISABLED Value */
 #define MXC_S_MSRADC_INTR_UNDRFIE_DISABLED                 (MXC_V_MSRADC_INTR_UNDRFIE_DISABLED << MXC_F_MSRADC_INTR_UNDRFIE_POS) /**< INT_UNDRFIE_DISABLED Setting */
 #define MXC_V_MSRADC_INTR_UNDRFIE_ENABLED                  ((uint32_t)0x1) /**< INT_UNDRFIE_ENABLED Value */
 #define MXC_S_MSRADC_INTR_UNDRFIE_ENABLED                  (MXC_V_MSRADC_INTR_UNDRFIE_ENABLED << MXC_F_MSRADC_INTR_UNDRFIE_POS) /**< INT_UNDRFIE_ENABLED Setting */

 #define MXC_F_MSRADC_INTR_FIFOLVLIE_POS                    (8) /**< INT_FIFOLVLIE Position */
 #define MXC_F_MSRADC_INTR_FIFOLVLIE                        ((uint32_t)(0x1 << MXC_F_MSRADC_INTR_FIFOLVLIE_POS)) /**< INT_FIFOLVLIE Mask */
 #define MXC_V_MSRADC_INTR_FIFOLVLIE_DISABLED               ((uint32_t)0x0) /**< INT_FIFOLVLIE_DISABLED Value */
 #define MXC_S_MSRADC_INTR_FIFOLVLIE_DISABLED               (MXC_V_MSRADC_INTR_FIFOLVLIE_DISABLED << MXC_F_MSRADC_INTR_FIFOLVLIE_POS) /**< INT_FIFOLVLIE_DISABLED Setting */
 #define MXC_V_MSRADC_INTR_FIFOLVLIE_ENABLED                ((uint32_t)0x1) /**< INT_FIFOLVLIE_ENABLED Value */
 #define MXC_S_MSRADC_INTR_FIFOLVLIE_ENABLED                (MXC_V_MSRADC_INTR_FIFOLVLIE_ENABLED << MXC_F_MSRADC_INTR_FIFOLVLIE_POS) /**< INT_FIFOLVLIE_ENABLED Setting */

 #define MXC_F_MSRADC_INTR_GLOBIE_POS                       (9) /**< INT_GLOBIE Position */
 #define MXC_F_MSRADC_INTR_GLOBIE                           ((uint32_t)(0x1 << MXC_F_MSRADC_INTR_GLOBIE_POS)) /**< INT_GLOBIE Mask */
 #define MXC_V_MSRADC_INTR_GLOBIE_DISABLED                  ((uint32_t)0x0) /**< INT_GLOBIE_DISABLED Value */
 #define MXC_S_MSRADC_INTR_GLOBIE_DISABLED                  (MXC_V_MSRADC_INTR_GLOBIE_DISABLED << MXC_F_MSRADC_INTR_GLOBIE_POS) /**< INT_GLOBIE_DISABLED Setting */
 #define MXC_V_MSRADC_INTR_GLOBIE_ENABLED                   ((uint32_t)0x1) /**< INT_GLOBIE_ENABLED Value */
 #define MXC_S_MSRADC_INTR_GLOBIE_ENABLED                   (MXC_V_MSRADC_INTR_GLOBIE_ENABLED << MXC_F_MSRADC_INTR_GLOBIE_POS) /**< INT_GLOBIE_ENABLED Setting */

/**@} end of group INT_Register */

/**
 * @ingroup  adc_registers
 * @defgroup STAT_Register
 * @brief    MSRADC Status Register
 * @{
 */
 #define MXC_F_MSRADC_STAT_FIFOCNT_POS                     (0) /**< STAT_FIFOCNT Position */
 #define MXC_F_MSRADC_STAT_FIFOCNT                         ((uint32_t)(0xF << MXC_F_MSRADC_STAT_FIFOCNT_POS)) /**< STAT_FIFOCNT Mask */
 #define MXC_V_MSRADC_STAT_FIFOCNT_FIFO_EMPTY              ((uint32_t)0x0) /**< STAT_FIFOCNT_FIFO_EMPTY Value */
 #define MXC_S_MSRADC_STAT_FIFOCNT_FIFO_EMPTY              (MXC_V_MSRADC_STAT_FIFOCNT_FIFO_EMPTY << MXC_F_MSRADC_STAT_FIFOCNT_POS) /**< STAT_FIFOCNT_FIFO_EMPTY Setting */
 #define MXC_V_MSRADC_STAT_FIFOCNT_ONE_SAMPLE              ((uint32_t)0x1) /**< STAT_FIFOCNT_ONE_SAMPLE Value */
 #define MXC_S_MSRADC_STAT_FIFOCNT_ONE_SAMPLE              (MXC_V_MSRADC_STAT_FIFOCNT_ONE_SAMPLE << MXC_F_MSRADC_STAT_FIFOCNT_POS) /**< STAT_FIFOCNT_ONE_SAMPLE Setting */
 #define MXC_V_MSRADC_STAT_FIFOCNT_TWO_SAMPLE              ((uint32_t)0x2) /**< STAT_FIFOCNT_TWO_SAMPLE Value */
 #define MXC_S_MSRADC_STAT_FIFOCNT_TWO_SAMPLE              (MXC_V_MSRADC_STAT_FIFOCNT_TWO_SAMPLE << MXC_F_MSRADC_STAT_FIFOCNT_POS) /**< STAT_FIFOCNT_TWO_SAMPLE Setting */
 #define MXC_V_MSRADC_STAT_FIFOCNT_THREE_SAMPLE            ((uint32_t)0x3) /**< STAT_FIFOCNT_THREE_SAMPLE Value */
 #define MXC_S_MSRADC_STAT_FIFOCNT_THREE_SAMPLE            (MXC_V_MSRADC_STAT_FIFOCNT_THREE_SAMPLE << MXC_F_MSRADC_STAT_FIFOCNT_POS) /**< STAT_FIFOCNT_THREE_SAMPLE Setting */
 #define MXC_V_MSRADC_STAT_FIFOCNT_FOUR_SAMPLE             ((uint32_t)0x4) /**< STAT_FIFOCNT_FOUR_SAMPLE Value */
 #define MXC_S_MSRADC_STAT_FIFOCNT_FOUR_SAMPLE             (MXC_V_MSRADC_STAT_FIFOCNT_FOUR_SAMPLE << MXC_F_MSRADC_STAT_FIFOCNT_POS) /**< STAT_FIFOCNT_FOUR_SAMPLE Setting */
 #define MXC_V_MSRADC_STAT_FIFOCNT_FIVE_SAMPLE             ((uint32_t)0x5) /**< STAT_FIFOCNT_FIVE_SAMPLE Value */
 #define MXC_S_MSRADC_STAT_FIFOCNT_FIVE_SAMPLE             (MXC_V_MSRADC_STAT_FIFOCNT_FIVE_SAMPLE << MXC_F_MSRADC_STAT_FIFOCNT_POS) /**< STAT_FIFOCNT_FIVE_SAMPLE Setting */
 #define MXC_V_MSRADC_STAT_FIFOCNT_SIX_SAMPLE              ((uint32_t)0x6) /**< STAT_FIFOCNT_SIX_SAMPLE Value */
 #define MXC_S_MSRADC_STAT_FIFOCNT_SIX_SAMPLE              (MXC_V_MSRADC_STAT_FIFOCNT_SIX_SAMPLE << MXC_F_MSRADC_STAT_FIFOCNT_POS) /**< STAT_FIFOCNT_SIX_SAMPLE Setting */
 #define MXC_V_MSRADC_STAT_FIFOCNT_SEVEN_SAMPLE            ((uint32_t)0x7) /**< STAT_FIFOCNT_SEVEN_SAMPLE Value */
 #define MXC_S_MSRADC_STAT_FIFOCNT_SEVEN_SAMPLE            (MXC_V_MSRADC_STAT_FIFOCNT_SEVEN_SAMPLE << MXC_F_MSRADC_STAT_FIFOCNT_POS) /**< STAT_FIFOCNT_SEVEN_SAMPLE Setting */
 #define MXC_V_MSRADC_STAT_FIFOCNT_EIGHT_SAMPLE            ((uint32_t)0x8) /**< STAT_FIFOCNT_EIGHT_SAMPLE Value */
 #define MXC_S_MSRADC_STAT_FIFOCNT_EIGHT_SAMPLE            (MXC_V_MSRADC_STAT_FIFOCNT_EIGHT_SAMPLE << MXC_F_MSRADC_STAT_FIFOCNT_POS) /**< STAT_FIFOCNT_EIGHT_SAMPLE Setting */

 #define MXC_F_MSRADC_STAT_FULL_POS                        (4) /**< STAT_FULL Position */
 #define MXC_F_MSRADC_STAT_FULL                            ((uint32_t)(0x1 << MXC_F_MSRADC_STAT_FULL_POS)) /**< STAT_FULL Mask */
 #define MXC_V_MSRADC_STAT_FULL_FIFO_NOT_FULL              ((uint32_t)0x0) /**< STAT_FULL_FIFO_NOT_FULL Value */
 #define MXC_S_MSRADC_STAT_FULL_FIFO_NOT_FULL              (MXC_V_MSRADC_STAT_FULL_FIFO_NOT_FULL << MXC_F_MSRADC_STAT_FULL_POS) /**< STAT_FULL_FIFO_NOT_FULL Setting */
 #define MXC_V_MSRADC_STAT_FULL_FIFO_FULL                  ((uint32_t)0x1) /**< STAT_FULL_FIFO_FULL Value */
 #define MXC_S_MSRADC_STAT_FULL_FIFO_FULL                  (MXC_V_MSRADC_STAT_FULL_FIFO_FULL << MXC_F_MSRADC_STAT_FULL_POS) /**< STAT_FULL_FIFO_FULL Setting */

 #define MXC_F_MSRADC_STAT_EMPTY_POS                       (5) /**< STAT_EMPTY Position */
 #define MXC_F_MSRADC_STAT_EMPTY                           ((uint32_t)(0x1 << MXC_F_MSRADC_STAT_EMPTY_POS)) /**< STAT_EMPTY Mask */
 #define MXC_V_MSRADC_STAT_EMPTY_FIFO_NOT_EMPTY            ((uint32_t)0x0) /**< STAT_EMPTY_FIFO_NOT_EMPTY Value */
 #define MXC_S_MSRADC_STAT_EMPTY_FIFO_NOT_EMPTY            (MXC_V_MSRADC_STAT_EMPTY_FIFO_NOT_EMPTY << MXC_F_MSRADC_STAT_EMPTY_POS) /**< STAT_EMPTY_FIFO_NOT_EMPTY Setting */
 #define MXC_V_MSRADC_STAT_EMPTY_FIFO_EMPTY                ((uint32_t)0x1) /**< STAT_EMPTY_FIFO_EMPTY Value */
 #define MXC_S_MSRADC_STAT_EMPTY_FIFO_EMPTY                (MXC_V_MSRADC_STAT_EMPTY_FIFO_EMPTY << MXC_F_MSRADC_STAT_EMPTY_POS) /**< STAT_EMPTY_FIFO_EMPTY Setting */

 #define MXC_F_MSRADC_STAT_OVERFINT_POS                    (6) /**< STAT_OVERFINT Position */
 #define MXC_F_MSRADC_STAT_OVERFINT                        ((uint32_t)(0x1 << MXC_F_MSRADC_STAT_OVERFINT_POS)) /**< STAT_OVERFINT Mask */
 #define MXC_V_MSRADC_STAT_OVERFINT_NOT_FIFO_OVERFLOW      ((uint32_t)0x0) /**< STAT_OVERFINT_NOT_FIFO_OVERFLOW Value */
 #define MXC_S_MSRADC_STAT_OVERFINT_NOT_FIFO_OVERFLOW      (MXC_V_MSRADC_STAT_OVERFINT_NOT_FIFO_OVERFLOW << MXC_F_MSRADC_STAT_OVERFINT_POS) /**< STAT_OVERFINT_NOT_FIFO_OVERFLOW Setting */
 #define MXC_V_MSRADC_STAT_OVERFINT_FIFO_OVERFLOW          ((uint32_t)0x1) /**< STAT_OVERFINT_FIFO_OVERFLOW Value */
 #define MXC_S_MSRADC_STAT_OVERFINT_FIFO_OVERFLOW          (MXC_V_MSRADC_STAT_OVERFINT_FIFO_OVERFLOW << MXC_F_MSRADC_STAT_OVERFINT_POS) /**< STAT_OVERFINT_FIFO_OVERFLOW Setting */

 #define MXC_F_MSRADC_STAT_UNDRFINT_POS                    (7) /**< STAT_UNDRFINT Position */
 #define MXC_F_MSRADC_STAT_UNDRFINT                        ((uint32_t)(0x1 << MXC_F_MSRADC_STAT_UNDRFINT_POS)) /**< STAT_UNDRFINT Mask */
 #define MXC_V_MSRADC_STAT_UNDRFINT_NOT_FIFO_UNDERFLOW     ((uint32_t)0x0) /**< STAT_UNDRFINT_NOT_FIFO_UNDERFLOW Value */
 #define MXC_S_MSRADC_STAT_UNDRFINT_NOT_FIFO_UNDERFLOW     (MXC_V_MSRADC_STAT_UNDRFINT_NOT_FIFO_UNDERFLOW << MXC_F_MSRADC_STAT_UNDRFINT_POS) /**< STAT_UNDRFINT_NOT_FIFO_UNDERFLOW Setting */
 #define MXC_V_MSRADC_STAT_UNDRFINT_FIFO_UNDERFLOW         ((uint32_t)0x1) /**< STAT_UNDRFINT_FIFO_UNDERFLOW Value */
 #define MXC_S_MSRADC_STAT_UNDRFINT_FIFO_UNDERFLOW         (MXC_V_MSRADC_STAT_UNDRFINT_FIFO_UNDERFLOW << MXC_F_MSRADC_STAT_UNDRFINT_POS) /**< STAT_UNDRFINT_FIFO_UNDERFLOW Setting */

 #define MXC_F_MSRADC_STAT_FIFOLVLST_POS                   (8) /**< STAT_FIFOLVLST Position */
 #define MXC_F_MSRADC_STAT_FIFOLVLST                       ((uint32_t)(0x1 << MXC_F_MSRADC_STAT_FIFOLVLST_POS)) /**< STAT_FIFOLVLST Mask */
 #define MXC_V_MSRADC_STAT_FIFOLVLST_BELOW_LVL             ((uint32_t)0x0) /**< STAT_FIFOLVLST_BELOW_LVL Value */
 #define MXC_S_MSRADC_STAT_FIFOLVLST_BELOW_LVL             (MXC_V_MSRADC_STAT_FIFOLVLST_BELOW_LVL << MXC_F_MSRADC_STAT_FIFOLVLST_POS) /**< STAT_FIFOLVLST_BELOW_LVL Setting */
 #define MXC_V_MSRADC_STAT_FIFOLVLST_ABOVE_LVL             ((uint32_t)0x1) /**< STAT_FIFOLVLST_ABOVE_LVL Value */
 #define MXC_S_MSRADC_STAT_FIFOLVLST_ABOVE_LVL             (MXC_V_MSRADC_STAT_FIFOLVLST_ABOVE_LVL << MXC_F_MSRADC_STAT_FIFOLVLST_POS) /**< STAT_FIFOLVLST_ABOVE_LVL Setting */

 #define MXC_F_MSRADC_STAT_GLOBINT_POS                     (9) /**< STAT_GLOBINT Position */
 #define MXC_F_MSRADC_STAT_GLOBINT                         ((uint32_t)(0x1 << MXC_F_MSRADC_STAT_GLOBINT_POS)) /**< STAT_GLOBINT Mask */
 #define MXC_V_MSRADC_STAT_GLOBINT_NOT_ACTIVE              ((uint32_t)0x0) /**< STAT_GLOBINT_NOT_ACTIVE Value */
 #define MXC_S_MSRADC_STAT_GLOBINT_NOT_ACTIVE              (MXC_V_MSRADC_STAT_GLOBINT_NOT_ACTIVE << MXC_F_MSRADC_STAT_GLOBINT_POS) /**< STAT_GLOBINT_NOT_ACTIVE Setting */
 #define MXC_V_MSRADC_STAT_GLOBINT_ACTIVE                  ((uint32_t)0x1) /**< STAT_GLOBINT_ACTIVE Value */
 #define MXC_S_MSRADC_STAT_GLOBINT_ACTIVE                  (MXC_V_MSRADC_STAT_GLOBINT_ACTIVE << MXC_F_MSRADC_STAT_GLOBINT_POS) /**< STAT_GLOBINT_ACTIVE Setting */

/**@} end of group STAT_Register */

#ifdef __cplusplus
}
#endif

#endif /* _MSRADC_REGS_H_ */
  