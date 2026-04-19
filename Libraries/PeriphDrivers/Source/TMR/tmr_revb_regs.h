/**
 * @file    tmr_revb_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the TMR_REVB Peripheral Module.
 * @note    This file is @generated.
 * @ingroup tmr_revb_registers
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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_TMR_TMR_REVB_REGS_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_TMR_TMR_REVB_REGS_H_

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
 * @ingroup     tmr_revb
 * @defgroup    tmr_revb_registers TMR_REVB_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the TMR_REVB Peripheral Module.
 * @details     Low-Power Configurable Timer
 */

/**
 * @ingroup tmr_revb_registers
 * Structure type to access the TMR_REVB Registers.
 */
typedef struct {
    __IO uint32_t cnt;                  /**< <tt>\b 0x00:</tt> TMR_REVB CNT Register */
    __IO uint32_t cmp;                  /**< <tt>\b 0x04:</tt> TMR_REVB CMP Register */
    __IO uint32_t pwm;                  /**< <tt>\b 0x08:</tt> TMR_REVB PWM Register */
    __IO uint32_t intfl;                /**< <tt>\b 0x0C:</tt> TMR_REVB INTFL Register */
    __IO uint32_t ctrl0;                /**< <tt>\b 0x10:</tt> TMR_REVB CTRL0 Register */
    __IO uint32_t nolcmp;               /**< <tt>\b 0x14:</tt> TMR_REVB NOLCMP Register */
    __IO uint32_t ctrl1;                /**< <tt>\b 0x18:</tt> TMR_REVB CTRL1 Register */
    __IO uint32_t wkfl;                 /**< <tt>\b 0x1C:</tt> TMR_REVB WKFL Register */
} mxc_tmr_revb_regs_t;

/**
 * @ingroup  tmr_revb_registers
 * @defgroup TMR_REVB_CNT TMR_REVB_CNT
 * @brief    Timer Counter Register.
 * @{
 */
#define MXC_F_TMR_REVB_CNT_COUNT_POS                   0 /**< CNT_COUNT Position */
#define MXC_F_TMR_REVB_CNT_COUNT                       ((uint32_t)(0xFFFFFFFFUL << MXC_F_TMR_REVB_CNT_COUNT_POS)) /**< CNT_COUNT Mask */

/**@} end of group TMR_REVB_CNT_Register */

/**
 * @ingroup  tmr_revb_registers
 * @defgroup TMR_REVB_CMP TMR_REVB_CMP
 * @brief    Timer Compare Register.
 * @{
 */
#define MXC_F_TMR_REVB_CMP_COMPARE_POS                 0 /**< CMP_COMPARE Position */
#define MXC_F_TMR_REVB_CMP_COMPARE                     ((uint32_t)(0xFFFFFFFFUL << MXC_F_TMR_REVB_CMP_COMPARE_POS)) /**< CMP_COMPARE Mask */

/**@} end of group TMR_REVB_CMP_Register */

/**
 * @ingroup  tmr_revb_registers
 * @defgroup TMR_REVB_PWM TMR_REVB_PWM
 * @brief    Timer PWM Register.
 * @{
 */
#define MXC_F_TMR_REVB_PWM_PWM_POS                     0 /**< PWM_PWM Position */
#define MXC_F_TMR_REVB_PWM_PWM                         ((uint32_t)(0xFFFFFFFFUL << MXC_F_TMR_REVB_PWM_PWM_POS)) /**< PWM_PWM Mask */

/**@} end of group TMR_REVB_PWM_Register */

/**
 * @ingroup  tmr_revb_registers
 * @defgroup TMR_REVB_INTFL TMR_REVB_INTFL
 * @brief    Timer Interrupt Status Register.
 * @{
 */
#define MXC_F_TMR_REVB_INTFL_IRQ_A_POS                 0 /**< INTFL_IRQ_A Position */
#define MXC_F_TMR_REVB_INTFL_IRQ_A                     ((uint32_t)(0x1UL << MXC_F_TMR_REVB_INTFL_IRQ_A_POS)) /**< INTFL_IRQ_A Mask */

#define MXC_F_TMR_REVB_INTFL_WRDONE_A_POS              8 /**< INTFL_WRDONE_A Position */
#define MXC_F_TMR_REVB_INTFL_WRDONE_A                  ((uint32_t)(0x1UL << MXC_F_TMR_REVB_INTFL_WRDONE_A_POS)) /**< INTFL_WRDONE_A Mask */

#define MXC_F_TMR_REVB_INTFL_WR_DIS_A_POS              9 /**< INTFL_WR_DIS_A Position */
#define MXC_F_TMR_REVB_INTFL_WR_DIS_A                  ((uint32_t)(0x1UL << MXC_F_TMR_REVB_INTFL_WR_DIS_A_POS)) /**< INTFL_WR_DIS_A Mask */

#define MXC_F_TMR_REVB_INTFL_IRQ_B_POS                 16 /**< INTFL_IRQ_B Position */
#define MXC_F_TMR_REVB_INTFL_IRQ_B                     ((uint32_t)(0x1UL << MXC_F_TMR_REVB_INTFL_IRQ_B_POS)) /**< INTFL_IRQ_B Mask */

#define MXC_F_TMR_REVB_INTFL_WRDONE_B_POS              24 /**< INTFL_WRDONE_B Position */
#define MXC_F_TMR_REVB_INTFL_WRDONE_B                  ((uint32_t)(0x1UL << MXC_F_TMR_REVB_INTFL_WRDONE_B_POS)) /**< INTFL_WRDONE_B Mask */

#define MXC_F_TMR_REVB_INTFL_WR_DIS_B_POS              25 /**< INTFL_WR_DIS_B Position */
#define MXC_F_TMR_REVB_INTFL_WR_DIS_B                  ((uint32_t)(0x1UL << MXC_F_TMR_REVB_INTFL_WR_DIS_B_POS)) /**< INTFL_WR_DIS_B Mask */

/**@} end of group TMR_REVB_INTFL_Register */

/**
 * @ingroup  tmr_revb_registers
 * @defgroup TMR_REVB_CTRL0 TMR_REVB_CTRL0
 * @brief    Timer Control Register.
 * @{
 */
#define MXC_F_TMR_REVB_CTRL0_MODE_A_POS                0 /**< CTRL0_MODE_A Position */
#define MXC_F_TMR_REVB_CTRL0_MODE_A                    ((uint32_t)(0xFUL << MXC_F_TMR_REVB_CTRL0_MODE_A_POS)) /**< CTRL0_MODE_A Mask */
#define MXC_V_TMR_REVB_CTRL0_MODE_A_ONE_SHOT           ((uint32_t)0x0UL) /**< CTRL0_MODE_A_ONE_SHOT Value */
#define MXC_S_TMR_REVB_CTRL0_MODE_A_ONE_SHOT           (MXC_V_TMR_REVB_CTRL0_MODE_A_ONE_SHOT << MXC_F_TMR_REVB_CTRL0_MODE_A_POS) /**< CTRL0_MODE_A_ONE_SHOT Setting */
#define MXC_V_TMR_REVB_CTRL0_MODE_A_CONTINUOUS         ((uint32_t)0x1UL) /**< CTRL0_MODE_A_CONTINUOUS Value */
#define MXC_S_TMR_REVB_CTRL0_MODE_A_CONTINUOUS         (MXC_V_TMR_REVB_CTRL0_MODE_A_CONTINUOUS << MXC_F_TMR_REVB_CTRL0_MODE_A_POS) /**< CTRL0_MODE_A_CONTINUOUS Setting */
#define MXC_V_TMR_REVB_CTRL0_MODE_A_COUNTER            ((uint32_t)0x2UL) /**< CTRL0_MODE_A_COUNTER Value */
#define MXC_S_TMR_REVB_CTRL0_MODE_A_COUNTER            (MXC_V_TMR_REVB_CTRL0_MODE_A_COUNTER << MXC_F_TMR_REVB_CTRL0_MODE_A_POS) /**< CTRL0_MODE_A_COUNTER Setting */
#define MXC_V_TMR_REVB_CTRL0_MODE_A_PWM                ((uint32_t)0x3UL) /**< CTRL0_MODE_A_PWM Value */
#define MXC_S_TMR_REVB_CTRL0_MODE_A_PWM                (MXC_V_TMR_REVB_CTRL0_MODE_A_PWM << MXC_F_TMR_REVB_CTRL0_MODE_A_POS) /**< CTRL0_MODE_A_PWM Setting */
#define MXC_V_TMR_REVB_CTRL0_MODE_A_CAPTURE            ((uint32_t)0x4UL) /**< CTRL0_MODE_A_CAPTURE Value */
#define MXC_S_TMR_REVB_CTRL0_MODE_A_CAPTURE            (MXC_V_TMR_REVB_CTRL0_MODE_A_CAPTURE << MXC_F_TMR_REVB_CTRL0_MODE_A_POS) /**< CTRL0_MODE_A_CAPTURE Setting */
#define MXC_V_TMR_REVB_CTRL0_MODE_A_COMPARE            ((uint32_t)0x5UL) /**< CTRL0_MODE_A_COMPARE Value */
#define MXC_S_TMR_REVB_CTRL0_MODE_A_COMPARE            (MXC_V_TMR_REVB_CTRL0_MODE_A_COMPARE << MXC_F_TMR_REVB_CTRL0_MODE_A_POS) /**< CTRL0_MODE_A_COMPARE Setting */
#define MXC_V_TMR_REVB_CTRL0_MODE_A_GATED              ((uint32_t)0x6UL) /**< CTRL0_MODE_A_GATED Value */
#define MXC_S_TMR_REVB_CTRL0_MODE_A_GATED              (MXC_V_TMR_REVB_CTRL0_MODE_A_GATED << MXC_F_TMR_REVB_CTRL0_MODE_A_POS) /**< CTRL0_MODE_A_GATED Setting */
#define MXC_V_TMR_REVB_CTRL0_MODE_A_CAPCOMP            ((uint32_t)0x7UL) /**< CTRL0_MODE_A_CAPCOMP Value */
#define MXC_S_TMR_REVB_CTRL0_MODE_A_CAPCOMP            (MXC_V_TMR_REVB_CTRL0_MODE_A_CAPCOMP << MXC_F_TMR_REVB_CTRL0_MODE_A_POS) /**< CTRL0_MODE_A_CAPCOMP Setting */
#define MXC_V_TMR_REVB_CTRL0_MODE_A_DUAL_EDGE          ((uint32_t)0x8UL) /**< CTRL0_MODE_A_DUAL_EDGE Value */
#define MXC_S_TMR_REVB_CTRL0_MODE_A_DUAL_EDGE          (MXC_V_TMR_REVB_CTRL0_MODE_A_DUAL_EDGE << MXC_F_TMR_REVB_CTRL0_MODE_A_POS) /**< CTRL0_MODE_A_DUAL_EDGE Setting */
#define MXC_V_TMR_REVB_CTRL0_MODE_A_IGATED             ((uint32_t)0xEUL) /**< CTRL0_MODE_A_IGATED Value */
#define MXC_S_TMR_REVB_CTRL0_MODE_A_IGATED             (MXC_V_TMR_REVB_CTRL0_MODE_A_IGATED << MXC_F_TMR_REVB_CTRL0_MODE_A_POS) /**< CTRL0_MODE_A_IGATED Setting */

#define MXC_F_TMR_REVB_CTRL0_CLKDIV_A_POS              4 /**< CTRL0_CLKDIV_A Position */
#define MXC_F_TMR_REVB_CTRL0_CLKDIV_A                  ((uint32_t)(0xFUL << MXC_F_TMR_REVB_CTRL0_CLKDIV_A_POS)) /**< CTRL0_CLKDIV_A Mask */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_1         ((uint32_t)0x0UL) /**< CTRL0_CLKDIV_A_DIV_BY_1 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_1         (MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_1 << MXC_F_TMR_REVB_CTRL0_CLKDIV_A_POS) /**< CTRL0_CLKDIV_A_DIV_BY_1 Setting */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_2         ((uint32_t)0x1UL) /**< CTRL0_CLKDIV_A_DIV_BY_2 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_2         (MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_2 << MXC_F_TMR_REVB_CTRL0_CLKDIV_A_POS) /**< CTRL0_CLKDIV_A_DIV_BY_2 Setting */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_4         ((uint32_t)0x2UL) /**< CTRL0_CLKDIV_A_DIV_BY_4 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_4         (MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_4 << MXC_F_TMR_REVB_CTRL0_CLKDIV_A_POS) /**< CTRL0_CLKDIV_A_DIV_BY_4 Setting */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_8         ((uint32_t)0x3UL) /**< CTRL0_CLKDIV_A_DIV_BY_8 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_8         (MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_8 << MXC_F_TMR_REVB_CTRL0_CLKDIV_A_POS) /**< CTRL0_CLKDIV_A_DIV_BY_8 Setting */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_16        ((uint32_t)0x4UL) /**< CTRL0_CLKDIV_A_DIV_BY_16 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_16        (MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_16 << MXC_F_TMR_REVB_CTRL0_CLKDIV_A_POS) /**< CTRL0_CLKDIV_A_DIV_BY_16 Setting */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_32        ((uint32_t)0x5UL) /**< CTRL0_CLKDIV_A_DIV_BY_32 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_32        (MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_32 << MXC_F_TMR_REVB_CTRL0_CLKDIV_A_POS) /**< CTRL0_CLKDIV_A_DIV_BY_32 Setting */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_64        ((uint32_t)0x6UL) /**< CTRL0_CLKDIV_A_DIV_BY_64 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_64        (MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_64 << MXC_F_TMR_REVB_CTRL0_CLKDIV_A_POS) /**< CTRL0_CLKDIV_A_DIV_BY_64 Setting */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_128       ((uint32_t)0x7UL) /**< CTRL0_CLKDIV_A_DIV_BY_128 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_128       (MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_128 << MXC_F_TMR_REVB_CTRL0_CLKDIV_A_POS) /**< CTRL0_CLKDIV_A_DIV_BY_128 Setting */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_256       ((uint32_t)0x8UL) /**< CTRL0_CLKDIV_A_DIV_BY_256 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_256       (MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_256 << MXC_F_TMR_REVB_CTRL0_CLKDIV_A_POS) /**< CTRL0_CLKDIV_A_DIV_BY_256 Setting */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_512       ((uint32_t)0x9UL) /**< CTRL0_CLKDIV_A_DIV_BY_512 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_512       (MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_512 << MXC_F_TMR_REVB_CTRL0_CLKDIV_A_POS) /**< CTRL0_CLKDIV_A_DIV_BY_512 Setting */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_1024      ((uint32_t)0xAUL) /**< CTRL0_CLKDIV_A_DIV_BY_1024 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_1024      (MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_1024 << MXC_F_TMR_REVB_CTRL0_CLKDIV_A_POS) /**< CTRL0_CLKDIV_A_DIV_BY_1024 Setting */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_2048      ((uint32_t)0xBUL) /**< CTRL0_CLKDIV_A_DIV_BY_2048 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_2048      (MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_2048 << MXC_F_TMR_REVB_CTRL0_CLKDIV_A_POS) /**< CTRL0_CLKDIV_A_DIV_BY_2048 Setting */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_4096      ((uint32_t)0xCUL) /**< CTRL0_CLKDIV_A_DIV_BY_4096 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_4096      (MXC_V_TMR_REVB_CTRL0_CLKDIV_A_DIV_BY_4096 << MXC_F_TMR_REVB_CTRL0_CLKDIV_A_POS) /**< CTRL0_CLKDIV_A_DIV_BY_4096 Setting */

#define MXC_F_TMR_REVB_CTRL0_POL_A_POS                 8 /**< CTRL0_POL_A Position */
#define MXC_F_TMR_REVB_CTRL0_POL_A                     ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL0_POL_A_POS)) /**< CTRL0_POL_A Mask */

#define MXC_F_TMR_REVB_CTRL0_PWMSYNC_A_POS             9 /**< CTRL0_PWMSYNC_A Position */
#define MXC_F_TMR_REVB_CTRL0_PWMSYNC_A                 ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL0_PWMSYNC_A_POS)) /**< CTRL0_PWMSYNC_A Mask */

#define MXC_F_TMR_REVB_CTRL0_NOLHPOL_A_POS             10 /**< CTRL0_NOLHPOL_A Position */
#define MXC_F_TMR_REVB_CTRL0_NOLHPOL_A                 ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL0_NOLHPOL_A_POS)) /**< CTRL0_NOLHPOL_A Mask */

#define MXC_F_TMR_REVB_CTRL0_NOLLPOL_A_POS             11 /**< CTRL0_NOLLPOL_A Position */
#define MXC_F_TMR_REVB_CTRL0_NOLLPOL_A                 ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL0_NOLLPOL_A_POS)) /**< CTRL0_NOLLPOL_A Mask */

#define MXC_F_TMR_REVB_CTRL0_PWMCKBD_A_POS             12 /**< CTRL0_PWMCKBD_A Position */
#define MXC_F_TMR_REVB_CTRL0_PWMCKBD_A                 ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL0_PWMCKBD_A_POS)) /**< CTRL0_PWMCKBD_A Mask */

#define MXC_F_TMR_REVB_CTRL0_RST_A_POS                 13 /**< CTRL0_RST_A Position */
#define MXC_F_TMR_REVB_CTRL0_RST_A                     ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL0_RST_A_POS)) /**< CTRL0_RST_A Mask */

#define MXC_F_TMR_REVB_CTRL0_CLKEN_A_POS               14 /**< CTRL0_CLKEN_A Position */
#define MXC_F_TMR_REVB_CTRL0_CLKEN_A                   ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL0_CLKEN_A_POS)) /**< CTRL0_CLKEN_A Mask */

#define MXC_F_TMR_REVB_CTRL0_EN_A_POS                  15 /**< CTRL0_EN_A Position */
#define MXC_F_TMR_REVB_CTRL0_EN_A                      ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL0_EN_A_POS)) /**< CTRL0_EN_A Mask */

#define MXC_F_TMR_REVB_CTRL0_MODE_B_POS                16 /**< CTRL0_MODE_B Position */
#define MXC_F_TMR_REVB_CTRL0_MODE_B                    ((uint32_t)(0xFUL << MXC_F_TMR_REVB_CTRL0_MODE_B_POS)) /**< CTRL0_MODE_B Mask */
#define MXC_V_TMR_REVB_CTRL0_MODE_B_ONE_SHOT           ((uint32_t)0x0UL) /**< CTRL0_MODE_B_ONE_SHOT Value */
#define MXC_S_TMR_REVB_CTRL0_MODE_B_ONE_SHOT           (MXC_V_TMR_REVB_CTRL0_MODE_B_ONE_SHOT << MXC_F_TMR_REVB_CTRL0_MODE_B_POS) /**< CTRL0_MODE_B_ONE_SHOT Setting */
#define MXC_V_TMR_REVB_CTRL0_MODE_B_CONTINUOUS         ((uint32_t)0x1UL) /**< CTRL0_MODE_B_CONTINUOUS Value */
#define MXC_S_TMR_REVB_CTRL0_MODE_B_CONTINUOUS         (MXC_V_TMR_REVB_CTRL0_MODE_B_CONTINUOUS << MXC_F_TMR_REVB_CTRL0_MODE_B_POS) /**< CTRL0_MODE_B_CONTINUOUS Setting */
#define MXC_V_TMR_REVB_CTRL0_MODE_B_COUNTER            ((uint32_t)0x2UL) /**< CTRL0_MODE_B_COUNTER Value */
#define MXC_S_TMR_REVB_CTRL0_MODE_B_COUNTER            (MXC_V_TMR_REVB_CTRL0_MODE_B_COUNTER << MXC_F_TMR_REVB_CTRL0_MODE_B_POS) /**< CTRL0_MODE_B_COUNTER Setting */
#define MXC_V_TMR_REVB_CTRL0_MODE_B_PWM                ((uint32_t)0x3UL) /**< CTRL0_MODE_B_PWM Value */
#define MXC_S_TMR_REVB_CTRL0_MODE_B_PWM                (MXC_V_TMR_REVB_CTRL0_MODE_B_PWM << MXC_F_TMR_REVB_CTRL0_MODE_B_POS) /**< CTRL0_MODE_B_PWM Setting */
#define MXC_V_TMR_REVB_CTRL0_MODE_B_CAPTURE            ((uint32_t)0x4UL) /**< CTRL0_MODE_B_CAPTURE Value */
#define MXC_S_TMR_REVB_CTRL0_MODE_B_CAPTURE            (MXC_V_TMR_REVB_CTRL0_MODE_B_CAPTURE << MXC_F_TMR_REVB_CTRL0_MODE_B_POS) /**< CTRL0_MODE_B_CAPTURE Setting */
#define MXC_V_TMR_REVB_CTRL0_MODE_B_COMPARE            ((uint32_t)0x5UL) /**< CTRL0_MODE_B_COMPARE Value */
#define MXC_S_TMR_REVB_CTRL0_MODE_B_COMPARE            (MXC_V_TMR_REVB_CTRL0_MODE_B_COMPARE << MXC_F_TMR_REVB_CTRL0_MODE_B_POS) /**< CTRL0_MODE_B_COMPARE Setting */
#define MXC_V_TMR_REVB_CTRL0_MODE_B_GATED              ((uint32_t)0x6UL) /**< CTRL0_MODE_B_GATED Value */
#define MXC_S_TMR_REVB_CTRL0_MODE_B_GATED              (MXC_V_TMR_REVB_CTRL0_MODE_B_GATED << MXC_F_TMR_REVB_CTRL0_MODE_B_POS) /**< CTRL0_MODE_B_GATED Setting */
#define MXC_V_TMR_REVB_CTRL0_MODE_B_CAPCOMP            ((uint32_t)0x7UL) /**< CTRL0_MODE_B_CAPCOMP Value */
#define MXC_S_TMR_REVB_CTRL0_MODE_B_CAPCOMP            (MXC_V_TMR_REVB_CTRL0_MODE_B_CAPCOMP << MXC_F_TMR_REVB_CTRL0_MODE_B_POS) /**< CTRL0_MODE_B_CAPCOMP Setting */
#define MXC_V_TMR_REVB_CTRL0_MODE_B_DUAL_EDGE          ((uint32_t)0x8UL) /**< CTRL0_MODE_B_DUAL_EDGE Value */
#define MXC_S_TMR_REVB_CTRL0_MODE_B_DUAL_EDGE          (MXC_V_TMR_REVB_CTRL0_MODE_B_DUAL_EDGE << MXC_F_TMR_REVB_CTRL0_MODE_B_POS) /**< CTRL0_MODE_B_DUAL_EDGE Setting */
#define MXC_V_TMR_REVB_CTRL0_MODE_B_IGATED             ((uint32_t)0xEUL) /**< CTRL0_MODE_B_IGATED Value */
#define MXC_S_TMR_REVB_CTRL0_MODE_B_IGATED             (MXC_V_TMR_REVB_CTRL0_MODE_B_IGATED << MXC_F_TMR_REVB_CTRL0_MODE_B_POS) /**< CTRL0_MODE_B_IGATED Setting */

#define MXC_F_TMR_REVB_CTRL0_CLKDIV_B_POS              20 /**< CTRL0_CLKDIV_B Position */
#define MXC_F_TMR_REVB_CTRL0_CLKDIV_B                  ((uint32_t)(0xFUL << MXC_F_TMR_REVB_CTRL0_CLKDIV_B_POS)) /**< CTRL0_CLKDIV_B Mask */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_1         ((uint32_t)0x0UL) /**< CTRL0_CLKDIV_B_DIV_BY_1 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_1         (MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_1 << MXC_F_TMR_REVB_CTRL0_CLKDIV_B_POS) /**< CTRL0_CLKDIV_B_DIV_BY_1 Setting */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_2         ((uint32_t)0x1UL) /**< CTRL0_CLKDIV_B_DIV_BY_2 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_2         (MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_2 << MXC_F_TMR_REVB_CTRL0_CLKDIV_B_POS) /**< CTRL0_CLKDIV_B_DIV_BY_2 Setting */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_4         ((uint32_t)0x2UL) /**< CTRL0_CLKDIV_B_DIV_BY_4 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_4         (MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_4 << MXC_F_TMR_REVB_CTRL0_CLKDIV_B_POS) /**< CTRL0_CLKDIV_B_DIV_BY_4 Setting */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_8         ((uint32_t)0x3UL) /**< CTRL0_CLKDIV_B_DIV_BY_8 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_8         (MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_8 << MXC_F_TMR_REVB_CTRL0_CLKDIV_B_POS) /**< CTRL0_CLKDIV_B_DIV_BY_8 Setting */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_16        ((uint32_t)0x4UL) /**< CTRL0_CLKDIV_B_DIV_BY_16 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_16        (MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_16 << MXC_F_TMR_REVB_CTRL0_CLKDIV_B_POS) /**< CTRL0_CLKDIV_B_DIV_BY_16 Setting */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_32        ((uint32_t)0x5UL) /**< CTRL0_CLKDIV_B_DIV_BY_32 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_32        (MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_32 << MXC_F_TMR_REVB_CTRL0_CLKDIV_B_POS) /**< CTRL0_CLKDIV_B_DIV_BY_32 Setting */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_64        ((uint32_t)0x6UL) /**< CTRL0_CLKDIV_B_DIV_BY_64 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_64        (MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_64 << MXC_F_TMR_REVB_CTRL0_CLKDIV_B_POS) /**< CTRL0_CLKDIV_B_DIV_BY_64 Setting */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_128       ((uint32_t)0x7UL) /**< CTRL0_CLKDIV_B_DIV_BY_128 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_128       (MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_128 << MXC_F_TMR_REVB_CTRL0_CLKDIV_B_POS) /**< CTRL0_CLKDIV_B_DIV_BY_128 Setting */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_256       ((uint32_t)0x8UL) /**< CTRL0_CLKDIV_B_DIV_BY_256 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_256       (MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_256 << MXC_F_TMR_REVB_CTRL0_CLKDIV_B_POS) /**< CTRL0_CLKDIV_B_DIV_BY_256 Setting */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_512       ((uint32_t)0x9UL) /**< CTRL0_CLKDIV_B_DIV_BY_512 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_512       (MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_512 << MXC_F_TMR_REVB_CTRL0_CLKDIV_B_POS) /**< CTRL0_CLKDIV_B_DIV_BY_512 Setting */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_1024      ((uint32_t)0xAUL) /**< CTRL0_CLKDIV_B_DIV_BY_1024 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_1024      (MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_1024 << MXC_F_TMR_REVB_CTRL0_CLKDIV_B_POS) /**< CTRL0_CLKDIV_B_DIV_BY_1024 Setting */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_2048      ((uint32_t)0xBUL) /**< CTRL0_CLKDIV_B_DIV_BY_2048 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_2048      (MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_2048 << MXC_F_TMR_REVB_CTRL0_CLKDIV_B_POS) /**< CTRL0_CLKDIV_B_DIV_BY_2048 Setting */
#define MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_4096      ((uint32_t)0xCUL) /**< CTRL0_CLKDIV_B_DIV_BY_4096 Value */
#define MXC_S_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_4096      (MXC_V_TMR_REVB_CTRL0_CLKDIV_B_DIV_BY_4096 << MXC_F_TMR_REVB_CTRL0_CLKDIV_B_POS) /**< CTRL0_CLKDIV_B_DIV_BY_4096 Setting */

#define MXC_F_TMR_REVB_CTRL0_POL_B_POS                 24 /**< CTRL0_POL_B Position */
#define MXC_F_TMR_REVB_CTRL0_POL_B                     ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL0_POL_B_POS)) /**< CTRL0_POL_B Mask */

#define MXC_F_TMR_REVB_CTRL0_PWMSYNC_B_POS             25 /**< CTRL0_PWMSYNC_B Position */
#define MXC_F_TMR_REVB_CTRL0_PWMSYNC_B                 ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL0_PWMSYNC_B_POS)) /**< CTRL0_PWMSYNC_B Mask */

#define MXC_F_TMR_REVB_CTRL0_NOLHPOL_B_POS             26 /**< CTRL0_NOLHPOL_B Position */
#define MXC_F_TMR_REVB_CTRL0_NOLHPOL_B                 ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL0_NOLHPOL_B_POS)) /**< CTRL0_NOLHPOL_B Mask */

#define MXC_F_TMR_REVB_CTRL0_NOLLPOL_B_POS             27 /**< CTRL0_NOLLPOL_B Position */
#define MXC_F_TMR_REVB_CTRL0_NOLLPOL_B                 ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL0_NOLLPOL_B_POS)) /**< CTRL0_NOLLPOL_B Mask */

#define MXC_F_TMR_REVB_CTRL0_PWMCKBD_B_POS             28 /**< CTRL0_PWMCKBD_B Position */
#define MXC_F_TMR_REVB_CTRL0_PWMCKBD_B                 ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL0_PWMCKBD_B_POS)) /**< CTRL0_PWMCKBD_B Mask */

#define MXC_F_TMR_REVB_CTRL0_RST_B_POS                 29 /**< CTRL0_RST_B Position */
#define MXC_F_TMR_REVB_CTRL0_RST_B                     ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL0_RST_B_POS)) /**< CTRL0_RST_B Mask */

#define MXC_F_TMR_REVB_CTRL0_CLKEN_B_POS               30 /**< CTRL0_CLKEN_B Position */
#define MXC_F_TMR_REVB_CTRL0_CLKEN_B                   ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL0_CLKEN_B_POS)) /**< CTRL0_CLKEN_B Mask */

#define MXC_F_TMR_REVB_CTRL0_EN_B_POS                  31 /**< CTRL0_EN_B Position */
#define MXC_F_TMR_REVB_CTRL0_EN_B                      ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL0_EN_B_POS)) /**< CTRL0_EN_B Mask */

/**@} end of group TMR_REVB_CTRL0_Register */

/**
 * @ingroup  tmr_revb_registers
 * @defgroup TMR_REVB_NOLCMP TMR_REVB_NOLCMP
 * @brief    Timer Non-Overlapping Compare Register.
 * @{
 */
#define MXC_F_TMR_REVB_NOLCMP_LO_A_POS                 0 /**< NOLCMP_LO_A Position */
#define MXC_F_TMR_REVB_NOLCMP_LO_A                     ((uint32_t)(0xFFUL << MXC_F_TMR_REVB_NOLCMP_LO_A_POS)) /**< NOLCMP_LO_A Mask */

#define MXC_F_TMR_REVB_NOLCMP_HI_A_POS                 8 /**< NOLCMP_HI_A Position */
#define MXC_F_TMR_REVB_NOLCMP_HI_A                     ((uint32_t)(0xFFUL << MXC_F_TMR_REVB_NOLCMP_HI_A_POS)) /**< NOLCMP_HI_A Mask */

#define MXC_F_TMR_REVB_NOLCMP_LO_B_POS                 16 /**< NOLCMP_LO_B Position */
#define MXC_F_TMR_REVB_NOLCMP_LO_B                     ((uint32_t)(0xFFUL << MXC_F_TMR_REVB_NOLCMP_LO_B_POS)) /**< NOLCMP_LO_B Mask */

#define MXC_F_TMR_REVB_NOLCMP_HI_B_POS                 24 /**< NOLCMP_HI_B Position */
#define MXC_F_TMR_REVB_NOLCMP_HI_B                     ((uint32_t)(0xFFUL << MXC_F_TMR_REVB_NOLCMP_HI_B_POS)) /**< NOLCMP_HI_B Mask */

/**@} end of group TMR_REVB_NOLCMP_Register */

/**
 * @ingroup  tmr_revb_registers
 * @defgroup TMR_REVB_CTRL1 TMR_REVB_CTRL1
 * @brief    Timer Configuration Register.
 * @{
 */
#define MXC_F_TMR_REVB_CTRL1_CLKSEL_A_POS              0 /**< CTRL1_CLKSEL_A Position */
#define MXC_F_TMR_REVB_CTRL1_CLKSEL_A                  ((uint32_t)(0x3UL << MXC_F_TMR_REVB_CTRL1_CLKSEL_A_POS)) /**< CTRL1_CLKSEL_A Mask */

#define MXC_F_TMR_REVB_CTRL1_CLKEN_A_POS               2 /**< CTRL1_CLKEN_A Position */
#define MXC_F_TMR_REVB_CTRL1_CLKEN_A                   ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL1_CLKEN_A_POS)) /**< CTRL1_CLKEN_A Mask */

#define MXC_F_TMR_REVB_CTRL1_CLKRDY_A_POS              3 /**< CTRL1_CLKRDY_A Position */
#define MXC_F_TMR_REVB_CTRL1_CLKRDY_A                  ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL1_CLKRDY_A_POS)) /**< CTRL1_CLKRDY_A Mask */

#define MXC_F_TMR_REVB_CTRL1_EVENT_SEL_A_POS           4 /**< CTRL1_EVENT_SEL_A Position */
#define MXC_F_TMR_REVB_CTRL1_EVENT_SEL_A               ((uint32_t)(0x7UL << MXC_F_TMR_REVB_CTRL1_EVENT_SEL_A_POS)) /**< CTRL1_EVENT_SEL_A Mask */

#define MXC_F_TMR_REVB_CTRL1_NEGTRIG_A_POS             7 /**< CTRL1_NEGTRIG_A Position */
#define MXC_F_TMR_REVB_CTRL1_NEGTRIG_A                 ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL1_NEGTRIG_A_POS)) /**< CTRL1_NEGTRIG_A Mask */

#define MXC_F_TMR_REVB_CTRL1_IE_A_POS                  8 /**< CTRL1_IE_A Position */
#define MXC_F_TMR_REVB_CTRL1_IE_A                      ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL1_IE_A_POS)) /**< CTRL1_IE_A Mask */

#define MXC_F_TMR_REVB_CTRL1_CAPEVENT_SEL_A_POS        9 /**< CTRL1_CAPEVENT_SEL_A Position */
#define MXC_F_TMR_REVB_CTRL1_CAPEVENT_SEL_A            ((uint32_t)(0x3UL << MXC_F_TMR_REVB_CTRL1_CAPEVENT_SEL_A_POS)) /**< CTRL1_CAPEVENT_SEL_A Mask */

#define MXC_F_TMR_REVB_CTRL1_SW_CAPEVENT_A_POS         11 /**< CTRL1_SW_CAPEVENT_A Position */
#define MXC_F_TMR_REVB_CTRL1_SW_CAPEVENT_A             ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL1_SW_CAPEVENT_A_POS)) /**< CTRL1_SW_CAPEVENT_A Mask */

#define MXC_F_TMR_REVB_CTRL1_WE_A_POS                  12 /**< CTRL1_WE_A Position */
#define MXC_F_TMR_REVB_CTRL1_WE_A                      ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL1_WE_A_POS)) /**< CTRL1_WE_A Mask */

#define MXC_F_TMR_REVB_CTRL1_OUTEN_A_POS               13 /**< CTRL1_OUTEN_A Position */
#define MXC_F_TMR_REVB_CTRL1_OUTEN_A                   ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL1_OUTEN_A_POS)) /**< CTRL1_OUTEN_A Mask */

#define MXC_F_TMR_REVB_CTRL1_OUTBEN_A_POS              14 /**< CTRL1_OUTBEN_A Position */
#define MXC_F_TMR_REVB_CTRL1_OUTBEN_A                  ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL1_OUTBEN_A_POS)) /**< CTRL1_OUTBEN_A Mask */

#define MXC_F_TMR_REVB_CTRL1_ASYNC_POS                 15 /**< CTRL1_ASYNC Position */
#define MXC_F_TMR_REVB_CTRL1_ASYNC                     ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL1_ASYNC_POS)) /**< CTRL1_ASYNC Mask */

#define MXC_F_TMR_REVB_CTRL1_CLKSEL_B_POS              16 /**< CTRL1_CLKSEL_B Position */
#define MXC_F_TMR_REVB_CTRL1_CLKSEL_B                  ((uint32_t)(0x3UL << MXC_F_TMR_REVB_CTRL1_CLKSEL_B_POS)) /**< CTRL1_CLKSEL_B Mask */

#define MXC_F_TMR_REVB_CTRL1_CLKEN_B_POS               18 /**< CTRL1_CLKEN_B Position */
#define MXC_F_TMR_REVB_CTRL1_CLKEN_B                   ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL1_CLKEN_B_POS)) /**< CTRL1_CLKEN_B Mask */

#define MXC_F_TMR_REVB_CTRL1_CLKRDY_B_POS              19 /**< CTRL1_CLKRDY_B Position */
#define MXC_F_TMR_REVB_CTRL1_CLKRDY_B                  ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL1_CLKRDY_B_POS)) /**< CTRL1_CLKRDY_B Mask */

#define MXC_F_TMR_REVB_CTRL1_EVENT_SEL_B_POS           20 /**< CTRL1_EVENT_SEL_B Position */
#define MXC_F_TMR_REVB_CTRL1_EVENT_SEL_B               ((uint32_t)(0x7UL << MXC_F_TMR_REVB_CTRL1_EVENT_SEL_B_POS)) /**< CTRL1_EVENT_SEL_B Mask */

#define MXC_F_TMR_REVB_CTRL1_NEGTRIG_B_POS             23 /**< CTRL1_NEGTRIG_B Position */
#define MXC_F_TMR_REVB_CTRL1_NEGTRIG_B                 ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL1_NEGTRIG_B_POS)) /**< CTRL1_NEGTRIG_B Mask */

#define MXC_F_TMR_REVB_CTRL1_IE_B_POS                  24 /**< CTRL1_IE_B Position */
#define MXC_F_TMR_REVB_CTRL1_IE_B                      ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL1_IE_B_POS)) /**< CTRL1_IE_B Mask */

#define MXC_F_TMR_REVB_CTRL1_CAPEVENT_SEL_B_POS        25 /**< CTRL1_CAPEVENT_SEL_B Position */
#define MXC_F_TMR_REVB_CTRL1_CAPEVENT_SEL_B            ((uint32_t)(0x3UL << MXC_F_TMR_REVB_CTRL1_CAPEVENT_SEL_B_POS)) /**< CTRL1_CAPEVENT_SEL_B Mask */

#define MXC_F_TMR_REVB_CTRL1_SW_CAPEVENT_B_POS         27 /**< CTRL1_SW_CAPEVENT_B Position */
#define MXC_F_TMR_REVB_CTRL1_SW_CAPEVENT_B             ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL1_SW_CAPEVENT_B_POS)) /**< CTRL1_SW_CAPEVENT_B Mask */

#define MXC_F_TMR_REVB_CTRL1_WE_B_POS                  28 /**< CTRL1_WE_B Position */
#define MXC_F_TMR_REVB_CTRL1_WE_B                      ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL1_WE_B_POS)) /**< CTRL1_WE_B Mask */

#define MXC_F_TMR_REVB_CTRL1_CASCADE_POS               31 /**< CTRL1_CASCADE Position */
#define MXC_F_TMR_REVB_CTRL1_CASCADE                   ((uint32_t)(0x1UL << MXC_F_TMR_REVB_CTRL1_CASCADE_POS)) /**< CTRL1_CASCADE Mask */

/**@} end of group TMR_REVB_CTRL1_Register */

/**
 * @ingroup  tmr_revb_registers
 * @defgroup TMR_REVB_WKFL TMR_REVB_WKFL
 * @brief    Timer Wakeup Status Register.
 * @{
 */
#define MXC_F_TMR_REVB_WKFL_A_POS                      0 /**< WKFL_A Position */
#define MXC_F_TMR_REVB_WKFL_A                          ((uint32_t)(0x1UL << MXC_F_TMR_REVB_WKFL_A_POS)) /**< WKFL_A Mask */

#define MXC_F_TMR_REVB_WKFL_B_POS                      16 /**< WKFL_B Position */
#define MXC_F_TMR_REVB_WKFL_B                          ((uint32_t)(0x1UL << MXC_F_TMR_REVB_WKFL_B_POS)) /**< WKFL_B Mask */

/**@} end of group TMR_REVB_WKFL_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_TMR_TMR_REVB_REGS_H_
