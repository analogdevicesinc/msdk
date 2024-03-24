/**
 * @file    tmr_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the TMR Peripheral Module.
 * @note    This file is @generated.
 * @ingroup tmr_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_TMR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_TMR_REGS_H_

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
 * @ingroup     tmr
 * @defgroup    tmr_registers TMR_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the TMR Peripheral Module.
 * @details     32-bit reloadable timer that can be used for timing and event counting.
 */

/**
 * @ingroup tmr_registers
 * Structure type to access the TMR Registers.
 */
typedef struct {
    __IO uint32_t cnt;                  /**< <tt>\b 0x00:</tt> TMR CNT Register */
    __IO uint32_t cmp;                  /**< <tt>\b 0x04:</tt> TMR CMP Register */
    __IO uint32_t pwm;                  /**< <tt>\b 0x08:</tt> TMR PWM Register */
    __IO uint32_t intfl;                /**< <tt>\b 0x0C:</tt> TMR INTFL Register */
    __IO uint32_t ctrl;                 /**< <tt>\b 0x10:</tt> TMR CTRL Register */
    __IO uint32_t nolcmp;               /**< <tt>\b 0x14:</tt> TMR NOLCMP Register */
} mxc_tmr_regs_t;

/* Register offsets for module TMR */
/**
 * @ingroup    tmr_registers
 * @defgroup   TMR_Register_Offsets Register Offsets
 * @brief      TMR Peripheral Register Offsets from the TMR Base Peripheral Address.
 * @{
 */
#define MXC_R_TMR_CNT                      ((uint32_t)0x00000000UL) /**< Offset from TMR Base Address: <tt> 0x0000</tt> */
#define MXC_R_TMR_CMP                      ((uint32_t)0x00000004UL) /**< Offset from TMR Base Address: <tt> 0x0004</tt> */
#define MXC_R_TMR_PWM                      ((uint32_t)0x00000008UL) /**< Offset from TMR Base Address: <tt> 0x0008</tt> */
#define MXC_R_TMR_INTFL                    ((uint32_t)0x0000000CUL) /**< Offset from TMR Base Address: <tt> 0x000C</tt> */
#define MXC_R_TMR_CTRL                     ((uint32_t)0x00000010UL) /**< Offset from TMR Base Address: <tt> 0x0010</tt> */
#define MXC_R_TMR_NOLCMP                   ((uint32_t)0x00000014UL) /**< Offset from TMR Base Address: <tt> 0x0014</tt> */
/**@} end of group tmr_registers */

/**
 * @ingroup  tmr_registers
 * @defgroup TMR_CNT TMR_CNT
 * @brief    Count.  This register stores the current timer count.
 * @{
 */
#define MXC_F_TMR_CNT_COUNT_POS                        0 /**< CNT_COUNT Position */
#define MXC_F_TMR_CNT_COUNT                            ((uint32_t)(0xFFFFFFFFUL << MXC_F_TMR_CNT_COUNT_POS)) /**< CNT_COUNT Mask */

/**@} end of group TMR_CNT_Register */

/**
 * @ingroup  tmr_registers
 * @defgroup TMR_CMP TMR_CMP
 * @brief    Compare.  This register stores the compare value, which is used to set the
 *           maximum count value to initiate a reload of the timer to 0x0001.
 * @{
 */
#define MXC_F_TMR_CMP_COMPARE_POS                      0 /**< CMP_COMPARE Position */
#define MXC_F_TMR_CMP_COMPARE                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_TMR_CMP_COMPARE_POS)) /**< CMP_COMPARE Mask */

/**@} end of group TMR_CMP_Register */

/**
 * @ingroup  tmr_registers
 * @defgroup TMR_PWM TMR_PWM
 * @brief    PWM.  This register stores the value that is compared to the current timer
 *           count.
 * @{
 */
#define MXC_F_TMR_PWM_PWM_POS                          0 /**< PWM_PWM Position */
#define MXC_F_TMR_PWM_PWM                              ((uint32_t)(0xFFFFFFFFUL << MXC_F_TMR_PWM_PWM_POS)) /**< PWM_PWM Mask */

/**@} end of group TMR_PWM_Register */

/**
 * @ingroup  tmr_registers
 * @defgroup TMR_INTFL TMR_INTFL
 * @brief    Clear Interrupt. Writing a value (0 or 1) to a bit in this register clears the
 *           associated interrupt.
 * @{
 */
#define MXC_F_TMR_INTFL_IRQ_POS                        0 /**< INTFL_IRQ Position */
#define MXC_F_TMR_INTFL_IRQ                            ((uint32_t)(0x1UL << MXC_F_TMR_INTFL_IRQ_POS)) /**< INTFL_IRQ Mask */

/**@} end of group TMR_INTFL_Register */

/**
 * @ingroup  tmr_registers
 * @defgroup TMR_CTRL TMR_CTRL
 * @brief    Timer Control Register.
 * @{
 */
#define MXC_F_TMR_CTRL_MODE_POS                        0 /**< CTRL_MODE Position */
#define MXC_F_TMR_CTRL_MODE                            ((uint32_t)(0x7UL << MXC_F_TMR_CTRL_MODE_POS)) /**< CTRL_MODE Mask */
#define MXC_V_TMR_CTRL_MODE_ONESHOT                    ((uint32_t)0x0UL) /**< CTRL_MODE_ONESHOT Value */
#define MXC_S_TMR_CTRL_MODE_ONESHOT                    (MXC_V_TMR_CTRL_MODE_ONESHOT << MXC_F_TMR_CTRL_MODE_POS) /**< CTRL_MODE_ONESHOT Setting */
#define MXC_V_TMR_CTRL_MODE_CONTINUOUS                 ((uint32_t)0x1UL) /**< CTRL_MODE_CONTINUOUS Value */
#define MXC_S_TMR_CTRL_MODE_CONTINUOUS                 (MXC_V_TMR_CTRL_MODE_CONTINUOUS << MXC_F_TMR_CTRL_MODE_POS) /**< CTRL_MODE_CONTINUOUS Setting */
#define MXC_V_TMR_CTRL_MODE_COUNTER                    ((uint32_t)0x2UL) /**< CTRL_MODE_COUNTER Value */
#define MXC_S_TMR_CTRL_MODE_COUNTER                    (MXC_V_TMR_CTRL_MODE_COUNTER << MXC_F_TMR_CTRL_MODE_POS) /**< CTRL_MODE_COUNTER Setting */
#define MXC_V_TMR_CTRL_MODE_PWM                        ((uint32_t)0x3UL) /**< CTRL_MODE_PWM Value */
#define MXC_S_TMR_CTRL_MODE_PWM                        (MXC_V_TMR_CTRL_MODE_PWM << MXC_F_TMR_CTRL_MODE_POS) /**< CTRL_MODE_PWM Setting */
#define MXC_V_TMR_CTRL_MODE_CAPTURE                    ((uint32_t)0x4UL) /**< CTRL_MODE_CAPTURE Value */
#define MXC_S_TMR_CTRL_MODE_CAPTURE                    (MXC_V_TMR_CTRL_MODE_CAPTURE << MXC_F_TMR_CTRL_MODE_POS) /**< CTRL_MODE_CAPTURE Setting */
#define MXC_V_TMR_CTRL_MODE_COMPARE                    ((uint32_t)0x5UL) /**< CTRL_MODE_COMPARE Value */
#define MXC_S_TMR_CTRL_MODE_COMPARE                    (MXC_V_TMR_CTRL_MODE_COMPARE << MXC_F_TMR_CTRL_MODE_POS) /**< CTRL_MODE_COMPARE Setting */
#define MXC_V_TMR_CTRL_MODE_GATED                      ((uint32_t)0x6UL) /**< CTRL_MODE_GATED Value */
#define MXC_S_TMR_CTRL_MODE_GATED                      (MXC_V_TMR_CTRL_MODE_GATED << MXC_F_TMR_CTRL_MODE_POS) /**< CTRL_MODE_GATED Setting */
#define MXC_V_TMR_CTRL_MODE_CAPTURECOMPARE             ((uint32_t)0x7UL) /**< CTRL_MODE_CAPTURECOMPARE Value */
#define MXC_S_TMR_CTRL_MODE_CAPTURECOMPARE             (MXC_V_TMR_CTRL_MODE_CAPTURECOMPARE << MXC_F_TMR_CTRL_MODE_POS) /**< CTRL_MODE_CAPTURECOMPARE Setting */

#define MXC_F_TMR_CTRL_CLKDIV_POS                      3 /**< CTRL_CLKDIV Position */
#define MXC_F_TMR_CTRL_CLKDIV                          ((uint32_t)(0x7UL << MXC_F_TMR_CTRL_CLKDIV_POS)) /**< CTRL_CLKDIV Mask */
#define MXC_V_TMR_CTRL_CLKDIV_DIV1                     ((uint32_t)0x0UL) /**< CTRL_CLKDIV_DIV1 Value */
#define MXC_S_TMR_CTRL_CLKDIV_DIV1                     (MXC_V_TMR_CTRL_CLKDIV_DIV1 << MXC_F_TMR_CTRL_CLKDIV_POS) /**< CTRL_CLKDIV_DIV1 Setting */
#define MXC_V_TMR_CTRL_CLKDIV_DIV2                     ((uint32_t)0x1UL) /**< CTRL_CLKDIV_DIV2 Value */
#define MXC_S_TMR_CTRL_CLKDIV_DIV2                     (MXC_V_TMR_CTRL_CLKDIV_DIV2 << MXC_F_TMR_CTRL_CLKDIV_POS) /**< CTRL_CLKDIV_DIV2 Setting */
#define MXC_V_TMR_CTRL_CLKDIV_DIV4                     ((uint32_t)0x2UL) /**< CTRL_CLKDIV_DIV4 Value */
#define MXC_S_TMR_CTRL_CLKDIV_DIV4                     (MXC_V_TMR_CTRL_CLKDIV_DIV4 << MXC_F_TMR_CTRL_CLKDIV_POS) /**< CTRL_CLKDIV_DIV4 Setting */
#define MXC_V_TMR_CTRL_CLKDIV_DIV8                     ((uint32_t)0x3UL) /**< CTRL_CLKDIV_DIV8 Value */
#define MXC_S_TMR_CTRL_CLKDIV_DIV8                     (MXC_V_TMR_CTRL_CLKDIV_DIV8 << MXC_F_TMR_CTRL_CLKDIV_POS) /**< CTRL_CLKDIV_DIV8 Setting */
#define MXC_V_TMR_CTRL_CLKDIV_DIV16                    ((uint32_t)0x4UL) /**< CTRL_CLKDIV_DIV16 Value */
#define MXC_S_TMR_CTRL_CLKDIV_DIV16                    (MXC_V_TMR_CTRL_CLKDIV_DIV16 << MXC_F_TMR_CTRL_CLKDIV_POS) /**< CTRL_CLKDIV_DIV16 Setting */
#define MXC_V_TMR_CTRL_CLKDIV_DIV32                    ((uint32_t)0x5UL) /**< CTRL_CLKDIV_DIV32 Value */
#define MXC_S_TMR_CTRL_CLKDIV_DIV32                    (MXC_V_TMR_CTRL_CLKDIV_DIV32 << MXC_F_TMR_CTRL_CLKDIV_POS) /**< CTRL_CLKDIV_DIV32 Setting */
#define MXC_V_TMR_CTRL_CLKDIV_DIV64                    ((uint32_t)0x6UL) /**< CTRL_CLKDIV_DIV64 Value */
#define MXC_S_TMR_CTRL_CLKDIV_DIV64                    (MXC_V_TMR_CTRL_CLKDIV_DIV64 << MXC_F_TMR_CTRL_CLKDIV_POS) /**< CTRL_CLKDIV_DIV64 Setting */
#define MXC_V_TMR_CTRL_CLKDIV_DIV128                   ((uint32_t)0x7UL) /**< CTRL_CLKDIV_DIV128 Value */
#define MXC_S_TMR_CTRL_CLKDIV_DIV128                   (MXC_V_TMR_CTRL_CLKDIV_DIV128 << MXC_F_TMR_CTRL_CLKDIV_POS) /**< CTRL_CLKDIV_DIV128 Setting */

#define MXC_F_TMR_CTRL_POL_POS                         6 /**< CTRL_POL Position */
#define MXC_F_TMR_CTRL_POL                             ((uint32_t)(0x1UL << MXC_F_TMR_CTRL_POL_POS)) /**< CTRL_POL Mask */

#define MXC_F_TMR_CTRL_EN_POS                          7 /**< CTRL_EN Position */
#define MXC_F_TMR_CTRL_EN                              ((uint32_t)(0x1UL << MXC_F_TMR_CTRL_EN_POS)) /**< CTRL_EN Mask */

#define MXC_F_TMR_CTRL_CLKDIV3_POS                     8 /**< CTRL_CLKDIV3 Position */
#define MXC_F_TMR_CTRL_CLKDIV3                         ((uint32_t)(0x1UL << MXC_F_TMR_CTRL_CLKDIV3_POS)) /**< CTRL_CLKDIV3 Mask */

#define MXC_F_TMR_CTRL_PWMSYNC_POS                     9 /**< CTRL_PWMSYNC Position */
#define MXC_F_TMR_CTRL_PWMSYNC                         ((uint32_t)(0x1UL << MXC_F_TMR_CTRL_PWMSYNC_POS)) /**< CTRL_PWMSYNC Mask */

#define MXC_F_TMR_CTRL_NOLHPOL_POS                     10 /**< CTRL_NOLHPOL Position */
#define MXC_F_TMR_CTRL_NOLHPOL                         ((uint32_t)(0x1UL << MXC_F_TMR_CTRL_NOLHPOL_POS)) /**< CTRL_NOLHPOL Mask */

#define MXC_F_TMR_CTRL_NOLLPOL_POS                     11 /**< CTRL_NOLLPOL Position */
#define MXC_F_TMR_CTRL_NOLLPOL                         ((uint32_t)(0x1UL << MXC_F_TMR_CTRL_NOLLPOL_POS)) /**< CTRL_NOLLPOL Mask */

#define MXC_F_TMR_CTRL_PWMCKBD_POS                     12 /**< CTRL_PWMCKBD Position */
#define MXC_F_TMR_CTRL_PWMCKBD                         ((uint32_t)(0x1UL << MXC_F_TMR_CTRL_PWMCKBD_POS)) /**< CTRL_PWMCKBD Mask */

/**@} end of group TMR_CTRL_Register */

/**
 * @ingroup  tmr_registers
 * @defgroup TMR_NOLCMP TMR_NOLCMP
 * @brief    Timer Non-Overlapping Compare Register.
 * @{
 */
#define MXC_F_TMR_NOLCMP_LO_POS                        0 /**< NOLCMP_LO Position */
#define MXC_F_TMR_NOLCMP_LO                            ((uint32_t)(0xFFUL << MXC_F_TMR_NOLCMP_LO_POS)) /**< NOLCMP_LO Mask */

#define MXC_F_TMR_NOLCMP_HI_POS                        8 /**< NOLCMP_HI Position */
#define MXC_F_TMR_NOLCMP_HI                            ((uint32_t)(0xFFUL << MXC_F_TMR_NOLCMP_HI_POS)) /**< NOLCMP_HI Mask */

/**@} end of group TMR_NOLCMP_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32572_INCLUDE_TMR_REGS_H_
