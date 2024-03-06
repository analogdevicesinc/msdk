/**
 * @file    wut_reva_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the WUT_REVA Peripheral Module.
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

#ifndef _WUT_REVA_REGS_H_
#define _WUT_REVA_REGS_H_

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
 * @ingroup     wut_reva
 * @defgroup    wut_reva_registers WUT_REVA_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the WUT_REVA Peripheral Module.
 * @details Wake Up Timer
 */

/**
 * @ingroup wut_reva_registers
 * Structure type to access the WUT_REVA Registers.
 */
typedef struct {
    __IO uint32_t cnt;                  /**< <tt>\b 0x0000:</tt> WUT_REVA CNT Register */
    __IO uint32_t cmp;                  /**< <tt>\b 0x0004:</tt> WUT_REVA CMP Register */
    __IO uint32_t pwm;                  /**< <tt>\b 0x0008:</tt> WUT_REVA PWM Register */
    __IO uint32_t intfl;                /**< <tt>\b 0x000C:</tt> WUT_REVA INTFL Register */
    __IO uint32_t ctrl;                 /**< <tt>\b 0x0010:</tt> WUT_REVA CTRL Register */
    __IO uint32_t nolcmp;               /**< <tt>\b 0x0014:</tt> WUT_REVA NOLCMP Register */
    __IO uint32_t preset;               /**< <tt>\b 0x0018:</tt> WUT_REVA PRESET Register */
    __IO uint32_t reload;               /**< <tt>\b 0x001C:</tt> WUT_REVA RELOAD Register */
    __IO uint32_t snapshot;             /**< <tt>\b 0x0020:</tt> WUT_REVA SNAPSHOT Register */
} mxc_wut_reva_regs_t;

/**
 * @ingroup  wut_reva_registers
 * @defgroup WUT_REVA_CNT WUT_REVA_CNT
 * @brief    Wakeup Timer Count Register
 * @{
 */
 #define MXC_F_WUT_REVA_CNT_COUNT_POS                        0 /**< CNT_COUNT Position */
 #define MXC_F_WUT_REVA_CNT_COUNT                            ((uint32_t)(0xFFFFFFFFUL << MXC_F_WUT_REVA_CNT_COUNT_POS)) /**< CNT_COUNT Mask */

/**@} end of group WUT_REVA_CNT_Register */

/**
 * @ingroup  wut_reva_registers
 * @defgroup WUT_REVA_CMP WUT_REVA_CMP
 * @brief    Wakeup Timer Compare Register
 * @{
 */
 #define MXC_F_WUT_REVA_CMP_COMPARE_POS                      0 /**< CMP_COMPARE Position */
 #define MXC_F_WUT_REVA_CMP_COMPARE                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_WUT_REVA_CMP_COMPARE_POS)) /**< CMP_COMPARE Mask */

/**@} end of group WUT_REVA_CMP_Register */

/**
 * @ingroup  wut_reva_registers
 * @defgroup WUT_REVA_PWM WUT_REVA_PWM
 * @brief    Wakeup Timer PWM Register
 * @{
 */
 #define MXC_F_WUT_REVA_PWM_PWM_POS                          0 /**< PWM_PWM Position */
 #define MXC_F_WUT_REVA_PWM_PWM                              ((uint32_t)(0xFFFFFFFFUL << MXC_F_WUT_REVA_PWM_PWM_POS)) /**< PWM_PWM Mask */

/**@} end of group WUT_REVA_PWM_Register */

/**
 * @ingroup  wut_reva_registers
 * @defgroup WUT_REVA_INTFL WUT_REVA_INTFL
 * @brief    Wakeup Timer Interrupt Register
 * @{
 */
 #define MXC_F_WUT_REVA_INTFL_IRQ_CLR_POS                    0 /**< INTFL_IRQ_CLR Position */
 #define MXC_F_WUT_REVA_INTFL_IRQ_CLR                        ((uint32_t)(0x1UL << MXC_F_WUT_REVA_INTFL_IRQ_CLR_POS)) /**< INTFL_IRQ_CLR Mask */

/**@} end of group WUT_REVA_INTFL_Register */

/**
 * @ingroup  wut_reva_registers
 * @defgroup WUT_REVA_CTRL WUT_REVA_CTRL
 * @brief    Wakeup Timer Control Register
 * @{
 */
 #define MXC_F_WUT_REVA_CTRL_TMODE_POS                       0 /**< CTRL_TMODE Position */
 #define MXC_F_WUT_REVA_CTRL_TMODE                           ((uint32_t)(0x7UL << MXC_F_WUT_REVA_CTRL_TMODE_POS)) /**< CTRL_TMODE Mask */
 #define MXC_V_WUT_REVA_CTRL_TMODE_ONESHOT                   ((uint32_t)0x0UL) /**< CTRL_TMODE_ONESHOT Value */
 #define MXC_S_WUT_REVA_CTRL_TMODE_ONESHOT                   (MXC_V_WUT_REVA_CTRL_TMODE_ONESHOT << MXC_F_WUT_REVA_CTRL_TMODE_POS) /**< CTRL_TMODE_ONESHOT Setting */
 #define MXC_V_WUT_REVA_CTRL_TMODE_CONTINUOUS                ((uint32_t)0x1UL) /**< CTRL_TMODE_CONTINUOUS Value */
 #define MXC_S_WUT_REVA_CTRL_TMODE_CONTINUOUS                (MXC_V_WUT_REVA_CTRL_TMODE_CONTINUOUS << MXC_F_WUT_REVA_CTRL_TMODE_POS) /**< CTRL_TMODE_CONTINUOUS Setting */

 #define MXC_F_WUT_REVA_CTRL_PRES_POS                        3 /**< CTRL_PRES Position */
 #define MXC_F_WUT_REVA_CTRL_PRES                            ((uint32_t)(0x7UL << MXC_F_WUT_REVA_CTRL_PRES_POS)) /**< CTRL_PRES Mask */
 #define MXC_V_WUT_REVA_CTRL_PRES_DIV1                       ((uint32_t)0x0UL) /**< CTRL_PRES_DIV1 Value */
 #define MXC_S_WUT_REVA_CTRL_PRES_DIV1                       (MXC_V_WUT_REVA_CTRL_PRES_DIV1 << MXC_F_WUT_REVA_CTRL_PRES_POS) /**< CTRL_PRES_DIV1 Setting */
 #define MXC_V_WUT_REVA_CTRL_PRES_DIV2                       ((uint32_t)0x1UL) /**< CTRL_PRES_DIV2 Value */
 #define MXC_S_WUT_REVA_CTRL_PRES_DIV2                       (MXC_V_WUT_REVA_CTRL_PRES_DIV2 << MXC_F_WUT_REVA_CTRL_PRES_POS) /**< CTRL_PRES_DIV2 Setting */
 #define MXC_V_WUT_REVA_CTRL_PRES_DIV4                       ((uint32_t)0x2UL) /**< CTRL_PRES_DIV4 Value */
 #define MXC_S_WUT_REVA_CTRL_PRES_DIV4                       (MXC_V_WUT_REVA_CTRL_PRES_DIV4 << MXC_F_WUT_REVA_CTRL_PRES_POS) /**< CTRL_PRES_DIV4 Setting */
 #define MXC_V_WUT_REVA_CTRL_PRES_DIV8                       ((uint32_t)0x3UL) /**< CTRL_PRES_DIV8 Value */
 #define MXC_S_WUT_REVA_CTRL_PRES_DIV8                       (MXC_V_WUT_REVA_CTRL_PRES_DIV8 << MXC_F_WUT_REVA_CTRL_PRES_POS) /**< CTRL_PRES_DIV8 Setting */
 #define MXC_V_WUT_REVA_CTRL_PRES_DIV16                      ((uint32_t)0x4UL) /**< CTRL_PRES_DIV16 Value */
 #define MXC_S_WUT_REVA_CTRL_PRES_DIV16                      (MXC_V_WUT_REVA_CTRL_PRES_DIV16 << MXC_F_WUT_REVA_CTRL_PRES_POS) /**< CTRL_PRES_DIV16 Setting */
 #define MXC_V_WUT_REVA_CTRL_PRES_DIV32                      ((uint32_t)0x5UL) /**< CTRL_PRES_DIV32 Value */
 #define MXC_S_WUT_REVA_CTRL_PRES_DIV32                      (MXC_V_WUT_REVA_CTRL_PRES_DIV32 << MXC_F_WUT_REVA_CTRL_PRES_POS) /**< CTRL_PRES_DIV32 Setting */
 #define MXC_V_WUT_REVA_CTRL_PRES_DIV64                      ((uint32_t)0x6UL) /**< CTRL_PRES_DIV64 Value */
 #define MXC_S_WUT_REVA_CTRL_PRES_DIV64                      (MXC_V_WUT_REVA_CTRL_PRES_DIV64 << MXC_F_WUT_REVA_CTRL_PRES_POS) /**< CTRL_PRES_DIV64 Setting */
 #define MXC_V_WUT_REVA_CTRL_PRES_DIV128                     ((uint32_t)0x7UL) /**< CTRL_PRES_DIV128 Value */
 #define MXC_S_WUT_REVA_CTRL_PRES_DIV128                     (MXC_V_WUT_REVA_CTRL_PRES_DIV128 << MXC_F_WUT_REVA_CTRL_PRES_POS) /**< CTRL_PRES_DIV128 Setting */
 #define MXC_V_WUT_REVA_CTRL_PRES_DIV256                     ((uint32_t)0x0UL) /**< CTRL_PRES_DIV256 Value */
 #define MXC_S_WUT_REVA_CTRL_PRES_DIV256                     (MXC_V_WUT_REVA_CTRL_PRES_DIV256 << MXC_F_WUT_REVA_CTRL_PRES_POS) /**< CTRL_PRES_DIV256 Setting */
 #define MXC_V_WUT_REVA_CTRL_PRES_DIV512                     ((uint32_t)0x2UL) /**< CTRL_PRES_DIV512 Value */
 #define MXC_S_WUT_REVA_CTRL_PRES_DIV512                     (MXC_V_WUT_REVA_CTRL_PRES_DIV512 << MXC_F_WUT_REVA_CTRL_PRES_POS) /**< CTRL_PRES_DIV512 Setting */
 #define MXC_V_WUT_REVA_CTRL_PRES_DIV1024                    ((uint32_t)0x3UL) /**< CTRL_PRES_DIV1024 Value */
 #define MXC_S_WUT_REVA_CTRL_PRES_DIV1024                    (MXC_V_WUT_REVA_CTRL_PRES_DIV1024 << MXC_F_WUT_REVA_CTRL_PRES_POS) /**< CTRL_PRES_DIV1024 Setting */
 #define MXC_V_WUT_REVA_CTRL_PRES_DIV2048                    ((uint32_t)0x4UL) /**< CTRL_PRES_DIV2048 Value */
 #define MXC_S_WUT_REVA_CTRL_PRES_DIV2048                    (MXC_V_WUT_REVA_CTRL_PRES_DIV2048 << MXC_F_WUT_REVA_CTRL_PRES_POS) /**< CTRL_PRES_DIV2048 Setting */
 #define MXC_V_WUT_REVA_CTRL_PRES_DIV4096                    ((uint32_t)0x5UL) /**< CTRL_PRES_DIV4096 Value */
 #define MXC_S_WUT_REVA_CTRL_PRES_DIV4096                    (MXC_V_WUT_REVA_CTRL_PRES_DIV4096 << MXC_F_WUT_REVA_CTRL_PRES_POS) /**< CTRL_PRES_DIV4096 Setting */

 #define MXC_F_WUT_REVA_CTRL_TPOL_POS                        6 /**< CTRL_TPOL Position */
 #define MXC_F_WUT_REVA_CTRL_TPOL                            ((uint32_t)(0x1UL << MXC_F_WUT_REVA_CTRL_TPOL_POS)) /**< CTRL_TPOL Mask */

 #define MXC_F_WUT_REVA_CTRL_TEN_POS                         7 /**< CTRL_TEN Position */
 #define MXC_F_WUT_REVA_CTRL_TEN                             ((uint32_t)(0x1UL << MXC_F_WUT_REVA_CTRL_TEN_POS)) /**< CTRL_TEN Mask */

 #define MXC_F_WUT_REVA_CTRL_PRES3_POS                       8 /**< CTRL_PRES3 Position */
 #define MXC_F_WUT_REVA_CTRL_PRES3                           ((uint32_t)(0x1UL << MXC_F_WUT_REVA_CTRL_PRES3_POS)) /**< CTRL_PRES3 Mask */

 #define MXC_F_WUT_REVA_CTRL_PWMSYNC_POS                     9 /**< CTRL_PWMSYNC Position */
 #define MXC_F_WUT_REVA_CTRL_PWMSYNC                         ((uint32_t)(0x1UL << MXC_F_WUT_REVA_CTRL_PWMSYNC_POS)) /**< CTRL_PWMSYNC Mask */

 #define MXC_F_WUT_REVA_CTRL_NOLHPOL_POS                     10 /**< CTRL_NOLHPOL Position */
 #define MXC_F_WUT_REVA_CTRL_NOLHPOL                         ((uint32_t)(0x1UL << MXC_F_WUT_REVA_CTRL_NOLHPOL_POS)) /**< CTRL_NOLHPOL Mask */

 #define MXC_F_WUT_REVA_CTRL_NOLLPOL_POS                     11 /**< CTRL_NOLLPOL Position */
 #define MXC_F_WUT_REVA_CTRL_NOLLPOL                         ((uint32_t)(0x1UL << MXC_F_WUT_REVA_CTRL_NOLLPOL_POS)) /**< CTRL_NOLLPOL Mask */

 #define MXC_F_WUT_REVA_CTRL_PWMCKBD_POS                     12 /**< CTRL_PWMCKBD Position */
 #define MXC_F_WUT_REVA_CTRL_PWMCKBD                         ((uint32_t)(0x1UL << MXC_F_WUT_REVA_CTRL_PWMCKBD_POS)) /**< CTRL_PWMCKBD Mask */

/**@} end of group WUT_REVA_CTRL_Register */

/**
 * @ingroup  wut_reva_registers
 * @defgroup WUT_REVA_NOLCMP WUT_REVA_NOLCMP
 * @brief    Non Overlaping Compare Register
 * @{
 */
 #define MXC_F_WUT_REVA_NOLCMP_NOLLCMP_POS                   0 /**< NOLCMP_NOLLCMP Position */
 #define MXC_F_WUT_REVA_NOLCMP_NOLLCMP                       ((uint32_t)(0xFFUL << MXC_F_WUT_REVA_NOLCMP_NOLLCMP_POS)) /**< NOLCMP_NOLLCMP Mask */

 #define MXC_F_WUT_REVA_NOLCMP_NOLHCMP_POS                   8 /**< NOLCMP_NOLHCMP Position */
 #define MXC_F_WUT_REVA_NOLCMP_NOLHCMP                       ((uint32_t)(0xFFUL << MXC_F_WUT_REVA_NOLCMP_NOLHCMP_POS)) /**< NOLCMP_NOLHCMP Mask */

/**@} end of group WUT_REVA_NOLCMP_Register */

#ifdef __cplusplus
}
#endif

#endif /* _WUT_REVA_REGS_H_ */
