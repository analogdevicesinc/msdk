/**
 * @file    flc_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the FLC Peripheral Module.
 * @note    This file is @generated.
 * @ingroup flc_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78002_INCLUDE_FLC_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78002_INCLUDE_FLC_REGS_H_

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
 * @ingroup     flc
 * @defgroup    flc_registers FLC_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the FLC Peripheral Module.
 * @details     Flash Memory Control.
 */

/**
 * @ingroup flc_registers
 * Structure type to access the FLC Registers.
 */
typedef struct {
    __IO uint32_t addr;                 /**< <tt>\b 0x00:</tt> FLC ADDR Register */
    __IO uint32_t clkdiv;               /**< <tt>\b 0x04:</tt> FLC CLKDIV Register */
    __IO uint32_t ctrl;                 /**< <tt>\b 0x08:</tt> FLC CTRL Register */
    __R  uint32_t rsv_0xc_0x23[6];
    __IO uint32_t intr;                 /**< <tt>\b 0x24:</tt> FLC INTR Register */
    __R  uint32_t rsv_0x28_0x2f[2];
    __IO uint32_t data[4];              /**< <tt>\b 0x30:</tt> FLC DATA Register */
    __O  uint32_t actrl;                /**< <tt>\b 0x40:</tt> FLC ACTRL Register */
    __R  uint32_t rsv_0x44_0x7f[15];
    __IO uint32_t welr0;                /**< <tt>\b 0x80:</tt> FLC WELR0 Register */
    __IO uint32_t rlr0;                 /**< <tt>\b 0x84:</tt> FLC RLR0 Register */
    __IO uint32_t welr1;                /**< <tt>\b 0x88:</tt> FLC WELR1 Register */
    __IO uint32_t rlr1;                 /**< <tt>\b 0x8C:</tt> FLC RLR1 Register */
    __IO uint32_t welr2;                /**< <tt>\b 90:</tt> FLC WELR2 Register */
    __IO uint32_t rlr2;                 /**< <tt>\b 0x94:</tt> FLC RLR2 Register */
    __IO uint32_t welr3;                /**< <tt>\b 0x98:</tt> FLC WELR3 Register */
    __IO uint32_t rlr3;                 /**< <tt>\b 0x9C:</tt> FLC RLR3 Register */
    __IO uint32_t welr4;                /**< <tt>\b 0xA0:</tt> FLC WELR4 Register */
    __IO uint32_t rlr4;                 /**< <tt>\b 0xA4:</tt> FLC RLR4 Register */
} mxc_flc_regs_t;

/* Register offsets for module FLC */
/**
 * @ingroup    flc_registers
 * @defgroup   FLC_Register_Offsets Register Offsets
 * @brief      FLC Peripheral Register Offsets from the FLC Base Peripheral Address.
 * @{
 */
#define MXC_R_FLC_ADDR                     ((uint32_t)0x00000000UL) /**< Offset from FLC Base Address: <tt> 0x0000</tt> */
#define MXC_R_FLC_CLKDIV                   ((uint32_t)0x00000004UL) /**< Offset from FLC Base Address: <tt> 0x0004</tt> */
#define MXC_R_FLC_CTRL                     ((uint32_t)0x00000008UL) /**< Offset from FLC Base Address: <tt> 0x0008</tt> */
#define MXC_R_FLC_INTR                     ((uint32_t)0x00000024UL) /**< Offset from FLC Base Address: <tt> 0x0024</tt> */
#define MXC_R_FLC_DATA                     ((uint32_t)0x00000030UL) /**< Offset from FLC Base Address: <tt> 0x0030</tt> */
#define MXC_R_FLC_ACTRL                    ((uint32_t)0x00000040UL) /**< Offset from FLC Base Address: <tt> 0x0040</tt> */
#define MXC_R_FLC_WELR0                    ((uint32_t)0x00000080UL) /**< Offset from FLC Base Address: <tt> 0x0080</tt> */
#define MXC_R_FLC_RLR0                     ((uint32_t)0x00000084UL) /**< Offset from FLC Base Address: <tt> 0x0084</tt> */
#define MXC_R_FLC_WELR1                    ((uint32_t)0x00000088UL) /**< Offset from FLC Base Address: <tt> 0x0088</tt> */
#define MXC_R_FLC_RLR1                     ((uint32_t)0x0000008CUL) /**< Offset from FLC Base Address: <tt> 0x008C</tt> */
#define MXC_R_FLC_WELR2                    ((uint32_t)0x00000090UL) /**< Offset from FLC Base Address: <tt> 0x0090</tt> */
#define MXC_R_FLC_RLR2                     ((uint32_t)0x00000094UL) /**< Offset from FLC Base Address: <tt> 0x0094</tt> */
#define MXC_R_FLC_WELR3                    ((uint32_t)0x00000098UL) /**< Offset from FLC Base Address: <tt> 0x0098</tt> */
#define MXC_R_FLC_RLR3                     ((uint32_t)0x0000009CUL) /**< Offset from FLC Base Address: <tt> 0x009C</tt> */
#define MXC_R_FLC_WELR4                    ((uint32_t)0x000000A0UL) /**< Offset from FLC Base Address: <tt> 0x00A0</tt> */
#define MXC_R_FLC_RLR4                     ((uint32_t)0x000000A4UL) /**< Offset from FLC Base Address: <tt> 0x00A4</tt> */
/**@} end of group flc_registers */

/**
 * @ingroup  flc_registers
 * @defgroup FLC_ADDR FLC_ADDR
 * @brief    Flash Write Address.
 * @{
 */
#define MXC_F_FLC_ADDR_ADDR_POS                        0 /**< ADDR_ADDR Position */
#define MXC_F_FLC_ADDR_ADDR                            ((uint32_t)(0xFFFFFFFFUL << MXC_F_FLC_ADDR_ADDR_POS)) /**< ADDR_ADDR Mask */

/**@} end of group FLC_ADDR_Register */

/**
 * @ingroup  flc_registers
 * @defgroup FLC_CLKDIV FLC_CLKDIV
 * @brief    Flash Clock Divide. The clock (PLL0) is divided by this value to generate a 1
 *           MHz clock for Flash controller.
 * @{
 */
#define MXC_F_FLC_CLKDIV_CLKDIV_POS                    0 /**< CLKDIV_CLKDIV Position */
#define MXC_F_FLC_CLKDIV_CLKDIV                        ((uint32_t)(0xFFUL << MXC_F_FLC_CLKDIV_CLKDIV_POS)) /**< CLKDIV_CLKDIV Mask */

/**@} end of group FLC_CLKDIV_Register */

/**
 * @ingroup  flc_registers
 * @defgroup FLC_CTRL FLC_CTRL
 * @brief    Flash Control Register.
 * @{
 */
#define MXC_F_FLC_CTRL_WR_POS                          0 /**< CTRL_WR Position */
#define MXC_F_FLC_CTRL_WR                              ((uint32_t)(0x1UL << MXC_F_FLC_CTRL_WR_POS)) /**< CTRL_WR Mask */

#define MXC_F_FLC_CTRL_ME_POS                          1 /**< CTRL_ME Position */
#define MXC_F_FLC_CTRL_ME                              ((uint32_t)(0x1UL << MXC_F_FLC_CTRL_ME_POS)) /**< CTRL_ME Mask */

#define MXC_F_FLC_CTRL_PGE_POS                         2 /**< CTRL_PGE Position */
#define MXC_F_FLC_CTRL_PGE                             ((uint32_t)(0x1UL << MXC_F_FLC_CTRL_PGE_POS)) /**< CTRL_PGE Mask */

#define MXC_F_FLC_CTRL_WDTH_POS                        4 /**< CTRL_WDTH Position */
#define MXC_F_FLC_CTRL_WDTH                            ((uint32_t)(0x1UL << MXC_F_FLC_CTRL_WDTH_POS)) /**< CTRL_WDTH Mask */

#define MXC_F_FLC_CTRL_ERASE_CODE_POS                  8 /**< CTRL_ERASE_CODE Position */
#define MXC_F_FLC_CTRL_ERASE_CODE                      ((uint32_t)(0xFFUL << MXC_F_FLC_CTRL_ERASE_CODE_POS)) /**< CTRL_ERASE_CODE Mask */
#define MXC_V_FLC_CTRL_ERASE_CODE_NOP                  ((uint32_t)0x0UL) /**< CTRL_ERASE_CODE_NOP Value */
#define MXC_S_FLC_CTRL_ERASE_CODE_NOP                  (MXC_V_FLC_CTRL_ERASE_CODE_NOP << MXC_F_FLC_CTRL_ERASE_CODE_POS) /**< CTRL_ERASE_CODE_NOP Setting */
#define MXC_V_FLC_CTRL_ERASE_CODE_ERASEPAGE            ((uint32_t)0x55UL) /**< CTRL_ERASE_CODE_ERASEPAGE Value */
#define MXC_S_FLC_CTRL_ERASE_CODE_ERASEPAGE            (MXC_V_FLC_CTRL_ERASE_CODE_ERASEPAGE << MXC_F_FLC_CTRL_ERASE_CODE_POS) /**< CTRL_ERASE_CODE_ERASEPAGE Setting */
#define MXC_V_FLC_CTRL_ERASE_CODE_ERASEALL             ((uint32_t)0xAAUL) /**< CTRL_ERASE_CODE_ERASEALL Value */
#define MXC_S_FLC_CTRL_ERASE_CODE_ERASEALL             (MXC_V_FLC_CTRL_ERASE_CODE_ERASEALL << MXC_F_FLC_CTRL_ERASE_CODE_POS) /**< CTRL_ERASE_CODE_ERASEALL Setting */

#define MXC_F_FLC_CTRL_PEND_POS                        24 /**< CTRL_PEND Position */
#define MXC_F_FLC_CTRL_PEND                            ((uint32_t)(0x1UL << MXC_F_FLC_CTRL_PEND_POS)) /**< CTRL_PEND Mask */

#define MXC_F_FLC_CTRL_LVE_POS                         25 /**< CTRL_LVE Position */
#define MXC_F_FLC_CTRL_LVE                             ((uint32_t)(0x1UL << MXC_F_FLC_CTRL_LVE_POS)) /**< CTRL_LVE Mask */

#define MXC_F_FLC_CTRL_UNLOCK_POS                      28 /**< CTRL_UNLOCK Position */
#define MXC_F_FLC_CTRL_UNLOCK                          ((uint32_t)(0xFUL << MXC_F_FLC_CTRL_UNLOCK_POS)) /**< CTRL_UNLOCK Mask */
#define MXC_V_FLC_CTRL_UNLOCK_UNLOCKED                 ((uint32_t)0x2UL) /**< CTRL_UNLOCK_UNLOCKED Value */
#define MXC_S_FLC_CTRL_UNLOCK_UNLOCKED                 (MXC_V_FLC_CTRL_UNLOCK_UNLOCKED << MXC_F_FLC_CTRL_UNLOCK_POS) /**< CTRL_UNLOCK_UNLOCKED Setting */
#define MXC_V_FLC_CTRL_UNLOCK_LOCKED                   ((uint32_t)0x3UL) /**< CTRL_UNLOCK_LOCKED Value */
#define MXC_S_FLC_CTRL_UNLOCK_LOCKED                   (MXC_V_FLC_CTRL_UNLOCK_LOCKED << MXC_F_FLC_CTRL_UNLOCK_POS) /**< CTRL_UNLOCK_LOCKED Setting */

/**@} end of group FLC_CTRL_Register */

/**
 * @ingroup  flc_registers
 * @defgroup FLC_INTR FLC_INTR
 * @brief    Flash Interrupt Register.
 * @{
 */
#define MXC_F_FLC_INTR_DONE_POS                        0 /**< INTR_DONE Position */
#define MXC_F_FLC_INTR_DONE                            ((uint32_t)(0x1UL << MXC_F_FLC_INTR_DONE_POS)) /**< INTR_DONE Mask */

#define MXC_F_FLC_INTR_AF_POS                          1 /**< INTR_AF Position */
#define MXC_F_FLC_INTR_AF                              ((uint32_t)(0x1UL << MXC_F_FLC_INTR_AF_POS)) /**< INTR_AF Mask */

#define MXC_F_FLC_INTR_DONEIE_POS                      8 /**< INTR_DONEIE Position */
#define MXC_F_FLC_INTR_DONEIE                          ((uint32_t)(0x1UL << MXC_F_FLC_INTR_DONEIE_POS)) /**< INTR_DONEIE Mask */

#define MXC_F_FLC_INTR_AFIE_POS                        9 /**< INTR_AFIE Position */
#define MXC_F_FLC_INTR_AFIE                            ((uint32_t)(0x1UL << MXC_F_FLC_INTR_AFIE_POS)) /**< INTR_AFIE Mask */

/**@} end of group FLC_INTR_Register */

/**
 * @ingroup  flc_registers
 * @defgroup FLC_DATA FLC_DATA
 * @brief    Flash Write Data.
 * @{
 */
#define MXC_F_FLC_DATA_DATA_POS                        0 /**< DATA_DATA Position */
#define MXC_F_FLC_DATA_DATA                            ((uint32_t)(0xFFFFFFFFUL << MXC_F_FLC_DATA_DATA_POS)) /**< DATA_DATA Mask */

/**@} end of group FLC_DATA_Register */

/**
 * @ingroup  flc_registers
 * @defgroup FLC_ACTRL FLC_ACTRL
 * @brief    Access Control Register. Writing the ACTRL register with the following values in
 *           the order shown, allows read and write access to the system and user Information
 *           block:     pflc-actrl = 0x3a7f5ca3;     pflc-actrl = 0xa1e34f20;     pflc-actrl
 *           = 0x9608b2c1. When unlocked, a write of any word will disable access to system
 *           and user information block. Readback of this register is always zero.
 * @{
 */
#define MXC_F_FLC_ACTRL_ACTRL_POS                      0 /**< ACTRL_ACTRL Position */
#define MXC_F_FLC_ACTRL_ACTRL                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_FLC_ACTRL_ACTRL_POS)) /**< ACTRL_ACTRL Mask */

/**@} end of group FLC_ACTRL_Register */

/**
 * @ingroup  flc_registers
 * @defgroup FLC_WELR0 FLC_WELR0
 * @brief    WELR0
 * @{
 */
#define MXC_F_FLC_WELR0_WELR0_POS                      0 /**< WELR0_WELR0 Position */
#define MXC_F_FLC_WELR0_WELR0                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_FLC_WELR0_WELR0_POS)) /**< WELR0_WELR0 Mask */

/**@} end of group FLC_WELR0_Register */

/**
 * @ingroup  flc_registers
 * @defgroup FLC_RLR0 FLC_RLR0
 * @brief    RLR0
 * @{
 */
#define MXC_F_FLC_RLR0_RLR0_POS                        0 /**< RLR0_RLR0 Position */
#define MXC_F_FLC_RLR0_RLR0                            ((uint32_t)(0xFFFFFFFFUL << MXC_F_FLC_RLR0_RLR0_POS)) /**< RLR0_RLR0 Mask */

/**@} end of group FLC_RLR0_Register */

/**
 * @ingroup  flc_registers
 * @defgroup FLC_WELR1 FLC_WELR1
 * @brief    WELR1
 * @{
 */
#define MXC_F_FLC_WELR1_WELR1_POS                      0 /**< WELR1_WELR1 Position */
#define MXC_F_FLC_WELR1_WELR1                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_FLC_WELR1_WELR1_POS)) /**< WELR1_WELR1 Mask */

/**@} end of group FLC_WELR1_Register */

/**
 * @ingroup  flc_registers
 * @defgroup FLC_RLR1 FLC_RLR1
 * @brief    RLR1
 * @{
 */
#define MXC_F_FLC_RLR1_RLR1_POS                        0 /**< RLR1_RLR1 Position */
#define MXC_F_FLC_RLR1_RLR1                            ((uint32_t)(0xFFFFFFFFUL << MXC_F_FLC_RLR1_RLR1_POS)) /**< RLR1_RLR1 Mask */

/**@} end of group FLC_RLR1_Register */

/**
 * @ingroup  flc_registers
 * @defgroup FLC_WELR2 FLC_WELR2
 * @brief    WELR2
 * @{
 */
#define MXC_F_FLC_WELR2_WELR2_POS                      0 /**< WELR2_WELR2 Position */
#define MXC_F_FLC_WELR2_WELR2                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_FLC_WELR2_WELR2_POS)) /**< WELR2_WELR2 Mask */

/**@} end of group FLC_WELR2_Register */

/**
 * @ingroup  flc_registers
 * @defgroup FLC_RLR2 FLC_RLR2
 * @brief    RLR2
 * @{
 */
#define MXC_F_FLC_RLR2_RLR2_POS                        0 /**< RLR2_RLR2 Position */
#define MXC_F_FLC_RLR2_RLR2                            ((uint32_t)(0xFFFFFFFFUL << MXC_F_FLC_RLR2_RLR2_POS)) /**< RLR2_RLR2 Mask */

/**@} end of group FLC_RLR2_Register */

/**
 * @ingroup  flc_registers
 * @defgroup FLC_WELR3 FLC_WELR3
 * @brief    WELR3
 * @{
 */
#define MXC_F_FLC_WELR3_WELR3_POS                      0 /**< WELR3_WELR3 Position */
#define MXC_F_FLC_WELR3_WELR3                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_FLC_WELR3_WELR3_POS)) /**< WELR3_WELR3 Mask */

/**@} end of group FLC_WELR3_Register */

/**
 * @ingroup  flc_registers
 * @defgroup FLC_RLR3 FLC_RLR3
 * @brief    RLR3
 * @{
 */
#define MXC_F_FLC_RLR3_RLR3_POS                        0 /**< RLR3_RLR3 Position */
#define MXC_F_FLC_RLR3_RLR3                            ((uint32_t)(0xFFFFFFFFUL << MXC_F_FLC_RLR3_RLR3_POS)) /**< RLR3_RLR3 Mask */

/**@} end of group FLC_RLR3_Register */

/**
 * @ingroup  flc_registers
 * @defgroup FLC_WELR4 FLC_WELR4
 * @brief    WELR4
 * @{
 */
#define MXC_F_FLC_WELR4_WELR4_POS                      0 /**< WELR4_WELR4 Position */
#define MXC_F_FLC_WELR4_WELR4                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_FLC_WELR4_WELR4_POS)) /**< WELR4_WELR4 Mask */

/**@} end of group FLC_WELR4_Register */

/**
 * @ingroup  flc_registers
 * @defgroup FLC_RLR4 FLC_RLR4
 * @brief    RLR4
 * @{
 */
#define MXC_F_FLC_RLR4_RLR4_POS                        0 /**< RLR4_RLR4 Position */
#define MXC_F_FLC_RLR4_RLR4                            ((uint32_t)(0xFFFFFFFFUL << MXC_F_FLC_RLR4_RLR4_POS)) /**< RLR4_RLR4 Mask */

/**@} end of group FLC_RLR4_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78002_INCLUDE_FLC_REGS_H_
