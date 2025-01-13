/**
 * @file    flc_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the FLC Peripheral Module.
 * @note    This file is @generated.
 * @ingroup flc_registers
 */

/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_FLC_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_FLC_REGS_H_

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
    __IO uint32_t intr;                 /**< <tt>\b 0x024:</tt> FLC INTR Register */
    __R  uint32_t rsv_0x28;
    __IO uint32_t eccdata;              /**< <tt>\b 0x2C:</tt> FLC ECCDATA Register */
    __IO uint32_t data[4];              /**< <tt>\b 0x30:</tt> FLC DATA Register */
    __O  uint32_t actrl;                /**< <tt>\b 0x40:</tt> FLC ACTRL Register */
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
#define MXC_R_FLC_ECCDATA                  ((uint32_t)0x0000002CUL) /**< Offset from FLC Base Address: <tt> 0x002C</tt> */
#define MXC_R_FLC_DATA                     ((uint32_t)0x00000030UL) /**< Offset from FLC Base Address: <tt> 0x0030</tt> */
#define MXC_R_FLC_ACTRL                    ((uint32_t)0x00000040UL) /**< Offset from FLC Base Address: <tt> 0x0040</tt> */
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
#define MXC_F_FLC_INTR_DONE_IF_POS                     0 /**< INTR_DONE_IF Position */
#define MXC_F_FLC_INTR_DONE_IF                         ((uint32_t)(0x1UL << MXC_F_FLC_INTR_DONE_IF_POS)) /**< INTR_DONE_IF Mask */

#define MXC_F_FLC_INTR_AF_IF_POS                       1 /**< INTR_AF_IF Position */
#define MXC_F_FLC_INTR_AF_IF                           ((uint32_t)(0x1UL << MXC_F_FLC_INTR_AF_IF_POS)) /**< INTR_AF_IF Mask */

#define MXC_F_FLC_INTR_DONE_IE_POS                     8 /**< INTR_DONE_IE Position */
#define MXC_F_FLC_INTR_DONE_IE                         ((uint32_t)(0x1UL << MXC_F_FLC_INTR_DONE_IE_POS)) /**< INTR_DONE_IE Mask */

#define MXC_F_FLC_INTR_AF_IE_POS                       9 /**< INTR_AF_IE Position */
#define MXC_F_FLC_INTR_AF_IE                           ((uint32_t)(0x1UL << MXC_F_FLC_INTR_AF_IE_POS)) /**< INTR_AF_IE Mask */

/**@} end of group FLC_INTR_Register */

/**
 * @ingroup  flc_registers
 * @defgroup FLC_ECCDATA FLC_ECCDATA
 * @brief    ECC Data Register.
 * @{
 */
#define MXC_F_FLC_ECCDATA_EVEN_POS                     0 /**< ECCDATA_EVEN Position */
#define MXC_F_FLC_ECCDATA_EVEN                         ((uint32_t)(0x1FFUL << MXC_F_FLC_ECCDATA_EVEN_POS)) /**< ECCDATA_EVEN Mask */

#define MXC_F_FLC_ECCDATA_ODD_POS                      16 /**< ECCDATA_ODD Position */
#define MXC_F_FLC_ECCDATA_ODD                          ((uint32_t)(0x1FFUL << MXC_F_FLC_ECCDATA_ODD_POS)) /**< ECCDATA_ODD Mask */

/**@} end of group FLC_ECCDATA_Register */

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

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_FLC_REGS_H_
