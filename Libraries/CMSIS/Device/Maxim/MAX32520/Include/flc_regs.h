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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32520_INCLUDE_FLC_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32520_INCLUDE_FLC_REGS_H_

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
    __IO uint32_t flsh_addr;            /**< <tt>\b 0x00:</tt> FLC FLSH_ADDR Register */
    __IO uint32_t flsh_clkdiv;          /**< <tt>\b 0x04:</tt> FLC FLSH_CLKDIV Register */
    __IO uint32_t flsh_cn;              /**< <tt>\b 0x08:</tt> FLC FLSH_CN Register */
    __R  uint32_t rsv_0xc_0x23[6];
    __IO uint32_t flsh_int;             /**< <tt>\b 0x24:</tt> FLC FLSH_INT Register */
    __R  uint32_t rsv_0x28_0x2f[2];
    __IO uint32_t flsh_data[4];         /**< <tt>\b 0x30:</tt> FLC FLSH_DATA Register */
    __O  uint32_t acntl;                /**< <tt>\b 0x40:</tt> FLC ACNTL Register */
} mxc_flc_regs_t;

/* Register offsets for module FLC */
/**
 * @ingroup    flc_registers
 * @defgroup   FLC_Register_Offsets Register Offsets
 * @brief      FLC Peripheral Register Offsets from the FLC Base Peripheral Address.
 * @{
 */
#define MXC_R_FLC_FLSH_ADDR                ((uint32_t)0x00000000UL) /**< Offset from FLC Base Address: <tt> 0x0000</tt> */
#define MXC_R_FLC_FLSH_CLKDIV              ((uint32_t)0x00000004UL) /**< Offset from FLC Base Address: <tt> 0x0004</tt> */
#define MXC_R_FLC_FLSH_CN                  ((uint32_t)0x00000008UL) /**< Offset from FLC Base Address: <tt> 0x0008</tt> */
#define MXC_R_FLC_FLSH_INT                 ((uint32_t)0x00000024UL) /**< Offset from FLC Base Address: <tt> 0x0024</tt> */
#define MXC_R_FLC_FLSH_DATA                ((uint32_t)0x00000030UL) /**< Offset from FLC Base Address: <tt> 0x0030</tt> */
#define MXC_R_FLC_ACNTL                    ((uint32_t)0x00000040UL) /**< Offset from FLC Base Address: <tt> 0x0040</tt> */
/**@} end of group flc_registers */

/**
 * @ingroup  flc_registers
 * @defgroup FLC_FLSH_ADDR FLC_FLSH_ADDR
 * @brief    Flash Write Address.
 * @{
 */
#define MXC_F_FLC_FLSH_ADDR_ADDR_POS                   0 /**< FLSH_ADDR_ADDR Position */
#define MXC_F_FLC_FLSH_ADDR_ADDR                       ((uint32_t)(0xFFFFFFFFUL << MXC_F_FLC_FLSH_ADDR_ADDR_POS)) /**< FLSH_ADDR_ADDR Mask */

/**@} end of group FLC_FLSH_ADDR_Register */

/**
 * @ingroup  flc_registers
 * @defgroup FLC_FLSH_CLKDIV FLC_FLSH_CLKDIV
 * @brief    Flash Clock Divide. The clock (PLL0) is divided by this value to generate a 1
 *           MHz clock for Flash controller.
 * @{
 */
#define MXC_F_FLC_FLSH_CLKDIV_CLKDIV_POS               0 /**< FLSH_CLKDIV_CLKDIV Position */
#define MXC_F_FLC_FLSH_CLKDIV_CLKDIV                   ((uint32_t)(0xFFUL << MXC_F_FLC_FLSH_CLKDIV_CLKDIV_POS)) /**< FLSH_CLKDIV_CLKDIV Mask */

/**@} end of group FLC_FLSH_CLKDIV_Register */

/**
 * @ingroup  flc_registers
 * @defgroup FLC_FLSH_CN FLC_FLSH_CN
 * @brief    Flash Control Register.
 * @{
 */
#define MXC_F_FLC_FLSH_CN_WR_POS                       0 /**< FLSH_CN_WR Position */
#define MXC_F_FLC_FLSH_CN_WR                           ((uint32_t)(0x1UL << MXC_F_FLC_FLSH_CN_WR_POS)) /**< FLSH_CN_WR Mask */

#define MXC_F_FLC_FLSH_CN_ME_POS                       1 /**< FLSH_CN_ME Position */
#define MXC_F_FLC_FLSH_CN_ME                           ((uint32_t)(0x1UL << MXC_F_FLC_FLSH_CN_ME_POS)) /**< FLSH_CN_ME Mask */

#define MXC_F_FLC_FLSH_CN_PGE_POS                      2 /**< FLSH_CN_PGE Position */
#define MXC_F_FLC_FLSH_CN_PGE                          ((uint32_t)(0x1UL << MXC_F_FLC_FLSH_CN_PGE_POS)) /**< FLSH_CN_PGE Mask */

#define MXC_F_FLC_FLSH_CN_ERASE_CODE_POS               8 /**< FLSH_CN_ERASE_CODE Position */
#define MXC_F_FLC_FLSH_CN_ERASE_CODE                   ((uint32_t)(0xFFUL << MXC_F_FLC_FLSH_CN_ERASE_CODE_POS)) /**< FLSH_CN_ERASE_CODE Mask */
#define MXC_V_FLC_FLSH_CN_ERASE_CODE_NOP               ((uint32_t)0x0UL) /**< FLSH_CN_ERASE_CODE_NOP Value */
#define MXC_S_FLC_FLSH_CN_ERASE_CODE_NOP               (MXC_V_FLC_FLSH_CN_ERASE_CODE_NOP << MXC_F_FLC_FLSH_CN_ERASE_CODE_POS) /**< FLSH_CN_ERASE_CODE_NOP Setting */
#define MXC_V_FLC_FLSH_CN_ERASE_CODE_ERASEPAGE         ((uint32_t)0x55UL) /**< FLSH_CN_ERASE_CODE_ERASEPAGE Value */
#define MXC_S_FLC_FLSH_CN_ERASE_CODE_ERASEPAGE         (MXC_V_FLC_FLSH_CN_ERASE_CODE_ERASEPAGE << MXC_F_FLC_FLSH_CN_ERASE_CODE_POS) /**< FLSH_CN_ERASE_CODE_ERASEPAGE Setting */
#define MXC_V_FLC_FLSH_CN_ERASE_CODE_ERASEALL          ((uint32_t)0xAAUL) /**< FLSH_CN_ERASE_CODE_ERASEALL Value */
#define MXC_S_FLC_FLSH_CN_ERASE_CODE_ERASEALL          (MXC_V_FLC_FLSH_CN_ERASE_CODE_ERASEALL << MXC_F_FLC_FLSH_CN_ERASE_CODE_POS) /**< FLSH_CN_ERASE_CODE_ERASEALL Setting */

#define MXC_F_FLC_FLSH_CN_PEND_POS                     24 /**< FLSH_CN_PEND Position */
#define MXC_F_FLC_FLSH_CN_PEND                         ((uint32_t)(0x1UL << MXC_F_FLC_FLSH_CN_PEND_POS)) /**< FLSH_CN_PEND Mask */

#define MXC_F_FLC_FLSH_CN_UNLOCK_POS                   28 /**< FLSH_CN_UNLOCK Position */
#define MXC_F_FLC_FLSH_CN_UNLOCK                       ((uint32_t)(0xFUL << MXC_F_FLC_FLSH_CN_UNLOCK_POS)) /**< FLSH_CN_UNLOCK Mask */
#define MXC_V_FLC_FLSH_CN_UNLOCK_UNLOCKED              ((uint32_t)0x2UL) /**< FLSH_CN_UNLOCK_UNLOCKED Value */
#define MXC_S_FLC_FLSH_CN_UNLOCK_UNLOCKED              (MXC_V_FLC_FLSH_CN_UNLOCK_UNLOCKED << MXC_F_FLC_FLSH_CN_UNLOCK_POS) /**< FLSH_CN_UNLOCK_UNLOCKED Setting */
#define MXC_V_FLC_FLSH_CN_UNLOCK_LOCKED                ((uint32_t)0x3UL) /**< FLSH_CN_UNLOCK_LOCKED Value */
#define MXC_S_FLC_FLSH_CN_UNLOCK_LOCKED                (MXC_V_FLC_FLSH_CN_UNLOCK_LOCKED << MXC_F_FLC_FLSH_CN_UNLOCK_POS) /**< FLSH_CN_UNLOCK_LOCKED Setting */

/**@} end of group FLC_FLSH_CN_Register */

/**
 * @ingroup  flc_registers
 * @defgroup FLC_FLSH_INT FLC_FLSH_INT
 * @brief    Flash Interrupt Register.
 * @{
 */
#define MXC_F_FLC_FLSH_INT_DONE_POS                    0 /**< FLSH_INT_DONE Position */
#define MXC_F_FLC_FLSH_INT_DONE                        ((uint32_t)(0x1UL << MXC_F_FLC_FLSH_INT_DONE_POS)) /**< FLSH_INT_DONE Mask */

#define MXC_F_FLC_FLSH_INT_AF_POS                      1 /**< FLSH_INT_AF Position */
#define MXC_F_FLC_FLSH_INT_AF                          ((uint32_t)(0x1UL << MXC_F_FLC_FLSH_INT_AF_POS)) /**< FLSH_INT_AF Mask */

#define MXC_F_FLC_FLSH_INT_DONEIE_POS                  8 /**< FLSH_INT_DONEIE Position */
#define MXC_F_FLC_FLSH_INT_DONEIE                      ((uint32_t)(0x1UL << MXC_F_FLC_FLSH_INT_DONEIE_POS)) /**< FLSH_INT_DONEIE Mask */

#define MXC_F_FLC_FLSH_INT_AFIE_POS                    9 /**< FLSH_INT_AFIE Position */
#define MXC_F_FLC_FLSH_INT_AFIE                        ((uint32_t)(0x1UL << MXC_F_FLC_FLSH_INT_AFIE_POS)) /**< FLSH_INT_AFIE Mask */

/**@} end of group FLC_FLSH_INT_Register */

/**
 * @ingroup  flc_registers
 * @defgroup FLC_FLSH_DATA FLC_FLSH_DATA
 * @brief    Flash Write Data.
 * @{
 */
#define MXC_F_FLC_FLSH_DATA_DATA_POS                   0 /**< FLSH_DATA_DATA Position */
#define MXC_F_FLC_FLSH_DATA_DATA                       ((uint32_t)(0xFFFFFFFFUL << MXC_F_FLC_FLSH_DATA_DATA_POS)) /**< FLSH_DATA_DATA Mask */

/**@} end of group FLC_FLSH_DATA_Register */

/**
 * @ingroup  flc_registers
 * @defgroup FLC_ACNTL FLC_ACNTL
 * @brief    Access Control Register. Writing the ACTRL register with the following values in
 *           the order shown, allows read and write access to the system and user Information
 *           block:                 pflc-actrl = 0x3a7f5ca3;                 pflc-actrl =
 *           0xa1e34f20;                 pflc-actrl = 0x9608b2c1. When unlocked, a write of
 *           any word will disable access to system and user information block. Readback of
 *           this register is always zero.
 * @{
 */
#define MXC_F_FLC_ACNTL_ADATA_POS                      0 /**< ACNTL_ADATA Position */
#define MXC_F_FLC_ACNTL_ADATA                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_FLC_ACNTL_ADATA_POS)) /**< ACNTL_ADATA Mask */

/**@} end of group FLC_ACNTL_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32520_INCLUDE_FLC_REGS_H_
