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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_FLC_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_FLC_REGS_H_

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
    __IO uint32_t intr;                 /**< <tt>\b 0x024:</tt> FLC INTR Register */
    __R  uint32_t rsv_0x28_0x2f[2];
    __IO uint32_t data[4];              /**< <tt>\b 0x30:</tt> FLC DATA Register */
    __O  uint32_t actnl;                /**< <tt>\b 0x40:</tt> FLC ACTNL Register */
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
#define MXC_R_FLC_ACTNL                    ((uint32_t)0x00000040UL) /**< Offset from FLC Base Address: <tt> 0x0040</tt> */
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
#define MXC_F_FLC_CTRL_WRITE_POS                       0 /**< CTRL_WRITE Position */
#define MXC_F_FLC_CTRL_WRITE                           ((uint32_t)(0x1UL << MXC_F_FLC_CTRL_WRITE_POS)) /**< CTRL_WRITE Mask */
#define MXC_V_FLC_CTRL_WRITE_COMPLETE                  ((uint32_t)0x0UL) /**< CTRL_WRITE_COMPLETE Value */
#define MXC_S_FLC_CTRL_WRITE_COMPLETE                  (MXC_V_FLC_CTRL_WRITE_COMPLETE << MXC_F_FLC_CTRL_WRITE_POS) /**< CTRL_WRITE_COMPLETE Setting */
#define MXC_V_FLC_CTRL_WRITE_START_WR                  ((uint32_t)0x1UL) /**< CTRL_WRITE_START_WR Value */
#define MXC_S_FLC_CTRL_WRITE_START_WR                  (MXC_V_FLC_CTRL_WRITE_START_WR << MXC_F_FLC_CTRL_WRITE_POS) /**< CTRL_WRITE_START_WR Setting */

#define MXC_F_FLC_CTRL_MASS_ERASE_POS                  1 /**< CTRL_MASS_ERASE Position */
#define MXC_F_FLC_CTRL_MASS_ERASE                      ((uint32_t)(0x1UL << MXC_F_FLC_CTRL_MASS_ERASE_POS)) /**< CTRL_MASS_ERASE Mask */
#define MXC_V_FLC_CTRL_MASS_ERASE_COMPLETE             ((uint32_t)0x0UL) /**< CTRL_MASS_ERASE_COMPLETE Value */
#define MXC_S_FLC_CTRL_MASS_ERASE_COMPLETE             (MXC_V_FLC_CTRL_MASS_ERASE_COMPLETE << MXC_F_FLC_CTRL_MASS_ERASE_POS) /**< CTRL_MASS_ERASE_COMPLETE Setting */
#define MXC_V_FLC_CTRL_MASS_ERASE_START_ME             ((uint32_t)0x1UL) /**< CTRL_MASS_ERASE_START_ME Value */
#define MXC_S_FLC_CTRL_MASS_ERASE_START_ME             (MXC_V_FLC_CTRL_MASS_ERASE_START_ME << MXC_F_FLC_CTRL_MASS_ERASE_POS) /**< CTRL_MASS_ERASE_START_ME Setting */

#define MXC_F_FLC_CTRL_PAGE_ERASE_POS                  2 /**< CTRL_PAGE_ERASE Position */
#define MXC_F_FLC_CTRL_PAGE_ERASE                      ((uint32_t)(0x1UL << MXC_F_FLC_CTRL_PAGE_ERASE_POS)) /**< CTRL_PAGE_ERASE Mask */
#define MXC_V_FLC_CTRL_PAGE_ERASE_COMPLETE             ((uint32_t)0x0UL) /**< CTRL_PAGE_ERASE_COMPLETE Value */
#define MXC_S_FLC_CTRL_PAGE_ERASE_COMPLETE             (MXC_V_FLC_CTRL_PAGE_ERASE_COMPLETE << MXC_F_FLC_CTRL_PAGE_ERASE_POS) /**< CTRL_PAGE_ERASE_COMPLETE Setting */
#define MXC_V_FLC_CTRL_PAGE_ERASE_START_PGE            ((uint32_t)0x1UL) /**< CTRL_PAGE_ERASE_START_PGE Value */
#define MXC_S_FLC_CTRL_PAGE_ERASE_START_PGE            (MXC_V_FLC_CTRL_PAGE_ERASE_START_PGE << MXC_F_FLC_CTRL_PAGE_ERASE_POS) /**< CTRL_PAGE_ERASE_START_PGE Setting */

#define MXC_F_FLC_CTRL_WIDTH_POS                       4 /**< CTRL_WIDTH Position */
#define MXC_F_FLC_CTRL_WIDTH                           ((uint32_t)(0x1UL << MXC_F_FLC_CTRL_WIDTH_POS)) /**< CTRL_WIDTH Mask */
#define MXC_V_FLC_CTRL_WIDTH_128_BIT                   ((uint32_t)0x0UL) /**< CTRL_WIDTH_128_BIT Value */
#define MXC_S_FLC_CTRL_WIDTH_128_BIT                   (MXC_V_FLC_CTRL_WIDTH_128_BIT << MXC_F_FLC_CTRL_WIDTH_POS) /**< CTRL_WIDTH_128_BIT Setting */
#define MXC_V_FLC_CTRL_WIDTH_32_BIT                    ((uint32_t)0x1UL) /**< CTRL_WIDTH_32_BIT Value */
#define MXC_S_FLC_CTRL_WIDTH_32_BIT                    (MXC_V_FLC_CTRL_WIDTH_32_BIT << MXC_F_FLC_CTRL_WIDTH_POS) /**< CTRL_WIDTH_32_BIT Setting */

#define MXC_F_FLC_CTRL_ERASE_CODE_POS                  8 /**< CTRL_ERASE_CODE Position */
#define MXC_F_FLC_CTRL_ERASE_CODE                      ((uint32_t)(0xFFUL << MXC_F_FLC_CTRL_ERASE_CODE_POS)) /**< CTRL_ERASE_CODE Mask */
#define MXC_V_FLC_CTRL_ERASE_CODE_DIS                  ((uint32_t)0x0UL) /**< CTRL_ERASE_CODE_DIS Value */
#define MXC_S_FLC_CTRL_ERASE_CODE_DIS                  (MXC_V_FLC_CTRL_ERASE_CODE_DIS << MXC_F_FLC_CTRL_ERASE_CODE_POS) /**< CTRL_ERASE_CODE_DIS Setting */
#define MXC_V_FLC_CTRL_ERASE_CODE_PGE                  ((uint32_t)0x55UL) /**< CTRL_ERASE_CODE_PGE Value */
#define MXC_S_FLC_CTRL_ERASE_CODE_PGE                  (MXC_V_FLC_CTRL_ERASE_CODE_PGE << MXC_F_FLC_CTRL_ERASE_CODE_POS) /**< CTRL_ERASE_CODE_PGE Setting */
#define MXC_V_FLC_CTRL_ERASE_CODE_ME                   ((uint32_t)0xAAUL) /**< CTRL_ERASE_CODE_ME Value */
#define MXC_S_FLC_CTRL_ERASE_CODE_ME                   (MXC_V_FLC_CTRL_ERASE_CODE_ME << MXC_F_FLC_CTRL_ERASE_CODE_POS) /**< CTRL_ERASE_CODE_ME Setting */

#define MXC_F_FLC_CTRL_BUSY_POS                        24 /**< CTRL_BUSY Position */
#define MXC_F_FLC_CTRL_BUSY                            ((uint32_t)(0x1UL << MXC_F_FLC_CTRL_BUSY_POS)) /**< CTRL_BUSY Mask */
#define MXC_V_FLC_CTRL_BUSY_IDLE                       ((uint32_t)0x0UL) /**< CTRL_BUSY_IDLE Value */
#define MXC_S_FLC_CTRL_BUSY_IDLE                       (MXC_V_FLC_CTRL_BUSY_IDLE << MXC_F_FLC_CTRL_BUSY_POS) /**< CTRL_BUSY_IDLE Setting */
#define MXC_V_FLC_CTRL_BUSY_BUSY                       ((uint32_t)0x1UL) /**< CTRL_BUSY_BUSY Value */
#define MXC_S_FLC_CTRL_BUSY_BUSY                       (MXC_V_FLC_CTRL_BUSY_BUSY << MXC_F_FLC_CTRL_BUSY_POS) /**< CTRL_BUSY_BUSY Setting */

#define MXC_F_FLC_CTRL_UNLOCK_CODE_POS                 28 /**< CTRL_UNLOCK_CODE Position */
#define MXC_F_FLC_CTRL_UNLOCK_CODE                     ((uint32_t)(0xFUL << MXC_F_FLC_CTRL_UNLOCK_CODE_POS)) /**< CTRL_UNLOCK_CODE Mask */
#define MXC_V_FLC_CTRL_UNLOCK_CODE_UNLOCKED            ((uint32_t)0x2UL) /**< CTRL_UNLOCK_CODE_UNLOCKED Value */
#define MXC_S_FLC_CTRL_UNLOCK_CODE_UNLOCKED            (MXC_V_FLC_CTRL_UNLOCK_CODE_UNLOCKED << MXC_F_FLC_CTRL_UNLOCK_CODE_POS) /**< CTRL_UNLOCK_CODE_UNLOCKED Setting */
#define MXC_V_FLC_CTRL_UNLOCK_CODE_LOCKED              ((uint32_t)0x3UL) /**< CTRL_UNLOCK_CODE_LOCKED Value */
#define MXC_S_FLC_CTRL_UNLOCK_CODE_LOCKED              (MXC_V_FLC_CTRL_UNLOCK_CODE_LOCKED << MXC_F_FLC_CTRL_UNLOCK_CODE_POS) /**< CTRL_UNLOCK_CODE_LOCKED Setting */

/**@} end of group FLC_CTRL_Register */

/**
 * @ingroup  flc_registers
 * @defgroup FLC_INTR FLC_INTR
 * @brief    Flash Interrupt Register.
 * @{
 */
#define MXC_F_FLC_INTR_DONE_POS                        0 /**< INTR_DONE Position */
#define MXC_F_FLC_INTR_DONE                            ((uint32_t)(0x1UL << MXC_F_FLC_INTR_DONE_POS)) /**< INTR_DONE Mask */
#define MXC_V_FLC_INTR_DONE_INACTIVE                   ((uint32_t)0x0UL) /**< INTR_DONE_INACTIVE Value */
#define MXC_S_FLC_INTR_DONE_INACTIVE                   (MXC_V_FLC_INTR_DONE_INACTIVE << MXC_F_FLC_INTR_DONE_POS) /**< INTR_DONE_INACTIVE Setting */
#define MXC_V_FLC_INTR_DONE_PENDING                    ((uint32_t)0x1UL) /**< INTR_DONE_PENDING Value */
#define MXC_S_FLC_INTR_DONE_PENDING                    (MXC_V_FLC_INTR_DONE_PENDING << MXC_F_FLC_INTR_DONE_POS) /**< INTR_DONE_PENDING Setting */

#define MXC_F_FLC_INTR_ACCESS_FAIL_POS                 1 /**< INTR_ACCESS_FAIL Position */
#define MXC_F_FLC_INTR_ACCESS_FAIL                     ((uint32_t)(0x1UL << MXC_F_FLC_INTR_ACCESS_FAIL_POS)) /**< INTR_ACCESS_FAIL Mask */
#define MXC_V_FLC_INTR_ACCESS_FAIL_NOERR               ((uint32_t)0x0UL) /**< INTR_ACCESS_FAIL_NOERR Value */
#define MXC_S_FLC_INTR_ACCESS_FAIL_NOERR               (MXC_V_FLC_INTR_ACCESS_FAIL_NOERR << MXC_F_FLC_INTR_ACCESS_FAIL_POS) /**< INTR_ACCESS_FAIL_NOERR Setting */
#define MXC_V_FLC_INTR_ACCESS_FAIL_ERROR               ((uint32_t)0x1UL) /**< INTR_ACCESS_FAIL_ERROR Value */
#define MXC_S_FLC_INTR_ACCESS_FAIL_ERROR               (MXC_V_FLC_INTR_ACCESS_FAIL_ERROR << MXC_F_FLC_INTR_ACCESS_FAIL_POS) /**< INTR_ACCESS_FAIL_ERROR Setting */

#define MXC_F_FLC_INTR_DONE_IE_POS                     8 /**< INTR_DONE_IE Position */
#define MXC_F_FLC_INTR_DONE_IE                         ((uint32_t)(0x1UL << MXC_F_FLC_INTR_DONE_IE_POS)) /**< INTR_DONE_IE Mask */
#define MXC_V_FLC_INTR_DONE_IE_DIS                     ((uint32_t)0x0UL) /**< INTR_DONE_IE_DIS Value */
#define MXC_S_FLC_INTR_DONE_IE_DIS                     (MXC_V_FLC_INTR_DONE_IE_DIS << MXC_F_FLC_INTR_DONE_IE_POS) /**< INTR_DONE_IE_DIS Setting */
#define MXC_V_FLC_INTR_DONE_IE_EN                      ((uint32_t)0x1UL) /**< INTR_DONE_IE_EN Value */
#define MXC_S_FLC_INTR_DONE_IE_EN                      (MXC_V_FLC_INTR_DONE_IE_EN << MXC_F_FLC_INTR_DONE_IE_POS) /**< INTR_DONE_IE_EN Setting */

#define MXC_F_FLC_INTR_ACCESS_FAIL_IE_POS              9 /**< INTR_ACCESS_FAIL_IE Position */
#define MXC_F_FLC_INTR_ACCESS_FAIL_IE                  ((uint32_t)(0x1UL << MXC_F_FLC_INTR_ACCESS_FAIL_IE_POS)) /**< INTR_ACCESS_FAIL_IE Mask */
#define MXC_V_FLC_INTR_ACCESS_FAIL_IE_DIS              ((uint32_t)0x0UL) /**< INTR_ACCESS_FAIL_IE_DIS Value */
#define MXC_S_FLC_INTR_ACCESS_FAIL_IE_DIS              (MXC_V_FLC_INTR_ACCESS_FAIL_IE_DIS << MXC_F_FLC_INTR_ACCESS_FAIL_IE_POS) /**< INTR_ACCESS_FAIL_IE_DIS Setting */
#define MXC_V_FLC_INTR_ACCESS_FAIL_IE_EN               ((uint32_t)0x1UL) /**< INTR_ACCESS_FAIL_IE_EN Value */
#define MXC_S_FLC_INTR_ACCESS_FAIL_IE_EN               (MXC_V_FLC_INTR_ACCESS_FAIL_IE_EN << MXC_F_FLC_INTR_ACCESS_FAIL_IE_POS) /**< INTR_ACCESS_FAIL_IE_EN Setting */

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
 * @defgroup FLC_ACTNL FLC_ACTNL
 * @brief    Access Control Register. Writing the ACNTL register with the following values in
 *           the order shown, allows read and write access to the system and user Information
 *           block: pflc-acntl = 0x3a7f5ca3; pflc-acntl = 0xa1e34f20; pflc-acntl =
 *           0x9608b2c1. When unlocked, a write of any word will disable access to system and
 *           user information block. Readback of this register is always zero.
 * @{
 */
#define MXC_F_FLC_ACTNL_ACNTL_POS                      0 /**< ACTNL_ACNTL Position */
#define MXC_F_FLC_ACTNL_ACNTL                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_FLC_ACTNL_ACNTL_POS)) /**< ACTNL_ACNTL Mask */

/**@} end of group FLC_ACTNL_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_FLC_REGS_H_
