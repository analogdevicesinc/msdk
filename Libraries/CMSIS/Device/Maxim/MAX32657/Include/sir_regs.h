/**
 * @file    sir_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SIR Peripheral Module.
 * @note    This file is @generated.
 * @ingroup sir_registers
 */

/******************************************************************************
 *
 * Analog Devices, Inc.),
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_SIR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_SIR_REGS_H_

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
 * @ingroup     sir
 * @defgroup    sir_registers SIR_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SIR Peripheral Module.
 * @details     System Initialization Registers.
 */

/**
 * @ingroup sir_registers
 * Structure type to access the SIR Registers.
 */
typedef struct {
    __I  uint32_t sistat;               /**< <tt>\b 0x00:</tt> SIR SISTAT Register */
    __I  uint32_t addr;                 /**< <tt>\b 0x04:</tt> SIR ADDR Register */
    __IO uint32_t sicfg0;               /**< <tt>\b 0x08:</tt> SIR SICFG0 Register */
    __IO uint32_t sicfg1;               /**< <tt>\b 0x0C:</tt> SIR SICFG1 Register */
    __IO uint32_t sir4;                 /**< <tt>\b 0x10:</tt> SIR SIR4 Register */
    __IO uint32_t sir5;                 /**< <tt>\b 0x14:</tt> SIR SIR5 Register */
    __IO uint32_t sir6;                 /**< <tt>\b 0x18:</tt> SIR SIR6 Register */
    __IO uint32_t sir7;                 /**< <tt>\b 0x1C:</tt> SIR SIR7 Register */
    __IO uint32_t sir8;                 /**< <tt>\b 0x20:</tt> SIR SIR8 Register */
    __IO uint32_t sir9;                 /**< <tt>\b 0x24:</tt> SIR SIR9 Register */
    __IO uint32_t sir10;                /**< <tt>\b 0x28:</tt> SIR SIR10 Register */
    __IO uint32_t sir11;                /**< <tt>\b 0x2C:</tt> SIR SIR11 Register */
    __IO uint32_t sir12;                /**< <tt>\b 0x30:</tt> SIR SIR12 Register */
    __IO uint32_t sir13;                /**< <tt>\b 0x34:</tt> SIR SIR13 Register */
    __IO uint32_t sir14;                /**< <tt>\b 0x38:</tt> SIR SIR14 Register */
    __IO uint32_t sir15;                /**< <tt>\b 0x3C:</tt> SIR SIR15 Register */
    __IO uint32_t sir16;                /**< <tt>\b 0x40:</tt> SIR SIR16 Register */
    __IO uint32_t sir17;                /**< <tt>\b 0x44:</tt> SIR SIR17 Register */
    __R  uint32_t rsv_0x48_0x103[47];
    __IO uint32_t sfstat;               /**< <tt>\b 0x104:</tt> SIR SFSTAT Register */
} mxc_sir_regs_t;

/* Register offsets for module SIR */
/**
 * @ingroup    sir_registers
 * @defgroup   SIR_Register_Offsets Register Offsets
 * @brief      SIR Peripheral Register Offsets from the SIR Base Peripheral Address.
 * @{
 */
#define MXC_R_SIR_SISTAT                   ((uint32_t)0x00000000UL) /**< Offset from SIR Base Address: <tt> 0x0000</tt> */
#define MXC_R_SIR_ADDR                     ((uint32_t)0x00000004UL) /**< Offset from SIR Base Address: <tt> 0x0004</tt> */
#define MXC_R_SIR_SICFG0                   ((uint32_t)0x00000008UL) /**< Offset from SIR Base Address: <tt> 0x0008</tt> */
#define MXC_R_SIR_SICFG1                   ((uint32_t)0x0000000CUL) /**< Offset from SIR Base Address: <tt> 0x000C</tt> */
#define MXC_R_SIR_SIR4                     ((uint32_t)0x00000010UL) /**< Offset from SIR Base Address: <tt> 0x0010</tt> */
#define MXC_R_SIR_SIR5                     ((uint32_t)0x00000014UL) /**< Offset from SIR Base Address: <tt> 0x0014</tt> */
#define MXC_R_SIR_SIR6                     ((uint32_t)0x00000018UL) /**< Offset from SIR Base Address: <tt> 0x0018</tt> */
#define MXC_R_SIR_SIR7                     ((uint32_t)0x0000001CUL) /**< Offset from SIR Base Address: <tt> 0x001C</tt> */
#define MXC_R_SIR_SIR8                     ((uint32_t)0x00000020UL) /**< Offset from SIR Base Address: <tt> 0x0020</tt> */
#define MXC_R_SIR_SIR9                     ((uint32_t)0x00000024UL) /**< Offset from SIR Base Address: <tt> 0x0024</tt> */
#define MXC_R_SIR_SIR10                    ((uint32_t)0x00000028UL) /**< Offset from SIR Base Address: <tt> 0x0028</tt> */
#define MXC_R_SIR_SIR11                    ((uint32_t)0x0000002CUL) /**< Offset from SIR Base Address: <tt> 0x002C</tt> */
#define MXC_R_SIR_SIR12                    ((uint32_t)0x00000030UL) /**< Offset from SIR Base Address: <tt> 0x0030</tt> */
#define MXC_R_SIR_SIR13                    ((uint32_t)0x00000034UL) /**< Offset from SIR Base Address: <tt> 0x0034</tt> */
#define MXC_R_SIR_SIR14                    ((uint32_t)0x00000038UL) /**< Offset from SIR Base Address: <tt> 0x0038</tt> */
#define MXC_R_SIR_SIR15                    ((uint32_t)0x0000003CUL) /**< Offset from SIR Base Address: <tt> 0x003C</tt> */
#define MXC_R_SIR_SIR16                    ((uint32_t)0x00000040UL) /**< Offset from SIR Base Address: <tt> 0x0040</tt> */
#define MXC_R_SIR_SIR17                    ((uint32_t)0x00000044UL) /**< Offset from SIR Base Address: <tt> 0x0044</tt> */
#define MXC_R_SIR_SFSTAT                   ((uint32_t)0x00000104UL) /**< Offset from SIR Base Address: <tt> 0x0104</tt> */
/**@} end of group sir_registers */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_SISTAT SIR_SISTAT
 * @brief    System Initialization Status Register.
 * @{
 */
#define MXC_F_SIR_SISTAT_MAGIC_POS                     0 /**< SISTAT_MAGIC Position */
#define MXC_F_SIR_SISTAT_MAGIC                         ((uint32_t)(0x1UL << MXC_F_SIR_SISTAT_MAGIC_POS)) /**< SISTAT_MAGIC Mask */

#define MXC_F_SIR_SISTAT_CRCERR_POS                    1 /**< SISTAT_CRCERR Position */
#define MXC_F_SIR_SISTAT_CRCERR                        ((uint32_t)(0x1UL << MXC_F_SIR_SISTAT_CRCERR_POS)) /**< SISTAT_CRCERR Mask */

/**@} end of group SIR_SISTAT_Register */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_ADDR SIR_ADDR
 * @brief    System Initialization Address Error Register.
 * @{
 */
#define MXC_F_SIR_ADDR_ERRADDR_POS                     0 /**< ADDR_ERRADDR Position */
#define MXC_F_SIR_ADDR_ERRADDR                         ((uint32_t)(0xFFFFFFFFUL << MXC_F_SIR_ADDR_ERRADDR_POS)) /**< ADDR_ERRADDR Mask */

/**@} end of group SIR_ADDR_Register */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_SICFG1 SIR_SICFG1
 * @brief    System Initialization Configuration Register 1.
 * @{
 */
#define MXC_F_SIR_SICFG1_SRAMCLKGDIS_POS               0 /**< SICFG1_SRAMCLKGDIS Position */
#define MXC_F_SIR_SICFG1_SRAMCLKGDIS                   ((uint32_t)(0x1UL << MXC_F_SIR_SICFG1_SRAMCLKGDIS_POS)) /**< SICFG1_SRAMCLKGDIS Mask */

#define MXC_F_SIR_SICFG1_ICCLKGDIS_POS                 1 /**< SICFG1_ICCLKGDIS Position */
#define MXC_F_SIR_SICFG1_ICCLKGDIS                     ((uint32_t)(0x1UL << MXC_F_SIR_SICFG1_ICCLKGDIS_POS)) /**< SICFG1_ICCLKGDIS Mask */

#define MXC_F_SIR_SICFG1_CRBDIS_POS                    6 /**< SICFG1_CRBDIS Position */
#define MXC_F_SIR_SICFG1_CRBDIS                        ((uint32_t)(0x1UL << MXC_F_SIR_SICFG1_CRBDIS_POS)) /**< SICFG1_CRBDIS Mask */

#define MXC_F_SIR_SICFG1_FLCREDDIS_POS                 7 /**< SICFG1_FLCREDDIS Position */
#define MXC_F_SIR_SICFG1_FLCREDDIS                     ((uint32_t)(0x1UL << MXC_F_SIR_SICFG1_FLCREDDIS_POS)) /**< SICFG1_FLCREDDIS Mask */

/**@} end of group SIR_SICFG1_Register */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_SIR6 SIR_SIR6
 * @brief    System Initialization Register 6.
 * @{
 */
#define MXC_F_SIR_SIR6_SECBOOTL_POS                    0 /**< SIR6_SECBOOTL Position */
#define MXC_F_SIR_SIR6_SECBOOTL                        ((uint32_t)(0xFUL << MXC_F_SIR_SIR6_SECBOOTL_POS)) /**< SIR6_SECBOOTL Mask */

#define MXC_F_SIR_SIR6_SECEXTL_POS                     4 /**< SIR6_SECEXTL Position */
#define MXC_F_SIR_SIR6_SECEXTL                         ((uint32_t)(0xFUL << MXC_F_SIR_SIR6_SECEXTL_POS)) /**< SIR6_SECEXTL Mask */

#define MXC_F_SIR_SIR6_SECBOOTH_POS                    16 /**< SIR6_SECBOOTH Position */
#define MXC_F_SIR_SIR6_SECBOOTH                        ((uint32_t)(0xFUL << MXC_F_SIR_SIR6_SECBOOTH_POS)) /**< SIR6_SECBOOTH Mask */

#define MXC_F_SIR_SIR6_SECEXTH_POS                     20 /**< SIR6_SECEXTH Position */
#define MXC_F_SIR_SIR6_SECEXTH                         ((uint32_t)(0xFUL << MXC_F_SIR_SIR6_SECEXTH_POS)) /**< SIR6_SECEXTH Mask */

/**@} end of group SIR_SIR6_Register */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_SIR8 SIR_SIR8
 * @brief    System Initialization Register 8.
 * @{
 */
#define MXC_F_SIR_SIR8_RNGEN_POS                       0 /**< SIR8_RNGEN Position */
#define MXC_F_SIR_SIR8_RNGEN                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_SIR_SIR8_RNGEN_POS)) /**< SIR8_RNGEN Mask */

/**@} end of group SIR_SIR8_Register */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_SIR9 SIR_SIR9
 * @brief    System Initialization Register 9.
 * @{
 */
#define MXC_F_SIR_SIR9_XOBIAS_POS                      0 /**< SIR9_XOBIAS Position */
#define MXC_F_SIR_SIR9_XOBIAS                          ((uint32_t)(0x7UL << MXC_F_SIR_SIR9_XOBIAS_POS)) /**< SIR9_XOBIAS Mask */
#define MXC_V_SIR_SIR9_XOBIAS_24MHZ                    ((uint32_t)0x1UL) /**< SIR9_XOBIAS_24MHZ Value */
#define MXC_S_SIR_SIR9_XOBIAS_24MHZ                    (MXC_V_SIR_SIR9_XOBIAS_24MHZ << MXC_F_SIR_SIR9_XOBIAS_POS) /**< SIR9_XOBIAS_24MHZ Setting */
#define MXC_V_SIR_SIR9_XOBIAS_32MHZ                    ((uint32_t)0x3UL) /**< SIR9_XOBIAS_32MHZ Value */
#define MXC_S_SIR_SIR9_XOBIAS_32MHZ                    (MXC_V_SIR_SIR9_XOBIAS_32MHZ << MXC_F_SIR_SIR9_XOBIAS_POS) /**< SIR9_XOBIAS_32MHZ Setting */

/**@} end of group SIR_SIR9_Register */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_SIR10 SIR_SIR10
 * @brief    System Initialization Register 10.
 * @{
 */
#define MXC_F_SIR_SIR10_RNGEN_POS                      0 /**< SIR10_RNGEN Position */
#define MXC_F_SIR_SIR10_RNGEN                          ((uint32_t)(0xFFFFFFFFUL << MXC_F_SIR_SIR10_RNGEN_POS)) /**< SIR10_RNGEN Mask */

/**@} end of group SIR_SIR10_Register */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_SIR11 SIR_SIR11
 * @brief    System Initialization Register 11.
 * @{
 */
#define MXC_F_SIR_SIR11_ENTROPYDIS_POS                 0 /**< SIR11_ENTROPYDIS Position */
#define MXC_F_SIR_SIR11_ENTROPYDIS                     ((uint32_t)(0x1UL << MXC_F_SIR_SIR11_ENTROPYDIS_POS)) /**< SIR11_ENTROPYDIS Mask */

/**@} end of group SIR_SIR11_Register */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_SIR12 SIR_SIR12
 * @brief    System Initialization Register 12.
 * @{
 */
#define MXC_F_SIR_SIR12_RNGEN2_POS                     0 /**< SIR12_RNGEN2 Position */
#define MXC_F_SIR_SIR12_RNGEN2                         ((uint32_t)(0xFFFFFFFFUL << MXC_F_SIR_SIR12_RNGEN2_POS)) /**< SIR12_RNGEN2 Mask */

/**@} end of group SIR_SIR12_Register */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_SIR14 SIR_SIR14
 * @brief    System Initialization Register 14.
 * @{
 */
#define MXC_F_SIR_SIR14_RNGEN2_POS                     0 /**< SIR14_RNGEN2 Position */
#define MXC_F_SIR_SIR14_RNGEN2                         ((uint32_t)(0xFFFFFFFFUL << MXC_F_SIR_SIR14_RNGEN2_POS)) /**< SIR14_RNGEN2 Mask */

/**@} end of group SIR_SIR14_Register */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_SIR15 SIR_SIR15
 * @brief    System Initialization Register 15.
 * @{
 */
#define MXC_F_SIR_SIR15_I2CIRRX_POS                    0 /**< SIR15_I2CIRRX Position */
#define MXC_F_SIR_SIR15_I2CIRRX                        ((uint32_t)(0x3FUL << MXC_F_SIR_SIR15_I2CIRRX_POS)) /**< SIR15_I2CIRRX Mask */

/**@} end of group SIR_SIR15_Register */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_SIR16 SIR_SIR16
 * @brief    System Initialization Register 16.
 * @{
 */
#define MXC_F_SIR_SIR16_ACMIN_POS                      0 /**< SIR16_ACMIN Position */
#define MXC_F_SIR_SIR16_ACMIN                          ((uint32_t)(0x1FFUL << MXC_F_SIR_SIR16_ACMIN_POS)) /**< SIR16_ACMIN Mask */

#define MXC_F_SIR_SIR16_ACMAX_POS                      16 /**< SIR16_ACMAX Position */
#define MXC_F_SIR_SIR16_ACMAX                          ((uint32_t)(0x1FFUL << MXC_F_SIR_SIR16_ACMAX_POS)) /**< SIR16_ACMAX Mask */

/**@} end of group SIR_SIR16_Register */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_SFSTAT SIR_SFSTAT
 * @brief    System Initialization Security Function Status Register.
 * @{
 */
#define MXC_F_SIR_SFSTAT_SECBOOT_POS                   0 /**< SFSTAT_SECBOOT Position */
#define MXC_F_SIR_SFSTAT_SECBOOT                       ((uint32_t)(0x1UL << MXC_F_SIR_SFSTAT_SECBOOT_POS)) /**< SFSTAT_SECBOOT Mask */

#define MXC_F_SIR_SFSTAT_SECEXT_POS                    1 /**< SFSTAT_SECEXT Position */
#define MXC_F_SIR_SFSTAT_SECEXT                        ((uint32_t)(0x1UL << MXC_F_SIR_SFSTAT_SECEXT_POS)) /**< SFSTAT_SECEXT Mask */

/**@} end of group SIR_SFSTAT_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_SIR_REGS_H_
