/**
 * @file    sir_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SIR Peripheral Module.
 * @note    This file is @generated.
 * @ingroup sir_registers
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
    __I  uint32_t sistat;               /**< <tt>\b 0x000:</tt> SIR SISTAT Register */
    __I  uint32_t addr;                 /**< <tt>\b 0x004:</tt> SIR ADDR Register */
    __R  uint32_t rsv_0x8_0x1b[5];
    __IO uint32_t btleldo_bb;           /**< <tt>\b 0x01C:</tt> SIR BTLELDO_BB Register */
    __R  uint32_t rsv_0x20_0x2b[3];
    __IO uint32_t btleldo_rf;           /**< <tt>\b 0x002C:</tt> SIR BTLELDO_RF Register */
    __R  uint32_t rsv_0x30_0x103[53];
    __I  uint32_t sfstat;               /**< <tt>\b 0x104:</tt> SIR SFSTAT Register */
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
#define MXC_R_SIR_BTLELDO_BB               ((uint32_t)0x0000001CUL) /**< Offset from SIR Base Address: <tt> 0x001C</tt> */
#define MXC_R_SIR_BTLELDO_RF               ((uint32_t)0x0000002CUL) /**< Offset from SIR Base Address: <tt> 0x002C</tt> */
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
 * @brief    Read-only field set by the SIB block if a CRC error occurs during the read of
 *           the OTP memory. Contains the failing address in OTP memory (when CRCERR equals
 *           1).
 * @{
 */
#define MXC_F_SIR_ADDR_ERRADDR_POS                     0 /**< ADDR_ERRADDR Position */
#define MXC_F_SIR_ADDR_ERRADDR                         ((uint32_t)(0xFFFFFFFFUL << MXC_F_SIR_ADDR_ERRADDR_POS)) /**< ADDR_ERRADDR Mask */

/**@} end of group SIR_ADDR_Register */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_BTLELDO_BB SIR_BTLELDO_BB
 * @brief    BTLE LDO TRIM BB Register.
 * @{
 */
#define MXC_F_SIR_BTLELDO_BB_TRIM_POS                  0 /**< BTLELDO_BB_TRIM Position */
#define MXC_F_SIR_BTLELDO_BB_TRIM                      ((uint32_t)(0x1FUL << MXC_F_SIR_BTLELDO_BB_TRIM_POS)) /**< BTLELDO_BB_TRIM Mask */

/**@} end of group SIR_BTLELDO_BB_Register */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_BTLELDO_RF SIR_BTLELDO_RF
 * @brief    BTLE LDO TRIM RF Register.
 * @{
 */
#define MXC_F_SIR_BTLELDO_RF_TRIM_POS                  0 /**< BTLELDO_RF_TRIM Position */
#define MXC_F_SIR_BTLELDO_RF_TRIM                      ((uint32_t)(0x1FUL << MXC_F_SIR_BTLELDO_RF_TRIM_POS)) /**< BTLELDO_RF_TRIM Mask */

/**@} end of group SIR_BTLELDO_RF_Register */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_SFSTAT SIR_SFSTAT
 * @brief    Security function status register.
 * @{
 */
#define MXC_F_SIR_SFSTAT_SECBOOT_EN_POS                0 /**< SFSTAT_SECBOOT_EN Position */
#define MXC_F_SIR_SFSTAT_SECBOOT_EN                    ((uint32_t)(0x1UL << MXC_F_SIR_SFSTAT_SECBOOT_EN_POS)) /**< SFSTAT_SECBOOT_EN Mask */

#define MXC_F_SIR_SFSTAT_SECEXT_EN_POS                 1 /**< SFSTAT_SECEXT_EN Position */
#define MXC_F_SIR_SFSTAT_SECEXT_EN                     ((uint32_t)(0x1UL << MXC_F_SIR_SFSTAT_SECEXT_EN_POS)) /**< SFSTAT_SECEXT_EN Mask */

/**@} end of group SIR_SFSTAT_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_SIR_REGS_H_
