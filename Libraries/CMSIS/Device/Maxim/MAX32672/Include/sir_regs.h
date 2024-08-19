/**
 * @file    sir_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SIR Peripheral Module.
 * @note    This file is @generated.
 * @ingroup sir_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32672_INCLUDE_SIR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32672_INCLUDE_SIR_REGS_H_

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
    __I  uint32_t status;               /**< <tt>\b 0x00:</tt> SIR STATUS Register */
    __I  uint32_t addr;                 /**< <tt>\b 0x04:</tt> SIR ADDR Register */
    __R  uint32_t rsv_0x8_0xff[62];
    __I  uint32_t fstat;                /**< <tt>\b 0x100:</tt> SIR FSTAT Register */
    __I  uint32_t sfstat;               /**< <tt>\b 0x104:</tt> SIR SFSTAT Register */
} mxc_sir_regs_t;

/* Register offsets for module SIR */
/**
 * @ingroup    sir_registers
 * @defgroup   SIR_Register_Offsets Register Offsets
 * @brief      SIR Peripheral Register Offsets from the SIR Base Peripheral Address.
 * @{
 */
#define MXC_R_SIR_STATUS                   ((uint32_t)0x00000000UL) /**< Offset from SIR Base Address: <tt> 0x0000</tt> */
#define MXC_R_SIR_ADDR                     ((uint32_t)0x00000004UL) /**< Offset from SIR Base Address: <tt> 0x0004</tt> */
#define MXC_R_SIR_FSTAT                    ((uint32_t)0x00000100UL) /**< Offset from SIR Base Address: <tt> 0x0100</tt> */
#define MXC_R_SIR_SFSTAT                   ((uint32_t)0x00000104UL) /**< Offset from SIR Base Address: <tt> 0x0104</tt> */
/**@} end of group sir_registers */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_STATUS SIR_STATUS
 * @brief    System Initialization Status Register.
 * @{
 */
#define MXC_F_SIR_STATUS_CFG_VALID_POS                 0 /**< STATUS_CFG_VALID Position */
#define MXC_F_SIR_STATUS_CFG_VALID                     ((uint32_t)(0x1UL << MXC_F_SIR_STATUS_CFG_VALID_POS)) /**< STATUS_CFG_VALID Mask */

#define MXC_F_SIR_STATUS_CFG_ERR_POS                   1 /**< STATUS_CFG_ERR Position */
#define MXC_F_SIR_STATUS_CFG_ERR                       ((uint32_t)(0x1UL << MXC_F_SIR_STATUS_CFG_ERR_POS)) /**< STATUS_CFG_ERR Mask */

#define MXC_F_SIR_STATUS_USER_CFG_ERR_POS              2 /**< STATUS_USER_CFG_ERR Position */
#define MXC_F_SIR_STATUS_USER_CFG_ERR                  ((uint32_t)(0x1UL << MXC_F_SIR_STATUS_USER_CFG_ERR_POS)) /**< STATUS_USER_CFG_ERR Mask */

/**@} end of group SIR_STATUS_Register */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_ADDR SIR_ADDR
 * @brief    Read-only field set by the SIB block if a CRC error occurs during the read of
 *           the OTP memory. Contains the failing address in OTP memory (when CRCERR equals
 *           1).
 * @{
 */
#define MXC_F_SIR_ADDR_ADDR_POS                        0 /**< ADDR_ADDR Position */
#define MXC_F_SIR_ADDR_ADDR                            ((uint32_t)(0xFFFFFFFFUL << MXC_F_SIR_ADDR_ADDR_POS)) /**< ADDR_ADDR Mask */

/**@} end of group SIR_ADDR_Register */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_FSTAT SIR_FSTAT
 * @brief    Function Status Register.
 * @{
 */
#define MXC_F_SIR_FSTAT_FPU_POS                        0 /**< FSTAT_FPU Position */
#define MXC_F_SIR_FSTAT_FPU                            ((uint32_t)(0x1UL << MXC_F_SIR_FSTAT_FPU_POS)) /**< FSTAT_FPU Mask */

#define MXC_F_SIR_FSTAT_TRNG_POS                       14 /**< FSTAT_TRNG Position */
#define MXC_F_SIR_FSTAT_TRNG                           ((uint32_t)(0x1UL << MXC_F_SIR_FSTAT_TRNG_POS)) /**< FSTAT_TRNG Mask */

#define MXC_F_SIR_FSTAT_DS_ACK_POS                     15 /**< FSTAT_DS_ACK Position */
#define MXC_F_SIR_FSTAT_DS_ACK                         ((uint32_t)(0x1UL << MXC_F_SIR_FSTAT_DS_ACK_POS)) /**< FSTAT_DS_ACK Mask */

/**@} end of group SIR_FSTAT_Register */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_SFSTAT SIR_SFSTAT
 * @brief    Security Function Status Register.
 * @{
 */
#define MXC_F_SIR_SFSTAT_SECFUNC0_POS                  0 /**< SFSTAT_SECFUNC0 Position */
#define MXC_F_SIR_SFSTAT_SECFUNC0                      ((uint32_t)(0x1UL << MXC_F_SIR_SFSTAT_SECFUNC0_POS)) /**< SFSTAT_SECFUNC0 Mask */

/**@} end of group SIR_SFSTAT_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32672_INCLUDE_SIR_REGS_H_
