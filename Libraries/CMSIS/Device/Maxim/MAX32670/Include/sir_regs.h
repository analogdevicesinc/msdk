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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32670_INCLUDE_SIR_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32670_INCLUDE_SIR_REGS_H_

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
    __I  uint32_t sir_status;           /**< <tt>\b 0x00:</tt> SIR SIR_STATUS Register */
    __I  uint32_t sir_addr;             /**< <tt>\b 0x04:</tt> SIR SIR_ADDR Register */
} mxc_sir_regs_t;

/* Register offsets for module SIR */
/**
 * @ingroup    sir_registers
 * @defgroup   SIR_Register_Offsets Register Offsets
 * @brief      SIR Peripheral Register Offsets from the SIR Base Peripheral Address.
 * @{
 */
#define MXC_R_SIR_SIR_STATUS               ((uint32_t)0x00000000UL) /**< Offset from SIR Base Address: <tt> 0x0000</tt> */
#define MXC_R_SIR_SIR_ADDR                 ((uint32_t)0x00000004UL) /**< Offset from SIR Base Address: <tt> 0x0004</tt> */
/**@} end of group sir_registers */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_SIR_STATUS SIR_SIR_STATUS
 * @brief    System Initialization Status Register.
 * @{
 */
#define MXC_F_SIR_SIR_STATUS_CFG_VALID_POS             0 /**< SIR_STATUS_CFG_VALID Position */
#define MXC_F_SIR_SIR_STATUS_CFG_VALID                 ((uint32_t)(0x1UL << MXC_F_SIR_SIR_STATUS_CFG_VALID_POS)) /**< SIR_STATUS_CFG_VALID Mask */

#define MXC_F_SIR_SIR_STATUS_CFG_ERR_POS               1 /**< SIR_STATUS_CFG_ERR Position */
#define MXC_F_SIR_SIR_STATUS_CFG_ERR                   ((uint32_t)(0x1UL << MXC_F_SIR_SIR_STATUS_CFG_ERR_POS)) /**< SIR_STATUS_CFG_ERR Mask */

/**@} end of group SIR_SIR_STATUS_Register */

/**
 * @ingroup  sir_registers
 * @defgroup SIR_SIR_ADDR SIR_SIR_ADDR
 * @brief    Read-only field set by the SIB block if a CRC error occurs during the read of
 *           the OTP memory. Contains the failing address in OTP memory (when CRCERR equals
 *           1).
 * @{
 */
#define MXC_F_SIR_SIR_ADDR_ADDR_POS                    0 /**< SIR_ADDR_ADDR Position */
#define MXC_F_SIR_SIR_ADDR_ADDR                        ((uint32_t)(0xFFFFFFFFUL << MXC_F_SIR_SIR_ADDR_ADDR_POS)) /**< SIR_ADDR_ADDR Mask */

/**@} end of group SIR_SIR_ADDR_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32670_INCLUDE_SIR_REGS_H_
