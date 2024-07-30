/**
 * @file    nspc_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the NSPC Peripheral Module.
 * @note    This file is @generated.
 * @ingroup nspc_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_NSPC_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_NSPC_REGS_H_

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
 * @ingroup     nspc
 * @defgroup    nspc_registers NSPC_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the NSPC Peripheral Module.
 * @details     Non-Secure Privilege Controller.
 */

/**
 * @ingroup nspc_registers
 * Structure type to access the NSPC Registers.
 */
typedef struct {
    __R  uint32_t rsv_0x0_0x15f[88];
    __IO uint32_t apbpriv;              /**< <tt>\b 0x0160:</tt> NSPC APBPRIV Register */
} mxc_nspc_regs_t;

/* Register offsets for module NSPC */
/**
 * @ingroup    nspc_registers
 * @defgroup   NSPC_Register_Offsets Register Offsets
 * @brief      NSPC Peripheral Register Offsets from the NSPC Base Peripheral Address.
 * @{
 */
#define MXC_R_NSPC_APBPRIV                 ((uint32_t)0x00000160UL) /**< Offset from NSPC Base Address: <tt> 0x0160</tt> */
/**@} end of group nspc_registers */

/**
 * @ingroup  nspc_registers
 * @defgroup NSPC_APBPRIV NSPC_APBPRIV
 * @brief    APB Tartet Privileged/Non-privileged PPC Access Register.
 * @{
 */
#define MXC_F_NSPC_APBPRIV_PERIPH_POS                  0 /**< APBPRIV_PERIPH Position */
#define MXC_F_NSPC_APBPRIV_PERIPH                      ((uint32_t)(0xFFFFFFFFUL << MXC_F_NSPC_APBPRIV_PERIPH_POS)) /**< APBPRIV_PERIPH Mask */

/**@} end of group NSPC_APBPRIV_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_NSPC_REGS_H_
