/**
 * @file    bbfc_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the BBFC Peripheral Module.
 * @note    This file is @generated.
 * @ingroup bbfc_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_BBFC_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_BBFC_REGS_H_

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
 * @ingroup     bbfc
 * @defgroup    bbfc_registers BBFC_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the BBFC Peripheral Module.
 * @details     Battery Backed Function Control Register.
 */

/**
 * @ingroup bbfc_registers
 * Structure type to access the BBFC Registers.
 */
typedef struct {
    __IO uint32_t bbfcr0;               /**< <tt>\b 0x00:</tt> BBFC BBFCR0 Register */
} mxc_bbfc_regs_t;

/* Register offsets for module BBFC */
/**
 * @ingroup    bbfc_registers
 * @defgroup   BBFC_Register_Offsets Register Offsets
 * @brief      BBFC Peripheral Register Offsets from the BBFC Base Peripheral Address.
 * @{
 */
#define MXC_R_BBFC_BBFCR0                  ((uint32_t)0x00000000UL) /**< Offset from BBFC Base Address: <tt> 0x0000</tt> */
/**@} end of group bbfc_registers */

/**
 * @ingroup  bbfc_registers
 * @defgroup BBFC_BBFCR0 BBFC_BBFCR0
 * @brief    Register 0.
 * @{
 */
#define MXC_F_BBFC_BBFCR0_CKPDRV_POS                   0 /**< BBFCR0_CKPDRV Position */
#define MXC_F_BBFC_BBFCR0_CKPDRV                       ((uint32_t)(0xFUL << MXC_F_BBFC_BBFCR0_CKPDRV_POS)) /**< BBFCR0_CKPDRV Mask */

#define MXC_F_BBFC_BBFCR0_CKNPDRV_POS                  4 /**< BBFCR0_CKNPDRV Position */
#define MXC_F_BBFC_BBFCR0_CKNPDRV                      ((uint32_t)(0xFUL << MXC_F_BBFC_BBFCR0_CKNPDRV_POS)) /**< BBFCR0_CKNPDRV Mask */

#define MXC_F_BBFC_BBFCR0_RDSDLLEN_POS                 8 /**< BBFCR0_RDSDLLEN Position */
#define MXC_F_BBFC_BBFCR0_RDSDLLEN                     ((uint32_t)(0x1UL << MXC_F_BBFC_BBFCR0_RDSDLLEN_POS)) /**< BBFCR0_RDSDLLEN Mask */
#define MXC_V_BBFC_BBFCR0_RDSDLLEN_DIS                 ((uint32_t)0x0UL) /**< BBFCR0_RDSDLLEN_DIS Value */
#define MXC_S_BBFC_BBFCR0_RDSDLLEN_DIS                 (MXC_V_BBFC_BBFCR0_RDSDLLEN_DIS << MXC_F_BBFC_BBFCR0_RDSDLLEN_POS) /**< BBFCR0_RDSDLLEN_DIS Setting */
#define MXC_V_BBFC_BBFCR0_RDSDLLEN_EN                  ((uint32_t)0x1UL) /**< BBFCR0_RDSDLLEN_EN Value */
#define MXC_S_BBFC_BBFCR0_RDSDLLEN_EN                  (MXC_V_BBFC_BBFCR0_RDSDLLEN_EN << MXC_F_BBFC_BBFCR0_RDSDLLEN_POS) /**< BBFCR0_RDSDLLEN_EN Setting */

/**@} end of group BBFC_BBFCR0_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_BBFC_REGS_H_
