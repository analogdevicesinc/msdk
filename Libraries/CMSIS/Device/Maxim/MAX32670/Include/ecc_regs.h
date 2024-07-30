/**
 * @file    ecc_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the ECC Peripheral Module.
 * @note    This file is @generated.
 * @ingroup ecc_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32670_INCLUDE_ECC_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32670_INCLUDE_ECC_REGS_H_

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
 * @ingroup     ecc
 * @defgroup    ecc_registers ECC_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the ECC Peripheral Module.
 * @details     Trim System Initilazation Registers. ECC Registers for MAX32670.
 */

/**
 * @ingroup ecc_registers
 * Structure type to access the ECC Registers.
 */
typedef struct {
    __R  uint32_t rsv_0x0_0x7[2];
    __IO uint32_t en;                   /**< <tt>\b 0x08:</tt> ECC EN Register */
} mxc_ecc_regs_t;

/* Register offsets for module ECC */
/**
 * @ingroup    ecc_registers
 * @defgroup   ECC_Register_Offsets Register Offsets
 * @brief      ECC Peripheral Register Offsets from the ECC Base Peripheral Address.
 * @{
 */
#define MXC_R_ECC_EN                       ((uint32_t)0x00000008UL) /**< Offset from ECC Base Address: <tt> 0x0008</tt> */
/**@} end of group ecc_registers */

/**
 * @ingroup  ecc_registers
 * @defgroup ECC_EN ECC_EN
 * @brief    ECC Enable Register.
 * @{
 */
#define MXC_F_ECC_EN_SRAM_POS                          8 /**< EN_SRAM Position */
#define MXC_F_ECC_EN_SRAM                              ((uint32_t)(0x1UL << MXC_F_ECC_EN_SRAM_POS)) /**< EN_SRAM Mask */

#define MXC_F_ECC_EN_ICC_POS                           9 /**< EN_ICC Position */
#define MXC_F_ECC_EN_ICC                               ((uint32_t)(0x1UL << MXC_F_ECC_EN_ICC_POS)) /**< EN_ICC Mask */

#define MXC_F_ECC_EN_FLASH_POS                         10 /**< EN_FLASH Position */
#define MXC_F_ECC_EN_FLASH                             ((uint32_t)(0x1UL << MXC_F_ECC_EN_FLASH_POS)) /**< EN_FLASH Mask */

/**@} end of group ECC_EN_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32670_INCLUDE_ECC_REGS_H_
