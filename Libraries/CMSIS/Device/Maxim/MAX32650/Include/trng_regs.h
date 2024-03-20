/**
 * @file    trng_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the TRNG Peripheral Module.
 * @note    This file is @generated.
 * @ingroup trng_registers
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_TRNG_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_TRNG_REGS_H_

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
 * @ingroup     trng
 * @defgroup    trng_registers TRNG_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the TRNG Peripheral Module.
 * @details     Random Number Generator.
 */

/**
 * @ingroup trng_registers
 * Structure type to access the TRNG Registers.
 */
typedef struct {
    __IO uint32_t ctrl;                 /**< <tt>\b 0x00:</tt> TRNG CTRL Register */
    __I  uint32_t data;                 /**< <tt>\b 0x04:</tt> TRNG DATA Register */
} mxc_trng_regs_t;

/* Register offsets for module TRNG */
/**
 * @ingroup    trng_registers
 * @defgroup   TRNG_Register_Offsets Register Offsets
 * @brief      TRNG Peripheral Register Offsets from the TRNG Base Peripheral Address.
 * @{
 */
#define MXC_R_TRNG_CTRL                    ((uint32_t)0x00000000UL) /**< Offset from TRNG Base Address: <tt> 0x0000</tt> */
#define MXC_R_TRNG_DATA                    ((uint32_t)0x00000004UL) /**< Offset from TRNG Base Address: <tt> 0x0004</tt> */
/**@} end of group trng_registers */

/**
 * @ingroup  trng_registers
 * @defgroup TRNG_CTRL TRNG_CTRL
 * @brief    TRNG Control Register.
 * @{
 */
#define MXC_F_TRNG_CTRL_RNG_IE_POS                     2 /**< CTRL_RNG_IE Position */
#define MXC_F_TRNG_CTRL_RNG_IE                         ((uint32_t)(0x1UL << MXC_F_TRNG_CTRL_RNG_IE_POS)) /**< CTRL_RNG_IE Mask */
#define MXC_V_TRNG_CTRL_RNG_IE_DIS                     ((uint32_t)0x0UL) /**< CTRL_RNG_IE_DIS Value */
#define MXC_S_TRNG_CTRL_RNG_IE_DIS                     (MXC_V_TRNG_CTRL_RNG_IE_DIS << MXC_F_TRNG_CTRL_RNG_IE_POS) /**< CTRL_RNG_IE_DIS Setting */
#define MXC_V_TRNG_CTRL_RNG_IE_EN                      ((uint32_t)0x1UL) /**< CTRL_RNG_IE_EN Value */
#define MXC_S_TRNG_CTRL_RNG_IE_EN                      (MXC_V_TRNG_CTRL_RNG_IE_EN << MXC_F_TRNG_CTRL_RNG_IE_POS) /**< CTRL_RNG_IE_EN Setting */

#define MXC_F_TRNG_CTRL_RNG_ISC_POS                    3 /**< CTRL_RNG_ISC Position */
#define MXC_F_TRNG_CTRL_RNG_ISC                        ((uint32_t)(0x1UL << MXC_F_TRNG_CTRL_RNG_ISC_POS)) /**< CTRL_RNG_ISC Mask */
#define MXC_V_TRNG_CTRL_RNG_ISC_CLEAR                  ((uint32_t)0x1UL) /**< CTRL_RNG_ISC_CLEAR Value */
#define MXC_S_TRNG_CTRL_RNG_ISC_CLEAR                  (MXC_V_TRNG_CTRL_RNG_ISC_CLEAR << MXC_F_TRNG_CTRL_RNG_ISC_POS) /**< CTRL_RNG_ISC_CLEAR Setting */

#define MXC_F_TRNG_CTRL_RNG_I4S_POS                    4 /**< CTRL_RNG_I4S Position */
#define MXC_F_TRNG_CTRL_RNG_I4S                        ((uint32_t)(0x1UL << MXC_F_TRNG_CTRL_RNG_I4S_POS)) /**< CTRL_RNG_I4S Mask */
#define MXC_V_TRNG_CTRL_RNG_I4S_NOT_READY              ((uint32_t)0x0UL) /**< CTRL_RNG_I4S_NOT_READY Value */
#define MXC_S_TRNG_CTRL_RNG_I4S_NOT_READY              (MXC_V_TRNG_CTRL_RNG_I4S_NOT_READY << MXC_F_TRNG_CTRL_RNG_I4S_POS) /**< CTRL_RNG_I4S_NOT_READY Setting */
#define MXC_V_TRNG_CTRL_RNG_I4S_READY                  ((uint32_t)0x1UL) /**< CTRL_RNG_I4S_READY Value */
#define MXC_S_TRNG_CTRL_RNG_I4S_READY                  (MXC_V_TRNG_CTRL_RNG_I4S_READY << MXC_F_TRNG_CTRL_RNG_I4S_POS) /**< CTRL_RNG_I4S_READY Setting */

#define MXC_F_TRNG_CTRL_RNG_IS_POS                     5 /**< CTRL_RNG_IS Position */
#define MXC_F_TRNG_CTRL_RNG_IS                         ((uint32_t)(0x1UL << MXC_F_TRNG_CTRL_RNG_IS_POS)) /**< CTRL_RNG_IS Mask */
#define MXC_V_TRNG_CTRL_RNG_IS_NOT_READY               ((uint32_t)0x0UL) /**< CTRL_RNG_IS_NOT_READY Value */
#define MXC_S_TRNG_CTRL_RNG_IS_NOT_READY               (MXC_V_TRNG_CTRL_RNG_IS_NOT_READY << MXC_F_TRNG_CTRL_RNG_IS_POS) /**< CTRL_RNG_IS_NOT_READY Setting */
#define MXC_V_TRNG_CTRL_RNG_IS_READY                   ((uint32_t)0x1UL) /**< CTRL_RNG_IS_READY Value */
#define MXC_S_TRNG_CTRL_RNG_IS_READY                   (MXC_V_TRNG_CTRL_RNG_IS_READY << MXC_F_TRNG_CTRL_RNG_IS_POS) /**< CTRL_RNG_IS_READY Setting */

#define MXC_F_TRNG_CTRL_AESKG_POS                      6 /**< CTRL_AESKG Position */
#define MXC_F_TRNG_CTRL_AESKG                          ((uint32_t)(0x1UL << MXC_F_TRNG_CTRL_AESKG_POS)) /**< CTRL_AESKG Mask */

/**@} end of group TRNG_CTRL_Register */

/**
 * @ingroup  trng_registers
 * @defgroup TRNG_DATA TRNG_DATA
 * @brief    Data. The content of this register is valid only when RNG_IS = 1. When TRNG is
 *           disabled, read returns 0x0000 0000.
 * @{
 */
#define MXC_F_TRNG_DATA_DATA_POS                       0 /**< DATA_DATA Position */
#define MXC_F_TRNG_DATA_DATA                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_TRNG_DATA_DATA_POS)) /**< DATA_DATA Mask */

/**@} end of group TRNG_DATA_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_TRNG_REGS_H_
