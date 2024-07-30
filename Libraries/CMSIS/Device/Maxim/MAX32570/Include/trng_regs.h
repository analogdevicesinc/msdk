/**
 * @file    trng_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the TRNG Peripheral Module.
 * @note    This file is @generated.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_TRNG_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_TRNG_REGS_H_

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
    __IO uint32_t cn;                   /**< <tt>\b 0x00:</tt> TRNG CN Register */
    __I  uint32_t st;                   /**< <tt>\b 0x04:</tt> TRNG ST Register */
    __I  uint32_t data;                 /**< <tt>\b 0x08:</tt> TRNG DATA Register */
} mxc_trng_regs_t;

/* Register offsets for module TRNG */
/**
 * @ingroup    trng_registers
 * @defgroup   TRNG_Register_Offsets Register Offsets
 * @brief      TRNG Peripheral Register Offsets from the TRNG Base Peripheral Address.
 * @{
 */
#define MXC_R_TRNG_CN                      ((uint32_t)0x00000000UL) /**< Offset from TRNG Base Address: <tt> 0x0000</tt> */
#define MXC_R_TRNG_ST                      ((uint32_t)0x00000004UL) /**< Offset from TRNG Base Address: <tt> 0x0004</tt> */
#define MXC_R_TRNG_DATA                    ((uint32_t)0x00000008UL) /**< Offset from TRNG Base Address: <tt> 0x0008</tt> */
/**@} end of group trng_registers */

/**
 * @ingroup  trng_registers
 * @defgroup TRNG_CN TRNG_CN
 * @brief    TRNG Control Register.
 * @{
 */
#define MXC_F_TRNG_CN_RND_IRQ_EN_POS                   1 /**< CN_RND_IRQ_EN Position */
#define MXC_F_TRNG_CN_RND_IRQ_EN                       ((uint32_t)(0x1UL << MXC_F_TRNG_CN_RND_IRQ_EN_POS)) /**< CN_RND_IRQ_EN Mask */

#define MXC_F_TRNG_CN_AESKG_POS                        3 /**< CN_AESKG Position */
#define MXC_F_TRNG_CN_AESKG                            ((uint32_t)(0x1UL << MXC_F_TRNG_CN_AESKG_POS)) /**< CN_AESKG Mask */

#define MXC_F_TRNG_CN_AESKG_MEMPROTE_POS               4 /**< CN_AESKG_MEMPROTE Position */
#define MXC_F_TRNG_CN_AESKG_MEMPROTE                   ((uint32_t)(0x1UL << MXC_F_TRNG_CN_AESKG_MEMPROTE_POS)) /**< CN_AESKG_MEMPROTE Mask */

/**@} end of group TRNG_CN_Register */

/**
 * @ingroup  trng_registers
 * @defgroup TRNG_ST TRNG_ST
 * @brief    Data. The content of this register is valid only when RNG_IS = 1. When TRNG is
 *           disabled, read returns 0x0000 0000.
 * @{
 */
#define MXC_F_TRNG_ST_RND_RDY_POS                      0 /**< ST_RND_RDY Position */
#define MXC_F_TRNG_ST_RND_RDY                          ((uint32_t)(0x1UL << MXC_F_TRNG_ST_RND_RDY_POS)) /**< ST_RND_RDY Mask */

#define MXC_F_TRNG_ST_AESKGD_MEU_S_POS                 4 /**< ST_AESKGD_MEU_S Position */
#define MXC_F_TRNG_ST_AESKGD_MEU_S                     ((uint32_t)(0x1UL << MXC_F_TRNG_ST_AESKGD_MEU_S_POS)) /**< ST_AESKGD_MEU_S Mask */

/**@} end of group TRNG_ST_Register */

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

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32570_INCLUDE_TRNG_REGS_H_
