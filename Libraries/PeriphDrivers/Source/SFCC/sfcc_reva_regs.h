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

 /**
 * @file    sfcc_reva_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SFCC_REVA Peripheral Module.
 */

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_SFCC_SFCC_REVA_REGS_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_SFCC_SFCC_REVA_REGS_H_

/* **** Includes **** */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined (__SFCCARM__)
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
 * @ingroup     sfcc_reva
 * @defgroup    sfcc_reva_registers SFCC_REVA_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SFCC_REVA Peripheral Module.
 * @details Instruction Cache Controller Registers
 */

/**
 * @ingroup sfcc_reva_registers
 * Structure type to access the SFCC_REVA Registers.
 */
typedef struct {
    __I  uint32_t info;                 /**< <tt>\b 0x0000:</tt> SFCC_REVA INFO Register */
    __I  uint32_t sz;                   /**< <tt>\b 0x0004:</tt> SFCC_REVA SZ Register */
    __R  uint32_t rsv_0x8_0xff[62];
    __IO uint32_t ctrl;                 /**< <tt>\b 0x0100:</tt> SFCC_REVA CTRL Register */
    __R  uint32_t rsv_0x104_0x6ff[383];
    __IO uint32_t invalidate;           /**< <tt>\b 0x0700:</tt> SFCC_REVA INVALIDATE Register */
} mxc_sfcc_reva_regs_t;

/* Register offsets for module SFCC_REVA */
/**
 * @ingroup    sfcc_reva_registers
 * @defgroup   SFCC_REVA_Register_Offsets Register Offsets
 * @brief      SFCC_REVA Peripheral Register Offsets from the SFCC_REVA Base Peripheral Address.
 * @{
 */
#define MXC_R_SFCC_REVA_INFO                ((uint32_t)0x00000000UL) /**< Offset from SFCC_REVA Base Address: <tt> 0x0000</tt> */
#define MXC_R_SFCC_REVA_SZ                  ((uint32_t)0x00000004UL) /**< Offset from SFCC_REVA Base Address: <tt> 0x0004</tt> */
#define MXC_R_SFCC_REVA_CTRL                ((uint32_t)0x00000100UL) /**< Offset from SFCC_REVA Base Address: <tt> 0x0100</tt> */
#define MXC_R_SFCC_REVA_INVALIDATE          ((uint32_t)0x00000700UL) /**< Offset from SFCC_REVA Base Address: <tt> 0x0700</tt> */
/**@} end of group sfcc_reva_registers */

/**
 * @ingroup  sfcc_reva_registers
 * @defgroup SFCC_REVA_INFO SFCC_REVA_INFO
 * @brief    Cache ID Register.
 * @{
 */
#define MXC_F_SFCC_REVA_INFO_RELNUM_POS                 0 /**< INFO_RELNUM Position */
#define MXC_F_SFCC_REVA_INFO_RELNUM                     ((uint32_t)(0x3FUL << MXC_F_SFCC_REVA_INFO_RELNUM_POS)) /**< INFO_RELNUM Mask */

#define MXC_F_SFCC_REVA_INFO_PARTNUM_POS                6 /**< INFO_PARTNUM Position */
#define MXC_F_SFCC_REVA_INFO_PARTNUM                    ((uint32_t)(0xFUL << MXC_F_SFCC_REVA_INFO_PARTNUM_POS)) /**< INFO_PARTNUM Mask */

#define MXC_F_SFCC_REVA_INFO_ID_POS                     10 /**< INFO_ID Position */
#define MXC_F_SFCC_REVA_INFO_ID                         ((uint32_t)(0x3FUL << MXC_F_SFCC_REVA_INFO_ID_POS)) /**< INFO_ID Mask */

/**@} end of group SFCC_REVA_INFO_Register */

/**
 * @ingroup  sfcc_reva_registers
 * @defgroup SFCC_REVA_SZ SFCC_REVA_SZ
 * @brief    Memory Configuration Register.
 * @{
 */
#define MXC_F_SFCC_REVA_SZ_CCH_POS                      0 /**< SZ_CCH Position */
#define MXC_F_SFCC_REVA_SZ_CCH                          ((uint32_t)(0xFFFFUL << MXC_F_SFCC_REVA_SZ_CCH_POS)) /**< SZ_CCH Mask */

#define MXC_F_SFCC_REVA_SZ_MEM_POS                      16 /**< SZ_MEM Position */
#define MXC_F_SFCC_REVA_SZ_MEM                          ((uint32_t)(0xFFFFUL << MXC_F_SFCC_REVA_SZ_MEM_POS)) /**< SZ_MEM Mask */

/**@} end of group SFCC_REVA_SZ_Register */

/**
 * @ingroup  sfcc_reva_registers
 * @defgroup SFCC_REVA_CTRL SFCC_REVA_CTRL
 * @brief    Cache Control and Status Register.
 * @{
 */
#define MXC_F_SFCC_REVA_CTRL_EN_POS                     0 /**< CTRL_EN Position */
#define MXC_F_SFCC_REVA_CTRL_EN                         ((uint32_t)(0x1UL << MXC_F_SFCC_REVA_CTRL_EN_POS)) /**< CTRL_EN Mask */

#define MXC_F_SFCC_REVA_CTRL_RDY_POS                    16 /**< CTRL_RDY Position */
#define MXC_F_SFCC_REVA_CTRL_RDY                        ((uint32_t)(0x1UL << MXC_F_SFCC_REVA_CTRL_RDY_POS)) /**< CTRL_RDY Mask */

/**@} end of group SFCC_REVA_CTRL_Register */

/**
 * @ingroup  sfcc_reva_registers
 * @defgroup SFCC_REVA_INVALIDATE SFCC_REVA_INVALIDATE
 * @brief    Invalidate All Registers.
 * @{
 */
#define MXC_F_SFCC_REVA_INVALIDATE_INVALID_POS              0 /**< INVALIDATE_INVALID Position */
#define MXC_F_SFCC_REVA_INVALIDATE_INVALID                  ((uint32_t)(0xFFFFFFFFUL << MXC_F_SFCC_REVA_INVALIDATE_INVALID_POS)) /**< INVALIDATE_INVALID Mask */

/**@} end of group SFCC_REVA_INVALIDATE_Register */

#ifdef __cplusplus
}
#endif

#endif  // LIBRARIES_PERIPHDRIVERS_SOURCE_SFCC_SFCC_REVA_REGS_H_

