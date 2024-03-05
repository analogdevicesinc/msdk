/**
 * @file    trng_reva_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the TRNG Peripheral Module.
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

#ifndef _TRNG_REVA_REGS_H_
#define _TRNG_REVA_REGS_H_

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
 * @defgroup    trng_reva_registers TRNG_REVA_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the TRNG Peripheral Module.
 * @details Random Number Generator.
 */

/**
 * @ingroup trng_reva_registers
 * Structure type to access the TRNG Registers.
 */
typedef struct {
    __IO uint32_t ctrl;                 /**< <tt>\b 0x00:</tt> TRNG CTRL Register */
    __IO uint32_t status;               /**< <tt>\b 0x04:</tt> TRNG STATUS Register */
    __I  uint32_t data;                 /**< <tt>\b 0x08:</tt> TRNG DATA Register */
} mxc_trng_reva_regs_t;

/* Register offsets for module TRNG */
/**
 * @ingroup    trng_reva_registers
 * @defgroup   TRNG_REVA_Register_Offsets Register Offsets
 * @brief      TRNG Peripheral Register Offsets from the TRNG Base Peripheral Address. 
 * @{
 */
 #define MXC_R_TRNG_REVA_CTRL                    ((uint32_t)0x00000000UL) /**< Offset from TRNG Base Address: <tt> 0x0000</tt> */ 
 #define MXC_R_TRNG_REVA_STATUS                  ((uint32_t)0x00000004UL) /**< Offset from TRNG Base Address: <tt> 0x0004</tt> */ 
 #define MXC_R_TRNG_REVA_DATA                    ((uint32_t)0x00000008UL) /**< Offset from TRNG Base Address: <tt> 0x0008</tt> */ 
/**@} end of group trng_reva_registers */

/**
 * @ingroup  trng_reva_registers
 * @defgroup TRNG_REVA_CTRL TRNG_REVA_CTRL
 * @brief    TRNG Control Register.
 * @{
 */
 #define MXC_F_TRNG_REVA_CTRL_ODHT_POS                       0 /**< CTRL_ODHT Position */
 #define MXC_F_TRNG_REVA_CTRL_ODHT                           ((uint32_t)(0x1UL << MXC_F_TRNG_REVA_CTRL_ODHT_POS)) /**< CTRL_ODHT Mask */

 #define MXC_F_TRNG_REVA_CTRL_RND_IE_POS                     1 /**< CTRL_RND_IE Position */
 #define MXC_F_TRNG_REVA_CTRL_RND_IE                         ((uint32_t)(0x1UL << MXC_F_TRNG_REVA_CTRL_RND_IE_POS)) /**< CTRL_RND_IE Mask */

 #define MXC_F_TRNG_REVA_CTRL_HEALTH_EN_POS                  2 /**< CTRL_HEALTH_EN Position */
 #define MXC_F_TRNG_REVA_CTRL_HEALTH_EN                      ((uint32_t)(0x1UL << MXC_F_TRNG_REVA_CTRL_HEALTH_EN_POS)) /**< CTRL_HEALTH_EN Mask */

 #define MXC_F_TRNG_REVA_CTRL_AESKG_USR_POS                  3 /**< CTRL_AESKG_USR Position */
 #define MXC_F_TRNG_REVA_CTRL_AESKG_USR                      ((uint32_t)(0x1UL << MXC_F_TRNG_REVA_CTRL_AESKG_USR_POS)) /**< CTRL_AESKG_USR Mask */

 #define MXC_F_TRNG_REVA_CTRL_AESKG_SYS_POS                  4 /**< CTRL_AESKG_SYS Position */
 #define MXC_F_TRNG_REVA_CTRL_AESKG_SYS                      ((uint32_t)(0x1UL << MXC_F_TRNG_REVA_CTRL_AESKG_SYS_POS)) /**< CTRL_AESKG_SYS Mask */

 #define MXC_F_TRNG_REVA_CTRL_KEYWIPE_POS                    15 /**< CTRL_KEYWIPE Position */
 #define MXC_F_TRNG_REVA_CTRL_KEYWIPE                        ((uint32_t)(0x1UL << MXC_F_TRNG_REVA_CTRL_KEYWIPE_POS)) /**< CTRL_KEYWIPE Mask */

/**@} end of group TRNG_REVA_CTRL_Register */

/**
 * @ingroup  trng_reva_registers
 * @defgroup TRNG_REVA_STATUS TRNG_REVA_STATUS
 * @brief    Data. The content of this register is valid only when RNG_IS = 1. When TRNG is
 *           disabled, read returns 0x0000 0000.
 * @{
 */
 #define MXC_F_TRNG_REVA_STATUS_RDY_POS                      0 /**< STATUS_RDY Position */
 #define MXC_F_TRNG_REVA_STATUS_RDY                          ((uint32_t)(0x1UL << MXC_F_TRNG_REVA_STATUS_RDY_POS)) /**< STATUS_RDY Mask */

 #define MXC_F_TRNG_REVA_STATUS_ODHT_POS                     1 /**< STATUS_ODHT Position */
 #define MXC_F_TRNG_REVA_STATUS_ODHT                         ((uint32_t)(0x1UL << MXC_F_TRNG_REVA_STATUS_ODHT_POS)) /**< STATUS_ODHT Mask */

 #define MXC_F_TRNG_REVA_STATUS_HT_POS                       2 /**< STATUS_HT Position */
 #define MXC_F_TRNG_REVA_STATUS_HT                           ((uint32_t)(0x1UL << MXC_F_TRNG_REVA_STATUS_HT_POS)) /**< STATUS_HT Mask */

 #define MXC_F_TRNG_REVA_STATUS_SRCFAIL_POS                  3 /**< STATUS_SRCFAIL Position */
 #define MXC_F_TRNG_REVA_STATUS_SRCFAIL                      ((uint32_t)(0x1UL << MXC_F_TRNG_REVA_STATUS_SRCFAIL_POS)) /**< STATUS_SRCFAIL Mask */

 #define MXC_F_TRNG_REVA_STATUS_AESKGD_POS                   4 /**< STATUS_AESKGD Position */
 #define MXC_F_TRNG_REVA_STATUS_AESKGD                       ((uint32_t)(0x1UL << MXC_F_TRNG_REVA_STATUS_AESKGD_POS)) /**< STATUS_AESKGD Mask */

 #define MXC_F_TRNG_REVA_STATUS_LD_CNT_POS                   24 /**< STATUS_LD_CNT Position */
 #define MXC_F_TRNG_REVA_STATUS_LD_CNT                       ((uint32_t)(0xFFUL << MXC_F_TRNG_REVA_STATUS_LD_CNT_POS)) /**< STATUS_LD_CNT Mask */

/**@} end of group TRNG_REVA_STATUS_Register */

/**
 * @ingroup  trng_reva_registers
 * @defgroup TRNG_REVA_DATA TRNG_REVA_DATA
 * @brief    Data. The content of this register is valid only when RNG_IS = 1. When TRNG is
 *           disabled, read returns 0x0000 0000.
 * @{
 */
 #define MXC_F_TRNG_REVA_DATA_DATA_POS                       0 /**< DATA_DATA Position */
 #define MXC_F_TRNG_REVA_DATA_DATA                           ((uint32_t)(0xFFFFFFFFUL << MXC_F_TRNG_REVA_DATA_DATA_POS)) /**< DATA_DATA Mask */

/**@} end of group TRNG_REVA_DATA_Register */

#ifdef __cplusplus
}
#endif

#endif /* _TRNG_REVA_REGS_H_ */
