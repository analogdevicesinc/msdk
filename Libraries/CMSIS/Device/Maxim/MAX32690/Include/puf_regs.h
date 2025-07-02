/**
 * @file    puf_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the PUF Peripheral Module.
 * @note    This file is @generated.
 * @ingroup puf_registers
 */

/******************************************************************************
 *
 * Copyright (C) 2025 Analog Devices, Inc.
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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_PUF_REGS_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_PUF_REGS_H_

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
 * @ingroup     puf
 * @defgroup    puf_registers PUF_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the PUF Peripheral Module.
 * @details     PUF Registers
 */

/**
 * @ingroup puf_registers
 * Structure type to access the PUF Registers.
 */
typedef struct {
    __IO uint32_t ctrl;                 /**< <tt>\b 0x0000:</tt> PUF CTRL Register */
    __IO uint32_t stat;                 /**< <tt>\b 0x0004:</tt> PUF STAT Register */
} mxc_puf_regs_t;

/* Register offsets for module PUF */
/**
 * @ingroup    puf_registers
 * @defgroup   PUF_Register_Offsets Register Offsets
 * @brief      PUF Peripheral Register Offsets from the PUF Base Peripheral Address.
 * @{
 */
#define MXC_R_PUF_CTRL                     ((uint32_t)0x00000000UL) /**< Offset from PUF Base Address: <tt> 0x0000</tt> */
#define MXC_R_PUF_STAT                     ((uint32_t)0x00000004UL) /**< Offset from PUF Base Address: <tt> 0x0004</tt> */
/**@} end of group puf_registers */

/**
 * @ingroup  puf_registers
 * @defgroup PUF_CTRL PUF_CTRL
 * @brief    PUF Control Register.
 * @{
 */
#define MXC_F_PUF_CTRL_PUF_EN_POS                      0 /**< CTRL_PUF_EN Position */
#define MXC_F_PUF_CTRL_PUF_EN                          ((uint32_t)(0x1UL << MXC_F_PUF_CTRL_PUF_EN_POS)) /**< CTRL_PUF_EN Mask */

#define MXC_F_PUF_CTRL_KEY_CLR_EN_POS                  1 /**< CTRL_KEY_CLR_EN Position */
#define MXC_F_PUF_CTRL_KEY_CLR_EN                      ((uint32_t)(0x1UL << MXC_F_PUF_CTRL_KEY_CLR_EN_POS)) /**< CTRL_KEY_CLR_EN Mask */

#define MXC_F_PUF_CTRL_KEY0_GEN_EN_POS                 8 /**< CTRL_KEY0_GEN_EN Position */
#define MXC_F_PUF_CTRL_KEY0_GEN_EN                     ((uint32_t)(0x1UL << MXC_F_PUF_CTRL_KEY0_GEN_EN_POS)) /**< CTRL_KEY0_GEN_EN Mask */

#define MXC_F_PUF_CTRL_KEY1_GEN_EN_POS                 9 /**< CTRL_KEY1_GEN_EN Position */
#define MXC_F_PUF_CTRL_KEY1_GEN_EN                     ((uint32_t)(0x1UL << MXC_F_PUF_CTRL_KEY1_GEN_EN_POS)) /**< CTRL_KEY1_GEN_EN Mask */

#define MXC_F_PUF_CTRL_KEYGEN_ERR_IE_POS               16 /**< CTRL_KEYGEN_ERR_IE Position */
#define MXC_F_PUF_CTRL_KEYGEN_ERR_IE                   ((uint32_t)(0x1UL << MXC_F_PUF_CTRL_KEYGEN_ERR_IE_POS)) /**< CTRL_KEYGEN_ERR_IE Mask */

#define MXC_F_PUF_CTRL_KEY0_DN_IE_POS                  24 /**< CTRL_KEY0_DN_IE Position */
#define MXC_F_PUF_CTRL_KEY0_DN_IE                      ((uint32_t)(0x1UL << MXC_F_PUF_CTRL_KEY0_DN_IE_POS)) /**< CTRL_KEY0_DN_IE Mask */

#define MXC_F_PUF_CTRL_KEY1_DN_IE_POS                  25 /**< CTRL_KEY1_DN_IE Position */
#define MXC_F_PUF_CTRL_KEY1_DN_IE                      ((uint32_t)(0x1UL << MXC_F_PUF_CTRL_KEY1_DN_IE_POS)) /**< CTRL_KEY1_DN_IE Mask */

/**@} end of group PUF_CTRL_Register */

/**
 * @ingroup  puf_registers
 * @defgroup PUF_STAT PUF_STAT
 * @brief    PUF Status Register.
 * @{
 */
#define MXC_F_PUF_STAT_BUSY_POS                        0 /**< STAT_BUSY Position */
#define MXC_F_PUF_STAT_BUSY                            ((uint32_t)(0x1UL << MXC_F_PUF_STAT_BUSY_POS)) /**< STAT_BUSY Mask */

#define MXC_F_PUF_STAT_MAGIC_ERR_POS                   1 /**< STAT_MAGIC_ERR Position */
#define MXC_F_PUF_STAT_MAGIC_ERR                       ((uint32_t)(0x1UL << MXC_F_PUF_STAT_MAGIC_ERR_POS)) /**< STAT_MAGIC_ERR Mask */

#define MXC_F_PUF_STAT_KEYGEN_EN_ERR_POS               2 /**< STAT_KEYGEN_EN_ERR Position */
#define MXC_F_PUF_STAT_KEYGEN_EN_ERR                   ((uint32_t)(0x1UL << MXC_F_PUF_STAT_KEYGEN_EN_ERR_POS)) /**< STAT_KEYGEN_EN_ERR Mask */

#define MXC_F_PUF_STAT_KEYGEN_ERR_POS                  3 /**< STAT_KEYGEN_ERR Position */
#define MXC_F_PUF_STAT_KEYGEN_ERR                      ((uint32_t)(0x1UL << MXC_F_PUF_STAT_KEYGEN_ERR_POS)) /**< STAT_KEYGEN_ERR Mask */

#define MXC_F_PUF_STAT_ADC_FREQ_FL_POS                 7 /**< STAT_ADC_FREQ_FL Position */
#define MXC_F_PUF_STAT_ADC_FREQ_FL                     ((uint32_t)(0x1UL << MXC_F_PUF_STAT_ADC_FREQ_FL_POS)) /**< STAT_ADC_FREQ_FL Mask */

#define MXC_F_PUF_STAT_KEY0_INIT_ERR_POS               8 /**< STAT_KEY0_INIT_ERR Position */
#define MXC_F_PUF_STAT_KEY0_INIT_ERR                   ((uint32_t)(0x1UL << MXC_F_PUF_STAT_KEY0_INIT_ERR_POS)) /**< STAT_KEY0_INIT_ERR Mask */

#define MXC_F_PUF_STAT_KEY0_CNST_ERR_POS               9 /**< STAT_KEY0_CNST_ERR Position */
#define MXC_F_PUF_STAT_KEY0_CNST_ERR                   ((uint32_t)(0x1UL << MXC_F_PUF_STAT_KEY0_CNST_ERR_POS)) /**< STAT_KEY0_CNST_ERR Mask */

#define MXC_F_PUF_STAT_KEY1_INIT_ERR_POS               16 /**< STAT_KEY1_INIT_ERR Position */
#define MXC_F_PUF_STAT_KEY1_INIT_ERR                   ((uint32_t)(0x1UL << MXC_F_PUF_STAT_KEY1_INIT_ERR_POS)) /**< STAT_KEY1_INIT_ERR Mask */

#define MXC_F_PUF_STAT_KEY1_CNST_ERR_POS               17 /**< STAT_KEY1_CNST_ERR Position */
#define MXC_F_PUF_STAT_KEY1_CNST_ERR                   ((uint32_t)(0x1UL << MXC_F_PUF_STAT_KEY1_CNST_ERR_POS)) /**< STAT_KEY1_CNST_ERR Mask */

#define MXC_F_PUF_STAT_KEY0_DN_POS                     24 /**< STAT_KEY0_DN Position */
#define MXC_F_PUF_STAT_KEY0_DN                         ((uint32_t)(0x1UL << MXC_F_PUF_STAT_KEY0_DN_POS)) /**< STAT_KEY0_DN Mask */

#define MXC_F_PUF_STAT_KEY1_DN_POS                     25 /**< STAT_KEY1_DN Position */
#define MXC_F_PUF_STAT_KEY1_DN                         ((uint32_t)(0x1UL << MXC_F_PUF_STAT_KEY1_DN_POS)) /**< STAT_KEY1_DN Mask */

/**@} end of group PUF_STAT_Register */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_PUF_REGS_H_
