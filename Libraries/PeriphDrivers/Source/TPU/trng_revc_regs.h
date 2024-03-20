/**
 * @file    trng_revc_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the TRNG_REVC Peripheral Module.
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

#ifndef _TRNG_REVC_REGS_H_
#define _TRNG_REVC_REGS_H_

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
 * @ingroup     trng_revc
 * @defgroup    trng_revc_registers TRNG_REVC_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the TRNG_REVC Peripheral Module.
 * @details Random Number Generator.
 */

/**
 * @ingroup trng_revc_registers
 * Structure type to access the TRNG_REVC Registers.
 */
typedef struct {
    __IO uint32_t cn;                   /**< <tt>\b 0x00:</tt> TRNG_REVC CN Register */
    __I  uint32_t st;                   /**< <tt>\b 0x04:</tt> TRNG_REVC ST Register */
    __I  uint32_t data;                 /**< <tt>\b 0x08:</tt> TRNG_REVC DATA Register */
} mxc_trng_revc_regs_t;

/* Register offsets for module TRNG_REVC */
/**
 * @ingroup    trng_revc_registers
 * @defgroup   TRNG_REVC_Register_Offsets Register Offsets
 * @brief      TRNG_REVC Peripheral Register Offsets from the TRNG_REVC Base Peripheral Address. 
 * @{
 */
 #define MXC_R_TRNG_REVC_CN                 ((uint32_t)0x00000000UL) /**< Offset from TRNG_REVC Base Address: <tt> 0x0000</tt> */ 
 #define MXC_R_TRNG_REVC_ST                 ((uint32_t)0x00000004UL) /**< Offset from TRNG_REVC Base Address: <tt> 0x0004</tt> */ 
 #define MXC_R_TRNG_REVC_DATA               ((uint32_t)0x00000008UL) /**< Offset from TRNG_REVC Base Address: <tt> 0x0008</tt> */ 
/**@} end of group trng_revc_registers */

/**
 * @ingroup  trng_revc_registers
 * @defgroup TRNG_REVC_CN TRNG_REVC_CN
 * @brief    TRNG Control Register.
 * @{
 */
 #define MXC_F_TRNG_REVC_CN_ODHT_POS                    0 /**< CN_ODHT Position */
 #define MXC_F_TRNG_REVC_CN_ODHT                        ((uint32_t)(0x1UL << MXC_F_TRNG_REVC_CN_ODHT_POS)) /**< CN_ODHT Mask */

 #define MXC_F_TRNG_REVC_CN_RND_IRQ_EN_POS              1 /**< CN_RND_IRQ_EN Position */
 #define MXC_F_TRNG_REVC_CN_RND_IRQ_EN                  ((uint32_t)(0x1UL << MXC_F_TRNG_REVC_CN_RND_IRQ_EN_POS)) /**< CN_RND_IRQ_EN Mask */

 #define MXC_F_TRNG_REVC_CN_HEALTH_EN_POS               2 /**< CN_HEALTH_EN Position */
 #define MXC_F_TRNG_REVC_CN_HEALTH_EN                   ((uint32_t)(0x1UL << MXC_F_TRNG_REVC_CN_HEALTH_EN_POS)) /**< CN_HEALTH_EN Mask */

 #define MXC_F_TRNG_REVC_CN_AESKG_MEU_POS               3 /**< CN_AESKG_MEU Position */
 #define MXC_F_TRNG_REVC_CN_AESKG_MEU                   ((uint32_t)(0x1UL << MXC_F_TRNG_REVC_CN_AESKG_MEU_POS)) /**< CN_AESKG_MEU Mask */

 #define MXC_F_TRNG_REVC_CN_AESKG_MEMPROTE_POS          4 /**< CN_AESKG_MEMPROTE Position */
 #define MXC_F_TRNG_REVC_CN_AESKG_MEMPROTE              ((uint32_t)(0x1UL << MXC_F_TRNG_REVC_CN_AESKG_MEMPROTE_POS)) /**< CN_AESKG_MEMPROTE Mask */

 #define MXC_F_TRNG_REVC_CN_AESKG_MEMPROTA_POS          5 /**< CN_AESKG_MEMPROTA Position */
 #define MXC_F_TRNG_REVC_CN_AESKG_MEMPROTA              ((uint32_t)(0x1UL << MXC_F_TRNG_REVC_CN_AESKG_MEMPROTA_POS)) /**< CN_AESKG_MEMPROTA Mask */

 #define MXC_F_TRNG_REVC_CN_RSV16_POS                   16 /**< CN_RSV16 Position */
 #define MXC_F_TRNG_REVC_CN_RSV16                       ((uint32_t)(0x1UL << MXC_F_TRNG_REVC_CN_RSV16_POS)) /**< CN_RSV16 Mask */

 #define MXC_F_TRNG_REVC_CN_RSV17_POS                   17 /**< CN_RSV17 Position */
 #define MXC_F_TRNG_REVC_CN_RSV17                       ((uint32_t)(0x1UL << MXC_F_TRNG_REVC_CN_RSV17_POS)) /**< CN_RSV17 Mask */

/**@} end of group TRNG_REVC_CN_Register */

/**
 * @ingroup  trng_revc_registers
 * @defgroup TRNG_REVC_ST TRNG_REVC_ST
 * @brief    Data. The content of this register is valid only when RNG_IS = 1. When TRNG is
 *           disabled, read returns 0x0000 0000.
 * @{
 */
 #define MXC_F_TRNG_REVC_ST_RND_RDY_POS                 0 /**< ST_RND_RDY Position */
 #define MXC_F_TRNG_REVC_ST_RND_RDY                     ((uint32_t)(0x1UL << MXC_F_TRNG_REVC_ST_RND_RDY_POS)) /**< ST_RND_RDY Mask */

 #define MXC_F_TRNG_REVC_ST_ODHTS_POS                   1 /**< ST_ODHTS Position */
 #define MXC_F_TRNG_REVC_ST_ODHTS                       ((uint32_t)(0x1UL << MXC_F_TRNG_REVC_ST_ODHTS_POS)) /**< ST_ODHTS Mask */

 #define MXC_F_TRNG_REVC_ST_HTS_POS                     2 /**< ST_HTS Position */
 #define MXC_F_TRNG_REVC_ST_HTS                         ((uint32_t)(0x1UL << MXC_F_TRNG_REVC_ST_HTS_POS)) /**< ST_HTS Mask */

 #define MXC_F_TRNG_REVC_ST_SRCFAIL_POS                 3 /**< ST_SRCFAIL Position */
 #define MXC_F_TRNG_REVC_ST_SRCFAIL                     ((uint32_t)(0x1UL << MXC_F_TRNG_REVC_ST_SRCFAIL_POS)) /**< ST_SRCFAIL Mask */

 #define MXC_F_TRNG_REVC_ST_AESKGD_MEU_S_POS            4 /**< ST_AESKGD_MEU_S Position */
 #define MXC_F_TRNG_REVC_ST_AESKGD_MEU_S                ((uint32_t)(0x1UL << MXC_F_TRNG_REVC_ST_AESKGD_MEU_S_POS)) /**< ST_AESKGD_MEU_S Mask */

 #define MXC_F_TRNG_REVC_ST_RSV16_POS                   16 /**< ST_RSV16 Position */
 #define MXC_F_TRNG_REVC_ST_RSV16                       ((uint32_t)(0x1UL << MXC_F_TRNG_REVC_ST_RSV16_POS)) /**< ST_RSV16 Mask */

/**@} end of group TRNG_REVC_ST_Register */

/**
 * @ingroup  trng_revc_registers
 * @defgroup TRNG_REVC_DATA TRNG_REVC_DATA
 * @brief    Data. The content of this register is valid only when RNG_IS = 1. When TRNG is
 *           disabled, read returns 0x0000 0000.
 * @{
 */
 #define MXC_F_TRNG_REVC_DATA_DATA_POS                  0 /**< DATA_DATA Position */
 #define MXC_F_TRNG_REVC_DATA_DATA                      ((uint32_t)(0xFFFFFFFFUL << MXC_F_TRNG_REVC_DATA_DATA_POS)) /**< DATA_DATA Mask */

/**@} end of group TRNG_REVC_DATA_Register */

#ifdef __cplusplus
}
#endif

#endif /* _TRNG_REVC_REGS_H_ */
